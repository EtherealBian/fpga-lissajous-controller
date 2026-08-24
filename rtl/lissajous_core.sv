`timescale 1ns/1ps

// =============================================================================
// 模块名称：lissajous_core
// 功能说明：完成有线模式 ADC 预处理、连续 I/Q DPLL、100 kHz 独立标定以及三种李萨如波形生成。
// 数据通路：10 位偏移二进制 ADC→去中值/限幅→DPLL 相位与频率估计→相位模式选择
//           →正弦 LUT→2/4/6/8 div 幅值缩放→10 位偏移二进制 DAC。
// 图形映射：同频同相得到对角线；同频 +90° 得到圆/椭圆；二倍锁定相位得到水平“∞”。
// 时延补偿：PHASE_PIPELINE_COMP_Q8 把数字流水线与板级延迟换算为相位并叠加到输出。
// 独立性：KEY4 标定通路只生成冻结参考字，不参与有线 DAC1 的连续 DPLL 控制。
// 时钟/复位：全部逻辑位于转换器 clk 域，sample_ce 控制有效采样；rst_n 异步低有效。
// 作者：Ethereal
// =============================================================================
module lissajous_core #(
    parameter integer SAMPLE_RATE_HZ = 30_000_000,
    parameter integer ADC_MID_CODE = 512,
    parameter integer ADC_CAL_PEAK_CODE = 205,
    parameter integer DAC_MID_CODE = 512,
    parameter integer PHASE_PIPELINE_COMP_Q8 = 16'd1_638,
    parameter integer FREQUENCY_CAL_TARGET_HZ = 100_000,
    parameter integer FREQUENCY_CAL_BLOCK_SAMPLES =
        SAMPLE_RATE_HZ / 1_000,
    parameter integer FREQUENCY_CAL_AVERAGING_BLOCKS = 256,
    parameter integer DPLL_MIN_FREQUENCY_HZ = 1_000,
    parameter integer DPLL_MAX_FREQUENCY_HZ = 110_000,
    parameter integer DPLL_COARSE_STEP_HZ = 1_000,
    parameter integer DPLL_COARSE_WINDOW_SAMPLES =
        SAMPLE_RATE_HZ / 1_000,
    parameter integer DPLL_FINE_RADIUS_STEPS = 10,
    parameter integer DPLL_FINE_WINDOW_SAMPLES = 262_144,
    parameter integer DPLL_TRACK_WINDOW_SAMPLES = 65_536,
    parameter integer DPLL_LOW_TRACK_WINDOW_SAMPLES = 262_144,
    parameter integer DPLL_LOW_TRACK_THRESHOLD_HZ = 20_000
) (
    input  logic clk,
    input  logic rst_n,
    input  logic sample_ce,
    input  logic [9:0] ad_data,
    input  logic wired_dpll_enable,
    input  logic frequency_cal_start_pulse,
    input  logic [1:0] mode_sel,
    input  logic [1:0] amplitude_sel,
    output logic [9:0] da_data,
    output logic period_locked,
    output logic [16:0] measured_period,
    output logic phase_cal_locked,
    output logic frequency_cal_active,
    output logic frequency_cal_locked,
    output logic [47:0] frequency_cal_phase_step,
    output logic [47:0] wired_dpll_phase_step,
    output logic signed [15:0] phase_error_q8
);

    localparam logic [1:0] MODE_DIRECT = 2'd0;
    localparam logic [1:0] MODE_QUADRATURE = 2'd1;
    localparam logic [1:0] MODE_DOUBLE = 2'd2;
    localparam logic [31:0] PHASE_QUARTER = 32'h4000_0000;

    logic signed [12:0] centered_wide;
    logic signed [10:0] current_sample;
    logic dpll_tracking_active;
    logic dpll_locked;
    logic [31:0] dpll_tracked_phase;
    logic signed [31:0] dpll_phase_error_word;
    logic [63:0] pipeline_phase_product;
    logic [31:0] pipeline_phase_compensation_reg;
    logic [31:0] pipeline_phase_compensation;
    logic [31:0] compensated_phase;
    logic [31:0] selected_phase_next;
    logic [31:0] selected_phase_reg;
    logic signed [10:0] dds_sample;
    logic signed [10:0] dds_sample_reg;
    logic [1:0] amplitude_sel_phase_reg;
    logic [1:0] amplitude_sel_dds_reg;
    logic phase_valid_reg;
    logic dds_valid_reg;
    logic signed [10:0] amplitude_scaled_sample;
    logic [31:0] frequency_cal_measured_samples;

    // 把 ADC 去中值后的样本裁剪到 ±ADC_CAL_PEAK_CODE，限制异常输入对相关器的影响。
    function automatic logic signed [10:0] clamp_to_cal(
        input logic signed [12:0] value
    );
        begin
            if (value > ADC_CAL_PEAK_CODE) begin
                clamp_to_cal = ADC_CAL_PEAK_CODE;
            end else if (value < -ADC_CAL_PEAK_CODE) begin
                clamp_to_cal = -ADC_CAL_PEAK_CODE;
            end else begin
                clamp_to_cal = value[10:0];
            end
        end
    endfunction

    // 按题目 2/4/6/8 div 四档缩放正弦幅值，并对正负数做对称舍入。
    function automatic logic signed [10:0] scale_for_amplitude(
        input logic signed [10:0] value,
        input logic [1:0] selection
    );
        logic [9:0] selected_peak;
        logic signed [21:0] peak_product;
        logic signed [21:0] rounded_magnitude;
        begin
            case (selection)
                2'd0:
                    selected_peak = (ADC_CAL_PEAK_CODE + 2) / 4;
                2'd1:
                    selected_peak = (ADC_CAL_PEAK_CODE + 1) / 2;
                2'd2:
                    selected_peak =
                        ((ADC_CAL_PEAK_CODE * 3) + 2) / 4;
                default:
                    selected_peak = ADC_CAL_PEAK_CODE;
            endcase

            // One coefficient-selected multiply replaces the former
            // full-scale multiply followed by a second amplitude multiply.
            peak_product = value * $signed({1'b0, selected_peak});
            if (peak_product < 0) begin
                rounded_magnitude =
                    ((-peak_product) + 22'sd128) >>> 8;
                scale_for_amplitude =
                    -rounded_magnitude[10:0];
            end else begin
                rounded_magnitude =
                    (peak_product + 22'sd128) >>> 8;
                scale_for_amplitude =
                    rounded_magnitude[10:0];
            end
        end
    endfunction

    // 将中心化样本加上 DAC 中值，并饱和为合法 10 位偏移二进制编码。
    function automatic logic [9:0] signed_to_dac(
        input logic signed [10:0] value
    );
        logic signed [12:0] biased;
        begin
            biased = value + DAC_MID_CODE;
            if (biased < 0) begin
                signed_to_dac = 10'd0;
            end else if (biased > 1023) begin
                signed_to_dac = 10'd1023;
            end else begin
                signed_to_dac = biased[9:0];
            end
        end
    endfunction

    // 有线主环：从 AD1 全采样数据持续估计输入相位和频率。
    continuous_iq_dpll #(
        .SAMPLE_RATE_HZ(SAMPLE_RATE_HZ),
        .MIN_FREQUENCY_HZ(DPLL_MIN_FREQUENCY_HZ),
        .MAX_FREQUENCY_HZ(DPLL_MAX_FREQUENCY_HZ),
        .COARSE_STEP_HZ(DPLL_COARSE_STEP_HZ),
        .COARSE_WINDOW_SAMPLES(
            DPLL_COARSE_WINDOW_SAMPLES),
        .FINE_RADIUS_STEPS(DPLL_FINE_RADIUS_STEPS),
        .FINE_WINDOW_SAMPLES(
            DPLL_FINE_WINDOW_SAMPLES),
        .TRACK_WINDOW_SAMPLES(
            DPLL_TRACK_WINDOW_SAMPLES),
        .LOW_TRACK_WINDOW_SAMPLES(
            DPLL_LOW_TRACK_WINDOW_SAMPLES),
        .LOW_TRACK_THRESHOLD_HZ(
            DPLL_LOW_TRACK_THRESHOLD_HZ)
    ) u_continuous_iq_dpll (
        .clk(clk),
        .rst_n(rst_n),
        .sample_ce(sample_ce),
        .enable(wired_dpll_enable),
        .input_sample(current_sample),
        .tracking_active(dpll_tracking_active),
        .locked(dpll_locked),
        .tracked_phase(dpll_tracked_phase),
        .tracked_phase_step(wired_dpll_phase_step),
        .phase_error_word(dpll_phase_error_word)
    );

    // This is the independent frequency-only calibration/holdover path used
    // by DAC2 and wireless mode. It never controls the wired DAC1 DPLL.
    // 独立校准支路：KEY4 请求时估计 100 kHz 参考字，供 DAC2/无线模式使用。
    reference_frequency_calibrator #(
        .SAMPLE_RATE_HZ(SAMPLE_RATE_HZ),
        .TARGET_FREQUENCY_HZ(FREQUENCY_CAL_TARGET_HZ),
        .BLOCK_SAMPLES(FREQUENCY_CAL_BLOCK_SAMPLES),
        .AVERAGING_BLOCKS(FREQUENCY_CAL_AVERAGING_BLOCKS)
    ) u_reference_frequency_calibrator (
        .clk(clk),
        .rst_n(rst_n),
        .sample_ce(sample_ce),
        .start(frequency_cal_start_pulse),
        .input_sample(current_sample),
        .active(frequency_cal_active),
        .locked(frequency_cal_locked),
        .phase_step(frequency_cal_phase_step),
        .measured_samples(frequency_cal_measured_samples)
    );

    dds_sine_lut u_dds_sine_lut (
        .phase(selected_phase_reg),
        .sine_sample(dds_sample)
    );

    // 组合数据通路完成 ADC 中心化、流水线相位补偿及三种图形的相位选择。
    always @* begin
        centered_wide =
            $signed({1'b0, ad_data}) - ADC_MID_CODE;
        current_sample = clamp_to_cal(centered_wide);

        pipeline_phase_product =
            wired_dpll_phase_step *
            PHASE_PIPELINE_COMP_Q8;
        pipeline_phase_compensation =
            pipeline_phase_compensation_reg;
        compensated_phase =
            dpll_tracked_phase +
            pipeline_phase_compensation;

        case (mode_sel)
            MODE_DIRECT:
                selected_phase_next = compensated_phase;
            MODE_QUADRATURE:
                // +1/4 周相位产生正交输出，与 X 轴同频同幅时形成圆或椭圆。
                selected_phase_next =
                    compensated_phase + PHASE_QUARTER;
            MODE_DOUBLE:
                // 相位左移一位等价于频率加倍，形成左右对称的水平“∞”图形。
                selected_phase_next = compensated_phase << 1;
            default:
                selected_phase_next = compensated_phase;
        endcase

        amplitude_scaled_sample =
            scale_for_amplitude(
                dds_sample_reg,
                amplitude_sel_dds_reg);

        period_locked = dpll_locked;
        phase_cal_locked = dpll_locked;
        measured_period = 17'd0;
        phase_error_q8 =
            dpll_phase_error_word[31:16];
    end

    // 两级有效流水线对齐相位选择、LUT 输出和幅值控制，最终在样本节拍更新 DAC。
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pipeline_phase_compensation_reg <= 32'd0;
            selected_phase_reg <= 32'd0;
            dds_sample_reg <= 11'sd0;
            amplitude_sel_phase_reg <= 2'd3;
            amplitude_sel_dds_reg <= 2'd3;
            phase_valid_reg <= 1'b0;
            dds_valid_reg <= 1'b0;
            da_data <= DAC_MID_CODE[9:0];
        end else if (sample_ce) begin
            // 将相位选择、插值 LUT、幅值/DAC 编码分别寄存，缩短转换器时钟域关键路径。
            pipeline_phase_compensation_reg <=
                pipeline_phase_product[55:24];
            selected_phase_reg <= selected_phase_next;
            amplitude_sel_phase_reg <= amplitude_sel;
            phase_valid_reg <= dpll_tracking_active;
            dds_sample_reg <= dds_sample;
            amplitude_sel_dds_reg <= amplitude_sel_phase_reg;
            dds_valid_reg <= phase_valid_reg;

            // 仅在查表样本有效时提交新 DAC 码，其余周期输出安全中值。
            if (dds_valid_reg) begin
                da_data <=
                    signed_to_dac(amplitude_scaled_sample);
            end else begin
                da_data <= DAC_MID_CODE[9:0];
            end
        end
    end

endmodule
