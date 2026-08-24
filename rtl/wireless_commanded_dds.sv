`timescale 1ns/1ps

// =============================================================================
// 模块名称：wireless_commanded_dds
// 功能说明：执行上位机下发的任意频率与相位命令，生成无线自动模式的 DAC1 正弦。
// 算法说明：以冻结的 100 kHz 标定字为唯一时基，计算
//           phase_step = calibrated_step × frequency_millihz / 100000000；
//           除法仅在频率或标定值变化时运行，逐采样路径不包含除法。
// 相位控制：phase_q16 的 65536 对应一整周，可与频率命令独立或原子更新。
// 输出格式：sine_data 为 10 位偏移二进制，enable 无效或标定未完成时输出中值码。
// 作者：Ethereal
// =============================================================================
module wireless_commanded_dds #(
    parameter integer CALIBRATION_FREQUENCY_MILLIHZ =
        100_000_000,
    parameter integer DAC_MID_CODE = 512,
    parameter integer DAC_PEAK_CODE = 205
) (
    input  logic        clk,
    input  logic        rst_n,
    input  logic        sample_ce,
    input  logic        enable,
    input  logic        calibration_valid,
    input  logic [47:0] calibrated_phase_step,
    input  logic [31:0] frequency_millihz,
    input  logic [15:0] phase_q16,
    input  logic        frequency_update_pulse,
    input  logic        phase_update_pulse,
    output logic [9:0]  sine_data,
    output logic [47:0] phase_step,
    output logic        phase_step_valid
);

    localparam logic [31:0] STEP_DENOMINATOR =
        CALIBRATION_FREQUENCY_MILLIHZ;

    logic [79:0] divider_numerator;
    logic divider_request;
    logic divider_start;
    logic divider_busy;
    logic divider_valid;
    logic [79:0] divider_quotient;
    logic calibration_valid_d;
    logic [47:0] calibrated_phase_step_d;

    logic [47:0] phase_accumulator;
    logic [15:0] phase_offset_q16;
    logic [31:0] lut_phase;
    logic signed [10:0] sine_sample;
    logic signed [21:0] scaled_product;
    logic signed [21:0] rounded_product;
    logic signed [11:0] centered_code;

    // 相位累加器高 32 位叠加 Q0.16 周相位偏置后送入正弦 LUT。
    assign lut_phase =
        phase_accumulator[47:16] +
        {phase_offset_q16, 16'd0};

    dds_sine_lut u_sine_lut (
        .phase(lut_phase),
        .sine_sample(sine_sample)
    );

    // 80 位迭代除法器位于设置路径，保留足够乘积精度后再截取 48 位频率字。
    unsigned_fraction_divider #(
        .NUMERATOR_WIDTH(80),
        .DENOMINATOR_WIDTH(32)
    ) u_frequency_step_divider (
        .clk(clk),
        .rst_n(rst_n),
        .start(divider_start),
        .numerator(divider_numerator),
        .denominator(STEP_DENOMINATOR),
        .busy(divider_busy),
        .valid(divider_valid),
        .quotient(divider_quotient)
    );

    // 正弦幅值缩放、四舍五入、偏置二进制编码和输出限幅均为组合计算。
    always @* begin
        scaled_product =
            $signed(sine_sample) * DAC_PEAK_CODE;
        if (scaled_product < 0) begin
            rounded_product =
                scaled_product + 22'sd127;
        end else begin
            rounded_product =
                scaled_product + 22'sd128;
        end
        centered_code =
            $signed(DAC_MID_CODE) +
            ($signed(rounded_product) >>> 8);

        if (!enable ||
            !calibration_valid ||
            !phase_step_valid) begin
            sine_data = DAC_MID_CODE[9:0];
        end else if (centered_code < 0) begin
            sine_data = 10'd0;
        end else if (centered_code > 12'sd1023) begin
            sine_data = 10'd1023;
        end else begin
            sine_data = centered_code[9:0];
        end
    end

    // 检查标定分母和 10 位 DAC 编码范围，阻止无效参数进入硬件。
    initial begin
        if (CALIBRATION_FREQUENCY_MILLIHZ <= 0) begin
            $error("Wireless DDS calibration frequency is invalid");
        end
        if ((DAC_MID_CODE < 0) ||
            (DAC_MID_CODE > 1023) ||
            (DAC_PEAK_CODE < 0) ||
            (DAC_PEAK_CODE > DAC_MID_CODE)) begin
            $error("Wireless DDS DAC range is invalid");
        end
    end

    // 控制时序处理重算请求、除法结果、相位更新和逐采样相位累加。
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            divider_numerator <= 80'd0;
            divider_request <= 1'b0;
            divider_start <= 1'b0;
            phase_step <= 48'd0;
            phase_step_valid <= 1'b0;
            calibration_valid_d <= 1'b0;
            calibrated_phase_step_d <= 48'd0;
            phase_accumulator <= 48'd0;
            phase_offset_q16 <= 16'd0;
        end else begin
            divider_start <= 1'b0;
            calibration_valid_d <= calibration_valid;
            calibrated_phase_step_d <= calibrated_phase_step;

            // 新频率、首次获得标定或标定字变化都会触发一次比例换算。
            if (calibration_valid &&
                (frequency_update_pulse ||
                 !calibration_valid_d ||
                 (calibrated_phase_step !=
                  calibrated_phase_step_d))) begin
                divider_numerator <=
                    ({32'd0, calibrated_phase_step} *
                     frequency_millihz) +
                    (STEP_DENOMINATOR / 2);
                if (frequency_millihz == 32'd0) begin
                    divider_request <= 1'b0;
                    phase_step <= 48'd0;
                    phase_step_valid <= 1'b0;
                end else begin
                    divider_request <= 1'b1;
                end
            end else if (divider_request &&
                         !divider_busy) begin
                divider_start <= 1'b1;
                divider_request <= 1'b0;
            end

            // 标定失效时撤销频率字有效标志，禁止输出未经校正的自动波形。
            if (!calibration_valid) begin
                divider_request <= 1'b0;
                phase_step <= 48'd0;
                phase_step_valid <= 1'b0;
            end else if (divider_valid) begin
                phase_step <= divider_quotient[47:0];
                phase_step_valid <= 1'b1;
            end

            // 相位更新不清空频率累加器，避免调整时产生波形频率间断。
            if (phase_update_pulse) begin
                phase_offset_q16 <= phase_q16;
            end

            if (!enable) begin
                phase_accumulator <= 48'd0;
            end else if (sample_ce && phase_step_valid) begin
                phase_accumulator <=
                    phase_accumulator + phase_step;
            end
        end
    end

endmodule
