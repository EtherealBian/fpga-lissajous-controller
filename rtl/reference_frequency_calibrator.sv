`timescale 1ns/1ps

// =============================================================================
// 模块名称：reference_frequency_calibrator
// 功能说明：利用已知 100 kHz 输入对本地采样时钟进行全采样 I/Q 频率标定，输出冻结的 DDS 字。
// 算法流程：
//   1. ADC 样本与固定参考 NCO 的正弦/余弦逐点相关，每块形成复数 I/Q 向量；
//   2. 相邻块通过 cross/dot 构造相位增量，并由 CORDIC atan2 求精确角度；
//   3. 使用抛物线 Kay/最小二乘权重累加全部块的相位增量，抑制噪声；
//   4. 末端仅执行一次迭代除法，把加权斜率换算为 48 位频率修正并冻结。
// 设计特点：有效样本全部参与估计，不依赖过零点；标定完成前 locked 保持为 0。
// 时钟/复位：clk/ sample_ce 位于 ADC 时钟域；rst_n 异步低有效。
// 作者：Ethereal
// =============================================================================
module reference_frequency_calibrator #(
    parameter integer SAMPLE_RATE_HZ = 30_000_000,
    parameter integer TARGET_FREQUENCY_HZ = 100_000,
    parameter integer BLOCK_SAMPLES = SAMPLE_RATE_HZ / 1_000,
    parameter integer AVERAGING_BLOCKS = 256,
    parameter integer CORRELATION_SHIFT = 8,
    parameter integer CORDIC_PRODUCT_SHIFT = 16,
    parameter logic [63:0] MIN_VECTOR_ENERGY = 64'd100_000_000
) (
    input  logic clk,
    input  logic rst_n,
    input  logic sample_ce,
    input  logic start,
    input  logic signed [10:0] input_sample,
    output logic active,
    output logic locked,
    output logic [47:0] phase_step,
    output logic [31:0] measured_samples
);

    localparam integer BLOCK_COUNTER_WIDTH =
        (BLOCK_SAMPLES <= 2) ? 1 : $clog2(BLOCK_SAMPLES);
    localparam integer UPDATE_COUNTER_WIDTH =
        (AVERAGING_BLOCKS <= 2) ? 1 : $clog2(AVERAGING_BLOCKS);

    // 2^48 × 100 kHz 超过 64 位，先扩展为 80 位再除采样率，避免常量表达式溢出。
    localparam logic [79:0] NOMINAL_STEP_NUMERATOR =
        (80'd1 << 48) * TARGET_FREQUENCY_HZ;
    localparam logic [47:0] NOMINAL_PHASE_STEP =
        (NOMINAL_STEP_NUMERATOR + (SAMPLE_RATE_HZ / 2)) /
        SAMPLE_RATE_HZ;

    localparam logic [63:0] OBSERVATION_BLOCKS =
        AVERAGING_BLOCKS + 1;
    // Sum(k * (N-k)), k=1..N-1, where N is the number of phase
    // observations. These are the unnormalized least-squares slope weights.
    localparam logic [63:0] WEIGHT_SUM =
        (OBSERVATION_BLOCKS *
         ((OBSERVATION_BLOCKS * OBSERVATION_BLOCKS) - 1)) / 6;
    localparam logic [63:0] FINAL_DENOMINATOR =
        WEIGHT_SUM * BLOCK_SAMPLES;

    // The reference is expected to differ only by crystal ppm. Clamp the
    // final correction to +/-2000 ppm.
    localparam logic [63:0] MAX_CORRECTION_STEP =
        NOMINAL_PHASE_STEP / 500;
    localparam logic [63:0] TOTAL_MEASUREMENT_SAMPLES =
        (64'd1 * BLOCK_SAMPLES) * (AVERAGING_BLOCKS + 1);

    logic [47:0] reference_phase_accumulator;
    logic [31:0] reference_phase;
    logic [31:0] reference_quadrature_phase;
    logic signed [10:0] reference_sine;
    logic signed [10:0] reference_cosine;
    logic signed [21:0] in_phase_product;
    logic signed [21:0] quadrature_product;

    logic signed [39:0] in_phase_accumulator;
    logic signed [39:0] quadrature_accumulator;
    logic signed [39:0] in_phase_complete;
    logic signed [39:0] quadrature_complete;
    logic signed [31:0] current_i_vector;
    logic signed [31:0] current_q_vector;
    logic signed [31:0] previous_i_vector;
    logic signed [31:0] previous_q_vector;
    logic previous_vector_valid;

    logic signed [31:0] pair_current_i_reg;
    logic signed [31:0] pair_current_q_reg;
    logic signed [31:0] pair_previous_i_reg;
    logic signed [31:0] pair_previous_q_reg;
    logic signed [63:0] cross_product_a_reg;
    logic signed [63:0] cross_product_b_reg;
    logic signed [63:0] dot_product_a_reg;
    logic signed [63:0] dot_product_b_reg;
    logic signed [64:0] phase_cross_reg;
    logic signed [64:0] phase_dot_reg;
    logic pair_stage1;
    logic pair_stage2;
    logic pair_stage3;

    logic cordic_start;
    logic cordic_busy;
    logic cordic_valid;
    logic signed [31:0] cordic_x_input;
    logic signed [31:0] cordic_y_input;
    logic signed [31:0] cordic_angle;

    logic [BLOCK_COUNTER_WIDTH-1:0] block_sample_count;
    logic [UPDATE_COUNTER_WIDTH-1:0] update_count;
    logic [31:0] weight_index;
    logic [31:0] current_weight;
    logic signed [63:0] weighted_angle_product;
    logic signed [63:0] weighted_angle_sum;
    logic signed [63:0] weighted_angle_sum_candidate;
    logic [63:0] weighted_angle_sum_magnitude;

    logic final_divider_pending;
    logic final_divider_start;
    logic final_divider_busy;
    logic final_divider_valid;
    logic final_correction_negative;
    logic [79:0] final_divider_numerator;
    logic [79:0] final_divider_quotient;
    logic [63:0] correction_magnitude;
    logic signed [63:0] correction_value;
    logic signed [63:0] corrected_step_candidate;

    // 对块长、平均次数、目标频率和中间位宽做静态约束检查。
    initial begin
        if (BLOCK_SAMPLES < 64) begin
            $error("I/Q calibration block is too short");
        end
        if ((AVERAGING_BLOCKS < 4) ||
            ((AVERAGING_BLOCKS &
              (AVERAGING_BLOCKS - 1)) != 0)) begin
            $error("I/Q calibration block count must be a power of two");
        end
        if ((TARGET_FREQUENCY_HZ <= 0) ||
            (TARGET_FREQUENCY_HZ >= (SAMPLE_RATE_HZ / 2))) begin
            $error("I/Q calibration frequency is outside the sample band");
        end
        if ((CORDIC_PRODUCT_SHIFT < 0) ||
            (CORDIC_PRODUCT_SHIFT > 31)) begin
            $error("I/Q calibration CORDIC scaling is invalid");
        end
        if ((WEIGHT_SUM == 0) || (FINAL_DENOMINATOR == 0)) begin
            $error("I/Q calibration least-squares scale is invalid");
        end
        if (TOTAL_MEASUREMENT_SAMPLES > 64'hffff_ffff) begin
            $error("I/Q calibration sample count exceeds 32 bits");
        end
    end

    // 参考 NCO 使用 48 位相位累加，高 32 位驱动正交正弦查找表。
    assign reference_phase = reference_phase_accumulator[47:16];
    assign reference_quadrature_phase =
        reference_phase + 32'h4000_0000;

    dds_sine_lut u_reference_sine_lut (
        .phase(reference_phase),
        .sine_sample(reference_sine)
    );

    dds_sine_lut u_reference_cosine_lut (
        .phase(reference_quadrature_phase),
        .sine_sample(reference_cosine)
    );

    cordic_atan2 u_frequency_error_cordic (
        .clk(clk),
        .rst_n(rst_n),
        .start(cordic_start),
        .x_in(cordic_x_input),
        .y_in(cordic_y_input),
        .busy(cordic_busy),
        .valid(cordic_valid),
        .angle(cordic_angle)
    );

    // 最终一次 80 位除法把加权角度和转换为频率字修正量。
    unsigned_fraction_divider #(
        .NUMERATOR_WIDTH(80),
        .DENOMINATOR_WIDTH(64)
    ) u_final_frequency_error_divider (
        .clk(clk),
        .rst_n(rst_n),
        .start(final_divider_start),
        .numerator(final_divider_numerator),
        .denominator(FINAL_DENOMINATOR),
        .busy(final_divider_busy),
        .valid(final_divider_valid),
        .quotient(final_divider_quotient)
    );

    // 组合路径完成相关向量截位、cross/dot、Kay 权重、符号与限幅计算。
    always @* begin
        in_phase_product = input_sample * reference_sine;
        quadrature_product = input_sample * reference_cosine;

        in_phase_complete =
            in_phase_accumulator + in_phase_product;
        quadrature_complete =
            quadrature_accumulator + quadrature_product;
        current_i_vector = in_phase_complete >>> CORRELATION_SHIFT;
        current_q_vector =
            quadrature_complete >>> CORRELATION_SHIFT;

        weight_index = update_count + 1'b1;
        current_weight =
            weight_index *
            (OBSERVATION_BLOCKS[31:0] - weight_index);
        weighted_angle_product =
            $signed(cordic_angle) *
            $signed({1'b0, current_weight});
        weighted_angle_sum_candidate =
            weighted_angle_sum + weighted_angle_product;
        if (weighted_angle_sum_candidate < 0) begin
            weighted_angle_sum_magnitude =
                -weighted_angle_sum_candidate;
        end else begin
            weighted_angle_sum_magnitude =
                weighted_angle_sum_candidate;
        end

        if ((final_divider_quotient[79:64] != 0) ||
            (final_divider_quotient[63:0] >
             MAX_CORRECTION_STEP)) begin
            correction_magnitude = MAX_CORRECTION_STEP;
        end else begin
            correction_magnitude =
                final_divider_quotient[63:0];
        end
        if (final_correction_negative) begin
            correction_value = -$signed(correction_magnitude);
        end else begin
            correction_value = $signed(correction_magnitude);
        end
        corrected_step_candidate =
            $signed({16'd0, NOMINAL_PHASE_STEP}) +
            correction_value;
    end

    // 主时序流水线：采样相关→块向量→相邻向量乘法→CORDIC→加权累加→最终除法。
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            active <= 1'b0;
            locked <= 1'b0;
            phase_step <= 48'd0;
            measured_samples <= 32'd0;
            reference_phase_accumulator <= 48'd0;
            in_phase_accumulator <= 40'sd0;
            quadrature_accumulator <= 40'sd0;
            previous_i_vector <= 32'sd0;
            previous_q_vector <= 32'sd0;
            previous_vector_valid <= 1'b0;
            pair_current_i_reg <= 32'sd0;
            pair_current_q_reg <= 32'sd0;
            pair_previous_i_reg <= 32'sd0;
            pair_previous_q_reg <= 32'sd0;
            cross_product_a_reg <= 64'sd0;
            cross_product_b_reg <= 64'sd0;
            dot_product_a_reg <= 64'sd0;
            dot_product_b_reg <= 64'sd0;
            phase_cross_reg <= 65'sd0;
            phase_dot_reg <= 65'sd0;
            pair_stage1 <= 1'b0;
            pair_stage2 <= 1'b0;
            pair_stage3 <= 1'b0;
            cordic_start <= 1'b0;
            cordic_x_input <= 32'sd0;
            cordic_y_input <= 32'sd0;
            block_sample_count <= '0;
            update_count <= '0;
            weighted_angle_sum <= 64'sd0;
            final_divider_pending <= 1'b0;
            final_divider_start <= 1'b0;
            final_correction_negative <= 1'b0;
            final_divider_numerator <= 80'd0;
        end else begin
            pair_stage1 <= 1'b0;
            pair_stage2 <= 1'b0;
            pair_stage3 <= 1'b0;
            cordic_start <= 1'b0;
            final_divider_start <= 1'b0;

            if (start) begin
                // 新标定请求清空全部统计量，允许 KEY4 在运行后重新标定。
                active <= 1'b1;
                locked <= 1'b0;
                phase_step <= 48'd0;
                measured_samples <= 32'd0;
                reference_phase_accumulator <= 48'd0;
                in_phase_accumulator <= 40'sd0;
                quadrature_accumulator <= 40'sd0;
                previous_i_vector <= 32'sd0;
                previous_q_vector <= 32'sd0;
                previous_vector_valid <= 1'b0;
                pair_current_i_reg <= 32'sd0;
                pair_current_q_reg <= 32'sd0;
                pair_previous_i_reg <= 32'sd0;
                pair_previous_q_reg <= 32'sd0;
                cross_product_a_reg <= 64'sd0;
                cross_product_b_reg <= 64'sd0;
                dot_product_a_reg <= 64'sd0;
                dot_product_b_reg <= 64'sd0;
                phase_cross_reg <= 65'sd0;
                phase_dot_reg <= 65'sd0;
                cordic_x_input <= 32'sd0;
                cordic_y_input <= 32'sd0;
                block_sample_count <= '0;
                update_count <= '0;
                weighted_angle_sum <= 64'sd0;
                final_divider_pending <= 1'b0;
                final_correction_negative <= 1'b0;
                final_divider_numerator <= 80'd0;
            end else begin
                // 只在有效采样使能时推进 NCO 并累计 I/Q，保证频率单位与采样率一致。
                if (active && !locked && sample_ce) begin
                    reference_phase_accumulator <=
                        reference_phase_accumulator +
                        NOMINAL_PHASE_STEP;

                    if (block_sample_count ==
                        BLOCK_SAMPLES - 1) begin
                        block_sample_count <= '0;
                        in_phase_accumulator <= 40'sd0;
                        quadrature_accumulator <= 40'sd0;
                        previous_i_vector <= current_i_vector;
                        previous_q_vector <= current_q_vector;
                        previous_vector_valid <= 1'b1;

                        if (previous_vector_valid &&
                            !cordic_busy &&
                            !final_divider_busy &&
                            !final_divider_pending &&
                            !pair_stage1 &&
                            !pair_stage2 &&
                            !pair_stage3) begin
                            pair_current_i_reg <= current_i_vector;
                            pair_current_q_reg <= current_q_vector;
                            pair_previous_i_reg <= previous_i_vector;
                            pair_previous_q_reg <= previous_q_vector;
                            pair_stage1 <= 1'b1;
                        end
                    end else begin
                        block_sample_count <=
                            block_sample_count + 1'b1;
                        in_phase_accumulator <=
                            in_phase_complete;
                        quadrature_accumulator <=
                            quadrature_complete;
                    end
                end

                // 三级乘加流水线计算 cross=Iprev·Qcur-Qprev·Icur 和 dot。
                if (pair_stage1) begin
                    cross_product_a_reg <=
                        pair_previous_i_reg *
                        pair_current_q_reg;
                    cross_product_b_reg <=
                        pair_previous_q_reg *
                        pair_current_i_reg;
                    dot_product_a_reg <=
                        pair_previous_i_reg *
                        pair_current_i_reg;
                    dot_product_b_reg <=
                        pair_previous_q_reg *
                        pair_current_q_reg;
                    pair_stage2 <= 1'b1;
                end

                if (pair_stage2) begin
                    phase_cross_reg <=
                        $signed({cross_product_a_reg[63],
                                 cross_product_a_reg}) -
                        $signed({cross_product_b_reg[63],
                                 cross_product_b_reg});
                    phase_dot_reg <=
                        $signed({dot_product_a_reg[63],
                                 dot_product_a_reg}) +
                        $signed({dot_product_b_reg[63],
                                 dot_product_b_reg});
                    pair_stage3 <= 1'b1;
                end

                if (pair_stage3) begin
                    if (!phase_dot_reg[64] &&
                        (phase_dot_reg[64:0] >=
                         MIN_VECTOR_ENERGY) &&
                        !cordic_busy) begin
                        cordic_x_input <=
                            phase_dot_reg >> CORDIC_PRODUCT_SHIFT;
                        cordic_y_input <=
                            $signed(phase_cross_reg) >>>
                            CORDIC_PRODUCT_SHIFT;
                        cordic_start <= 1'b1;
                    end
                end

                // 每个相位增量乘以 Kay 权重后累加；最后一个增量触发唯一一次除法。
                if (cordic_valid) begin
                    weighted_angle_sum <=
                        weighted_angle_sum_candidate;
                    if (update_count ==
                        AVERAGING_BLOCKS - 1) begin
                        final_correction_negative <=
                            weighted_angle_sum_candidate < 0;
                        final_divider_numerator <=
                            ({weighted_angle_sum_magnitude,
                              16'd0}) +
                            (FINAL_DENOMINATOR / 2);
                        final_divider_pending <= 1'b1;
                    end else begin
                        update_count <= update_count + 1'b1;
                    end
                end

                if (final_divider_pending &&
                    !final_divider_busy) begin
                    final_divider_start <= 1'b1;
                    final_divider_pending <= 1'b0;
                end

                // 商有效后恢复修正符号、叠加名义频率字并原子发布锁定结果。
                if (final_divider_valid) begin
                    if (corrected_step_candidate > 0) begin
                        phase_step <=
                            corrected_step_candidate[47:0];
                        locked <= 1'b1;
                        measured_samples <=
                            TOTAL_MEASUREMENT_SAMPLES[31:0];
                    end
                end
            end
        end
    end

endmodule
