`timescale 1ns/1ps

// =============================================================================
// 模块名称：cordic_atan2
// 功能说明：使用迭代式向量模式 CORDIC 计算 atan2(y_in, x_in)。
// 算法说明：每周期执行一次移位加减旋转，共 ITERATIONS 轮；先根据 x 的符号
//           把向量预旋转到右半平面，再逐轮把 y 收敛到 0 并累积角度。
// 数值格式：输出 angle 为有符号 32 位“周”格式，2^30 表示 +90°，
//           32'h8000_0000 表示 ±180°。
// 握手规则：空闲时接受单周期 start；计算期间 busy=1；结果周期 valid=1。
// 作者：Ethereal
// =============================================================================
module cordic_atan2 #(
    parameter integer ITERATIONS = 24
) (
    input  logic clk,
    input  logic rst_n,
    input  logic start,
    input  logic signed [31:0] x_in,
    input  logic signed [31:0] y_in,
    output logic busy,
    output logic valid,
    output logic signed [31:0] angle
);

    localparam integer ITERATION_WIDTH =
        (ITERATIONS <= 2) ? 1 : $clog2(ITERATIONS);

    logic signed [33:0] x_value;
    logic signed [33:0] y_value;
    logic signed [32:0] z_value;
    logic signed [33:0] x_next;
    logic signed [33:0] y_next;
    logic signed [32:0] z_next;
    logic [ITERATION_WIDTH-1:0] iteration;
    logic signed [32:0] atan_step;

    // 每轮所需 atan(2^-i) 常量，采用与输出一致的“整周”定点格式。
    function automatic logic signed [32:0] atan_word(
        input integer index
    );
        begin
            case (index)
                0:  atan_word = 33'sh020000000;
                1:  atan_word = 33'sh012e4051e;
                2:  atan_word = 33'sh009fb385b;
                3:  atan_word = 33'sh0051111d4;
                4:  atan_word = 33'sh0028b0d43;
                5:  atan_word = 33'sh00145d7e1;
                6:  atan_word = 33'sh000a2f61e;
                7:  atan_word = 33'sh000517c55;
                8:  atan_word = 33'sh00028be53;
                9:  atan_word = 33'sh000145f2f;
                10: atan_word = 33'sh0000a2f98;
                11: atan_word = 33'sh0000517cc;
                12: atan_word = 33'sh000028be6;
                13: atan_word = 33'sh0000145f3;
                14: atan_word = 33'sh00000a2fa;
                15: atan_word = 33'sh00000517d;
                16: atan_word = 33'sh0000028be;
                17: atan_word = 33'sh00000145f;
                18: atan_word = 33'sh000000a30;
                19: atan_word = 33'sh000000518;
                20: atan_word = 33'sh00000028c;
                21: atan_word = 33'sh000000146;
                22: atan_word = 33'sh0000000a3;
                default: atan_word = 33'sh000000051;
            endcase
        end
    endfunction

    // 向量模式单步旋转：根据 y 符号选择顺/逆时针方向，使 |y| 逐步减小。
    always @* begin
        atan_step = atan_word(iteration);
        if (y_value >= 0) begin
            x_next = x_value + (y_value >>> iteration);
            y_next = y_value - (x_value >>> iteration);
            z_next = z_value + atan_step;
        end else begin
            x_next = x_value - (y_value >>> iteration);
            y_next = y_value + (x_value >>> iteration);
            z_next = z_value - atan_step;
        end
    end

    // 当前常量表覆盖 0～23 轮，参数越界时在仿真阶段立即报错。
    initial begin
        if ((ITERATIONS < 8) || (ITERATIONS > 24)) begin
            $error("CORDIC iteration count must be between 8 and 24");
        end
    end

    // 事务状态机：装载输入、象限预处理、逐轮迭代，最后给出角度有效脉冲。
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            x_value <= 34'sd0;
            y_value <= 34'sd0;
            z_value <= 33'sd0;
            iteration <= '0;
            busy <= 1'b0;
            valid <= 1'b0;
            angle <= 32'sd0;
        end else begin
            valid <= 1'b0;

            if (start && !busy) begin
                // x<0 时预旋转 ±180°，保证后续 CORDIC 在收敛区间内工作。
                iteration <= '0;
                busy <= 1'b1;
                if (x_in < 0) begin
                    x_value <=
                        -$signed({{2{x_in[31]}}, x_in});
                    y_value <=
                        -$signed({{2{y_in[31]}}, y_in});
                    if (y_in >= 0) begin
                        z_value <= 33'sd2_147_483_648;
                    end else begin
                        z_value <= -33'sd2_147_483_648;
                    end
                end else begin
                    x_value <=
                        $signed({{2{x_in[31]}}, x_in});
                    y_value <=
                        $signed({{2{y_in[31]}}, y_in});
                    z_value <= 33'sd0;
                end
            end else if (busy) begin
                // 每个时钟提交一次移位加减结果；最后一轮同时结束事务。
                x_value <= x_next;
                y_value <= y_next;
                z_value <= z_next;
                if (iteration == ITERATIONS - 1) begin
                    busy <= 1'b0;
                    valid <= 1'b1;
                    angle <= z_next[31:0];
                end else begin
                    iteration <= iteration + 1'b1;
                end
            end
        end
    end

endmodule
