`timescale 1ns/1ps

// =============================================================================
// 模块名称：unsigned_fraction_divider
// 功能说明：实现可参数化无符号恢复余数除法，供频率字比例换算等低速控制路径使用。
// 算法说明：从被除数最高位开始，每周期左移余数并试减除数，同时生成 1 位商；
//           NUMERATOR_WIDTH 个周期后输出完整商，不在 DDS 逐采样关键路径中展开大除法器。
// 边界行为：除数为 0 时返回全 1 商；busy 期间忽略新的 start。
// 握手规则：空闲时接受 start；计算期间 busy=1；结束周期 valid=1。
// 作者：Ethereal
// =============================================================================
module unsigned_fraction_divider #(
    parameter integer NUMERATOR_WIDTH = 34,
    parameter integer DENOMINATOR_WIDTH = 12
) (
    input  logic clk,
    input  logic rst_n,
    input  logic start,
    input  logic [NUMERATOR_WIDTH-1:0] numerator,
    input  logic [DENOMINATOR_WIDTH-1:0] denominator,
    output logic busy,
    output logic valid,
    output logic [NUMERATOR_WIDTH-1:0] quotient
);

    localparam integer BIT_INDEX_WIDTH =
        (NUMERATOR_WIDTH <= 2) ? 1 : $clog2(NUMERATOR_WIDTH);

    logic [NUMERATOR_WIDTH-1:0] numerator_reg;
    logic [DENOMINATOR_WIDTH-1:0] denominator_reg;
    logic [DENOMINATOR_WIDTH:0] remainder_reg;
    logic [DENOMINATOR_WIDTH:0] remainder_shifted;
    logic [NUMERATOR_WIDTH-1:0] quotient_work;
    logic [NUMERATOR_WIDTH-1:0] quotient_next;
    logic [BIT_INDEX_WIDTH-1:0] bit_index;

    // 组合试商：拼入当前被除数位，并决定本轮商位是否置 1。
    always @* begin
        remainder_shifted = {
            remainder_reg[DENOMINATOR_WIDTH-1:0],
            numerator_reg[bit_index]
        };
        quotient_next = quotient_work;
        if (remainder_shifted >= {1'b0, denominator_reg}) begin
            quotient_next[bit_index] = 1'b1;
        end
    end

    // 迭代控制：装载操作数后从最高位向最低位逐周期完成长除法。
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            numerator_reg <= '0;
            denominator_reg <= {{(DENOMINATOR_WIDTH-1){1'b0}}, 1'b1};
            remainder_reg <= '0;
            quotient_work <= '0;
            quotient <= '0;
            bit_index <= '0;
            busy <= 1'b0;
            valid <= 1'b0;
        end else begin
            valid <= 1'b0;

            if (start && !busy) begin
                // 除零采用饱和值，避免状态机进入无意义的迭代过程。
                if (denominator == '0) begin
                    quotient <= '0;
                    valid <= 1'b1;
                end else begin
                    numerator_reg <= numerator;
                    denominator_reg <= denominator;
                    remainder_reg <= '0;
                    quotient_work <= '0;
                    bit_index <= NUMERATOR_WIDTH - 1;
                    busy <= 1'b1;
                end
            end else if (busy) begin
                // 提交本轮余数和商位；bit_index=0 时同时发布最终结果。
                if (remainder_shifted >= {1'b0, denominator_reg}) begin
                    remainder_reg <=
                        remainder_shifted - {1'b0, denominator_reg};
                end else begin
                    remainder_reg <= remainder_shifted;
                end
                quotient_work <= quotient_next;

                if (bit_index == '0) begin
                    quotient <= quotient_next;
                    busy <= 1'b0;
                    valid <= 1'b1;
                end else begin
                    bit_index <= bit_index - 1'b1;
                end
            end
        end
    end

endmodule
