`timescale 1ns/1ps

// =============================================================================
// 模块名称：uart_byte_tx
// 功能说明：发送 8-N-1 UART 字节，并用分数波特率累加器消除整数分频误差。
// 算法说明：每位累加 BAUD_RATE，跨越 CLOCK_HZ 时推进一位，使位宽在相邻整数
//           时钟数之间自动抖动，长期平均波特率精确匹配参数值。
// 帧格式：1 个低电平起始位、8 个低位优先数据位、1 个高电平停止位。
// 握手规则：ready=1 时接受 valid 和 data；发送期间忽略新的请求。
// 作者：Ethereal
// =============================================================================
module uart_byte_tx #(
    parameter integer CLOCK_HZ = 100_000_000,
    parameter integer BAUD_RATE = 921_600
) (
    input  logic       clk,
    input  logic       rst_n,
    input  logic [7:0] data,
    input  logic       valid,
    output logic       ready,
    output logic       tx
);

    localparam integer ACCUMULATOR_WIDTH =
        (CLOCK_HZ <= 2) ? 1 : $clog2(CLOCK_HZ);

    logic [ACCUMULATOR_WIDTH-1:0] baud_accumulator;
    logic [ACCUMULATOR_WIDTH:0] baud_sum;
    logic [7:0] data_shift;
    logic [3:0] bit_index;
    logic busy;

    // busy 为内部占用标志，对外取反形成标准 ready/valid 握手。
    assign ready = !busy;

    // 计算下一周期的分数波特累加值，最高位用于判断是否到达位边界。
    always @* begin
        baud_sum =
            {1'b0, baud_accumulator} + BAUD_RATE;
    end

    // 参数检查防止时钟过低导致串行链路无法可靠工作。
    initial begin
        if ((BAUD_RATE <= 0) ||
            (CLOCK_HZ < (BAUD_RATE * 4))) begin
            $error("UART TX clock/baud parameters are invalid");
        end
    end

    // 发送状态机在装载时立即拉低起始位，之后按节拍移出数据和停止位。
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            baud_accumulator <= '0;
            data_shift <= 8'd0;
            bit_index <= 4'd0;
            busy <= 1'b0;
            tx <= 1'b1;
        end else if (!busy) begin
            baud_accumulator <= '0;
            tx <= 1'b1;
            if (valid) begin
                // 仅在空闲分支进入此处；锁存数据可避免上层随后改变总线。
                data_shift <= data;
                bit_index <= 4'd0;
                busy <= 1'b1;
                tx <= 1'b0;
            end
        end else if (baud_sum >= CLOCK_HZ) begin
            baud_accumulator <= baud_sum - CLOCK_HZ;
            if (bit_index < 4'd8) begin
                // bit_index=0～7 发送数据位，之后发送一个高电平停止位。
                tx <= data_shift[bit_index];
                bit_index <= bit_index + 1'b1;
            end else if (bit_index == 4'd8) begin
                tx <= 1'b1;
                bit_index <= 4'd9;
            end else begin
                tx <= 1'b1;
                busy <= 1'b0;
            end
        end else begin
            baud_accumulator <= baud_sum[
                ACCUMULATOR_WIDTH-1:0];
        end
    end

endmodule
