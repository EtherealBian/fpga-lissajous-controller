`timescale 1ns/1ps

// =============================================================================
// 模块名称：uart_byte_rx
// 功能说明：接收 8-N-1 UART 字节，支持 100 MHz/921600 等非整数分频波特率。
// 算法说明：使用相位累加器产生分数波特节拍；检测起始位下降沿后等待半位，
//           再在每一位中心采样 8 个数据位和 1 个停止位。
// 可靠性：异步 rx 经两级寄存器同步；停止位为低时输出 framing_error。
// 时钟/复位：clk 时钟域；rst_n 为异步低有效复位。
// 作者：Ethereal
// =============================================================================
module uart_byte_rx #(
    parameter integer CLOCK_HZ = 100_000_000,
    parameter integer BAUD_RATE = 921_600
) (
    input  logic       clk,
    input  logic       rst_n,
    input  logic       rx,
    output logic [7:0] data,
    output logic       valid,
    output logic       framing_error
);

    localparam integer ACCUMULATOR_WIDTH =
        (CLOCK_HZ <= 2) ? 1 : $clog2(CLOCK_HZ);

    localparam logic [1:0] RX_IDLE  = 2'd0;
    localparam logic [1:0] RX_START = 2'd1;
    localparam logic [1:0] RX_DATA  = 2'd2;
    localparam logic [1:0] RX_STOP  = 2'd3;

    logic rx_meta;
    logic rx_sync;
    logic rx_sync_d;
    logic [1:0] state;
    logic [2:0] bit_index;
    logic [7:0] data_work;
    logic [ACCUMULATOR_WIDTH-1:0] baud_accumulator;
    logic [ACCUMULATOR_WIDTH:0] baud_sum;

    // 分数-N 节拍：累加 BAUD_RATE，溢出 CLOCK_HZ 时产生下一采样时刻。
    always @* begin
        baud_sum =
            {1'b0, baud_accumulator} + BAUD_RATE;
    end

    // 至少保留 4 倍过采样余量，参数非法时在仿真开始即终止。
    initial begin
        if ((BAUD_RATE <= 0) ||
            (CLOCK_HZ < (BAUD_RATE * 4))) begin
            $error("UART RX clock/baud parameters are invalid");
        end
    end

    // 输入同步器单独成块，明确异步串口输入的时钟域边界。
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rx_meta <= 1'b1;
            rx_sync <= 1'b1;
            rx_sync_d <= 1'b1;
        end else begin
            rx_meta <= rx;
            rx_sync <= rx_meta;
            rx_sync_d <= rx_sync;
        end
    end

    // 接收状态机：下降沿确认、半位对齐、逐位采样、停止位校验。
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            data <= 8'd0;
            valid <= 1'b0;
            framing_error <= 1'b0;
            state <= RX_IDLE;
            bit_index <= 3'd0;
            data_work <= 8'd0;
            baud_accumulator <= '0;
        end else begin
            valid <= 1'b0;
            framing_error <= 1'b0;

            if (state == RX_IDLE) begin
                // 空闲时清零相位累加器，下降沿到来后从半位位置开始计时。
                baud_accumulator <= '0;
                if (rx_sync_d && !rx_sync) begin
                    // Begin with half an accumulator period so the first
                    // decision is made near the center of the start bit.
                    baud_accumulator <= CLOCK_HZ / 2;
                    state <= RX_START;
                end
            end else if (baud_sum >= CLOCK_HZ) begin
                baud_accumulator <= baud_sum - CLOCK_HZ;
                case (state)
                    RX_START: begin
                        if (!rx_sync) begin
                            bit_index <= 3'd0;
                            state <= RX_DATA;
                        end else begin
                            state <= RX_IDLE;
                        end
                    end

                    RX_DATA: begin
                        // UART 规定低位先发，按 bit_index 写入对应数据位。
                        data_work[bit_index] <= rx_sync;
                        if (bit_index == 3'd7) begin
                            state <= RX_STOP;
                        end else begin
                            bit_index <= bit_index + 1'b1;
                        end
                    end

                    default: begin
                        if (rx_sync) begin
                            data <= data_work;
                            valid <= 1'b1;
                        end else begin
                            framing_error <= 1'b1;
                        end
                        state <= RX_IDLE;
                    end
                endcase
            end else begin
                baud_accumulator <= baud_sum[
                    ACCUMULATOR_WIDTH-1:0];
            end
        end
    end

endmodule
