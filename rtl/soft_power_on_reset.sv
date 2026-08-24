`timescale 1ns/1ps

// =============================================================================
// 模块名称：soft_power_on_reset
// 功能说明：在外部时钟/PLL 可用后产生固定长度的上电软复位，工程无需额外复位引脚。
// 算法说明：enable 置高后累计 RESET_CYCLES 个时钟，计满时同步释放 rst_n；
//           enable 失效则立即重新拉低复位并清零计数器。
// 时钟/复位：寄存器依赖 FPGA 配置初值；rst_n 是本模块生成的输出而非输入。
// 作者：Ethereal
// =============================================================================
module soft_power_on_reset #(
    parameter integer RESET_CYCLES = 16
) (
    input  logic clk,
    input  logic enable,
    output logic rst_n = 1'b0
);

    localparam integer COUNTER_WIDTH =
        (RESET_CYCLES <= 1) ? 1 : $clog2(RESET_CYCLES);

    // Xilinx 7 系列配置完成后把该寄存器初始化为 0；复位保持时间由参数统一控制。
    logic [COUNTER_WIDTH-1:0] reset_counter = '0;

    // enable 常由 Clocking Wizard 的 locked 提供，防止时钟未稳定时释放逻辑。
    always_ff @(posedge clk) begin
        if (!enable) begin
            rst_n <= 1'b0;
            reset_counter <= '0;
        end else if (!rst_n) begin
            // 仅在复位保持阶段计数；rst_n 释放后寄存器保持不变。
            if (reset_counter >= RESET_CYCLES - 1) begin
                rst_n <= 1'b1;
            end else begin
                reset_counter <= reset_counter + 1'b1;
            end
        end
    end

endmodule
