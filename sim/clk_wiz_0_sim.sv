`timescale 1ns/1ps

// =============================================================================
// 仿真模型：clk_wiz_0；功能：替代 Vivado Clocking Wizard，产生 100 MHz/30 MHz 时钟。
// 注意：仅供行为仿真，不得加入综合源；锁定计数用于模拟 PLL 启动延迟。
// 作者：Ethereal
// =============================================================================
module clk_wiz_0 (
    input  logic clk_in1,
    output logic clk_out1,
    output logic clk_out2,
    output logic locked
);

    logic [3:0] lock_counter = 4'd0;

    // 两个独立永久进程按目标周期生成输出时钟。
    initial begin
        clk_out1 = 1'b0;
        clk_out2 = 1'b0;
        locked = 1'b0;
        fork
            forever #5 clk_out1 = ~clk_out1;
            forever #16.667 clk_out2 = ~clk_out2;
        join
    end

    // 输入参考时钟稳定 8 个周期后拉高 locked。
    always_ff @(posedge clk_in1) begin
        if (!locked) begin
            if (lock_counter == 4'd7) begin
                locked <= 1'b1;
            end else begin
                lock_counter <= lock_counter + 1'b1;
            end
        end
    end

endmodule
