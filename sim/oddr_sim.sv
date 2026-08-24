`timescale 1ns/1ps

// =============================================================================
// 仿真模型：ODDR；功能：近似 Xilinx 7 系列 ODDR 的双边沿输出行为。
// 算法：时钟高电平输出 D1、低电平输出 D2，并实现使能、置位和复位优先级。
// 注意：仅供行为仿真，综合时使用器件库原语。
// 作者：Ethereal
// =============================================================================
module ODDR #(
    parameter DDR_CLK_EDGE = "OPPOSITE_EDGE",
    parameter INIT = 1'b0,
    parameter SRTYPE = "SYNC"
) (
    output logic Q,
    input  logic C,
    input  logic CE,
    input  logic D1,
    input  logic D2,
    input  logic R,
    input  logic S
);

    initial Q = INIT;

    // 用电平敏感模型表达双边沿选择，足以验证转发时钟的逻辑连接。
    always @(C or CE or D1 or D2 or R or S) begin
        if (R) begin
            Q = 1'b0;
        end else if (S) begin
            Q = 1'b1;
        end else if (CE) begin
            if (C) begin
                Q = D1;
            end else begin
                Q = D2;
            end
        end
    end

endmodule
