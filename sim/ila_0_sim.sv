`timescale 1ns/1ps

// =============================================================================
// 仿真模型：ila_0；功能：保留 ILA 探针端口但不记录数据，解除顶层仿真的 IP 依赖。
// 注意：真实工程由 Vivado 生成 ILA IP，本空模型仅供仿真连接检查。
// 作者：Ethereal
// =============================================================================
module ila_0 (
    input logic clk,
    input logic [9:0] probe0,
    input logic [9:0] probe1,
    input logic probe2,
    input logic signed [15:0] probe3,
    input logic signed [7:0] probe4
);
endmodule
