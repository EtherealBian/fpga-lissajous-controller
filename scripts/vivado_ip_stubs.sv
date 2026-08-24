`timescale 1ns/1ps

// =============================================================================
// 综合检查桩：仅声明 clk_wiz_0 与 ila_0 的端口，使无工程模式综合能够解析顶层实例。
// 注意：这些模块是 black_box，不包含功能实现；正常 Vivado 工程必须使用实际生成的 IP，
//       本文件不得与真实 IP 同时加入 source set。
// 作者：Ethereal
// =============================================================================

// 双输出 Clocking Wizard 黑盒端口。
(* black_box = "true" *)
module clk_wiz_0 (
    input  wire clk_in1,
    output wire clk_out1,
    output wire clk_out2,
    output wire locked
);
endmodule

// ILA 调试核黑盒端口；探针位宽必须与顶层连接保持一致。
(* black_box = "true" *)
module ila_0 (
    input wire clk,
    input wire [9:0] probe0,
    input wire [9:0] probe1,
    input wire probe2,
    input wire signed [15:0] probe3,
    input wire signed [7:0] probe4
);
endmodule
