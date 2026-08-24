`timescale 1ns/1ps

// =============================================================================
// 模块名称：button_debounce
// 功能说明：对低有效机械按键进行双触发器同步、稳定时间判定，并生成单周期按下脉冲。
// 算法说明：输入连续保持 DEBOUNCE_CYCLES 个时钟后才更新稳定状态，可抑制触点抖动；
//           仅在稳定状态由高变低时输出 press_pulse，长按期间不会重复触发。
// 时钟/复位：clk 时钟域；rst_n 为异步低有效复位。
// 作者：Ethereal
// =============================================================================
module button_debounce #(
    parameter integer DEBOUNCE_CYCLES = 1_000_000
) (
    input  logic clk,
    input  logic rst_n,
    input  logic button_n,
    output logic press_pulse,
    output logic pressed
);

    localparam integer COUNTER_WIDTH =
        (DEBOUNCE_CYCLES <= 1) ? 1 : $clog2(DEBOUNCE_CYCLES);

    // button_n 来自异步引脚，先经两级寄存器降低亚稳态传播概率。
    logic [1:0] button_sync;
    // stable_n 保存已经通过消抖确认的低有效按键电平。
    logic stable_n;
    // debounce_count 记录“同步值与稳定值不一致”持续了多少个周期。
    logic [COUNTER_WIDTH-1:0] debounce_count;

    // 对外 pressed 使用高有效语义，便于上层状态机直接判断按下状态。
    always @* begin
        pressed = !stable_n;
    end

    // 消抖时序：同步输入、累计候选电平的稳定时间，并在确认按下时产生脉冲。
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            button_sync <= 2'b11;
            stable_n <= 1'b1;
            debounce_count <= '0;
            press_pulse <= 1'b0;
        end else begin
            button_sync <= {button_sync[0], button_n};
            press_pulse <= 1'b0;

            // 输入回到当前稳定状态时，候选变化无效，立即清零计数器。
            if (button_sync[1] == stable_n) begin
                debounce_count <= '0;
            // 参数为 1 时不等待；该分支也避免零位宽或无意义比较。
            end else if (DEBOUNCE_CYCLES <= 1) begin
                stable_n <= button_sync[1];
                press_pulse <= stable_n && !button_sync[1];
                debounce_count <= '0;
            // 候选电平保持足够时间后，提交新状态；只对下降沿报告按下事件。
            end else if (debounce_count == DEBOUNCE_CYCLES - 1) begin
                stable_n <= button_sync[1];
                press_pulse <= stable_n && !button_sync[1];
                debounce_count <= '0;
            end else begin
                debounce_count <= debounce_count + 1'b1;
            end
        end
    end

endmodule
