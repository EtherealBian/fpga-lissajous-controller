`timescale 1ns/1ps

// =============================================================================
// 模块名称：manual_control
// 功能说明：汇总 6 路低有效按键，管理有线/无线模式、图形、幅值、校准和取消命令。
// 算法说明：每个按键先独立消抖；有线模式下 KEY2 循环三种图形、KEY3 循环四档幅值、
//           KEY4 启动标定、KEY5 切换 DAC2；无线模式下 KEY2～KEY4 发起三种自动流程，
//           KEY6 取消当前流程。所有事件输出均为单时钟脉冲。
// 时钟/复位：clk 时钟域；rst_n 为异步低有效复位。
// 作者：Ethereal
// =============================================================================
module manual_control #(
    parameter integer DEBOUNCE_CYCLES = 1_000_000,
    // Retained in the interface for project compatibility. Continuous DPLL
    // operation no longer uses manual phase-key auto-repeat.
    parameter integer PHASE_HOLD_DELAY_CYCLES = 50_000_000,
    parameter integer PHASE_REPEAT_CYCLES = 500_000
) (
    input  logic clk,
    input  logic rst_n,
    input  logic key1_n,
    input  logic key2_n,
    input  logic key3_n,
    input  logic key4_n,
    input  logic key5_n,
    input  logic key6_n,
    output logic wireless_mode,
    output logic [1:0] mode_sel,
    output logic [1:0] amplitude_sel,
    output logic dac2_frequency_advance_pulse,
    output logic frequency_cal_start_pulse,
    output logic wireless_start_pulse,
    output logic [1:0] wireless_start_pattern,
    output logic wireless_abort_pulse
);

    localparam logic [1:0] MODE_DIRECT = 2'd0;
    localparam logic [1:0] MODE_QUADRATURE = 2'd1;
    localparam logic [1:0] MODE_DOUBLE = 2'd2;
    localparam logic [1:0] AMP_8DIV = 2'd3;

    logic key1_press;
    logic key2_press;
    logic key3_press;
    logic key4_press;
    logic key5_press;
    logic key6_press;

    // 六路按键使用相同消抖参数，以下实例分别产生单周期 press 事件。
    button_debounce #(
        .DEBOUNCE_CYCLES(DEBOUNCE_CYCLES)
    ) u_key1 (
        .clk(clk),
        .rst_n(rst_n),
        .button_n(key1_n),
        .press_pulse(key1_press),
        .pressed()
    );

    button_debounce #(
        .DEBOUNCE_CYCLES(DEBOUNCE_CYCLES)
    ) u_key2 (
        .clk(clk),
        .rst_n(rst_n),
        .button_n(key2_n),
        .press_pulse(key2_press),
        .pressed()
    );

    button_debounce #(
        .DEBOUNCE_CYCLES(DEBOUNCE_CYCLES)
    ) u_key3 (
        .clk(clk),
        .rst_n(rst_n),
        .button_n(key3_n),
        .press_pulse(key3_press),
        .pressed()
    );

    button_debounce #(
        .DEBOUNCE_CYCLES(DEBOUNCE_CYCLES)
    ) u_key4 (
        .clk(clk),
        .rst_n(rst_n),
        .button_n(key4_n),
        .press_pulse(key4_press),
        .pressed()
    );

    button_debounce #(
        .DEBOUNCE_CYCLES(DEBOUNCE_CYCLES)
    ) u_key5 (
        .clk(clk),
        .rst_n(rst_n),
        .button_n(key5_n),
        .press_pulse(key5_press),
        .pressed()
    );

    button_debounce #(
        .DEBOUNCE_CYCLES(DEBOUNCE_CYCLES)
    ) u_key6 (
        .clk(clk),
        .rst_n(rst_n),
        .button_n(key6_n),
        .press_pulse(key6_press),
        .pressed()
    );

    // 无线事件译码保持组合化：按键号直接对应直线、圆和二倍频“∞”图形编号。
    always @* begin
        frequency_cal_start_pulse =
            !wireless_mode && key4_press;
        dac2_frequency_advance_pulse =
            key5_press;
        wireless_start_pulse =
            wireless_mode &&
            (key2_press || key3_press || key4_press);
        wireless_abort_pulse =
            wireless_mode && key6_press;
        if (key2_press) begin
            // 图形编号从 1 开始，与 UART 协议和上位机显示保持一致。
            wireless_start_pattern = 2'd1;
        end else if (key3_press) begin
            wireless_start_pattern = 2'd2;
        end else if (key4_press) begin
            wireless_start_pattern = 2'd3;
        end else begin
            wireless_start_pattern = 2'd0;
        end
    end

    // 模式与有线设置寄存器；切换模式时保留确定的默认幅值和图形行为。
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wireless_mode <= 1'b0;
            mode_sel <= MODE_DIRECT;
            amplitude_sel <= AMP_8DIV;
        end else begin
            if (key1_press) begin
                // KEY1 是唯一模式切换入口，避免无线流程中误触其他键改变连接关系。
                wireless_mode <= ~wireless_mode;
            end

            if (!wireless_mode) begin
                if (key2_press) begin
                    // 有线图形按直线→正交圆→二倍频“∞”循环。
                    case (mode_sel)
                        MODE_DIRECT:
                            mode_sel <= MODE_QUADRATURE;
                        MODE_QUADRATURE:
                            mode_sel <= MODE_DOUBLE;
                        default:
                            mode_sel <= MODE_DIRECT;
                    endcase
                end

                if (key3_press) begin
                    // 幅值选择为 2/4/6/8 div 对应的四档，寄存器自然回卷。
                    amplitude_sel <= amplitude_sel + 1'b1;
                end
            end
        end
    end

endmodule
