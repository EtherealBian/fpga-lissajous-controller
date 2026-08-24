`timescale 1ns/1ps

// =============================================================================
// 模块名称：status_leds
// 功能说明：把有线校准状态和无线流程状态映射到 4 个低有效板载 LED。
// 映射规则：有线模式仅 LED4 表示频率校准锁定；无线模式依次显示模式、识别、扫描和完成。
// 组合特性：本模块无时序状态，输入变化会直接反映到 led_n。
// 作者：Ethereal
// =============================================================================
module status_leds (
    input  logic wireless_mode,
    input  logic frequency_cal_locked,
    input  logic wireless_pulse_active,
    input  logic wireless_scan_active,
    input  logic wireless_done,
    output logic [3:0] led_n
);

    // 统一在此处完成有效电平转换，其他模块始终使用直观的高有效状态信号。
    always @* begin
        // 四个 PL LED 均为低电平点亮。
        led_n[0] = ~wireless_mode;
        if (wireless_mode) begin
            // 无线模式下 LED2～LED4 分别跟踪识别、扫描/调相、完成状态。
            led_n[1] = ~wireless_pulse_active;
            led_n[2] = ~wireless_scan_active;
            led_n[3] = ~wireless_done;
        end else begin
            // 有线模式关闭无线流程指示，仅保留 100 kHz 校准完成指示。
            led_n[2:1] = 2'b11;
            led_n[3] = ~frequency_cal_locked;
        end
    end

endmodule
