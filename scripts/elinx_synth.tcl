# ==========================================================================
#  eLinx 3.0 综合脚本 —— Yosys Tcl 前端
#
#  用法: eLinx_synthesis.exe -c elinx_synth.tcl
#
#  将 SystemVerilog RTL 综合为 eHiChip6 系列的 VQM 网表。
#  输出: lissajous_top.vqm (在工程根目录)
# ==========================================================================

# ---- 读取所有 SystemVerilog 源文件 ----
# 支持 .sv 和 .v 扩展名；前端会自动检测
set rtl_dir [file normalize "../rtl"]
set sim_dir [file normalize "../sim"]

# RTL 设计文件 (不含 testbench)
read_verilog -sv \
    [file join $rtl_dir "button_debounce.sv"] \
    [file join $rtl_dir "calibrated_sine_test_tone.sv"] \
    [file join $rtl_dir "continuous_iq_dpll.sv"] \
    [file join $rtl_dir "cordic_atan2.sv"] \
    [file join $rtl_dir "dds_sine_lut.sv"] \
    [file join $rtl_dir "lissajous_core.sv"] \
    [file join $rtl_dir "lissajous_top.sv"] \
    [file join $rtl_dir "manual_control.sv"] \
    [file join $rtl_dir "reference_frequency_calibrator.sv"] \
    [file join $rtl_dir "soft_power_on_reset.sv"] \
    [file join $rtl_dir "status_leds.sv"] \
    [file join $rtl_dir "uart_byte_rx.sv"] \
    [file join $rtl_dir "uart_byte_tx.sv"] \
    [file join $rtl_dir "uart_packet_receiver.sv"] \
    [file join $rtl_dir "uart_packet_transmitter.sv"] \
    [file join $rtl_dir "unsigned_fraction_divider.sv"] \
    [file join $rtl_dir "wireless_commanded_dds.sv"] \
    [file join $rtl_dir "wireless_sawtooth_pulse.sv"] \
    [file join $rtl_dir "wireless_uart_controller.sv"]

# ---- 层次化检查 ----
hierarchy -top lissajous_top

# ---- 通用综合流程 ----
proc;          # 清理已有的 processes
opt;           # 初步优化
fsm;           # 有限状态机提取与优化
opt;           # 再次优化
wreduce;       # 缩减线宽
peepopt;       # 窥孔优化
opt_clean;     # 清理

# ---- 技术映射 (eHiChip6 4-LUT 架构) ----
# 注意: 如果 eLinx 提供了专用的技术库文件, 应替换为对应的 -lib 参数
# 当前使用通用 4-LUT 映射, eLinx P&R 工具链会做进一步转换
techmap;       # 技术映射
opt -fast;     # 快速优化

# ---- 输出 VQM 网表 ----
# 输出到项目根目录，文件名与工程名一致
write_verilog -noattr "../lissajous_top.vqm"

# ---- 统计报告 ----
stat -top lissajous_top
select -count t:* 2>/dev/null

puts ""
puts "========================================"
puts "  综合完成: lissajous_top.vqm"
puts "========================================"
