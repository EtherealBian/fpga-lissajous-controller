# ==========================================================================
#  Vivado 2025.2 全流程自动化脚本 — 李萨如图形显示控制装置
#
#  用法: vivado -mode batch -source lissajous_full_flow.tcl
#
#  流程: synth_design → opt_design → place_design → phys_opt_design
#        → route_design → report_timing → write_bitstream
#
#  目标器件: xc7z020clg400-2 (Mizar Z7)
#  顶层模块: lissajous_top
# ==========================================================================

# ---- 工程配置 ----
set projDir   [file normalize "[file dirname [info script]]/.."]
set rtlDir    [file join $projDir "rtl"]
set simDir    [file join $projDir "sim"]
set scriptDir [file join $projDir "scripts"]
set outDir    [file join $projDir "vivado_output"]

file mkdir $outDir

# ---- 创建内存工程 (不产生磁盘文件, 纯 Tcl 内存流) ----
create_project -in_memory -part xc7z020clg400-2

# ---- 添加 RTL 源文件 ----
set rtlFiles [glob -nocomplain [file join $rtlDir "*.sv"]]
if {[llength $rtlFiles] == 0} {
    puts "ERROR: No .sv files found in $rtlDir"
    exit 1
}
foreach f [lsort $rtlFiles] {
    read_verilog -sv $f
    puts "  \[RTL\] [file tail $f]"
}

# ---- 添加 IP 黑盒声明（ODDR 使用 Vivado 器件原语，不读取 sim/ 行为模型） ----
set ipStubPath [file join $scriptDir "vivado_ip_stubs.sv"]
if {[file exists $ipStubPath]} {
    read_verilog -sv $ipStubPath
    puts "  \[IP-STUB\] vivado_ip_stubs.sv"
}

# ---- 设置顶层 ----
set_property top lissajous_top [current_fileset]
puts ""
puts "========================================"
puts "  Vivado 2025.2 Full Flow"
puts "  Top:    lissajous_top"
puts "  Device: xc7z020clg400-2"
puts "  Files:  [llength $rtlFiles] RTL + 1 IP stub"
puts "========================================"
puts ""

# ============================================================================
#  Step 1: Synthesis
# ============================================================================
puts ">>> \[1/6\] synth_design"
synth_design -top lissajous_top -part xc7z020clg400-2 -flatten_hierarchy rebuilt
write_checkpoint -force [file join $outDir "post_synth.dcp"]
report_utilization  -file [file join $outDir "synth_utilization.rpt"]
puts "<<< synth_design OK"

# ============================================================================
#  Step 2: Opt Design
# ============================================================================
puts ">>> \[2/6\] opt_design"
opt_design
write_checkpoint -force [file join $outDir "post_opt.dcp"]
puts "<<< opt_design OK"

# ============================================================================
#  Step 3: Place Design
# ============================================================================
puts ">>> \[3/6\] place_design"
place_design
write_checkpoint -force [file join $outDir "post_place.dcp"]
puts "<<< place_design OK"

# ============================================================================
#  Step 4: Physical Optimization
# ============================================================================
puts ">>> \[4/6\] phys_opt_design"
phys_opt_design
write_checkpoint -force [file join $outDir "post_phys_opt.dcp"]
puts "<<< phys_opt_design OK"

# ============================================================================
#  Step 5: Route Design
# ============================================================================
puts ">>> \[5/6\] route_design"
route_design
write_checkpoint -force [file join $outDir "post_route.dcp"]
puts "<<< route_design OK"

# ============================================================================
#  Step 6: Reports & Bitstream
# ============================================================================
puts ">>> \[6/6\] reports + bitstream"

# 时序报告
report_timing_summary -file [file join $outDir "timing_summary.rpt"]
report_timing -delay_type min_max -file [file join $outDir "timing_detail.rpt"]
report_utilization -file [file join $outDir "final_utilization.rpt"]
report_power   -file [file join $outDir "power.rpt"]
report_drc     -file [file join $outDir "drc.rpt"]

# 比特流 (需要完整的引脚约束 .xdc, 当前为综合检查模式, 跳过)
# write_bitstream -force [file join $outDir "lissajous_top.bit"]
# write_debug_probes -force [file join $outDir "lissajous_top.ltx"]

puts "<<< reports OK"

# ============================================================================
#  结果汇总
# ============================================================================
puts ""
puts "========================================"
puts "  Vivado Full Flow Complete"
puts "  Output: $outDir"
puts "  Checkpoints:"
foreach dcp [glob -nocomplain [file join $outDir "*.dcp"]] {
    puts "    [file tail $dcp]"
}
puts "  Reports:"
foreach rpt [glob -nocomplain [file join $outDir "*.rpt"]] {
    puts "    [file tail $rpt]"
}
puts "========================================"
puts ""
puts "  下一步: 在 Vivado GUI 中打开 checkpoints 查看布局布线结果"
puts "         添加 .xdc 引脚约束后取消 write_bitstream 行注释即可生成比特流"
puts ""
