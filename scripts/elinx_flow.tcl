# ==========================================================================
#  eLinx 3.0 布局布线 + 比特流生成脚本
#
#  用法: Infrastructure.exe -f elinx_flow.tcl
#
#  流程: UDM(解析VQM) → ARC(器件架构) → PACK(打包) → PLACE(布局)
#        → ROUTE(布线) → BITGEN(比特流)
#
#  前置条件:
#    1. 综合已完成, 生成了 lissajous_top.vqm
#    2. 若有时序约束, 需提供 .sdc 文件
#    3. 若有引脚约束, 需提供 lissajous_top.psf 文件
# ==========================================================================

# ==========================================================================
#  用户配置区 —— 根据实际硬件修改
# ==========================================================================

set projectName    "lissajous_top"       ;# 工程名 (与 VQM 文件名一致, 不含扩展名)
set projectDir     [pwd]                 ;# 工程目录 (VQM 文件所在位置)
set topEntity      "lissajous_top"       ;# 顶层实体名
set seriesName     "eHiChip6"            ;# 器件系列
set deviceName     "EQ6HL130"            ;# 器件型号 (EQ6HL130 / EQ6HL45 / EQ6HL9)

# ---- 如果本脚本不在工程根目录运行, 取消注释并修改以下路径 ----
# set projectDir     "D:/path/to/nuedc-2026-f-lissajous-fpga"
# cd $projectDir

# ==========================================================================
#  全局设置
# ==========================================================================

set_global_assignment -name TOP_LEVEL_ENTITY $topEntity
set_global_assignment -name DESIGN_PATH     $projectDir
set_global_assignment -name PROJECT_NAME    $projectName
set_global_assignment -name DEVICE_SERIES   $seriesName
set_global_assignment -name DEVICE_NAME     $deviceName

set vecname $projectName
set vecdir  $projectDir
regsub -all {\\\\} $vecdir {/} vecdir
regsub -all {\\\\} $vecdir {/} resultdir

# ---- 日志文件 ----
setlogfile $projectDir/log4cpp.property $projectDir/psk_fit.log

# ---- 列出所有 RTL 源文件 (用于网表追溯和调试) ----
# set_global_assignment -name VERILOG_FILE button_debounce.sv
# set_global_assignment -name VERILOG_FILE calibrated_sine_test_tone.sv
# ...
# 注意: 已综合为 VQM 后不再需要逐文件列出

puts ""
puts "========================================"
puts "  工程配置"
puts "========================================"
puts "  顶层实体:   $topEntity"
puts "  工程目录:   $projectDir"
puts "  器件系列:   $seriesName"
puts "  器件型号:   $deviceName"
puts "========================================"
puts ""

# ==========================================================================
#  步骤 1: UDM —— 解析综合后的 VQM 网表
# ==========================================================================
puts ">>> \[1/6\] UDM: 解析 VQM 网表..."

set vqmdir $vecdir/$vecname.vqm

if {[catch {udm_parse_vqm} err_msg]} {
    set udm_gene_line "$vecname  $vecdir  udm exec Error:$err_msg"
    set udm_gene 0
    puts "ERROR: $udm_gene_line"
    exit 1
} else {
    set udm_gene_line "$vecname  $vecdir  udm exec Success"
    set udm_gene 1
}
puts "  \[OK\] UDM 解析完成"
puts ""

# ==========================================================================
#  步骤 2: ARC —— 加载器件架构
# ==========================================================================
puts ">>> \[2/6\] ARC: 加载器件架构..."

# ---- 架构配置参数 (eHiChip6 系列固定值) ----
set_arc_ucm_parameter BLK           glj_blk_hc
set_arc_ucm_parameter IOB_T         wb_iob_top_hc
set_arc_ucm_parameter IOB_T_M4K     wb_m4klabiob_top_hc
set_arc_ucm_parameter IOB_B         wb_iob_bottom_hc
set_arc_ucm_parameter IOB_B_M4K     wb_m4klabiob_bottom_hc
set_arc_ucm_parameter IOB_L         glj_leftiob_hc
set_arc_ucm_parameter IOB_R         glj_rightiob_hc
set_arc_ucm_parameter IOB_R_T       glj_rightopiob_hc
set_arc_ucm_parameter IOB_R_B       glj_rightbottomiob_hc
set_arc_ucm_parameter CLK           czj_clk_center
set_arc_ucm_parameter CLK_CHIP      clk_top_8x8
set_arc_ucm_parameter VIR_BLK_GBUF  blk4x4_gbuf
set_arc_ucm_parameter VIR_M4K_GBUF  m4k4x4_gbuf
set_arc_ucm_parameter VIR_DSP_GBUF  dsp8x4_gbuf
set_arc_ucm_parameter VIR_DSP_CLUSTER virtual_clustring
set_arc_ucm_parameter SPECIO        special_io_hc
set_arc_ucm_parameter SPECIO_CHIP   special_io_hc_chip

# ---- 根据器件型号选择 M4K 架构配置 ----
set is_EQ6HL150_device [string compare $deviceName EQ6HL150]
if {$is_EQ6HL150_device == 0} {
    set_arc_ucm_parameter M4K    wb_m4kblk_hc
    set_arc_ucm_parameter M4KLAB wb_m4klab_hc
} else {
    set_arc_ucm_parameter M4K    hzh_m4kblk_hc
    set_arc_ucm_parameter M4KLAB hzh_m4klab_hc
}

set_arc_ucm_parameter DSP           lw_dspblk_8x1_hc
set_arc_ucm_parameter DSP1          lw_dspblk_hc
set_arc_ucm_parameter DSPLAB        ylq_dsplab_hc
set_arc_ucm_parameter IOB_T_DSP     wb_dspiob_top_hc
set_arc_ucm_parameter IOB_B_DSP     wb_dspiob_bottom_hc
set_arc_ucm_parameter SIGTAP        virtual_iob

set_arc_ucm_parameter qxg_06io_cfgcell_CELL_LIB           qxg_06io_cfgcell_CELL_LIB
set_arc_ucm_parameter qxg_06io_cfgcell__minout_CELL_LIB   qxg_06io_cfgcell__minout_CELL_LIB

# ---- 器件描述数据路径 ----
set_arc_ucm_parameter TIEL_DAT_PATH   ..\\Infrastructure\\psk_intern\\$seriesName\\series_library\\tile
set_arc_ucm_parameter STRUCTURE_PATH  ..\\Infrastructure\\psk_intern\\$seriesName\\$deviceName\\structure.dat

if {[catch {arc_deserialize_device_info} err_msg] ||
    [catch {arc_deserialize_tile_info}   err_msg]} {
    set arc_gene_line "$vecname  $vecdir  arc generate Error:$err_msg"
    set arc_gene 0
    puts "ERROR: $arc_gene_line"
    exit 1
} else {
    set arc_gene_line "$vecname  $vecdir  arc generate Success"
    set arc_gene 1
}
puts "  \[OK\] ARC 架构加载完成"
puts ""

# ==========================================================================
#  步骤 3: PACK —— 逻辑打包
# ==========================================================================
puts ">>> \[3/6\] PACK: 逻辑打包..."

set vecname $projectName
set vecdir  $projectDir
regsub -all {\\\\} $vecdir {/} vecdir
regsub -all {\\\\} $vecdir {/} resultdir

set comet02g  COMET02G
set comet03   COMET03
set er2c3000g ER2C3000G

set_global_assignment -name PackArchFileName ../Infrastructure/psk_intern/$seriesName/series_library/pack_guide.xml

# ---- 加载引脚约束文件 (如果存在) ----
set psf_file [file join $vecdir "$vecname.psf"]
if {[file exists $psf_file]} {
    puts "  \[INFO\] 加载引脚约束: $psf_file"
    source $psf_file
} else {
    puts "  \[WARN\] 未找到 .psf 引脚约束文件, 使用自动分配"
    puts "  \[WARN\] 请通过 eLinx GUI 生成管脚分配后, 导出 $vecname.psf"
}

# ---- 打包算法参数 ----
proc InitialAssignment {} {
    set_global_assignment -name PackAlgName        VPack
    set_global_assignment -name GainHashSize       16
    set_global_assignment -name PackAlgSeedPolicy  PINS_SEED_POLICY
    set_global_assignment -name PackAlgGainPolicy  SHAREDPINS_GAIN_POLICY
    set_global_assignment -name VPACK_ALPHA        50
    set_global_assignment -name CLB_MaxNumLEs      10
    set_global_assignment -name CLB_MaxNumIns      22
    set_global_assignment -name CLB_MaxNumAclrs    2
    set_global_assignment -name CLB_MaxNumEnas     2
    set_global_assignment -name BBFACTOR           8
    set_global_assignment -name PRES_FAC_MULT      1.5
    set_global_assignment -name ASTAR_FAC          1.0
}

if {$udm_gene == 1 && $arc_gene == 1} {
    regsub -all {\\} $vecdir {\\\\} packdir
    InitialAssignment

    set pack_param_type ChipName
    set_global_assignment -name ChipName $er2c3000g
    set pack_param_val $er2c3000g

    set_global_assignment -name ProjectName      $vecname
    set packvecdir $vecdir\\
    puts "  packvecdir = $packvecdir"
    set_global_assignment -name ProjectDirectory $packvecdir

    if {[catch {DoReadPackData} err_msg]} {
        set packline "$vecname  $vecdir  Error   --- $pack_param_type=$pack_param_val"
        set pack_gene 0
    } else {
        if {[catch {DoTVPack $vecdir $vecname} err_msg]} {
            set packline "$vecname  $vecdir  Error   --- $pack_param_type=$pack_param_val"
            set pack_gene 0
        } else {
            if {[catch {DoPostPackData $vecdir $vecname} err_msg]} {
                set packline "$vecname  $vecdir  Error   --- $pack_param_type=$pack_param_val"
                set pack_gene 0
            } else {
                if {[catch {DoGenerateVOFile $vecdir $vecname} err_msg]} {
                    set packline "$vecname  $vecdir  Error   --- $pack_param_type=$pack_param_val"
                    set pack_gene 0
                } else {
                    if {[catch {DoReleasePData $vecdir $vecname} err_msg]} {
                        set packline "$vecname  $vecdir  Error   --- $pack_param_type=$pack_param_val"
                        set pack_gene 0
                    } else {
                        set packline "$vecname  $vecdir  Success --- $pack_param_type=$pack_param_val"
                        set pack_gene 1
                    }
                }
            }
        }
    }
    puts "  pack_gene = $pack_gene"
    puts "  \[OK\] PACK 打包完成"
}

if {$pack_gene == 0} {
    puts "ERROR: PACK 打包失败"
    exit 1
}
puts ""

# ==========================================================================
#  步骤 4: PLACE —— 布局
# ==========================================================================
puts ">>> \[4/6\] PLACE: 布局..."

set_global_assignment -name INNERNUM 1
set_global_assignment -name ALPHA    0.5

if {[catch {read_port_info -f ../Infrastructure/psk_intern/$seriesName/series_library/place_route_guide.xml} err_msg]} {
    set read_file "$vecname $vecdir read_port_info Error:$err_msg"
    set read_xml 0
    puts "ERROR: $read_file"
    exit 1
} else {
    set read_file "$vecname $vecdir read_port_info Success"
    set read_xml 1
}

if {$udm_gene == 1 && $arc_gene == 1 && $pack_gene == 1 && $read_xml == 1} {
    if {[catch {try_place -f $resultdir/$vecname.place} err_msg]} {
        set result_line "$vecname $vecdir try_place Error:$err_msg"
        set result 0
        puts "ERROR: $result_line"
        exit 1
    } else {
        set result_line "$vecname $vecdir try_place Success"
        set result 1
    }
}
puts "  \[OK\] PLACE 布局完成"
puts ""

# ==========================================================================
#  步骤 5: ROUTE —— 布线
# ==========================================================================
puts ">>> \[5/6\] ROUTE: 布线..."

if {[catch {route_init_arc} err_msg]} {
    puts "ERROR: route_init_arc failed: $err_msg"
    exit 1
}

if {$result == 1} {
    # 布线预处理: UCM 模块处理
    if {[catch {route_pro_ucm} err_msg]} {
        set ucm_proc_line "$vecname $vecdir route_pro_ucm Error:$err_msg"
        set ucm_proc 0
        puts "ERROR: $ucm_proc_line"
        exit 1
    } else {
        set ucm_proc_line "$vecname $vecdir route_pro_ucm Success"
        set ucm_proc 1
    }

    # 布线预处理: UDM 模块处理
    if {[catch {route_pro_udm} err_msg]} {
        set udm_proc_line "$vecname  $vecdir  route_pro_udm Error:$err_msg"
        set udm_proc 0
        puts "ERROR: $udm_proc_line"
        exit 1
    } else {
        set udm_proc_line "$vecname  $vecdir  route_pro_udm Success"
        set udm_proc 1
    }

    if {$ucm_proc == 1 && $udm_proc == 1} {
        # 通道布线
        if {[catch {route_channel} err_msg]} {
            set route_channel_line "$vecname $vecdir route_channel Error:$err_msg"
            set route_channel 0
            puts "ERROR: $route_channel_line"
            exit 1
        } else {
            set route_channel_line "$vecname $vecdir route_channel Success"
            set route_channel 1
        }
    }

    if {$route_channel == 1} {
        # 内部布线
        if {[catch {route_internal} err_msg]} {
            set route_internal_line "$vecname $vecdir route_internal Error:$err_msg"
            set route_internal 0
            puts "ERROR: $route_internal_line"
            exit 1
        } else {
            set route_internal_line "$vecname $vecdir route_internal Success"
            set route_internal 1
        }
        set route_internal 1
    }

    if {$route_internal == 1} {
        # 写入布线结果
        if {[catch {route_write_results -f $resultdir/$vecname.route} err_msg]} {
            set writeudm_line "$vecname $vecdir write_routing_results -f $resultdir/$vecname.route Error:$err_msg"
            puts "ERROR: $writeudm_line"
            exit 1
        } else {
            set writeudm_line "$vecname $vecdir write_routing_results -f $resultdir/$vecname.route Success"
        }

        # 写入 UDM 布线数据
        if {[catch {route_write_results -udm} err_msg]} {
            set writeudm_line "$vecname $vecdir write_routing_results -udm Error:$err_msg"
            puts "ERROR: $writeudm_line"
            exit 1
        } else {
            set writeudm_line "$vecname $vecdir write_routing_results -udm Success"
            puts "  $writeudm_line"

            # 写入统计信息 (资源利用率、时序摘要)
            if {[catch {write_statistical_info $vecname -f $resultdir/$vecname.fit.result} err_msg]} {
                set write_statistical_line "$vecname $vecdir write statistical info Error:$err_msg"
                puts "  \[WARN\] $write_statistical_line"
            }
        }
    }

    route_delete_interface
    set route_circuit_line "$vecname $vecdir routing is completed"
}
puts "  \[OK\] ROUTE 布线完成"
puts ""

# ---- 保存设计检查点 ----
udm_save_checkpoint
udm_write_signal
udm_write_cluster

# ==========================================================================
#  步骤 6: BITGEN —— 比特流生成
# ==========================================================================
puts ">>> \[6/6\] BITGEN: 生成比特流..."

set_global_assignment -name PROJECT_NAME    $projectName
set_global_assignment -name COMPRESS_SWITCH ON
set_global_assignment -name GENBIN          ON

set vecname $projectName
set vecdir  $projectDir
regsub -all {\\\\} $vecdir {/} vecdir
regsub -all {\\\\} $vecdir {/} resultdir

set_global_assignment -name DEVICE_SERIES $seriesName
set_global_assignment -name DEVICE_NAME   $deviceName

# ---- 重新设置日志 (bitgen 阶段) ----
setlogfile $projectDir/log4cpp.property $projectDir/psk_fit.log

# ---- 加载 PSF (bitgen 需要引脚信息) ----
if {[file exists $psf_file]} {
    source $psf_file
}

# ---- 加载 ARC 配置模式数据 ----
set_arc_ucm_parameter STRUCTURE_PATH  ..\\Infrastructure\\psk_intern\\$seriesName\\$deviceName\\structure.dat

if {[catch {arc_deserialize_device_info} err_msg]} {
    set arc_gene_line "$vecname  $vecdir  arc generate Error:$err_msg"
    set arc_gene 0
    puts "ERROR: $arc_gene_line"
    exit 1
} else {
    set arc_gene_line "$vecname  $vecdir  arc generate Success"
    set arc_gene 1
}

set_arc_ucm_parameter SRAM_CORRELATION_PATH  ..\\Infrastructure\\psk_appendix\\$seriesName\\series_library\\correlation
set_arc_ucm_parameter CORRELATION_DAT_PATH   ..\\Infrastructure\\psk_intern\\$seriesName\\series_library\\correlation
set_arc_ucm_parameter CONFIGMODE_DAT_PATH    ..\\Infrastructure\\psk_intern\\$seriesName\\series_library\\configmode

arc_deserialize_cfgcell_info
arc_deserialize_configmode_info

# ---- BITGEN 导向文件 ----
set cur_dir [pwd]
set_global_assignment -name PASSKEY_PATH "$cur_dir"
set_global_assignment -name BITGEN_GUIDE  ..\\Infrastructure\\psk_intern\\$seriesName\\$deviceName\\bitgen_guide.xml

# ---- 加载设计数据 ----
udm_load_checkpoint
udm_parse_meminitial
udm_write_cluster
udm_write_signal

# ---- 生成比特流 (psk 格式) ----
set bitgen_error 0
if {[catch {bitgen_cfgcram} err_msg]} {
    puts "ERROR: bitgen_cfgcram failed: $err_msg"
    set bitgen_error 1
}
if {!$bitgen_error && [catch {bitgen_cfgbram} err_msg]} {
    puts "ERROR: bitgen_cfgbram failed: $err_msg"
    set bitgen_error 1
}
if {!$bitgen_error && [catch {bitgen_genfst} err_msg]} {
    puts "WARN: bitgen_genfst (慢仿) failed: $err_msg"
}
if {!$bitgen_error && [catch {bitgen_genmin} err_msg]} {
    puts "WARN: bitgen_genmin failed: $err_msg"
}
if {!$bitgen_error && [catch {bitgen_genpsk} err_msg]} {
    puts "ERROR: bitgen_genpsk failed: $err_msg"
    set bitgen_error 1
}
if {!$bitgen_error && [catch {bitgen_genrpt} err_msg]} {
    puts "WARN: bitgen_genrpt failed: $err_msg"
}

if {$bitgen_error} {
    puts "ERROR: BITGEN 比特流生成失败"
    exit 1
}

# ==========================================================================
#  构建完成
# ==========================================================================
puts ""
puts "========================================"
puts "  全流程构建成功!"
puts "========================================"
puts "  比特流文件: $projectDir/$projectName.psk"
puts "  布线结果:   $projectDir/$projectName.route"
puts "  布局结果:   $projectDir/$projectName.place"
puts "  适配报告:   $projectDir/$projectName.fit.result"
puts "  日志文件:   $projectDir/psk_fit.log"
puts "========================================"
puts ""
puts "  烧录命令 (JTAG):"
puts "    Programmer.exe --mode=jtag --file=$projectName.psk"
puts ""
puts "All was well!"
