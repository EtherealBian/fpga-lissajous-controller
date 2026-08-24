# 算法与文件索引

本文按“文件—算法—工程职责”列出仓库中的核心实现。可综合设计位于 `rtl/`，测试平台位于 `sim/`，Jetson Nano 视觉程序位于 `vision/`。

## 可综合 RTL

| 文件 | 算法或功能 | 在系统中的职责 |
|---|---|---|
| `rtl/lissajous_top.sv` | 多时钟域集成、翻转位脉冲跨域、ODDR 时钟转发、双 DAC 安全路由 | 工程顶层，连接时钟、按键、ADC/DAC、UART、LED 和各算法模块 |
| `rtl/lissajous_core.sv` | ADC 去中值与限幅、DPLL 相位模式选择、幅值缩放、DAC 编码 | 有线模式主信号链，生成对角线、圆/椭圆和“∞”图形 |
| `rtl/continuous_iq_dpll.sv` | 1 kHz 粗扫、100 Hz 细扫、全采样 I/Q 相关、CORDIC 双环 DPLL | 在 1～110 kHz 范围捕获并连续跟踪输入频率和相位 |
| `rtl/reference_frequency_calibrator.sv` | 复相关相位增量、Kay/最小二乘加权、末端迭代除法 | KEY4 启动的独立 100 kHz 时基标定，冻结 48 位 DDS 字 |
| `rtl/cordic_atan2.sv` | 24 轮向量模式 CORDIC | 计算复相关向量的 `atan2` 相角，不使用乘法器或浮点运算 |
| `rtl/dds_sine_lut.sv` | 第一象限压缩 LUT、象限镜像、相邻点线性插值 | 为有线 DPLL、标定器和无线 DDS 提供公共正弦采样 |
| `rtl/calibrated_sine_test_tone.sv` | 标定频率字比例换算、五档 DDS | DAC2 产生 1/20.4/50/80.3/100 kHz 校准测试音 |
| `rtl/wireless_sawtooth_pulse.sv` | 标定时基换算、相位斜坡、周期突发 | 无线识别阶段由 DAC1 输出 10 kHz 锯齿突发 |
| `rtl/wireless_commanded_dds.sv` | mHz 频率换算、Q0.16 周相位叠加、任意频率 DDS | 执行上位机扫频与调相命令 |
| `rtl/wireless_uart_controller.sv` | IDLE/RECOGNIZE/SCAN/ADJUST/DONE 状态机、回复/事件仲裁 | 无线流程控制面，保存频率与相位设定并检查命令上下文 |
| `rtl/uart_byte_rx.sv` | 分数波特率累加、起始位检测、位中心采样 | 接收 921600 8-N-1 串口字节 |
| `rtl/uart_byte_tx.sv` | 分数波特率累加、8-N-1 串行发送 | 发送 921600 串口字节 |
| `rtl/uart_packet_receiver.sv` | A5/5A 同步、变长载荷解析、超时恢复、CRC-8/ATM | 把字节流还原为可靠命令包 |
| `rtl/uart_packet_transmitter.sv` | 帧锁存、CRC-8/ATM、逐字节串行化 | 把 ACK、STATUS、START 等响应封装为数据包 |
| `rtl/unsigned_fraction_divider.sv` | 恢复余数长除法 | 在低速控制路径完成 34/64/80 位比例换算，避免组合大除法器 |
| `rtl/manual_control.sv` | 六键消抖事件汇总、图形/幅值循环、模式切换 | 把板载按键转换为有线设置或无线启动/取消事件 |
| `rtl/button_debounce.sv` | 两级同步、稳定时间判定、下降沿脉冲 | 对单个低有效机械按键消抖 |
| `rtl/status_leds.sv` | 状态到低有效 LED 的组合映射 | 显示无线识别/扫描/完成及有线标定状态 |
| `rtl/soft_power_on_reset.sv` | PLL 锁定后的定长软复位 | 无外部复位引脚时保证寄存器确定启动 |

## 验证与辅助软件

| 文件 | 验证内容 |
|---|---|
| `sim/tb_lissajous_top.sv` | 有线模式捕获、三种图形、四档幅值及按键隔离 |
| `sim/tb_continuous_iq_dpll.sv` | DPLL 多频点捕获、连续漂移修正和相位相关性 |
| `sim/tb_high_frequency_dpll.sv` | 97.8～100.005 kHz 高频边界和重捕获 |
| `sim/tb_reference_frequency_defaults.sv` | 标定器默认频率字和快速锁定实例 |
| `sim/tb_wired_calibration_output.sv` | KEY4 标定、DAC2 五档频率、DAC1/DAC2 隔离 |
| `sim/tb_wireless_uart_protocol.sv` | UART 正常/异常帧、状态约束、CRC 与 DDS 更新 |
| `sim/tb_wireless_dac_routing.sv` | 无线端到端命令、双 DAC 路由和取消流程 |
| `sim/clk_wiz_0_sim.sv`、`sim/oddr_sim.sv`、`sim/ila_0_sim.sv` | Xilinx IP/原语的仿真替代模型，不可加入综合源 |
| `tools/wireless_uart_host.py` | 数据包编解码、自检、串口交互参考实现 |
| `phase_sim/direct_calibration_sim.py` | 独立晶振直接电气标定误差模型 |
| `phase_sim/frequency_offset_sim.py` | 摄像头相位观测、鲁棒频偏估计和校正模型 |

## Jetson Nano 视觉闭环

| 文件或内部子系统 | 算法或功能 | 在系统中的职责 |
|---|---|---|
| `vision/jetson_freq.py` 主控层 | 摄像头采集、四档状态调度、粗测频融合、精扫会话、绝对相位搜索、完成后监控 | 接收 FPGA START 事件，组织完整视觉闭环并返回频率、判图和相位控制命令 |
| 内嵌 `frequency_measure_core` | 九点分片透视校正、右侧包络折返点检测、周期候选拟合、MAD 剔除、八帧中值融合、100 Hz 量化 | 在 20～100 kHz 观察范围内估计输入频率 |
| 内嵌 `range_switch_core` | 折返点/周期证据、多帧投票、迟滞阈值 | 在四档时基之间选择适合测频的观察窗口 |
| 内嵌 `simple_uart_protocol` | CRC-8/ATM、串口收发线程、协议帧解析 | 与 FPGA 的 921600 bps UART 状态机互操作 |
| 内嵌 `lissajous_judge_core` | 阈值分割、持久轨迹、全图白色占比、两帧投票 | 对 FPGA 的双侧 100 Hz 精扫测试点返回正确/错误判定 |
| 外部 `lissajous_phase_local` | `Config`、`Rectifier`、`calculate_phase`、`annotate` 接口 | 负责相位提取和调试标注；原始归档缺失，当前不能直接复现完整相位补偿 |

更完整的信号流和时钟域关系见 [系统架构](ARCHITECTURE.md)，命令字段见 [UART 协议](uart-protocol.md)。
