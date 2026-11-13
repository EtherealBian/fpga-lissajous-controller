# FPGA Lissajous Controller

[中文](#中文) · [English](#english)

![FPGA Lissajous controller signal path](docs/project-overview.png)

## 中文

2026 年电赛江苏赛区 F 题省冠军作品的开源整理版。工程包含 FPGA 数字锁相与双 DAC 输出、无线 UART 控制，以及 Jetson Nano 视觉测频与反馈。

### 工作原理

1. **连续锁相**。AD1 输入经 I/Q 相关粗扫与细扫。进入跟踪后，CORDIC 提取相关相角，校正 48 位 DDS 的频率字和相位。
2. **双通路输出**。有线 DAC1 按锁定相位合成同相、正交或二倍频波形。独立的 100 kHz 校准结果供 DAC2 和无线模式使用。
3. **视觉反馈**。Jetson 从李萨如图像估计频率与相位，经 UART 与 FPGA 交换控制信息。

### 构建与验证

```powershell
python tools/wireless_uart_host.py --self-test
.un_sim.ps1
```

RTL 仿真需要 Icarus Verilog。有线基线为 Mizar Z7 / XC7Z020。板级实现需补齐引脚和时序约束。归档缺少 `lissajous_phase_local.py`，完整视觉相位闭环仍需恢复该模块和相机标定。

[构建与测试](docs/BUILD_AND_TEST.md) · [系统架构](docs/ARCHITECTURE.md) · [算法索引](docs/ALGORITHM_INDEX.md) · [Jetson 说明](vision/README.md) · [赛题来源](docs/PROBLEM_SOURCE.md)

## English

This project won first place for Problem F in the 2026 Jiangsu regional electronic design contest. It implements FPGA digital phase locking, dual DAC output, wireless UART control, and a Jetson Nano vision path for Lissajous feedback.

The wired path uses coarse and fine I/Q correlation searches, followed by CORDIC phase estimation to update a 48-bit DDS. DAC1 synthesizes in-phase, quadrature, or doubled-frequency outputs. An independent 100 kHz calibration supplies DAC2 and wireless mode. The vision path estimates frequency and phase from the displayed pattern and exchanges control data over UART.

Run the UART self-test and Icarus Verilog simulation using the commands above. Board implementation requires the final pin and timing constraints. Full vision-loop operation requires the missing `lissajous_phase_local.py` module and camera calibration. See the [build guide](docs/BUILD_AND_TEST.md), [architecture](docs/ARCHITECTURE.md), and [Jetson guide](vision/README.md).
