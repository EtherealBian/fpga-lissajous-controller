# 构建与验证

## RTL 仿真

需要 Icarus Verilog（支持 SystemVerilog）和 `vvp`：

```powershell
.\run_sim.ps1
```

选择专项测试平台：

```powershell
.\run_sim.ps1 -Testbench sim/tb_wireless_uart_protocol.sv `
  -Top tb_wireless_uart_protocol `
  -Output icarus/tb_wireless_uart_protocol.vvp `
  -Waveform sim/tb_wireless_uart_protocol.vcd
```

`sim/clk_wiz_0_sim.sv`、`sim/oddr_sim.sv` 和 `sim/ila_0_sim.sv` 仅供仿真，不能加入综合源。

## Python 自检

UART 编解码与 CRC：

```powershell
python tools/wireless_uart_host.py --self-test
```

频偏模型：

```powershell
python -m pip install -r phase_sim/requirements.txt
python phase_sim/test_direct_calibration_sim.py
python phase_sim/test_frequency_offset_sim.py
```

## Vivado

独立综合检查脚本支持 Vivado 2019.2 和 2025.2：

> Vivado 2025.2 在部分 Windows 环境中无法稳定处理含中文字符的工程路径。本仓库已在纯 ASCII 临时路径完成综合检查；建议克隆到 `D:\fpga\nuedc-f` 等纯英文路径后运行 Vivado。

```powershell
& "$env:VIVADO_HOME\bin\vivado.bat" -mode batch `
  -source scripts/vivado_synth_check.tcl -nojournal -nolog
```

完整实验流程脚本按 Vivado 2025.2 编写，可通过参数或环境变量指定安装目录：

```powershell
.\vivado_build.ps1 -VivadoHome C:\Xilinx\Vivado\2025.2
```

## eLinx 3.0

```powershell
.\elinx_build.ps1 -ELinxHome C:\eLinx3.0 -Device EQ6HL130
```

eLinx 脚本为实验迁移流程，目标器件、IP 替代和引脚约束需在实际工程中重新确认。

## 生成 bitstream 前必须完成

1. 生成端口匹配的 `clk_wiz_0` 与 `ila_0` IP。
2. 按实际 AD/DA 板卡补充所有引脚、IOSTANDARD 和输入/输出延迟约束。
3. 确认外部模拟级是否反相；若硬件不反相，应移除顶层数字反相补偿。
4. 完成实现后时序分析，不以综合阶段 WNS 代替最终时序结论。
5. 在示波器和全频段信号源上复测幅值、相位、控制时间和稳定时间。
