# Jetson Nano 视觉测频与闭环控制

`jetson_freq.py` 是本作品运行在 Jetson Nano 上的视觉部分主程序，作者为 Ethereal。它通过摄像头观察示波器李萨如图形，并经 921600 bps UART 与 FPGA 协同完成自动选档、测频、精扫、相位补偿、完成判定和频率变化监控。

## 文件与算法

| 文件或模块 | 功能 |
|---|---|
| `jetson_freq.py` 顶层 | 主事件循环、状态管理、摄像头采集、四档切换、八帧融合、精扫与相位闭环 |
| 内嵌 `frequency_measure_core` | 8×8 九点透视校正、右侧包络折返点检测、周期拟合、MAD 异常值剔除和 100 Hz 量化 |
| 内嵌 `range_switch_core` | 根据折返点数量、周期和多帧投票在四个观察档位之间切换 |
| 内嵌 `simple_uart_protocol` | 921600 bps、8-N-1、CRC-8/ATM 帧协议及收发线程 |
| 内嵌 `lissajous_judge_core` | 二值化、持久轨迹融合、白色像素比例和两帧投票，用于 100 Hz 双侧精扫判图 |
| 外部 `lissajous_phase_local` | 相位拟合、相位计算与调试标注；当前归档中缺失，详见“已知限制” |

四个核心模块以内嵌源码字符串形式保存在同一个 Python 文件中，目的是减少 Jetson 现场部署时的多文件版本不一致问题。

## 公开整理说明

公开整理时补充了作者与模块边界说明，并修复原始合并文件中的两处纯语法缺陷：将误用作 Python 引号的中文智能引号改回标准引号；为缩进异常的 `raise ProtocolError` 补回文档所述的 `10 <= frequency_code <= 1000` 范围判断。其余算法参数、状态机和通信字段保持参赛归档内容。

## 运行环境

- Jetson Nano 与 JetPack 提供的 Python 3、OpenCV/GStreamer 环境
- USB 或 CSI 摄像头
- Mizar Z7 FPGA 串口，默认设备 `/dev/ttyTHS1`
- Python 包：NumPy、PySerial；若系统未提供 `cv2`，还需要与 JetPack 兼容的 OpenCV Python 包

安装通用 Python 依赖：

```bash
python3 -m pip install -r vision/requirements.txt
```

Jetson 上通常优先使用 JetPack 自带的硬件加速 OpenCV，不建议在未确认兼容性时用 PyPI 的 `opencv-python` 覆盖系统版本。

## 标定与配置文件

程序默认在当前工作目录读取：

- `scope_8x8_nine_point_calibration.json`：示波器 8×8 网格九点标定数据；首次部署需按实际摄像头视角生成。
- `range_profiles.json`：四档观察参数；文件不存在时使用程序内置安全参数。

这些文件与摄像头安装位置、焦距、示波器画面和现场参数相关，不属于可复用源码，因此仓库不附带个人设备的标定结果。

## 启动方式

在仓库根目录运行：

```bash
# 联机闭环：默认 /dev/ttyTHS1，921600 bps
python3 vision/jetson_freq.py

# 不连接 FPGA，仅使用摄像头验证测频流程
python3 vision/jetson_freq.py --standalone

# 显示视觉调试画面
python3 vision/jetson_freq.py --display

# 查看全部摄像头、串口和算法参数
python3 vision/jetson_freq.py --help
```

访问板载串口可能需要把运行用户加入 `dialout` 组，并确认 Jetson 串口未被系统控制台占用。

## 与 FPGA 的闭环流程

```text
FPGA START
  -> Jetson 四档自动选档
  -> 八帧粗测频与 100 Hz 量化
  -> FREQUENCY_RESULT
  -> FPGA 依次输出 0、-100、+100、-200、+200 ... Hz 测试点
  -> Jetson 双帧投票返回 SCAN_CORRECT / SCAN_WRONG
  -> SET_PHASE 绝对相位搜索与图形几何判定
  -> DONE
  -> 持续监视频率变化，必要时触发 RESCAN
```

帧格式、命令编号和状态约束见 [`docs/uart-protocol.md`](../docs/uart-protocol.md)。

## 已知限制

原始项目目录未包含 `lissajous_phase_local.py`。`jetson_freq.py` 调用该模块的 `Config`、`Rectifier`、`calculate_phase` 和 `annotate` 接口，因此当前公开版本可以完整阅读测频、选档、UART 和精扫算法，但在补回相位模块前不能直接启动完整闭环程序。

仓库没有伪造或用空实现替代参赛算法。若找到原模块，应将其放在 `vision/lissajous_phase_local.py`，随后重新执行语法、导入和硬件联调验证。
