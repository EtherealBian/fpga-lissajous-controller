#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
==============================================================================
  Jetson Nano 视觉测频与闭环控制 —— 单文件嵌入式模块架构
==============================================================================

  赛题：2026年全国大学生电子设计竞赛 F题 —— 李萨如图形显示控制装置
  作者：Ethereal
  硬件：Jetson Nano + USB/CSI摄像头 + Mizar Z7 FPGA (XC7Z020)
  通信：UART 921600 bps 8-N-1, 自定义帧协议 (CRC-8/ATM)

  整体数据流
  ──────────
  FPGA START 事件
      │
      ▼
  四档自动选档 (gear 0→1→2→3，右半网格轻量判断)
      │  每档基于右侧包络折返点数量/周期决定是否切到更长观察窗
      ▼
  八帧粗测频 (完整九点校正 + 右侧全高度格点拟合 + 多帧鲁棒融合)
      │  单帧→主频簇筛选→MAD剔除→median融合→100 Hz量化
      ▼
  FREQUENCY_RESULT → FPGA
      │  2字节频率码 (单位100 Hz, u16 小端)
      ▼
  双侧100 Hz精扫判定 (FPGA序列 0,-1,+1,-2,+2,... → Jetson逐点判图)
      │  阈值二值化 + 全图白色像素占比 + 两帧稳定窗口投票
      ▼
  SCAN_CORRECT → 相位补偿 (SET_PHASE绝对值搜索 + 方向验证)
      │  对角线→直线拟合, 圆→椭圆+霍夫验证, ∞→中心交叉高度
      ▼
  DONE → 保持输出 → 声光提示 → 持续监控外部频率变化 → RESCAN

  文件结构
  ────────
  本文件通过 _load_embedded_module() 将四个子系统以内嵌字符串形式加载：
    - measure_core  : 8×8九点标定 + 右侧全高度格点测频 + 八帧融合 (V6)
    - switch_core   : 四档快速切换判断 (只判断不计算频率)
    - uart_core     : 与 FPGA 兼容的 CRC-8/ATM 串口帧编解码
    - lissajous_core: 精扫判图引擎 (多帧持久轨迹 + 全局白色占比)

  相位提取另依赖 lissajous_phase_local 模块，负责左右双分支相位拟合、
  李萨如图相位计算和调试画面标注。原始归档未包含该模块，接口与影响
  详见本目录 README；除补回该依赖外，本文件保持参赛版本控制逻辑。

  主控层 (本文件顶层代码) 负责：
    - UART 921600 bps 帧编解码与协议状态机 (FpgaCompatibleUartLink)
    - 四档选档器驱动与量程切换决策
    - 八帧粗测频参数组装与融合器调度
    - 100 Hz 精扫会话管理 (SCAN_TEST/SCAN_CORRECT/SCAN_WRONG)
    - 相位补偿闭环 (SET_PHASE 搜索 + 霍夫圆验证 + ∞死区逃脱)
    - 主事件循环 (START → run_once → fine_sweep → phase → DONE → monitor)

  命令行
  ──────
  # 联机模式 (UART /dev/ttyTHS1)
  python jetson_freq.py

  # 本地模拟 (不连FPGA, 直接摄像头测频)
  python jetson_freq.py --standalone

  # 显示调试画面
  python jetson_freq.py --display

  依赖
  ────
  python -m pip install numpy opencv-python pyserial
"""
from __future__ import annotations
import sys
import types


def _load_embedded_module(name: str, source: str):
    """动态加载内嵌模块为Python包。

    本文件将三个独立子系统 (measure_core/switch_core/lissajous_core)
    的源码以内嵌字符串形式保存，通过 exec() 编译并注册到 sys.modules，
    避免多文件部署的复杂性。所有模块共享同一个 Python 解释器进程。

    Args:
        name: 模块全限定名，如 'measure_core'
        source: 模块的完整 Python 源码字符串

    Returns:
        已加载的 module 对象，可直接通过 . 访问其类与函数
    """
    module = types.ModuleType(name)
    module.__file__ = '<embedded:{}>'.format(name)
    module.__package__ = ''
    sys.modules[name] = module
    exec(compile(source, module.__file__, 'exec'), module.__dict__)
    return module

MEASURE_CORE_SOURCE = '#!/usr/bin/env python3\n# -*- coding: utf-8 -*-\n"""\n8×8九点标定 + 右侧全高度格点测频 V6（Jetson八帧融合版）\n\n核心修正：\n1. 九点标定后的0～8 div全部参与测量；\n2. 不再把顶部、中央横轴、底部整块区域硬删除；\n3. 只移除“细而长”的水平网格/边框线；\n4. 从右侧包络X(y)寻找折返点；\n5. 周期候选不仅匹配已检测点，还会在预测格点位置重新搜索漏掉的点；\n6. 顶部、中央、底部的真实折返点均可参与最终拟合；\n7. 只使用右侧，避免左侧箭头和回扫结构干扰；\n8. 每张图片一次remap，strict成功后不执行后备档；\n9. 每8个有效帧先剔除异常值，再进行中值/众数/截尾均值融合；\n10. 最终输出按100 Hz步进量化。\n\n适用：\n- 8×8网格；\n- Jetson USB/CSI摄像头连续视频流；\n- 默认20～100 kHz；\n- 输入2740×1824或与标定图构图一致的其他分辨率。\n\n依赖：\n    python -m pip install numpy opencv-python\n"""\nfrom __future__ import annotations\nimport argparse\nimport csv\nimport json\nimport threading\nimport math\nimport re\nimport time\nfrom collections import deque\nfrom dataclasses import dataclass\nfrom pathlib import Path\nfrom typing import Optional\nimport cv2\nimport numpy as np\nVERSION = \'8X8-RIGHT-GRID-RECOVERY-JETSON-V6.0-8FRAME-FUSION\'\nCLICK_ORDER = [(\'TOP_LEFT\', 0, 0), (\'TOP_CENTER\', 0, 1), (\'TOP_RIGHT\', 0, 2), (\'BOTTOM_LEFT\', 2, 0), (\'BOTTOM_CENTER\', 2, 1), (\'BOTTOM_RIGHT\', 2, 2), (\'MIDDLE_LEFT\', 1, 0), (\'CENTER\', 1, 1), (\'MIDDLE_RIGHT\', 1, 2)]\n\n@dataclass(frozen=True)\nclass Config:\n    pixels_per_div: int = 160\n    frequency_min_hz: float = 20000.0\n    frequency_max_hz: float = 100000.0\n\n    # 标定模型：\n    #   scalar:             f = k_hz_div / p\n    #   reciprocal_affine:  f = k_hz_div / p + frequency_offset_hz\n    calibration_model: str = \'scalar\'\n    k_hz_div: float = 80000\n    frequency_offset_hz: float = 0.0\n    enable_boundary_rescue: bool = True\n\n    right_search_start_div: float = 4.2\n    right_tip_min_x_div: float = 7.2\n    horizontal_close_div: float = 0.02\n    horizontal_open_div: float = 0.06\n    long_line_length_div: float = 1.2\n    long_line_vertical_dilate_px: int = 1\n    profile_sigma_div: float = 0.04\n    peak_radius_div: float = 0.18\n    shoulder_inner_div: float = 0.14\n    shoulder_outer_div: float = 0.38\n    minimum_prominence_div: float = 0.18\n    minimum_support_width_div: float = 0.055\n    minimum_tip_spacing_div: float = 0.34\n    period_margin: float = 0.07\n    candidate_quantum_div: float = 0.003\n    maximum_candidates: int = 40\n    initial_match_tolerance_div: float = 0.16\n    recovery_window_ratio: float = 0.18\n    recovery_window_max_div: float = 0.26\n    recovery_min_prominence_div: float = 0.12\n    fit_outlier_div: float = 0.085\n    minimum_occupancy: float = 0.7\n    maximum_rms_div: float = 0.08\n    boundary_guard_ratio: float = 0.07\n    boundary_rms_div: float = 0.095\n\n    @property\n    def width(self) -> int:\n        return 8 * self.pixels_per_div + 1\n\n    @property\n    def height(self) -> int:\n        return 8 * self.pixels_per_div + 1\n\n    def frequency_from_period(\n        self,\n        period_div: float,\n    ) -> float:\n        if period_div <= 0:\n            return float(\'nan\')\n\n        return (\n            self.k_hz_div / period_div\n            + self.frequency_offset_hz\n        )\n\n    def period_from_frequency(\n        self,\n        frequency_hz: float,\n    ) -> float:\n        denominator = (\n            frequency_hz\n            - self.frequency_offset_hz\n        )\n\n        if denominator <= 0:\n            raise ValueError(\n                \'标定模型导致frequency-b<=0\'\n            )\n\n        return self.k_hz_div / denominator\n\n    @property\n    def nominal_period_min_div(self) -> float:\n        return self.period_from_frequency(\n            self.frequency_max_hz\n        )\n\n    @property\n    def nominal_period_max_div(self) -> float:\n        return self.period_from_frequency(\n            self.frequency_min_hz\n        )\n\n    @property\n    def period_min_div(self) -> float:\n        return (\n            self.nominal_period_min_div\n            * (1.0 - self.period_margin)\n        )\n\n    @property\n    def period_max_div(self) -> float:\n        return (\n            self.nominal_period_max_div\n            * (1.0 + self.period_margin)\n        )\n\n\n@dataclass\nclass Detection:\n    y_px: np.ndarray\n    quality: np.ndarray\n    profile_x_px: np.ndarray\n    raw_profile_x_px: np.ndarray\n    threshold: float\n    mask: np.ndarray\n    mode: str\n\n@dataclass\nclass Fit:\n    ok: bool\n    period_px: float = float(\'nan\')\n    phase_px: float = float(\'nan\')\n    rms_px: float = float(\'inf\')\n    occupancy: float = 0.0\n    score: float = float(\'-inf\')\n    used_y_px: np.ndarray = None\n    indices: np.ndarray = None\n    detected_flags: np.ndarray = None\n    reason: str = \'UNKNOWN\'\n\n    def __post_init__(self) -> None:\n        if self.used_y_px is None:\n            self.used_y_px = np.empty(0, dtype=np.float64)\n        if self.indices is None:\n            self.indices = np.empty(0, dtype=np.int32)\n        if self.detected_flags is None:\n            self.detected_flags = np.empty(0, dtype=bool)\n\n@dataclass\nclass Result:\n    ok: bool\n    frequency_hz: float\n    raw_frequency_hz: float\n\n    # period_div可能经过边界一致性救援；\n    # measured_period_div始终保存视觉拟合的原始周期。\n    period_div: float\n    measured_period_div: float\n\n    rms_div: float\n    rectified: np.ndarray\n    detection: Detection\n    fit: Fit\n    elapsed_ms: float\n    boundary_rescued: bool = False\n\n\ndef natural_key(path: Path):\n    return [int(part) if part.isdigit() else part.lower() for part in re.split(\'(\\\\d+)\', path.name)]\n\ndef load_gray(path: Path) -> np.ndarray:\n    image = cv2.imread(str(path), cv2.IMREAD_UNCHANGED)\n    if image is None:\n        raise RuntimeError(f\'无法读取图片：{path}\')\n    if image.ndim == 3:\n        if image.shape[2] == 4:\n            image = cv2.cvtColor(image, cv2.COLOR_BGRA2GRAY)\n        else:\n            image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)\n    if image.dtype == np.uint8:\n        return image\n    array = image.astype(np.float32)\n    low, high = np.percentile(array, [0.2, 99.8])\n    if high <= low:\n        return np.zeros(array.shape, dtype=np.uint8)\n    return np.rint(np.clip((array - low) / (high - low), 0.0, 1.0) * 255.0).astype(np.uint8)\n\ndef validate_points(points: np.ndarray) -> None:\n    points = np.asarray(points, dtype=np.float32)\n    if points.shape != (3, 3, 2):\n        raise ValueError(\'九点标定数组必须为(3,3,2)\')\n    if np.any(np.diff(points[:, :, 0], axis=1) <= 0):\n        raise ValueError(\'九点左右顺序错误\')\n    if np.any(np.diff(points[:, :, 1], axis=0) <= 0):\n        raise ValueError(\'九点上下顺序错误\')\n    for row in range(2):\n        for column in range(2):\n            quad = np.float32([points[row, column], points[row, column + 1], points[row + 1, column + 1], points[row + 1, column]])\n            if abs(cv2.contourArea(quad)) < 100.0:\n                raise ValueError(\'九点标定存在退化区域\')\n\ndef select_nine_points(gray: np.ndarray) -> np.ndarray:\n    height, width = gray.shape\n    scale = min(1.0, 1600.0 / width, 980.0 / height)\n    view = cv2.resize(gray, (int(round(width * scale)), int(round(height * scale))), interpolation=cv2.INTER_AREA)\n    selected: list[tuple[int, int]] = []\n    mouse = [0, 0]\n    window = \'8x8 NINE POINT | Enter=OK R=reset Backspace=undo Esc=quit\'\n\n    def callback(event, x, y, flags, userdata):\n        mouse[:] = [x, y]\n        if event == cv2.EVENT_LBUTTONDOWN and len(selected) < 9:\n            selected.append((x, y))\n            next_name = CLICK_ORDER[len(selected)][0] if len(selected) < 9 else \'PRESS ENTER\'\n            print(\'[Calibration] next:\', next_name)\n    print(\'[Calibration] 顶部左中右 -> 底部左中右 -> 中间左中右\')\n    print(\'[Calibration] next: TOP_LEFT\')\n    cv2.namedWindow(window, cv2.WINDOW_AUTOSIZE)\n    cv2.setMouseCallback(window, callback)\n    while True:\n        shown = cv2.cvtColor(view, cv2.COLOR_GRAY2BGR)\n        cv2.line(shown, (mouse[0], 0), (mouse[0], shown.shape[0] - 1), (0, 255, 255), 1)\n        cv2.line(shown, (0, mouse[1]), (shown.shape[1] - 1, mouse[1]), (0, 255, 255), 1)\n        for index, point in enumerate(selected):\n            cv2.circle(shown, point, 7, (0, 0, 255), -1, cv2.LINE_AA)\n            cv2.putText(shown, str(index + 1), (point[0] + 8, max(20, point[1] - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.62, (0, 0, 255), 2, cv2.LINE_AA)\n        next_name = CLICK_ORDER[len(selected)][0] if len(selected) < 9 else \'PRESS ENTER\'\n        cv2.putText(shown, f\'NEXT: {next_name}\', (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.72, (0, 255, 255), 2, cv2.LINE_AA)\n        cv2.imshow(window, shown)\n        key = cv2.waitKey(15) & 255\n        if key in (10, 13) and len(selected) == 9:\n            break\n        if key in (ord(\'r\'), ord(\'R\')):\n            selected.clear()\n        if key in (8, 127) and selected:\n            selected.pop()\n        if key == 27:\n            cv2.destroyWindow(window)\n            raise RuntimeError(\'取消标定\')\n    cv2.destroyWindow(window)\n    points = np.zeros((3, 3, 2), dtype=np.float32)\n    for display_point, (_, row, column) in zip(selected, CLICK_ORDER):\n        points[row, column] = np.asarray(display_point, dtype=np.float32) / scale\n    validate_points(points)\n    return points\n\ndef save_calibration(path: Path, points: np.ndarray, image_shape: tuple[int, int]) -> None:\n    path.write_text(json.dumps({\'version\': 5, \'type\': \'8x8_nine_point_piecewise_dense_map\', \'image_width\': int(image_shape[1]), \'image_height\': int(image_shape[0]), \'points_3x3\': np.asarray(points, float).tolist()}, ensure_ascii=False, indent=2), encoding=\'utf-8\')\n\ndef load_calibration(path: Path, image_shape: tuple[int, int]) -> np.ndarray:\n    data = json.loads(path.read_text(encoding=\'utf-8\'))\n    points = np.asarray(data[\'points_3x3\'], dtype=np.float32)\n    points[:, :, 0] *= image_shape[1] / float(data[\'image_width\'])\n    points[:, :, 1] *= image_shape[0] / float(data[\'image_height\'])\n    validate_points(points)\n    return points\n\nclass Rectifier:\n    """九点形成四个4×4区域，并预生成一张稠密映射。"""\n\n    def __init__(self, points: np.ndarray, cfg: Config):\n        self.cfg = cfg\n        self.map_x = np.zeros((cfg.height, cfg.width), dtype=np.float32)\n        self.map_y = np.zeros_like(self.map_x)\n        half = 4 * cfg.pixels_per_div\n        for row in range(2):\n            for column in range(2):\n                source = np.float32([points[row, column], points[row, column + 1], points[row + 1, column + 1], points[row + 1, column]])\n                destination = np.float32([[column * half, row * half], [(column + 1) * half, row * half], [(column + 1) * half, (row + 1) * half], [column * half, (row + 1) * half]])\n                inverse = cv2.getPerspectiveTransform(destination, source)\n                x0 = column * half\n                y0 = row * half\n                x1 = (column + 1) * half if column == 0 else cfg.width - 1\n                y1 = (row + 1) * half if row == 0 else cfg.height - 1\n                xx, yy = np.meshgrid(np.arange(x0, x1 + 1, dtype=np.float32), np.arange(y0, y1 + 1, dtype=np.float32))\n                denominator = inverse[2, 0] * xx + inverse[2, 1] * yy + inverse[2, 2]\n                self.map_x[y0:y1 + 1, x0:x1 + 1] = (inverse[0, 0] * xx + inverse[0, 1] * yy + inverse[0, 2]) / denominator\n                self.map_y[y0:y1 + 1, x0:x1 + 1] = (inverse[1, 0] * xx + inverse[1, 1] * yy + inverse[1, 2]) / denominator\n\n    def __call__(self, gray: np.ndarray) -> np.ndarray:\n        return cv2.remap(gray, self.map_x, self.map_y, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_REPLICATE)\n\ndef remove_long_horizontal_lines(mask: np.ndarray, cfg: Config) -> np.ndarray:\n    """\n    只移除细长水平结构，不再删除顶部/中央/底部整个区域。\n\n    九点校正后，网格横线接近水平；波形斜边和圆弧不会被长水平\n    核完整保留，因此可以保住真实折返点。\n    """\n    ppd = cfg.pixels_per_div\n    kernel_width = max(11, int(round(cfg.long_line_length_div * ppd)))\n    long_lines = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((1, kernel_width), dtype=np.uint8))\n    if cfg.long_line_vertical_dilate_px > 0:\n        long_lines = cv2.dilate(long_lines, np.ones((2 * cfg.long_line_vertical_dilate_px + 1, 1), dtype=np.uint8))\n    return cv2.bitwise_and(mask, cv2.bitwise_not(long_lines))\n\ndef detection_mask(gray: np.ndarray, cfg: Config, mode: str) -> tuple[np.ndarray, float, np.ndarray]:\n    if mode == \'strict\':\n        processed = gray\n        offset = 20.0\n        minimum = 145.0\n        open_div = cfg.horizontal_open_div\n    elif mode == \'medium\':\n        processed = gray\n        offset = 5.0\n        minimum = 105.0\n        open_div = 0.045\n    elif mode == \'low\':\n        processed = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8)).apply(gray)\n        offset = -8.0\n        minimum = 70.0\n        open_div = 0.03\n    else:\n        raise ValueError(mode)\n    otsu, _ = cv2.threshold(processed, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)\n    threshold = float(np.clip(otsu + offset, minimum, 220.0))\n    mask = (processed >= threshold).astype(np.uint8)\n    ppd = cfg.pixels_per_div\n    close_width = max(3, int(round(cfg.horizontal_close_div * ppd)))\n    open_width = max(3, int(round(open_div * ppd)))\n    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((1, close_width), dtype=np.uint8))\n    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((1, open_width), dtype=np.uint8))\n    mask = remove_long_horizontal_lines(mask, cfg)\n    return (mask, threshold, processed)\n\ndef interpolate_profile(values: np.ndarray, valid: np.ndarray) -> Optional[np.ndarray]:\n    rows = np.arange(len(values), dtype=np.float32)\n    good = np.flatnonzero(valid)\n    if len(good) < 2:\n        return None\n    return np.interp(rows, good.astype(np.float32), values[good].astype(np.float32)).astype(np.float32)\n\ndef true_regions(flags: np.ndarray) -> list[tuple[int, int]]:\n    padded = np.r_[False, flags, False].astype(np.int8)\n    change = np.diff(padded)\n    return list(zip(np.flatnonzero(change == 1), np.flatnonzero(change == -1)))\n\ndef crossing(signal: np.ndarray, start: int, stop: int, level: float, direction: int) -> float:\n    index = start\n    while index != stop:\n        next_index = index + direction\n        if (signal[index] - level) * (signal[next_index] - level) <= 0:\n            y0 = float(signal[index])\n            y1 = float(signal[next_index])\n            if abs(y1 - y0) < 1e-09:\n                return float(index)\n            return float(index + direction * (level - y0) / (y1 - y0))\n        index = next_index\n    return float(stop)\n\ndef peak_evidence(profile: np.ndarray, center: int, cfg: Config) -> Optional[tuple[float, float]]:\n    """\n    返回亚像素Y和质量。\n\n    内部折返点使用左右肩部；\n    靠近顶部/底部时允许单侧肩部，但质量会降低。\n    """\n    ppd = cfg.pixels_per_div\n    inner = max(2, int(round(cfg.shoulder_inner_div * ppd)))\n    outer = max(inner + 2, int(round(cfg.shoulder_outer_div * ppd)))\n    left_available = center - inner >= 0\n    right_available = center + inner < len(profile)\n    if not left_available and (not right_available):\n        return None\n    peak = float(profile[center])\n    floors = []\n    if left_available:\n        left0 = max(0, center - outer)\n        left1 = center - inner + 1\n        if left1 > left0:\n            floors.append(float(np.min(profile[left0:left1])))\n    if right_available:\n        right0 = center + inner\n        right1 = min(len(profile), center + outer + 1)\n        if right1 > right0:\n            floors.append(float(np.min(profile[right0:right1])))\n    if not floors:\n        return None\n    prominence_px = peak - max(floors)\n    if prominence_px / ppd < cfg.recovery_min_prominence_div:\n        return None\n    support_level = max(cfg.right_tip_min_x_div * ppd, peak - 0.3 * ppd)\n    support_radius = max(4, int(round(0.18 * ppd)))\n    support_slice = profile[max(0, center - support_radius):min(len(profile), center + support_radius + 1)]\n    support_width = int(np.count_nonzero(support_slice >= support_level))\n    if support_width < cfg.minimum_support_width_div * ppd:\n        return None\n    baseline = max(floors)\n    centers = []\n    for fraction in (0.25, 0.4, 0.55):\n        level = baseline + fraction * max(peak - baseline, 1e-06)\n        if left_available:\n            left_cross = crossing(profile, center, max(0, center - outer), level, -1)\n        else:\n            left_cross = float(center)\n        if right_available:\n            right_cross = crossing(profile, center, min(len(profile) - 1, center + outer), level, 1)\n        else:\n            right_cross = float(center)\n        if left_available and right_available:\n            centers.append(0.5 * (left_cross + right_cross))\n        elif left_available:\n            centers.append(float(center))\n        else:\n            centers.append(float(center))\n    refined = float(np.median(centers))\n    boundary_penalty = 1.0 if left_available and right_available else 0.55\n    quality = boundary_penalty * (prominence_px / ppd + 0.03 * support_width)\n    return (refined, float(quality))\n\ndef detect_right_tips(gray: np.ndarray, mask: np.ndarray, threshold: float, cfg: Config, mode: str) -> Detection:\n    ppd = cfg.pixels_per_div\n    x0 = int(round(cfg.right_search_start_div * ppd))\n    roi = mask[:, x0:]\n    valid = np.any(roi != 0, axis=1)\n    extreme = (gray.shape[1] - 1 - np.argmax(roi[:, ::-1], axis=1)).astype(np.float32)\n    raw_profile = interpolate_profile(extreme, valid)\n    if raw_profile is None:\n        return Detection(np.empty(0), np.empty(0), np.zeros(gray.shape[0], dtype=np.float32), np.zeros(gray.shape[0], dtype=np.float32), threshold, mask, mode)\n    sigma = max(1.0, cfg.profile_sigma_div * ppd)\n    profile = cv2.GaussianBlur(raw_profile.reshape(-1, 1), (1, 0), sigmaX=0, sigmaY=sigma).reshape(-1)\n    radius = max(3, int(round(cfg.peak_radius_div * ppd)))\n    local_maximum = cv2.dilate(profile.reshape(-1, 1), np.ones((2 * radius + 1, 1), dtype=np.uint8)).reshape(-1)\n    flags = np.isclose(profile, local_maximum, atol=max(0.5, 0.01 * ppd))\n    flags &= profile >= cfg.right_tip_min_x_div * ppd\n    candidates = []\n    for begin, end in true_regions(flags):\n        center = begin + int(np.argmax(profile[begin:end]))\n        evidence = peak_evidence(profile, center, cfg)\n        if evidence is None:\n            continue\n        refined, quality = evidence\n        candidates.append((refined, quality, float(profile[center])))\n    candidates.sort(key=lambda item: item[1], reverse=True)\n    minimum_distance = cfg.minimum_tip_spacing_div * ppd\n    selected = []\n    for candidate in candidates:\n        if all((abs(candidate[0] - old[0]) >= minimum_distance for old in selected)):\n            selected.append(candidate)\n    selected.sort(key=lambda item: item[0])\n    return Detection(np.asarray([item[0] for item in selected], dtype=np.float64), np.asarray([item[1] for item in selected], dtype=np.float64), profile, raw_profile, threshold, mask, mode)\n\ndef weighted_line(indices: np.ndarray, y: np.ndarray, quality: np.ndarray) -> tuple[float, float]:\n    weight = np.sqrt(np.maximum(quality, 1e-06))\n    sum_w = float(np.sum(weight))\n    sum_x = float(np.sum(weight * indices))\n    sum_y = float(np.sum(weight * y))\n    sum_xx = float(np.sum(weight * indices * indices))\n    sum_xy = float(np.sum(weight * indices * y))\n    denominator = sum_w * sum_xx - sum_x * sum_x\n    if abs(denominator) < 1e-12:\n        return (float(\'nan\'), float(\'nan\'))\n    period = (sum_w * sum_xy - sum_x * sum_y) / denominator\n    phase = (sum_y - period * sum_x) / sum_w\n    return (float(period), float(phase))\n\ndef period_candidates(detected_y: np.ndarray, cfg: Config) -> list[float]:\n    ppd = cfg.pixels_per_div\n    minimum = cfg.period_min_div * ppd\n    maximum = cfg.period_max_div * ppd\n    raw: list[tuple[float, float]] = []\n    if len(detected_y) >= 2:\n        adjacent = np.diff(detected_y)\n        for distance in adjacent:\n            if minimum <= distance <= maximum:\n                raw.append((float(distance), 8.0))\n        valid_adjacent = adjacent[(adjacent >= minimum) & (adjacent <= maximum)]\n        if len(valid_adjacent):\n            raw.append((float(np.median(valid_adjacent)), 12.0))\n        for first in range(len(detected_y) - 1):\n            for second in range(first + 1, len(detected_y)):\n                distance = float(detected_y[second] - detected_y[first])\n                maximum_step = int(distance // minimum)\n                for step in range(1, maximum_step + 1):\n                    period = distance / step\n                    if minimum <= period <= maximum:\n                        weight = 5.0 / step ** 1.2 + 0.25 * (second - first)\n                        raw.append((period, weight))\n    raw.extend([(cfg.nominal_period_min_div * ppd, 1.0), (cfg.nominal_period_max_div * ppd, 1.0)])\n    quantum = max(0.25, cfg.candidate_quantum_div * ppd)\n    bins: dict[int, list[float]] = {}\n    for period, weight in raw:\n        key = int(round(period / quantum))\n        weighted_sum, total_weight = bins.get(key, [0.0, 0.0])\n        bins[key] = [weighted_sum + period * weight, total_weight + weight]\n    ranked = sorted(bins.values(), key=lambda item: item[1], reverse=True)\n    return [weighted_sum / total_weight for weighted_sum, total_weight in ranked[:cfg.maximum_candidates]]\n\ndef local_peak_near(profile: np.ndarray, predicted_y: float, period_px: float, cfg: Config) -> Optional[tuple[float, float]]:\n    ppd = cfg.pixels_per_div\n    half_window = int(round(min(cfg.recovery_window_max_div * ppd, cfg.recovery_window_ratio * period_px)))\n    half_window = max(5, half_window)\n    center = int(round(predicted_y))\n    begin = max(0, center - half_window)\n    end = min(len(profile), center + half_window + 1)\n    if end - begin < 3:\n        return None\n    local = profile[begin:end]\n    peak_index = begin + int(np.argmax(local))\n    if profile[peak_index] < cfg.right_tip_min_x_div * ppd:\n        return None\n    return peak_evidence(profile, peak_index, cfg)\n\ndef recover_lattice(detection: Detection, candidate_period: float, anchor: float, cfg: Config) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:\n    """\n    在整个0～8 div范围内按候选格点搜索。\n\n    即使顶部/中央/底部的点没有进入初始候选，也能在预测位置重新找回。\n    """\n    height = len(detection.profile_x_px)\n    first = math.ceil((0.0 - anchor) / candidate_period)\n    last = math.floor((height - 1 - anchor) / candidate_period)\n    recovered_y = []\n    recovered_i = []\n    recovered_q = []\n    detected_flags = []\n    detected_y = detection.y_px\n    detected_q = detection.quality\n    match_tolerance = cfg.initial_match_tolerance_div * cfg.pixels_per_div\n    for index in range(first, last + 1):\n        predicted = anchor + index * candidate_period\n        if len(detected_y):\n            nearest = int(np.argmin(np.abs(detected_y - predicted)))\n            distance = abs(float(detected_y[nearest] - predicted))\n        else:\n            nearest = -1\n            distance = float(\'inf\')\n        if distance <= match_tolerance:\n            recovered_y.append(float(detected_y[nearest]))\n            recovered_i.append(index)\n            recovered_q.append(float(detected_q[nearest]) + 0.5)\n            detected_flags.append(True)\n            continue\n        evidence = local_peak_near(detection.profile_x_px, predicted, candidate_period, cfg)\n        if evidence is None:\n            continue\n        refined, quality = evidence\n        recovered_y.append(refined)\n        recovered_i.append(index)\n        recovered_q.append(quality)\n        detected_flags.append(False)\n    if not recovered_y:\n        return (np.empty(0), np.empty(0, dtype=np.int32), np.empty(0), np.empty(0, dtype=bool))\n    return (np.asarray(recovered_y, dtype=np.float64), np.asarray(recovered_i, dtype=np.int32), np.asarray(recovered_q, dtype=np.float64), np.asarray(detected_flags, dtype=bool))\n\ndef consecutive(indices: np.ndarray) -> bool:\n    if len(indices) < 2:\n        return True\n    return bool(np.all(np.diff(np.sort(indices)) == 1))\n\ndef evaluate_candidate(detection: Detection, candidate: float, anchor: float, cfg: Config) -> Optional[Fit]:\n    ppd = cfg.pixels_per_div\n    minimum = cfg.period_min_div * ppd\n    maximum = cfg.period_max_div * ppd\n    used_y, indices, quality, detected_flags = recover_lattice(detection, candidate, anchor, cfg)\n    if len(used_y) < 2:\n        return None\n    valid = np.ones(len(used_y), dtype=bool)\n    outlier = cfg.fit_outlier_div * ppd\n    for _ in range(3):\n        period, phase = weighted_line(indices[valid], used_y[valid], quality[valid])\n        if not np.isfinite(period):\n            return None\n        residual = np.abs(used_y - (period * indices + phase))\n        new_valid = residual <= outlier\n        if np.array_equal(valid, new_valid):\n            valid = new_valid\n            break\n        valid = new_valid\n    if np.count_nonzero(valid) < 2:\n        return None\n    period, phase = weighted_line(indices[valid], used_y[valid], quality[valid])\n    if not minimum <= period <= maximum:\n        return None\n    final_y = used_y[valid]\n    final_indices = indices[valid]\n    final_quality = quality[valid]\n    final_detected = detected_flags[valid]\n    if not consecutive(final_indices):\n        return None\n    residual = final_y - (period * final_indices + phase)\n    rms = float(np.sqrt(np.mean(residual * residual)))\n    expected = len(final_indices)\n    span = int(np.max(final_indices)) - int(np.min(final_indices)) + 1\n    occupancy = expected / max(span, 1)\n    if occupancy < cfg.minimum_occupancy or rms / ppd > cfg.maximum_rms_div:\n        return None\n    detected_count = int(np.count_nonzero(final_detected))\n    recovered_count = len(final_y) - detected_count\n    quality_score = float(np.sum(final_quality / max(float(np.max(final_quality)), 1e-09)))\n    score = 170.0 * len(final_y) + 75.0 * max(span - 1, 0) + 520.0 * occupancy + 35.0 * quality_score + 18.0 * (period / ppd) - 85.0 * (rms / ppd) - 18.0 * recovered_count\n    return Fit(True, float(period), float(phase), rms, float(occupancy), float(score), final_y, final_indices, final_detected, \'RIGHT_FULL_GRID\')\n\ndef fit_period(detection: Detection, cfg: Config) -> Fit:\n    if len(detection.y_px) < 2:\n        return Fit(False, reason=\'RIGHT_TIPS_LESS_THAN_2\')\n    best: Optional[Fit] = None\n    anchor_indices = list(dict.fromkeys([0, int(np.argmax(detection.quality)), len(detection.y_px) // 2, len(detection.y_px) - 1]))\n    for candidate in period_candidates(detection.y_px, cfg):\n        candidate_best: Optional[Fit] = None\n        for anchor_index in anchor_indices:\n            fit = evaluate_candidate(detection, candidate, float(detection.y_px[anchor_index]), cfg)\n            if fit is not None and (candidate_best is None or fit.score > candidate_best.score):\n                candidate_best = fit\n        if candidate_best is None:\n            continue\n        if best is None or candidate_best.score > best.score:\n            best = candidate_best\n        first = math.ceil((0.0 - candidate_best.phase_px) / candidate_best.period_px)\n        last = math.floor((cfg.height - 1 - candidate_best.phase_px) / candidate_best.period_px)\n        expected = max(last - first + 1, 0)\n        if len(candidate_best.used_y_px) >= expected and candidate_best.occupancy >= 0.999 and (candidate_best.rms_px / cfg.pixels_per_div <= 0.025):\n            best = candidate_best\n            break\n    if best is None:\n        return Fit(False, reason=\'RIGHT_FULL_GRID_REJECTED\')\n    refined = evaluate_candidate(detection, best.period_px, best.phase_px, cfg)\n    if refined is not None and refined.score >= best.score - 30.0:\n        best = refined\n    return best\n\ndef boundary_rescue(\n    fit: Fit,\n    raw_frequency: float,\n    cfg: Config,\n) -> tuple[Fit, float, bool]:\n    if not cfg.enable_boundary_rescue:\n        return fit, raw_frequency, False\n\n    if (\n        cfg.frequency_min_hz\n        <= raw_frequency\n        <= cfg.frequency_max_hz\n    ):\n        return fit, raw_frequency, False\n\n    if (\n        cfg.frequency_max_hz\n        < raw_frequency\n        <= cfg.frequency_max_hz\n        * (1.0 + cfg.boundary_guard_ratio)\n    ):\n        target_frequency = cfg.frequency_max_hz\n        reason = \'BOUNDARY_RESCUE_HIGH\'\n    elif (\n        cfg.frequency_min_hz\n        > raw_frequency\n        >= cfg.frequency_min_hz\n        * (1.0 - cfg.boundary_guard_ratio)\n    ):\n        target_frequency = cfg.frequency_min_hz\n        reason = \'BOUNDARY_RESCUE_LOW\'\n    else:\n        return fit, raw_frequency, False\n\n    target_period = (\n        cfg.period_from_frequency(\n            target_frequency\n        )\n        * cfg.pixels_per_div\n    )\n\n    phase = float(np.mean(\n        fit.used_y_px\n        - target_period * fit.indices\n    ))\n    residual = (\n        fit.used_y_px\n        - (\n            target_period\n            * fit.indices\n            + phase\n        )\n    )\n    rms = float(\n        np.sqrt(\n            np.mean(residual * residual)\n        )\n    )\n\n    if (\n        rms / cfg.pixels_per_div\n        > cfg.boundary_rms_div\n    ):\n        return fit, raw_frequency, False\n\n    rescued = Fit(\n        True,\n        float(target_period),\n        phase,\n        rms,\n        fit.occupancy,\n        fit.score,\n        fit.used_y_px,\n        fit.indices,\n        fit.detected_flags,\n        reason,\n    )\n\n    return rescued, float(target_frequency), True\n\n\ndef measure(\n    gray: np.ndarray,\n    rectifier: Optional[Rectifier],\n    cfg: Config,\n    already_rectified: bool = False,\n) -> Result:\n    start = time.perf_counter()\n\n    if already_rectified:\n        rectified = cv2.resize(\n            gray,\n            (cfg.width, cfg.height),\n            interpolation=cv2.INTER_AREA,\n        )\n    else:\n        if rectifier is None:\n            raise RuntimeError(\n                \'缺少九点标定映射\'\n            )\n\n        rectified = rectifier(gray)\n\n    best_failure = None\n\n    for mode in (\'strict\', \'medium\', \'low\'):\n        mask, threshold, processed = (\n            detection_mask(\n                rectified,\n                cfg,\n                mode,\n            )\n        )\n        detection = detect_right_tips(\n            processed,\n            mask,\n            threshold,\n            cfg,\n            mode,\n        )\n        fit = fit_period(\n            detection,\n            cfg,\n        )\n\n        if fit.ok:\n            measured_period_div = (\n                fit.period_px\n                / cfg.pixels_per_div\n            )\n            raw_frequency = (\n                cfg.frequency_from_period(\n                    measured_period_div\n                )\n            )\n\n            (\n                fit,\n                frequency,\n                rescued,\n            ) = boundary_rescue(\n                fit,\n                raw_frequency,\n                cfg,\n            )\n\n            accepted = (\n                cfg.frequency_min_hz\n                <= frequency\n                <= cfg.frequency_max_hz\n            )\n\n            # 关闭边界救援时用于采集标定数据。\n            # 此时允许周期搜索范围内的轻微越界值输出，\n            # 防止100 kHz/20 kHz被钳位后失去原始周期。\n            if not cfg.enable_boundary_rescue:\n                accepted = True\n\n            if accepted:\n                return Result(\n                    True,\n                    float(frequency),\n                    float(raw_frequency),\n                    float(\n                        fit.period_px\n                        / cfg.pixels_per_div\n                    ),\n                    float(measured_period_div),\n                    float(\n                        fit.rms_px\n                        / cfg.pixels_per_div\n                    ),\n                    rectified,\n                    detection,\n                    fit,\n                    (\n                        time.perf_counter()\n                        - start\n                    ) * 1000.0,\n                    rescued,\n                )\n\n            fit.reason = (\n                \'OUT_OF_FREQUENCY_RANGE\'\n            )\n\n        if (\n            best_failure is None\n            or len(detection.y_px)\n            > len(best_failure[0].y_px)\n        ):\n            best_failure = (\n                detection,\n                fit,\n            )\n\n    detection, fit = best_failure\n\n    return Result(\n        False,\n        float(\'nan\'),\n        float(\'nan\'),\n        float(\'nan\'),\n        float(\'nan\'),\n        float(\'inf\'),\n        rectified,\n        detection,\n        fit,\n        (\n            time.perf_counter()\n            - start\n        ) * 1000.0,\n    )\n\n\ndef annotate(result: Result, cfg: Config, name: str) -> np.ndarray:\n    shown = cv2.cvtColor(result.rectified, cv2.COLOR_GRAY2BGR)\n    ppd = cfg.pixels_per_div\n    for index in range(9):\n        coordinate = index * ppd\n        cv2.line(shown, (coordinate, 0), (coordinate, shown.shape[0] - 1), (255, 100, 0), 1, cv2.LINE_AA)\n        cv2.line(shown, (0, coordinate), (shown.shape[1] - 1, coordinate), (255, 100, 0), 1, cv2.LINE_AA)\n    x = int(round(7.82 * ppd))\n    for y in result.detection.y_px:\n        cv2.circle(shown, (x, int(round(y))), max(4, int(round(0.035 * ppd))), (0, 165, 255), 1, cv2.LINE_AA)\n    if result.fit.ok:\n        for y, detected in zip(result.fit.used_y_px, result.fit.detected_flags):\n            color = (0, 255, 255) if detected else (0, 255, 0)\n            cv2.circle(shown, (x, int(round(y))), max(6, int(round(0.052 * ppd))), color, 2, cv2.LINE_AA)\n    if result.ok:\n        detected_used = int(np.count_nonzero(result.fit.detected_flags))\n        recovered_used = len(result.fit.used_y_px) - detected_used\n        lines = [\n            name,\n            (\n                f\'output={result.frequency_hz / 1000:.5f} kHz, \'\n                f\'measured={result.raw_frequency_hz / 1000:.5f} kHz\'\n            ),\n            (\n                f\'period={result.period_div:.8f} div, \'\n                f\'period_raw={result.measured_period_div:.8f} div\'\n            ),\n            (\n                f\'right tips={len(result.fit.used_y_px)} used, \'\n                f\'{len(result.detection.y_px)} seed\'\n            ),\n            f\'direct/recovered={detected_used}/{recovered_used}\',\n            (\n                f\'RMS={result.rms_div:.6f} div, \'\n                f\'occupancy={result.fit.occupancy:.3f}\'\n            ),\n            f\'mode={result.detection.mode}/{result.fit.reason}\',\n            f\'time={result.elapsed_ms:.2f} ms\',\n        ]\n    else:\n        lines = [name, \'MEASUREMENT FAILED\', f\'reason={result.fit.reason}\', f\'right seed tips={len(result.detection.y_px)}\', f\'mode={result.detection.mode}\', f\'time={result.elapsed_ms:.2f} ms\']\n    for row, line in enumerate(lines):\n        cv2.putText(shown, line, (12, 26 + 24 * row), cv2.FONT_HERSHEY_SIMPLEX, 0.53, (255, 255, 255), 1, cv2.LINE_AA)\n    return shown\n\ndef calibration_preview(gray: np.ndarray, points: np.ndarray) -> np.ndarray:\n    shown = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)\n    for row in range(3):\n        cv2.polylines(shown, [np.rint(points[row]).astype(np.int32)], False, (0, 255, 255), 2, cv2.LINE_AA)\n    for column in range(3):\n        cv2.polylines(shown, [np.rint(points[:, column]).astype(np.int32)], False, (0, 255, 255), 2, cv2.LINE_AA)\n    for row in range(3):\n        for column in range(3):\n            point = tuple(np.rint(points[row, column]).astype(int))\n            cv2.circle(shown, point, max(5, int(round(shown.shape[1] / 350))), (0, 0, 255), -1, cv2.LINE_AA)\n    return shown\n\n\n@dataclass\nclass StableOutput:\n    """多帧融合后的输出状态。"""\n\n    valid: bool\n    frequency_hz: float\n    period_div: float\n\n    # 当前正在收集的有效帧数；融合完成后重新从0开始。\n    sample_count: int\n    batch_size: int\n\n    # 最近一次成功融合所使用/剔除的样本数。\n    accepted_count: int = 0\n    rejected_count: int = 0\n\n    # 本次update是否刚刚产生了一组新的融合结果。\n    updated: bool = False\n\n    # 本次8帧批次是否因为有效内点不足而被丢弃。\n    batch_rejected: bool = False\n\n    batch_index: int = 0\n    method: str = \'median\'\n\n\nclass MultiFrameFusion:\n    """\n    8帧鲁棒融合器。\n\n    每次只接收单帧视觉算法成功的结果，累计batch_size个有效结果后：\n    1. 用“最密集频率簇”排除明显跳点、倍频/分频误识别；\n    2. 在主簇内部使用MAD（中位绝对偏差）再次剔除异常值；\n    3. 对剩余结果执行median/mode/trimmed_mean融合；\n    4. 最终频率量化到frequency_step_hz，默认100 Hz步进；\n    5. 一组计算完成后清空本组数据，开始下一组8帧。\n\n    注意：这里的8帧是“8个测量成功的有效帧”，识别失败帧不进入计算。\n    """\n\n    def __init__(\n        self,\n        cfg: Config,\n        batch_size: int = 8,\n        method: str = \'median\',\n        frequency_step_hz: float = 100.0,\n        minimum_inliers: int = 5,\n        cluster_absolute_hz: float = 500.0,\n        cluster_relative_ratio: float = 0.012,\n        mad_scale: float = 3.5,\n        mad_floor_hz: float = 200.0,\n        lost_reset_frames: int = 8,\n    ):\n        if batch_size < 1:\n            raise ValueError(\'fusion-frames必须大于0\')\n        if method not in (\'median\', \'mode\', \'trimmed_mean\'):\n            raise ValueError(\'fusion-method必须为median、mode或trimmed_mean\')\n        if frequency_step_hz <= 0:\n            raise ValueError(\'frequency-step-hz必须大于0\')\n        if not 1 <= minimum_inliers <= batch_size:\n            raise ValueError(\'minimum-inliers必须位于1～fusion-frames\')\n        if cluster_absolute_hz <= 0:\n            raise ValueError(\'cluster-absolute-hz必须大于0\')\n        if cluster_relative_ratio < 0:\n            raise ValueError(\'cluster-relative-ratio不能小于0\')\n        if mad_scale <= 0:\n            raise ValueError(\'mad-scale必须大于0\')\n        if mad_floor_hz < 0:\n            raise ValueError(\'mad-floor-hz不能小于0\')\n        if lost_reset_frames < 1:\n            raise ValueError(\'lost-reset-frames必须大于0\')\n\n        self.cfg = cfg\n        self.batch_size = int(batch_size)\n        self.method = method\n        self.frequency_step_hz = float(frequency_step_hz)\n        self.minimum_inliers = int(minimum_inliers)\n        self.cluster_absolute_hz = float(cluster_absolute_hz)\n        self.cluster_relative_ratio = float(cluster_relative_ratio)\n        self.mad_scale = float(mad_scale)\n        self.mad_floor_hz = float(mad_floor_hz)\n        self.lost_reset_frames = int(lost_reset_frames)\n\n        self.frequency_samples: list[float] = []\n        self.period_samples: list[float] = []\n        self.lost_count = 0\n        self.batch_index = 0\n\n        self.last_valid = False\n        self.last_frequency_hz = float(\'nan\')\n        self.last_period_div = float(\'nan\')\n        self.last_accepted_count = 0\n        self.last_rejected_count = 0\n\n    def clear_batch(self) -> None:\n        self.frequency_samples.clear()\n        self.period_samples.clear()\n\n    def clear(self) -> None:\n        """C键调用：清空当前8帧和已经保持的融合输出。"""\n        self.clear_batch()\n        self.lost_count = 0\n        self.last_valid = False\n        self.last_frequency_hz = float(\'nan\')\n        self.last_period_div = float(\'nan\')\n        self.last_accepted_count = 0\n        self.last_rejected_count = 0\n\n    def _quantize_frequency(self, frequency_hz: float) -> float:\n        """四舍五入到指定频率步进；默认100 Hz。"""\n        step = self.frequency_step_hz\n        return float(math.floor(frequency_hz / step + 0.5) * step)\n\n    def _dominant_cluster(self, frequencies: np.ndarray) -> np.ndarray:\n        """\n        找到包含样本数最多的局部频率簇。\n\n        对80 kHz误识别为40/20 kHz等情况，错误值与正确值距离很大，\n        会在这一阶段被排除。容差由固定Hz和相对比例共同决定。\n        """\n        count = len(frequencies)\n        if count == 0:\n            return np.zeros(0, dtype=bool)\n\n        global_median = float(np.median(frequencies))\n        best_mask = np.ones(count, dtype=bool)\n        best_key = (-1, float(\'inf\'), float(\'inf\'))\n\n        for center in frequencies:\n            tolerance = max(\n                self.cluster_absolute_hz,\n                self.cluster_relative_ratio * max(abs(float(center)), 1.0),\n            )\n            mask = np.abs(frequencies - center) <= tolerance\n            selected = frequencies[mask]\n            selected_count = int(np.count_nonzero(mask))\n            dispersion = (\n                float(np.median(np.abs(selected - np.median(selected))))\n                if selected_count > 0\n                else float(\'inf\')\n            )\n            center_distance = abs(float(center) - global_median)\n\n            # 样本数越多越优；样本数相同时，离散度越小越优。\n            key = (-selected_count, dispersion, center_distance)\n            if key < best_key:\n                best_key = key\n                best_mask = mask\n\n        return best_mask\n\n    def _remove_outliers(self, frequencies: np.ndarray) -> np.ndarray:\n        """主簇筛选 + MAD二次筛选，返回最终内点布尔数组。"""\n        cluster_mask = self._dominant_cluster(frequencies)\n        if int(np.count_nonzero(cluster_mask)) < self.minimum_inliers:\n            return np.zeros(len(frequencies), dtype=bool)\n\n        clustered = frequencies[cluster_mask]\n        center = float(np.median(clustered))\n        absolute_deviation = np.abs(clustered - center)\n        mad = float(np.median(absolute_deviation))\n        robust_sigma = 1.4826 * mad\n        threshold = max(\n            self.mad_floor_hz,\n            2.0 * self.frequency_step_hz,\n            self.mad_scale * robust_sigma,\n        )\n\n        clustered_inliers = absolute_deviation <= threshold\n        final_mask = np.zeros(len(frequencies), dtype=bool)\n        final_mask[np.flatnonzero(cluster_mask)] = clustered_inliers\n\n        if int(np.count_nonzero(final_mask)) < self.minimum_inliers:\n            return np.zeros(len(frequencies), dtype=bool)\n        return final_mask\n\n    def _fuse_frequency(self, inlier_frequencies: np.ndarray) -> float:\n        if self.method == \'median\':\n            fused = float(np.median(inlier_frequencies))\n\n        elif self.method == \'mode\':\n            quantized = np.asarray(\n                [self._quantize_frequency(value) for value in inlier_frequencies],\n                dtype=np.float64,\n            )\n            values, counts = np.unique(quantized, return_counts=True)\n            maximum_count = int(np.max(counts))\n            candidates = values[counts == maximum_count]\n\n            # 众数并列时，选最接近原始中位数的100 Hz档位。\n            reference = float(np.median(inlier_frequencies))\n            fused = float(candidates[int(np.argmin(np.abs(candidates - reference)))])\n\n        else:  # trimmed_mean\n            ordered = np.sort(inlier_frequencies)\n            if len(ordered) >= 5:\n                ordered = ordered[1:-1]\n            fused = float(np.mean(ordered))\n\n        return self._quantize_frequency(fused)\n\n    def _make_output(\n        self,\n        *,\n        updated: bool = False,\n        batch_rejected: bool = False,\n    ) -> StableOutput:\n        return StableOutput(\n            valid=self.last_valid,\n            frequency_hz=self.last_frequency_hz,\n            period_div=self.last_period_div,\n            sample_count=len(self.frequency_samples),\n            batch_size=self.batch_size,\n            accepted_count=self.last_accepted_count,\n            rejected_count=self.last_rejected_count,\n            updated=updated,\n            batch_rejected=batch_rejected,\n            batch_index=self.batch_index,\n            method=self.method,\n        )\n\n    def update(self, result: Result) -> StableOutput:\n        if not result.ok or not np.isfinite(result.frequency_hz):\n            self.lost_count += 1\n            if self.lost_count >= self.lost_reset_frames:\n                # 连续识别失败时丢弃未完成批次，避免旧帧与新场景混合。\n                self.clear_batch()\n            return self._make_output()\n\n        self.lost_count = 0\n        self.frequency_samples.append(float(result.frequency_hz))\n        self.period_samples.append(float(result.period_div))\n\n        if len(self.frequency_samples) < self.batch_size:\n            return self._make_output()\n\n        frequencies = np.asarray(self.frequency_samples, dtype=np.float64)\n        inlier_mask = self._remove_outliers(frequencies)\n        accepted_count = int(np.count_nonzero(inlier_mask))\n        rejected_count = self.batch_size - accepted_count\n        self.batch_index += 1\n\n        if accepted_count < self.minimum_inliers:\n            # 当前批次不可靠：保持上一组输出，但不更新最终值。\n            self.last_accepted_count = accepted_count\n            self.last_rejected_count = rejected_count\n            self.clear_batch()\n            return self._make_output(batch_rejected=True)\n\n        fused_frequency = self._fuse_frequency(frequencies[inlier_mask])\n        fused_frequency = float(np.clip(\n            fused_frequency,\n            self.cfg.frequency_min_hz,\n            self.cfg.frequency_max_hz,\n        ))\n        fused_period = float(self.cfg.period_from_frequency(fused_frequency))\n\n        self.last_valid = True\n        self.last_frequency_hz = fused_frequency\n        self.last_period_div = fused_period\n        self.last_accepted_count = accepted_count\n        self.last_rejected_count = rejected_count\n        self.clear_batch()\n\n        return self._make_output(updated=True)\n\n\ndef frame_to_gray(frame: np.ndarray) -> np.ndarray:\n    if frame is None:\n        raise ValueError(\'摄像头帧为空\')\n    if frame.ndim == 2:\n        gray = frame\n    elif frame.shape[2] == 4:\n        gray = cv2.cvtColor(frame, cv2.COLOR_BGRA2GRAY)\n    else:\n        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)\n\n    if gray.dtype == np.uint8:\n        return gray\n\n    array = gray.astype(np.float32)\n    low, high = np.percentile(array, [0.2, 99.8])\n    if high <= low:\n        return np.zeros(array.shape, dtype=np.uint8)\n    return np.rint(\n        np.clip((array - low) / (high - low), 0.0, 1.0) * 255.0\n    ).astype(np.uint8)\n\n\ndef open_capture(args: argparse.Namespace) -> cv2.VideoCapture:\n    if args.gst_pipeline:\n        capture = cv2.VideoCapture(args.gst_pipeline, cv2.CAP_GSTREAMER)\n    elif args.video:\n        capture = cv2.VideoCapture(str(args.video))\n    else:\n        backend = cv2.CAP_V4L2 if hasattr(cv2, \'CAP_V4L2\') else cv2.CAP_ANY\n        capture = cv2.VideoCapture(args.camera, backend)\n\n        if args.fourcc:\n            fourcc = args.fourcc.upper()\n            if len(fourcc) != 4:\n                raise ValueError(\'--fourcc必须为4个字符，例如MJPG或YUYV\')\n            capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*fourcc))\n\n        capture.set(cv2.CAP_PROP_FRAME_WIDTH, float(args.width))\n        capture.set(cv2.CAP_PROP_FRAME_HEIGHT, float(args.height))\n        capture.set(cv2.CAP_PROP_FPS, float(args.fps))\n        capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)\n\n    if not capture.isOpened():\n        raise RuntimeError(\'无法打开视频源\')\n    return capture\n\n\nclass LatestFrameReader:\n    """后台持续读帧，处理端始终获取最新帧，避免Jetson积累旧帧延迟。"""\n\n    def __init__(self, capture: cv2.VideoCapture):\n        self.capture = capture\n        self.lock = threading.Lock()\n        self.frame: Optional[np.ndarray] = None\n        self.sequence = 0\n        self.running = True\n        self.failed = False\n        self.thread = threading.Thread(target=self._run, daemon=True)\n        self.thread.start()\n\n    def _run(self) -> None:\n        while self.running:\n            ok, frame = self.capture.read()\n            if not ok:\n                self.failed = True\n                self.running = False\n                break\n            with self.lock:\n                self.frame = frame\n                self.sequence += 1\n\n    def read(self, last_sequence: int, timeout_s: float = 1.0):\n        deadline = time.monotonic() + timeout_s\n        while self.running or not self.failed:\n            with self.lock:\n                if self.frame is not None and self.sequence != last_sequence:\n                    return True, self.frame.copy(), self.sequence\n            if time.monotonic() >= deadline:\n                return False, None, last_sequence\n            time.sleep(0.001)\n        return False, None, last_sequence\n\n    def close(self) -> None:\n        self.running = False\n        if self.thread.is_alive():\n            self.thread.join(timeout=1.0)\n        self.capture.release()\n\n\ndef annotate_live(\n    result: Result,\n    stable: StableOutput,\n    cfg: Config,\n    frame_index: int,\n    loop_fps: float,\n) -> np.ndarray:\n    shown = annotate(result, cfg, f\'frame={frame_index}\')\n\n    if stable.valid:\n        update_tag = \' NEW\' if stable.updated else \'\'\n        stable_text = (\n            f\'FUSED{update_tag}={stable.frequency_hz / 1000.0:.1f} kHz, \'\n            f\'next={stable.sample_count}/{stable.batch_size}, \'\n            f\'inlier={stable.accepted_count}, reject={stable.rejected_count}\'\n        )\n        color = (0, 255, 0)\n    else:\n        stable_text = (\n            f\'FUSED=COLLECTING {stable.sample_count}/{stable.batch_size}\'\n        )\n        color = (0, 255, 255)\n\n    if stable.batch_rejected:\n        stable_text = (\n            f\'BATCH REJECTED: inlier={stable.accepted_count}/\'\n            f\'{stable.batch_size}; keep previous output\'\n        )\n        color = (0, 165, 255)\n\n    cv2.rectangle(\n        shown,\n        (0, shown.shape[0] - 58),\n        (shown.shape[1] - 1, shown.shape[0] - 1),\n        (0, 0, 0),\n        -1,\n    )\n    cv2.putText(\n        shown,\n        stable_text,\n        (12, shown.shape[0] - 32),\n        cv2.FONT_HERSHEY_SIMPLEX,\n        0.58,\n        color,\n        2,\n        cv2.LINE_AA,\n    )\n    cv2.putText(\n        shown,\n        f\'loop={loop_fps:.1f} FPS | Q/Esc quit | C clear fusion\',\n        (12, shown.shape[0] - 9),\n        cv2.FONT_HERSHEY_SIMPLEX,\n        0.48,\n        (255, 255, 255),\n        1,\n        cv2.LINE_AA,\n    )\n    return shown\n\n\ndef load_frequency_model(\n    path: Optional[Path],\n    fallback_k_hz_div: float,\n) -> tuple[str, float, float]:\n    """\n    读取frequency_calibration.json。\n\n    返回：\n        model\n        a_hz_div\n        b_hz\n    """\n    if path is None:\n        return (\n            \'scalar\',\n            float(fallback_k_hz_div),\n            0.0,\n        )\n\n    if not path.exists():\n        raise FileNotFoundError(\n            f\'找不到频率标定文件：{path}\'\n        )\n\n    data = json.loads(\n        path.read_text(\n            encoding=\'utf-8\'\n        )\n    )\n\n    model = str(\n        data.get(\'model\', \'scalar\')\n    )\n    a_hz_div = float(\n        data.get(\n            \'a_hz_div\',\n            data.get(\n                \'k_hz_div\',\n                fallback_k_hz_div,\n            ),\n        )\n    )\n    b_hz = float(\n        data.get(\'b_hz\', 0.0)\n    )\n\n    if a_hz_div <= 0:\n        raise ValueError(\n            \'频率标定a_hz_div必须大于0\'\n        )\n\n    if model not in (\n        \'scalar\',\n        \'reciprocal_affine\',\n    ):\n        raise ValueError(\n            f\'不支持的频率标定模型：{model}\'\n        )\n\n    if model == \'scalar\':\n        b_hz = 0.0\n\n    return model, a_hz_div, b_hz\n\n\ndef main() -> None:\n    parser = argparse.ArgumentParser(\n        description=\'Jetson 8×8九点标定右侧全高度八帧融合测频V6\'\n    )\n\n    source = parser.add_mutually_exclusive_group()\n    source.add_argument(\'--camera\', type=int, default=0, help=\'V4L2摄像头编号\')\n    source.add_argument(\'--video\', type=Path, help=\'本地视频文件，用于离线验证\')\n    source.add_argument(\'--gst-pipeline\', default=\'\', help=\'Jetson CSI/GStreamer pipeline\')\n\n    parser.add_argument(\'--width\', type=int, default=2740)\n    parser.add_argument(\'--height\', type=int, default=1824)\n    parser.add_argument(\'--fps\', type=float, default=30.0)\n    parser.add_argument(\'--fourcc\', default=\'MJPG\', help=\'USB摄像头格式；空字符串表示不设置\')\n    parser.add_argument(\'--calibration\', type=Path, default=Path(\'scope_8x8_nine_point_calibration.json\'))\n    parser.add_argument(\'--recalibrate\', action=\'store_true\', help=\'使用启动后的首帧重新九点标定\')\n\n    parser.add_argument(\'--pixels-per-div\', type=int, default=160)\n    parser.add_argument(\'--frequency-min\', type=float, default=20000.0)\n    parser.add_argument(\'--frequency-max\', type=float, default=100000.0)\n    parser.add_argument(\'--k-cal\', type=float, default=80700.4)\n    parser.add_argument(\n        \'--frequency-calibration\',\n        type=Path,\n        default=None,\n        help=(\n            \'frequency_calibration.json；\'\n            \'提供后覆盖--k-cal并支持f=a/p+b\'\n        ),\n    )\n    parser.add_argument(\n        \'--disable-boundary-rescue\',\n        action=\'store_true\',\n        help=(\n            \'采集标定日志时使用：保留100/20 kHz\'\n            \'附近的原始周期，不钳位到边界\'\n        ),\n    )\n    parser.add_argument(\'--right-tip-min-x\', type=float, default=7.2)\n\n    # 多帧融合：默认8个有效测量组成一组。\n    parser.add_argument(\n        \'--fusion-frames\',\n        type=int,\n        default=8,\n        help=\'每组融合使用的有效帧数，默认8\',\n    )\n    parser.add_argument(\n        \'--fusion-method\',\n        choices=(\'median\', \'mode\', \'trimmed_mean\'),\n        default=\'median\',\n        help=\'融合方法：中值/众数/截尾均值，默认median\',\n    )\n    parser.add_argument(\n        \'--frequency-step-hz\',\n        type=float,\n        default=100.0,\n        help=\'最终输出频率步进，默认100 Hz\',\n    )\n    parser.add_argument(\n        \'--minimum-inliers\',\n        type=int,\n        default=5,\n        help=\'8帧中至少保留多少个内点才输出，默认5\',\n    )\n    parser.add_argument(\n        \'--cluster-absolute-hz\',\n        type=float,\n        default=500.0,\n        help=\'主频率簇最小固定容差，默认500 Hz\',\n    )\n    parser.add_argument(\n        \'--cluster-relative-ratio\',\n        type=float,\n        default=0.012,\n        help=\'主频率簇相对容差，默认1.2%%\',\n    )\n    parser.add_argument(\n        \'--mad-scale\',\n        type=float,\n        default=3.5,\n        help=\'MAD异常值阈值倍数，默认3.5\',\n    )\n    parser.add_argument(\n        \'--mad-floor-hz\',\n        type=float,\n        default=200.0,\n        help=\'MAD异常值阈值下限，默认200 Hz\',\n    )\n    parser.add_argument(\n        \'--lost-reset-frames\',\n        type=int,\n        default=8,\n        help=\'连续失败多少帧后清空未完成的8帧批次\',\n    )\n\n    parser.add_argument(\'--display\', action=\'store_true\', help=\'显示校正图和识别点；无桌面环境不要开启\')\n    parser.add_argument(\'--display-scale\', type=float, default=0.72)\n    parser.add_argument(\'--print-every\', type=int, default=1)\n    parser.add_argument(\'--warmup-frames\', type=int, default=8)\n    parser.add_argument(\'--max-frames\', type=int, default=0)\n    parser.add_argument(\'--direct-read\', action=\'store_true\', help=\'不使用最新帧后台线程\')\n    args = parser.parse_args()\n\n    if args.frequency_max <= args.frequency_min:\n        raise SystemExit(\'frequency-max必须大于frequency-min\')\n    if args.display_scale <= 0:\n        raise SystemExit(\'display-scale必须大于0\')\n\n    (\n        frequency_model,\n        calibrated_a_hz_div,\n        calibrated_b_hz,\n    ) = load_frequency_model(\n        args.frequency_calibration,\n        args.k_cal,\n    )\n\n    cfg = Config(\n        pixels_per_div=args.pixels_per_div,\n        frequency_min_hz=args.frequency_min,\n        frequency_max_hz=args.frequency_max,\n        calibration_model=frequency_model,\n        k_hz_div=calibrated_a_hz_div,\n        frequency_offset_hz=calibrated_b_hz,\n        enable_boundary_rescue=(\n            not args.disable_boundary_rescue\n        ),\n        right_tip_min_x_div=args.right_tip_min_x,\n    )\n\n    capture = open_capture(args)\n\n    first_frame = None\n    for _ in range(max(1, args.warmup_frames)):\n        ok, frame = capture.read()\n        if ok:\n            first_frame = frame\n    if first_frame is None:\n        capture.release()\n        raise RuntimeError(\'无法读取启动帧\')\n\n    first_gray = frame_to_gray(first_frame)\n\n    if args.recalibrate or not args.calibration.exists():\n        if not args.display:\n            capture.release()\n            raise RuntimeError(\'重新标定需要--display和图形桌面环境\')\n        points = select_nine_points(first_gray)\n        save_calibration(args.calibration, points, first_gray.shape)\n        print(\'[Calibration] saved:\', args.calibration)\n    else:\n        points = load_calibration(args.calibration, first_gray.shape)\n        print(\'[Calibration] loaded:\', args.calibration)\n\n    rectifier = Rectifier(points, cfg)\n    smoother = MultiFrameFusion(\n        cfg=cfg,\n        batch_size=args.fusion_frames,\n        method=args.fusion_method,\n        frequency_step_hz=args.frequency_step_hz,\n        minimum_inliers=args.minimum_inliers,\n        cluster_absolute_hz=args.cluster_absolute_hz,\n        cluster_relative_ratio=args.cluster_relative_ratio,\n        mad_scale=args.mad_scale,\n        mad_floor_hz=args.mad_floor_hz,\n        lost_reset_frames=args.lost_reset_frames,\n    )\n\n    print(\'[VERSION]\', VERSION)\n    print(f\'[Camera] requested={args.width}x{args.height}@{args.fps:.1f}\')\n    print(\n        f\'[Camera] actual={int(capture.get(cv2.CAP_PROP_FRAME_WIDTH))}x\'\n        f\'{int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT))}@\'\n        f\'{capture.get(cv2.CAP_PROP_FPS):.2f}\'\n    )\n    print(\n        f\'[Frequency model] {cfg.calibration_model}: \'\n        f\'f={cfg.k_hz_div:.9f}/p\'\n        f\'{cfg.frequency_offset_hz:+.9f} Hz\'\n    )\n    print(\n        f\'[Boundary rescue] \'\n        f\'{cfg.enable_boundary_rescue}\'\n    )\n    print(f\'[Range] {cfg.frequency_min_hz / 1000:.1f}～{cfg.frequency_max_hz / 1000:.1f} kHz\')\n    print(f\'[Rectified] {cfg.width}×{cfg.height}\')\n    print(\n        f\'[Fusion] {args.fusion_frames}个有效帧/组 -> 主簇剔除异常 -> \'\n        f\'MAD剔除 -> {args.fusion_method}融合 -> \'\n        f\'{args.frequency_step_hz:.0f} Hz量化\'\n    )\n    print(\'[Control] Ctrl+C退出；显示模式下Q/Esc退出、C清空融合历史\')\n\n    latest_reader: Optional[LatestFrameReader] = None\n    if not args.direct_read and args.video is None:\n        latest_reader = LatestFrameReader(capture)\n\n    frame_index = 0\n    last_sequence = -1\n    fps_times = deque(maxlen=30)\n\n    try:\n        while True:\n            loop_start = time.perf_counter()\n\n            if latest_reader is not None:\n                ok, frame, last_sequence = latest_reader.read(last_sequence)\n            else:\n                ok, frame = capture.read()\n\n            if not ok or frame is None:\n                print(\'[Video] 读取失败或视频结束\')\n                break\n\n            gray = frame_to_gray(frame)\n\n            # 摄像头分辨率在运行中变化时，按标定JSON比例重建映射。\n            if gray.shape != first_gray.shape:\n                first_gray = gray\n                points = load_calibration(args.calibration, gray.shape)\n                rectifier = Rectifier(points, cfg)\n                smoother.clear()\n                print(\'[Video] 分辨率改变，已重建九点映射:\', gray.shape[::-1])\n\n            result = measure(gray, rectifier, cfg, already_rectified=False)\n            stable = smoother.update(result)\n\n            now = time.perf_counter()\n            fps_times.append(now)\n            loop_fps = 0.0\n            if len(fps_times) >= 2:\n                loop_fps = (len(fps_times) - 1) / max(fps_times[-1] - fps_times[0], 1e-9)\n\n            if frame_index % max(1, args.print_every) == 0:\n                if result.ok:\n                    stable_text = (\n                        f\'{stable.frequency_hz / 1000.0:.1f}\'\n                        if stable.valid\n                        else \'collecting\'\n                    )\n                    detected_used = int(np.count_nonzero(result.fit.detected_flags))\n                    recovered_used = len(result.fit.used_y_px) - detected_used\n                    print(\n                        f\'[{frame_index:06d}] \'\n                        f\'raw={result.raw_frequency_hz / 1000.0:9.5f} kHz, \'\n                        f\'single={result.frequency_hz / 1000.0:9.5f} kHz, \'\n                        f\'fused={stable_text} kHz, \'\n                        f\'collect={stable.sample_count}/{stable.batch_size}, \'\n                        f\'period_raw={result.measured_period_div:.7f} div, \'\n                        f\'period={result.period_div:.7f} div, \'\n                        f\'tips={len(result.fit.used_y_px)}\'\n                        f\'({detected_used}+{recovered_used}), \'\n                        f\'RMS={result.rms_div:.5f}, \'\n                        f\'mode={result.detection.mode}/{result.fit.reason}, \'\n                        f\'alg={result.elapsed_ms:.2f} ms, \'\n                        f\'loop={loop_fps:.1f} FPS\'\n                    )\n\n                else:\n                    print(\n                        f\'[{frame_index:06d}] FAILED \'\n                        f\'reason={result.fit.reason}, \'\n                        f\'seed={len(result.detection.y_px)}, \'\n                        f\'mode={result.detection.mode}, \'\n                        f\'alg={result.elapsed_ms:.2f} ms, loop={loop_fps:.1f} FPS\'\n                    )\n\n            # 最终融合结果不受--print-every限制，保证每完成一组都输出。\n            if stable.updated:\n                print(\n                    f\'[FUSION {stable.batch_index:04d}] \'\n                    f\'FINAL={stable.frequency_hz:.0f} Hz \'\n                    f\'({stable.frequency_hz / 1000.0:.1f} kHz), \'\n                    f\'method={stable.method}, \'\n                    f\'inlier={stable.accepted_count}/{stable.batch_size}, \'\n                    f\'rejected={stable.rejected_count}, \'\n                    f\'step={args.frequency_step_hz:.0f} Hz\'\n                )\n            elif stable.batch_rejected:\n                print(\n                    f\'[FUSION {stable.batch_index:04d}] REJECTED: \'\n                    f\'inlier={stable.accepted_count}/{stable.batch_size}, \'\n                    f\'需要至少{args.minimum_inliers}个内点\'\n                )\n\n            if args.display:\n                shown = annotate_live(result, stable, cfg, frame_index, loop_fps)\n                if args.display_scale != 1.0:\n                    shown = cv2.resize(\n                        shown,\n                        None,\n                        fx=args.display_scale,\n                        fy=args.display_scale,\n                        interpolation=cv2.INTER_AREA,\n                    )\n                cv2.imshow(\'Jetson scope frequency V6 8-frame fusion\', shown)\n                key = cv2.waitKey(1) & 0xFF\n                if key in (27, ord(\'q\'), ord(\'Q\')):\n                    break\n                if key in (ord(\'c\'), ord(\'C\')):\n                    smoother.clear()\n                    print(\'[Control] fusion history cleared\')\n\n            frame_index += 1\n            if args.max_frames > 0 and frame_index >= args.max_frames:\n                break\n\n            _ = loop_start  # 方便后续插入额外耗时统计。\n\n    except KeyboardInterrupt:\n        print(\'\\n[Control] Ctrl+C\')\n    finally:\n        if latest_reader is not None:\n            latest_reader.close()\n        else:\n            capture.release()\n        if args.display:\n            cv2.destroyAllWindows()\n\n\nif __name__ == \'__main__\':\n    main()\n'

SWITCH_CORE_SOURCE = '#!/usr/bin/env python3\n# -*- coding: utf-8 -*-\n"""\nJetson四档快速切换判断（只判断，不计算频率）\n\n档位：\n    0 = 0.1 ms\n    1 = 0.4 ms\n    2 = 1.0 ms\n    3 = 4.0 ms\n\n只有ready=True时才使用should_switch：\n    True  -> 切到next_range\n    False -> 保持当前档\n\n依赖：\n    numpy\n    opencv-python\n\n标定：\n    复用已有scope_8x8_nine_point_calibration.json\n"""\n\nfrom __future__ import annotations\n\nimport argparse\nimport json\nimport time\nfrom collections import deque\nfrom dataclasses import dataclass\nfrom pathlib import Path\nfrom typing import Optional\n\nimport cv2\nimport numpy as np\n\n\nVERSION = "JETSON-4RANGE-JUDGE-COMPACT-V1.1"\n\n\n@dataclass(frozen=True)\nclass Config:\n    pixels_per_div: int = 96\n\n    # 右半局部坐标0～4 div；3.05等价于完整网格x=7.05 div。\n    right_tip_min_x_div: float = 3.05\n\n    # 二值化和轻量去网格。\n    otsu_offset: float = 12.0\n    threshold_min: float = 105.0\n    threshold_max: float = 225.0\n    horizontal_close_div: float = 0.025\n    horizontal_open_div: float = 0.055\n    persistent_row_ratio: float = 0.56\n    persistent_col_ratio: float = 0.56\n    persistent_expand_px: int = 2\n\n    # 右侧包络折返点检测。\n    envelope_sigma_div: float = 0.040\n    peak_window_div: float = 0.24\n    shoulder_inner_div: float = 0.14\n    shoulder_outer_div: float = 0.36\n    minimum_shoulder_drop_div: float = 0.045\n    minimum_total_drop_div: float = 0.30\n    minimum_tip_distance_div: float = 0.42\n\n    # 周期格点拟合。\n    period_min_div: float = 0.55\n    period_max_div: float = 4.60\n    period_match_tolerance_div: float = 0.11\n    period_rms_limit_div: float = 0.09\n    minimum_period_points: int = 3\n    maximum_period_candidates: int = 24\n\n    # 只用于确认画面中有波形。\n    minimum_white_ratio: float = 0.0020\n    minimum_valid_row_ratio: float = 0.25\n\n    # 档0/1/2切到下一档的周期阈值。\n    switch_period_div: tuple[float, float, float] = (2.95, 1.90, 2.55)\n    keep_hysteresis_ratio: float = 0.90\n\n    # 5帧中至少4帧一致。\n    decision_window: int = 5\n    required_votes: int = 4\n\n    @property\n    def roi_width(self) -> int:\n        return 4 * self.pixels_per_div + 1\n\n    @property\n    def roi_height(self) -> int:\n        return 8 * self.pixels_per_div + 1\n\n\n@dataclass\nclass Evidence:\n    waveform_present: bool\n    tips: np.ndarray\n    period_valid: bool\n    period_div: float\n    rms_div: float\n    occupancy: float\n    mode: str\n    elapsed_ms: float\n\n\n@dataclass\nclass Decision:\n    ready: bool\n    should_switch: bool\n    current_range: int\n    next_range: int\n    reason: str\n    frame_state: str\n    switch_votes: int\n    keep_votes: int\n    tip_count: int\n    period_div: float\n    elapsed_ms: float\n\n\ndef parse_thresholds(text: str) -> tuple[float, float, float]:\n    values = [float(value.strip()) for value in text.split(",")]\n\n    if len(values) != 3:\n        raise argparse.ArgumentTypeError(\n            "格式应为三个逗号分隔的格数，例如2.95,1.90,2.55"\n        )\n\n    return values[0], values[1], values[2]\n\n\ndef to_gray_u8(frame: np.ndarray) -> np.ndarray:\n    if frame.ndim == 2:\n        gray = frame\n    elif frame.shape[2] == 4:\n        gray = cv2.cvtColor(frame, cv2.COLOR_BGRA2GRAY)\n    else:\n        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)\n\n    if gray.dtype == np.uint8:\n        return gray\n\n    array = gray.astype(np.float32)\n    low, high = np.percentile(array, [0.2, 99.8])\n\n    if high <= low:\n        return np.zeros(array.shape, dtype=np.uint8)\n\n    return np.rint(\n        np.clip((array - low) / (high - low), 0.0, 1.0) * 255.0\n    ).astype(np.uint8)\n\n\ndef load_points(path: Path, image_shape: tuple[int, int]) -> np.ndarray:\n    data = json.loads(path.read_text(encoding="utf-8"))\n    points = np.asarray(data["points_3x3"], dtype=np.float32)\n\n    if points.shape != (3, 3, 2):\n        raise ValueError("points_3x3必须是3×3×2")\n\n    points[:, :, 0] *= image_shape[1] / float(data["image_width"])\n    points[:, :, 1] *= image_shape[0] / float(data["image_height"])\n\n    if np.any(np.diff(points[:, :, 0], axis=1) <= 0):\n        raise ValueError("九点左右顺序错误")\n\n    if np.any(np.diff(points[:, :, 1], axis=0) <= 0):\n        raise ValueError("九点上下顺序错误")\n\n    return points\n\n\nclass RightRectifier:\n    """只映射8×8网格右半部分，降低remap开销。"""\n\n    def __init__(self, points: np.ndarray, cfg: Config):\n        self.cfg = cfg\n        self.map_x, self.map_y = self._build(points)\n\n    def _build(self, points: np.ndarray) -> tuple[np.ndarray, np.ndarray]:\n        width, height = self.cfg.roi_width, self.cfg.roi_height\n        half_height = 4 * self.cfg.pixels_per_div\n        map_x = np.zeros((height, width), np.float32)\n        map_y = np.zeros_like(map_x)\n\n        for row in range(2):\n            source = np.float32([\n                points[row, 1],\n                points[row, 2],\n                points[row + 1, 2],\n                points[row + 1, 1],\n            ])\n            y0 = row * half_height\n            y1 = (row + 1) * half_height if row == 0 else height - 1\n            destination = np.float32([\n                [0, y0],\n                [width - 1, y0],\n                [width - 1, y1],\n                [0, y1],\n            ])\n            inverse = cv2.getPerspectiveTransform(destination, source)\n            xx, yy = np.meshgrid(\n                np.arange(width, dtype=np.float32),\n                np.arange(y0, y1 + 1, dtype=np.float32),\n            )\n            denominator = inverse[2, 0] * xx + inverse[2, 1] * yy + inverse[2, 2]\n\n            map_x[y0:y1 + 1] = (\n                inverse[0, 0] * xx + inverse[0, 1] * yy + inverse[0, 2]\n            ) / denominator\n            map_y[y0:y1 + 1] = (\n                inverse[1, 0] * xx + inverse[1, 1] * yy + inverse[1, 2]\n            ) / denominator\n\n        return map_x, map_y\n\n    def __call__(self, gray: np.ndarray) -> np.ndarray:\n        return cv2.remap(\n            gray,\n            self.map_x,\n            self.map_y,\n            cv2.INTER_LINEAR,\n            borderMode=cv2.BORDER_REPLICATE,\n        )\n\n\ndef expand_flags(flags: np.ndarray, radius: int) -> np.ndarray:\n    if radius <= 0:\n        return flags\n\n    return cv2.dilate(\n        flags.astype(np.uint8).reshape(-1, 1),\n        np.ones((2 * radius + 1, 1), np.uint8),\n    ).reshape(-1).astype(bool)\n\n\ndef build_mask(\n    rectified: np.ndarray,\n    cfg: Config,\n    fallback: bool,\n) -> np.ndarray:\n    if fallback:\n        processed = cv2.createCLAHE(1.8, (8, 8)).apply(rectified)\n        offset, threshold_min, open_div = -6.0, 70.0, 0.038\n    else:\n        processed = rectified\n        offset = cfg.otsu_offset\n        threshold_min = cfg.threshold_min\n        open_div = cfg.horizontal_open_div\n\n    otsu, _ = cv2.threshold(\n        processed,\n        0,\n        255,\n        cv2.THRESH_BINARY + cv2.THRESH_OTSU,\n    )\n    threshold = float(np.clip(\n        otsu + offset,\n        threshold_min,\n        cfg.threshold_max,\n    ))\n    mask = (processed >= threshold).astype(np.uint8)\n    ppd = cfg.pixels_per_div\n\n    mask = cv2.morphologyEx(\n        mask,\n        cv2.MORPH_CLOSE,\n        np.ones((1, max(3, round(cfg.horizontal_close_div * ppd))), np.uint8),\n    )\n    mask = cv2.morphologyEx(\n        mask,\n        cv2.MORPH_OPEN,\n        np.ones((1, max(3, round(open_div * ppd))), np.uint8),\n    )\n\n    bad_rows = expand_flags(\n        np.mean(mask != 0, axis=1) >= cfg.persistent_row_ratio,\n        cfg.persistent_expand_px,\n    )\n    bad_cols = expand_flags(\n        np.mean(mask != 0, axis=0) >= cfg.persistent_col_ratio,\n        cfg.persistent_expand_px,\n    )\n\n    mask[bad_rows] = 0\n    mask[:, bad_cols] = 0\n    return mask\n\n\ndef interpolate_envelope(values: np.ndarray, valid: np.ndarray) -> Optional[np.ndarray]:\n    valid_indices = np.flatnonzero(valid)\n\n    if len(valid_indices) < 2:\n        return None\n\n    indices = np.arange(len(values), dtype=np.float32)\n\n    return np.interp(\n        indices,\n        valid_indices.astype(np.float32),\n        values[valid_indices].astype(np.float32),\n    ).astype(np.float32)\n\n\ndef true_groups(flags: np.ndarray) -> list[tuple[int, int]]:\n    groups: list[tuple[int, int]] = []\n    start: Optional[int] = None\n\n    for index, value in enumerate(flags):\n        if value and start is None:\n            start = index\n\n        if start is not None and (not value or index == len(flags) - 1):\n            groups.append((start, index if not value else index + 1))\n            start = None\n\n    return groups\n\n\ndef refine_peak(profile: np.ndarray, center: int, half_width: int) -> float:\n    left = max(0, center - half_width)\n    right = min(len(profile), center + half_width + 1)\n    x = np.arange(left, right, dtype=np.float64) - center\n    y = profile[left:right].astype(np.float64)\n\n    if len(x) < 5:\n        return float(center)\n\n    a, b, _ = np.polyfit(x, y, 2)\n\n    if a >= -1e-9:\n        return float(center)\n\n    offset = -b / (2.0 * a)\n\n    if abs(offset) > half_width:\n        return float(center)\n\n    return float(center + offset)\n\n\ndef detect_tips(mask: np.ndarray, cfg: Config) -> tuple[np.ndarray, bool]:\n    valid_rows = np.any(mask != 0, axis=1)\n    right_x = (mask.shape[1] - 1 - np.argmax(mask[:, ::-1], axis=1)).astype(np.float32)\n    right_x[~valid_rows] = np.nan\n\n    envelope = interpolate_envelope(right_x, valid_rows)\n    waveform_present = (\n        np.mean(mask != 0) >= cfg.minimum_white_ratio\n        and np.mean(valid_rows) >= cfg.minimum_valid_row_ratio\n    )\n\n    if envelope is None:\n        return np.empty(0, np.float64), waveform_present\n\n    ppd = cfg.pixels_per_div\n    smoothed = cv2.GaussianBlur(\n        envelope.reshape(-1, 1),\n        (1, 0),\n        sigmaX=0,\n        sigmaY=max(1.0, cfg.envelope_sigma_div * ppd),\n    ).reshape(-1)\n\n    peak_radius = max(3, round(cfg.peak_window_div * ppd))\n    local_maximum = cv2.dilate(\n        smoothed.reshape(-1, 1),\n        np.ones((2 * peak_radius + 1, 1), np.uint8),\n    ).reshape(-1)\n    peak_flags = np.isclose(\n        smoothed,\n        local_maximum,\n        atol=max(0.5, 0.012 * ppd),\n    )\n\n    inner = max(2, round(cfg.shoulder_inner_div * ppd))\n    outer = max(inner + 2, round(cfg.shoulder_outer_div * ppd))\n    candidates: list[tuple[float, float]] = []\n\n    for begin, end in true_groups(peak_flags):\n        if end <= begin:\n            continue\n\n        center = begin + int(np.argmax(smoothed[begin:end]))\n\n        if center - outer < 0 or center + outer >= len(smoothed):\n            continue\n\n        peak_x = float(smoothed[center])\n\n        if peak_x < cfg.right_tip_min_x_div * ppd:\n            continue\n\n        left_floor = float(np.min(smoothed[center - outer:center - inner + 1]))\n        right_floor = float(np.min(smoothed[center + inner:center + outer + 1]))\n        left_drop = (peak_x - left_floor) / ppd\n        right_drop = (peak_x - right_floor) / ppd\n\n        if (\n            min(left_drop, right_drop) < cfg.minimum_shoulder_drop_div\n            or left_drop + right_drop < cfg.minimum_total_drop_div\n        ):\n            continue\n\n        position = refine_peak(smoothed, center, max(3, round(0.10 * ppd)))\n        quality = 2.0 * min(left_drop, right_drop) + 0.35 * (left_drop + right_drop)\n        candidates.append((position, quality))\n\n    candidates.sort(key=lambda item: item[1], reverse=True)\n    selected: list[tuple[float, float]] = []\n    minimum_distance = cfg.minimum_tip_distance_div * ppd\n\n    for candidate in candidates:\n        if all(abs(candidate[0] - old[0]) >= minimum_distance for old in selected):\n            selected.append(candidate)\n\n    selected.sort(key=lambda item: item[0])\n\n    return (\n        np.asarray([item[0] for item in selected], np.float64),\n        waveform_present,\n    )\n\n\ndef fit_line(indices: np.ndarray, y: np.ndarray) -> tuple[float, float]:\n    count = float(len(indices))\n    sum_x = float(np.sum(indices))\n    sum_y = float(np.sum(y))\n    sum_xx = float(np.sum(indices * indices))\n    sum_xy = float(np.sum(indices * y))\n    denominator = count * sum_xx - sum_x * sum_x\n\n    if abs(denominator) < 1e-12:\n        return float("nan"), float("nan")\n\n    period = (count * sum_xy - sum_x * sum_y) / denominator\n    phase = (sum_y - period * sum_x) / count\n    return period, phase\n\n\ndef period_candidates(tips: np.ndarray, cfg: Config) -> list[float]:\n    ppd = cfg.pixels_per_div\n    minimum = cfg.period_min_div * ppd\n    maximum = cfg.period_max_div * ppd\n    candidates: list[float] = []\n\n    for first in range(len(tips) - 1):\n        for second in range(first + 1, len(tips)):\n            distance = float(tips[second] - tips[first])\n\n            for step in range(1, min(6, int(distance // minimum)) + 1):\n                candidate = distance / step\n\n                if minimum <= candidate <= maximum:\n                    candidates.append(candidate)\n\n    output: list[float] = []\n\n    # 大周期优先，避免低点数时先接受二倍频。\n    for candidate in sorted(candidates, reverse=True):\n        if all(abs(candidate - old) > 0.025 * ppd for old in output):\n            output.append(candidate)\n\n        if len(output) >= cfg.maximum_period_candidates:\n            break\n\n    return output\n\n\ndef consecutive(indices: np.ndarray) -> bool:\n    return (\n        len(indices) < 2\n        or np.all(np.diff(np.sort(indices.astype(np.int32))) == 1)\n    )\n\n\ndef estimate_period(tips: np.ndarray, cfg: Config) -> tuple[bool, float, float, float]:\n    if len(tips) < cfg.minimum_period_points:\n        return False, float("nan"), float("inf"), 0.0\n\n    ppd = cfg.pixels_per_div\n    tolerance = cfg.period_match_tolerance_div * ppd\n    best = None\n\n    for candidate in period_candidates(tips, cfg):\n        for anchor in (tips[0], tips[len(tips) // 2], tips[-1]):\n            indices = np.rint((tips - anchor) / candidate).astype(np.int32)\n            residual = np.abs(tips - (anchor + indices * candidate))\n            selected_y, selected_n = [], []\n\n            for value in np.unique(indices):\n                members = np.flatnonzero(indices == value)\n                member = members[np.argmin(residual[members])]\n\n                if residual[member] <= tolerance:\n                    selected_y.append(float(tips[member]))\n                    selected_n.append(float(value))\n\n            y = np.asarray(selected_y, np.float64)\n            n = np.asarray(selected_n, np.float64)\n\n            if len(y) < cfg.minimum_period_points or not consecutive(n):\n                continue\n\n            period, phase = fit_line(n, y)\n\n            if not np.isfinite(period):\n                continue\n\n            period_div = period / ppd\n\n            if not cfg.period_min_div <= period_div <= cfg.period_max_div:\n                continue\n\n            rms_div = float(np.sqrt(np.mean((y - (period * n + phase)) ** 2)) / ppd)\n\n            if rms_div > cfg.period_rms_limit_div:\n                continue\n\n            occupancy = min(1.0, len(y) / max(1.0, 8.0 / period_div + 1.0))\n            score = 120.0 * len(y) + 160.0 * occupancy + 8.0 * period_div - 80.0 * rms_div\n            result = (score, period_div, rms_div, occupancy)\n\n            if best is None or result[0] > best[0]:\n                best = result\n\n    if best is None:\n        return False, float("nan"), float("inf"), 0.0\n\n    return True, float(best[1]), float(best[2]), float(best[3])\n\n\nclass Analyzer:\n    def __init__(self, rectifier: RightRectifier, cfg: Config):\n        self.rectifier = rectifier\n        self.cfg = cfg\n\n    def analyze(self, gray: np.ndarray) -> Evidence:\n        start = time.perf_counter()\n        rectified = self.rectifier(gray)\n        mask = build_mask(rectified, self.cfg, fallback=False)\n        tips, present = detect_tips(mask, self.cfg)\n        mode = "strict"\n\n        # 正常帧不执行后备；仅无足够端点时额外尝试一次。\n        if present and len(tips) < 2:\n            fallback_mask = build_mask(rectified, self.cfg, fallback=True)\n            fallback_tips, fallback_present = detect_tips(fallback_mask, self.cfg)\n\n            if len(fallback_tips) > len(tips):\n                tips, present, mode = fallback_tips, fallback_present, "fallback"\n\n        valid, period, rms, occupancy = estimate_period(tips, self.cfg)\n\n        return Evidence(\n            waveform_present=present,\n            tips=tips,\n            period_valid=valid,\n            period_div=period,\n            rms_div=rms,\n            occupancy=occupancy,\n            mode=mode,\n            elapsed_ms=(time.perf_counter() - start) * 1000.0,\n        )\n\n\nclass RangeJudge:\n    def __init__(self, cfg: Config, range_index: int):\n        self.cfg = cfg\n        self.current_range = range_index\n        self.history: deque[str] = deque(maxlen=cfg.decision_window)\n\n    def set_range(self, range_index: int) -> None:\n        if range_index not in (0, 1, 2, 3):\n            raise ValueError("档位必须是0、1、2、3")\n\n        if range_index != self.current_range:\n            self.current_range = range_index\n            self.history.clear()\n\n    def classify(self, evidence: Evidence) -> tuple[str, str]:\n        if not evidence.waveform_present:\n            return "UNKNOWN", "NO_SIGNAL"\n\n        if self.current_range == 3:\n            return "KEEP", "LAST_RANGE"\n\n        threshold = self.cfg.switch_period_div[self.current_range]\n\n        if evidence.period_valid:\n            if evidence.period_div >= threshold:\n                return "SWITCH", "PERIOD_TOO_LONG"\n\n            if evidence.period_div <= threshold * self.cfg.keep_hysteresis_ratio:\n                return "KEEP", "PERIOD_SAFE"\n\n            return "UNKNOWN", "NEAR_BOUNDARY"\n\n        if len(evidence.tips) <= 2:\n            return "SWITCH", "TOO_FEW_TIPS"\n\n        return "UNKNOWN", "PERIOD_UNCERTAIN"\n\n    def update(self, evidence: Evidence) -> Decision:\n        frame_state, frame_reason = self.classify(evidence)\n        self.history.append(frame_state)\n\n        switch_votes = sum(state == "SWITCH" for state in self.history)\n        keep_votes = sum(state == "KEEP" for state in self.history)\n        ready = False\n        should_switch = False\n        reason = "COLLECTING_OR_UNCERTAIN"\n\n        if self.current_range == 3:\n            if keep_votes >= self.cfg.required_votes:\n                ready, reason = True, "LAST_RANGE_KEEP"\n        elif switch_votes >= self.cfg.required_votes:\n            ready, should_switch, reason = True, True, frame_reason\n        elif keep_votes >= self.cfg.required_votes:\n            ready, reason = True, frame_reason\n\n        return Decision(\n            ready=ready,\n            should_switch=should_switch,\n            current_range=self.current_range,\n            next_range=min(self.current_range + 1, 3),\n            reason=reason,\n            frame_state=frame_state,\n            switch_votes=switch_votes,\n            keep_votes=keep_votes,\n            tip_count=len(evidence.tips),\n            period_div=evidence.period_div,\n            elapsed_ms=evidence.elapsed_ms,\n        )\n\n\ndef open_camera(args) -> cv2.VideoCapture:\n    if args.gst_pipeline:\n        capture = cv2.VideoCapture(args.gst_pipeline, cv2.CAP_GSTREAMER)\n    else:\n        backend = cv2.CAP_V4L2 if hasattr(cv2, "CAP_V4L2") else cv2.CAP_ANY\n        capture = cv2.VideoCapture(args.camera, backend)\n        capture.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)\n        capture.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)\n        capture.set(cv2.CAP_PROP_FPS, args.fps)\n        capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)\n\n    if not capture.isOpened():\n        raise RuntimeError("无法打开摄像头")\n\n    return capture\n\n\ndef main() -> None:\n    parser = argparse.ArgumentParser(description="Jetson四档快速切换判断")\n    parser.add_argument("--camera", type=int, default=0)\n    parser.add_argument("--gst-pipeline", default="")\n    parser.add_argument("--width", type=int, default=2740)\n    parser.add_argument("--height", type=int, default=1824)\n    parser.add_argument("--fps", type=float, default=30.0)\n    parser.add_argument(\n        "--calibration",\n        type=Path,\n        default=Path("scope_8x8_nine_point_calibration.json"),\n    )\n    parser.add_argument("--range-index", type=int, choices=[0, 1, 2, 3], default=0)\n    parser.add_argument("--pixels-per-div", type=int, default=96)\n    parser.add_argument(\n        "--switch-thresholds",\n        type=parse_thresholds,\n        default=(2.95, 1.90, 2.55),\n    )\n    parser.add_argument("--right-tip-min-x", type=float, default=3.05)\n    parser.add_argument("--warmup-frames", type=int, default=8)\n    parser.add_argument("--print-every", type=int, default=1)\n    parser.add_argument("--max-frames", type=int, default=0)\n    args = parser.parse_args()\n\n    if not args.calibration.exists():\n        raise SystemExit(f"找不到标定文件：{args.calibration}")\n\n    cfg = Config(\n        pixels_per_div=args.pixels_per_div,\n        right_tip_min_x_div=args.right_tip_min_x,\n        switch_period_div=args.switch_thresholds,\n    )\n    capture = open_camera(args)\n    first = None\n\n    for _ in range(max(1, args.warmup_frames)):\n        ok, frame = capture.read()\n\n        if ok:\n            first = frame\n\n    if first is None:\n        capture.release()\n        raise RuntimeError("无法读取第一帧")\n\n    gray = to_gray_u8(first)\n    points = load_points(args.calibration, gray.shape)\n    analyzer = Analyzer(RightRectifier(points, cfg), cfg)\n    judge = RangeJudge(cfg, args.range_index)\n\n    print(f"[VERSION] {VERSION}")\n    print("[Meaning] 只有ready=True时才使用should_switch")\n    print(f"[Range] {args.range_index}, thresholds={cfg.switch_period_div}")\n    print(f"[Right ROI] {cfg.roi_width}x{cfg.roi_height}")\n\n    frame_index = 0\n\n    try:\n        while True:\n            ok, frame = capture.read()\n\n            if not ok:\n                break\n\n            evidence = analyzer.analyze(to_gray_u8(frame))\n            decision = judge.update(evidence)\n\n            if frame_index % max(1, args.print_every) == 0:\n                period = (\n                    f"{decision.period_div:.4f}"\n                    if np.isfinite(decision.period_div)\n                    else "invalid"\n                )\n                print(\n                    f"[{frame_index:06d}] "\n                    f"range={decision.current_range}, "\n                    f"ready={decision.ready}, "\n                    f"should_switch={decision.should_switch}, "\n                    f"next={decision.next_range}, "\n                    f"tips={decision.tip_count}, "\n                    f"period={period} div, "\n                    f"votes={decision.switch_votes}/{decision.keep_votes}, "\n                    f"frame={decision.frame_state}, "\n                    f"reason={decision.reason}, "\n                    f"mode={evidence.mode}, "\n                    f"time={decision.elapsed_ms:.2f} ms"\n                )\n\n            frame_index += 1\n\n            if args.max_frames > 0 and frame_index >= args.max_frames:\n                break\n    finally:\n        capture.release()\n\n\nif __name__ == "__main__":\n    main()\n'

UART_PROTOCOL_SOURCE = '#!/usr/bin/env python3\n# -*- coding: utf-8 -*-\n"""\nFPGA <-> Jetson 简化UART协议。\n\n沿用现有协议的基本帧格式：\n    A5 5A VER CMD SEQ LEN PAYLOAD CRC8\n\nCRC-8/ATM：\n    poly=0x07, init=0x00, refin=false, refout=false, xorout=0x00\n\n核心握手：\n    FPGA  -> START\n    Jetson-> SET_GEAR\n    FPGA  -> GEAR_CHANGED\n    Jetson-> FREQUENCY_RESULT\n    FPGA  -> ACK\n\n最终频率载荷只有一个u16：\n    frequency_code = round(frequency_hz / 100)\n"""\n\nfrom __future__ import annotations\n\nimport collections\nimport queue\nimport threading\nimport time\nfrom dataclasses import dataclass\nfrom typing import Optional\n\ntry:\n    import serial\nexcept ImportError:\n    serial = None\n\n\nSOF = b"\\xA5\\x5A"\nPROTOCOL_VERSION = 0x01\nMAX_PAYLOAD = 8\n\n# Jetson -> FPGA\nCMD_SET_GEAR = 0x18\nCMD_FREQUENCY_RESULT = 0x19\nCMD_MEASURE_FAILED = 0x1A\nCMD_SCAN_CORRECT = 0x1B\nCMD_SCAN_WRONG = 0x1C\n\n# FPGA -> Jetson\nCMD_ACK = 0x80\nCMD_START = 0x81\nCMD_ABORTED = 0x83\nCMD_GEAR_CHANGED = 0x84\nCMD_SCAN_TEST = 0x85\nCMD_SCAN_END = 0x86\n\nSTATUS_OK = 0\nSTATUS_BAD_LENGTH = 1\nSTATUS_BAD_VALUE = 2\nSTATUS_WRONG_STATE = 3\nSTATUS_UNSUPPORTED = 4\nSTATUS_BUSY = 5\nSTATUS_INTERNAL_ERROR = 6\n\n\nclass ProtocolError(RuntimeError):\n    pass\n\n\nclass ResponseTimeout(ProtocolError):\n    pass\n\n\nclass CommandRejected(ProtocolError):\n    def __init__(self, command: int, status: int):\n        super().__init__(\n            "command 0x{:02X} rejected, status={}".format(\n                command,\n                status,\n            )\n        )\n        self.command = command\n        self.status = status\n\n\nclass RemoteAborted(ProtocolError):\n    pass\n\n\n@dataclass\nclass Frame:\n    command: int\n    sequence: int\n    payload: bytes\n    version: int = PROTOCOL_VERSION\n\n\ndef crc8_atm(data: bytes) -> int:\n    crc = 0x00\n\n    for value in bytearray(data):\n        crc ^= value\n\n        for _ in range(8):\n            if crc & 0x80:\n                crc = ((crc << 1) ^ 0x07) & 0xFF\n            else:\n                crc = (crc << 1) & 0xFF\n\n    return crc\n\n\ndef encode_frame(frame: Frame) -> bytes:\n    payload = bytes(frame.payload)\n\n    if len(payload) > MAX_PAYLOAD:\n        raise ValueError(\n            "payload长度{}超过{}".format(\n                len(payload),\n                MAX_PAYLOAD,\n            )\n        )\n\n    body = bytes(bytearray([\n        frame.version & 0xFF,\n        frame.command & 0xFF,\n        frame.sequence & 0xFF,\n        len(payload) & 0xFF,\n    ])) + payload\n\n    return SOF + body + bytes(bytearray([\n        crc8_atm(body),\n    ]))\n\n\nclass FrameParser:\n    """支持任意分包、粘包和CRC错误后的重新同步。"""\n\n    def __init__(self):\n        self.buffer = bytearray()\n        self.crc_error_count = 0\n        self.version_error_count = 0\n        self.length_error_count = 0\n\n    def feed(self, data: bytes):\n        if data:\n            self.buffer.extend(data)\n\n        output = []\n\n        while True:\n            start = self.buffer.find(SOF)\n\n            if start < 0:\n                # 保留可能是同步头第一个字节的0xA5。\n                if self.buffer[-1:] == SOF[:1]:\n                    self.buffer[:] = self.buffer[-1:]\n                else:\n                    self.buffer.clear()\n                break\n\n            if start > 0:\n                del self.buffer[:start]\n\n            # SOF 2 + VER/CMD/SEQ/LEN 4 + CRC 1\n            if len(self.buffer) < 7:\n                break\n\n            version = self.buffer[2]\n            payload_length = self.buffer[5]\n\n            if payload_length > MAX_PAYLOAD:\n                self.length_error_count += 1\n                del self.buffer[0]\n                continue\n\n            total_length = 2 + 4 + payload_length + 1\n\n            if len(self.buffer) < total_length:\n                break\n\n            raw = bytes(self.buffer[:total_length])\n            body = raw[2:-1]\n            received_crc = raw[-1]\n            calculated_crc = crc8_atm(body)\n\n            if received_crc != calculated_crc:\n                self.crc_error_count += 1\n                del self.buffer[0]\n                continue\n\n            del self.buffer[:total_length]\n\n            if version != PROTOCOL_VERSION:\n                self.version_error_count += 1\n                continue\n\n            output.append(Frame(\n                version=version,\n                command=raw[3],\n                sequence=raw[4],\n                payload=raw[6:6 + payload_length],\n            ))\n\n        return output\n\n\nclass SimpleUartLink:\n    """\n    单测量流程UART链路。\n\n    SET_GEAR：\n        等待同SEQ的GEAR_CHANGED，载荷必须返回相同gear。\n\n    FREQUENCY_RESULT / MEASURE_FAILED：\n        等待同SEQ的ACK。\n\n    超时重发使用相同SEQ和完全相同的帧。\n    FPGA必须保证重复命令幂等，不能重复切档或重复提交结果。\n    """\n\n    def __init__(\n        self,\n        port: str,\n        baudrate: int = 921600,\n        response_timeout: float = 0.50,\n        retries: int = 3,\n    ):\n        if serial is None:\n            raise RuntimeError(\n                "缺少pyserial，请执行："\n                "python3 -m pip install pyserial"\n            )\n\n        self.response_timeout = float(response_timeout)\n        self.retries = int(retries)\n        self.sequence = 0\n        self.running = True\n\n        self.serial = serial.Serial(\n            port=port,\n            baudrate=baudrate,\n            bytesize=serial.EIGHTBITS,\n            parity=serial.PARITY_NONE,\n            stopbits=serial.STOPBITS_ONE,\n            timeout=0.02,\n            write_timeout=0.20,\n            xonxoff=False,\n            rtscts=False,\n            dsrdtr=False,\n        )\n\n        self.parser = FrameParser()\n        self.receive_queue = queue.Queue()\n        self.backlog = collections.deque()\n        self.write_lock = threading.Lock()\n\n        self.serial.reset_input_buffer()\n        self.serial.reset_output_buffer()\n\n        self.receive_thread = threading.Thread(\n            target=self._receive_loop,\n            name="uart-receive",\n        )\n        self.receive_thread.daemon = True\n        self.receive_thread.start()\n\n    def _next_sequence(self) -> int:\n        self.sequence = (self.sequence + 1) & 0xFF\n\n        if self.sequence == 0:\n            self.sequence = 1\n\n        return self.sequence\n\n    def _receive_loop(self) -> None:\n        while self.running:\n            try:\n                data = self.serial.read(256)\n            except Exception as error:\n                self.receive_queue.put(error)\n                return\n\n            for frame in self.parser.feed(data):\n                self.receive_queue.put(frame)\n\n    def _send_raw(self, raw: bytes) -> None:\n        with self.write_lock:\n            self.serial.write(raw)\n            self.serial.flush()\n\n    def _get_frame(self, timeout: Optional[float]):\n        if self.backlog:\n            return self.backlog.popleft()\n\n        item = self.receive_queue.get(timeout=timeout)\n\n        if isinstance(item, Exception):\n            raise ProtocolError(\n                "UART接收线程异常：{}".format(item)\n            )\n\n        return item\n\n    def _wait_matching(\n        self,\n        predicate,\n        timeout: float,\n    ) -> Frame:\n        deadline = time.monotonic() + timeout\n        skipped = []\n\n        try:\n            while True:\n                remaining = deadline - time.monotonic()\n\n                if remaining <= 0:\n                    raise ResponseTimeout(\n                        "等待FPGA响应超时"\n                    )\n\n                try:\n                    frame = self._get_frame(remaining)\n                except queue.Empty:\n                    raise ResponseTimeout(\n                        "等待FPGA响应超时"\n                    )\n\n                if (\n                    isinstance(frame, Frame)\n                    and frame.command == CMD_ABORTED\n                ):\n                    raise RemoteAborted(\n                        "FPGA主动取消测量"\n                    )\n\n                if predicate(frame):\n                    return frame\n\n                skipped.append(frame)\n        finally:\n            while skipped:\n                self.backlog.appendleft(\n                    skipped.pop()\n                )\n\n    def get_event(\n        self,\n        timeout: Optional[float] = None,\n    ) -> Frame:\n        return self._get_frame(timeout)\n\n    def poll_event(self):\n        try:\n            return self._get_frame(0.0)\n        except queue.Empty:\n            return None\n\n    def set_gear_and_wait_changed(\n        self,\n        gear: int,\n        timeout: Optional[float] = None,\n    ) -> Frame:\n        if gear not in (0, 1, 2, 3):\n            raise ValueError("gear必须为0～3")\n\n        sequence = self._next_sequence()\n        raw = encode_frame(Frame(\n            command=CMD_SET_GEAR,\n            sequence=sequence,\n            payload=bytes(bytearray([gear])),\n        ))\n        wait_timeout = (\n            self.response_timeout\n            if timeout is None\n            else float(timeout)\n        )\n\n        for attempt in range(self.retries + 1):\n            self._send_raw(raw)\n\n            try:\n                return self._wait_matching(\n                    lambda frame: (\n                        isinstance(frame, Frame)\n                        and frame.command\n                        == CMD_GEAR_CHANGED\n                        and frame.sequence == sequence\n                        and frame.payload\n                        == bytes(bytearray([gear]))\n                    ),\n                    wait_timeout,\n                )\n            except ResponseTimeout:\n                if attempt >= self.retries:\n                    raise\n\n        raise ResponseTimeout("不可达")\n\n    def send_frequency_and_wait_ack(\n        self,\n        frequency_code: int,\n        timeout: Optional[float] = None,\n    ) -> Frame:\n        if not 0 <= frequency_code <= 0xFFFF:\n            raise ValueError(\n                "frequency_code超出u16范围"\n            )\n\n        payload = bytes(bytearray([\n            frequency_code & 0xFF,\n            (frequency_code >> 8) & 0xFF,\n        ]))\n\n        return self._send_command_wait_ack(\n            command=CMD_FREQUENCY_RESULT,\n            payload=payload,\n            timeout=timeout,\n        )\n\n    def send_failure_and_wait_ack(\n        self,\n        reason: int,\n        timeout: Optional[float] = None,\n    ) -> Frame:\n        return self._send_command_wait_ack(\n            command=CMD_MEASURE_FAILED,\n            payload=bytes(bytearray([\n                reason & 0xFF,\n            ])),\n            timeout=timeout,\n        )\n\n    def _send_command_wait_ack(\n        self,\n        command: int,\n        payload: bytes,\n        timeout: Optional[float],\n    ) -> Frame:\n        sequence = self._next_sequence()\n        raw = encode_frame(Frame(\n            command=command,\n            sequence=sequence,\n            payload=payload,\n        ))\n        wait_timeout = (\n            self.response_timeout\n            if timeout is None\n            else float(timeout)\n        )\n\n        for attempt in range(self.retries + 1):\n            self._send_raw(raw)\n\n            try:\n                ack = self._wait_matching(\n                    lambda frame: (\n                        isinstance(frame, Frame)\n                        and frame.command == CMD_ACK\n                        and frame.sequence == sequence\n                        and len(frame.payload) == 2\n                    ),\n                    wait_timeout,\n                )\n            except ResponseTimeout:\n                if attempt >= self.retries:\n                    raise\n                continue\n\n            original_command = ack.payload[0]\n            status = ack.payload[1]\n\n            if original_command != command:\n                raise ProtocolError(\n                    "ACK中的原命令不匹配"\n                )\n\n            if status != STATUS_OK:\n                raise CommandRejected(\n                    command,\n                    status,\n                )\n\n            return ack\n\n        raise ResponseTimeout("不可达")\n\n\n    def send_scan_decision_and_wait_ack(\n        self,\n        scan_offset_steps: int,\n        correct: bool,\n        timeout: Optional[float] = None,\n    ) -> Frame:\n        """\n        回复当前扫频点判定结果。\n\n        scan_offset_steps采用有符号i16：\n            0, -1, +1, -2, +2, ...\n\n        correct=True  -> CMD_SCAN_CORRECT\n        correct=False -> CMD_SCAN_WRONG\n        """\n        if not -32768 <= int(scan_offset_steps) <= 32767:\n            raise ValueError(\n                "scan_offset_steps超出i16范围"\n            )\n\n        encoded = int(scan_offset_steps) & 0xFFFF\n        payload = bytes(bytearray([\n            encoded & 0xFF,\n            (encoded >> 8) & 0xFF,\n        ]))\n\n        command = (\n            CMD_SCAN_CORRECT\n            if correct\n            else CMD_SCAN_WRONG\n        )\n\n        return self._send_command_wait_ack(\n            command=command,\n            payload=payload,\n            timeout=timeout,\n        )\n\n    def close(self) -> None:\n        if not self.running:\n            return\n\n        self.running = False\n\n        try:\n            self.receive_thread.join(\n                timeout=0.30\n            )\n        finally:\n            self.serial.close()\n\n    def __enter__(self):\n        return self\n\n    def __exit__(\n        self,\n        exc_type,\n        exc_value,\n        traceback,\n    ):\n        self.close()\n'

LISSAJOUS_JUDGE_SOURCE = '#!/usr/bin/env python3\n# -*- coding: utf-8 -*-\n"""\n全图白色像素占比判定器。\n\n彻底撤销最大连通域、外接矩形、椭圆拟合、直线拟合、IoU和覆盖率方案。\n每帧只执行：\n1. 九点校正后的图像转灰度；\n2. 灰度值大于binary_threshold的像素记为白色；\n3. 直接统计全图白色像素数；\n4. global_white_ratio = 白色像素数 / 全图总像素数；\n5. 全图白色占比不超过maximum_duty_ratio时记为一票正确。\n\n默认灰度阈值90。帧投票支持提前结束，避免无意义地采满全部帧。\n"""\nfrom __future__ import print_function\n\nimport json\nimport time\nfrom dataclasses import asdict, dataclass\nfrom pathlib import Path\n\nimport cv2\nimport numpy as np\n\nVERSION = "JETSON-GLOBAL-WHITE-RATIO-V5.0"\n\n\n@dataclass\nclass LissajousMetrics:\n    frame_count: int\n    required_frames: int\n    verdict: str\n    is_lissajous: bool\n    shape_name: str\n    confidence: float\n\n    # 兼容旧控制器字段。\n    bright_ratio: float\n    union_ratio: float\n    persistent_ratio: float\n    mean_iou: float\n    temporal_spread: float\n    thickness_px: float\n    largest_component_ratio: float\n    x_span_ratio: float\n    y_span_ratio: float\n    trace_contrast: float\n    hole_count: int\n    line_error: float\n    ellipse_error: float\n    ellipse_coverage: float\n    phase_cosine: float\n    phase_degrees: float\n    threshold: float\n    reason: str\n\n    # 新方案真正使用的字段。\n    duty_ratio: float = 1.0\n    duty_ratio_percent: float = 100.0\n    global_white_pixels: int = 0\n    total_pixels: int = 0\n    global_white_ratio: float = 1.0\n\n    # 继续保留旧日志字段，统一映射为全图数据。\n    component_area: int = 0\n    component_area_ratio: float = 0.0\n    bbox_x: int = 0\n    bbox_y: int = 0\n    bbox_width: int = 0\n    bbox_height: int = 0\n    bbox_area: int = 0\n    bbox_area_ratio: float = 1.0\n    white_pixels_in_bbox: int = 0\n\n    def to_dict(self):\n        return asdict(self)\n\n\nclass Config(object):\n    """只保留灰度阈值、全图白色占比上限和帧投票。"""\n\n    _PRESETS = {\n        # mode: (最多帧数, 需要通过票数, 每格像素)\n        \'fast\': (1, 1, 32),\n        \'balanced\': (3, 2, 32),\n        \'robust\': (5, 3, 40),\n    }\n\n    def __init__(\n        self,\n        mode=\'balanced\',\n        pixels_per_div=None,\n        window_frames=None,\n        binary_threshold=90,\n        maximum_duty_ratio=0.75,\n        **_legacy_unused\n    ):\n        mode = str(mode).lower()\n        if mode not in self._PRESETS:\n            raise ValueError(\'judge mode必须为fast、balanced或robust\')\n\n        preset_frames, preset_votes, preset_ppd = self._PRESETS[mode]\n        self.mode = mode\n        self.pixels_per_div = int(\n            preset_ppd if pixels_per_div is None else pixels_per_div\n        )\n        self.window_frames = int(\n            preset_frames if window_frames is None else window_frames\n        )\n        self.required_matched_windows = min(\n            self.window_frames,\n            preset_votes if window_frames is None\n            else max(1, (self.window_frames + 1) // 2),\n        )\n\n        self.binary_threshold = int(binary_threshold)\n        self.maximum_duty_ratio = float(maximum_duty_ratio)\n\n        # 兼容旧控制器字段。\n        self.max_windows = 1\n        self.search_frames = self.window_frames\n        self.confirmation_repeats = 1\n        self.confirmation_frames = self.window_frames\n        self.confirmation_settle_ms = 0.0\n        self.phase_tolerance_degrees = 3.0\n        self.trace_threshold_scale = 1.0\n\n        if self.pixels_per_div < 16:\n            raise ValueError(\'pixels_per_div不能小于16\')\n        if self.window_frames < 1:\n            raise ValueError(\'window_frames不能小于1\')\n        if not 0 <= self.binary_threshold <= 255:\n            raise ValueError(\'binary_threshold必须位于0～255\')\n        if not 0.0 <= self.maximum_duty_ratio <= 1.0:\n            raise ValueError(\'maximum_duty_ratio必须位于0～1\')\n\n    @property\n    def width(self):\n        return 8 * self.pixels_per_div + 1\n\n    @property\n    def height(self):\n        return 8 * self.pixels_per_div + 1\n\n\nclass FrameResult(object):\n    def __init__(\n        self,\n        metrics,\n        matched,\n        batch_index,\n        window_index,\n        source=None,\n        diagnostic=None,\n        latest=None,\n        union=None,\n    ):\n        self.metrics = metrics\n        self.passed = bool(matched)\n        self.shape = metrics.shape_name if metrics.shape_name != \'NONE\' else metrics.verdict\n        self.quality = float(metrics.confidence)\n        self.reason = str(metrics.reason)\n        self.batch_index = int(batch_index)\n        self.window_index = int(window_index)\n        self.rectified = source\n        self.diagnostic = diagnostic\n        self.mask = latest\n        self.union_mask = union\n        self.elapsed_ms = 0.0\n\n        # 兼容旧控制器字段。\n        self.line_error = float(\'nan\')\n        self.opening_angle_deg = float(\'nan\')\n        self.ellipse_error = float(\'nan\')\n        self.fill_ratio = float(metrics.duty_ratio)\n        self.profile_coverage = float(metrics.duty_ratio)\n        self.center_wander_ratio = float(\'nan\')\n        self.thickness_div = float(\'nan\')\n        self.center_offset_div = float(\'nan\')\n        self.axis_ratio = float(\'nan\')\n        self.ellipse = None\n\n\nclass JudgeResult(object):\n    def __init__(\n        self,\n        correct,\n        reason,\n        passed_votes,\n        failed_votes,\n        frames_used,\n        elapsed_ms,\n        frame_results,\n        batch_confirmations,\n        total_batches,\n        best_metrics,\n    ):\n        self.correct = bool(correct)\n        self.reason = str(reason)\n        self.passed_votes = int(passed_votes)\n        self.failed_votes = int(failed_votes)\n        self.frames_used = int(frames_used)\n        self.elapsed_ms = float(elapsed_ms)\n        self.frame_results = list(frame_results)\n        self.batch_confirmations = int(batch_confirmations)\n        self.total_batches = int(total_batches)\n        self.best_metrics = best_metrics\n\n\ndef to_bgr_u8(frame):\n    if frame is None:\n        raise ValueError(\'图像为空\')\n    if frame.ndim == 2:\n        gray = frame\n        if gray.dtype != np.uint8:\n            array = gray.astype(np.float32)\n            low, high = np.percentile(array, [0.2, 99.8])\n            if high <= low:\n                gray = np.zeros(array.shape, np.uint8)\n            else:\n                gray = np.rint(\n                    np.clip((array - low) / (high - low), 0, 1) * 255\n                ).astype(np.uint8)\n        return cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)\n    if frame.shape[2] == 4:\n        frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)\n    if frame.dtype == np.uint8:\n        return frame\n    array = frame.astype(np.float32)\n    low, high = np.percentile(array, [0.2, 99.8])\n    if high <= low:\n        return np.zeros(array.shape, np.uint8)\n    return np.rint(\n        np.clip((array - low) / (high - low), 0, 1) * 255\n    ).astype(np.uint8)\n\n\ndef validate_points(points):\n    points = np.asarray(points, np.float32)\n    if points.shape != (3, 3, 2):\n        raise ValueError(\'points_3x3必须是3×3×2\')\n    if np.any(np.diff(points[:, :, 0], axis=1) <= 0):\n        raise ValueError(\'九点标定左右顺序错误\')\n    if np.any(np.diff(points[:, :, 1], axis=0) <= 0):\n        raise ValueError(\'九点标定上下顺序错误\')\n    for row in range(2):\n        for col in range(2):\n            quad = np.float32([\n                points[row, col],\n                points[row, col + 1],\n                points[row + 1, col + 1],\n                points[row + 1, col],\n            ])\n            if abs(cv2.contourArea(quad)) < 100:\n                raise ValueError(\'九点标定存在退化区域\')\n\n\ndef load_calibration(path, image_shape):\n    with open(str(path), \'r\', encoding=\'utf-8\') as file:\n        data = json.load(file)\n    points = np.asarray(data[\'points_3x3\'], np.float32)\n    height, width = image_shape[:2]\n    points[:, :, 0] *= width / float(data[\'image_width\'])\n    points[:, :, 1] *= height / float(data[\'image_height\'])\n    validate_points(points)\n    return points\n\n\nclass Rectifier(object):\n    def __init__(self, points, cfg):\n        self.cfg = cfg\n        self.map_x = np.zeros((cfg.height, cfg.width), np.float32)\n        self.map_y = np.zeros_like(self.map_x)\n        half = 4 * cfg.pixels_per_div\n        for row in range(2):\n            for col in range(2):\n                source = np.float32([\n                    points[row, col],\n                    points[row, col + 1],\n                    points[row + 1, col + 1],\n                    points[row + 1, col],\n                ])\n                destination = np.float32([\n                    [col * half, row * half],\n                    [(col + 1) * half, row * half],\n                    [(col + 1) * half, (row + 1) * half],\n                    [col * half, (row + 1) * half],\n                ])\n                inverse = cv2.getPerspectiveTransform(destination, source)\n                x0 = col * half\n                y0 = row * half\n                x1 = (col + 1) * half if col == 0 else cfg.width - 1\n                y1 = (row + 1) * half if row == 0 else cfg.height - 1\n                xx, yy = np.meshgrid(\n                    np.arange(x0, x1 + 1, dtype=np.float32),\n                    np.arange(y0, y1 + 1, dtype=np.float32),\n                )\n                denominator = (\n                    inverse[2, 0] * xx\n                    + inverse[2, 1] * yy\n                    + inverse[2, 2]\n                )\n                self.map_x[y0:y1 + 1, x0:x1 + 1] = (\n                    inverse[0, 0] * xx\n                    + inverse[0, 1] * yy\n                    + inverse[0, 2]\n                ) / denominator\n                self.map_y[y0:y1 + 1, x0:x1 + 1] = (\n                    inverse[1, 0] * xx\n                    + inverse[1, 1] * yy\n                    + inverse[1, 2]\n                ) / denominator\n\n    def __call__(self, image):\n        return cv2.remap(\n            image,\n            self.map_x,\n            self.map_y,\n            cv2.INTER_LINEAR,\n            borderMode=cv2.BORDER_REPLICATE,\n        )\n\n\ndef read_frame(capture):\n    ok, frame = capture.read()\n    if not ok or frame is None:\n        raise RuntimeError(\'读取图像失败\')\n    return frame\n\n\ndef discard_frames(capture, count):\n    for _ in range(max(0, int(count))):\n        if not capture.grab():\n            break\n\n\ndef _gray_u8(image):\n    bgr = to_bgr_u8(image)\n    return cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)\n\n\ndef analyse_global_white_ratio(frame, cfg):\n    """直接统计阈值二值图的全局白色像素占比。"""\n    gray = _gray_u8(frame)\n\n    # 椭圆更亮：灰度值严格大于阈值的像素记为白色。\n    _, binary = cv2.threshold(\n        gray,\n        cfg.binary_threshold,\n        255,\n        cv2.THRESH_BINARY,\n    )\n\n    height, width = binary.shape\n    total_pixels = int(binary.size)\n    white_pixels = int(cv2.countNonZero(binary))\n    global_ratio = white_pixels / float(max(total_pixels, 1))\n\n    # 用户指定：稳定时白像素占全图比例较小，因此小于等于上限即通过。\n    passed = bool(global_ratio <= cfg.maximum_duty_ratio)\n\n    if passed:\n        reason = \'全图白色像素占比{:.2f}%不超过{:.2f}%\'.format(\n            100.0 * global_ratio,\n            100.0 * cfg.maximum_duty_ratio,\n        )\n        margin = max(cfg.maximum_duty_ratio, 1e-9)\n        confidence = 0.5 + 0.5 * max(\n            0.0,\n            min(1.0, 1.0 - global_ratio / margin),\n        )\n    else:\n        reason = \'全图白色像素占比{:.2f}%超过{:.2f}%\'.format(\n            100.0 * global_ratio,\n            100.0 * cfg.maximum_duty_ratio,\n        )\n        confidence = 0.0\n\n    metrics = LissajousMetrics(\n        frame_count=1,\n        required_frames=1,\n        verdict=\'YES\' if passed else \'NO\',\n        is_lissajous=passed,\n        shape_name=\'GLOBAL_WHITE\' if passed else \'NONE\',\n        confidence=float(confidence),\n        bright_ratio=float(global_ratio),\n        union_ratio=float(global_ratio),\n        persistent_ratio=float(global_ratio),\n        mean_iou=1.0 if passed else 0.0,\n        temporal_spread=0.0,\n        thickness_px=0.0,\n        largest_component_ratio=1.0,\n        x_span_ratio=1.0,\n        y_span_ratio=1.0,\n        trace_contrast=0.0,\n        hole_count=0,\n        line_error=float(\'nan\'),\n        ellipse_error=float(\'nan\'),\n        ellipse_coverage=float(global_ratio),\n        phase_cosine=float(\'nan\'),\n        phase_degrees=float(\'nan\'),\n        threshold=float(cfg.binary_threshold),\n        reason=reason,\n        duty_ratio=float(global_ratio),\n        duty_ratio_percent=float(100.0 * global_ratio),\n        global_white_pixels=white_pixels,\n        total_pixels=total_pixels,\n        global_white_ratio=float(global_ratio),\n        component_area=white_pixels,\n        component_area_ratio=float(global_ratio),\n        bbox_x=0,\n        bbox_y=0,\n        bbox_width=width,\n        bbox_height=height,\n        bbox_area=total_pixels,\n        bbox_area_ratio=1.0,\n        white_pixels_in_bbox=white_pixels,\n    )\n\n    diagnostic = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)\n    color = (0, 255, 0) if passed else (0, 0, 255)\n    cv2.putText(\n        diagnostic,\n        \'gray>{} white={:.2f}% limit={:.2f}%\'.format(\n            cfg.binary_threshold,\n            100.0 * global_ratio,\n            100.0 * cfg.maximum_duty_ratio,\n        ),\n        (8, 22),\n        cv2.FONT_HERSHEY_SIMPLEX,\n        0.50,\n        color,\n        1,\n        cv2.LINE_AA,\n    )\n    cv2.putText(\n        diagnostic,\n        \'white={}/{} {}\'.format(\n            white_pixels,\n            total_pixels,\n            \'CORRECT\' if passed else \'WRONG\',\n        ),\n        (8, 44),\n        cv2.FONT_HERSHEY_SIMPLEX,\n        0.46,\n        color,\n        1,\n        cv2.LINE_AA,\n    )\n\n    return metrics, binary, diagnostic\n\n\n# 兼容旧函数名，内部已完全改为全图白像素统计。\ndef analyse_max_component(frame, cfg):\n    return analyse_global_white_ratio(frame, cfg)\n\n\ndef analyse_lissajous_window(frames, required_frames=1, cfg=None):\n    config = cfg if cfg is not None else Config()\n    items = []\n    for frame in list(frames)[-max(1, int(required_frames)):]:\n        metrics, mask, diagnostic = analyse_global_white_ratio(frame, config)\n        items.append((metrics, mask, diagnostic))\n    if not items:\n        raise ValueError(\'frames为空\')\n\n    # 稳定目标的白色占比更小，因此取占比最小的一帧作为代表。\n    metrics, mask, diagnostic = min(\n        items,\n        key=lambda item: item[0].global_white_ratio,\n    )\n    return metrics, mask, mask, diagnostic\n\n\ndef classify_frequency_candidate(m, cfg=None, allow_low_coverage_full_span=True):\n    del cfg, allow_low_coverage_full_span\n    return bool(m.is_lissajous), m.shape_name, m.reason\n\n\ndef is_stable_frequency_candidate(m, allow_low_coverage_full_span=True, cfg=None):\n    del allow_low_coverage_full_span, cfg\n    return bool(m.is_lissajous)\n\n\ndef is_fast_accept_candidate(m, cfg=None):\n    del cfg\n    return bool(m.is_lissajous)\n\n\nclass FastLissajousJudge(object):\n    def __init__(\n        self,\n        capture,\n        rectifier,\n        cfg,\n        settle_ms=100.0,\n        discard_count=1,\n        debug_dir=None,\n    ):\n        self.capture = capture\n        self.rectifier = rectifier\n        self.cfg = cfg\n        self.settle_ms = float(settle_ms)\n        self.discard_count = int(discard_count)\n        self.debug_dir = Path(debug_dir) if debug_dir else None\n        self.request_index = 0\n        if self.debug_dir:\n            self.debug_dir.mkdir(parents=True, exist_ok=True)\n\n    def _capture_one_rectified(self):\n        frame = to_bgr_u8(read_frame(self.capture))\n        return self.rectifier(frame)\n\n    def _analyse_frame(self, frame, frame_index):\n        metrics, binary, diagnostic = analyse_global_white_ratio(\n            frame,\n            self.cfg,\n        )\n        return FrameResult(\n            metrics=metrics,\n            matched=metrics.is_lissajous,\n            batch_index=1,\n            window_index=frame_index,\n            source=frame,\n            diagnostic=diagnostic,\n            latest=binary,\n            union=binary,\n        )\n\n    def _save_debug(self, request_index, result):\n        if self.debug_dir is None:\n            return\n        prefix = \'request_{:04d}_frame_{:02d}_{}\'.format(\n            request_index,\n            result.window_index,\n            \'YES\' if result.passed else \'NO\',\n        )\n        if result.rectified is not None:\n            cv2.imwrite(\n                str(self.debug_dir / (prefix + \'_source.png\')),\n                result.rectified,\n            )\n        if result.mask is not None:\n            cv2.imwrite(\n                str(self.debug_dir / (prefix + \'_binary.png\')),\n                result.mask,\n            )\n        if result.diagnostic is not None:\n            cv2.imwrite(\n                str(self.debug_dir / (prefix + \'_diagnostic.png\')),\n                result.diagnostic,\n            )\n\n    @staticmethod\n    def _best_metrics(results):\n        # 目标稳定时全局白占比最小。\n        return min(\n            (item.metrics for item in results),\n            key=lambda metrics: metrics.global_white_ratio,\n        )\n\n    def judge_current_frequency(self):\n        started = time.perf_counter()\n        self.request_index += 1\n        request_index = self.request_index\n\n        if self.settle_ms > 0:\n            time.sleep(self.settle_ms / 1000.0)\n        discard_frames(self.capture, self.discard_count)\n\n        results = []\n        passed = 0\n\n        # 逐帧采集和投票。一旦结果已确定，立即停止，不再采多余帧。\n        for frame_index in range(1, self.cfg.window_frames + 1):\n            frame = self._capture_one_rectified()\n            result = self._analyse_frame(frame, frame_index)\n            results.append(result)\n            self._save_debug(request_index, result)\n\n            if result.passed:\n                passed += 1\n\n            remaining = self.cfg.window_frames - frame_index\n\n            # 已达到通过票数，立即返回。\n            if passed >= self.cfg.required_matched_windows:\n                break\n\n            # 即使剩余帧全部通过也达不到票数，立即判错。\n            if passed + remaining < self.cfg.required_matched_windows:\n                break\n\n        failed = len(results) - passed\n        correct = passed >= self.cfg.required_matched_windows\n        best = self._best_metrics(results)\n        reason = (\n            \'GLOBAL_WHITE_VOTE_PASSED\'\n            if correct else\n            \'GLOBAL_WHITE_VOTE_REJECTED\'\n        )\n\n        return JudgeResult(\n            correct=correct,\n            reason=reason,\n            passed_votes=passed,\n            failed_votes=failed,\n            frames_used=len(results),\n            elapsed_ms=(time.perf_counter() - started) * 1000.0,\n            frame_results=results,\n            batch_confirmations=1 if correct else 0,\n            total_batches=1,\n            best_metrics=best,\n        )\n'

_load_embedded_module('frequency_measure_core', MEASURE_CORE_SOURCE)
_load_embedded_module('range_switch_core', SWITCH_CORE_SOURCE)
_load_embedded_module('simple_uart_protocol', UART_PROTOCOL_SOURCE)
_load_embedded_module('lissajous_judge_core', LISSAJOUS_JUDGE_SOURCE)


# -*- coding: utf-8 -*-
"""
Jetson四档量程判断 + 八帧频率测量（简化UART握手版）

严格执行以下流程：

FPGA发送START，且FPGA已经处于gear0
    ↓
Jetson的switch算法连续判断5帧
    ↓
4票认为周期太长？
    ├─ 是：
    │    Jetson发送SET_GEAR(next)
    │    FPGA完成切换后发送GEAR_CHANGED(next)
    │    Jetson等待画面稳定、丢弃旧帧、清空投票
    │    继续判断下一档
    │
    └─ 否：
         锁定当前档位
    ↓
按档位选择有效时间、频率范围和K
    ↓
复用frequency_measure_core的周期测量
    ↓
收集8个有效帧
    ↓
主频簇过滤 + MAD异常过滤 + 中值融合
    ↓
量化到100 Hz
    ↓
UART只发送frequency_code:u16
frequency_code = frequency_hz / 100

例如：
    20.4 kHz -> 204
    80.3 kHz -> 803
    100.0 kHz -> 1000
"""


import argparse
import json
import math
import queue
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Optional

import cv2
import numpy as np
import lissajous_phase_local as phase_core

import frequency_measure_core as measure_core
import range_switch_core as switch_core
import lissajous_judge_core as lissajous_core

from simple_uart_protocol import (
    CMD_ACK,
    CMD_ABORTED,
    CMD_GEAR_CHANGED,
    CMD_MEASURE_FAILED,
    CMD_SCAN_END,
    CMD_SCAN_TEST,
    CMD_START,
    Frame,
    ProtocolError,
    RemoteAborted,
    ResponseTimeout,
    SimpleUartLink,
    encode_frame,
)


# ==========================================================================
#  全局常量
# ==========================================================================

VERSION = "JETSON-FREQ-SIMPLE-ADAPTIVE-V6.0"

# 精扫判图：全图白色像素占比超过此阈值则判定为"方块"(超量程噪声)。
# 正常李萨如图形在二值化后白色比例通常远低于30%。
BLOCK_WHITE_RATIO_LIMIT = 0.30

# ---- UART 协议扩展命令 (Jetson → FPGA) ----
# 设置绝对相位补偿。payload为i32小端，单位0.01度 (cdeg)。
# 范围 [-18000, +18000] 即 ±180.00 度。
CMD_SET_PHASE = 0x1D
# DONE：精扫+调相成功后通知FPGA保持当前输出、点亮LED4。
# payload为空，FPGA回复ACK后进入保持状态。
CMD_DONE = 0x1E
# RESCAN：DONE保护期结束后通知FPGA重新输出gear0锯齿波、
# 清除无线状态机，ACK后Jetson不等待新START直接开始新一轮粗测。
CMD_RESCAN = 0x1F

# ---- 相位补偿相关常量 ----
# FPGA的SET_PHASE按0.01度(cdeg)单位接收，不做额外缩放。
PHASE_FPGA_TEST_SCALE = 1
# 大相位跨度阈值：当实测椭圆相位超过6.0 div时 (约48.59°)，
# 不直接做一次性大步长补偿，而是先±60°探测找到可拟合的中间状态。
PHASE_LARGE_SPAN_DIV = 6.0
PHASE_LARGE_SPAN_DEG = math.degrees(math.asin(PHASE_LARGE_SPAN_DIV / 8.0))
# 探测步长：±60.00度 (单位cdeg)
PHASE_PROBE_CDEG = 6000
PHASE_ESCAPE_CANDIDATES_CDEG = (
    PHASE_PROBE_CDEG,
    -PHASE_PROBE_CDEG,
)

# 精扫首帧额外稳定时间 (50 ms)，
# 补偿FPGA频率切换后示波器模拟带宽和摄像头曝光的过渡过程。
FINE_SWEEP_FIRST_FRAME_EXTRA_SETTLE_SECONDS = 0.050

# ---- 无限型(∞)相位死区常量 ----
# 当∞形中心交叉点偏移超过3.0 div时，arcsin(3/4)≈48.59°，
# 交叉点已靠近图形顶部/底部，数值拟合极易失败。
# 此情况下启动±60°相位逃脱序列。
INFINITY_DEAD_ZONE_DIV = 3.0
INFINITY_DEAD_ZONE_DEG = math.degrees(math.asin(INFINITY_DEAD_ZONE_DIV / 4.0))

# DONE后保护期：6秒内禁止因画面变化发送RESCAN。
# 保证赛题要求的"稳定时间≥5s"并被声光提示后不会立即被重扫覆盖。
DONE_RESCAN_HOLDOFF_SECONDS = 6.0

# ---- 图形类型编号 (与FPGA START协议一致) ----
PATTERN_DIAGONAL = 0   # 对角线/直线 (信号源直通或1:1同相)
PATTERN_CIRCLE = 1     # 圆/椭圆 (1:1正交90°)
PATTERN_INFINITY = 2   # 水平∞形 (1:2二倍频)
SUPPORTED_PATTERNS = (
    PATTERN_DIAGONAL,
    PATTERN_CIRCLE,
    PATTERN_INFINITY,
)


# ==========================================================================
#  数据结构定义
# ==========================================================================

@dataclass(frozen=True)
class StartRequest:
    """FPGA START事件解析结果。

    由FPGA KEY2/KEY3/KEY4 在无线模式下触发，包含起始档位和图形类型。
    payload为空时默认 gear=0, pattern=DIAGONAL。
    """
    initial_gear: int = 0       # FPGA当前起始锯齿档位 (0~3)
    pattern_id: int = PATTERN_DIAGONAL  # 目标图形类型

    @property
    def target_phase_abs_deg(self) -> float:
        """返回当前图形的目标绝对相位差 (度)。

        对角线/∞形 → 0°, 圆 → 90° (正交)。
        """
        if self.pattern_id in (PATTERN_DIAGONAL, PATTERN_INFINITY):
            return 0.0
        if self.pattern_id == PATTERN_CIRCLE:
            return 90.0
        raise ValueError("未知pattern_id={}".format(self.pattern_id))


# ---- MEASURE_FAILED 失败原因码 (Jetson → FPGA) ----
FAIL_RANGE_TIMEOUT = 1    # 四档选档超时 (锯齿周期无法匹配输入频率)
FAIL_MEASURE_TIMEOUT = 2  # 八帧粗测频超时或无法收敛
FAIL_CAMERA = 3           # 摄像头读取失败
FAIL_ABORTED = 4          # FPGA发送ABORTED主动取消
FAIL_INTERNAL = 255       # 内部异常 (协议错误、参数无效等)


@dataclass(frozen=True)
class RangeProfile:
    """单个量程档位的标定参数。

    Attributes:
        gear: 档位编号 0~3
        effective_time_us: FPGA锯齿波有效扫描时间 (μs)
        frequency_min_hz: 该档位可测频段下限
        frequency_max_hz: 该档位可测频段上限
        k_hz_div: 标定系数 k, f = k / period_div (单位 Hz·div)
        switch_threshold_div: 触发切档的周期阈值 (div)；gear=3 为 None
    """
    gear: int
    effective_time_us: int
    frequency_min_hz: float
    frequency_max_hz: float
    k_hz_div: float
    switch_threshold_div: Optional[float]


@dataclass
class FinalMeasurement:
    """八帧融合测频的最终输出。

    frequency_code 以 100 Hz 为步进量化为 u16 (10~1000 对应 1~100 kHz)。
    """
    gear: int               # 实际使用的量程档位
    frequency_hz: float     # 融合输出频率 (Hz, 已量化到 100 Hz 步进)
    frequency_code: int     # 频率编码 (frequency_hz / 100, 范围 10~1000)
    period_div: float       # 对应周期 (div)
    used_frames: int        # 融合使用的有效帧数 (内点)
    rejected_frames: int    # 融合剔除的帧数 (外点)


# ==========================================================================
#  异常类定义 —— 控制流信号
#  这些异常不是"错误"而是控制流分支信号，在 main() 的 try/except 中被捕获
# ==========================================================================

class MeasurementAborted(RuntimeError):
    """用户按Q/Esc停止，或FPGA发送ABORTED事件。"""
    pass


class MeasurementNeedsHigherGear(RuntimeError):
    """当前档正式测频无法收敛，需切到更长锯齿观察时间的下一档。"""
    pass


class StartPreempted(RuntimeError):
    """FPGA新START事件始终拥有最高优先级，抢占当前正在执行的流程。

    被捕获后将新START帧放回队首，main()重新按新pattern和initial_gear开始。
    """

    def __init__(self, frame: Frame):
        super().__init__("FPGA新START抢占当前流程")
        self.frame = frame


# ==========================================================================
#  FpgaCompatibleUartLink —— FPGA UART 协议适配层
# ==========================================================================

class FpgaCompatibleUartLink(SimpleUartLink):
    """FPGA无线模式UART协议的 Jetson 侧适配层。

    封装了 SimpleUartLink (来自 uart_link.py 内嵌模块) 的底层帧收发，
    处理 FPGA 的事件/响应不对称行为：

    关键行为差异：
    1. MEASURE_FAILED 不触发 ACK -- FPGA 故意静默，避免失败重试风暴。
    2. START/SCAN_TEST/SCAN_END 是异步事件，进入 backlog 供外层轮询。
    3. ACK/GEAR_CHANGED 是同步响应，仅在被等待时消费，不入 backlog。
       ---- 防止延迟ACK在序号回绕后被误认为新响应。
    4. ABORTED/START 在 _wait_matching 中直接抛异常，不做常规事件处理。
    """

    # 异步事件命令号集合 -- 这些帧进入 backlog, 由外层 get_event() 轮询
    ASYNC_EVENT_COMMANDS = {
        CMD_START,
        CMD_ABORTED,
        CMD_SCAN_TEST,
        CMD_SCAN_END,
    }
    # 同步响应命令号集合 -- 仅在 _wait_matching 中被消费
    RESPONSE_COMMANDS = {
        CMD_ACK,
        CMD_GEAR_CHANGED,
    }

    def _wait_matching(
        self,
        predicate,
        timeout: float,
    ) -> Frame:
        """等待满足条件的响应帧。

        在等待期间：
        - ABORTED → 抛出 RemoteAborted
        - START → 抛出 StartPreempted (更高优先级抢占)
        - 不匹配的帧如果是异步事件 → 暂存到 skipped_events
        - 不匹配的帧如果是同步响应但条件不符 → 丢弃 (可能是过期/重传)

        finally 块保证所有 skipped_events 按原序重新入队到 backlog。
        """
        deadline = time.monotonic() + timeout
        skipped_events = []

        try:
            while True:
                remaining = deadline - time.monotonic()

                if remaining <= 0:
                    raise ResponseTimeout(
                        "等待FPGA响应超时"
                    )

                try:
                    frame = self._get_frame(remaining)
                except queue.Empty:
                    raise ResponseTimeout(
                        "等待FPGA响应超时"
                    )

                if (
                    isinstance(frame, Frame)
                    and frame.command == CMD_ABORTED
                ):
                    raise RemoteAborted(
                        "FPGA请求取消流程"
                    )

                if (
                    isinstance(frame, Frame)
                    and frame.command == CMD_START
                ):
                    raise StartPreempted(frame)

                if predicate(frame):
                    return frame

                if (
                    isinstance(frame, Frame)
                    and frame.command
                    in self.ASYNC_EVENT_COMMANDS
                ):
                    skipped_events.append(frame)
                elif (
                    isinstance(frame, Frame)
                    and frame.command
                    in self.RESPONSE_COMMANDS
                ):
                    pass
                # Unmatched ACK/GEAR_CHANGED responses are stale.  Do not
                # return them to the backlog, especially after SEQ wraps.
        finally:
            while skipped_events:
                self.backlog.appendleft(
                    skipped_events.pop()
                )

    def send_failure_no_ack(
        self,
        reason: int,
    ) -> int:
        """Send MEASURE_FAILED once; the FPGA deliberately stays silent."""
        sequence = self._next_sequence()
        raw = encode_frame(Frame(
            command=CMD_MEASURE_FAILED,
            sequence=sequence,
            payload=bytes(bytearray([
                int(reason) & 0xFF,
            ])),
        ))
        self._send_raw(raw)
        return sequence

    def set_phase_and_wait_ack(
        self,
        phase_cdeg: int,
        timeout: Optional[float] = None,
    ) -> Frame:
        """设置绝对相位；phase_cdeg为线上传输的有符号i32原始值。"""
        value = int(phase_cdeg)
        if not -180000 <= value <= 180000:
            raise ValueError("临时放大后的相位原始值超出允许范围")

        encoded = value & 0xFFFFFFFF
        payload = bytes(bytearray([
            encoded & 0xFF,
            (encoded >> 8) & 0xFF,
            (encoded >> 16) & 0xFF,
            (encoded >> 24) & 0xFF,
        ]))
        return self._send_command_wait_ack(
            command=CMD_SET_PHASE,
            payload=payload,
            timeout=timeout,
        )

    def send_done_and_wait_ack(
        self,
        timeout: Optional[float] = None,
    ) -> Frame:
        """CMD_DONE(0x1E)：LEN=0，PAYLOAD为空。"""
        return self._send_command_wait_ack(
            command=CMD_DONE,
            payload=b"",
            timeout=timeout,
        )

    def send_rescan_and_wait_ack(
        self,
        timeout: Optional[float] = None,
    ) -> Frame:
        """CMD_RESCAN(0x1F)：LEN=0，PAYLOAD为空。"""
        return self._send_command_wait_ack(
            command=CMD_RESCAN,
            payload=b"",
            timeout=timeout,
        )

    def parser_diagnostics(self) -> str:
        return (
            "crc_errors={} version_errors={} "
            "length_errors={}".format(
                self.parser.crc_error_count,
                self.parser.version_error_count,
                self.parser.length_error_count,
            )
        )


# ==========================================================================
#  四档选档逻辑 —— 量程切换判断
# ==========================================================================

def classify_range_evidence_without_deadzone(
    judge,
    evidence,
):
    """量程切换的核心判断函数 —— 修复原 RangeJudge.classify 的四个问题：

    1. **有效周期优先**：周期拟合有效时比 waveform_present (白色像素比例)
       有更强的证据力，避免”已有周期却报 NO_SIGNAL”的死区。
    2. **gear 0~2 低点数强制切档**：右侧折返点 ≤2 个时直接投切换票，
       不等周期拟合，因为在短周期档位中只有2个点无法可靠判断等间距。
    3. **取消单向迟滞死区**：单向选档中不需要电流滞回，直接比较阈值。
    4. **禁止两点零RMS假锁定**：两点总能完美拟合零RMS，不能作为锁定证据。

    返回值：
        (“KEEP”, reason)   → 当前档位足够，可锁定
        (“SWITCH”, reason) → 应切到下一档(更长观察时间)
        (“UNKNOWN”, reason) → 证据不足，继续采集
    """
    tip_count = len(evidence.tips)

    # gear 0~2 且只有 ≤2 个折返点 → 强制切档
    if judge.current_range < 3 and tip_count <= 2:
        return "SWITCH", "TIPS_TOO_FEW_FOR_RELIABLE_PERIOD"

    # 周期拟合有效 → 最高优先级证据
    # detect_tips可能在稀疏波形上错误地将waveform_present判为False,
    # 但只要周期拟合有效就具备更强的信号存在证据。
    if evidence.period_valid:
        if judge.current_range == 3:
            return "KEEP", "LAST_RANGE_VALID_PERIOD"

        # 检查周期是否超过当前档位的切换阈值
        threshold = judge.cfg.switch_period_div[
            judge.current_range
        ]
        if evidence.period_div > threshold:
            return "SWITCH", "PERIOD_TOO_LONG"
        return "KEEP", "PERIOD_SAFE"

    # gear=3 (最长观察窗) 的降级判断
    if judge.current_range == 3:
        if tip_count >= 2:
            return "KEEP", "LAST_RANGE_TIPS_PRESENT"
        if evidence.waveform_present:
            return "KEEP", "LAST_RANGE_WAVEFORM_PRESENT"
        return "UNKNOWN", "LAST_RANGE_NO_SIGNAL"

    # 3个折返点但无法形成可靠等间距周期 → 保守切到更长档位
    if tip_count == 3:
        return "SWITCH", "THREE_TIPS_INVALID_PERIOD_SWITCH"

    if not evidence.waveform_present:
        return "UNKNOWN", "NO_SIGNAL_WITH_INVALID_PERIOD"

    return "UNKNOWN", "PERIOD_UNCERTAIN"


# Monkey-patch: 用无死区版本的 classify 替换 switch_core 内置方法,
# 保持整个项目仍为单文件架构。
# 在创建 FrequencyController 之前就已生效。
switch_core.RangeJudge.classify = (
    classify_range_evidence_without_deadzone
)


# ==========================================================================
#  四档量程配置与默认参数
# ==========================================================================

# 三个选档交界点：28 kHz、10 kHz、2.8 kHz。
# 交界点刻意放在相邻两档都能可靠测量的重叠区。
# 测量范围额外留少量重叠，防止交界处受视觉抖动影响而越界。
SELECTION_BOUNDARY_HZ = {
    0: 28000.0,   # gear 0↔1 交界
    1: 10000.0,   # gear 1↔2 交界
    2: 2800.0,    # gear 2↔3 交界
}

# 各档位的有效测量频段 (带重叠余量)
MEASUREMENT_BANDS_HZ = {
    0: (27500.0, 100000.0),   # gear 0: ~27.5k ~ 100 kHz
    1: (9500.0, 28500.0),     # gear 1: ~9.5k ~ 28.5 kHz
    2: (2700.0, 10500.0),     # gear 2: ~2.7k ~ 10.5 kHz
    3: (1000.0, 2900.0),      # gear 3: 1k ~ 2.9 kHz
}


# 默认四档参数 —— 当无 range_profiles.json 时使用
# effective_time_us: FPGA锯齿波的有效扫描时间 (决定视觉观察窗)
# k_hz_div: 标定系数, 频率 = k_hz_div / 周期(div)
# switch_threshold_div: 超过此周期(div)则切到下一档
DEFAULT_PROFILES = [
    RangeProfile(
        gear=0,
        effective_time_us=100,       # 0.1 ms 锯齿 → 适合 60~100 kHz
        frequency_min_hz=27500.0,
        frequency_max_hz=100000.0,
        k_hz_div=80000.0,
        switch_threshold_div=(80000.0 / 28000.0),  # ≈2.857 div
    ),
    RangeProfile(
        gear=1,
        effective_time_us=400,       # 0.4 ms 锯齿 → 适合 20~60 kHz
        frequency_min_hz=9500.0,
        frequency_max_hz=28500.0,
        k_hz_div=20000.0,
        switch_threshold_div=2.0,    # 2.0 div
    ),
    RangeProfile(
        gear=2,
        effective_time_us=1000,      # 1.0 ms 锯齿 → 适合 5~25 kHz
        frequency_min_hz=2700.0,
        frequency_max_hz=10500.0,
        k_hz_div=8000.0,
        switch_threshold_div=(8000.0 / 2800.0),  # ≈2.857 div
    ),
    RangeProfile(
        gear=3,
        effective_time_us=4000,      # 4.0 ms 锯齿 → 适合 1~10 kHz
        frequency_min_hz=1000.0,
        frequency_max_hz=2900.0,
        k_hz_div=2000.0,
        switch_threshold_div=None,   # 最高档, 无上一档可切
    ),
]


def load_profiles(
    path: Optional[Path],
) -> Dict[int, RangeProfile]:
    """加载四档量程参数 (JSON文件 或 内置默认值)。

    无论JSON来源如何，统一应用 V2 安全测量频段：
    - 保留 JSON 中的 effective_time_us 和 k_hz_div (标定值)
    - 用 MEASUREMENT_BANDS_HZ 替换逻辑量程
    - 用 SELECTION_BOUNDARY_HZ 重新计算切档阈值

    确保各档之间有充足余量，避免频率位于交界处时在两档之间抖动。
    """
    if path is None:
        profiles = DEFAULT_PROFILES
    else:
        data = json.loads(
            path.read_text(encoding="utf-8")
        )
        profiles = []

        for item in data["profiles"]:
            threshold = item.get(
                "switch_threshold_div"
            )

            profiles.append(RangeProfile(
                gear=int(item["gear"]),
                effective_time_us=int(
                    item["effective_time_us"]
                ),
                frequency_min_hz=float(
                    item["frequency_min_hz"]
                ),
                frequency_max_hz=float(
                    item["frequency_max_hz"]
                ),
                k_hz_div=float(
                    item["k_hz_div"]
                ),
                switch_threshold_div=(
                    None
                    if threshold is None
                    else float(threshold)
                ),
            ))

    loaded = {
        profile.gear: profile
        for profile in profiles
    }

    if set(loaded) != {0, 1, 2, 3}:
        raise ValueError(
            "range_profiles必须包含gear 0、1、2、3"
        )

    for gear, profile in loaded.items():
        if (
            profile.effective_time_us <= 0
            or profile.frequency_min_hz <= 0
            or profile.frequency_max_hz
            <= profile.frequency_min_hz
            or profile.k_hz_div <= 0
        ):
            raise ValueError(
                "gear {}参数无效".format(gear)
            )

    # 即使Jetson目录里还存在旧range_profiles.json，也统一采用V2安全频段。
    # 保留JSON中的有效时间和K标定值，仅替换逻辑量程与切档阈值。
    output = {}

    for gear in range(4):
        source = loaded[gear]
        minimum_hz, maximum_hz = (
            MEASUREMENT_BANDS_HZ[gear]
        )
        threshold = (
            None
            if gear == 3
            else source.k_hz_div
            / SELECTION_BOUNDARY_HZ[gear]
        )
        output[gear] = RangeProfile(
            gear=gear,
            effective_time_us=(
                source.effective_time_us
            ),
            frequency_min_hz=minimum_hz,
            frequency_max_hz=maximum_hz,
            k_hz_div=source.k_hz_div,
            switch_threshold_div=threshold,
        )

    return output


def audit_full_frequency_grid(
    profiles: Dict[int, RangeProfile],
) -> Dict[int, int]:
    """静态审计：验证 1~100 kHz 全部 991 个 100 Hz 频点的理论选档。

    对每个频点模拟选档逻辑 (从 gear 0 开始按 switch_threshold_div 比较)，
    检查：
    1. 最终选中的档位是否覆盖该频点 (在 measurement band 内)
    2. 周期是否在 switch_core 的搜索范围内 (0.55~4.60 div)

    不通过则立即抛出 ValueError，防止"存在某频点无档可用"的配置错误。
    """
    selected_counts = {
        0: 0,
        1: 0,
        2: 0,
        3: 0,
    }
    failures = []

    for frequency_hz in range(
        1000,
        100001,
        100,
    ):
        gear = 0

        while gear < 3:
            profile = profiles[gear]
            period_div = (
                profile.k_hz_div
                / frequency_hz
            )

            if (
                period_div
                <= profile.switch_threshold_div
            ):
                break

            gear += 1

        profile = profiles[gear]
        period_div = (
            profile.k_hz_div
            / frequency_hz
        )
        selected_counts[gear] += 1

        if not (
            profile.frequency_min_hz
            <= frequency_hz
            <= profile.frequency_max_hz
        ):
            failures.append((
                frequency_hz,
                gear,
                "OUT_OF_MEASUREMENT_BAND",
            ))

        if not 0.55 <= period_div <= 4.60:
            failures.append((
                frequency_hz,
                gear,
                "PERIOD_OUT_OF_SWITCH_SEARCH",
            ))

    if failures:
        raise ValueError(
            "全频段静态审计失败：{}".format(
                failures[:8]
            )
        )

    return selected_counts


def frame_to_gray(
    frame: np.ndarray,
) -> np.ndarray:
    return measure_core.frame_to_gray(frame)


# ==========================================================================
#  图形检测与相位提取 —— 对角线 / 圆 / ∞形 的视觉判定
# ==========================================================================

def detect_degenerate_diagonal_line(waveform_mask, pixels_per_div):
    """检测李萨如波形是否退化为斜直线 (对角线)。

    用于相位补偿流程：当相位拟合 (双分支方法) 失败时，
    用逐行中位点做鲁棒直线拟合来判定是否已形成斜线。
    同时检查：覆盖率、X跨度、斜率范围、RMS残差、线宽 (排除胖椭圆误判)。

    Returns: (passed: bool, metrics: dict)
    """
    mask = np.asarray(waveform_mask) > 0
    height, width = mask.shape
    rows = []
    centers = []
    row_widths = []
    for y in range(height):
        xs = np.flatnonzero(mask[y])
        if xs.size >= 3:
            rows.append(float(y))
            centers.append(float(np.median(xs)))
            row_widths.append(float(xs[-1] - xs[0] + 1))

    metrics = {
        "coverage": 0.0,
        "x_span": 0.0,
        "slope": float("nan"),
        "rms_div": float("inf"),
        "width_p75_div": float("inf"),
        "inliers": 0,
    }
    if len(rows) < max(20, int(round(0.35 * height))):
        return False, metrics

    yy = np.asarray(rows, dtype=np.float64)
    xx = np.asarray(centers, dtype=np.float64)
    widths = np.asarray(row_widths, dtype=np.float64)
    keep = np.ones(len(yy), dtype=bool)
    residual_limit = max(4.0, 0.18 * float(pixels_per_div))
    for _ in range(2):
        if int(np.count_nonzero(keep)) < 20:
            return False, metrics
        slope, intercept = np.polyfit(yy[keep], xx[keep], 1)
        residual = xx - (slope * yy + intercept)
        keep = np.abs(residual) <= residual_limit

    used_y = yy[keep]
    used_x = xx[keep]
    if used_y.size < 20:
        return False, metrics
    slope, intercept = np.polyfit(used_y, used_x, 1)
    residual = used_x - (slope * used_y + intercept)
    coverage = (float(np.max(used_y) - np.min(used_y)) + 1.0) / height
    x_span = (float(np.max(used_x) - np.min(used_x)) + 1.0) / width
    rms_div = float(np.sqrt(np.mean(residual * residual))) / max(
        float(pixels_per_div), 1.0
    )
    width_p75_div = float(np.percentile(widths[keep], 75.0)) / max(
        float(pixels_per_div), 1.0
    )
    metrics.update({
        "coverage": coverage,
        "x_span": x_span,
        "slope": float(slope),
        "rms_div": rms_div,
        "width_p75_div": width_p75_div,
        "inliers": int(used_y.size),
    })
    passed = bool(
        coverage >= 0.65
        # FPGA现在以0度起始，圆形流程的初始画面也是斜直线。
        # 实际直线受示波器幅度/视野影响可能只覆盖约40%宽度；
        # 仍由覆盖率、斜率和RMS共同约束，避免宽松误判。
        and x_span >= 0.38
        and 0.45 <= abs(float(slope)) <= 2.20
        and rms_div <= 0.12
        # 不能只看逐行中位点；胖椭圆的中位点也可能成线。
        # 75%有效行的实际波形宽度必须是细直线量级。
        and width_p75_div <= 0.45
    )
    return passed, metrics


def detect_degenerate_circle(waveform_mask, pixels_per_div):
    """常规相位拟合失败后的90度圆形兜底检测。

    用 cv2.fitEllipse 做迭代 RANSAC-like 椭圆拟合，
    评估：中心偏移、轴比(是否为近圆形)、直径范围、径向RMS、角度覆盖率。

    仅在常规双分支相位拟合无法给出有效结果时被调用。
    Returns: (passed: bool, metrics: dict)
    """
    mask = np.asarray(waveform_mask) > 0
    height, width = mask.shape
    yy, xx = np.nonzero(mask)
    metrics = {
        "points": int(xx.size),
        "center_offset_div": float("inf"),
        "axis_ratio": 0.0,
        "diameter_x_div": 0.0,
        "diameter_y_div": 0.0,
        "radial_rms": float("inf"),
        "angular_coverage": 0.0,
    }
    if xx.size < 200:
        return False, metrics

    points = np.column_stack((xx, yy)).astype(np.float32)
    keep = np.ones(points.shape[0], dtype=bool)
    ellipse = None
    radial_error = None
    angle = None
    for _ in range(3):
        if int(np.count_nonzero(keep)) < 100:
            return False, metrics
        ellipse = cv2.fitEllipse(points[keep].reshape(-1, 1, 2))
        (cx, cy), (diameter_a, diameter_b), rotation_deg = ellipse
        if diameter_a <= 1.0 or diameter_b <= 1.0:
            return False, metrics
        theta = math.radians(rotation_deg)
        cosine = math.cos(theta)
        sine = math.sin(theta)
        dx = points[:, 0] - cx
        dy = points[:, 1] - cy
        local_x = cosine * dx + sine * dy
        local_y = -sine * dx + cosine * dy
        norm_x = local_x / (0.5 * diameter_a)
        norm_y = local_y / (0.5 * diameter_b)
        radius = np.sqrt(norm_x * norm_x + norm_y * norm_y)
        radial_error = np.abs(radius - 1.0)
        angle = np.arctan2(norm_y, norm_x)
        new_keep = radial_error <= 0.18
        if np.array_equal(new_keep, keep):
            break
        keep = new_keep

    if ellipse is None or int(np.count_nonzero(keep)) < 100:
        return False, metrics
    (cx, cy), (diameter_a, diameter_b), _ = ellipse
    major = max(float(diameter_a), float(diameter_b))
    minor = min(float(diameter_a), float(diameter_b))
    center_offset_div = math.hypot(
        cx - 0.5 * (width - 1),
        cy - 0.5 * (height - 1),
    ) / max(float(pixels_per_div), 1.0)
    axis_ratio = minor / max(major, 1.0)
    radial_rms = float(np.sqrt(np.mean(radial_error[keep] ** 2)))
    bins = np.floor((angle[keep] + math.pi) * 36.0 / (2.0 * math.pi))
    bins = np.clip(bins.astype(np.int32), 0, 35)
    angular_coverage = float(np.unique(bins).size) / 36.0
    metrics.update({
        "points": int(np.count_nonzero(keep)),
        "center_offset_div": center_offset_div,
        "axis_ratio": axis_ratio,
        "diameter_x_div": float(diameter_a) / pixels_per_div,
        "diameter_y_div": float(diameter_b) / pixels_per_div,
        "radial_rms": radial_rms,
        "angular_coverage": angular_coverage,
    })
    passed = bool(
        center_offset_div <= 0.45
        and axis_ratio >= 0.85
        and minor / pixels_per_div >= 6.0
        and major / pixels_per_div <= 8.6
        and radial_rms <= 0.10
        and angular_coverage >= 0.55
    )
    return passed, metrics


def detect_hough_circle(waveform_mask, pixels_per_div):
    """用霍夫圆检测最终圆形 —— 相位补偿完成后的最终验证。

    将 mask 缩放到约 64 px/div (降低 Jetson 计算量),
    用 cv2.HoughCircles 检测候选圆，
    然后按圆周采样点逐一验证支持率(允许一定线宽和局部断裂)。

    综合评分 = 圆周支持率 - 0.20 × 中心偏移，
    排除示波器网格被误检为圆形的情况。
    Returns: (passed: bool, metrics: dict)
    """
    mask = (np.asarray(waveform_mask) > 0).astype(np.uint8) * 255
    height, width = mask.shape
    ppd = max(float(pixels_per_div), 1.0)
    metrics = {
        "found": 0,
        "center_offset_div": float("inf"),
        "radius_div": 0.0,
        "support": 0.0,
    }
    if cv2.countNonZero(mask) < 200:
        return False, metrics

    # 将检测分辨率固定在约64 px/div，降低Jetson上的计算量。
    scale = min(1.0, 64.0 / ppd)
    work = cv2.resize(
        mask,
        None,
        fx=scale,
        fy=scale,
        interpolation=cv2.INTER_AREA,
    )
    work_ppd = ppd * scale
    work = cv2.GaussianBlur(work, (7, 7), 1.5)
    circles = cv2.HoughCircles(
        work,
        cv2.HOUGH_GRADIENT,
        dp=1.2,
        minDist=max(20.0, 2.0 * work_ppd),
        param1=100.0,
        param2=22.0,
        minRadius=max(5, int(round(2.5 * work_ppd))),
        maxRadius=max(6, int(round(4.3 * work_ppd))),
    )
    if circles is None:
        return False, metrics

    center_x = 0.5 * (work.shape[1] - 1)
    center_y = 0.5 * (work.shape[0] - 1)
    binary = work >= 40
    best = None
    for cx, cy, radius in circles[0]:
        center_offset_div = math.hypot(
            float(cx) - center_x,
            float(cy) - center_y,
        ) / work_ppd
        angles = np.linspace(0.0, 2.0 * math.pi, 144, endpoint=False)
        supported = np.zeros(len(angles), dtype=bool)
        # 允许圆环有一定线宽和局部断裂。
        for radial_delta in np.linspace(-0.14, 0.14, 5) * work_ppd:
            sample_radius = float(radius) + float(radial_delta)
            xs = np.rint(cx + sample_radius * np.cos(angles)).astype(np.int32)
            ys = np.rint(cy + sample_radius * np.sin(angles)).astype(np.int32)
            valid = (
                (xs >= 0) & (xs < work.shape[1])
                & (ys >= 0) & (ys < work.shape[0])
            )
            hit = np.zeros(len(angles), dtype=bool)
            hit[valid] = binary[ys[valid], xs[valid]]
            supported |= hit
        support = float(np.mean(supported))
        score = support - 0.20 * center_offset_div
        candidate = (score, center_offset_div, float(radius) / work_ppd, support)
        if best is None or candidate[0] > best[0]:
            best = candidate

    _, center_offset_div, radius_div, support = best
    metrics.update({
        "found": int(len(circles[0])),
        "center_offset_div": center_offset_div,
        "radius_div": radius_div,
        "support": support,
    })
    passed = bool(
        center_offset_div <= 0.55
        and 2.5 <= radius_div <= 4.3
        and support >= 0.58
    )
    return passed, metrics


def measure_infinity_phase_absolute(waveform_mask, pixels_per_div):
    """由∞形(1:2李萨如图)在x=0处的交叉高度估算绝对相位。

    原理：1:2 李萨如图 x = sin(θ), y = sin(2θ + φ)，
    当 x = 0 (θ = 0 或 π) 时，y = sin(φ)。
    因此中心垂直交叉点的高度直接编码了相位差 φ。

    取 x = center_x ± half_band 范围内的垂直交叉点中位值,
    然后用 arcsin(y_offset / 4div_amplitude) 反算相位。
    Returns: (phase_deg, crossing_y, evidence_count)
    """
    mask = np.asarray(waveform_mask) > 0
    height, width = mask.shape
    center_x = 0.5 * (width - 1)
    center_y = 0.5 * (height - 1)
    half_band = max(3, int(round(0.16 * float(pixels_per_div))))
    x0 = max(0, int(round(center_x)) - half_band)
    x1 = min(width, int(round(center_x)) + half_band + 1)

    row_centers = []
    for x in range(x0, x1):
        ys = np.flatnonzero(mask[:, x])
        if ys.size >= 2:
            row_centers.append(float(np.median(ys)))
    if len(row_centers) < max(5, half_band // 2):
        raise RuntimeError("INFINITY_CENTER_EVIDENCE_TOO_FEW")

    crossing_y = float(np.median(np.asarray(row_centers)))
    amplitude_y = max(1.0, 4.0 * float(pixels_per_div))
    sin_value = float(np.clip(
        abs(crossing_y - center_y) / amplitude_y,
        0.0,
        1.0,
    ))
    phase_deg = float(math.degrees(math.asin(sin_value)))
    return phase_deg, crossing_y, len(row_centers)


def open_camera(args) -> cv2.VideoCapture:
    if args.gst_pipeline:
        capture = cv2.VideoCapture(
            args.gst_pipeline,
            cv2.CAP_GSTREAMER,
        )
    else:
        backend = (
            cv2.CAP_V4L2
            if hasattr(cv2, "CAP_V4L2")
            else cv2.CAP_ANY
        )
        capture = cv2.VideoCapture(
            args.camera,
            backend,
        )
        capture.set(
            cv2.CAP_PROP_FRAME_WIDTH,
            args.width,
        )
        capture.set(
            cv2.CAP_PROP_FRAME_HEIGHT,
            args.height,
        )
        capture.set(
            cv2.CAP_PROP_FPS,
            args.fps,
        )
        capture.set(
            cv2.CAP_PROP_BUFFERSIZE,
            1,
        )

        if args.fourcc:
            capture.set(
                cv2.CAP_PROP_FOURCC,
                cv2.VideoWriter_fourcc(
                    *args.fourcc[:4]
                ),
            )

    if not capture.isOpened():
        raise RuntimeError("无法打开摄像头")

    return capture


def read_frame(
    capture: cv2.VideoCapture,
) -> np.ndarray:
    ok, frame = capture.read()

    if not ok or frame is None:
        raise RuntimeError("摄像头读取失败")

    return frame


def discard_frames(
    capture: cv2.VideoCapture,
    count: int,
) -> None:
    for _ in range(max(0, count)):
        capture.grab()


HARMONIC_RATIOS = (
    0.2,
    0.25,
    1.0 / 3.0,
    0.4,
    0.5,
    1.0,
    2.0,
    2.5,
    3.0,
    4.0,
    5.0,
)


def _tip_count_consistency(
    frequency_hz: float,
    tip_count: int,
    cfg,
) -> float:
    """频率对应的理论折返点数量与实际tips数量的归一化误差。"""
    if frequency_hz <= 0 or tip_count < 1:
        return float("inf")

    period_div = cfg.period_from_frequency(
        frequency_hz
    )
    expected_tips = 8.0 / period_div

    # 相位不同会使可见数量落在floor(expected)或ceil(expected)。
    # 超出这个整数区间才是真正的数量矛盾。
    lower_count = math.floor(expected_tips)
    upper_count = math.ceil(expected_tips)

    if lower_count <= tip_count <= upper_count:
        count_error = 0.0
    else:
        count_error = min(
            abs(float(tip_count) - lower_count),
            abs(float(tip_count) - upper_count),
        )

    return count_error / max(
        expected_tips,
        2.0,
    )


def normalize_harmonic_result(
    result,
    cfg,
):
    """谐波倍率消歧 —— 利用折返点数量消除倍周期误锁。

    李萨如图形中，周期测量可能误锁到真实频率的 1/2、1/3、2× 等谐波。
    但”8格高度内应出现多少个右侧折返点”这一先验可用来区分候选频率。

    典型场景：
    - 29 kHz 误测为 14.5 kHz → 实际折返点数量更符合 29 kHz
    - 3 kHz 误测为 1.5 kHz → 实际折返点数量更符合 3 kHz
    - 真实高频被误测为 2× 周期 → tips数量会支持较低的真实频率

    关键设计：
    - 必须同时比较原始检测点和拟合恢复点的数量 (取点数多的一组),
      避免”拟合误锁到某周期后只保留隔点 → 再次支持错误周期”的循环证据。
    - 只在视觉数量证据**明显更强** (consistency + harmonic_penalty
      优于当前值 ≥ 0.12) 时才修正，避免无条件偏向高次/低次谐波。
    - 局部观测 (tip_span_ratio < 0.60) 不进行谐波修正。
    """
    result.original_frequency_hz = float(
        result.frequency_hz
        if np.isfinite(result.frequency_hz)
        else float("nan")
    )
    result.harmonic_ratio = 1.0
    result.harmonic_corrected = False

    fit = result.fit
    if not getattr(fit, "ok", False):
        return result

    # 谐波消歧必须同时比较原始检测点和拟合恢复点。只使用fit.used_y_px
    # 会形成循环证据：一旦拟合误锁到2倍/2.5倍周期，它只保留隔点，
    # 后续tips计数便会反过来支持错误周期。两组不能直接相加，否则
    # 错误拟合位置会变成虚假新tips；选点数更多的一组，既保住88kHz
    # 等高频恢复点，也保住低频错误拟合丢弃的原始点。
    fitted_y = np.asarray(
        getattr(fit, "used_y_px", np.empty(0)),
        dtype=np.float64,
    )
    detected_y = np.asarray(
        getattr(result.detection, "y_px", np.empty(0)),
        dtype=np.float64,
    )
    fitted_y = np.sort(fitted_y[np.isfinite(fitted_y)])
    detected_y = np.sort(detected_y[np.isfinite(detected_y)])

    def evidence_rank(values):
        span = (
            float(values[-1] - values[0])
            if values.size >= 2 else 0.0
        )
        return int(values.size), span

    evidence_y = max(
        (fitted_y, detected_y),
        key=evidence_rank,
    )
    if evidence_y.size >= 2:
        tip_count = int(evidence_y.size)
        tip_span_ratio = (
            float(np.max(evidence_y) - np.min(evidence_y))
            / max(float(cfg.height - 1), 1.0)
        )
    else:
        tip_count = int(evidence_y.size)
        tip_span_ratio = (
            float(np.max(evidence_y) - np.min(evidence_y))
            / max(float(cfg.height - 1), 1.0)
            if evidence_y.size >= 2 else 0.0
        )
    if tip_count < 2:
        return result

    # 只看到顶部或底部局部几个点时，点间距仍能准确测周期，
    # 但“8格内应有多少点”已不再是有效证据。此时不做谐波
    # 倍率修正，避免把稳定的0.9 div周期翻成1.8 div。
    if tip_span_ratio < 0.60:
        result.fit.reason = "LOCAL_SPAN_TRUST_RAW_PERIOD"
        return result

    if (
        np.isfinite(result.measured_period_div)
        and result.measured_period_div > 0
    ):
        visual_period_div = float(
            result.measured_period_div
        )
    elif (
        np.isfinite(fit.period_px)
        and fit.period_px > 0
    ):
        visual_period_div = float(
            fit.period_px
            / cfg.pixels_per_div
        )
    else:
        return result

    raw_frequency_hz = float(
        cfg.frequency_from_period(
            visual_period_div
        )
    )

    candidates = []

    def add_candidate(
        frequency_hz: float,
        ratio: float,
        source: str,
    ) -> None:
        if not (
            np.isfinite(frequency_hz)
            and cfg.frequency_min_hz
            <= frequency_hz
            <= cfg.frequency_max_hz
        ):
            return

        if any(
            abs(frequency_hz - old[0]) < 1.0
            for old in candidates
        ):
            return

        consistency = _tip_count_consistency(
            frequency_hz,
            tip_count,
            cfg,
        )
        harmonic_penalty = (
            0.035
            * abs(math.log(max(ratio, 1e-9), 2.0))
        )
        candidates.append((
            float(frequency_hz),
            float(ratio),
            float(consistency + harmonic_penalty),
            float(consistency),
            source,
        ))

    if result.ok and np.isfinite(
        result.frequency_hz
    ):
        displayed_ratio = (
            float(result.frequency_hz)
            / raw_frequency_hz
        )
        add_candidate(
            float(result.frequency_hz),
            displayed_ratio,
            "MEASURED",
        )

    for ratio in HARMONIC_RATIOS:
        add_candidate(
            raw_frequency_hz * ratio,
            ratio,
            "HARMONIC",
        )

    if not candidates:
        return result

    best = min(
        candidates,
        key=lambda item: item[2],
    )
    current = None

    if result.ok and np.isfinite(
        result.frequency_hz
    ):
        current = min(
            candidates,
            key=lambda item: abs(
                item[0]
                - float(result.frequency_hz)
            ),
        )

    should_correct = False

    if current is None:
        # 原测量因越界失败，但谐波映射后与tips数量高度一致。
        should_correct = best[3] <= 0.18
    elif (
        abs(best[0] - current[0]) >= 50.0
        and best[2] + 0.12 < current[2]
        and best[3] <= 0.22
    ):
        should_correct = True

    if should_correct:
        corrected_frequency_hz = float(best[0])
        corrected_period_div = float(
            cfg.period_from_frequency(
                corrected_frequency_hz
            )
        )
        result.ok = True
        result.frequency_hz = corrected_frequency_hz
        result.period_div = corrected_period_div
        result.raw_frequency_hz = raw_frequency_hz
        result.boundary_rescued = False
        result.harmonic_ratio = float(best[1])
        result.harmonic_corrected = True
        result.fit.reason = (
            "HARMONIC_CORRECTED_X{:.3f}".format(
                best[1]
            )
        )
        return result

    # 边界救援值若连tips数量都解释不了，不能让它凭重复出现支配融合。
    if (
        result.ok
        and result.boundary_rescued
        and current is not None
        and current[3] > 0.25
    ):
        result.ok = False
        result.fit.reason = (
            "BOUNDARY_RESCUE_TIP_MISMATCH"
        )

    return result


# ==========================================================================
#  TwoFrameLissajousJudge —— 精扫判图器 (两帧稳定窗口)
# ==========================================================================

class TwoFrameLissajousJudge(lissajous_core.FastLissajousJudge):
    """继承 FastLissajousJudge，增加两帧稳定性检查和方块过滤。

    在两帧稳定窗口中：
    1. 连续两帧的 global_white_ratio 差值 ≤ 2% → 稳定
    2. 稳定后两帧均需通过判图 → 判定为 CORRECT
    3. 任意帧的白色占比 > BLOCK_WHITE_RATIO_LIMIT → 判为方块/噪声

    稳定性超时 (0.60s) 后即使不稳定也返回失败。
    """

    STABILITY_RANGE = 0.02          # 两帧白色占比差值阈值 (2%)
    STABILITY_TIMEOUT_SECONDS = 0.60  # 稳定超时

    def _analyse_frame(self, frame, frame_index):
        result = super()._analyse_frame(frame, frame_index)
        if result.metrics.global_white_ratio > BLOCK_WHITE_RATIO_LIMIT:
            result.metrics.shape_name = "BLOCK"
            result.metrics.verdict = "BLOCK"
            result.metrics.reason = (
                "方块：全图白色像素占比{:.2f}%超过30.00%".format(
                    100.0 * result.metrics.global_white_ratio
                )
            )
            result.shape = "BLOCK"
            result.reason = result.metrics.reason
            result.passed = False
        return result

    def judge_current_frequency(self):
        started = time.perf_counter()
        self.request_index += 1
        request_index = self.request_index

        if self.settle_ms > 0:
            time.sleep(self.settle_ms / 1000.0)
        discard_frames(self.capture, self.discard_count)

        recent = []
        all_results = []
        stable = False
        deadline = time.perf_counter() + self.STABILITY_TIMEOUT_SECONDS
        frame_index = 0

        while time.perf_counter() < deadline:
            frame_index += 1
            frame = self._capture_one_rectified()
            result = self._analyse_frame(frame, frame_index)
            all_results.append(result)
            recent.append(result)
            recent = recent[-2:]
            self._save_debug(request_index, result)

            if len(recent) < 2:
                continue

            ratios = [
                item.metrics.global_white_ratio
                for item in recent
            ]
            ratio_range = max(ratios) - min(ratios)
            print(
                "  [STABILITY] frames={}-{} ratios={} range={:.2f}%".format(
                    frame_index - 1,
                    frame_index,
                    ",".join("{:.2f}%".format(100.0 * value) for value in ratios),
                    100.0 * ratio_range,
                )
            )

            if ratio_range <= self.STABILITY_RANGE:
                stable = True
                break

        results = list(recent)
        passed = sum(int(item.passed) for item in results)
        failed = len(results) - passed
        correct = bool(stable and passed == 2)
        best = min(
            (item.metrics for item in results),
            key=lambda metrics: metrics.global_white_ratio,
        )

        return lissajous_core.JudgeResult(
            correct=correct,
            reason=(
                "THREE_FRAME_STABLE_PASSED"
                if correct else (
                    "THREE_FRAME_STABLE_REJECTED"
                    if stable else
                    "THREE_FRAME_STABILITY_TIMEOUT"
                )
            ),
            passed_votes=passed,
            failed_votes=failed,
            frames_used=len(all_results),
            elapsed_ms=(time.perf_counter() - started) * 1000.0,
            frame_results=results,
            batch_confirmations=1 if correct else 0,
            total_batches=1,
            best_metrics=best,
        )


# ==========================================================================
#  FrequencyController —— 完整自动模式主控制器
# ==========================================================================

class FrequencyController:
    """将 FPGA UART 协议、视觉测量引擎、扫频策略串联起来的顶层状态机。

    生命周期：
        1. __init__: 加载标定、初始化摄像头、创建子模块
        2. run_once(): 四档选档 → 八帧粗测频 → 返回 FinalMeasurement
        3. send_final_frequency(): 将粗测结果发送给 FPGA
        4. run_fine_sweep_session(): 双侧 100 Hz 精扫判图闭环
        5. run_phase_compensation(): 相位补偿到目标图形
        6. send_done(): 通知 FPGA 保持输出、声光提示
        7. wait_for_external_frequency_change(): DONE 后持续监控
        8. send_rescan(): 重新开始下一轮

    关键设计约束：
        - 所有方法都调用 check_abort() 以支持 FPGA START 抢占和 ABORTED 取消
        - 摄像头分辨率变化时自动重建九点映射
        - 异常分类处理：MeasurementNeedsHigherGear → 自动升档不抛异常
    """

    def __init__(
        self,
        args,
        link: Optional[SimpleUartLink],
    ):
        """初始化频率控制器。

        加载配置、创建子模块：
        - profiles: 四档量程参数
        - switch_cfg/switch_analyzer: 选档判断器 (96 px/div, 右半网格)
        - lissajous_cfg/lissajous_judge: 精扫判图器
        - phase_cfg/phase_rectifier: 相位提取器
        """
        self.args = args
        self.link = link
        self.profiles = load_profiles(
            args.profiles
        )
        audit_counts = audit_full_frequency_grid(
            self.profiles
        )
        print(
            "[PROFILE AUDIT] PASS "
            "1～100 kHz/100 Hz, 991 points, "
            "selected={}".format(
                audit_counts
            )
        )
        self.capture = open_camera(args)

        first_frame = None

        for _ in range(
            max(1, args.warmup_frames)
        ):
            first_frame = read_frame(
                self.capture
            )

        first_gray = frame_to_gray(
            first_frame
        )
        self.image_shape = first_gray.shape

        if not args.calibration.exists():
            self.capture.release()
            raise FileNotFoundError(
                "找不到九点标定文件：{}".format(
                    args.calibration
                )
            )

        self.points = (
            measure_core.load_calibration(
                args.calibration,
                self.image_shape,
            )
        )

        self.switch_cfg = switch_core.Config(
            pixels_per_div=(
                args.switch_pixels_per_div
            ),
            right_tip_min_x_div=(
                args.switch_right_tip_min_x
            ),
            switch_period_div=(
                self.profiles[0].switch_threshold_div,
                self.profiles[1].switch_threshold_div,
                self.profiles[2].switch_threshold_div,
            ),
            minimum_period_points=3,
            decision_window=5,
            required_votes=4,
        )

        self.switch_analyzer = (
            switch_core.Analyzer(
                switch_core.RightRectifier(
                    self.points,
                    self.switch_cfg,
                ),
                self.switch_cfg,
            )
        )


        self.lissajous_cfg = lissajous_core.Config(
            mode=args.judge_mode,
            pixels_per_div=args.judge_pixels_per_div,
            window_frames=args.judge_window_frames,
            binary_threshold=args.judge_binary_threshold,
            maximum_duty_ratio=BLOCK_WHITE_RATIO_LIMIT,
        )

        self.lissajous_rectifier = lissajous_core.Rectifier(
            self.points,
            self.lissajous_cfg,
        )
        self.lissajous_judge = TwoFrameLissajousJudge(
            capture=self.capture,
            rectifier=self.lissajous_rectifier,
            cfg=self.lissajous_cfg,
            settle_ms=args.judge_settle_ms,
            discard_count=args.judge_discard_frames,
            debug_dir=args.judge_debug_dir,
        )

        self.phase_cfg = phase_core.Config(
            pixels_per_div=args.phase_pixels_per_div,
            left_x_min_div=0.20,
            left_x_max_div=3.95,
            right_x_min_div=4.05,
            right_x_max_div=7.80,
        )
        self.phase_rectifier = phase_core.Rectifier(
            self.points,
            self.phase_cfg,
        )

        print(
            "[JUDGE GLOBAL WHITE] mode={} rectified={}x{} ppd={} "
            "gray_threshold={} global_white_limit={:.1f}% max_frames={} "
            "required_votes={} settle_ms={:.0f}".format(
                self.lissajous_cfg.mode,
                self.lissajous_cfg.width,
                self.lissajous_cfg.height,
                self.lissajous_cfg.pixels_per_div,
                self.lissajous_cfg.binary_threshold,
                100.0 * self.lissajous_cfg.maximum_duty_ratio,
                self.lissajous_cfg.window_frames,
                self.lissajous_cfg.required_matched_windows,
                args.judge_settle_ms,
            )
        )

    def close(self) -> None:
        self.capture.release()

        if self.args.display:
            cv2.destroyAllWindows()

    def check_abort(self) -> None:
        """轮询 FPGA 的异步事件队列，处理 ABORTED / START 抢占。

        在所有耗时操作 (选档、测频、精扫、调相) 的主循环中被频繁调用，
        保证 FPGA 的取消请求和新 START 事件能在 <50ms 内被响应。

        Raises:
            MeasurementAborted: FPGA 发送 ABORTED 或用户按 Q/Esc
            StartPreempted: FPGA 发送新 START，抢占当前流程
        """
        if self.link is None:
            return

        while True:
            frame = self.link.poll_event()
            if frame is None:
                return

            if (
                isinstance(frame, Frame)
                and frame.command == CMD_ABORTED
            ):
                raise MeasurementAborted("FPGA发送ABORTED")

            if (
                isinstance(frame, Frame)
                and frame.command == CMD_START
            ):
                raise StartPreempted(frame)

            # 过期 SCAN_TEST/SCAN_END 在此被消费，不会污染后续精扫

    def wait_camera_after_gear(self) -> None:
        """切档后的相机稳定等待。

        GEAR_CHANGED ACK 只表示 FPGA 寄存器已切换完成。
        相机内部帧缓冲仍可能包含旧档锯齿的画面，
        因此需要额外等待 camera_settle_ms 并丢弃 camera_discard_frames 帧。
        """
        time.sleep(
            max(
                0.0,
                self.args.camera_settle_ms,
            )
            / 1000.0
        )
        discard_frames(
            self.capture,
            self.args.camera_discard_frames,
        )

    def request_next_gear(
        self,
        next_gear: int,
    ) -> None:
        if self.link is None:
            # 本地测试模式：认为FPGA立即完成切档。
            print(
                "[SIMULATE] GEAR_CHANGED {}".format(
                    next_gear
                )
            )
            self.wait_camera_after_gear()
            return

        print(
            "[UART] SET_GEAR {}".format(
                next_gear
            )
        )

        self.link.set_gear_and_wait_changed(
            gear=next_gear,
            timeout=self.args.gear_changed_timeout,
        )

        print(
            "[UART] GEAR_CHANGED {}".format(
                next_gear
            )
        )

        self.wait_camera_after_gear()

    def select_gear(
        self,
        initial_gear: int,
    ) -> int:
        """四档自动选档主循环 —— 从 initial_gear 开始确定最佳量程。

        流程：
        1. 从 initial_gear 开始，每帧用 switch_core.Analyzer 分析右半画面
        2. RangeJudge 累计 5 帧投票 (需 4/5 票才输出 decision.ready=True)
        3. ready 后：should_switch=True → 升到下一档继续
                     should_switch=False → 锁定当前档位
        4. 40 帧内未锁定 → 超时升档 (gear 3 仍超时则抛异常)

        每次进入新档都重新创建 RangeJudge，确保上一档投票完全清空。
        """

        if gear not in (0, 1, 2, 3):
            raise RuntimeError(
                "START中的initial_gear必须为0～3"
            )

        # FPGA发送START前应已经把输出切到initial_gear。
        self.wait_camera_after_gear()

        while True:
            # 每次进入新档都重新创建RangeJudge，
            # 确保上一档5帧投票完全清空。
            judge = switch_core.RangeJudge(
                self.switch_cfg,
                gear,
            )

            switched_to_next = False

            for frame_index in range(
                self.args.switch_max_frames
            ):
                self.check_abort()

                gray = frame_to_gray(
                    read_frame(self.capture)
                )

                evidence = (
                    self.switch_analyzer.analyze(
                        gray
                    )
                )
                decision = judge.update(
                    evidence
                )

                # Decision.reason在尚未达到4票时会统一显示
                # COLLECTING_OR_UNCERTAIN。额外取得当前帧的真实原因，
                # 方便现场区分无端点、边界切档和周期拟合失败。
                _, frame_reason = judge.classify(
                    evidence
                )
                reason_text = (
                    decision.reason
                    if decision.ready
                    else frame_reason
                )

                period_text = (
                    "{:.5f}".format(
                        decision.period_div
                    )
                    if np.isfinite(
                        decision.period_div
                    )
                    else "invalid"
                )

                print(
                    "[SWITCH {:02d}] gear={} "
                    "state={} votes={}/{} "
                    "tips={} period={} "
                    "ready={} switch={} reason={}".format(
                        frame_index,
                        gear,
                        decision.frame_state,
                        decision.switch_votes,
                        decision.keep_votes,
                        decision.tip_count,
                        period_text,
                        decision.ready,
                        decision.should_switch,
                        reason_text,
                    )
                )

                if self.args.display:
                    rectified = (
                        self.switch_analyzer.rectifier(
                            gray
                        )
                    )
                    shown = cv2.cvtColor(
                        rectified,
                        cv2.COLOR_GRAY2BGR,
                    )
                    cv2.putText(
                        shown,
                        (
                            "gear={} S/K={}/{} "
                            "ready={} switch={}"
                        ).format(
                            gear,
                            decision.switch_votes,
                            decision.keep_votes,
                            decision.ready,
                            decision.should_switch,
                        ),
                        (15, 30),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.62,
                        (0, 255, 255),
                        2,
                        cv2.LINE_AA,
                    )
                    cv2.imshow(
                        "Range selection",
                        cv2.resize(
                            shown,
                            None,
                            fx=0.75,
                            fy=0.75,
                            interpolation=cv2.INTER_AREA,
                        ),
                    )

                    key = cv2.waitKey(1) & 0xFF

                    if key in (
                        27,
                        ord("q"),
                        ord("Q"),
                    ):
                        raise MeasurementAborted(
                            "用户停止"
                        )

                if not decision.ready:
                    continue

                if (
                    decision.should_switch
                    and gear < 3
                ):
                    next_gear = gear + 1
                    self.request_next_gear(
                        next_gear
                    )
                    gear = next_gear
                    switched_to_next = True
                    break

                print(
                    "[GEAR LOCKED] gear={}".format(
                        gear
                    )
                )
                return gear

            if switched_to_next:
                continue

            if gear < 3:
                next_gear = gear + 1
                print(
                    "[GEAR TIMEOUT FALLBACK] gear={} had no reliable "
                    "decision in {} frames; switch to gear={}".format(
                        gear,
                        self.args.switch_max_frames,
                        next_gear,
                    ),
                    flush=True,
                )
                self.request_next_gear(next_gear)
                gear = next_gear
                continue

            raise RuntimeError(
                "量程判断超时：gear={}".format(
                    gear
                )
            )

    def build_measurement_objects(
        self,
        gear: int,
    ):
        profile = self.profiles[gear]

        cfg = measure_core.Config(
            pixels_per_div=(
                self.args.measure_pixels_per_div
            ),
            frequency_min_hz=(
                profile.frequency_min_hz
            ),
            frequency_max_hz=(
                profile.frequency_max_hz
            ),
            calibration_model="scalar",
            k_hz_div=profile.k_hz_div,
            frequency_offset_hz=0.0,
            enable_boundary_rescue=True,
            right_tip_min_x_div=(
                self.args.measure_right_tip_min_x
            ),
        )

        rectifier = measure_core.Rectifier(
            self.points,
            cfg,
        )

        fusion = measure_core.MultiFrameFusion(
            cfg=cfg,
            batch_size=8,
            method="median",
            frequency_step_hz=100.0,
            minimum_inliers=5,
            cluster_absolute_hz=(
                self.args.cluster_absolute_hz
            ),
            cluster_relative_ratio=(
                self.args.cluster_relative_ratio
            ),
            mad_scale=3.5,
            mad_floor_hz=100.0,
            lost_reset_frames=8,
        )

        return profile, cfg, rectifier, fusion

    def measure_frequency(
        self,
        gear: int,
    ) -> FinalMeasurement:
        """八帧粗测频主循环 —— 在当前档位下采集并融合出 100 Hz 量化的频率。

        流程：
        1. 根据档位参数创建 measure_core.Config 和 Rectifier (160 px/div)
        2. 创建 MultiFrameFusion (8帧中值融合, 最少5帧内点)
        3. 逐帧: 九点校正 → 三档二值化 → 折返点检测 → 格点拟合 →
                谐波消歧 → 两点过滤 → 送入融合器
        4. 融合器完成一批 (stable.updated=True) → 返回 FinalMeasurement
        5. 120 帧未收敛 → gear<3 抛 MeasurementNeedsHigherGear;
                              gear=3 抛 RuntimeError

        Returns:
            FinalMeasurement: 包含频率码 (100Hz步进), 周期, 帧统计
        """

        print(
            "[MEASURE CONFIG] gear={} "
            "effective={} us "
            "range={:.1f}～{:.1f} kHz "
            "K={:.3f}".format(
                gear,
                profile.effective_time_us,
                profile.frequency_min_hz
                / 1000.0,
                profile.frequency_max_hz
                / 1000.0,
                profile.k_hz_div,
            )
        )

        discard_frames(
            self.capture,
            self.args.measure_discard_frames,
        )

        for frame_index in range(
            self.args.measure_max_frames
        ):
            self.check_abort()

            gray = frame_to_gray(
                read_frame(self.capture)
            )

            if gray.shape != self.image_shape:
                raise RuntimeError(
                    "测量过程中摄像头分辨率改变"
                )

            result = measure_core.measure(
                gray,
                rectifier,
                cfg,
                already_rectified=False,
            )
            result = normalize_harmonic_result(
                result,
                cfg,
            )
            if (
                result.ok
                and len(result.fit.used_y_px) < 3
            ):
                # 两点总能得到RMS=0，无法证明周期真实。
                # 不让这类弱证据进入八帧融合。
                result.ok = False
                result.fit.ok = False
                result.fit.reason = "FIT_POINTS_LESS_THAN_3"
            stable = fusion.update(result)

            if result.ok:
                harmonic_text = (
                    " corrected=x{:.3f}".format(
                        result.harmonic_ratio
                    )
                    if result.harmonic_corrected
                    else ""
                )
                print(
                    "[MEASURE {:03d}] "
                    "single={:.2f} Hz "
                    "period={:.7f} div "
                    "rms={:.5f} "
                    "tips={} valid={}/8{}".format(
                        frame_index,
                        result.frequency_hz,
                        result.period_div,
                        result.rms_div,
                        "{}/{}".format(
                            len(result.detection.y_px),
                            len(result.fit.used_y_px),
                        ),
                        stable.sample_count,
                        harmonic_text,
                    )
                )
            else:
                print(
                    "[MEASURE {:03d}] "
                    "failed reason={}".format(
                        frame_index,
                        result.fit.reason,
                    )
                )

            if self.args.display:
                shown = (
                    measure_core.annotate_live(
                        result,
                        stable,
                        cfg,
                        frame_index,
                        0.0,
                    )
                )
                cv2.imshow(
                    "Frequency measurement",
                    cv2.resize(
                        shown,
                        None,
                        fx=0.70,
                        fy=0.70,
                        interpolation=cv2.INTER_AREA,
                    ),
                )

                key = cv2.waitKey(1) & 0xFF

                if key in (
                    27,
                    ord("q"),
                    ord("Q"),
                ):
                    raise MeasurementAborted(
                        "用户停止"
                    )

            if not stable.updated:
                continue

            # MultiFrameFusion已经按100 Hz步进量化。
            frequency_hz = float(
                stable.frequency_hz
            )
            frequency_code = int(round(
                frequency_hz / 100.0
            ))

            if not 0 <= frequency_code <= 0xFFFF:
                raise RuntimeError(
                    "频率编码超出u16"
                )

            final = FinalMeasurement(
                gear=gear,
                frequency_hz=frequency_hz,
                frequency_code=frequency_code,
                period_div=float(
                    stable.period_div
                ),
                used_frames=int(
                    stable.accepted_count
                ),
                rejected_frames=int(
                    stable.rejected_count
                ),
            )

            print(
                "[FINAL] gear={} "
                "frequency={:.0f} Hz "
                "code={} "
                "period={:.7f} div "
                "used={} rejected={}".format(
                    final.gear,
                    final.frequency_hz,
                    final.frequency_code,
                    final.period_div,
                    final.used_frames,
                    final.rejected_frames,
                )
            )

            return final

        if gear < 3:
            raise MeasurementNeedsHigherGear(
                "gear={}八帧测频不收敛".format(gear)
            )
        raise RuntimeError("gear=3八帧测频超时")

    def run_once(
        self,
        initial_gear: int,
    ) -> FinalMeasurement:
        """执行一次完整的选档+粗测频流程。

        select_gear 选档 → measure_frequency 测频。
        若测频无法在当前档位收敛 (MeasurementNeedsHigherGear)，
        自动升档重试 (最多到 gear 3)。
        """
        gear = self.select_gear(initial_gear)
        while True:
            try:
                return self.measure_frequency(gear)
            except MeasurementNeedsHigherGear as error:
                next_gear = gear + 1
                print(
                    "[MEASURE AUTO GEAR] {} -> gear={} reason={}".format(
                        gear,
                        next_gear,
                        error,
                    ),
                    flush=True,
                )
                self.request_next_gear(next_gear)
                gear = next_gear


    @staticmethod
    def _decode_i16_le(payload: bytes) -> int:
        value = int(payload[0]) | (
            int(payload[1]) << 8
        )
        return (
            value - 0x10000
            if value & 0x8000
            else value
        )

    def parse_scan_test(
        self,
        frame: Frame,
        center_frequency_code: int,
    ):
        """解析FPGA的 SCAN_TEST 事件帧。

        载荷固定4字节 (小端):
            byte0..1: offset_steps (i16) — FPGA 扫描序列 0, -1, +1, -2, +2, ...
            byte2..3: frequency_code  (u16) — 当前频点 (单位 100 Hz)

        验证: center_code + offset_steps 必须等于 frequency_code
              (防止 FPGA 和 Jetson 的扫频序列不同步)

        Returns: (offset_steps, frequency_code)
        """
        if frame.command != CMD_SCAN_TEST:
            raise ProtocolError(
                "当前帧不是SCAN_TEST"
            )

        if len(frame.payload) != 4:
            raise ProtocolError(
                "SCAN_TEST载荷长度必须为4"
            )

        offset_steps = self._decode_i16_le(
            frame.payload[:2]
        )
        frequency_code = (
            int(frame.payload[2])
            | (int(frame.payload[3]) << 8)
        )

        expected_code = (
            int(center_frequency_code)
            + int(offset_steps)
        )

        if frequency_code != expected_code:
            raise ProtocolError(
                "SCAN_TEST频率不一致："
                "center={} offset={} expected={} got={}".format(
                    center_frequency_code,
                    offset_steps,
                    expected_code,
                    frequency_code,
                )
            )

        if not 10 <= frequency_code <= 1000:
            raise ProtocolError(
                "精扫频率超出1～100 kHz"
            )

        return offset_steps, frequency_code

    @staticmethod
    def following_scan_offset(
        offset_steps: int,
    ) -> int:
        """精确镜像FPGA的扫描序列: 0 → -1 → +1 → -2 → +2 → -3 → +3 → ..."""
        offset_steps = int(offset_steps)

        if offset_steps == 0:
            return -1

        if offset_steps < 0:
            return -offset_steps

        return -(offset_steps + 1)

    @classmethod
    def next_valid_scan_offset(
        cls,
        center_frequency_code: int,
        current_offset_steps: int,
    ) -> Optional[int]:
        """返回扫频序列的下一个有效偏移量。

        镜像FPGA的单侧跳步逻辑：
        当 center_code + offset 超出 1~100 kHz 范围时，
        FPGA 会跳过该侧、直接进到反方向的下一个点。
        Jetson 必须独立预测这个行为，以正确丢弃过期 SCAN_TEST。
        当两侧均越界时返回 None (SCAN_END)。
        """
        candidate = cls.following_scan_offset(
            current_offset_steps
        )

        for _ in range(2):
            frequency_code = (
                int(center_frequency_code)
                + candidate
            )

            if 10 <= frequency_code <= 1000:
                return candidate

            candidate = cls.following_scan_offset(
                candidate
            )

        return None


    def print_judge_detail(
        self,
        scan_offset_steps: int,
        frequency_code: int,
        result,
    ) -> None:
        decision = "CORRECT" if result.correct else "WRONG"
        best = result.best_metrics
        print(
            "[JUDGE GLOBAL WHITE] offset={:+d} frequency={:.1f} kHz "
            "decision={} votes={}/{} frames={} gray_threshold={} "
            "white={}/{} global_white={:.2f}% limit={:.2f}% "
            "reason={} time={:.2f}ms".format(
                scan_offset_steps,
                frequency_code / 10.0,
                decision,
                result.passed_votes,
                result.passed_votes + result.failed_votes,
                result.frames_used,
                int(best.threshold),
                best.global_white_pixels,
                best.total_pixels,
                100.0 * best.global_white_ratio,
                100.0 * self.lissajous_cfg.maximum_duty_ratio,
                result.reason,
                result.elapsed_ms,
            )
        )
        if not self.args.judge_verbose:
            return
        for item in result.frame_results:
            m = item.metrics
            print(
                "  [FRAME {:02d}] match={} verdict={} threshold={} "
                "white={}/{} global_white={:.2f}% limit={:.2f}% "
                "reason={}".format(
                    item.window_index,
                    int(item.passed),
                    m.verdict,
                    int(m.threshold),
                    m.global_white_pixels,
                    m.total_pixels,
                    100.0 * m.global_white_ratio,
                    100.0 * self.lissajous_cfg.maximum_duty_ratio,
                    m.reason,
                )
            )

    def capture_confirmed_gray(
        self,
        scan_offset_steps: int,
        frequency_code: int,
    ) -> None:
        """SCAN_CORRECT已获FPGA ACK后，重新等待并采集最终稳定画面。"""
        time.sleep(max(0.0, self.args.judge_settle_ms) / 1000.0)
        discard_frames(self.capture, self.args.judge_discard_frames)
        rectified = self.lissajous_judge._capture_one_rectified()
        gray = lissajous_core._gray_u8(rectified)
        counts = np.bincount(gray.reshape(-1), minlength=256)
        total = int(gray.size)

        print(
            "[CONFIRMED GRAY] offset={:+d} frequency={:.1f} kHz "
            "shape={}x{} total_pixels={}".format(
                scan_offset_steps,
                frequency_code / 10.0,
                gray.shape[1],
                gray.shape[0],
                total,
            )
        )
        for begin in range(0, 256, 16):
            values = []
            for level in range(begin, begin + 16):
                values.append(
                    "{:03d}={}/{:.4f}%".format(
                        level,
                        int(counts[level]),
                        100.0 * counts[level] / max(total, 1),
                    )
                )
            print("  [CONFIRMED HIST] " + " ".join(values))

        if self.args.judge_debug_dir is not None:
            self.args.judge_debug_dir.mkdir(parents=True, exist_ok=True)
            gray_path = self.args.judge_debug_dir / (
                "confirmed_{:+05d}_code_{:04d}_gray.png".format(
                    scan_offset_steps,
                    frequency_code,
                )
            )
            cv2.imwrite(str(gray_path), gray)
            print("[CONFIRMED GRAY SAVED] {}".format(gray_path))

        if self.args.display:
            cv2.imshow("Confirmed frequency grayscale", gray)
            cv2.waitKey(1)

    def measure_phase_absolute(
        self,
        label: str = "measure",
        pattern_id: int = PATTERN_DIAGONAL,
    ) -> float:
        """采集多帧李萨如图，返回绝对相位差的中位值 (度)。

        按图形类型选择不同的相位提取策略：
        - 对角线/圆: 先尝试左右双分支相位拟合 (phase_core.calculate_phase)
                      拟合失败后降级到退化斜直线检测 (detect_degenerate_diagonal_line)
                      2帧确认直线 → 0°; 圆形额外尝试椭圆拟合 → 90°
        - ∞形: 使用中心交叉高度法 (measure_infinity_phase_absolute)

        采集 phase_samples 个有效帧后返回中位值。
        若在 3×phase_samples 帧内仍不足 → 抛出 RuntimeError。
        """
        time.sleep(max(0.0, self.args.phase_settle_ms) / 1000.0)
        discard_frames(self.capture, self.args.phase_discard_frames)

        values = []
        line_votes = 0
        circle_votes = 0
        attempts = max(
            self.args.phase_samples,
            self.args.phase_samples * 3,
        )
        for frame_index in range(1, attempts + 1):
            self.check_abort()
            gray = frame_to_gray(read_frame(self.capture))
            result = phase_core.calculate_phase(
                gray,
                self.phase_rectifier,
                self.phase_cfg,
                already_rectified=False,
            )
            # 斜线判定只用于真正退化到无法做双分支相位拟合的情况。
            # 带小相位差的窄椭圆在低分辨率下也很像斜线，但只要
            # calculate_phase已给出有效结果，就不能用“斜线=0度”覆盖它。
            phase_fit_valid = bool(
                result.ok and np.isfinite(result.phase_deg)
            )
            line_passed = False
            line_metrics = {
                "coverage": 0.0,
                "x_span": 0.0,
                "slope": float("nan"),
                "rms_div": float("inf"),
                "width_p75_div": float("inf"),
                "inliers": 0,
            }
            if pattern_id in (PATTERN_DIAGONAL, PATTERN_CIRCLE):
                line_candidate, line_metrics = detect_degenerate_diagonal_line(
                    result.waveform_mask,
                    self.phase_cfg.pixels_per_div,
                )
                line_passed = bool(line_candidate and not phase_fit_valid)
            if line_passed:
                line_votes += 1
            print(
                "[PHASE LINE {} {:02d}] pass={} votes={}/2 "
                "coverage={:.3f} x_span={:.3f} slope={:.3f} "
                "rms={:.4f}div width_p75={:.3f}div inliers={}".format(
                    label,
                    frame_index,
                    "YES" if line_passed else "NO",
                    line_votes,
                    line_metrics["coverage"],
                    line_metrics["x_span"],
                    line_metrics["slope"],
                    line_metrics["rms_div"],
                    line_metrics["width_p75_div"],
                    line_metrics["inliers"],
                )
            )
            if line_votes >= 2:
                print(
                    "[PHASE LINE FINAL] pattern={} line=YES votes={}/{} "
                    "phase_abs=0.0000 deg".format(
                        pattern_id,
                        line_votes,
                        frame_index,
                    )
                )
                return 0.0
            if pattern_id == PATTERN_CIRCLE and not phase_fit_valid:
                circle_passed, circle_metrics = detect_degenerate_circle(
                    result.waveform_mask,
                    self.phase_cfg.pixels_per_div,
                )
                if circle_passed:
                    circle_votes += 1
                print(
                    "[PHASE CIRCLE {} {:02d}] pass={} votes={}/2 "
                    "center={:.3f}div ratio={:.3f} diameter={:.3f}/{:.3f}div "
                    "rms={:.4f} coverage={:.3f} points={}".format(
                        label,
                        frame_index,
                        "YES" if circle_passed else "NO",
                        circle_votes,
                        circle_metrics["center_offset_div"],
                        circle_metrics["axis_ratio"],
                        circle_metrics["diameter_x_div"],
                        circle_metrics["diameter_y_div"],
                        circle_metrics["radial_rms"],
                        circle_metrics["angular_coverage"],
                        circle_metrics["points"],
                    )
                )
                if circle_votes >= 2:
                    print(
                        "[PHASE CIRCLE FINAL] circle=YES votes={}/{} "
                        "phase_abs=90.0000 deg".format(
                            circle_votes,
                            frame_index,
                        )
                    )
                    return 90.0
            measured_phase = float("nan")
            if pattern_id == PATTERN_INFINITY:
                try:
                    measured_phase, crossing_y, evidence = (
                        measure_infinity_phase_absolute(
                            result.waveform_mask,
                            self.phase_cfg.pixels_per_div,
                        )
                    )
                    print(
                        "[PHASE INFINITY {} {:02d}] abs={:.4f} deg "
                        "crossing_y={:.2f} evidence={}".format(
                            label,
                            frame_index,
                            measured_phase,
                            crossing_y,
                            evidence,
                        )
                    )
                except RuntimeError as error:
                    print(
                        "[PHASE INFINITY {} {:02d}] failed reason={}".format(
                            label,
                            frame_index,
                            error,
                        )
                    )
            elif result.ok and np.isfinite(result.phase_deg):
                measured_phase = float(abs(result.phase_deg))

            if np.isfinite(measured_phase):
                values.append(measured_phase)
                print(
                    "[PHASE {} {:02d}] abs={:.4f} deg "
                    "direction={} rms={:.5f}/{:.5f} div valid={}/{}".format(
                        label,
                        frame_index,
                        measured_phase,
                        result.tilt_direction,
                        result.left_fit.rms_div,
                        result.right_fit.rms_div,
                        len(values),
                        self.args.phase_samples,
                    )
                )
                if self.args.phase_debug_dir is not None:
                    self.args.phase_debug_dir.mkdir(parents=True, exist_ok=True)
                    shown = phase_core.annotate(
                        result,
                        self.phase_cfg,
                        "phase_{}_{:02d}".format(label, frame_index),
                    )
                    cv2.imwrite(
                        str(self.args.phase_debug_dir / (
                            "phase_{}_{:02d}.png".format(label, frame_index)
                        )),
                        shown,
                    )
                if len(values) >= self.args.phase_samples:
                    break
            elif pattern_id != PATTERN_INFINITY:
                print(
                    "[PHASE {} {:02d}] failed reason={}".format(
                        label,
                        frame_index,
                        result.reason,
                    )
                )

        if len(values) < self.args.phase_samples:
            raise RuntimeError(
                "相位测量有效帧不足：{}/{}".format(
                    len(values),
                    self.args.phase_samples,
                )
            )

        phase_abs = float(np.median(np.asarray(values, dtype=np.float64)))
        print(
            "[PHASE FINAL] abs={:.4f} deg samples={}".format(
                phase_abs,
                ",".join("{:.4f}".format(value) for value in values),
            )
        )
        return phase_abs

    def verify_final_circle_hough(
        self,
        label: str,
    ) -> bool:
        """圆形模式：调相稳定后用霍夫圆检测做最终确认 (3帧2票)。

        在每次调相完成后、判定前被调用。
        一旦2/3帧检测到符合标准的圆 → 当前SET_PHASE即为最终值。
        这是圆形判定的最后一道防线，能排除椭圆但非正圆的情况。
        """
        time.sleep(max(0.0, self.args.phase_settle_ms) / 1000.0)
        discard_frames(self.capture, self.args.phase_discard_frames)
        votes = 0
        total = 3
        for frame_index in range(1, total + 1):
            gray = frame_to_gray(read_frame(self.capture))
            result = phase_core.calculate_phase(
                gray,
                self.phase_rectifier,
                self.phase_cfg,
                already_rectified=False,
            )
            passed, metrics = detect_hough_circle(
                result.waveform_mask,
                self.phase_cfg.pixels_per_div,
            )
            if passed:
                votes += 1
            print(
                "[PHASE HOUGH CIRCLE {} {:02d}] pass={} votes={}/2 "
                "found={} center={:.3f}div radius={:.3f}div "
                "support={:.3f}".format(
                    label,
                    frame_index,
                    "YES" if passed else "NO",
                    votes,
                    metrics["found"],
                    metrics["center_offset_div"],
                    metrics["radius_div"],
                    metrics["support"],
                ),
                flush=True,
            )
            if votes >= 2:
                print(
                    "[PHASE HOUGH FINAL] circle=YES; "
                    "keep current FPGA phase and stop updating",
                    flush=True,
                )
                return True
            if votes + (total - frame_index) < 2:
                break
        return False

    @staticmethod
    def _wrap_phase_cdeg(value: int) -> int:
        """将绝对相位命令折回[-180, 180]度。"""
        wrapped = ((int(value) + 18000) % 36000) - 18000
        return 18000 if wrapped == -18000 and value > 0 else wrapped

    def run_phase_from_verified_line(
        self,
        final_offset_cdeg: int,
    ) -> bool:
        """对角线/圆形模式的统一相位搜索 —— 先找到直线，再按需偏置。

        核心思路：圆的相位 = 直线相位 + 90°。
        因此两种图形共享同一套"找直线"搜索逻辑，
        只在最后根据 final_offset_cdeg 偏置。

        搜索策略：
        1. 从 0° 开始测量绝对相位
        2. 若相位跨度 > 48.59° (大椭圆) → 先 ±60° 探测找中间状态
        3. 否则 ±|误差| 两个方向尝试补偿
        4. 圆形模式：找到直线后偏置 +90°，不再额外验证

        Returns: True=成功, False=未找到
        """
        tested = set()
        # 0度优先；若初始靠近圆导致数值拟合失败，
        # 仿真最优的±60度与初始0度形成三个等距采样点；
        # 全相位范围最坏残余30度，只需额外测试两个点。
        pending = [0, *PHASE_ESCAPE_CANDIDATES_CDEG]
        line_cdeg = None

        while pending and len(tested) < 16:
            command_cdeg = self._wrap_phase_cdeg(pending.pop(0))
            if command_cdeg in tested:
                continue
            tested.add(command_cdeg)
            self.link.set_phase_and_wait_ack(
                command_cdeg * PHASE_FPGA_TEST_SCALE,
                timeout=self.args.phase_command_timeout,
            )
            print(
                "[CIRCLE LINE SEARCH] SET_PHASE={:+.2f} deg ACK".format(
                    command_cdeg / 100.0
                ),
                flush=True,
            )
            try:
                phase_abs = self.measure_phase_absolute(
                    "circle_line_{:+05d}".format(command_cdeg),
                    PATTERN_DIAGONAL,
                )
            except RuntimeError as error:
                print(
                    "[CIRCLE LINE SEARCH] command={:+.2f} invalid: {}".format(
                        command_cdeg / 100.0,
                        error,
                    ),
                    flush=True,
                )
                continue

            if phase_abs <= self.args.phase_target_tolerance_deg:
                line_cdeg = command_cdeg
                break

            if phase_abs > PHASE_LARGE_SPAN_DEG:
                # 大于6格（约48.59度）时不直接做大步长补偿。
                # 先在当前绝对相位两侧±60度，找到间距更小的
                # 椭圆，再复用同一套直线寻找逻辑。
                pending.insert(
                    0,
                    self._wrap_phase_cdeg(command_cdeg - PHASE_PROBE_CDEG),
                )
                pending.insert(
                    0,
                    self._wrap_phase_cdeg(command_cdeg + PHASE_PROBE_CDEG),
                )
                continue

            delta_cdeg = int(round(abs(phase_abs) * 100.0))
            # 静态椭圆只能给出绝对相位，两个方向都入队验证。
            pending.insert(0, self._wrap_phase_cdeg(command_cdeg - delta_cdeg))
            pending.insert(0, self._wrap_phase_cdeg(command_cdeg + delta_cdeg))

        if line_cdeg is None:
            print("[CIRCLE LINE SEARCH] no verified line", flush=True)
            return False

        print(
            "[CIRCLE LINE FINAL] verified_line_phase={:+.2f} deg".format(
                line_cdeg / 100.0
            ),
            flush=True,
        )
        final_cdeg = self._wrap_phase_cdeg(
            line_cdeg + int(final_offset_cdeg)
        )
        if int(final_offset_cdeg) == 0:
            print(
                "[PHASE DONE] verified line; keep SET_PHASE={:+.2f} deg".format(
                    line_cdeg / 100.0
                ),
                flush=True,
            )
            return True

        self.link.set_phase_and_wait_ack(
            final_cdeg * PHASE_FPGA_TEST_SCALE,
            timeout=self.args.phase_command_timeout,
        )
        print(
            "[CIRCLE +90 TX] line={:+.2f} deg +90.00 deg "
            "=> SET_PHASE={:+.2f} deg ACK".format(
                line_cdeg / 100.0,
                final_cdeg / 100.0,
            ),
            flush=True,
        )
        print(
            "[PHASE DONE] circle command sent; no further verification or update",
            flush=True,
        )
        return True

    def run_phase_compensation(
        self,
        request: StartRequest,
    ) -> bool:
        """相位补偿闭环 —— 精扫找到正确频率后的最后一步。

        策略因图形类型而异：

        **对角线 (PATTERN_DIAGONAL)**：
            复用直线搜索逻辑 (run_phase_from_verified_line)，
            从 0° 开始 → 测量绝对相位 → 补偿到 0°。

        **圆 (PATTERN_CIRCLE)**：
            先按对角线逻辑找到直线相位 → 再 +90° 发送圆形相位。
            每次调相后优先用霍夫圆验证，一旦确认为圆则立即停止更新。

        **∞形 (PATTERN_INFINITY)**：
            1. 从 0° 开始测量初始相位
            2. 若交叉点位于死区 (>3 div, ≈48.59°) → ±60° 相位逃脱序列
            3. 找到逃离死区的基准相位后 → ±|误差| 两个方向尝试
            4. 补偿到目标相位 (0°)

        所有图形：两个方向都失败后恢复 0° 并返回 False。

        Returns:
            True: 相位补偿成功
            False: 两个方向均无法达到目标
        """
        if self.link is None:
            print("[PHASE] standalone模式不发送相位命令")
            return False

        if request.pattern_id in (PATTERN_DIAGONAL, PATTERN_CIRCLE):
            return self.run_phase_from_verified_line(
                9000 if request.pattern_id == PATTERN_CIRCLE else 0
            )

        self.link.set_phase_and_wait_ack(
            0,
            timeout=self.args.phase_command_timeout,
        )
        print("[UART] SET_PHASE +0.00 deg ACK")

        target_phase_abs = request.target_phase_abs_deg
        print(
            "[PHASE TARGET] pattern={} target_abs={:.2f} deg".format(
                request.pattern_id,
                target_phase_abs,
            )
        )
        # 无限型中央交点超过±3 div时，绝对相位大于
        # asin(3/4)≈48.59度；仿真选择±60度逃离边界死区。
        initial_error = None
        try:
            phase_abs = self.measure_phase_absolute(
                "initial",
                request.pattern_id,
            )
        except RuntimeError as error:
            phase_abs = float("nan")
            initial_error = error

        base_command_cdeg = 0
        needs_phase_escape = (
            request.pattern_id == PATTERN_INFINITY
            and (
                not np.isfinite(phase_abs)
                or phase_abs > INFINITY_DEAD_ZONE_DEG
            )
        )
        if needs_phase_escape:
            print(
                "[PHASE INFINITY DEAD ZONE] initial={} "
                "crossing>|{:.1f}|div threshold={:.4f} deg; "
                "probe sequence=+/-60 deg".format(
                    (
                        "{:.4f} deg".format(phase_abs)
                        if np.isfinite(phase_abs)
                        else "invalid ({})".format(initial_error)
                    ),
                    INFINITY_DEAD_ZONE_DIV,
                    INFINITY_DEAD_ZONE_DEG,
                )
            )
            probe_results = []
            for probe_cdeg in PHASE_ESCAPE_CANDIDATES_CDEG:
                self.link.set_phase_and_wait_ack(
                    probe_cdeg * PHASE_FPGA_TEST_SCALE,
                    timeout=self.args.phase_command_timeout,
                )
                print(
                    "[UART] SET_PHASE probe={:+.2f} deg ACK".format(
                        probe_cdeg / 100.0
                    )
                )
                try:
                    measured = self.measure_phase_absolute(
                        "probe_{:+05d}".format(probe_cdeg),
                        request.pattern_id,
                    )
                except RuntimeError as error:
                    print(
                        "[PHASE INFINITY PROBE] {:+.2f} invalid reason={}".format(
                            probe_cdeg / 100.0,
                            error,
                        )
                    )
                    continue
                probe_results.append((measured, probe_cdeg))


            if probe_results:
                # 角度越小，无限型中央交点越靠近中心。
                phase_abs, base_command_cdeg = min(
                    probe_results,
                    key=lambda item: item[0],
                )
                print(
                    "[PHASE INFINITY PROBE DONE] selected={:+.2f} deg "
                    "measured_abs={:.4f} deg".format(
                        base_command_cdeg / 100.0,
                        phase_abs,
                    )
                )
            else:
                raise RuntimeError(
                    "无限型死区±60度试探均无效"
                )
        elif not np.isfinite(phase_abs):
            raise initial_error or RuntimeError("初始相位测量失败")
        correction_abs = abs(phase_abs - target_phase_abs)
        phase_cdeg = int(round(correction_abs * 100.0))
        phase_cdeg = min(18000, max(0, phase_cdeg))
        print(
            "[PHASE ERROR] measured={:.4f} target={:.4f} "
            "correction_abs={:.4f} deg".format(
                phase_abs,
                target_phase_abs,
                correction_abs,
            )
        )

        # SET_PHASE是绝对值；经过±60度试探后，必须以选中的
        # probe命令为基准加/减残余相位，不能丢掉逃脱命令基准。
        trials = [
            self._wrap_phase_cdeg(base_command_cdeg + phase_cdeg)
        ]
        second_trial = self._wrap_phase_cdeg(
            base_command_cdeg - phase_cdeg
        )
        if second_trial != trials[0]:
            trials.append(second_trial)

        for trial_index, trial_cdeg in enumerate(trials, 1):
            wire_cdeg = trial_cdeg * PHASE_FPGA_TEST_SCALE
            self.link.set_phase_and_wait_ack(
                wire_cdeg,
                timeout=self.args.phase_command_timeout,
            )
            print(
                "[UART] SET_PHASE desired={:+.2f} deg wire_raw={} "
                "scale=x{} ACK trial={}/{}".format(
                    trial_cdeg / 100.0,
                    wire_cdeg,
                    PHASE_FPGA_TEST_SCALE,
                    trial_index,
                    len(trials),
                )
            )
            # 圆形模式每次调相点稳定后优先做霍夫圆验证。
            # 一旦确认为圆，当前SET_PHASE就是最终值，不再更新。
            if (
                request.pattern_id == PATTERN_CIRCLE
                and self.verify_final_circle_hough(
                    "trial_{:+05d}".format(trial_cdeg)
                )
            ):
                print(
                    "[PHASE DONE] Hough circle confirmed at "
                    "{:+.2f} deg".format(trial_cdeg / 100.0),
                    flush=True,
                )
                return True
            try:
                residual_phase_abs = self.measure_phase_absolute(
                    "trial_{:+05d}".format(trial_cdeg),
                    request.pattern_id,
                )
                phase_error = abs(
                    residual_phase_abs - target_phase_abs
                )
                target_ok = (
                    phase_error
                    <= self.args.phase_target_tolerance_deg
                )
                reason = "PHASE_WITHIN_TOLERANCE" if target_ok else "PHASE_OUT_OF_TOLERANCE"
            except RuntimeError as error:
                residual_phase_abs = float("nan")
                phase_error = float("inf")
                target_ok = False
                reason = str(error)
            print(
                "[PHASE TRIAL] command={:+.2f} deg residual_abs={} "
                "target_abs={:.2f} error={} tolerance={:.2f} "
                "target={} reason={}".format(
                    trial_cdeg / 100.0,
                    (
                        "{:.4f}".format(residual_phase_abs)
                        if np.isfinite(residual_phase_abs)
                        else "invalid"
                    ),
                    target_phase_abs,
                    (
                        "{:.4f}".format(phase_error)
                        if np.isfinite(phase_error)
                        else "invalid"
                    ),
                    self.args.phase_target_tolerance_deg,
                    "YES" if target_ok else "NO",
                    reason,
                )
            )
            if target_ok:
                print(
                    "[PHASE DONE] final_phase={:+.2f} deg".format(
                        trial_cdeg / 100.0
                    )
                )
                return True

        self.link.set_phase_and_wait_ack(
            0,
            timeout=self.args.phase_command_timeout,
        )
        print("[PHASE FAILED] 两个方向均未形成目标图像，已恢复0.00度")
        return False

    def run_fine_sweep_session(
        self,
        coarse_result: FinalMeasurement,
    ) -> bool:
        """双侧 100 Hz 精扫会话 —— Jetson 侧的核心闭环。

        前置条件: 粗测频率已通过 FINAL_FREQUENCY 发送给FPGA，
                  FPGA已进入扫频状态并开始发送 SCAN_TEST。

        FPGA 职责 (本方法不控制, 只响应)：
            1. 显示中心频率作为初始输出
            2. 按序列 0, -1, +1, -2, +2, ... 依次设置频率
            3. 每次频率稳定后发送 SCAN_TEST 事件
            4. 收到 SCAN_CORRECT 后停在当前频点, 等待 SET_PHASE
            5. 扫描完所有有效频点仍未 CORRECT → 发送 SCAN_END

        Jetson 职责 (本方法实现)：
            1. 收到 SCAN_TEST → 丢帧稳定 → 执行判图 (两帧稳定窗口)
            2. 判定为 CORRECT → 回复 SCAN_CORRECT → 采集确认灰度图 → 返回 True
            3. 判定为 WRONG → 回复 SCAN_WRONG → 等待下一个 SCAN_TEST
            4. 收到 SCAN_END → 返回 False (精扫不收敛)
            5. SCAN_TEST 超时 → 按 FPGA 确定性序列恢复当前频点继续判定

        Returns:
            True: 找到正确图形
            False: 精扫完毕未找到 (FPGA 发送 SCAN_END)
        """
        center_code = int(
            coarse_result.frequency_code
        )

        if self.link is None:
            print(
                "[FINE SWEEP SIM] center={} "
                "({:.1f} kHz), only judge offset 0".format(
                    center_code,
                    center_code / 10.0,
                )
            )
            result = (
                self.lissajous_judge
                .judge_current_frequency()
            )
            self.print_judge_detail(
                0,
                center_code,
                result,
            )
            return bool(result.correct)

        print(
            "[FINE SWEEP] waiting SCAN_TEST, "
            "center_code={} center={:.1f} kHz".format(
                center_code,
                center_code / 10.0,
            )
        )

        request_index = 0
        expected_offset = 0

        while True:
            timeout = (
                None
                if self.args.scan_idle_timeout
                <= 0
                else self.args.scan_idle_timeout
            )
            frame = None

            try:
                frame = self.link.get_event(
                    timeout=timeout
                )
            except queue.Empty:
                if expected_offset is None:
                    raise ResponseTimeout(
                        "等待FPGA的SCAN_END超时"
                    )

                # The FPGA has already applied this deterministic point
                # before emitting SCAN_TEST.  If only the event frame was
                # lost, judge the known current point instead of deadlocking.
                scan_offset_steps = expected_offset
                frequency_code = (
                    center_code + scan_offset_steps
                )
                print(
                    "[FINE SWEEP RECOVERY] SCAN_TEST timeout; "
                    "use FPGA deterministic point "
                    "offset={:+d} code={}".format(
                        scan_offset_steps,
                        frequency_code,
                    )
                )

            if frame is not None:
                if not isinstance(frame, Frame):
                    continue

                if frame.command == CMD_ABORTED:
                    raise MeasurementAborted(
                        "FPGA在精扫阶段发送ABORTED"
                    )

                if frame.command == CMD_START:
                    raise StartPreempted(frame)

                if frame.command == CMD_SCAN_END:
                    if len(frame.payload) != 1:
                        raise ProtocolError(
                            "SCAN_END载荷长度必须为1"
                        )

                    reason = int(frame.payload[0])
                    print(
                        "[FINE SWEEP] FPGA结束精扫，"
                        "reason={}".format(reason)
                    )
                    return False

                if frame.command != CMD_SCAN_TEST:
                    print(
                        "[FINE SWEEP] ignore command "
                        "0x{:02X}".format(
                            frame.command
                        )
                    )
                    continue

                (
                    scan_offset_steps,
                    frequency_code,
                ) = self.parse_scan_test(
                    frame,
                    center_code,
                )

                if (
                    expected_offset is None
                    or scan_offset_steps
                    != expected_offset
                ):
                    print(
                        "[FINE SWEEP] discard stale SCAN_TEST "
                        "expected={} received={:+d}".format(
                            (
                                "SCAN_END"
                                if expected_offset is None
                                else "{:+d}".format(
                                    expected_offset
                                )
                            ),
                            scan_offset_steps,
                        )
                    )
                    continue

            print(
                "[SCAN POINT {:03d}] "
                "offset={:+d} "
                "frequency_code={} "
                "frequency={:.1f} kHz".format(
                    request_index,
                    scan_offset_steps,
                    frequency_code,
                    frequency_code / 10.0,
                )
            )

            if request_index == 0:
                print(
                    "[FINE SWEEP FIRST FRAME] extra settle=50 ms",
                    flush=True,
                )
                time.sleep(FINE_SWEEP_FIRST_FRAME_EXTRA_SETTLE_SECONDS)

            judge_result = (
                self.lissajous_judge
                .judge_current_frequency()
            )

            self.print_judge_detail(
                scan_offset_steps,
                frequency_code,
                judge_result,
            )

            self.link.send_scan_decision_and_wait_ack(
                scan_offset_steps=(
                    scan_offset_steps
                ),
                correct=(
                    judge_result.correct
                ),
                timeout=(
                    self.args.scan_result_ack_timeout
                ),
            )

            print(
                "[UART] sent {} offset={:+d}".format(
                    (
                        "SCAN_CORRECT"
                        if judge_result.correct
                        else "SCAN_WRONG"
                    ),
                    scan_offset_steps,
                )
            )

            if judge_result.correct:
                self.capture_confirmed_gray(
                    scan_offset_steps,
                    frequency_code,
                )
                print(
                    "[FINE SWEEP DONE] "
                    "final_frequency_code={} "
                    "final_frequency={:.1f} kHz".format(
                        frequency_code,
                        frequency_code / 10.0,
                    )
                )
                return True

            expected_offset = (
                self.next_valid_scan_offset(
                    center_code,
                    scan_offset_steps,
                )
            )
            request_index += 1

    def send_final_frequency(
        self,
        result: FinalMeasurement,
    ) -> None:
        """将粗测结果通过 FINAL_FREQUENCY 命令发送给FPGA。

        载荷: 2字节 frequency_code (u16 小端, 单位 100 Hz), 范围 10~1000。
        FPGA收到后进入SCAN状态，以该频点为中心开始双侧精扫。
        不发送档位、周期或质量信息。
        """
        if not 10 <= result.frequency_code <= 1000:
            raise ProtocolError(
                "FPGA只接受1～100 kHz（code 10～1000）"
            )

        # 必须在真正写UART之前输出：即使FPGA未回ACK或超时，
        # 终端也能明确看到本次粗测结果和已尝试发送的频率。
        print(
            "[COARSE FPGA TX] frequency={:.1f} kHz "
            "frequency_hz={} code_100hz={}".format(
                result.frequency_code / 10.0,
                result.frequency_code * 100,
                result.frequency_code,
            ),
            flush=True,
        )

        # 结果载荷严格只有2字节频率码，不发送档位、周期或质量。
        self.link.send_frequency_and_wait_ack(
            result.frequency_code,
            timeout=self.args.result_ack_timeout,
        )

        print(
            "[COARSE FPGA ACK] frequency={:.1f} kHz "
            "frequency_hz={} code_100hz={}".format(
                result.frequency_code / 10.0,
                result.frequency_code * 100,
                result.frequency_code,
            ),
            flush=True,
        )

    def send_done(self) -> None:
        """通知FPGA保持当前频率和相位，点亮LED4，发出声光提示。

        只有在精扫判图通过且相位补偿完成后才被调用。
        """
        if self.link is None:
            return
        print("[UART TX] DONE cmd=0x1E len=0", flush=True)
        self.link.send_done_and_wait_ack(
            timeout=self.args.result_ack_timeout,
        )
        print("[UART ACK] DONE cmd=0x1E status=0", flush=True)

    def wait_for_external_frequency_change(self, holdoff_seconds: float = 0.0) -> None:
        """DONE 后持续监控图形变化 —— 检测外部输入改变或场景切换。

        分两个阶段：
        1. **保护期 (holdoff_seconds=6s)**: 刚从DONE完成，满足赛题
           "稳定时间≥5s"的要求。保护期内禁止RESCAN，但START仍可抢占。
        2. **监控期**: 保护期结束后持续判图。连续2次判断为 CHANGED
           → 返回，触发 send_rescan() 开始新一轮粗测。

        这一机制保证：
        - 首次完成后的6秒内图形被正确展示且不被重扫打断
        - 6秒后若外部频率改变（如裁判/测试换频点），系统能自动重新锁定
        """
        holdoff_seconds = max(0.0, float(holdoff_seconds))
        if holdoff_seconds > 0.0:
            deadline = time.monotonic() + holdoff_seconds
            print(
                "[DONE HOLDOFF] keep locked phase for {:.1f}s; RESCAN disabled".format(
                    holdoff_seconds
                ),
                flush=True,
            )
            while True:
                # START始终拥有最高优先级，即使处于无限型锁相保护期。
                self.check_abort()
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    break
                time.sleep(min(0.05, remaining))
            print(
                "[DONE HOLDOFF] finished; external-change monitor enabled",
                flush=True,
            )
        failed_checks = 0
        check_index = 0
        print(
            "[DONE MONITOR] waiting for external frequency change",
            flush=True,
        )
        while True:
            self.check_abort()
            verdict = self.lissajous_judge.judge_current_frequency()
            check_index += 1
            if verdict.correct:
                failed_checks = 0
            else:
                failed_checks += 1
            print(
                "[DONE MONITOR {:04d}] shape={} failures={}/2 "
                "votes={}/{} white={:.2f}% reason={}".format(
                    check_index,
                    "STABLE" if verdict.correct else "CHANGED",
                    failed_checks,
                    verdict.passed_votes,
                    verdict.frames_used,
                    100.0 * verdict.best_metrics.global_white_ratio,
                    verdict.reason,
                ),
                flush=True,
            )
            if failed_checks >= 2:
                return

    def send_rescan(self) -> None:
        """请求FPGA回到gear0锯齿波；ACK后不等待新START。"""
        if self.link is None:
            return
        print("[UART TX] RESCAN cmd=0x1F len=0", flush=True)
        self.link.send_rescan_and_wait_ack(
            timeout=self.args.result_ack_timeout,
        )
        print(
            "[UART ACK] RESCAN cmd=0x1F status=0; restart from gear0",
            flush=True,
        )

    def report_failure(
        self,
        reason: int,
    ) -> None:
        if self.link is None:
            return

        try:
            sequence = self.link.send_failure_no_ack(
                reason=reason,
            )
            print(
                "[UART] MEASURE_FAILED sent "
                "without ACK, seq={}".format(sequence)
            )
        except Exception as error:
            print(
                "[UART] 失败信息发送失败：{}".format(
                    error
                )
            )


def parse_start(
    frame: Frame,
) -> StartRequest:
    if frame.command != CMD_START:
        raise ValueError("不是START帧")

    if len(frame.payload) == 0:
        return StartRequest()

    if len(frame.payload) == 1:
        pattern_id = int(frame.payload[0])
        if pattern_id not in SUPPORTED_PATTERNS:
            raise ProtocolError(
                "当前版本不支持pattern_id={}".format(pattern_id)
            )
        return StartRequest(pattern_id=pattern_id)

    raise ProtocolError(
        "START载荷必须为空或包含1字节pattern_id"
    )


def build_argument_parser():
    parser = argparse.ArgumentParser(
        description=(
            "FPGA START -> Jetson四档判断 -> "
            "八帧粗测频 -> FPGA 100Hz双侧精扫 -> Jetson规则图形判定"
        )
    )

    parser.add_argument(
        "--standalone",
        action="store_true",
        help=(
            "不连接UART，在本地模拟FPGA切档成功，"
            "直接执行一次完整测量"
        ),
    )

    parser.add_argument(
        "--port",
        default="/dev/ttyTHS1",
    )
    parser.add_argument(
        "--baud",
        type=int,
        default=921600,
    )
    parser.add_argument(
        "--uart-timeout",
        type=float,
        default=0.50,
    )
    parser.add_argument(
        "--uart-retries",
        type=int,
        default=3,
    )
    parser.add_argument(
        "--gear-changed-timeout",
        type=float,
        default=0.50,
    )
    parser.add_argument(
        "--result-ack-timeout",
        type=float,
        default=0.50,
    )

    parser.add_argument(
        "--camera",
        type=int,
        default=0,
    )
    parser.add_argument(
        "--gst-pipeline",
        default="",
    )
    parser.add_argument(
        "--width",
        type=int,
        default=2740,
    )
    parser.add_argument(
        "--height",
        type=int,
        default=1824,
    )
    parser.add_argument(
        "--fps",
        type=float,
        default=30.0,
    )
    parser.add_argument(
        "--fourcc",
        default="MJPG",
    )

    parser.add_argument(
        "--calibration",
        type=Path,
        default=Path(
            "scope_8x8_nine_point_calibration.json"
        ),
    )
    parser.add_argument(
        "--profiles",
        type=Path,
        default=Path(
            "range_profiles.json"
        ),
    )
    parser.add_argument(
        "--warmup-frames",
        type=int,
        default=8,
    )

    parser.add_argument(
        "--switch-pixels-per-div",
        type=int,
        default=96,
    )
    parser.add_argument(
        "--switch-right-tip-min-x",
        type=float,
        default=3.05,
    )
    parser.add_argument(
        "--switch-max-frames",
        type=int,
        default=40,
    )

    parser.add_argument(
        "--camera-settle-ms",
        type=float,
        default=150.0,
    )
    parser.add_argument(
        "--camera-discard-frames",
        type=int,
        default=3,
    )

    parser.add_argument(
        "--measure-pixels-per-div",
        type=int,
        default=160,
    )
    parser.add_argument(
        "--measure-right-tip-min-x",
        type=float,
        default=7.2,
    )
    parser.add_argument(
        "--measure-discard-frames",
        type=int,
        default=2,
    )
    parser.add_argument(
        "--measure-max-frames",
        type=int,
        default=120,
    )

    parser.add_argument(
        "--cluster-absolute-hz",
        type=float,
        default=200.0,
    )
    parser.add_argument(
        "--cluster-relative-ratio",
        type=float,
        default=0.015,
    )

    parser.add_argument(
        "--scan-idle-timeout",
        type=float,
        default=1.0,
        help=(
            "等待下一条SCAN_TEST的超时秒数；"
            "超时后按FPGA当前确定频点继续判定；"
            "0表示一直等待"
        ),
    )
    parser.add_argument(
        "--scan-result-ack-timeout",
        type=float,
        default=0.50,
    )

    # 精扫判定只保留灰度阈值、全图白色占比阈值和帧投票。
    parser.add_argument(
        "--judge-mode",
        choices=("fast", "balanced", "robust"),
        default="balanced",
        help="判定模式：fast最快，balanced默认，robust更保守",
    )
    parser.add_argument(
        "--judge-settle-ms",
        type=float,
        default=350.0,
        help="收到SCAN_TEST后等待画面稳定的毫秒数，默认350；随后使用两帧稳定窗口",
    )
    parser.add_argument(
        "--judge-discard-frames",
        type=int,
        default=1,
        help="两帧判定前丢弃的相机缓存帧数，默认1",
    )
    parser.add_argument(
        "--judge-binary-threshold",
        type=int,
        default=50,
        help="灰度二值化阈值，默认50",
    )
    parser.add_argument(
        "--judge-duty-threshold",
        type=float,
        default=30.0,
        help="全图白色像素占比上限固定为30%%；超过即判为方块",
    )
    parser.add_argument("--judge-debug-dir", type=Path, default=None)
    parser.add_argument("--judge-verbose", action="store_true")

    parser.add_argument(
        "--disable-phase-compensation",
        action="store_true",
        help="频率精扫成功后不执行自动相位补偿",
    )
    parser.add_argument("--phase-pixels-per-div", type=int, default=160)
    parser.add_argument("--phase-samples", type=int, default=3)
    parser.add_argument("--phase-settle-ms", type=float, default=350.0)
    parser.add_argument("--phase-discard-frames", type=int, default=1)
    parser.add_argument("--phase-command-timeout", type=float, default=0.50)
    parser.add_argument(
        "--phase-target-tolerance-deg",
        type=float,
        default=5.0,
        help="补偿后实测绝对相位与pattern目标的允许误差，默认5度",
    )
    parser.add_argument("--phase-debug-dir", type=Path, default=None)

    # 兼容旧启动命令：这些参数仍可被解析，但不再形成多套独立门槛。
    parser.add_argument("--judge-pixels-per-div", type=int, default=None, help=argparse.SUPPRESS)
    parser.add_argument("--judge-window-frames", type=int, default=None, help=argparse.SUPPRESS)
    parser.add_argument("--judge-trace-threshold-scale", type=float, default=None, help=argparse.SUPPRESS)
    parser.add_argument("--judge-search-frames", type=int, default=None, help=argparse.SUPPRESS)
    parser.add_argument("--judge-required-matched-windows", type=int, default=None, help=argparse.SUPPRESS)
    parser.add_argument("--judge-confirmation-repeats", type=int, default=None, help=argparse.SUPPRESS)
    parser.add_argument("--judge-confirmation-frames", type=int, default=None, help=argparse.SUPPRESS)
    parser.add_argument("--judge-confirmation-settle-ms", type=float, default=None, help=argparse.SUPPRESS)
    parser.add_argument("--judge-minimum-confidence", type=float, default=None, help=argparse.SUPPRESS)
    parser.add_argument("--judge-minimum-trace-contrast", type=float, default=None, help=argparse.SUPPRESS)
    parser.add_argument("--judge-minimum-mean-iou", type=float, default=None, help=argparse.SUPPRESS)
    parser.add_argument("--judge-maximum-temporal-spread", type=float, default=None, help=argparse.SUPPRESS)
    parser.add_argument("--judge-maximum-union-ratio", type=float, default=None, help=argparse.SUPPRESS)
    parser.add_argument("--judge-maximum-bright-ratio", type=float, default=None, help=argparse.SUPPRESS)
    parser.add_argument("--judge-relaxed-minimum-iou", type=float, default=None, help=argparse.SUPPRESS)
    parser.add_argument("--judge-relaxed-maximum-spread", type=float, default=None, help=argparse.SUPPRESS)
    parser.add_argument("--judge-partial-ellipse-error-limit", type=float, default=None, help=argparse.SUPPRESS)
    parser.add_argument("--judge-partial-ellipse-coverage-limit", type=float, default=None, help=argparse.SUPPRESS)
    parser.add_argument("--judge-full-ellipse-error-limit", type=float, default=None, help=argparse.SUPPRESS)
    parser.add_argument("--judge-full-ellipse-coverage-limit", type=float, default=None, help=argparse.SUPPRESS)
    parser.add_argument("--judge-disable-fast-accept", action="store_true", help=argparse.SUPPRESS)

    parser.add_argument(
        "--display",
        action="store_true",
    )

    return parser


def main() -> int:
    """程序入口 —— 无线自动模式的主事件循环。

    两种运行模式：

    1. --standalone (本地测试)：
       执行一次 run_once → fine_sweep，不连接 FPGA UART。
       在本地模拟 FPGA 切档成功，直接摄像头测频。

    2. 联机模式 (默认)：
       连接 FPGA UART (/dev/ttyTHS1, 921600 bps)，
       循环等待 FPGA START 事件：
           START → run_once → send_final_frequency → fine_sweep
               → phase_compensation → send_done → monitor → RESCAN
       异常处理：
           - StartPreempted: 新START抢占 → 放回帧队首，重新开始
           - MeasurementAborted/RemoteAborted: 取消 → 循环继续
           - ProtocolError/ResponseTimeout: UART异常 → report_failure
           - RuntimeError: 摄像头/选档超时 → report_failure

    返回码：0=成功, 1=失败
    """
    args = build_argument_parser().parse_args()

    if (
        args.profiles is not None
        and not args.profiles.exists()
    ):
        args.profiles = None

    link = None
    controller = None

    try:
        if not args.standalone:
            link = FpgaCompatibleUartLink(
                port=args.port,
                baudrate=args.baud,
                response_timeout=(
                    args.uart_timeout
                ),
                retries=args.uart_retries,
            )

        controller = FrequencyController(
            args,
            link,
        )

        print("[VERSION]", VERSION)

        if args.standalone:
            result = controller.run_once(
                initial_gear=0
            )

            print(json.dumps({
                "gear": result.gear,
                "frequency_hz": (
                    result.frequency_hz
                ),
                "frequency_code_100hz": (
                    result.frequency_code
                ),
                "period_div": (
                    result.period_div
                ),
                "used_frames": (
                    result.used_frames
                ),
                "rejected_frames": (
                    result.rejected_frames
                ),
            }, ensure_ascii=False, indent=2))

            fine_ok = controller.run_fine_sweep_session(
                result
            )
            print(
                "[STANDALONE FINE] {}".format(
                    "CORRECT"
                    if fine_ok
                    else "WRONG"
                )
            )

            return 0 if fine_ok else 1

        print(
            "[UART] 等待FPGA START，"
            "port={} baud={}".format(
                args.port,
                args.baud,
            )
        )

        while True:
            try:
                frame = link.get_event(
                    timeout=None
                )
            except queue.Empty:
                continue

            if not isinstance(frame, Frame):
                continue

            if frame.command != CMD_START:
                continue

            start_request = parse_start(frame)

            print(
                "[START] initial_gear={} pattern={} target_phase_abs={:.2f} deg".format(
                    start_request.initial_gear,
                    start_request.pattern_id,
                    start_request.target_phase_abs_deg,
                )
            )

            try:
                # START只用于首轮。DONE后由Jetson发RESCAN，
                # FPGA不再发START，所以在此保留pattern_id循环。
                next_initial_gear = start_request.initial_gear
                while True:
                    result = controller.run_once(
                        initial_gear=next_initial_gear
                    )
                    controller.send_final_frequency(
                        result
                    )
                    fine_ok = controller.run_fine_sweep_session(
                        result
                    )
                    if not fine_ok:
                        print(
                            "[PROTOCOL] fine sweep did not converge; "
                            "DONE not sent",
                            flush=True,
                        )
                        break

                    phase_ok = True
                    if not args.disable_phase_compensation:
                        phase_ok = False
                        phase_attempt = 0
                        # FPGA此时已在调相状态，失败后不能发
                        # MEASURE_FAILED并回去等START，否则双方会互等。
                        # 保持当前协议状态，重新从SET_PHASE=0测量。
                        while not phase_ok:
                            controller.check_abort()
                            phase_attempt += 1
                            print(
                                "[PHASE SESSION] attempt={} pattern={}".format(
                                    phase_attempt,
                                    start_request.pattern_id,
                                ),
                                flush=True,
                            )
                            try:
                                phase_ok = controller.run_phase_compensation(
                                    start_request
                                )
                            except RuntimeError as error:
                                message = str(error)
                                if "摄像头" in message or "读取图像" in message:
                                    raise
                                print(
                                    "[PHASE RETRY] measurement failed: {}".format(
                                        message
                                    ),
                                    flush=True,
                                )
                            if not phase_ok:
                                print(
                                    "[PHASE RETRY] stay in phase state; "
                                    "MEASURE_FAILED not sent",
                                    flush=True,
                                )
                                time.sleep(0.5)

                    controller.send_done()
                    # 所有图形锁相并完成DONE握手后，至少保持6秒；
                    # 保护期内禁止因画面变化发送RESCAN，START仍可抢占。
                    controller.wait_for_external_frequency_change(
                        holdoff_seconds=DONE_RESCAN_HOLDOFF_SECONDS
                    )
                    controller.send_rescan()

                    # RESCAN ACK表示FPGA已切回gear0锯齿波。
                    # 不等START，立即开始新一轮完整粗测。
                    next_initial_gear = 0
            except StartPreempted as error:
                # check_abort/精扫等流程已取出该START，放回队首，
                # 让外层立即按新pattern和initial_gear开始。
                link.backlog.appendleft(error.frame)
                print(
                    "[START PREEMPT] current workflow stopped; "
                    "new START seq={} has highest priority".format(
                        error.frame.sequence
                    ),
                    flush=True,
                )
            except MeasurementAborted as error:
                print("[ABORT]", error)
            except RemoteAborted as error:
                print("[ABORT]", error)
            except ResponseTimeout as error:
                print("[UART TIMEOUT]", error)
                print(
                    "[UART RX]",
                    link.parser_diagnostics(),
                )
                controller.report_failure(
                    FAIL_INTERNAL
                )
            except ProtocolError as error:
                print("[UART ERROR]", error)
            except RuntimeError as error:
                message = str(error)
                print("[FAILED]", message)

                if "量程判断" in message:
                    reason = FAIL_RANGE_TIMEOUT
                elif "摄像头" in message:
                    reason = FAIL_CAMERA
                else:
                    reason = FAIL_MEASURE_TIMEOUT

                controller.report_failure(
                    reason
                )
            except Exception as error:
                print(
                    "[INTERNAL]",
                    repr(error),
                )
                controller.report_failure(
                    FAIL_INTERNAL
                )

    except KeyboardInterrupt:
        print("\n[STOP] 用户停止")
        return 130
    except ProtocolError as error:
        print("[UART ERROR]", error)
        return 2
    except Exception as error:
        print("[ERROR]", repr(error))
        return 1
    finally:
        if controller is not None:
            controller.close()

        if link is not None:
            link.close()

    return 0


if __name__ == "__main__":
    sys.exit(main())
