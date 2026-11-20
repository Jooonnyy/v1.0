#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""视觉链路公共件：协议编码、五点单应换算、颜色掩膜、相机与实时画面。

坐标系约定（全工程统一，改之前先改这里和 Docs/视觉方案.md）：
    车体坐标系：x 指向车头前方，y 指向车体左侧，z 向上，
                原点在**车体中心在地面的投影**，单位 mm。
    相机坐标系：OpenCV 惯例，x 向右、y 向下、z 朝前。
    平面模型：像素 ↔ 车体毫米由 calib_plane_h.py 的五点单应一次性求出。
              对准是闭环，单平面单应就够；要区分高度再另说。

每个脚本都通过 Preview 出实时画面：默认开，没有图形界面时自动降级，不影响主逻辑。
"""

import json
import math
import os

import numpy as np

# 与固件 BSP/Inc/board_vision.h 的宏一一对应，改一处必须改另一处。
VD_TARGET_LIMIT_MM = 2000
VD_YAW_LIMIT_MDEG = 180000
VD_CONF_MAX_PERMILLE = 1000
VD_AGE_LIMIT_MS = 2000

# 目标类别，与任务码的颜色编号保持一致。
CLASS_NONE = 0
CLASS_COLORS = {1: "red", 2: "yellow", 3: "blue", 4: "green", 5: "black", 6: "lightblue"}
CLASS_TRAY = 10
CLASS_RING = 11

# 颜色掩膜的形态学核尺寸，单位 px。vision_node 和 align_send 必须用同一个值，
# 否则同一块料在两边一个检得出来、一个检不出来。
MASK_KERNEL_PX = 5

# 相机默认用序号 0：Windows / PC 上直接跑就能出画面。
# 树莓派上换成 udev 固定过的设备名（/dev/cam_target），别用 /dev/videoN——编号会随插拔顺序变。
DEFAULT_CAMERA = "0"

# 相机相对车头的安装角，单位度，逆时针为正。0 = 相机正装（画面正上方就是车头）。
# 本车 2026-11-16 由 TRACK 绕圈日志反推为 +90°：目标在画面里始终绕中心转、
# 半径不收敛，就是"指令方向"和"车实际走的方向"差了 90°。
CAMERA_MOUNT_DEG = 90.0

# 本文件所在目录（= Tools/vision）。数据文件都以它为基准，见 data_path()。
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))


# ---------------------------------------------------------------------------
# 协议
# ---------------------------------------------------------------------------

def crc8(data: bytes) -> int:
    """CRC-8/ATM：多项式 0x07，初值 0x00，不反射、不异或输出。"""
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


def format_line(seq, cls, x_mm, y_mm, yaw_mdeg, conf_permille, age_ms) -> bytes:
    """拼一行 $V,...*CR\\n。越界的字段直接夹到合法范围，不抛异常。"""
    x_mm = int(max(-VD_TARGET_LIMIT_MM, min(VD_TARGET_LIMIT_MM, x_mm)))
    y_mm = int(max(-VD_TARGET_LIMIT_MM, min(VD_TARGET_LIMIT_MM, y_mm)))
    yaw_mdeg = int(max(-VD_YAW_LIMIT_MDEG, min(VD_YAW_LIMIT_MDEG, yaw_mdeg)))
    conf_permille = int(max(0, min(VD_CONF_MAX_PERMILLE, conf_permille)))
    age_ms = int(max(0, min(VD_AGE_LIMIT_MS, age_ms)))
    body = f"V,{int(seq)},{int(cls)},{x_mm},{y_mm},{yaw_mdeg},{conf_permille},{age_ms}"
    checksum = crc8(body.encode("ascii"))
    return f"${body}*{checksum:02X}\n".encode("ascii")


def normalize_yaw_mdeg(yaw_mdeg: float) -> int:
    """把角度归到 (-180000, 180000] 的 0.001 度。"""
    yaw = int(round(yaw_mdeg))
    while yaw <= -VD_YAW_LIMIT_MDEG:
        yaw += 2 * VD_YAW_LIMIT_MDEG
    while yaw > VD_YAW_LIMIT_MDEG:
        yaw -= 2 * VD_YAW_LIMIT_MDEG
    return yaw


# ---------------------------------------------------------------------------
# 文件读写
# ---------------------------------------------------------------------------

def data_path(path, for_write=False):
    """把 colors.json / calib_*.json 这类相对路径统一落到脚本目录。

    从哪个目录运行都不影响结果：PC 上双击、在仓库根敲 python、Thonny 里按 F5，
    读写的都是同一份数据文件。绝对路径原样返回。
    """
    if not path:
        return path
    if os.path.isabs(path):
        return path
    if not for_write and os.path.exists(path):
        return path          # 当前目录里就有，沿用旧用法
    return os.path.join(SCRIPT_DIR, path)


def load_json(path, default=None):
    path = data_path(path)
    if not os.path.exists(path):
        return default
    with open(path, "r", encoding="utf-8") as handle:
        return json.load(handle)


def save_json(path, data):
    path = data_path(path, for_write=True)
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    with open(path, "w", encoding="utf-8") as handle:
        json.dump(data, handle, indent=2, ensure_ascii=False)
    return path


# ---------------------------------------------------------------------------
# 颜色掩膜
# ---------------------------------------------------------------------------

def color_mask(frame_hsv, entry):
    """按 colors.json 的 HSV 区间做掩膜（一段或两段，红色会跨 0）。

    这一类没写 hsv_ranges（例如 mode=dark 的黑色）时返回 None，
    由调用方决定是跳过还是走自己的判据。
    """
    import cv2

    mask = None
    for low, high in entry.get("hsv_ranges") or []:
        current = cv2.inRange(frame_hsv, np.array(low, np.uint8), np.array(high, np.uint8))
        mask = current if mask is None else cv2.bitwise_or(mask, current)
    if mask is None:
        return None
    kernel = np.ones((MASK_KERNEL_PX, MASK_KERNEL_PX), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    return cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)


# ---------------------------------------------------------------------------
# 五点单应（像素 ↔ 车体毫米）
# ---------------------------------------------------------------------------

def load_homography(path):
    """读 calib_plane_h.json。没有文件或字段不对返回 None。"""
    data = load_json(path)
    if not data or "homography" not in data:
        return None
    return np.array(data["homography"], dtype=np.float64).reshape(3, 3)


def pixel_to_plane_h(u, v, homography):
    """像素 → 车体平面 mm（单应版）。H 的第三维为 0 时返回 None。"""
    projected = homography @ np.array([u, v, 1.0], dtype=np.float64)
    if abs(projected[2]) < 1e-9:
        return None
    return float(projected[0] / projected[2]), float(projected[1] / projected[2])


def plane_h_to_pixel(x_mm, y_mm, homography):
    """毫米 → 像素（单应版的逆），画校验网格用。"""
    inverse = np.linalg.inv(homography)
    projected = inverse @ np.array([x_mm, y_mm, 1.0], dtype=np.float64)
    if abs(projected[2]) < 1e-9:
        return None
    return float(projected[0] / projected[2]), float(projected[1] / projected[2])


# ---------------------------------------------------------------------------
# 相机安装角：相机系 → 车体系
# ---------------------------------------------------------------------------

def rotate_to_body(x_mm, y_mm, mount_deg):
    """把"相机坐标系"的目标位置转成车体坐标系（x 前、y 左），单位 mm。

    相机相对车头逆时针装了 mount_deg 度时，视觉按"画面正上方=车头"算出来的
    (x, y) 相当于把车体坐标转了 -mount_deg，这里转回来：body = R(mount_deg) @ raw。
    mount_deg 为 0 时原样返回，不做浮点运算。

    怎么判断装了多少：把物料放在车头正前方 300 mm 处看打印的 x/y——
    正常应是 x≈+300、y≈0；如果变成 x≈0、y≈±300，就是 90° 装歪了，
    符号看 y：y=+300（物料显示在左边）→ 用 +90；y=-300 → 用 -90。
    """
    if not mount_deg:
        return x_mm, y_mm
    theta = math.radians(mount_deg)
    cos_t = math.cos(theta)
    sin_t = math.sin(theta)
    return (x_mm * cos_t - y_mm * sin_t,
            x_mm * sin_t + y_mm * cos_t)


# ---------------------------------------------------------------------------
# 相机
# ---------------------------------------------------------------------------

def parse_camera_source(value):
    """把命令行给的相机参数转成 open_camera 能用的值。

    纯数字（"0" / 1）当序号，其余当设备路径（如 /dev/cam_target）。
    双相机时必须用路径：树莓派 /dev/videoN 的编号会随插拔顺序变。
    """
    if isinstance(value, int):
        return value
    text = str(value).strip()
    if text.lstrip("+-").isdigit():
        return int(text)
    return text


def open_camera(source, width_px=640, height_px=480, lock_exposure=True,
                fourcc=None, fps=None, exposure=120, wb=None):
    """打开相机。source 是序号（int）或设备路径（str）。失败返回 None。

    设备路径必须显式指定 CAP_V4L2，否则 OpenCV 会把它当成视频文件去读。
    只在 V4L2 后端下锁手动曝光/白平衡 —— Windows 的 MSMF/DSHOW 量纲完全不同，
    硬设会直接把画面弄黑，那就违背了"每台电脑都能看到实时画面"。
    """
    import cv2

    # 设备路径在树莓派上直接走 V4L2：默认后端会先试 GStreamer，失败时刷一堆 warning。
    if isinstance(source, str):
        capture = cv2.VideoCapture(source, cv2.CAP_V4L2)
    else:
        capture = cv2.VideoCapture(source)
    if not capture.isOpened():
        capture.release()
        capture = cv2.VideoCapture(source)
    if not capture.isOpened():
        capture.release()
        return None

    # FOURCC 要在分辨率之前设：UVC 切格式会重置分辨率。
    if fourcc:
        capture.set(cv2.CAP_PROP_FOURCC,
                    cv2.VideoWriter_fourcc(*str(fourcc)[:4].ljust(4)))
    capture.set(cv2.CAP_PROP_FRAME_WIDTH, width_px)
    capture.set(cv2.CAP_PROP_FRAME_HEIGHT, height_px)
    if fps:
        capture.set(cv2.CAP_PROP_FPS, fps)
    # 只留最新一帧：处理慢了就丢旧帧，换算出来的坐标才不会越来越滞后。
    capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    if lock_exposure:
        lock_manual_exposure(capture, exposure, wb)
    else:
        unlock_auto_exposure(capture)
    return capture


def unlock_auto_exposure(capture):
    """切回自动曝光/自动白平衡；只对 V4L2 生效。"""
    import cv2

    try:
        backend = capture.getBackendName()
    except Exception:
        backend = ""
    if "V4L2" not in backend:
        return
    capture.set(cv2.CAP_PROP_AUTO_EXPOSURE, 3)   # 3 = aperture priority (auto)
    capture.set(cv2.CAP_PROP_AUTO_WB, 1)


def lock_manual_exposure(capture, exposure=120, wb=None):
    """关掉自动曝光/自动白平衡。颜色不稳的头号原因就是它们。

    只对 V4L2 后端生效：1 = 手动模式、曝光单位 100us。
    这台相机不认这组属性时 set() 返回 False，忽略即可。
    """
    import cv2

    try:
        backend = capture.getBackendName()
    except Exception:
        backend = ""
    if "V4L2" not in backend:
        return
    capture.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)
    capture.set(cv2.CAP_PROP_EXPOSURE, exposure)
    capture.set(cv2.CAP_PROP_AUTO_WB, 0)
    if wb:
        capture.set(cv2.CAP_PROP_WB_TEMPERATURE, wb)
    capture.set(cv2.CAP_PROP_GAIN, 0)


# ---------------------------------------------------------------------------
# 串口（$V 发送）
# ---------------------------------------------------------------------------

# 常见 USB 转串口芯片的 VID：CH340/CH341、CP210x、FTDI、PL2303、STM32 VCP、
# Arduino / 树莓派 Pico 的 CDC。
USB_SERIAL_VIDS = (0x1A86, 0x10C4, 0x0403, 0x067B, 0x0483, 0x2341, 0x2E8A, 0x303A)


def list_serial_candidates():
    """列出可能"就是我们的那根线"的串口，外接 USB 串口排前面。

    蓝牙串口要排掉：Windows 上配对过的手机会占一个 COM，自动挑到它比不挑更糟——
    发出去没人接，还会以为固件坏了。
    """
    try:
        from serial.tools import list_ports
    except ImportError:
        return []

    candidates = []
    for port in list_ports.comports():
        description = f"{port.description or ''} {port.hwid or ''}".lower()
        if "bluetooth" in description or "蓝牙" in description:
            continue
        candidates.append(port)

    def rank(port):
        if port.vid in USB_SERIAL_VIDS:
            return 0      # CH340 / CP210x / FTDI / STM32 VCP，基本就是它
        if port.vid is not None:
            return 1      # 其它 USB 设备带的串口
        return 2          # 主板自带的 COM1 / 通信端口，最后才考虑

    return sorted(candidates, key=rank)


def pick_serial_candidates(explicit=None):
    """按优先级列出要尝试的串口：显式给的只有一个；否则 /dev/serial0 + 扫到的 USB 串口。"""
    if explicit:
        return [explicit]
    candidates = []
    if os.name != "nt" and os.path.exists("/dev/serial0"):
        candidates.append("/dev/serial0")
    for port in list_serial_candidates():
        if port.device not in candidates:
            candidates.append(port.device)
    return candidates


def open_link(port=None, baud=115200):
    """打开 $V 发送串口。**默认自动选口**；挑不上就返回 None（只打印、不发送）。

    自动模式会按 `pick_serial_candidates()` 的顺序逐个试开——Windows 上常有一堆
    "通信端口 (COM11)" 这种打不开的幽灵口，只试第一个就报错会让人以为线接错了。
    接错了用 `--port COM7` / `--port /dev/serial0` 指定；不想发就加 `--no-send`。
    """
    try:
        import serial
    except ImportError:
        print("缺少 pyserial（pip3 install pyserial），不发串口，只打印帧")
        return None

    candidates = pick_serial_candidates(port)
    if not candidates:
        print("没找到可用串口：接上 USB-TTL，或用 --port COM7 / --port /dev/serial0 指定")
        return None

    errors = []
    for device in candidates:
        try:
            link = serial.Serial(device, baud, timeout=0.05)
        except Exception as error:
            errors.append(f"{device}: {error}")
            continue
        print(f"串口已打开 {device} @{baud}")
        return link

    if len(candidates) == 1:
        print(f"打开串口 {candidates[0]} 失败：{errors[0].split(': ', 1)[-1]}")
    else:
        print("串口都打不开：" + "；".join(errors))
    print("接好 USB-TTL 后用 --port COMx / --port /dev/serial0 指定")
    return None


def add_send_argument(parser):
    """给脚本加 --no-send：默认发串口，加了它才只预览/打印。"""
    parser.add_argument("--no-send", dest="send", action="store_false",
                        help="不发串口，只出画面/打印帧（默认发）")
    parser.set_defaults(send=True)


# 固件 → 视觉的类别指令：`$C,<cls>\n`（1..6 颜色、10 载物盘、11 色环）。
CLS_HINT_PREFIX = b"$C,"


def parse_class_hint_full(raw):
    """解析 `$C,<cls>[,<idx>[,<rot_deg>[,<stage>]]]` → (cls, idx, rot, stage)。

    idx = 目标序号，rot = 转台角，stage = 0 顶部粗对 / 1 底部精对。
    没有的字段返回 0 / None。
    """
    raw = raw.strip()
    if not raw.startswith(CLS_HINT_PREFIX):
        return None
    fields = raw[len(CLS_HINT_PREFIX):].split(b",")
    try:
        cls = int(fields[0].strip())
        index = int(fields[1].strip()) if len(fields) > 1 else 0
        rot = None
        if len(fields) > 2:
            rot = int(float(fields[2].strip()))
        stage = None
        if len(fields) > 3:
            stage = int(fields[3].strip())
    except ValueError:
        return None
    if not 1 <= cls <= 11:
        return None
    return cls, index, rot, stage


def parse_class_hint(raw):
    """兼容旧调用：只要 (cls, idx)。"""
    parsed = parse_class_hint_full(raw)
    if parsed is None:
        return None
    cls, index, _rot, _stage = parsed
    return cls, index


def poll_class_hint_ex(link, buffer):
    """非阻塞读下行，解析 `$C,<cls>[,<idx>]`。返回 (cls, idx, 新 buffer)。

    没有 link、没数据、或不是 $C 行都返回 (None, 0, buffer)；一次收到多行时取最后一行。
    """
    if link is None:
        return None, 0, buffer
    try:
        pending = link.in_waiting
    except Exception:
        return None, 0, buffer
    if pending:
        try:
            buffer += link.read(pending)
        except Exception:
            return None, 0, buffer

    hint = None
    index = 0
    while b"\n" in buffer:
        raw, buffer = buffer.split(b"\n", 1)
        parsed = parse_class_hint(raw)
        if parsed is not None:
            hint, index = parsed
    if len(buffer) > 128:      # 半截垃圾不无限攒
        buffer = buffer[-128:]
    return hint, index, buffer


def poll_class_hint(link, buffer):
    """兼容旧调用：只要 cls，不要序号。"""
    hint, _index, buffer = poll_class_hint_ex(link, buffer)
    return hint, buffer


# ---------------------------------------------------------------------------
# 实时画面
# ---------------------------------------------------------------------------

def add_preview_argument(parser):
    """给脚本加上"默认开画面、--no-preview 关掉"的开关。"""
    parser.add_argument("--no-preview", dest="preview", action="store_false",
                        help="关掉实时画面窗口（默认开）")
    parser.set_defaults(preview=True)


class Preview:
    """实时画面窗口。所有脚本共用，保证"跑起来就能在电脑上看到摄像头画面"。

    用法：
        preview = Preview("vision_node")
        key = preview.show(frame)     # 返回按键，-1 表示这次没按 / 没有窗口
        if preview.quit:              # 用户按了 q 或 ESC
            break
        preview.close()

    没有图形界面（纯 ssh）时只打印一行提示，其余逻辑照常跑，不抛异常。
    """

    def __init__(self, title):
        self.title = title
        self.enabled = False
        self.quit = False
        try:
            import cv2
            cv2.namedWindow(title, cv2.WINDOW_AUTOSIZE)
            self.enabled = True
        except Exception:
            print(f"没有图形界面，{title} 不开窗（串口/统计照常跑）")

    def show(self, frame, wait_ms=1):
        if not self.enabled:
            return -1
        import cv2

        try:
            cv2.imshow(self.title, frame)
            key = cv2.waitKey(wait_ms) & 0xFF
        except cv2.error:
            self.enabled = False
            print("实时画面窗口出错，已关闭窗口继续跑")
            return -1
        if key in (ord("q"), 27):
            self.quit = True
        return key

    def close(self):
        if not self.enabled:
            return
        import cv2

        self.enabled = False
        try:
            cv2.destroyWindow(self.title)
        except cv2.error:
            pass


def close_all_windows():
    """关掉所有 OpenCV 窗口。没有 opencv / 没有图形界面时静默跳过。"""
    try:
        import cv2
    except ImportError:
        return

    try:
        cv2.destroyAllWindows()
    except cv2.error:
        pass
