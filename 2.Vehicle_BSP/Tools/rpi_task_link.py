#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
工创赛小车：二维码任务码 -> 串口 -> STM32 -> TJC 串口屏

链路：
    Pi 摄像头/图片 --(decode)--> 任务码 --USART3/115200--> STM32 --USART6/115200--> 屏

协议（ASCII 单行，\\n 结尾）：
    TASK 156+123+516+231   ->  OK TASK      / ERR TASK FORMAT
    STAT 3 5               ->  OK STAT      / ERR STAT FORMAT/RANGE
    PING                   ->  PONG

用法：
    # 最常用：直接运行，打开摄像头 0 扫描二维码，识别到就上屏
    python3 rpi_task_link.py --port /dev/serial0

    # 不带串口，只看要发什么
    python3 rpi_task_link.py --task 156+123+516+231 --dry-run

    # 手动发任务码（PC 或 Pi）
    python3 rpi_task_link.py --port /dev/serial0 --task 156+123+516+231

    # 发比赛结束统计（抓取正确 3 次、放置正确 5 次）
    python3 rpi_task_link.py --port /dev/serial0 --stat 3 5

    # 识别一张图片里的二维码并下发
    python3 rpi_task_link.py --port /dev/serial0 --image qr.jpg

    # 摄像头连续扫描（识别到后下发；默认同码只发一次）
    python3 rpi_task_link.py --port /dev/serial0 --camera 0

依赖：
    pip3 install pyserial
    pip3 install opencv-python        # 二维码识别（cv2.QRCodeDetector）
    可选：pip3 install pyzbar         # 复杂光照/角度下更稳，需要系统 libzbar0
"""

import argparse
import re
import sys
import time

# 任务码：4 组三位数，组间 '+', 每组 1..6（颜色 1..6、圆环号 1..6）。
TASK_PATTERN = re.compile(r"(?<!\d)([1-6]{3}\+[1-6]{3}\+[1-6]{3}\+[1-6]{3})(?!\d)")

DEFAULT_BAUD = 115200
DEFAULT_TIMEOUT_S = 3.0


def validate_task(code):
    """与 STM32 端 task_code_valid() 完全一致的格式检查。"""
    if len(code) != 15:
        return False
    for index, char in enumerate(code):
        if index in (3, 7, 11):
            if char != "+":
                return False
        elif char not in "123456":
            return False
    return True


def extract_task(text):
    """从二维码原始内容里提取任务码；兼容 '任务码:156+...' 这类前缀。"""
    if not text:
        return None
    match = TASK_PATTERN.search(text)
    return match.group(1) if match else None


# ---------------------------------------------------------------------------
# 二维码识别
# ---------------------------------------------------------------------------

def decode_image(path):
    """识别图片中的二维码，返回原始字符串；失败返回 None。

    优先用 OpenCV；没装 OpenCV 时退回 Pillow + pyzbar（树莓派上更轻）。
    """
    try:
        import cv2
    except ImportError:
        cv2 = None

    if cv2 is not None:
        image = cv2.imread(path)
        if image is None:
            print(f"打不开图片：{path}")
            return None
        detector = cv2.QRCodeDetector()
        data, _points, _straight = detector.detectAndDecode(image)
        if data:
            return data

    # OpenCV 没装或没识别出来，再试 Pillow + pyzbar
    try:
        from PIL import Image
        from pyzbar.pyzbar import decode as zbar_decode
        results = zbar_decode(Image.open(path))
        if results:
            return results[0].data.decode("utf-8", "replace")
    except ImportError:
        print("缺少二维码识别库：pip3 install opencv-python 或 pip3 install pillow pyzbar")
    except Exception as error:  # zbar 在部分系统上会因库版本报错，不让它中断主流程
        print(f"pyzbar 识别失败：{error}")

    return None


def _decode_frame(frame):
    """对一帧做识别：先 cv2，失败再试 pyzbar。"""
    try:
        import cv2
        detector = cv2.QRCodeDetector()
        data, _points, _straight = detector.detectAndDecode(frame)
        if data:
            return data
    except Exception:
        pass

    try:
        from pyzbar.pyzbar import decode as zbar_decode
        results = zbar_decode(frame)
        if results:
            return results[0].data.decode("utf-8", "replace")
    except Exception:
        pass
    return None


def scan_camera(index):
    """打开摄像头持续扫描，只在任务码**变化**时 yield 一次。

    去重按“任务码内容”而不是“这一帧有没有识别到”：摄像头偶尔丢一帧、
    画面短暂反光都会让识别结果变空，但同一个码不应因此重复下发。
    Ctrl+C 结束扫描（由调用方处理）。
    """
    try:
        import cv2
    except ImportError:
        print("缺少 opencv，请先 pip3 install opencv-python")
        return

    capture = cv2.VideoCapture(index)
    if not capture.isOpened():
        # 部分树莓派系统要用 V4L2 后端才能打开 CSI 摄像头
        capture = cv2.VideoCapture(index, cv2.CAP_V4L2)
    if not capture.isOpened():
        print(f"打不开摄像头 {index}（检查排线、是否被占用、CSI 摄像头需先启用）")
        print("没有摄像头可以先用 --image qr.jpg 识别现成的二维码图片")
        return

    print("开始扫描二维码（Ctrl+C 退出）...")
    last_report_s = time.time()
    last_task = None
    try:
        while True:
            ok, frame = capture.read()
            if not ok:
                time.sleep(0.05)
                continue

            raw = _decode_frame(frame)
            if raw:
                task = extract_task(raw)
                if task:
                    if task != last_task:
                        print(f"识别到二维码：{raw!r} -> 任务码 {task}")
                        last_task = task
                        yield task
                else:
                    print(f"识别到二维码但不是合法任务码：{raw!r}")
            else:
                now_s = time.time()
                if now_s - last_report_s >= 5.0:
                    print("未识别到二维码，继续扫描...")
                    last_report_s = now_s
    finally:
        capture.release()


# ---------------------------------------------------------------------------
# 串口
# ---------------------------------------------------------------------------

def list_serial_ports():
    try:
        from serial.tools import list_ports
    except ImportError:
        return []
    return [(item.device, item.description) for item in list_ports.comports()]


def open_serial(port, baud):
    try:
        import serial
    except ImportError:
        print("缺少 pyserial，请先 pip3 install pyserial")
        return None

    if port is None:
        for candidate in ("/dev/serial0", "/dev/ttyAMA0", "/dev/ttyS0",
                          "/dev/ttyUSB0", "/dev/ttyACM0"):
            try:
                return serial.Serial(candidate, baud, timeout=0.2)
            except Exception:
                continue
        ports = list_serial_ports()
        if ports:
            print("未自动找到串口，可用端口：")
            for device, description in ports:
                print(f"  {device}  {description}")
        print("请用 --port 指定（树莓派一般是 /dev/serial0）")
        return None

    try:
        return serial.Serial(port, baud, timeout=0.2)
    except Exception as error:
        print(f"打开串口 {port} 失败：{error}")
        if sys.platform.startswith("linux"):
            print("检查权限：sudo usermod -aG dialout $USER，重新登录后再试")
        return None


def send_command(serial_port, line, expect_prefix, timeout_s):
    """发一行命令，按行读回显，直到收到期望前缀或超时。返回 (是否成功, 最后一行)。"""
    serial_port.reset_input_buffer()
    payload = (line + "\n").encode("ascii")
    serial_port.write(payload)
    serial_port.flush()
    print(f"发送：{line}")

    deadline = time.time() + timeout_s
    last_line = ""
    while time.time() < deadline:
        raw = serial_port.readline()
        if not raw:
            continue
        text = raw.decode("ascii", "replace").strip()
        if not text:
            continue
        print(f"STM32：{text}")
        last_line = text
        if expect_prefix and text.startswith(expect_prefix):
            return True, text
        if text.startswith("ERR"):
            return False, text
    return False, last_line


def wait_ready(serial_port, timeout_s):
    """可选：吃一条开机 READY，确认 STM32 固件在跑。读不到也不算失败。"""
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        raw = serial_port.readline()
        if not raw:
            continue
        text = raw.decode("ascii", "replace").strip()
        if text:
            print(f"STM32：{text}")
            if text == "READY":
                return True
    return False


# ---------------------------------------------------------------------------
# 主流程
# ---------------------------------------------------------------------------

def run_task(serial_port, task, timeout_s):
    return send_command(serial_port, f"TASK {task}", "OK TASK", timeout_s)


def run_stat(serial_port, grab_ok, place_ok, timeout_s):
    return send_command(serial_port, f"STAT {grab_ok} {place_ok}", "OK STAT", timeout_s)


def interactive_session(serial_port, timeout_s):
    """不带参数运行时进入：在终端里逐条输入任务码 / stat / ping。"""
    print("交互模式：输入任务码回车发送；stat 3 5 发统计；ping 测链路；直接回车退出。")
    while True:
        try:
            line = input("任务码> ").strip()
        except (EOFError, KeyboardInterrupt):
            print()
            return 0
        if not line or line.lower() in ("q", "quit", "exit"):
            return 0

        lowered = line.lower()
        if lowered == "ping":
            send_command(serial_port, "PING", "PONG", timeout_s)
        elif lowered.startswith("stat"):
            parts = line.split()
            if len(parts) != 3 or not parts[1].isdigit() or not parts[2].isdigit():
                print("格式：stat <抓取次数> <放置次数>，例如 stat 3 5")
                continue
            run_stat(serial_port, int(parts[1]), int(parts[2]), timeout_s)
        else:
            if not validate_task(line):
                print("任务码格式不对：应为 4 组三位数、每位 1..6，例如 156+123+516+231")
                continue
            run_task(serial_port, line, timeout_s)


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="二维码任务码 -> USART3 -> STM32 -> TJC 串口屏")
    parser.add_argument("--port", help="串口设备，树莓派通常是 /dev/serial0")
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD,
                        help=f"波特率，默认 {DEFAULT_BAUD}（必须与 BOARD_RPI_BAUD 一致）")
    parser.add_argument("--timeout", type=float, default=DEFAULT_TIMEOUT_S,
                        help="等待 STM32 应答的秒数")

    action = parser.add_mutually_exclusive_group(required=False)
    action.add_argument("--task", help="直接下发任务码，如 156+123+516+231")
    action.add_argument("--stat", nargs=2, type=int, metavar=("GRAB", "PLACE"),
                        help="下发统计：抓取正确次数 放置正确次数")
    action.add_argument("--ping", action="store_true", help="链路测试")
    action.add_argument("--image", help="识别指定图片里的二维码并下发")
    action.add_argument("--camera", type=str, metavar="DEVICE",
                        help="用摄像头连续扫描：序号或设备路径（如 0 或 /dev/cam_qr）")

    parser.add_argument("--once", action="store_true",
                        help="摄像头模式：识别到第一个任务码后退出")
    parser.add_argument("--interactive", action="store_true",
                        help="进入手动输入模式（不打开摄像头）")
    parser.add_argument("--dry-run", action="store_true", help="只打印，不打开串口")
    args = parser.parse_args(argv)

    # 1. 先确定要发的内容。格式错误不发给 STM32，直接本地报错。
    pending_task = None

    if args.task is not None:
        if not validate_task(args.task):
            print(f"任务码格式不对：{args.task!r}")
            print("应为 4 组三位数，组间 '+', 每位 1..6，例如 156+123+516+231")
            return 2
        pending_task = args.task

    if args.stat is not None:
        grab_ok, place_ok = args.stat
        if not (0 <= grab_ok <= 999 and 0 <= place_ok <= 999):
            print("统计次数应在 0..999")
            return 2

    if args.image is not None:
        raw = decode_image(args.image)
        if not raw:
            print("图片里没有识别到二维码")
            return 1
        print(f"二维码内容：{raw!r}")
        pending_task = extract_task(raw)
        if not pending_task:
            print("二维码内容里没有合法任务码（应为 4 组三位数，1..6）")
            return 1

    # 2. dry-run：只显示要发什么。
    has_action = (pending_task is not None or args.stat is not None or
                  args.ping or args.camera is not None)

    # 不带任何动作参数：直接进摄像头扫码，识别到就上屏（最常用的现场用法）。
    if not has_action and not args.interactive and not args.dry_run:
        args.camera = 0
        has_action = True
        print("未指定动作：默认用摄像头 0 扫描二维码。识别到合法任务码后自动上屏。")

    if args.dry_run:
        if pending_task:
            print(f"将发送：TASK {pending_task}")
        elif args.ping:
            print("将发送：PING")
        elif args.stat:
            print(f"将发送：STAT {args.stat[0]} {args.stat[1]}")
        elif args.camera is not None:
            print(f"将打开摄像头 {args.camera} 扫描，识别后发送 TASK <code>")
        else:
            print("将打开摄像头 0 扫描二维码，识别后发送 TASK <code>")
        return 0

    serial_port = open_serial(args.port, args.baud)
    if serial_port is None:
        return 1

    try:
        # 给 STM32 一点时间，顺手确认它是否在发 READY（不是必须）。
        wait_ready(serial_port, 0.5)

        # 只有显式 --interactive 才会走到这里。
        if not has_action:
            return interactive_session(serial_port, args.timeout)

        if args.ping:
            ok, _ = send_command(serial_port, "PING", "PONG", args.timeout)
            return 0 if ok else 1

        if args.stat is not None:
            ok, _ = run_stat(serial_port, args.stat[0], args.stat[1], args.timeout)
            return 0 if ok else 1

        if pending_task is not None:
            ok, _ = run_task(serial_port, pending_task, args.timeout)
            return 0 if ok else 1

        # 摄像头模式
        if args.camera is not None:
            sent_any = False
            try:
                for task in scan_camera(args.camera):
                    ok, _ = run_task(serial_port, task, args.timeout)
                    if ok:
                        sent_any = True
                        print("任务码已上屏")
                        if args.once:
                            return 0
                    else:
                        print("下发失败，继续扫描")
            except KeyboardInterrupt:
                print("\n停止扫描")
            return 0 if sent_any else 1

        return 2
    finally:
        serial_port.close()


if __name__ == "__main__":
    sys.exit(main())
