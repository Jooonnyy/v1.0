#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""相机成像参数实时调参台：曝光 / 增益 / 白平衡，边看画面边调。

为什么需要它：v4l2-ctl 是命令行，改一个值是"设一下→跑程序看效果→退出→再设"，
调一轮曝光要来回十几次。这个工具把参数做成滑条，画面实时反馈，
鼠标指到哪就显示那个像素的 HSV，还能直接看到过曝比例。

用法：
    python3 cam_tune.py                      # 默认相机 0
    python3 cam_tune.py --camera /dev/cam_qr
    python3 cam_tune.py --exposure 80        # 指定初始曝光

没有 v4l2-ctl 的机器（Windows）也能跑：画面实时出，滑条只改显示值、不下发，
曝光请在相机自带软件里调。树莓派上才是真正调 v4l2。

窗口里的滑条：
    exposure        曝光时间，单位 100us。60 = 6ms。**这是最该调的那个**
    gain            增益。优先用曝光，光圈不够再加增益（增益会放大噪声）
    wb_temp         白平衡色温，单位 K。偏低偏蓝，偏高偏黄

按键：
    s   把当前参数写成 camera_locked.sh（下次插拔后直接跑它）
    r   重新从相机读回当前值（滑条会和实际对齐）
    q   退出（ESC 也行）

鼠标：在画面上移动，左上角实时显示该点的 HSV 和 BGR。

判据（画面右上角）：V 顶到 255 的比例应当 **低于 5%**；
超过 15% 就是过曝，红色会发白、色相失真，必须降曝光。
"""

import argparse
import os
import shutil
import subprocess
import sys
import time

import numpy as np

from common import (DEFAULT_CAMERA, Preview, close_all_windows, open_camera,
                    parse_camera_source)

CLIP_WARN_PERMILLE = 50      # V=255 的像素占比超过 5% 就标红
CLIP_BAD_PERMILLE = 150      # 超过 15% 视为过曝
APPLY_INTERVAL_S = 0.15      # 两次下发 v4l2 控制的最小间隔，避免拖滑条时刷爆

CTRL_DEFAULTS = {
    "exposure": 60,       # 100us
    "gain": 0,
    "wb_temp": 4600,
}


def v4l2(device, *settings):
    """调 v4l2-ctl 下发控制。找不到命令或相机不支持就静默跳过。"""
    if shutil.which("v4l2-ctl") is None:
        return False
    command = ["v4l2-ctl", "-d", device]
    for setting in settings:
        command += ["-c", setting]
    try:
        subprocess.run(command, check=False, timeout=2,
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        return True
    except (OSError, subprocess.SubprocessError):
        return False


def lock_manual(device):
    """先关掉自动曝光和自动白平衡，否则滑条设了也会被相机自己改回去。"""
    v4l2(device,
         "auto_exposure=1",
         "white_balance_automatic=0",
         "exposure_dynamic_framerate=0")


def list_controls(device):
    """列出相机支持的控件名，用来提前告诉用户哪个滑条是无效的。"""
    if shutil.which("v4l2-ctl") is None:
        return set()
    try:
        result = subprocess.run(["v4l2-ctl", "-d", device, "--list-ctrls"],
                                check=False, timeout=3,
                                stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)
    except (OSError, subprocess.SubprocessError):
        return set()
    names = set()
    for line in result.stdout.decode("utf-8", "ignore").splitlines():
        parts = line.split()
        if parts:
            names.add(parts[0])
    return names


def report_controls(device):
    """启动时体检：不支持的控件提前说明，免得拖了半天滑条没反应还以为程序坏了。"""
    supported = list_controls(device)
    if not supported:
        return
    wanted = [("auto_exposure", "自动曝光开关"),
              ("exposure_time_absolute", "曝光时间"),
              ("gain", "增益"),
              ("white_balance_automatic", "自动白平衡开关"),
              ("white_balance_temperature", "白平衡色温")]
    missing = [(name, label) for name, label in wanted if name not in supported]
    if missing:
        print("注意：这台相机不支持下列控件，对应的滑条不起作用 ——")
        for name, label in missing:
            print(f"  {name}（{label}）")
        print("曝光那一条如果不在列表里，只能靠增益和补光灯亮度来调画面。")
        print("")


def open_camera_window(device, width_px, height_px):
    """相机交给 common.open_camera：不锁曝光，否则滑条调完又被锁回固定值。"""
    return open_camera(parse_camera_source(device), width_px, height_px,
                       lock_exposure=False)


class Tuning(object):
    """滑条状态 + 节流下发。"""

    def __init__(self, device, initial):
        self.device = device
        self.exposure = int(initial["exposure"])
        self.gain = int(initial["gain"])
        self.wb_temp = int(initial["wb_temp"])
        self.last_apply_s = 0.0
        self.dirty = True

    def snapshot(self):
        return {"exposure": self.exposure, "gain": self.gain,
                "wb_temp": self.wb_temp}

    def maybe_apply(self):
        """拖滑条时高频回调，这里节流，避免每帧都 spawn 一个 v4l2-ctl。"""
        now_s = time.time()
        if not self.dirty or (now_s - self.last_apply_s) < APPLY_INTERVAL_S:
            return
        self.dirty = False
        self.last_apply_s = now_s
        v4l2(self.device,
             f"exposure_time_absolute={max(1, self.exposure)}",
             f"gain={self.gain}",
             f"white_balance_temperature={self.wb_temp}")

    def write_script(self, path):
        values = self.snapshot()
        lines = [
            "#!/bin/bash",
            "# 由 cam_tune.py 生成：把当时调好的相机参数重新套回去。",
            "# 每次 USB 插拔或重启后都要跑一次（v4l2 设置不跨插拔保存）。",
            "",
            f"DEV=${{DEV:-{self.device}}}",
            "v4l2-ctl -d \"$DEV\" \\",
            "    -c auto_exposure=1 \\",
            "    -c exposure_dynamic_framerate=0 \\",
            "    -c white_balance_automatic=0 \\",
            f"    -c exposure_time_absolute={max(1, values['exposure'])} \\",
            f"    -c gain={values['gain']} \\",
            f"    -c white_balance_temperature={values['wb_temp']}",
            "",
            "v4l2-ctl -d \"$DEV\" --get-ctrl "
            "auto_exposure,exposure_time_absolute,gain,white_balance_automatic",
            "",
            f"echo \"已套用：曝光 {max(1, values['exposure'])} (x100us)，"
            f"增益 {values['gain']}，白平衡 {values['wb_temp']}K\"",
        ]
        with open(path, "w", encoding="utf-8", newline="\n") as handle:
            handle.write("\n".join(lines) + "\n")
        os.chmod(path, 0o755)
        return path


def draw_overlay(frame, stats, tuning, cursor):
    import cv2

    height_px = frame.shape[0]
    cv2.rectangle(frame, (0, 0), (frame.shape[1], 92), (0, 0, 0), -1)

    line1 = (f"exposure={max(1, tuning.exposure)}x100us  gain={tuning.gain}  "
             f"wb={tuning.wb_temp}K")
    cv2.putText(frame, line1, (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                (0, 255, 0), 2)

    clip = stats["clip_permille"]
    color = (0, 255, 0)
    if clip >= CLIP_BAD_PERMILLE:
        color = (0, 0, 255)
    elif clip >= CLIP_WARN_PERMILLE:
        color = (0, 255, 255)
    line2 = (f"meanV={stats['mean_v']:.0f}  meanS={stats['mean_s']:.0f}  "
             f"clip(V=255)={clip / 10:.1f}%")
    cv2.putText(frame, line2, (8, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2)

    if clip >= CLIP_BAD_PERMILLE:
        cv2.putText(frame, "OVEREXPOSED - lower exposure", (8, 76),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 255), 2)
    elif stats["mean_v"] < 40:
        cv2.putText(frame, "TOO DARK - raise exposure", (8, 76),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)

    if cursor is not None:
        h, s, v, b, g, r = cursor
        text = f"H{h:3d} S{s:3d} V{v:3d}  B{b:3d} G{g:3d} R{r:3d}"
        cv2.putText(frame, text, (8, height_px - 12),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)
    return frame


def main(argv=None):
    parser = argparse.ArgumentParser(description="相机成像参数实时调参台")
    parser.add_argument("--camera", default=DEFAULT_CAMERA,
                        help="相机序号或设备路径")
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--exposure", type=int, default=CTRL_DEFAULTS["exposure"])
    parser.add_argument("--gain", type=int, default=CTRL_DEFAULTS["gain"])
    parser.add_argument("--wb-temp", type=int, default=CTRL_DEFAULTS["wb_temp"])
    parser.add_argument("--out", default="camera_locked.sh",
                        help="按 s 时写出的脚本名")
    args = parser.parse_args(argv)

    try:
        import cv2
    except ImportError:
        print("缺少 opencv：pip3 install opencv-python numpy")
        return 2

    # 没有 v4l2-ctl（Windows）也照样出画面：滑条只改显示值，不下发。
    has_v4l2 = shutil.which("v4l2-ctl") is not None
    # v4l2-ctl 只认设备文件：给序号时补成 /dev/videoN，否则 Pi 上 -d 0 会失败。
    v4l2_device = args.camera.strip()
    if v4l2_device.isdigit():
        v4l2_device = f"/dev/video{v4l2_device}"
    if has_v4l2:
        lock_manual(v4l2_device)
        report_controls(v4l2_device)
    else:
        print("找不到 v4l2-ctl（Windows 上正常）：画面照看，滑条不会真的下发。")
        print("要改曝光用相机自带软件；真实调 v4l2 请到树莓派上跑。")
        print("")

    capture = open_camera_window(args.camera, args.width, args.height)
    if capture is None:
        print(f"打不开相机 {args.camera}（PC 上先试 0 / 1）")
        return 1

    tuning = Tuning(v4l2_device, {"exposure": args.exposure,
                                  "gain": args.gain,
                                  "wb_temp": args.wb_temp})

    window = "cam_tune"
    preview = Preview(window)
    if not preview.enabled:
        print("cam_tune 需要图形界面，退出。")
        capture.release()
        return 1

    def on_exposure(value):
        tuning.exposure = max(1, value)
        tuning.dirty = True

    def on_gain(value):
        tuning.gain = value
        tuning.dirty = True

    def on_wb(value):
        tuning.wb_temp = value
        tuning.dirty = True

    # 曝光滑条上限 5000 x100us = 500ms，覆盖大多数 UVC 相机的量程
    cv2.createTrackbar("exposure", window, min(5000, tuning.exposure), 5000, on_exposure)
    cv2.createTrackbar("gain", window, min(100, tuning.gain), 100, on_gain)
    # 色温 2800..6500K，滑条只能从 0 起，这里加偏移
    cv2.createTrackbar("wb_temp-2800", window,
                       max(0, min(3700, tuning.wb_temp - 2800)), 3700,
                       lambda value: on_wb(2800 + value))

    cursor_point = [None]

    def on_mouse(event, x, y, flags, param):
        # 回调里只记坐标。颜色要等下一帧读了像素才知道，不在这里取。
        if event in (cv2.EVENT_MOUSEMOVE, cv2.EVENT_LBUTTONDOWN):
            cursor_point[0] = (x, y)

    cv2.setMouseCallback(window, on_mouse)

    print(f"相机 {args.camera}  滑条调参中。s=保存 r=重读 q=退出")
    print("判据：屏幕右上角 clip(V=255) 低于 5%；红色物料的 V 建议落在 100~180。")

    try:
        while True:
            ok, frame = capture.read()
            if not ok or frame is None:
                time.sleep(0.02)
                continue

            tuning.maybe_apply()

            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            value_channel = hsv[:, :, 2]
            stats = {
                "mean_v": float(value_channel.mean()),
                "mean_s": float(hsv[:, :, 1].mean()),
                "clip_permille": int(1000 * np.count_nonzero(value_channel >= 250)
                                     / value_channel.size),
            }

            sample = None
            if cursor_point[0] is not None:
                cx, cy = cursor_point[0]
                if 0 <= cy < frame.shape[0] and 0 <= cx < frame.shape[1]:
                    b, g, r = frame[cy, cx]
                    h, s, v = hsv[cy, cx]
                    sample = (int(h), int(s), int(v), int(b), int(g), int(r))

            draw_overlay(frame, stats, tuning, sample)
            key = preview.show(frame)
            if preview.quit:
                break
            if key == ord("r"):
                tuning.dirty = True
                print("重新下发当前滑条值")
            if key == ord("s"):
                path = tuning.write_script(args.out)
                print(f"已写出 {path}")
                print(f"  exposure={max(1, tuning.exposure)} "
                      f"gain={tuning.gain} wb={tuning.wb_temp}K")
    except KeyboardInterrupt:
        print("\n停止")
    finally:
        capture.release()
        preview.close()
        close_all_windows()
    return 0


if __name__ == "__main__":
    sys.exit(main())
