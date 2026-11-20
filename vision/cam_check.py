#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""确认两路 USB 相机分别是哪一台：A、B 并排显示，画面上标着分辨率和编号。

用法：
    # PC 上直接跑：默认 A=0、B=1，只有一台相机就只显示 A
    python3 cam_check.py

    # 树莓派装好 udev 规则之后
    python3 cam_check.py --a /dev/cam_target --b /dev/cam_qr

    # 没显示器（纯 ssh）：不开窗口，各存一张 cam_a.png / cam_b.png
    python3 cam_check.py --snap

按 q 或 ESC 退出。

注意：每台 UVC 相机在 /dev 下出两个节点，index0 才是采集节点，
index1 是元数据节点（能打开、但永远读不到帧）。本脚本会把这种情况单独报出来。
"""

import argparse
import sys
import time

import numpy as np

from common import close_all_windows, open_camera, parse_camera_source

BLANK_SIZE = (480, 640)   # 没画面时占位用的高、宽


def read_one(capture, tries=100):
    """连读几帧再取一张 —— 刚开流时的头几帧经常是空的或全黑。"""
    for _ in range(tries):
        ok, frame = capture.read()
        if ok and frame is not None and frame.size:
            return frame
        time.sleep(0.02)
    return None


def probe(name, text):
    """打开并试读，返回 (capture, frame)，失败返回 (None, None)。"""
    import cv2

    source = parse_camera_source(text)
    capture = open_camera(source, lock_exposure=False)
    if capture is None:
        print(f"[{name}] 打不开 {text}")
        print("      PC 上换成 0 / 1 试；树莓派看 ls -l /dev/video* 有没有这个节点，")
        print("      也可能是被别的程序（浏览器、另一个脚本）占着")
        return None, None

    frame = read_one(capture)
    if frame is None:
        print(f"[{name}] {text} 能打开，但一帧都读不到")
        print("      树莓派上这是 UVC 元数据节点（/dev/v4l/by-path/*-video-index1）的典型表现，")
        print("      换同组里 video-index0 那个节点再试；PC 上先确认相机没被别的程序占用")
        capture.release()
        return None, None

    asked_w = int(capture.get(cv2.CAP_PROP_FRAME_WIDTH))
    asked_h = int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f"[{name}] {text}  OK  实际 {frame.shape[1]}x{frame.shape[0]}"
          f"（请求 {asked_w}x{asked_h}）")
    return capture, frame


def label(frame, text):
    """在左上角写一行字，返回新图（不改原图）。"""
    import cv2

    result = frame.copy()
    cv2.rectangle(result, (0, 0), (min(result.shape[1], 40 + 12 * len(text)), 42),
                  (0, 0, 0), -1)
    cv2.putText(result, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.75,
                (0, 255, 0), 2)
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description="确认两路相机分别是哪一台")
    parser.add_argument("--a", default="0", help="相机 A：序号或设备路径")
    parser.add_argument("--b", default="1", help="相机 B：序号或设备路径")
    parser.add_argument("--snap", action="store_true",
                        help="不开窗口，各存一张 cam_a.png / cam_b.png 后退出")
    args = parser.parse_args(argv)

    try:
        import cv2
    except ImportError:
        print("缺少 opencv：pip3 install opencv-python numpy")
        return 2

    captures = {}
    for name, text in (("A", args.a), ("B", args.b)):
        capture, frame = probe(name, text)
        if capture is not None:
            captures[name] = capture
            if args.snap:
                path = f"cam_{name.lower()}.png"
                cv2.imwrite(path, frame)
                print(f"      已存 {path}")

    if args.snap:
        for capture in captures.values():
            capture.release()
        return 0 if captures else 1

    if not captures:
        print("两路都没出帧：先查接线、供电，再查 udev 规则有没有生效")
        return 1

    print("按 q 或 ESC 退出。对照画面确认哪台对着物料区、哪台对着二维码。")
    try:
        while True:
            panels = []
            for name in ("A", "B"):
                if name not in captures:
                    continue
                ok, frame = captures[name].read()
                if not ok or frame is None or not frame.size:
                    frame = np.zeros((BLANK_SIZE[0], BLANK_SIZE[1], 3), np.uint8)
                    frame = label(frame, f"{name}  no frame")
                else:
                    frame = label(frame, f"{name}  {frame.shape[1]}x{frame.shape[0]}")
                panels.append(frame)

            # 两台分辨率不同时按较矮的那张等比缩放，才能拼成一张图。
            target_h = min(panel.shape[0] for panel in panels)
            resized = []
            for panel in panels:
                if panel.shape[0] != target_h:
                    scale = target_h / float(panel.shape[0])
                    panel = cv2.resize(panel, (int(panel.shape[1] * scale), target_h))
                resized.append(panel)

            cv2.imshow("cam_check  A | B", np.hstack(resized))
            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                break
    finally:
        for capture in captures.values():
            capture.release()
        close_all_windows()
    return 0


if __name__ == "__main__":
    sys.exit(main())
