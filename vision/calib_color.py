#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""颜色阈值现场标定：对着实物采一块颜色，生成 colors.json 的 HSV 区间。

为什么必须现场标：同一块红色塑料，正午窗边和晚上日光灯下的 HSV 能差出一倍色相。
仓库里 colors.json 的默认区间只是"能跑起来"的起点，比赛前一定要用实物重标。

用法：
    # 相机取图：实时画面里按空格冻结，鼠标框住目标颜色，回车确认
    python3 calib_color.py --name 3
    python3 calib_color.py --camera /dev/cam_target --name 3

    # 用现成图片 + 坐标框（无界面时用）
    python3 calib_color.py --image card.jpg --box 120 80 60 60 --name 3

    # 一次标完 6 种颜色（重复跑，脚本会往同一个文件里叠加）
    for name in 1 2 3 4 5 6; do python3 calib_color.py --name $name; done

标完会自动开一个验证画面：绿色轮廓就是新区间检出的目标，按 q 结束。
输出：colors.json，结构见文件末尾写出的样例。
判据：同一颜色在 3 种光照下各标一次，三次区间的交集仍能覆盖实物。
"""

import argparse
import math
import sys
import time

import numpy as np

from common import (CLASS_COLORS, DEFAULT_CAMERA, Preview, add_preview_argument,
                    close_all_windows, color_mask, load_json, open_camera,
                    parse_camera_source, save_json)


def circular_range(hue_values, margin_h):
    """色相是环形的（0..179），红色会跨越 0。返回 1~2 段 [lo, hi] 区间。"""
    angles = hue_values.astype(np.float64) * (360.0 / 180.0)  # OpenCV 的 H 是 0..179
    radians = np.deg2rad(angles)
    mean_angle = math.degrees(math.atan2(np.mean(np.sin(radians)),
                                         np.mean(np.cos(radians))))
    relative = (np.degrees(radians) - mean_angle + 180.0) % 360.0 - 180.0
    spread = float(np.max(np.abs(relative))) + float(margin_h)
    low = mean_angle - spread
    high = mean_angle + spread
    # 转回 0..179 的整数，超出的部分拆成两段（红/洋红会用到）
    low_cv = (low / 2.0) % 180.0
    high_cv = (high / 2.0) % 180.0
    if spread >= 180.0:
        return [[0, 179]]
    if low_cv <= high_cv:
        return [[int(math.floor(low_cv)), int(math.ceil(high_cv))]]
    return [[0, int(math.ceil(high_cv))], [int(math.floor(low_cv)), 179]]


def describe_patch(patch_bgr, margin_h, margin_s, margin_v):
    """从一块 BGR 像素里统计 HSV 区间和 Lab 中心色。"""
    import cv2

    hsv = cv2.cvtColor(patch_bgr, cv2.COLOR_BGR2HSV)
    lab = cv2.cvtColor(patch_bgr, cv2.COLOR_BGR2LAB)

    hue = hsv[:, :, 0].reshape(-1)
    saturation = hsv[:, :, 1].reshape(-1)
    value = hsv[:, :, 2].reshape(-1)

    # 只统计"有颜色"的像素；高光/阴影里的灰点会把区间拉歪。
    keep = (saturation >= 40) & (value >= 30)
    if np.count_nonzero(keep) < 50:
        keep = np.ones_like(saturation, dtype=bool)
    hue, saturation, value = hue[keep], saturation[keep], value[keep]

    # 过曝检测：某个通道被压到 255 时色相已经失真，但算出来的区间看起来仍然正常，
    # 这是颜色标定最隐蔽的失败方式，所以单独统计出来告警。
    bgr = patch_bgr.reshape(-1, 3)
    clipped = np.count_nonzero(np.any(bgr >= 250, axis=1))
    clipped_permille = int(1000 * clipped / max(1, bgr.shape[0]))
    # 掩膜内（有颜色的像素）的 V 顶到 255 的比例，比整体更能说明目标本身是否过曝
    bright_permille = int(1000 * np.count_nonzero(value >= 250) / max(1, value.size))

    hue_ranges = circular_range(hue, margin_h)
    sat_low = max(0, int(np.percentile(saturation, 5)) - margin_s)
    sat_high = min(255, int(np.percentile(saturation, 95)) + margin_s)
    val_low = max(0, int(np.percentile(value, 5)) - margin_v)
    val_high = min(255, int(np.percentile(value, 95)) + margin_v)

    hsv_ranges = []
    for low_h, high_h in hue_ranges:
        hsv_ranges.append([[low_h, sat_low, val_low], [high_h, sat_high, val_high]])

    lab_flat = lab.reshape(-1, 3)[keep]
    return {
        "hsv_ranges": hsv_ranges,
        "lab_center": [float(np.median(lab_flat[:, 0])),
                       float(np.median(lab_flat[:, 1])),
                       float(np.median(lab_flat[:, 2]))],
        "samples": int(np.count_nonzero(keep)),
        "clipped_permille": clipped_permille,
        "bright_permille": bright_permille,
    }


def select_roi(camera_source, lock_exposure=False, exposure=120):
    """实时画面里空格冻结，鼠标框选。返回 BGR 小块，取消返回 None。"""
    import cv2

    capture = open_camera(parse_camera_source(camera_source),
                          lock_exposure=lock_exposure, exposure=exposure)
    if capture is None:
        print(f"打不开相机 {camera_source}（PC 上先试 0 / 1）")
        return None

    preview = Preview("calib_color")
    if not preview.enabled:
        capture.release()
        print("没有图形界面，没法框选；改用 --image shot.png --box X Y W H")
        return None
    frame = None
    try:
        while True:
            ok, current = capture.read()
            if not ok or current is None:
                time.sleep(0.02)
                continue
            frame = current
            view = frame.copy()
            cv2.putText(view, "space/enter = freeze & pick colour   q = quit",
                        (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            key = preview.show(view)
            if key in (32, 13):      # 空格 / 回车
                break
            if preview.quit:
                return None
        if frame is None:
            return None

        print("用鼠标框住目标颜色，回车确认，c 取消")
        box = cv2.selectROI("calib_color roi", frame, showCrosshair=True)
    finally:
        capture.release()
        preview.close()
        close_all_windows()

    x_px, y_px, width_px, height_px = [int(value) for value in box]
    if width_px <= 4 or height_px <= 4:
        return None
    return frame[max(0, y_px):y_px + height_px, max(0, x_px):x_px + width_px]


def verify_preview(camera_source, entry, name, lock_exposure=False, exposure=120):
    """把新标出来的区间套回实时画面，确认框住了目标、没把背景吃进来。"""
    import cv2

    capture = open_camera(parse_camera_source(camera_source),
                          lock_exposure=lock_exposure, exposure=exposure)
    if capture is None:
        return
    preview = Preview("calib_color  verify")
    if not preview.enabled:
        capture.release()
        return
    print("验证画面：绿色轮廓 = 新区间检出的目标，q / ESC 结束")
    try:
        while True:
            ok, frame = capture.read()
            if not ok or frame is None:
                time.sleep(0.02)
                continue
            mask = color_mask(cv2.cvtColor(frame, cv2.COLOR_BGR2HSV), entry)
            if mask is not None:
                contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                               cv2.CHAIN_APPROX_SIMPLE)
                cv2.drawContours(frame, contours, -1, (0, 255, 0), 2)
            cv2.putText(frame, f"class {name}  q = finish", (8, 24),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            preview.show(frame)
            if preview.quit:
                break
    finally:
        capture.release()
        preview.close()
        close_all_windows()


def main(argv=None):
    parser = argparse.ArgumentParser(description="颜色 HSV 区间现场标定")
    parser.add_argument("--image", help="图片路径")
    parser.add_argument("--camera", default=DEFAULT_CAMERA, help="相机序号或设备路径")
    parser.add_argument("--box", nargs=4, type=int, metavar=("X", "Y", "W", "H"),
                        help="无界面时的取样框，单位像素")
    parser.add_argument("--name", required=True,
                        help="颜色编号 1..6（与任务码一致），或 tray / ring")
    parser.add_argument("--margin-h", type=int, default=8, help="色相余量，越大越宽松")
    parser.add_argument("--margin-s", type=int, default=35, help="饱和度余量")
    parser.add_argument("--margin-v", type=int, default=35, help="亮度余量")
    parser.add_argument("--colors", default="colors.json", help="输出文件")
    parser.add_argument("--lock-exposure", action="store_true",
                        help="锁手动曝光；要和 main.py --cam-lock-exposure 一致")
    parser.add_argument("--exposure", type=int, default=120,
                        help="手动曝光值（100us 单位），仅 --lock-exposure 时用")
    add_preview_argument(parser)
    args = parser.parse_args(argv)

    try:
        import cv2
    except ImportError:
        print("缺少 opencv：pip3 install opencv-python numpy")
        return 2

    key = args.name
    if key not in ("tray", "ring") and key not in [str(number) for number in CLASS_COLORS]:
        print("--name 只能是 1..6 或 tray / ring")
        return 2

    if args.image:
        frame = cv2.imread(args.image)
        if frame is None:
            print(f"打不开图片：{args.image}")
            return 1
        if not args.box:
            print("用 --image 时必须给 --box X Y W H")
            return 2
        x_px, y_px, width_px, height_px = args.box
        patch = frame[y_px:y_px + height_px, x_px:x_px + width_px]
    else:
        patch = select_roi(args.camera, args.lock_exposure, args.exposure)

    if patch is None or patch.size == 0:
        print("没有取到有效取样块")
        return 1

    entry = describe_patch(patch, args.margin_h, args.margin_s, args.margin_v)
    entry["name"] = CLASS_COLORS.get(int(key), key) if key.isdigit() else key

    data = load_json(args.colors, default={"version": 1, "classes": {}})
    classes = data.setdefault("classes", {})
    # 重标只更新颜色相关字段，保留人工加的 min_circle_fill / min_solidity / min_area_px，
    # 否则每标一次黑色就把它的形状约束冲掉一次。
    previous = classes.get(key, {})
    merged = dict(previous)
    merged.update(entry)
    classes[key] = merged
    data["version"] = 1
    saved_path = save_json(args.colors, data)

    print(f"已写入 {saved_path} 的 class {key}（{entry['name']}）：")
    for low, high in entry["hsv_ranges"]:
        print(f"  H {low[0]:3d}..{high[0]:3d}  S {low[1]:3d}..{high[1]:3d}  V {low[2]:3d}..{high[2]:3d}")
    print(f"  Lab 中心色 {np.round(entry['lab_center'], 1).tolist()}，取样 {entry['samples']} 像素")

    # 过曝判定：目标像素里亮度顶到 255 的比例。超过 20% 基本可以确定是过曝。
    if entry["bright_permille"] >= 200:
        print(f"  !! 过曝告警：{entry['bright_permille'] / 10:.0f}% 的目标像素 V=255，"
              "色相已失真")
        print("     降低曝光（v4l2-ctl -c exposure_time_absolute=60）后重新标这个颜色。")
    elif entry["bright_permille"] >= 80:
        print(f"  提示：{entry['bright_permille'] / 10:.0f}% 的目标像素接近饱和，"
              "换光位或略降曝光会更稳")

    # 区间合理性粗检：色相跨度太宽通常意味着框里混进了别的东西。
    total_hue_width = sum(high[0] - low[0] + 1 for low, high in entry["hsv_ranges"])
    if total_hue_width > 45:
        print(f"  提示：色相跨度 {total_hue_width}，偏宽；检查取样框是否带到了背景")
    if entry["name"] not in ("black",) and entry["hsv_ranges"][0][0][1] < 40:
        print("  提示：饱和度下界低于 40，白墙白纸也会被当成这个颜色")

    # 标完立刻用实时画面复核一次，省得等到跑节点才发现区间不对。
    if args.preview and not args.image:
        verify_preview(args.camera, entry, key, args.lock_exposure, args.exposure)

    return 0


if __name__ == "__main__":
    sys.exit(main())
