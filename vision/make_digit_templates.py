# -*- coding: utf-8 -*-
"""从印刷的圆环图案照片里抠出 1/2/3 三个数字，生成 align_send.py 用的模板。

用法：
    python3 make_digit_templates.py --image 圆环照片.jpg
    python3 make_digit_templates.py --image 圆环照片.jpg --out digit_templates

模板会存成 `digit_templates/1.png`、`2.png`、`3.png`（64×64，黑底白字）。
拍照要求：三个环都完整入画、尽量正对；环从左到右是 1/2/3。
换印刷图案后重新跑一次即可，不用改代码。
"""

import argparse
import os
import sys

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import align_send  # noqa: E402

DIGIT_SIZE_PX = 64
GLYPH_SIZE_PX = 56


def crop_digit(gray, center, inner_radius_px, digit_size_px=DIGIT_SIZE_PX):
    """按环心裁一块，取最大的居中暗色块（数字），归一化到 digit_size_px 方图。"""
    cx, cy = int(center[0]), int(center[1])
    half = int(inner_radius_px * 0.98)
    x0, y0 = max(0, cx - half), max(0, cy - half)
    x1 = min(gray.shape[1], cx + half)
    y1 = min(gray.shape[0], cy + half)
    roi = gray[y0:y1, x0:x1]
    if roi.size == 0:
        return None

    height_px, width_px = roi.shape
    circle = np.zeros((height_px, width_px), np.uint8)
    cv2.circle(circle, (width_px // 2, height_px // 2),
               int(min(height_px, width_px) * 0.47), 255, -1)
    roi = cv2.bitwise_and(roi, circle)
    _, mask = cv2.threshold(roi, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
    mask = cv2.bitwise_and(mask, circle)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    best_box = None
    best_area = 0.0
    for contour in contours:
        x, y, w, h = cv2.boundingRect(contour)
        area = cv2.contourArea(contour)
        if area < 60.0 or h < min(height_px, width_px) * 0.25:
            continue
        if abs((x + w / 2.0) - width_px / 2.0) > width_px * 0.16:
            continue
        if abs((y + h / 2.0) - height_px / 2.0) > height_px * 0.16:
            continue
        if area > best_area:
            best_area = area
            best_box = (x, y, w, h)
    if best_box is None:
        return None

    x, y, w, h = best_box
    glyph = mask[y:y + h, x:x + w]
    scale = min(GLYPH_SIZE_PX / max(1, w), GLYPH_SIZE_PX / max(1, h))
    nw = max(1, int(round(w * scale)))
    nh = max(1, int(round(h * scale)))
    canvas = np.zeros((digit_size_px, digit_size_px), np.uint8)
    resized = cv2.resize(glyph, (nw, nh), interpolation=cv2.INTER_AREA)
    ox = (digit_size_px - nw) // 2
    oy = (digit_size_px - nh) // 2
    canvas[oy:oy + nh, ox:ox + nw] = resized
    return canvas


def main(argv=None):
    parser = argparse.ArgumentParser(description="从圆环图案照片生成 1/2/3 数字模板")
    parser.add_argument("--image", required=True, help="三个环都完整的照片")
    parser.add_argument("--out", default=os.path.join(HERE, "digit_templates"),
                        help="输出目录，默认脚本旁边的 digit_templates/")
    args = parser.parse_args(argv)

    image = cv2.imread(args.image)
    if image is None:
        print(f"打不开图片：{args.image}")
        return 1

    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    targets = align_send.detect_rings(image, 6, 0, 0.85)
    if len(targets) < 3:
        print(f"只找到 {len(targets)} 个环，需要 3 个（三个环都要完整入画）")
        return 1

    # 从左到右 = 1/2/3
    targets.sort(key=lambda item: item["center_px"][0])
    os.makedirs(args.out, exist_ok=True)
    for digit, target in enumerate(targets[:3], 1):
        inner_radius = min(circle[2] for circle in target["circles"])
        canvas = crop_digit(gray, target["center_px"], inner_radius)
        if canvas is None:
            print(f"第 {digit} 个环里没找到数字，检查照片清晰度/阈值")
            return 1
        path = os.path.join(args.out, f"{digit}.png")
        cv2.imwrite(path, canvas)
        print(f"写 {path}  (center={target['center_px']}, inner_r={inner_radius:.1f})")

    print("完成。align_send.py 会自动读这个目录。")
    return 0


if __name__ == "__main__":
    sys.exit(main())
