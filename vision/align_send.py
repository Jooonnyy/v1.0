#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""对准专用发送端：取目标颜色 → 像素偏移换算成毫米 → 发 $V 给 STM32。

**不需要任何标定文件。** 只用两个手工常数：
    PX_PER_MM    多少像素算 1 毫米（量一次就行）
    ROT_DEG      相机相对车头装了多歪，逆时针为正（装正就是 0；本车现在是 90）

为什么不用标定：ALIGN 是闭环，每帧重新测误差。尺度差 20% 只让车每步走多/走少，
照样收敛到 0；**角度差几度只让收敛变慢，差 90° 就会绕着物料转圈**。
所以先用这个把闭环跑通，等要精确读出坐标时再上五点单应（calib_plane_h.py）。

怎么量 PX_PER_MM：
    在桌面上放一个物料，看着画面记下像素 y；
    把物料沿车头方向往后挪 100 mm，再记一次像素 y；
    PX_PER_MM = 两次像素差 / 100。

怎么定 ROT_DEG：
    把物料放在车头正前方 300 mm 处，看终端打印的 x/y（或 --no-send 时的 TX 行）。
    x≈+300、y≈0 → 相机正装，ROT_DEG = 0；
    x≈0、y≈+300（物料显示在画面左边）→ ROT_DEG = +90；
    x≈0、y≈-300（显示在右边）→ ROT_DEG = -90。

为什么 90° 会绕圈（2026-11-16 实机 log 复盘）：
    视觉按"画面正上方=车头"算出误差，但相机转了 90°，于是固件拿到的误差向量
    和车真实的方向差 90°。控制器一直朝"看起来的方向"走，车就沿着以物料为圆心、
    半径约等于初始偏差的圆一直转；日志里 e 的模长恒定、角度每 0.5 s 转 ~50°，
    ff 也被这圈运动带到 ±80 mm/s。装错 180° 会发散而不是绕圈，差几度只是收敛慢。

用法：
    python3 align_send.py                            # 自动找串口发帧 + 实时画面
    python3 align_send.py --port COM7 --cls 3        # 指定串口（PC 上接 USB-TTL）
    python3 align_send.py --rot-deg 0                # 相机正装时
    python3 align_send.py --no-send                  # 只出画面，不发串口

类别可以运行时切换：固件每次 `ALIGN/TRACK/GRAB <cls>` 启动都会往 USART3 发一行
`$C,<cls>`，本脚本收到就把目标色换过去（`GRAB BLUE` 也行，名字在固件里映射成 3）。
`--cls` 只是启动时的默认值；`--no-send` 时收不到下行，只能用 `--cls`。

注意：vision_node.py 和本脚本不能同时跑，它们要抢同一个串口。
"""

import argparse
import math
import os
import sys
import time

import numpy as np

from common import (CAMERA_MOUNT_DEG, DEFAULT_CAMERA, Preview,
                    add_preview_argument, add_send_argument, close_all_windows,
                    color_mask, format_line, load_json, open_camera, open_link,
                    parse_camera_source, poll_class_hint_ex, rotate_to_body)
# 检测器复用 vision_node 的那一套：六种颜色全找（黑色走 mode=dark），
# 这里只负责"挑当前类别里最大的那块"再换算成毫米，避免两处算法走样。
from vision_node import (DRAW_COLORS_BGR, build_dark_mask,
                         candidate_from_contour, detect_blobs)

# ---- 默认参数，跟命令行同名的以命令行优先 ----
DEFAULT_PX_PER_MM = 1.5
# 本车相机相对车头装了 +90°（2026-11-16 由 TRACK 绕圈日志反推，见文件头）。
DEFAULT_ROT_DEG = CAMERA_MOUNT_DEG
MIN_AREA_PX = 400
# 检测默认用半分辨率：面积只有 1/4，Pi 上从 ~60 ms/帧降到 ~15 ms；
# 坐标会换算回原图，PX_PER_MM 不受影响。要极致精度时用 --detect-scale 1。
DEFAULT_DETECT_SCALE = 0.5
# 圆环类：固件 DROP 的 Align 步用 cls=11，脚本收到 $C,11 就切到环检测。
RING_CLASS = 11
DEFAULT_RING_MIN_RADIUS = 8
DEFAULT_RING_PARAM2 = 0.85


def detect_rings(frame, min_radius_px=DEFAULT_RING_MIN_RADIUS,
                 max_radius_px=0, param2=DEFAULT_RING_PARAM2, min_circles=3):
    """找画面里所有同心圆环目标，一个目标 = 一簇同心圆。

    预处理借 2025 西理工视觉的 color_circle_position：CLAHE → 高斯 → HoughCircles
    (HOUGH_GRADIENT_ALT)。圆心按"彼此很近"聚成簇，一簇就是一个环（三个环同框也能分开），
    簇内取中位数。**至少 min_circles 个不同半径的同心圆才算目标**，单圈/碎圆直接丢弃，
    避免地面杂圆被当成环。
    """
    import cv2

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    clahe = cv2.createCLAHE(clipLimit=5.0, tileGridSize=(8, 8)).apply(gray)
    blur = cv2.GaussianBlur(clahe, (5, 5), 0)
    height_px, width_px = gray.shape[:2]
    if max_radius_px <= 0:
        max_radius_px = min(height_px, width_px) // 2

    circles = cv2.HoughCircles(blur, cv2.HOUGH_GRADIENT_ALT, 1.5, 30,
                               param1=300, param2=param2,
                               minRadius=max(3, int(min_radius_px)),
                               maxRadius=max(4, int(max_radius_px)))
    if circles is None or len(circles[0]) == 0:
        return []

    points = [tuple(map(float, item)) for item in circles[0]]
    points.sort(key=lambda item: -item[2])     # 大圈先来，先定出目标中心
    groups = []
    for item in points:
        best = None
        best_dist = None
        for group in groups:
            cx = float(np.median([p[0] for p in group]))
            cy = float(np.median([p[1] for p in group]))
            radius = float(max(p[2] for p in group))
            dist = ((item[0] - cx) ** 2 + (item[1] - cy) ** 2) ** 0.5
            if dist <= max(15.0, 0.25 * radius) and (best_dist is None or dist < best_dist):
                best = group
                best_dist = dist
        if best is None:
            groups.append([item])
        else:
            best.append(item)

    # 同一个环的圆可能被拆成两簇（圆心噪声）；先把中心很近的簇合并，
    # 免得一帧报两个“目标”、锁环在两者之间跳。
    merged = []
    for group in groups:
        gx = float(np.median([p[0] for p in group]))
        gy = float(np.median([p[1] for p in group]))
        gr = float(max(p[2] for p in group))
        for existing in merged:
            ex = float(np.median([p[0] for p in existing]))
            ey = float(np.median([p[1] for p in existing]))
            er = float(max(p[2] for p in existing))
            if ((gx - ex) ** 2 + (gy - ey) ** 2) ** 0.5 <= max(25.0, 0.35 * max(gr, er)):
                existing.extend(group)
                break
        else:
            merged.append(list(group))
    groups = merged

    targets = []
    for group in groups:
        # 同一个圆可能被 Hough 重复检出；先按半径去重，再要求至少有
        # min_circles 个不同半径的同心圆，单圈/碎圆不算目标。
        radii = []
        for item in sorted(group, key=lambda p: p[2]):
            if not radii or abs(item[2] - radii[-1]) > 3.0:
                radii.append(item[2])
        if len(radii) < max(1, int(min_circles)):
            continue
        u = float(np.median([p[0] for p in group]))
        v = float(np.median([p[1] for p in group]))
        radius = float(max(p[2] for p in group))
        targets.append({"center_px": (u, v), "radius_px": radius,
                        "area_px": float(np.pi * radius * radius), "circles": group,
                        "radii": radii})
    targets.sort(key=lambda item: -item["radius_px"])
    return targets


def detect_shape_targets(frame, min_area_px=600, max_area_px=20000,
                         min_circularity=0.60, min_fill=0.55):
    """先按外形找圆柱/圆形目标：Canny + 轮廓 + 圆度/填充率/长宽比。"""
    import cv2

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    kernel = np.ones((5, 5), np.uint8)

    edges = cv2.Canny(blur, 50, 150)
    edges = cv2.dilate(edges, kernel, iterations=1)
    edges = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel)

    # 黑料在深色桌面上 Canny 边缘经常断开；再用自适应阈值和 Otsu 各找一遍。
    adaptive = cv2.adaptiveThreshold(blur, 255, cv2.ADAPTIVE_THRESH_MEAN_C,
                                     cv2.THRESH_BINARY_INV, 31, 5)
    adaptive = cv2.morphologyEx(adaptive, cv2.MORPH_OPEN, kernel)
    _, otsu = cv2.threshold(blur, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
    otsu = cv2.morphologyEx(otsu, cv2.MORPH_OPEN, kernel)

    targets = []
    for mask in (edges, adaptive, otsu):
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            area = cv2.contourArea(contour)
            if area < min_area_px or area > max_area_px:
                continue
            perimeter = cv2.arcLength(contour, True)
            if perimeter <= 0.0:
                continue
            circularity = 4.0 * math.pi * area / (perimeter * perimeter)
            (cx, cy), radius = cv2.minEnclosingCircle(contour)
            if radius <= 1.0:
                continue
            fill = area / (math.pi * radius * radius)
            _, _, w, h = cv2.boundingRect(contour)
            aspect = min(w, h) / max(w, h) if max(w, h) > 0 else 0.0
            if circularity < min_circularity or fill < min_fill or aspect < 0.60:
                continue
            duplicate = False
            for other in targets:
                ox, oy = other["center_px"]
                if ((cx - ox) ** 2 + (cy - oy) ** 2) ** 0.5 <= max(10.0, 0.35 * max(radius, other["radius_px"])):
                    duplicate = True
                    break
            if duplicate:
                continue
            targets.append({"center_px": (float(cx), float(cy)),
                            "radius_px": float(radius), "area_px": float(area),
                            "circularity": float(circularity), "fill": float(fill)})
    targets.sort(key=lambda item: -item["area_px"])
    return targets


def classify_shape_color(frame_hsv, target, colors, min_fraction=0.20):
    """在圆形轮廓内部采样，返回最像的颜色类别和占比。"""
    import cv2

    height_px, width_px = frame_hsv.shape[:2]
    mask = np.zeros((height_px, width_px), np.uint8)
    cx, cy = target["center_px"]
    radius = max(2, int(target["radius_px"] * 0.75))
    cv2.circle(mask, (int(cx), int(cy)), radius, 255, -1)
    total = int(cv2.countNonZero(mask))
    if total <= 0:
        return None, 0.0

    best_class = None
    best_score = 0.0
    for key, entry in colors["classes"].items():
        if entry.get("mode") == "dark":
            value = frame_hsv[..., 2]
            saturation = frame_hsv[..., 1]
            current = ((value <= int(entry.get("max_v", 170))) &
                       (saturation <= int(entry.get("max_s", 140)))).astype(np.uint8) * 255
            current = cv2.bitwise_and(current, mask)
        else:
            current = color_mask(frame_hsv, entry)
            if current is None:
                continue
            current = cv2.bitwise_and(current, mask)
        score = float(cv2.countNonZero(current)) / float(total)
        if score > best_score:
            best_score = score
            best_class = int(key)
    if best_score < min_fraction:
        return None, best_score
    return best_class, best_score


def detect_ring(frame, min_radius_px=DEFAULT_RING_MIN_RADIUS,
                max_radius_px=0, param2=DEFAULT_RING_PARAM2, min_circles=3):
    """兼容旧调用：只返回最大的那个目标。"""
    targets = detect_rings(frame, min_radius_px, max_radius_px, param2, min_circles)
    return targets[0] if targets else None


DIGIT_TEMPLATE_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                  "digit_templates")


def load_digit_templates():
    """读 `digit_templates/1..3.png`（由 make_digit_templates.py 从印刷图案抠出来）。"""
    import cv2

    templates = {}
    for digit in (1, 2, 3):
        path = os.path.join(DIGIT_TEMPLATE_DIR, f"{digit}.png")
        image = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
        if image is not None:
            templates[digit] = image
    return templates


def read_ring_digit(gray, center, inner_radius_px, templates, min_score=0.35):
    """读同心圆里的数字（1/2/3）。

    裁剪环心 → 圆形掩膜 → OTSU → 最大的居中轮廓 → 归一化 64×64 → 与 3 个模板做
    四方向旋转匹配。模板就是印刷体的 1/2/3，所以不用 OCR 依赖。
    返回 (digit, score)；认不出返回 (None, 0.0)。
    """
    import cv2

    if not templates:
        return None, 0.0

    cx, cy = int(center[0]), int(center[1])
    half = int(inner_radius_px * 0.98)
    x0, y0 = max(0, cx - half), max(0, cy - half)
    x1 = min(gray.shape[1], cx + half)
    y1 = min(gray.shape[0], cy + half)
    roi = gray[y0:y1, x0:x1]
    if roi.size == 0:
        return None, 0.0

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
        return None, 0.0

    x, y, w, h = best_box
    glyph = mask[y:y + h, x:x + w]
    scale = min(56.0 / max(1, w), 56.0 / max(1, h))
    nw = max(1, int(round(w * scale)))
    nh = max(1, int(round(h * scale)))
    canvas = np.zeros((64, 64), np.uint8)
    resized = cv2.resize(glyph, (nw, nh), interpolation=cv2.INTER_AREA)
    ox, oy = (64 - nw) // 2, (64 - nh) // 2
    canvas[oy:oy + nh, ox:ox + nw] = resized

    best_digit = None
    best_score = -2.0
    for digit, template in templates.items():
        for turns in range(4):
            score = cv2.matchTemplate(canvas, np.rot90(template, turns),
                                      cv2.TM_CCOEFF_NORMED)[0, 0]
            if score > best_score:
                best_score = float(score)
                best_digit = digit
    if best_score < min_score:
        return None, best_score
    return best_digit, best_score


def pick_by_class(candidates, cls):
    """从"这一帧所有颜色"的候选里，挑指定类别中面积最大的那块。"""
    pool = [item for item in candidates if item["cls"] == cls]
    if not pool:
        return None
    return max(pool, key=lambda item: item["area_px"])


def detect_class(frame_hsv, colors, cls, min_area_px):
    """只算当前类别的候选。单类掩膜约 0.5 ms，比六色全检快两个数量级。

    这是"帧率优先"模式：类别切换靠 $C，切成哪类就只算哪类；
    代价是这一帧看不到别的颜色（`--show-all` 会自动改用全检）。
    """
    import cv2

    if not frame_hsv.size:
        return []
    entry = colors["classes"].get(str(cls))
    if entry is None:
        return []
    if entry.get("mode") == "dark":
        mask = build_dark_mask(frame_hsv, entry, colors, min_area_px)
    else:
        mask = color_mask(frame_hsv, entry)
    if mask is None:
        return []

    class_min_area = int(entry.get("min_area_px", min_area_px))
    min_circle_fill = float(entry.get("min_circle_fill", 0.0))
    min_solidity = float(entry.get("min_solidity", 0.0))

    candidates = []
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for contour in contours:
        candidate = candidate_from_contour(contour, int(cls), class_min_area)
        if not candidate:
            continue
        if candidate["circle_fill"] < min_circle_fill:
            continue
        if candidate["solidity"] < min_solidity:
            continue
        candidates.append(candidate)
    return candidates


def shrink_frame(frame, scale):
    """检测降分辨率：返回 (缩图, 用到的比例)。scale >= 1 时原样返回。"""
    if scale >= 0.999:
        return frame, 1.0
    import cv2

    height_px, width_px = frame.shape[:2]
    small = cv2.resize(frame,
                       (max(8, int(width_px * scale)), max(8, int(height_px * scale))),
                       interpolation=cv2.INTER_AREA)
    return small, scale


def scale_candidates(candidates, scale):
    """把缩图上算出来的中心/面积/轮廓换回原图坐标。"""
    if scale >= 0.999:
        return candidates
    inv = 1.0 / scale
    for item in candidates:
        item["center_px"] = (item["center_px"][0] * inv, item["center_px"][1] * inv)
        item["area_px"] = item["area_px"] * inv * inv
        item["radius_px"] = item["radius_px"] * inv
        item["contour"] = item["contour"].astype(np.float32) * inv
    return candidates


def draw_candidates(frame, candidates, selected, show_all):
    """预览用：show_all 时把六种颜色都画出来，选中的那块再加粗。"""
    import cv2

    if show_all:
        for item in candidates:
            if "contour" in item:
                corners = cv2.boxPoints(cv2.minAreaRect(item["contour"])).astype(int)
                color = DRAW_COLORS_BGR.get(item["cls"], (200, 200, 200))
                cv2.drawContours(frame, [corners], -1, color, 1)
            color = DRAW_COLORS_BGR.get(item["cls"], (200, 200, 200))
            center = (int(item["center_px"][0]), int(item["center_px"][1]))
            cv2.putText(frame, f"{item['cls']}", (center[0] + 4, center[1] - 4),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
    if selected is not None:
        if "contour" in selected:
            corners = cv2.boxPoints(cv2.minAreaRect(selected["contour"])).astype(int)
            cv2.drawContours(frame, [corners], -1, (0, 255, 0), 2)
        else:
            center = (int(selected["center_px"][0]), int(selected["center_px"][1]))
            radius = max(4, int(selected.get("radius_px", 20)))
            cv2.circle(frame, center, radius, (0, 255, 0), 2)
            cv2.drawMarker(frame, center, (0, 255, 0), cv2.MARKER_CROSS, 26, 2)
    return frame


def pixel_to_mm(u, v, gripper_px, px_per_mm, rot_deg):
    """像素 → 车体毫米。约定：先按"画面正上方 = 车头"算，再按相机安装角转回车体系。

    rot_deg 就是相机相对车头的安装角（逆时针为正）。0 时等价于
    x = -(v-cy)/k、y = -(u-cx)/k；90° 时 x 取画面水平方向。
    """
    cx, cy = gripper_px
    scale = max(1e-6, px_per_mm)
    raw_x_mm = -(v - cy) / scale
    raw_y_mm = -(u - cx) / scale
    return rotate_to_body(raw_x_mm, raw_y_mm, rot_deg)


def main(argv=None):
    parser = argparse.ArgumentParser(description="对准专用 $V 发送端（不需要标定）")
    parser.add_argument("--camera", default=DEFAULT_CAMERA, help="相机序号或设备路径")
    parser.add_argument("--width", type=int, default=640, help="请求的画面宽度，单位 px")
    parser.add_argument("--height", type=int, default=480, help="请求的画面高度，单位 px")
    parser.add_argument("--port", default=None,
                        help="串口，如 /dev/serial0 或 COM7；不给就自动找（/dev/serial0、USB-TTL）")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--cls", type=int, default=3,
                        help="初始目标类别 1-6/10/11；固件发 $C,<cls> 会覆盖它")
    parser.add_argument("--px-per-mm", type=float, default=DEFAULT_PX_PER_MM,
                        help="多少像素算 1 毫米，量一次")
    parser.add_argument("--rot-deg", type=float, default=DEFAULT_ROT_DEG,
                        help="相机相对车头的安装角，逆时针为正：正装 0、本车默认 90")
    parser.add_argument("--min-area-px", type=int, default=MIN_AREA_PX)
    parser.add_argument("--hz", type=float, default=10.0)
    parser.add_argument("--colors", default="colors.json")
    parser.add_argument("--gripper-px", nargs=2, type=float, default=None,
                        metavar=("U", "V"), help="夹爪在画面里的像素位置，默认画面中心")
    parser.add_argument("--show-all", action="store_true",
                        help="预览时把六种颜色的候选都画出来（每类一个颜色），查漏检/串色用")
    parser.add_argument("--detect", choices=("all", "single"), default="all",
                        help="all = 每帧六色全检（默认，能同时看到所有颜色）；"
                             "single = 只算当前类别，最快（~0.5 ms），切类靠 $C")
    parser.add_argument("--detect-scale", type=float, default=DEFAULT_DETECT_SCALE,
                        help="检测分辨率比例 0.25~1，默认 0.5（半分辨率，坐标自动换回原图）")
    parser.add_argument("--link-test", action="store_true",
                        help="只测下行：开串口把收到的原始字节打印出来（STM32 发 HINT 3 时应看到 $C,3）")
    parser.add_argument("--ring-px-per-mm", type=float, default=0.0,
                        help="环模式 px/mm（在释放高度量），0 = 用 --px-per-mm")
    parser.add_argument("--ring-min-radius", type=int, default=DEFAULT_RING_MIN_RADIUS,
                        help="HoughCircles 最小半径 px")
    parser.add_argument("--ring-max-radius", type=int, default=0,
                        help="HoughCircles 最大半径 px，0 = 图像短边/2")
    parser.add_argument("--ring-param2", type=float, default=DEFAULT_RING_PARAM2,
                        help="HOUGH_GRADIENT_ALT 的圆完美度 0.5~0.99")
    parser.add_argument("--ring-index", type=int, default=1,
                        help="期望的环号 1..3（固件 $C,11,<n> 会覆盖），用来和读到的数字比对")
    parser.add_argument("--ring-digit-check", choices=("off", "warn", "strict"), default="warn",
                        help="读到的数字和期望环号不一致时：off 不管 / warn 只提示（默认）/ strict 不发目标")
    parser.add_argument("--ring-digit-min-score", type=float, default=0.35,
                        help="数字模板匹配最低分，低于它当没读出")
    parser.add_argument("--ring-offset-x-mm", type=float, default=0.0,
                        help="放料偏差补偿（车体 x，mm）：10 次落点平均偏 +d 就填 +d")
    parser.add_argument("--ring-offset-y-mm", type=float, default=0.0,
                        help="放料偏差补偿（车体 y，mm）")
    parser.add_argument("--link-test-s", type=float, default=30.0,
                        help="--link-test 持续秒数，默认 30，0 = 一直跑到 Ctrl+C")
    add_preview_argument(parser)
    add_send_argument(parser)
    parser.add_argument("--require-serial", action="store_true",
                        help="串口打不开就直接退出（默认降级为只预览/打印，容易误以为在发）")
    args = parser.parse_args(argv)

    try:
        import cv2
    except ImportError:
        print("缺少 opencv：pip3 install opencv-python numpy")
        return 2

    # 只测下行链路：不需要相机、不需要 colors.json。
    if args.link_test:
        if not args.send:
            print("--link-test 需要串口；不要加 --no-send")
            return 2
        link = open_link(args.port, args.baud)
        if link is None:
            return 1
        print(f"下行测试 {args.link_test_s:.0f} s：在 STM32 命令口发 `HINT 3`（或 `GRAB BLUE`），"
              "这里应看到 b'$C,3\\n' 这样的原始字节。Ctrl+C 结束。")
        deadline = time.time() + args.link_test_s if args.link_test_s > 0 else None
        try:
            while deadline is None or time.time() < deadline:
                try:
                    pending = link.in_waiting
                except Exception:
                    pending = 0
                if pending:
                    print(f"RX {link.read(pending)!r}")
                else:
                    time.sleep(0.02)
        except KeyboardInterrupt:
            print("\n停止")
        finally:
            link.close()
        return 0

    colors = load_json(args.colors)
    if colors is None or not colors.get("classes"):
        print(f"找不到 {args.colors} 里的颜色，先用 calib_color.py 标颜色")
        return 1
    digit_templates = load_digit_templates()
    if not digit_templates:
        print("提示：digit_templates/ 里没有 1/2/3.png，环模式跳过数字识别")
    # 每帧检测全部颜色，这里只决定"发哪一类"；运行中会被固件下发的 `$C,<cls>` 改掉。
    current_cls = args.cls
    ring_index = max(1, min(9, args.ring_index))   # 期望的环号（和读到的数字比对）
    ring_digit_message = None                      # 上次提示过的 (期望, 读到)，防刷屏
    hint_buffer = b""
    if str(current_cls) not in colors["classes"] and current_cls != RING_CLASS:
        print(f"提示：colors.json 里没有 class {current_cls}，"
              "等固件发 $C 切到有效类别前不会发出目标")
    if args.ring_px_per_mm <= 0.0:
        args.ring_px_per_mm = args.px_per_mm
        print("环模式沿用 --px-per-mm；建议在释放高度用 100 mm 方格重量一次。")
    if args.cls == RING_CLASS:
        print(f"起始就是环模式：cls=11，px/mm={args.ring_px_per_mm:.2f}，"
              f"期望第 {args.ring_index} 环，数字校验={args.ring_digit_check}")
    print(f"检测全部颜色：{', '.join(sorted(colors['classes'], key=int))}"
          f"（当前发送 cls={current_cls}）")
    if not 0.2 <= args.detect_scale <= 1.0:
        print("--detect-scale 必须在 0.2~1.0")
        return 2
    detect_mode = "all" if args.show_all else args.detect
    print(f"检测模式 {detect_mode}，分辨率比例 {args.detect_scale:g}"
          f"（{args.width}x{args.height} → "
          f"{int(args.width * args.detect_scale)}x{int(args.height * args.detect_scale)}）")

    capture = open_camera(parse_camera_source(args.camera),
                          width_px=args.width, height_px=args.height)
    if capture is None:
        print(f"打不开相机 {args.camera}（PC 上先试 0 / 1）")
        return 1

    link = open_link(args.port, args.baud) if args.send else None
    if not args.send:
        print("串口发送已关闭（--no-send）：只出画面/打印帧")
    elif link is None:
        print("")
        print("[!] 串口没打开：本次只预览，不会发 $V，也收不到固件的 $C（换色会失效）。")
        print("    树莓派：sudo usermod -aG dialout $USER，然后注销重登；")
        print("    并确认没有别的进程占着这个口（sudo lsof /dev/serial0）。")
        print("    先单独确认能不能开：python3 align_send.py --link-test")
        print("")
        if args.require_serial:
            print("--require-serial：直接退出。")
            return 1
    preview = Preview("align_send") if args.preview else None

    port_text = link.port if link else ("关闭" if not args.send else "未打开（只打印）")
    print(f"相机 {args.camera}  cls={args.cls}  px/mm={args.px_per_mm:.2f}  "
          f"rot={args.rot_deg:.1f} deg  串口={port_text}")
    if abs(args.rot_deg) >= 45.0:
        print(f"相机按装歪 {args.rot_deg:.0f}° 处理：物料放在车头正前方时应读到 "
              "x>0、y≈0。如果车还绕着物料转圈，把 --rot-deg 的符号反过来再试。")
    print("STM32 那边：E 1 → V → GRAB 3 / GRAB BLUE（类别会通过 $C 自动切过来）")
    if preview:
        print("实时画面：q / ESC 退出")
    print("终端里每行 TX 就是实际发出去的 $V 帧，可以直接和固件 V 的回显对照。")

    period_s = 1.0 / max(1.0, args.hz)
    seq = 0
    sent = 0
    none_since_s = None
    gripper_px = tuple(args.gripper_px) if args.gripper_px else None
    last_frame_s = time.time()

    try:
        while True:
            started_s = time.time()
            ok, frame = capture.read()
            if not ok or frame is None:
                time.sleep(0.02)
                continue

            height_px, width_px = frame.shape[:2]
            if gripper_px is None:
                gripper_px = (width_px / 2.0, height_px / 2.0)

            # 固件在每次 ALIGN/TRACK/GRAB/DROP 启动时发 `$C,<cls>[,<n>]`，这里非阻塞地收。
            hint, hint_aux, hint_buffer = poll_class_hint_ex(link, hint_buffer)
            if hint is not None:
                if hint == RING_CLASS:
                    if current_cls != RING_CLASS:
                        print("\n[STM32] 目标切到 11（圆环），用环检测对准")
                    current_cls = RING_CLASS
                    if hint_aux > 0:
                        ring_index = hint_aux
                        print(f"\n[STM32] 选第 {ring_index} 个圆环")
                elif hint != current_cls and str(hint) in colors["classes"]:
                    current_cls = hint
                    print(f"\n[STM32] 目标类别切到 {current_cls}"
                          f"（{colors['classes'][str(hint)].get('name', '?')}）")
                elif hint != current_cls:
                    print(f"\n[STM32] 收到类别 {hint}，colors.json 里没有这一类，忽略")
            candidates = []
            ring_targets = []
            found = None
            if current_cls == RING_CLASS:
                # 环模式：只报离夹爪最近的一个环（车停在目标环前），数字用来确认是哪一环。
                ring_targets = detect_rings(frame, args.ring_min_radius,
                                            args.ring_max_radius, args.ring_param2)
                if ring_targets:
                    found = min(ring_targets,
                                key=lambda item: (item["center_px"][0] - gripper_px[0]) ** 2 +
                                                 (item["center_px"][1] - gripper_px[1]) ** 2)
                    inner_radius_px = min(cr for _, _, cr in found["circles"])
                    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                    digit, digit_score = read_ring_digit(
                        gray, found["center_px"], inner_radius_px,
                        digit_templates, args.ring_digit_min_score)
                    found["digit"] = digit
                    found["digit_score"] = digit_score
                    if digit is not None and digit != ring_index:
                        key = (ring_index, digit)
                        if key != ring_digit_message:
                            ring_digit_message = key
                            message = (f"[环] 请求第 {ring_index} 环，读到数字 {digit}"
                                       f"（score={digit_score:.2f}）")
                            if args.ring_digit_check == "strict":
                                print("\n" + message + "，strict：不发目标")
                            elif args.ring_digit_check == "warn":
                                print("\n" + message + "，继续发送（warn）")
                        if args.ring_digit_check == "strict":
                            found = None
                    elif digit == ring_index:
                        ring_digit_message = None
            else:
                # 检测可以在缩图上做：面积按 scale² 缩小，坐标算完再换回原图，
                # 这样 PX_PER_MM 仍然是按原分辨率量的那一个值。
                detect_frame, used_scale = shrink_frame(frame, args.detect_scale)
                frame_hsv = cv2.cvtColor(detect_frame, cv2.COLOR_BGR2HSV)
                detect_min_area = max(30, int(args.min_area_px * used_scale * used_scale))
                if detect_mode == "single":
                    candidates = detect_class(frame_hsv, colors, current_cls, detect_min_area)
                else:
                    candidates = detect_blobs(frame_hsv, colors, detect_min_area)
                scale_candidates(candidates, used_scale)
                found = pick_by_class(candidates, current_cls)
            line = None
            x_mm = y_mm = 0.0

            if found is not None:
                u, v = found["center_px"]
                area = found["area_px"]
                px_per_mm = args.ring_px_per_mm if current_cls == RING_CLASS else args.px_per_mm
                x_mm, y_mm = pixel_to_mm(u, v, gripper_px, px_per_mm, args.rot_deg)
                if current_cls == RING_CLASS:
                    # 放料偏差补偿：正偏 +d 就把目标再往前送 d，抵消固定的落点偏置
                    x_mm += args.ring_offset_x_mm
                    y_mm += args.ring_offset_y_mm
                age_ms = int((time.time() - started_s) * 1000.0)
                line = format_line(seq, current_cls, x_mm, y_mm, 0, 900, age_ms)
                none_since_s = None
                if preview:
                    cv2.circle(frame, (int(u), int(v)), 6, (0, 0, 255), -1)
                    cv2.putText(frame, f"x={x_mm:.0f} y={y_mm:.0f} mm  area={area:.0f}",
                                (int(u) + 10, int(v) - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)
            else:
                # 找不到目标时按 1 Hz 发一条 cls=0，让固件分清"视野里没有"和"链路断了"。
                now_s = time.time()
                if none_since_s is None:
                    none_since_s = now_s
                if (now_s - none_since_s) >= 1.0:
                    none_since_s = now_s
                    line = format_line(seq, 0, 0, 0, 0, 0,
                                       int((now_s - started_s) * 1000.0))

            if line is not None:
                seq += 1
                if link:
                    link.write(line)
                    sent += 1
                # 打印的就是**实际发出去的字节**，方便和串口助手/固件回显对照。
                state = "none" if found is None else f"x={x_mm:7.1f} y={y_mm:7.1f} mm"
                print(f"\r{state}  TX {line.decode('ascii').strip()}          ",
                      end="", flush=True)

            if preview:
                now_s = time.time()
                fps = 1.0 / max(1e-3, now_s - last_frame_s)
                last_frame_s = now_s
                if current_cls == RING_CLASS:
                    if found is not None:
                        for cx, cy, cr in found["circles"]:
                            cv2.circle(frame, (int(cx), int(cy)), int(cr), (0, 0, 255), 2)
                        tx, ty = found["center_px"]
                        cv2.drawMarker(frame, (int(tx), int(ty)), (0, 0, 255),
                                       cv2.MARKER_CROSS, 26, 2)
                        digit_text = "?" if found.get("digit") is None else str(found["digit"])
                        cv2.putText(frame, f"ring {digit_text} "
                                           f"({found.get('digit_score', 0.0):.2f})",
                                    (8, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)
                else:
                    draw_candidates(frame, candidates, found, args.show_all)
                cv2.drawMarker(frame, (int(gripper_px[0]), int(gripper_px[1])),
                               (0, 255, 0), cv2.MARKER_CROSS, 26, 2)
                shown_px_per_mm = args.ring_px_per_mm if current_cls == RING_CLASS else args.px_per_mm
                cv2.putText(frame, f"cls={current_cls} px/mm={shown_px_per_mm:.2f} "
                                   f"rot={args.rot_deg:.1f} deg  {fps:.0f} fps",
                            (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2)
                if link is None:
                    if args.send:
                        text = "NO SERIAL - preview only (check dialout / port)"
                        color = (0, 0, 255)
                    else:
                        text = "send off (--no-send)"
                        color = (0, 165, 255)
                    cv2.putText(frame, text, (8, 48),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
                preview.show(frame)
                if preview.quit:
                    break

            elapsed_s = time.time() - started_s
            if elapsed_s < period_s:
                time.sleep(period_s - elapsed_s)
    except KeyboardInterrupt:
        print("\n停止")
    finally:
        capture.release()
        if link:
            link.close()
        if preview:
            preview.close()
        close_all_windows()
    print(f"\n发出 {sent} 帧")
    return 0


if __name__ == "__main__":
    sys.exit(main())
