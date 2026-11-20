#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""视觉节点：相机 → 目标位姿 → 串口帧（$V,...*CRC）→ STM32。

三个任务（颜色块 / 载物盘 / 色环）都从同一个出口出去：每帧目标一个，字段全是整数，
时间戳由 STM32 收到时打，帧龄由本节点算。默认**开实时画面 + 自动打开串口发帧**；
没接线的机器自动降级成只打印，`--no-send` 可以显式关掉发送。

用法：
    # 最常用：默认相机 0、自动找串口（/dev/serial0 或 USB-TTL）
    python3 vision_node.py

    # 树莓派双相机：用 udev 固定过的设备名，别用 /dev/videoN（插拔顺序一变编号就换）
    python3 vision_node.py --camera /dev/cam_target

    # 存样本图（原始帧，未做任何校正），供算法回归复用
    python3 vision_node.py --save-dir samples/raw --save-every 10

    # 上车：要发毫米坐标必须有 calib_plane_h.json（没有就只能预览/打印像素）
    python3 vision_node.py --camera /dev/cam_target --homography calib_plane_h.json --want 3

    # 只看画面、不发串口
    python3 vision_node.py --no-send

    # 载物盘 / 色环
    python3 vision_node.py --task tray
    python3 vision_node.py --task ring --want 3

    # 拿一张图片验证整条换算链路（不连相机）
    python3 vision_node.py --source image --image shot.jpg

输出帧格式见 BSP/Inc/board_vision.h 顶部的协议说明。
"""

import argparse
import math
import os
import sys
import time

import numpy as np

from common import (CLASS_RING, CLASS_TRAY, DEFAULT_CAMERA, Preview,
                    add_preview_argument, add_send_argument, close_all_windows,
                    color_mask, format_line, load_homography, load_json,
                    normalize_yaw_mdeg, open_camera, open_link,
                    parse_camera_source, pixel_to_plane_h, poll_class_hint,
                    rotate_to_body)

DEFAULT_MIN_AREA_PX = 250

# 连续读不到帧多少次就认定相机绑错了。UVC 的元数据节点（video-index1）能被正常
# open，但 read() 永远返回失败 —— 没有这个计数就变成静默死循环，看不出错在哪。
MAX_READ_FAILS = 60

# --show-all 预览时每个类画一个颜色。黑色用白色描边，否则在深色背景上看不见。
DRAW_COLORS_BGR = {
    1: (0, 0, 255),        # 红
    2: (0, 255, 255),      # 黄
    3: (255, 0, 0),        # 蓝
    4: (0, 255, 0),        # 绿
    5: (255, 255, 255),    # 黑
    6: (255, 255, 0),      # 浅蓝
    CLASS_TRAY: (255, 0, 255),
    CLASS_RING: (0, 165, 255),
}

# ---- 形状判据 ----
# 一律用"面积 / 外接几何体面积"，比 4πA/P² 分得开：
#     正圆     外接圆填充率 ≈ 1.00，外接矩形填充率 ≈ 0.79
#     正方形   外接圆填充率 ≈ 0.64，外接矩形填充率 ≈ 1.00
#     正三角形 外接圆填充率 ≈ 0.41
# 4πA/P² 的问题是正方形也有 0.785，跟圆的 1.0 挤在一起，阈值怎么定都容易错判。
SHAPE_CIRCLE_FILL_MIN = 0.85   # 外接圆填充率高于它就判圆
SHAPE_RECT_FILL_MIN = 0.85     # 外接矩形填充率高于它就判矩形
# 凸包填充率低于它直接丢弃：轮廓被啃得七零八落（碎片掩膜），
# 质心没有意义，这类候选正是"背景杂物被误检"的主要来源。
SHAPE_SOLIDITY_MIN = 0.85

# ---- 暗色目标（黑料）判据 ----
# 黑色在 HSV 里和阴影完全同质，绝对阈值怎么调都会顾此失彼。改用三条相对判断：
#   ① 比"局部背景"暗到一定比例 —— 局部背景用大核高斯模糊估计，补光不均也能自适应
#   ② 绝对亮度别太高 —— 防止把中灰的背景木板也算成黑
#   ③ 不贴着彩色目标 —— 阴影永远紧贴它所属的物体，孤立黑料不会
BLACK_DARK_RATIO = 0.22        # 比局部背景暗 22% 以上才算暗目标
BLACK_MAX_V = 170              # 绝对亮度上限
BLACK_MAX_S = 140              # 饱和度上限（黑料偏灰，不该像彩色那样鲜艳）
DEFAULT_SHADOW_GUARD_PX = 28   # 彩色目标周围这么多像素内的暗区一律不当黑料
# 局部背景的估计尺度：高斯核 sigma 取画面宽度的这个比例。
# 太小会把黑料自己也当成背景，太大则跟不上补光的不均匀。
BACKGROUND_SIGMA_SCALE = 0.16


# ---------------------------------------------------------------------------
# 检测
# ---------------------------------------------------------------------------

def describe_shape(contour):
    """量一个轮廓的形状指标，返回字典。

    circle_fill  面积 / 外接圆面积：正圆≈1.00、正方形≈0.64、正三角≈0.41
    rect_fill    面积 / 最小外接矩形面积：正方形≈1.00、正圆≈0.79
    aspect       长宽比，圆和正方形都≈1，用来排除细长条
    solidity     面积 / 凸包面积，掩膜碎掉时显著下降
    """
    import cv2

    area = cv2.contourArea(contour)
    (_, _), radius_px = cv2.minEnclosingCircle(contour)
    if area <= 1e-6 or radius_px <= 1e-6:
        return None

    circle_fill = area / (math.pi * radius_px * radius_px)

    (_, _), (width_px, height_px), _ = cv2.minAreaRect(contour)
    rect_area = max(1.0, width_px * height_px)
    rect_fill = area / rect_area
    aspect = max(width_px, height_px) / max(1.0, min(width_px, height_px))

    hull_area = max(1.0, cv2.contourArea(cv2.convexHull(contour)))
    solidity = area / hull_area

    if circle_fill >= SHAPE_CIRCLE_FILL_MIN:
        shape = "circle"
    elif rect_fill >= SHAPE_RECT_FILL_MIN:
        shape = "rect"
    else:
        shape = "unknown"

    return {
        "shape": shape,
        "circle_fill": circle_fill,
        "rect_fill": rect_fill,
        "aspect": aspect,
        "solidity": solidity,
        "radius_px": radius_px,
    }


def candidate_from_contour(contour, class_code, min_area_px, kind=None):
    """把一个轮廓变成候选目标：像素中心 + 主轴方向 + 形状指标 + 置信度。"""
    import cv2

    area = cv2.contourArea(contour)
    if area < min_area_px:
        return None
    rect = cv2.minAreaRect(contour)
    (center_x_px, center_y_px), (width_px, height_px), angle_deg = rect
    if min(width_px, height_px) < 4.0:
        return None

    metrics = describe_shape(contour)
    if metrics is None:
        return None
    # 碎片掩膜的轮廓一律丢掉，不管形状看起来像什么。
    if metrics["solidity"] < SHAPE_SOLIDITY_MIN:
        return None

    # 主轴方向：minAreaRect 给的是"矩形边与 x 轴夹角"，长边才是主轴。
    major_angle_deg = angle_deg if width_px >= height_px else angle_deg - 90.0
    confidence = int(1000 * max(0.05, min(1.0, metrics["rect_fill"])) *
                     max(0.2, min(1.0, area / (4.0 * min_area_px))))

    return {
        "cls": class_code,
        "center_px": (float(center_x_px), float(center_y_px)),
        "major_angle_deg": float(major_angle_deg),
        "area_px": float(area),
        "shape": metrics["shape"],
        "kind": kind or metrics["shape"],
        "circle_fill": metrics["circle_fill"],
        "rect_fill": metrics["rect_fill"],
        "aspect": metrics["aspect"],
        "solidity": metrics["solidity"],
        "radius_px": metrics["radius_px"],
        "conf_permille": confidence,
        "contour": contour,
    }


def filter_by_shape(candidates, wanted):
    """按几何形状过滤候选。wanted 为 None / "any" 时原样返回。

    这是"只认圆形"的开关：圆度不合格的候选（背景杂物、色块边缘、反光斑）
    在这一步就被踢掉，不会进到选目标和换算坐标。
    """
    if wanted in (None, "", "any"):
        return candidates
    return [item for item in candidates if item["shape"] == wanted]


def save_debug_masks(directory, frame_hsv, colors, min_area_px, frame_index):
    """把每个类的掩膜存一张图，用来回答"为什么这个颜色检不出来"。

    直接看 HSV 区间很难判断问题出在阈值、形态学还是阴影，看图一眼就知道。
    """
    import cv2

    os.makedirs(directory, exist_ok=True)
    for key, entry in colors.get("classes", {}).items():
        if not key.isdigit():
            continue
        if entry.get("mode") == "dark":
            mask = build_dark_mask(frame_hsv, entry, colors, min_area_px)
            suffix = "dark"
        else:
            mask = color_mask(frame_hsv, entry)
            suffix = "hsv"
        if mask is None:
            continue
        name = f"{frame_index:04d}_cls{key}_{entry.get('name', '')}_{suffix}.png"
        cv2.imwrite(os.path.join(directory, name), mask)


def detect_blobs(frame_hsv, colors, min_area_px):
    """按 colors.json 里每个颜色找色块（圆柱 / 长方体）。

    两类目标走两条路：
      - 彩色目标：HSV 区间掩膜 + 形态学 + 轮廓，就是原来的做法
      - 暗色目标（entry 里写了 `"mode": "dark"`，例如黑色）：见 build_dark_mask()

    每个类可以额外带这些可选字段，只对这一类生效：
        min_area_px     覆盖全局 --min-area-px
        min_circle_fill 外接圆填充率下限
        min_solidity    凸包填充率下限
        dark_ratio / max_v / max_s / shadow_guard_px   仅暗色模式有效
    """
    import cv2

    # 先把彩色掩膜建好：暗色目标算"阴影禁区"时要用，别重复建一遍。
    color_masks = {}
    for key, entry in colors.get("classes", {}).items():
        if not key.isdigit() or entry.get("mode") == "dark":
            continue
        mask = color_mask(frame_hsv, entry)
        if mask is not None:
            color_masks[key] = mask

    candidates = []
    for key, entry in colors.get("classes", {}).items():
        if not key.isdigit():
            continue

        if entry.get("mode") == "dark":
            mask = build_dark_mask(frame_hsv, entry, colors, min_area_px, color_masks)
        else:
            mask = color_masks.get(key)
        if mask is None:
            continue

        class_min_area = int(entry.get("min_area_px", min_area_px))
        min_circle_fill = float(entry.get("min_circle_fill", 0.0))
        min_solidity = float(entry.get("min_solidity", 0.0))

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            candidate = candidate_from_contour(contour, int(key), class_min_area)
            if not candidate:
                continue
            if candidate["circle_fill"] < min_circle_fill:
                continue
            if candidate["solidity"] < min_solidity:
                continue
            candidates.append(candidate)
    return candidates


def colorful_target_mask(frame_hsv, colors, min_area_px, guard_px, color_masks=None):
    """把所有彩色类的掩膜并起来再向外扩张，得到"阴影禁区"。

    用途是给暗色目标判据提供先验：阴影永远贴在彩色物体旁边，黑料是孤立的。
    扩张圈里的暗区一律不当成黑料。
    color_masks 给了就直接复用（detect_blobs 已经建过一遍，别重复算）。
    """
    import cv2

    union = None
    for key, entry in colors.get("classes", {}).items():
        if not key.isdigit() or entry.get("mode") == "dark":
            continue
        if color_masks is not None:
            mask = color_masks.get(key)
        else:
            mask = color_mask(frame_hsv, entry)
        if mask is None:
            continue
        # 只保留面积够大的块：反光噪点不该扩张出保护圈，否则会吃掉黑料
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        clean = np.zeros_like(mask)
        for contour in contours:
            if cv2.contourArea(contour) >= max(120, min_area_px // 2):
                cv2.drawContours(clean, [contour], -1, 255, -1)
        union = clean if union is None else cv2.bitwise_or(union, clean)

    if union is None:
        return None

    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,
                                       (guard_px * 2 + 1, guard_px * 2 + 1))
    return cv2.dilate(union, kernel)


def build_dark_mask(frame_hsv, entry, colors, min_area_px, color_masks=None):
    """暗色目标（黑料）掩膜：比局部背景暗、绝对亮度不高、且不贴着彩色物体。

    为什么不直接卡 HSV 区间：黑料的 V 和它旁边阴影的 V 是同一个量级，
    而且补光在物料顶部打出高光后，黑料中心的 V 反而可能高于旁边的暗部。
    绝对阈值怎么调都是"压掉阴影就连黑料一起压掉"。

    背景亮度在 1/8 分辨率上估再放大：640×480 直接对全图做 σ≈102 的高斯要 700 ms，
    缩到 80×60 只要 ~1 ms。背景只是"慢变化的亮度趋势"，低分辨率完全够用。
    """
    import cv2

    value = frame_hsv[:, :, 2]
    saturation = frame_hsv[:, :, 1]
    height_px, width_px = value.shape

    # ① 局部背景亮度：低分辨率高斯估计，再插值回原尺寸
    small_w = max(16, width_px // 8)
    small_h = max(8, height_px * small_w // max(1, width_px))
    small = cv2.resize(value, (small_w, small_h), interpolation=cv2.INTER_AREA)
    sigma_small = max(1.0, small_w * BACKGROUND_SIGMA_SCALE)
    background_small = cv2.GaussianBlur(small, (0, 0), sigmaX=sigma_small, sigmaY=sigma_small)
    background = cv2.resize(background_small, (width_px, height_px),
                            interpolation=cv2.INTER_LINEAR)
    background = np.maximum(background.astype(np.float32), 1.0)

    dark_ratio = float(entry.get("dark_ratio", BLACK_DARK_RATIO))
    max_v = int(entry.get("max_v", BLACK_MAX_V))
    max_s = int(entry.get("max_s", BLACK_MAX_S))

    darker = (background - value.astype(np.float32)) / background
    mask = ((darker >= dark_ratio) & (value <= max_v) &
            (saturation <= max_s)).astype(np.uint8) * 255

    # ② 形态学：闭运算填掉高光造成的小洞，开运算去掉细小的暗噪点
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)

    # ③ 排除贴着彩色物体的暗区 —— 阴影永远在彩色目标旁边
    guard_px = int(entry.get("shadow_guard_px", DEFAULT_SHADOW_GUARD_PX))
    guard_mask = colorful_target_mask(frame_hsv, colors, min_area_px, guard_px, color_masks)
    if guard_mask is not None:
        mask[guard_mask > 0] = 0

    return mask


def detect_tray(gray, min_area_px):
    """载物盘：取画面里最大的凸四边形。靠外框而不是靠颜色，换底色也能用。"""
    import cv2

    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    edges = cv2.Canny(blurred, 60, 150)
    edges = cv2.dilate(edges, np.ones((3, 3), np.uint8), iterations=1)
    contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    best = None
    for contour in contours:
        area = cv2.contourArea(contour)
        if area < min_area_px:
            continue
        perimeter = cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, 0.02 * perimeter, True)
        if len(approx) != 4 or not cv2.isContourConvex(approx):
            continue
        if best is None or area > best[0]:
            best = (area, approx)

    if best is None:
        return []
    area, quad = best
    rect = cv2.minAreaRect(quad)
    (center_x_px, center_y_px), (width_px, height_px), angle_deg = rect
    major_angle_deg = angle_deg if width_px >= height_px else angle_deg - 90.0
    return [{
        "cls": CLASS_TRAY,
        "center_px": (float(center_x_px), float(center_y_px)),
        "major_angle_deg": float(major_angle_deg),
        "area_px": float(area),
        "shape": "tray",
        "kind": "tray",
        "circle_fill": 0.0,
        "rect_fill": 1.0,
        "aspect": 1.0,
        "solidity": 1.0,
        "radius_px": 0.0,
        "conf_permille": 600,
        "contour": quad,
    }]


def detect_rings(frame_hsv, colors, min_area_px):
    """色环：颜色掩膜里"带子轮廓的外轮廓"，且内外面积比落在环的合理范围。"""
    import cv2

    candidates = []
    for key, entry in colors.get("classes", {}).items():
        if not key.isdigit():
            continue
        mask = color_mask(frame_hsv, entry)
        if mask is None:
            continue
        contours, hierarchy = cv2.findContours(mask, cv2.RETR_CCOMP,
                                               cv2.CHAIN_APPROX_SIMPLE)
        if hierarchy is None:
            continue
        hierarchy = hierarchy[0]
        for index, contour in enumerate(contours):
            if hierarchy[index][3] != -1:      # 有父轮廓 = 是洞，不是环本身
                continue
            outer_area = cv2.contourArea(contour)
            if outer_area < min_area_px:
                continue
            # 找它最大的子轮廓（内孔）
            child = hierarchy[index][2]
            inner_area = 0.0
            while child != -1:
                inner_area = max(inner_area, cv2.contourArea(contours[child]))
                child = hierarchy[child][0]
            if inner_area <= 0.0:
                continue
            ratio = inner_area / outer_area
            if not (0.20 <= ratio <= 0.80):
                continue
            candidate = candidate_from_contour(contour, CLASS_RING, min_area_px, "ring")
            if not candidate:
                continue
            # 面积比越接近 0.45（典型色环）置信度越高
            candidate["conf_permille"] = int(1000 * max(0.1, 1.0 - abs(ratio - 0.45) / 0.35))
            candidate["inner_outer_ratio"] = float(ratio)
            candidates.append(candidate)
    return candidates


def pick_candidate(candidates, want_class):
    """选一个输出：指定了类别就只考虑它，否则取面积最大的（最近/最清楚）。"""
    pool = [item for item in candidates if want_class is None or item["cls"] == want_class]
    if not pool:
        return None
    return max(pool, key=lambda item: item["area_px"])


# ---------------------------------------------------------------------------
# 像素 → 车体坐标
# ---------------------------------------------------------------------------

def candidate_to_robot(candidate, homography, mount_deg=0.0):
    """把候选目标换算成车体坐标：中心点 + 主轴方向。

    方向一律用"中心 + 主轴外推 12 px"两点换算，免去相机朝向假设：
    不管相机怎么歪装，两点连线求 atan2 都对。

    mount_deg 是相机安装角：五点单应按物理角序标定时单应已经吸收了安装角，
    保持 0；如果当时是照着画面方向点的，这里补上 90（或 -90）。
    """
    center_x_px, center_y_px = candidate["center_px"]
    point = pixel_to_plane_h(center_x_px, center_y_px, homography)
    if point is None:
        return None

    theta = math.radians(candidate["major_angle_deg"])
    probe = pixel_to_plane_h(center_x_px + 12.0 * math.cos(theta),
                             center_y_px + 12.0 * math.sin(theta), homography)
    yaw_mdeg = 0
    if probe is not None:
        delta_x = probe[0] - point[0]
        delta_y = probe[1] - point[1]
        if abs(delta_x) > 1e-6 or abs(delta_y) > 1e-6:
            yaw_mdeg = normalize_yaw_mdeg(math.degrees(math.atan2(delta_y, delta_x)) * 1000.0)
    x_mm, y_mm = rotate_to_body(point[0], point[1], mount_deg)
    yaw_mdeg = normalize_yaw_mdeg(yaw_mdeg + mount_deg * 1000.0)
    return x_mm, y_mm, yaw_mdeg


# ---------------------------------------------------------------------------
# 预览绘制
# ---------------------------------------------------------------------------

def annotate(frame, candidate, robot, label):
    """画选中的那一个：绿色外接框 + 红色十字 + 文字。"""
    import cv2

    if candidate is None:
        return frame
    corners = cv2.boxPoints(cv2.minAreaRect(candidate["contour"])).astype(int)
    cv2.drawContours(frame, [corners], -1, (0, 255, 0), 2)
    center = (int(candidate["center_px"][0]), int(candidate["center_px"][1]))
    cv2.drawMarker(frame, center, (0, 0, 255), cv2.MARKER_CROSS, 18, 2)
    text = f"{label}"
    if robot:
        text += f" x={robot[0]:.0f} y={robot[1]:.0f} mm yaw={robot[2] / 1000.0:.1f} deg"
    cv2.putText(frame, text, (center[0] + 8, center[1] - 8),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
    return frame


def annotate_all(frame, candidates):
    """把这一帧的全部候选都画出来，每个类一个颜色。

    单目标预览只能看到 pick_candidate 选中的那一个，看不出别的颜色有没有被误检。
    这个模式专门用来查颜色区间串不串：如果一块绿料同时冒出 4 和 5 两个框，
    说明两条区间重叠了。
    """
    import cv2

    for candidate in candidates:
        contour = candidate["contour"]
        corners = cv2.boxPoints(cv2.minAreaRect(contour)).astype(int)
        color = DRAW_COLORS_BGR.get(candidate["cls"], (200, 200, 200))
        cv2.drawContours(frame, [corners], -1, color, 2)
        center = (int(candidate["center_px"][0]), int(candidate["center_px"][1]))
        text = f"{candidate['cls']} cf{candidate['circle_fill']:.2f}"
        cv2.putText(frame, text, (center[0] + 6, center[1] - 6),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

    counts = {}
    for candidate in candidates:
        counts[candidate["cls"]] = counts.get(candidate["cls"], 0) + 1
    summary = " ".join(f"{key}x{value}" for key, value in sorted(counts.items()))
    cv2.putText(frame, summary or "none", (8, frame.shape[0] - 12),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)
    return frame


# ---------------------------------------------------------------------------
# 主流程
# ---------------------------------------------------------------------------

def main(argv=None):
    parser = argparse.ArgumentParser(description="视觉节点：目标位姿 -> 串口帧")
    parser.add_argument("--source", choices=("camera", "image", "dir"), default="camera")
    parser.add_argument("--camera", default=DEFAULT_CAMERA,
                        help="相机序号或设备路径（如 0 或 /dev/cam_target）")
    parser.add_argument("--width", type=int, default=640, help="请求的画面宽度，单位 px")
    parser.add_argument("--height", type=int, default=480, help="请求的画面高度，单位 px")
    parser.add_argument("--image", help="--source image 时的图片路径")
    parser.add_argument("--dir", help="--source dir 时的图片目录")
    parser.add_argument("--task", choices=("color", "tray", "ring"), default="color")
    parser.add_argument("--want", type=int, default=None,
                        help="只输出这个类别（1..6 颜色，10 载物盘，11 色环）；"
                             "运行时会被固件发的 $C,<cls> 覆盖")
    parser.add_argument("--homography", default=None,
                        help="五点单应文件（calib_plane_h.json）。没有它只能预览，发不了毫米坐标")
    parser.add_argument("--mount-deg", type=float, default=0.0,
                        help="相机相对车头的安装角，逆时针为正。单应按物理角序标定时保持 0；"
                             "照画面方向点的话用 90 / -90")
    parser.add_argument("--colors", default="colors.json")
    parser.add_argument("--port", default=None,
                        help="串口，如 /dev/serial0 或 COM7；不给就自动找（/dev/serial0、USB-TTL）")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--hz", type=float, default=10.0, help="发送频率上限")
    parser.add_argument("--min-area-px", type=int, default=DEFAULT_MIN_AREA_PX)
    parser.add_argument("--shape", choices=("any", "circle", "rect"), default="any",
                        help="只保留这种几何形状的候选；any = 不过滤。"
                             "circle 用来滤掉背景杂物和色块边缘")
    add_preview_argument(parser)
    add_send_argument(parser)
    parser.add_argument("--show-all", action="store_true",
                        help="预览时画出全部候选（每类一个颜色），用来查颜色区间串不串")
    parser.add_argument("--hold", action="store_true", default=None,
                        help="预览每帧等按键再继续（默认：图片源等，相机源不等）")
    parser.add_argument("--debug-mask",
                        help="把每个类的掩膜存成图片到该目录，用来查为什么某个颜色检不出")
    parser.add_argument("--max-frames", type=int, default=0, help="只处理这么多帧后退出")
    parser.add_argument("--save-dir",
                        help="存原始帧的目录（只在 --source camera 时生效）")
    parser.add_argument("--save-every", type=int, default=10,
                        help="每 N 帧存一张，默认 10")
    parser.add_argument("--max-save", type=int, default=0,
                        help="最多存多少张，0 = 不限")
    args = parser.parse_args(argv)

    try:
        import cv2
    except ImportError:
        print("缺少 opencv：pip3 install opencv-python numpy")
        return 2

    # 标定只有一个入口：五点单应。没有它就只预览/存图，不发 $V（发出去也是废坐标）。
    homography = None
    if args.homography:
        homography = load_homography(args.homography)
        if homography is None:
            print(f"{args.homography} 里没有 homography 字段，先跑 calib_plane_h.py")
            return 1
        print(f"用单应标定 {args.homography}（像素 → 车体毫米）")
    else:
        print("没有单应标定：只预览，不发 $V。要发先跑：")
        print(f"    python3 calib_plane_h.py --camera {args.camera} --square-mm 100")

    colors = load_json(args.colors, default={"classes": {}})
    if args.task in ("color", "ring") and not colors.get("classes"):
        print(f"{args.colors} 是空的：先用 calib_color.py 标定颜色")
        return 1

    # 运行时可被固件下发的 `$C,<cls>` 改掉（蓝牙发 GRAB BLUE 也走这条）。
    want_class = args.want
    hint_buffer = b""

    link = None
    if args.send:
        link = open_link(args.port, args.baud)
        if link and homography is None:
            print("串口已打开，但没有单应标定 → 不发 $V（发了也是 0 坐标）")
    else:
        print("串口发送已关闭（--no-send）：只出画面/打印帧")

    capture = None
    image_list = []
    if args.source == "camera":
        capture = open_camera(parse_camera_source(args.camera),
                              width_px=args.width, height_px=args.height)
        if capture is None:
            print(f"打不开相机 {args.camera}")
            print("PC 上先试 0/1；树莓派用 v4l2-ctl --list-devices 确认设备名，"
                  "两路相机必须用 udev 固定，否则 /dev/videoN 会随插拔顺序互换")
            return 1
    elif args.source == "image":
        if not args.image:
            print("--source image 需要 --image")
            return 2
        image_list = [args.image]
    else:
        if not args.dir:
            print("--source dir 需要 --dir")
            return 2
        image_list = [os.path.join(args.dir, name) for name in sorted(os.listdir(args.dir))
                      if name.lower().endswith((".jpg", ".jpeg", ".png", ".bmp"))]

    # 存图只在相机源下有意义：image / dir 源本身已经是磁盘上的文件了。
    save_dir = args.save_dir if capture is not None else None
    if args.save_dir and save_dir is None:
        print("提示：--save-dir 只在 --source camera 下生效，本次忽略")
    elif save_dir:
        os.makedirs(save_dir, exist_ok=True)
        tail = f"，最多 {args.max_save} 张" if args.max_save > 0 else ""
        print(f"样本存到 {save_dir}，每 {max(1, args.save_every)} 帧一张{tail}")

    preview = Preview("vision_node") if args.preview else None
    if preview:
        print("实时画面：q / ESC 退出")
    hold = args.hold if args.hold is not None else args.source != "camera"
    wait_ms = 0 if hold else 1

    period_s = 1.0 / max(0.5, args.hz)
    seq = 0
    frames = 0
    saved = 0
    read_fails = 0
    sent = 0
    last_send_s = 0.0
    none_announced_s = 0.0
    last_report_s = 0.0
    last_frame_s = time.time()

    try:
        while True:
            started_s = time.time()
            if capture is not None:
                ok, frame = capture.read()
                if not ok or frame is None:
                    read_fails += 1
                    if read_fails >= MAX_READ_FAILS:
                        print(f"相机连续 {read_fails} 次读不到帧，停止。")
                        print("最常见的原因：设备名绑到了 UVC 元数据节点"
                              "（/dev/v4l/by-path/*-video-index1），或被别的程序占用。")
                        return 1
                    time.sleep(0.02)
                    continue
                read_fails = 0
                # 存**原始帧**（未做任何校正）。标单应、标颜色都要求原图。
                if save_dir and (frames % max(1, args.save_every) == 0):
                    if args.max_save <= 0 or saved < args.max_save:
                        path = os.path.join(save_dir, f"{args.task}_{saved:04d}.png")
                        if cv2.imwrite(path, frame):
                            saved += 1
                            print(f"存图 {path}")
                        else:
                            print(f"存图失败 {path}")
            else:
                if frames >= len(image_list):
                    break
                frame = cv2.imread(image_list[frames])
                if frame is None:
                    frames += 1
                    continue

            frame_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            # 固件每次 ALIGN/TRACK/GRAB 启动都会发 `$C,<cls>`，运行时切换目标类别。
            hint, hint_buffer = poll_class_hint(link, hint_buffer)
            if hint is not None and hint != want_class:
                want_class = hint
                print(f"[STM32] 目标类别切到 {want_class}")

            if args.task == "color":
                candidates = detect_blobs(frame_hsv, colors, args.min_area_px)
                if args.debug_mask:
                    save_debug_masks(args.debug_mask, frame_hsv, colors,
                                     args.min_area_px, frames)
            elif args.task == "tray":
                candidates = detect_tray(gray, args.min_area_px)
            else:
                candidates = detect_rings(frame_hsv, colors, args.min_area_px)

            candidates = filter_by_shape(candidates, args.shape)
            candidate = pick_candidate(candidates, want_class)
            robot = None
            if candidate and homography is not None:
                robot = candidate_to_robot(candidate, homography, args.mount_deg)

            frames += 1
            now_s = time.time()

            # 没有单应时不发 $V，但终端也要能看出"检到了什么"——0.5 s 一条，
            # 这样纯 ssh / 无窗口调试也有反馈。
            if homography is None and (now_s - last_report_s) >= 0.5:
                last_report_s = now_s
                if candidate:
                    print(f"[detect] cls={candidate['cls']} {candidate['kind']} "
                          f"cf={candidate['circle_fill']:.2f} "
                          f"area={candidate['area_px']:.0f}px "
                          f"px=({candidate['center_px'][0]:.0f},"
                          f"{candidate['center_px'][1]:.0f})")
                else:
                    print("[detect] none")

            # 发送：有目标就限频发；没目标时按 1 Hz 发一条 cls=0，
            # 让 STM32 能分清"视野里没东西"和"链路断了"。
            due = False
            if homography is None:
                pass   # 没有坐标换算，发出去也是废帧；宁可什么都不发
            elif candidate and robot and (now_s - last_send_s) >= period_s:
                due = True
            elif not candidate and (now_s - none_announced_s) >= 1.0:
                due = True

            if due:
                age_ms = int((now_s - started_s) * 1000.0)
                if candidate and robot:
                    line = format_line(seq, candidate["cls"], robot[0], robot[1],
                                       robot[2], candidate["conf_permille"], age_ms)
                else:
                    line = format_line(seq, 0, 0, 0, 0, 0, age_ms)
                seq += 1
                last_send_s = now_s
                if not candidate:
                    none_announced_s = now_s
                if link:
                    link.write(line)
                    sent += 1
                print(line.decode("ascii").strip())

            if preview:
                if args.show_all:
                    annotate_all(frame, candidates)
                else:
                    label = "none" if not candidate else \
                        f"cls={candidate['cls']} {candidate['kind']} " \
                        f"cf={candidate['circle_fill']:.2f}"
                    annotate(frame, candidate, robot, label)
                fps = 1.0 / max(1e-3, now_s - last_frame_s)
                last_frame_s = now_s
                cv2.putText(frame, f"{fps:.0f} fps", (8, 22),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                preview.show(frame, wait_ms)
                if preview.quit:
                    break

            if args.max_frames and frames >= args.max_frames:
                break
    except KeyboardInterrupt:
        print("\n停止")
    finally:
        if capture is not None:
            capture.release()
        if link:
            link.close()
        if preview:
            preview.close()
        close_all_windows()

    print(f"处理 {frames} 帧，发出 {sent} 帧，存图 {saved} 张")
    return 0


if __name__ == "__main__":
    sys.exit(main())
