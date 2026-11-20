# -*- coding: utf-8 -*-
"""视觉主入口：两路相机（视觉 + 扫码）、串口、STM32 指令分发。

一个进程做完：
    · 视觉相机：颜色/黑料/圆环检测，按 10 Hz 往 USART3 发 `$V,...`；
    · 扫码相机：收到 STM32 的 `@SCAN` 时扫二维码；
    · 串口下行：`$C,<cls>[,<n>]` 切目标类别/环号；`@SCAN` 触发扫码；
    · 扫码后：发 `TASK <code>` 上屏，解析两批颜色顺序，写 `task_plan.json`。

用法（树莓派）：
    python3 main.py --camera /dev/cam_target --qr-camera /dev/video2
    python3 main.py --camera 0 --qr-camera 1          # PC 上两个 USB 相机
    python3 main.py --no-send                        # 只预览，不发不扫

模块分工：
    common.py       串口/相机/CRC/单应等基础工具
    align_send.py   检测引擎（颜色、环、数字），本文件只做调度
    qr_scan.py      二维码解码与扫码相机
    task_rules.py   任务码解析（两批颜色/圆环/顺序）
"""

import argparse
import json
import os
import sys
import time

import cv2
import numpy as np

from common import (DEFAULT_CAMERA, Preview,
                    add_preview_argument, format_line, load_json,
                    open_camera, open_link, parse_camera_source,
                    parse_class_hint_full)

import align_send as engine
import qr_scan
import task_rules

RING_CLASS = engine.RING_CLASS
DEFAULT_DETECT_SCALE = engine.DEFAULT_DETECT_SCALE
DEFAULT_PX_PER_MM = engine.DEFAULT_PX_PER_MM
DEFAULT_ROT_DEG = engine.DEFAULT_ROT_DEG
MIN_AREA_PX = engine.MIN_AREA_PX
DEFAULT_RING_MIN_RADIUS = engine.DEFAULT_RING_MIN_RADIUS
DEFAULT_RING_PARAM2 = engine.DEFAULT_RING_PARAM2


def resolve_camera(value, default, udev_name):
    """树莓派上优先用 udev 固定名：默认值且 /dev/cam_* 存在就换过去。"""
    if value == default and os.path.exists(udev_name):
        return udev_name
    return value


def describe_controls(capture):
    """打印曝光/白平衡当前值，确认自动有没有真的生效。"""
    if capture is None:
        return "未打开"
    try:
        auto_exp = capture.get(cv2.CAP_PROP_AUTO_EXPOSURE)
        exposure = capture.get(cv2.CAP_PROP_EXPOSURE)
        auto_wb = capture.get(cv2.CAP_PROP_AUTO_WB)
        wb = capture.get(cv2.CAP_PROP_WB_TEMPERATURE)
        return (f"auto_exp={auto_exp:.0f} exposure={exposure:.0f} "
                f"auto_wb={auto_wb:.0f} wb={wb:.0f}K")
    except Exception:
        return "未知"


def describe_capture(capture):
    """打印相机实际分辨率/格式/帧率，排查“设了没生效”。"""
    if capture is None:
        return "未打开"
    try:
        width = int(capture.get(cv2.CAP_PROP_FRAME_WIDTH))
        height = int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
        raw = int(capture.get(cv2.CAP_PROP_FOURCC))
        fourcc = "".join(chr((raw >> (8 * i)) & 0xFF) for i in range(4))
        fps = capture.get(cv2.CAP_PROP_FPS)
        return f"{width}x{height} {fourcc} {fps:.0f}fps"
    except Exception:
        return "未知"



def drain_lines(link, buffer):
    """非阻塞读串口，按行拆开；没有 link 返回空。"""
    if link is None:
        return [], buffer
    try:
        pending = link.in_waiting
    except Exception:
        return [], buffer
    if pending:
        try:
            buffer += link.read(pending)
        except Exception:
            return [], buffer

    lines = []
    while b"\n" in buffer:
        raw, buffer = buffer.split(b"\n", 1)
        lines.append(raw.strip())
    if len(buffer) > 256:          # 半截垃圾不无限攒
        buffer = buffer[-256:]
    return lines, buffer


def handle_scan(qr_capture, link, args):
    """收到 @SCAN：扫二维码 → 发 TASK 上屏 → 解析顺序并写文件。返回识别到的内容。"""
    if qr_capture is None:
        print("\n[扫码] 没有扫码相机（--qr-camera 没打开），跳过")
        return None
    print("\n[扫码] 收到 @SCAN，开始识别...")
    code = qr_scan.scan_from_capture(qr_capture, args.qr_timeout)
    if not code:
        print("[扫码] 没扫到二维码")
        return None

    print(f"[扫码] 二维码：{code!r}")
    if link is not None:
        line = f"TASK {code}\n".encode("ascii")
        link.write(line)
        print(f"[扫码] TX {line!r}")

    try:
        plan = task_rules.parse(code)
    except ValueError as error:
        print(f"[扫码] 任务码无效：{error}")
        return code

    print("[扫码] " + task_rules.describe(plan).replace("\n", "\n[扫码] "))
    print(f"[扫码] 抓取顺序：{plan['grab_order']}")
    if args.plan_out:
        try:
            with open(args.plan_out, "w", encoding="utf-8") as file:
                json.dump(plan, file, ensure_ascii=False, indent=2)
            print(f"[扫码] 写入 {args.plan_out}")
        except OSError as error:
            print(f"[扫码] 写 {args.plan_out} 失败：{error}")
    return code


def main(argv=None):
    parser = argparse.ArgumentParser(description="视觉主入口：双相机 + 串口 + 任务解析")
    parser.add_argument("--camera", default=DEFAULT_CAMERA, help="视觉相机：序号或设备路径")
    parser.add_argument("--qr-camera", default="1", help="扫码相机：序号或设备路径")
    parser.add_argument("--qr-width", type=int, default=640)
    parser.add_argument("--qr-height", type=int, default=480)
    parser.add_argument("--qr-fps", type=float, default=30.0)
    parser.add_argument("--qr-fourcc", default="MJPG",
                        help="扫码相机像素格式：MJPG（默认，省带宽）/ YUYV")
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--cam-fourcc", default=None,
                        help="视觉相机格式（如 MJPG）；默认不强制，带宽不够时用")
    parser.add_argument("--cam-fps", type=float, default=None,
                        help="视觉相机帧率；默认不强制")
    parser.add_argument("--cam-exposure", type=int, default=120,
                        help="视觉相机手动曝光（100us 单位，默认 120=12ms）")
    parser.add_argument("--cam-wb", type=int, default=None,
                        help="视觉相机手动白平衡色温 K（如 4600）；默认不改")
    parser.add_argument("--cam-lock-exposure", action="store_true",
                        help="锁手动曝光/白平衡（默认自动）；配合 --cam-exposure/--cam-wb 用")
    parser.add_argument("--cam-auto-exposure", action="store_true",
                        help="（兼容）自动曝光/白平衡；现在默认就是自动，加不加一样")
    parser.add_argument("--port", default=None, help="串口，默认自动找 /dev/serial0")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--cls", type=int, default=3,
                        help="初始目标类别 1-6/10/11；固件 $C 会覆盖")
    parser.add_argument("--px-per-mm", type=float, default=DEFAULT_PX_PER_MM)
    parser.add_argument("--rot-deg", type=float, default=DEFAULT_ROT_DEG,
                        help="相机相对车头安装角（本车 90）")
    parser.add_argument("--arm-rot-deg", type=float, default=132.0,
                        help="转台相对车体角度；仅 --ring-use-arm 时使用")
    parser.add_argument("--ring-use-arm", action="store_true",
                        help="环模式叠加转台角（相机随转台转时用）；默认不叠加，和物料跟踪同用 --rot-deg")
    parser.add_argument("--ring-rot-deg", type=float, default=None,
                        help="直接指定环模式总旋转角，优先级最高")
    parser.add_argument("--min-area-px", type=int, default=MIN_AREA_PX)
    parser.add_argument("--detect", choices=("all", "single"), default="all")
    parser.add_argument("--detect-scale", type=float, default=DEFAULT_DETECT_SCALE)
    parser.add_argument("--show-all", action="store_true")
    parser.add_argument("--hz", type=float, default=10.0)
    parser.add_argument("--colors", default="colors.json")
    parser.add_argument("--plan-out", default="task_plan.json",
                        help="扫码解析结果写到这里")
    parser.add_argument("--qr-timeout", type=float, default=10.0,
                        help="每次扫码最长等多少秒")
    parser.add_argument("--qr-preview", dest="qr_preview", action="store_true",
                        default=False, help="显示扫码相机窗口（默认关，省 VNC/CPU）")
    parser.add_argument("--no-qr-preview", dest="qr_preview", action="store_false",
                        help="不显示扫码相机窗口")
    parser.add_argument("--no-send", action="store_true",
                        help="只预览，不开串口/不发 $V/不处理 $C 与 @SCAN")
    parser.add_argument("--require-serial", action="store_true",
                        help="串口打不开就退出")
    # 环模式
    parser.add_argument("--ring-px-per-mm", type=float, default=0.0,
                        help="底部精对准的 px/mm（在释放高度量）")
    parser.add_argument("--ring-px-per-mm-top", type=float, default=0.0,
                        help="顶部粗对准的 px/mm（在最高处量）；0 = 用 --ring-px-per-mm")
    parser.add_argument("--ring-outer-diameter-mm", type=float, default=0.0,
                        help="圆环外径 mm；>0 时按每帧检出的半径自动算 px/mm，顶部/底部通吃")
    parser.add_argument("--ring-min-radius", type=int, default=DEFAULT_RING_MIN_RADIUS)
    parser.add_argument("--ring-max-radius", type=int, default=0)
    parser.add_argument("--ring-param2", type=float, default=DEFAULT_RING_PARAM2)
    parser.add_argument("--ring-min-circles", type=int, default=3,
                        help="底部精对至少几个不同半径的同心圆才算环（默认 3）")
    parser.add_argument("--ring-min-circles-top", type=int, default=2,
                        help="顶部粗对至少几个同心圆（默认 2，顶部环小）")
    parser.add_argument("--ring-index", type=int, default=1)
    parser.add_argument("--ring-digit-check", choices=("off", "warn", "strict"), default="warn")
    parser.add_argument("--ring-digit-min-score", type=float, default=0.35)
    parser.add_argument("--ring-offset-x-mm", type=float, default=0.0)
    parser.add_argument("--ring-offset-y-mm", type=float, default=0.0)
    parser.add_argument("--ignore-top", type=float, default=0.10,
                        help="忽略画面上部比例（0..0.5），默认 0.10")
    parser.add_argument("--detect-always", action="store_true",
                        help="不等 @DETECT，始终识别（调试用）")
    parser.add_argument("--shape-fallback", dest="shape_fallback", action="store_true",
                        default=True,
                        help="颜色完全没检出时，先按圆形/圆柱轮廓找目标，再判颜色（默认开）")
    parser.add_argument("--no-shape-fallback", dest="shape_fallback", action="store_false",
                        help="关闭外形兜底")
    parser.add_argument("--shape-min-area-px", type=int, default=600)
    parser.add_argument("--shape-max-area-px", type=int, default=20000)
    parser.add_argument("--shape-min-circularity", type=float, default=0.60)
    parser.add_argument("--shape-min-fill", type=float, default=0.55)
    parser.add_argument("--shape-color-min-frac", type=float, default=0.20)
    parser.add_argument("--shape-scale", type=float, default=0.5,
                        help="外形兜底检测缩放，默认 0.5；卡就降到 0.35")
    parser.add_argument("--shape-hz", type=float, default=5.0,
                        help="外形兜底最多每秒跑几次，默认 5")
    parser.add_argument("--smooth-alpha", type=float, default=0.5,
                        help="目标中心低通系数，越小越稳但越滞后（默认 0.5）")
    parser.add_argument("--smooth-jump-px", type=float, default=80.0,
                        help="中心跳变超过这个像素就重置平滑（默认 80）")
    parser.add_argument("--center-mode", choices=("contour", "circle", "blend", "distance"),
                        default="contour",
                        help="物料中心：contour 质心 / circle 外接圆 / blend 平均 / distance 最大内接圆")
    add_preview_argument(parser)
    args = parser.parse_args(argv)

    colors = load_json(args.colors)
    if colors is None or not colors.get("classes"):
        print(f"找不到 {args.colors} 里的颜色")
        return 1
    # 只有三种都没给时才退回颜色 px/mm；否则保留 0，让底部精对能走外径自动算。
    if (args.ring_px_per_mm <= 0.0 and args.ring_px_per_mm_top <= 0.0 and
            args.ring_outer_diameter_mm <= 0.0):
        args.ring_px_per_mm = args.px_per_mm
    digit_templates = engine.load_digit_templates()
    if not digit_templates:
        print("提示：digit_templates/ 里没有 1/2/3.png，环模式跳过数字识别")

    current_cls = args.cls
    ring_index = max(1, min(9, args.ring_index))
    if args.ring_rot_deg is not None:
        ring_rot_deg = args.ring_rot_deg
    elif args.ring_use_arm:
        ring_rot_deg = args.rot_deg + args.arm_rot_deg
    else:
        ring_rot_deg = args.rot_deg      # 默认和物料跟踪同一套：只用相机安装角
    ring_stage = 1                      # 默认按底部精对准；$C 会带阶段
    ring_digit_message = None
    ring_lock_center = None             # 锁住已选中的环，避免两个环之间来回跳
    ring_lock_radius = 0.0
    ring_lock_stage = None
    ring_lock_miss = 0
    smooth_center = None
    smooth_class = None
    color_lock_center = None
    color_lock_miss = 0                # 锁定目标连续丢几帧；超过 5 帧才允许换环

    camera = resolve_camera(args.camera, DEFAULT_CAMERA, "/dev/cam_target")
    qr_camera = resolve_camera(args.qr_camera, "1", "/dev/cam_qr")

    capture = open_camera(parse_camera_source(camera),
                          width_px=args.width, height_px=args.height,
                          fourcc=args.cam_fourcc, fps=args.cam_fps,
                          lock_exposure=args.cam_lock_exposure,
                          exposure=args.cam_exposure, wb=args.cam_wb)
    if capture is None:
        print(f"打不开视觉相机 {camera}")
        return 1

    qr_capture = qr_scan.open_qr_camera(qr_camera, args.qr_width, args.qr_height,
                                       args.qr_fourcc, args.qr_fps)
    if qr_capture is None:
        print(f"提示：打不开扫码相机 {qr_camera}，@SCAN 会跳过（只跑视觉）")
        print("      树莓派上扫码相机是 /dev/cam_qr；用 ls -l /dev/cam_* 确认")

    link = None if args.no_send else open_link(args.port, args.baud)
    if link is None and args.require_serial and not args.no_send:
        capture.release()
        if qr_capture:
            qr_capture.release()
        return 1
    if args.no_send:
        print("--no-send：只预览，不发 $V、不收 $C/@SCAN")
    elif link is None:
        print("[!] 串口没打开：只预览，收不到 $C/@SCAN，也不发 $V")

    preview = Preview("vision") if args.preview else None
    qr_preview = Preview("qr_scan") if (args.preview and args.qr_preview and
                                       qr_capture is not None) else None
    detect_mode = "all" if args.show_all else args.detect
    force_detect = bool(args.detect_always)     # 热键 a 可切换
    gate_detect = False                          # 固件 @DETECT 1/0
    detect_enabled = force_detect
    period_s = 1.0 / max(1.0, args.hz)
    seq = 0
    none_since_s = None
    hint_buffer = b""
    gripper_px = None
    last_frame_s = time.time()
    last_scan_text = ""
    last_scan_until = 0.0
    last_print_s = 0.0
    last_shape_run_s = 0.0
    shape_cache = []

    print(f"视觉相机 {camera} 实际 {describe_capture(capture)}")
    print(f"视觉相机控制 {describe_controls(capture)}")
    print(f"扫码相机 {qr_camera} 实际 {describe_capture(qr_capture)}")
    ring_mode_note = ("指定" if args.ring_rot_deg is not None
                      else ("rot+arm" if args.ring_use_arm else "相机安装角"))
    print(f"px/mm={args.px_per_mm:.2f} rot={args.rot_deg:.0f} "
          f"环模式总角度={ring_rot_deg:.0f}（{ring_mode_note}）")
    print("等 STM32 指令：$C,<cls>[,<n>] 切目标；@SCAN 扫码")
    print("按键：1-4 中心模式  a 识别常开  s 外形兜底  d 外形调试  v 全色显示  +/- 平滑  r 清锁  p 打印  q 退出")
    if qr_preview:
        print("第二个窗口 qr_scan = 扫码相机；对着二维码调位置")

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

            ignore_top = max(0.0, min(0.5, args.ignore_top))
            roi_top_px = min(int(height_px * ignore_top), max(0, height_px - 2))
            roi = frame[roi_top_px:, :] if roi_top_px > 0 else frame

            if qr_preview is not None and qr_capture is not None:
                ok_qr, qr_frame = qr_capture.read()
                if ok_qr and qr_frame is not None:
                    cv2.putText(qr_frame, "QR camera", (8, 24),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                    qr_preview.show(qr_frame)
                    if qr_preview.quit:
                        break

            lines, hint_buffer = drain_lines(link, hint_buffer)
            for raw in lines:
                if raw.startswith(b"@DETECT"):
                    gate_detect = raw.strip().endswith(b"1")
                    detect_enabled = force_detect or gate_detect
                    ring_lock_center = None
                    ring_lock_miss = 0
                    smooth_center = None
                    color_lock_center = None
                    color_lock_miss = 0
                    print(f"\n[STM32] 识别 {'ON' if detect_enabled else 'OFF'}")
                    continue
                parsed = parse_class_hint_full(raw)
                if parsed is not None:
                    cls, aux, arm_rot, stage = parsed
                    if cls == RING_CLASS:
                        if current_cls != RING_CLASS:
                            print("\n[STM32] 切到环模式")
                        current_cls = RING_CLASS
                        ring_lock_center = None
                        smooth_center = None
                        color_lock_center = None
                        color_lock_miss = 0
                        color_lock_center = None
                        color_lock_miss = 0
                        if aux > 0:
                            ring_index = aux
                            print(f"\n[STM32] 期望第 {ring_index} 环")
                        if arm_rot is not None and args.ring_rot_deg is None:
                            if args.ring_use_arm:
                                ring_rot_deg = args.rot_deg + arm_rot
                                print(f"[STM32] 转台 {arm_rot}°，环模式总角 {ring_rot_deg:.0f}°")
                            else:
                                print(f"[STM32] 转台 {arm_rot}°（默认不叠加，环模式仍用 "
                                      f"{ring_rot_deg:.0f}°）")
                        if stage is not None:
                            ring_stage = stage
                            ring_lock_center = None
                            ring_lock_miss = 0
                            print(f"[STM32] 对准阶段 {ring_stage}（0 顶部粗对 / 1 底部精对）")
                    elif str(cls) in colors["classes"]:
                        current_cls = cls
                        ring_lock_center = None
                        smooth_center = None
                        print(f"\n[STM32] 目标类别切到 {cls}")
                    else:
                        print(f"\n[STM32] 忽略类别 {cls}")
                elif raw.startswith(b"@SCAN"):
                    scanned = handle_scan(qr_capture, link, args)
                    last_scan_text = (f"TASK {scanned}" if scanned else "QR not found")
                    last_scan_until = time.time() + 3.0
                elif raw:
                    text = raw.decode("ascii", "ignore").strip()
                    if text:
                        print(f"\n[STM32] {text}")

            candidates = []
            ring_targets = []
            found = None
            if detect_enabled and current_cls == RING_CLASS:
                min_circles = (args.ring_min_circles_top if ring_stage == 0
                               else args.ring_min_circles)
                ring_targets = engine.detect_rings(roi, args.ring_min_radius,
                                                   args.ring_max_radius, args.ring_param2,
                                                   min_circles)
                if roi_top_px > 0:
                    for target in ring_targets:
                        u, v = target["center_px"]
                        target["center_px"] = (u, v + roi_top_px)
                        target["circles"] = [(cx, cy + roi_top_px, cr)
                                               for cx, cy, cr in target["circles"]]
                if ring_targets:
                    if ring_lock_stage != ring_stage:
                        ring_lock_center = None
                        ring_lock_stage = ring_stage
                        ring_lock_miss = 0
                    chosen = None
                    if ring_lock_center is not None:
                        gate = max(60.0, 0.8 * ring_lock_radius)
                        near = [item for item in ring_targets
                                if ((item["center_px"][0] - ring_lock_center[0]) ** 2 +
                                    (item["center_px"][1] - ring_lock_center[1]) ** 2) ** 0.5
                                <= gate]
                        if near:
                            chosen = min(near, key=lambda item:
                                         (item["center_px"][0] - ring_lock_center[0]) ** 2 +
                                         (item["center_px"][1] - ring_lock_center[1]) ** 2)
                            ring_lock_miss = 0
                        else:
                            # 锁住之后，短时丢目标只发“无目标”，绝不在两个环之间换。
                            ring_lock_miss += 1
                            if ring_lock_miss >= 20:      # 约 2 s 都找不到才解锁
                                ring_lock_center = None
                                ring_lock_miss = 0
                    if chosen is None and ring_lock_center is None and ring_lock_miss == 0:
                        chosen = min(ring_targets,
                                     key=lambda item: (item["center_px"][0] - gripper_px[0]) ** 2 +
                                                      (item["center_px"][1] - gripper_px[1]) ** 2)
                    if chosen is not None:
                        found = chosen
                        ring_lock_center = found["center_px"]
                        ring_lock_radius = found["radius_px"]
                        inner_radius_px = min(cr for _, _, cr in found["circles"])
                        digit, digit_score = None, 0.0
                        if ring_stage != 0:
                            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                            digit, digit_score = engine.read_ring_digit(
                                gray, found["center_px"], inner_radius_px,
                                digit_templates, args.ring_digit_min_score)
                        found["digit"] = digit
                        found["digit_score"] = digit_score
                        if digit is not None and digit != ring_index:
                            key = (ring_index, digit)
                            if key != ring_digit_message:
                                ring_digit_message = key
                                message = (f"[环] 请求第 {ring_index} 环，读到数字 {digit}"
                                           f"（{digit_score:.2f}）")
                                if args.ring_digit_check == "strict":
                                    print("\n" + message + "，strict：不发目标")
                                elif args.ring_digit_check == "warn":
                                    print("\n" + message + "，继续发送")
                            if args.ring_digit_check == "strict":
                                found = None
                        elif digit == ring_index:
                            ring_digit_message = None
                else:
                    ring_lock_miss += 1
                    if ring_lock_miss >= 20:
                        ring_lock_center = None
                        ring_lock_miss = 0
            elif detect_enabled:
                detect_frame, used_scale = engine.shrink_frame(roi, args.detect_scale)
                frame_hsv = cv2.cvtColor(detect_frame, cv2.COLOR_BGR2HSV)
                detect_min_area = max(30, int(args.min_area_px * used_scale * used_scale))
                if detect_mode == "single":
                    candidates = engine.detect_class(frame_hsv, colors, current_cls,
                                                     detect_min_area)
                else:
                    candidates = engine.detect_blobs(frame_hsv, colors, detect_min_area)
                engine.scale_candidates(candidates, used_scale)
                if args.center_mode != "contour":
                    for candidate in candidates:
                        contour = candidate.get("contour")
                        if contour is None or len(contour) < 5:
                            continue
                        if args.center_mode == "distance":
                            mask = np.zeros(roi.shape[:2], np.uint8)
                            cv2.drawContours(mask, [contour.astype(np.int32)], -1, 255, -1)
                            distance = cv2.distanceTransform(mask, cv2.DIST_L2, 5)
                            _, max_value, _, max_loc = cv2.minMaxLoc(distance)
                            if max_value > 1.0:
                                candidate["center_px"] = (float(max_loc[0]), float(max_loc[1]))
                                candidate["radius_px"] = float(max_value)
                            continue
                        (circle_x, circle_y), _ = cv2.minEnclosingCircle(contour)
                        if args.center_mode == "circle":
                            candidate["center_px"] = (float(circle_x), float(circle_y))
                        else:
                            old_x, old_y = candidate["center_px"]
                            candidate["center_px"] = ((old_x + circle_x) / 2.0,
                                                       (old_y + circle_y) / 2.0)
                if roi_top_px > 0:
                    for candidate in candidates:
                        u, v = candidate["center_px"]
                        candidate["center_px"] = (u, v + roi_top_px)
                found = engine.pick_by_class(candidates, current_cls)
                if color_lock_center is not None:
                    gate = 100.0
                    near = [item for item in candidates
                            if item.get("cls") == current_cls and
                            ((item["center_px"][0] - color_lock_center[0]) ** 2 +
                             (item["center_px"][1] - color_lock_center[1]) ** 2) ** 0.5 <= gate]
                    if near:
                        found = min(near, key=lambda item:
                                    (item["center_px"][0] - color_lock_center[0]) ** 2 +
                                    (item["center_px"][1] - color_lock_center[1]) ** 2)
                        color_lock_miss = 0
                    else:
                        color_lock_miss += 1
                        if color_lock_miss < 8:
                            found = None
                        else:
                            color_lock_center = None
                            color_lock_miss = 0
                if found is not None:
                    color_lock_center = found["center_px"]
                    color_lock_miss = 0
                if args.shape_fallback and found is None:
                    now_s = time.time()
                    if now_s - last_shape_run_s >= 1.0 / max(1.0, args.shape_hz):
                        small, shape_scale = engine.shrink_frame(roi, args.shape_scale)
                        inv = 1.0 / max(1e-6, shape_scale)
                        shapes = engine.detect_shape_targets(
                            small,
                            max(120, int(args.shape_min_area_px * shape_scale * shape_scale)),
                            max(400, int(args.shape_max_area_px * shape_scale * shape_scale)),
                            args.shape_min_circularity, args.shape_min_fill)
                        for item in shapes:
                            u, v = item["center_px"]
                            item["center_px"] = (u * inv, v * inv)
                            item["radius_px"] *= inv
                            item["area_px"] *= inv * inv
                        shape_cache = shapes
                        last_shape_run_s = now_s
                    shapes = shape_cache
                    if shapes:
                        roi_hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
                        for shape in shapes:
                            color_class, color_score = engine.classify_shape_color(
                                roi_hsv, shape, colors, args.shape_color_min_frac)
                            if color_class is not None and color_class != current_cls:
                                continue
                            shape["color_class"] = color_class
                            shape["color_score"] = color_score
                            shape["shape_only"] = True
                            u, v = shape["center_px"]
                            shape["center_px"] = (u, v + roi_top_px)
                            found = shape
                            break

            line = None
            x_mm = y_mm = 0.0
            if detect_enabled and found is not None:
                raw_u, raw_v = found["center_px"]
                if smooth_center is None or smooth_class != current_cls:
                    smooth_center = (raw_u, raw_v)
                else:
                    jump = ((raw_u - smooth_center[0]) ** 2 +
                            (raw_v - smooth_center[1]) ** 2) ** 0.5
                    if jump > args.smooth_jump_px:
                        smooth_center = (raw_u, raw_v)
                    else:
                        alpha = max(0.05, min(1.0, args.smooth_alpha))
                        smooth_center = (smooth_center[0] * (1.0 - alpha) + raw_u * alpha,
                                         smooth_center[1] * (1.0 - alpha) + raw_v * alpha)
                smooth_class = current_cls
                found["center_px"] = smooth_center
                u, v = found["center_px"]
                if current_cls == RING_CLASS:
                    # 顶部环小，用外径自动算；底部精对优先用实测固定值，
                    # 避免每帧检出半径抖动把误差放大。
                    auto_scale = (args.ring_outer_diameter_mm > 0.0 and
                                  found["radius_px"] > 1.0)
                    if ring_stage == 0:
                        if auto_scale:
                            px_per_mm = (found["radius_px"] /
                                         (args.ring_outer_diameter_mm / 2.0))
                        elif args.ring_px_per_mm_top > 0.0:
                            px_per_mm = args.ring_px_per_mm_top
                        elif args.ring_px_per_mm > 0.0:
                            px_per_mm = args.ring_px_per_mm
                        else:
                            px_per_mm = args.px_per_mm
                    else:
                        if args.ring_px_per_mm > 0.0:
                            px_per_mm = args.ring_px_per_mm
                        elif auto_scale:
                            px_per_mm = (found["radius_px"] /
                                         (args.ring_outer_diameter_mm / 2.0))
                        else:
                            px_per_mm = args.px_per_mm
                    rot_deg = ring_rot_deg
                else:
                    px_per_mm = args.px_per_mm
                    rot_deg = args.rot_deg
                x_mm, y_mm = engine.pixel_to_mm(u, v, gripper_px, px_per_mm, rot_deg)
                if current_cls == RING_CLASS:
                    x_mm += args.ring_offset_x_mm
                    y_mm += args.ring_offset_y_mm
                age_ms = int((time.time() - started_s) * 1000.0)
                line = format_line(seq, current_cls, x_mm, y_mm, 0, 900, age_ms)
                none_since_s = None
            elif detect_enabled:
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
                    try:
                        link.write(line)
                    except Exception:
                        pass
                now_print = time.time()
                if now_print - last_print_s >= 1.0:
                    last_print_s = now_print
                    state = "none" if found is None else f"x={x_mm:7.1f} y={y_mm:7.1f} mm"
                    print(f"{state}  TX {line.decode('ascii').strip()}")

            if preview:
                now_s = time.time()
                fps = 1.0 / max(1e-3, now_s - last_frame_s)
                last_frame_s = now_s
                if current_cls == RING_CLASS:
                    if found is not None:
                        for cx, cy, cr in found["circles"]:
                            cv2.circle(frame, (int(cx), int(cy)), int(cr), (0, 0, 255), 2)
                        cv2.drawMarker(frame, (int(found["center_px"][0]),
                                               int(found["center_px"][1])),
                                       (0, 0, 255), cv2.MARKER_CROSS, 26, 2)
                        digit_text = "?" if found.get("digit") is None else str(found["digit"])
                        cv2.putText(frame, f"ring {digit_text} "
                                           f"({found.get('digit_score', 0.0):.2f})",
                                    (8, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                                    (0, 255, 255), 2)
                else:
                    engine.draw_candidates(frame, candidates, found, args.show_all)
                    if found is not None and found.get("shape_only"):
                        cv2.drawMarker(frame, (int(found["center_px"][0]),
                                               int(found["center_px"][1])),
                                       (255, 0, 255), cv2.MARKER_CROSS, 30, 2)
                        cv2.putText(frame, "SHAPE fallback", (8, 130),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 0, 255), 2)
                cv2.drawMarker(frame, (int(gripper_px[0]), int(gripper_px[1])),
                               (0, 255, 0), cv2.MARKER_CROSS, 26, 2)
                shown_px_per_mm = (args.ring_px_per_mm if current_cls == RING_CLASS
                                   else args.px_per_mm)
                cv2.putText(frame, f"cls={current_cls} px/mm={shown_px_per_mm:.2f} "
                                   f"rot={args.rot_deg:.1f}  {fps:.0f} fps "
                                   f"detect={'ON' if detect_enabled else 'OFF'}",
                            (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2)
                if time.time() < last_scan_until:
                    cv2.putText(frame, last_scan_text, (8, 100),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 165, 255), 2)
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
        if qr_capture:
            qr_capture.release()
        if link:
            link.close()
        if preview:
            preview.close()
        if qr_preview:
            qr_preview.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
