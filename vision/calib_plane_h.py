#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""五点单应平面标定：只要一张已知边长的方格纸，不需要棋盘格、不需要内参。

为什么对准不需要内参：ALIGN 是闭环，每帧重新测误差。就算换算出来的毫米尺度差
20%，车也只会"每一步走多/走少一点"，照样能收敛到 0。所以这里只求
"像素 → 车体毫米"的一张平面映射。

摆放要求（很重要，方向搞反了车会朝着远离目标的方向走）：
    把方格纸放在目标所在的平面（转盘台面）上，**一条边与车头方向平行**。
    四个角按顺序点：
        ① 靠近车尾、车体右侧
        ② 靠近车头、车体右侧
        ③ 靠近车头、车体左侧
        ④ 靠近车尾、车体左侧
    即 ①→② 指向车头（+x），②→③ 指向车体左侧（+y）。

用法：
    # 现场点击：五次点击 + 回车确认（实时画面，点哪儿看哪儿）
    python3 calib_plane_h.py --square-mm 100
    python3 calib_plane_h.py --camera /dev/cam_target --square-mm 100

    # 无界面：直接给像素坐标（按同样顺序，最后一个是车体原点那个点）
    python3 calib_plane_h.py --image shot.png --square-mm 100 \
        --points 100 400  500 410  510 120  110 115  300 260

    # 标完自动开验证画面；也可以单独复核
    python3 calib_plane_h.py --check

判据：四角重投影残差 < 3 mm；两条边长相对误差 < 10%（超了说明点歪了或纸不平）。

关于第 5 个点：它是"误差归零"的那个位置，也就是**夹爪正下方（= 相机光轴正下方）**。
本车夹爪和相机在同一个 xy 上，所以这两个是同一个点。做法：把物料摆到
`vision_node.py` 的十字准星正中间，十字所在那块地就是原点。
物料最终会对到这附近，所以方格纸也摆在画面中心区域，误差最小。
"""

import argparse
import math
import sys

import numpy as np

from common import (DEFAULT_CAMERA, Preview, add_preview_argument,
                    close_all_windows, load_homography, load_json, open_camera,
                    parse_camera_source, pixel_to_plane_h, plane_h_to_pixel,
                    save_json)


def collect_points_camera(camera, count):
    """开窗点 count 个点。左键取点、退格撤销、回车确认、q 退出。"""
    import cv2

    capture = open_camera(parse_camera_source(camera), lock_exposure=False)
    if capture is None:
        print(f"打不开相机 {camera}（PC 上先试 0 / 1）")
        return None

    points = []
    preview = Preview("calib_plane_h")
    if not preview.enabled:
        capture.release()
        print("没有图形界面，画不出点选框；改用 --image shot.png --points ...")
        return None

    def on_mouse(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN and len(points) < count:
            points.append((float(x), float(y)))

    cv2.setMouseCallback("calib_plane_h", on_mouse)

    try:
        while True:
            ok, frame = capture.read()
            if not ok or frame is None:
                continue
            view = frame.copy()
            for index, (px, py) in enumerate(points):
                cv2.circle(view, (int(px), int(py)), 5, (0, 0, 255), -1)
                cv2.putText(view, str(index + 1), (int(px) + 8, int(py) - 8),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            hint = f"{len(points)}/{count}  left=click  bs=undo  enter=ok  q=quit"
            cv2.putText(view, hint, (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                        (0, 255, 0), 2)
            key = preview.show(view)
            if preview.quit:
                return None
            if key == 8 and points:
                points.pop()
            if key == 13 and len(points) == count:
                return points
    finally:
        capture.release()
        preview.close()
        close_all_windows()


def build_homography(points, square_mm):
    """四点 + 原点 → 像素到车体毫米的单应。返回 (H, rms_mm, edge_error)。"""
    import cv2

    if len(points) != 5:
        raise ValueError("需要 5 个点：4 个方格角 + 1 个车体原点")

    # 以①为原点的车体局部坐标：+x 朝车头（①→②），+y 朝车体左侧（②→③）
    size = float(square_mm)
    local = np.array([[0.0, 0.0],
                      [size, 0.0],
                      [size, size],
                      [0.0, size]], dtype=np.float32)

    pixels = np.array(points[:4], dtype=np.float32)
    homography_local = cv2.getPerspectiveTransform(pixels, local)

    # 第 5 个点（车体原点）在这个局部坐标系里的位置，然后整体平移过去
    origin = pixel_to_plane_h(points[4][0], points[4][1], homography_local)
    if origin is None:
        raise ValueError("原点那个点算不出坐标，检查点击顺序")
    shift = np.array([[1.0, 0.0, -origin[0]],
                      [0.0, 1.0, -origin[1]],
                      [0.0, 0.0, 1.0]], dtype=np.float64)
    homography = shift @ homography_local

    # 残差：四个角反投回毫米，和它该在的位置比
    errors = []
    for index in range(4):
        got = pixel_to_plane_h(points[index][0], points[index][1], homography)
        if got is None:
            errors.append(float("inf"))
            continue
        expect = local[index] - np.array(origin, dtype=np.float32)
        errors.append(math.hypot(got[0] - expect[0], got[1] - expect[1]))
    rms_mm = float(np.sqrt(np.mean(np.square(errors))))

    # 两条边长是否接近（方格是正方形，透视下长度会不同，但不该差太多）
    edge_top = math.hypot(pixels[1][0] - pixels[0][0], pixels[1][1] - pixels[0][1])
    edge_left = math.hypot(pixels[3][0] - pixels[0][0], pixels[3][1] - pixels[0][1])
    edge_error = abs(edge_top - edge_left) / max(1.0, max(edge_top, edge_left))

    return homography, rms_mm, edge_error


def run_check(args, homography, square_mm):
    """实时画面复核：画面上叠出反投的方格和原点，同时打印画面中心的毫米坐标。"""
    import cv2

    capture = open_camera(parse_camera_source(args.camera))
    if capture is None:
        print(f"打不开相机 {args.camera}（PC 上先试 0 / 1）")
        return 1

    preview = Preview("calib_plane_h check")
    print("复核画面：绿框 = 方格纸反投位置，红十字 = 车体原点；q / ESC 退出")
    try:
        while True:
            ok, frame = capture.read()
            if not ok or frame is None:
                continue

            # 方格四角与原点按单应反投回像素，直接叠在实时画面上对照。
            corners_px = []
            for x_mm, y_mm in ((0.0, 0.0), (square_mm, 0.0),
                               (square_mm, square_mm), (0.0, square_mm)):
                point = plane_h_to_pixel(x_mm, y_mm, homography)
                if point:
                    corners_px.append((int(point[0]), int(point[1])))
            if len(corners_px) == 4:
                cv2.polylines(frame, [np.array(corners_px, np.int32)], True,
                              (0, 255, 0), 2)
            origin_px = plane_h_to_pixel(0.0, 0.0, homography)
            if origin_px:
                cv2.drawMarker(frame, (int(origin_px[0]), int(origin_px[1])),
                               (0, 0, 255), cv2.MARKER_CROSS, 24, 2)

            height_px, width_px = frame.shape[:2]
            center_x, center_y = width_px / 2.0, height_px / 2.0
            point = pixel_to_plane_h(center_x, center_y, homography)
            if point:
                text = f"center -> x={point[0]:7.1f} mm  y={point[1]:7.1f} mm"
            else:
                text = "center -> 单应退化"
            cv2.drawMarker(frame, (int(center_x), int(center_y)), (255, 0, 0),
                           cv2.MARKER_CROSS, 24, 2)
            cv2.putText(frame, text, (8, 24), cv2.FONT_HERSHEY_SIMPLEX,
                        0.6, (0, 255, 0), 2)
            preview.show(frame)
            if preview.quit:
                return 0
            print(text, end="\r", flush=True)
            if not preview.enabled:
                return 0
    finally:
        capture.release()
        preview.close()
        close_all_windows()


def main(argv=None):
    parser = argparse.ArgumentParser(description="五点单应平面标定（不需要内参）")
    parser.add_argument("--camera", default=DEFAULT_CAMERA, help="相机序号或设备路径")
    parser.add_argument("--image", help="用现成图片，配合 --points")
    parser.add_argument("--points", nargs=10, type=float,
                        metavar=("X1", "Y1", "X2", "Y2", "X3", "Y3", "X4", "Y4", "XO", "YO"),
                        help="无界面时直接给 4 个角 + 1 个原点的像素坐标")
    parser.add_argument("--square-mm", type=float, default=100.0,
                        help="方格纸边长，单位 mm（打印后实际量一次）")
    parser.add_argument("--out", default="calib_plane_h.json")
    parser.add_argument("--check", action="store_true",
                        help="用已存在的标定文件实时核对，不重新标定")
    add_preview_argument(parser)
    args = parser.parse_args(argv)

    import importlib.util

    if importlib.util.find_spec("cv2") is None:
        print("缺少 opencv：pip3 install opencv-python numpy")
        return 2

    if args.check:
        homography = load_homography(args.out)
        if homography is None:
            print(f"还没有 {args.out}，先标定一次")
            return 1
        saved = load_json(args.out) or {}
        return run_check(args, homography, float(saved.get("square_mm", args.square_mm)))

    if args.points:
        points = [(args.points[i], args.points[i + 1]) for i in range(0, 10, 2)]
    elif args.image:
        print("--image 模式必须同时给 --points")
        return 2
    else:
        print("按顺序点五个点：")
        print("  1 车尾右侧角   2 车头右侧角   3 车头左侧角   4 车尾左侧角")
        print("  5 车体原点（相机光轴正下方那个位置）")
        print("左键取点，退格撤销，回车确认，q 退出。")
        points = collect_points_camera(args.camera, 5)
        if points is None:
            print("\n已取消")
            return 1

    try:
        homography, rms_mm, edge_error = build_homography(points, args.square_mm)
    except ValueError as error:
        print(error)
        return 1

    print(f"\n四角残差 RMS = {rms_mm:.2f} mm")
    print(f"两条边长相对差 = {edge_error * 100:.1f}%")
    if rms_mm > 3.0:
        print("警告：残差超过 3 mm，检查点得准不准、纸是否贴平")
    if edge_error > 0.10:
        print("警告：两条边长差超过 10%，点歪了或者透视太强")

    origin = pixel_to_plane_h(points[4][0], points[4][1], homography)
    if origin:
        print(f"原点那个点 -> x={origin[0]:.1f} mm, y={origin[1]:.1f} mm（应接近 0）")

    saved_path = save_json(args.out, {
        "homography": homography.tolist(),
        "square_mm": float(args.square_mm),
        "rms_mm": rms_mm,
        "edge_error": edge_error,
        "points_px": [[float(x), float(y)] for x, y in points],
        "note": "五点单应：像素→车体 mm。①车尾右 ②车头右 ③车头左 ④车尾左 ⑤夹爪正下方。"
                "只对一张平面成立，用于对准这类单平面任务。",
    })
    print(f"已写入 {saved_path}")
    print(f"用法：vision_node.py --homography {args.out} --port /dev/serial0 ...")

    # 标完立刻叠回实时画面，用卷尺对一下最直观。
    if args.preview and args.camera:
        run_check(args, homography, args.square_mm)
    return 0


if __name__ == "__main__":
    sys.exit(main())
