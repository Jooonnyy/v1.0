# -*- coding: utf-8 -*-
"""二维码解码与扫码相机（独立于视觉相机）。"""

import time

from common import open_camera, parse_camera_source


def decode_qr(frame):
    """先 cv2，失败再 pyzbar；返回二维码原始字符串或 None。"""
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


def open_qr_camera(camera, width_px=640, height_px=480, fourcc="MJPG", fps=30):
    """打开扫码相机；失败返回 None。扫码用自动曝光；默认 MJPG 省 USB 带宽。"""
    return open_camera(parse_camera_source(camera), width_px=width_px,
                       height_px=height_px, lock_exposure=False,
                       fourcc=fourcc, fps=fps)


def scan_from_capture(capture, timeout_s):
    """在 timeout_s 内从已打开的相机里找一张能解的二维码。"""
    if capture is None:
        return None
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        ok, frame = capture.read()
        if not ok or frame is None:
            time.sleep(0.02)
            continue
        data = decode_qr(frame)
        if data:
            return data
    return None
