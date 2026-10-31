#!/usr/bin/env python3
"""Camera bring-up and MJPEG stream test for the Jetson companion.

Run from the repository root, for example:
    python3 scripts/test_cameras.py --camera both --display
Then open http://<jetson-ip>:5000/stream on a laptop connected to the same network.
"""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

import cv2
import numpy as np
import yaml

from perception.cameras import DownwardCamera
from perception.aruco_fiducial_engine import FiducialEngine
from perception.oakd_corridor_avoidance import OakDCamera, annotate_scene
from streaming.wifi_hud_streamer import WifiHudStreamer


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Test Pi CSI/V4L2 and OAK-D cameras with an MJPEG HUD.")
    parser.add_argument("--camera", choices=("downward", "oakd", "both"), default="both")
    parser.add_argument("--port", type=int, default=None)
    parser.add_argument("--stream-host", default=None, help="Address printed for operators to open; the server still binds all interfaces.")
    parser.add_argument("--color", choices=("green", "blue"), default="green", help="Banner colour used for OAK-D overlay analysis.")
    parser.add_argument("--csi-sensor-id", type=int, default=0, help="Jetson CSI sensor id; CAM0 is normally 0.")
    parser.add_argument("--downward-backend", choices=("auto", "subprocess", "opencv", "v4l2"), default="auto")
    parser.add_argument("--v4l2-device", type=int, default=0, help="V4L2 fallback device number.")
    parser.add_argument("--no-aruco", action="store_true", help="Disable live ArUco/QR/custom-grid overlay on the Pi camera.")
    parser.add_argument("--display", action="store_true", help="Show a local OpenCV preview window (requires a graphical desktop).")
    parser.add_argument("--duration", type=float, default=0, help="Stop after this many seconds; 0 runs until Ctrl+C.")
    return parser.parse_args()


def annotate_downward(frame, detector: FiducialEngine | None = None):
    image = frame.copy()
    height, width = image.shape[:2]
    cv2.drawMarker(image, (width // 2, height // 2), (0, 255, 255), cv2.MARKER_CROSS, 38, 2)
    cv2.putText(image, "Downward Pi Camera", (16, 34), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
    if detector:
        detection = detector.detect(image)
        if detection:
            if detection.corners is not None:
                corners = detection.corners.astype(np.int32).reshape((-1, 1, 2))
                cv2.polylines(image, [corners], True, (0, 255, 0), 3)
            label = f"ArUco {detection.aruco_id}" if detection.aruco_id is not None else (f"QR {detection.qr_data}" if detection.qr_data else "6x6 grid")
            cv2.putText(image, label, (16, 66), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    return image


def annotate_oakd(scene):
    return annotate_scene(scene)


def main() -> None:
    args = parse_args()
    network = yaml.safe_load((ROOT / "config" / "network.yaml").read_text()) or {}
    port = args.port if args.port is not None else int(network.get("stream_port", 5000))
    stream_host = args.stream_host or str(network.get("jetson_stream_host", "10.233.44.10"))
    downward = DownwardCamera(sensor_id=args.csi_sensor_id, backend=args.downward_backend, v4l2_device=args.v4l2_device) if args.camera in ("downward", "both") else None
    oakd = OakDCamera() if args.camera in ("oakd", "both") else None
    fiducial_engine = None if args.no_aruco else FiducialEngine()
    hud = WifiHudStreamer(port=port)
    hud.start()
    print(f"MJPEG stream ready at http://{stream_host}:{port}/stream")
    print("Press Ctrl+C to stop.")
    started = time.monotonic()
    previous = started
    frames = 0
    try:
        while not args.duration or time.monotonic() - started < args.duration:
            left = annotate_downward(downward.read(), fiducial_engine) if downward else None
            if downward and frames == 0:
                print(f"Downward camera backend: {downward.backend_name}")
            right = annotate_oakd(oakd.read(args.color)) if oakd else None
            hud.update(left, right)
            frames += 1
            now = time.monotonic()
            if now - previous >= 1.0:
                print(f"Camera stream: {frames / (now - previous):.1f} FPS")
                previous, frames = now, 0
            if args.display:
                local_left = left if left is not None else cv2.cvtColor(right, cv2.COLOR_BGR2RGB)
                local_right = right if right is not None else local_left
                if local_left.shape[0] != local_right.shape[0]:
                    local_right = cv2.resize(local_right, (int(local_right.shape[1] * local_left.shape[0] / local_right.shape[0]), local_left.shape[0]))
                cv2.imshow("AeroTHON camera test", cv2.hconcat([local_left, local_right]))
                if cv2.waitKey(1) & 0xFF in (27, ord("q")):
                    break
    finally:
        if downward:
            downward.close()
        if oakd:
            oakd.close()
        if args.display:
            cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
