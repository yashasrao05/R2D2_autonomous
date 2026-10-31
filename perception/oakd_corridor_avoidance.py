from __future__ import annotations

from dataclasses import dataclass

import cv2
import numpy as np

try:
    import depthai as dai
except ImportError:
    dai = None


@dataclass
class CorridorScene:
    rgb: np.ndarray
    depth: np.ndarray
    sectors: tuple[float, float, float]
    banner_offset: float | None


class OakDCamera:
    """OAK-D Lite RGB/stereo source using the DepthAI v3 API used by oak_pi_stream.py."""
    def __init__(self, tilt_deg: float = 45.0):
        self.tilt_deg = tilt_deg
        self.pipeline = None
        self.device = None
        self.rgb_q = None
        self.depth_q = None

    def open(self) -> None:
        if dai is None:
            raise RuntimeError("depthai is required; install requirements.txt on the Jetson")

        # This is deliberately the same DepthAI v3 construction pattern as the
        # proven oak_pi_stream.py: Camera nodes are built directly on CAM_A/B/C.
        self.pipeline = dai.Pipeline()
        self.device = self.pipeline.getDefaultDevice()

        rgb_camera = self.pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
        mono_left = self.pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
        mono_right = self.pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C)

        stereo = self.pipeline.create(dai.node.StereoDepth)
        stereo.setLeftRightCheck(True)
        stereo.setExtendedDisparity(True)

        mono_left.requestFullResolutionOutput().link(stereo.left)
        mono_right.requestFullResolutionOutput().link(stereo.right)

        # requestFullResolutionOutput() and createOutputQueue() are v3 APIs.
        self.rgb_q = rgb_camera.requestFullResolutionOutput().createOutputQueue(maxSize=2, blocking=False)
        self.depth_q = stereo.depth.createOutputQueue(maxSize=2, blocking=False)
        self.pipeline.start()

    def read(self, color: str = "green", floor_height_m: float = 0.25) -> CorridorScene:
        if self.pipeline is None:
            self.open()

        rgb_packet = self.rgb_q.get()
        depth_packet = self.depth_q.get()
        rgb = rgb_packet.getCvFrame()
        depth_mm = depth_packet.getFrame().astype(np.float32)

        # StereoDepth.depth is millimetres. Resize to RGB dimensions so the
        # corridor regions and visual overlay use one common frame geometry.
        if depth_mm.shape[:2] != rgb.shape[:2]:
            depth_mm = cv2.resize(depth_mm, (rgb.shape[1], rgb.shape[0]), interpolation=cv2.INTER_NEAREST)
        depth = depth_mm / 1000.0

        # Invalid pixels and grass-level clearance are excluded before sector
        # percentiles are measured. Values remain metres for flight control.
        valid = np.where((depth > floor_height_m) & np.isfinite(depth), depth, np.nan)
        height, width = valid.shape
        roi = valid[int(0.20 * height):int(0.80 * height)]
        thirds = np.array_split(roi, 3, axis=1)
        sectors = tuple(float(np.nanpercentile(item, 25)) if np.isfinite(item).any() else 99.0 for item in thirds)

        hsv = cv2.cvtColor(rgb, cv2.COLOR_BGR2HSV)
        lower, upper = ((45, 80, 50), (85, 255, 255)) if color == "green" else ((100, 100, 50), (135, 255, 255))
        mask = cv2.inRange(hsv, np.array(lower), np.array(upper))
        moments = cv2.moments(mask)
        banner_offset = None if moments["m00"] < 500 else moments["m10"] / moments["m00"] - width / 2
        return CorridorScene(rgb, depth, sectors, banner_offset)

    def calibrate_ground_plane(self, count: int = 20) -> dict:
        frames = [self.read().depth for _ in range(count)]
        points: list[np.ndarray] = []
        for depth in frames:
            yy, xx = np.indices(depth.shape)
            selected = np.isfinite(depth) & (depth > 0.2) & (depth < 20)
            points.append(np.c_[xx[selected], yy[selected], depth[selected]])
        cloud = np.concatenate(points)
        if len(cloud) < 3:
            raise RuntimeError("not enough valid OAK-D depth points for a ground plane")

        best: tuple[np.ndarray, np.ndarray, float] | None = None
        for _ in range(100):
            sample = cloud[np.random.choice(len(cloud), 3, replace=False)]
            normal = np.cross(sample[1] - sample[0], sample[2] - sample[0])
            magnitude = np.linalg.norm(normal)
            if magnitude < 1e-8:
                continue
            normal /= magnitude
            offset = -normal @ sample[0]
            inliers = np.abs(cloud @ normal + offset) < 0.08
            if best is None or inliers.sum() > best[0].sum():
                best = (inliers, normal, float(offset))
        if best is None:
            raise RuntimeError("could not fit OAK-D ground plane")
        _, normal, offset = best
        pitch = float(np.degrees(np.arctan2(normal[1], normal[2])))
        return {"normal": normal.tolist(), "offset": offset, "pitch_offset_deg": pitch - self.tilt_deg, "floor_clearance_m": 0.25}

    def close(self) -> None:
        if self.pipeline is not None:
            stop = getattr(self.pipeline, "stop", None)
            if callable(stop):
                stop()
            elif self.device is not None:
                close = getattr(self.device, "close", None)
                if callable(close):
                    close()
        self.pipeline = None
        self.device = None
        self.rgb_q = None
        self.depth_q = None
