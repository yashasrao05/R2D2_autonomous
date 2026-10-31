from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Callable

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
    detections: list["SpatialDetection"]
    banners: list["BannerDetection"] | None = None
    depth_valid_fraction: float = 0.0
    white_obstacle_m: float | None = None


@dataclass(frozen=True)
class BannerDetection:
    bbox: tuple[int, int, int, int]
    offset_px: float
    location: str
    aspect_ratio: float


@dataclass(frozen=True)
class SpatialDetection:
    label: str
    confidence: float
    bbox: tuple[float, float, float, float]
    xyz_m: tuple[float, float, float]


def camera_to_body_frd(x_c: float, y_c: float, z_c: float,
                       tilt_deg: float = 45.0) -> tuple[float, float, float]:
    """Map OAK/OpenCV right-down-forward coordinates into body FRD.

    Camera coordinates are +X right, +Y down, +Z optical-forward.  A positive
    mount tilt points the optical axis down from body-forward.
    """
    angle = np.radians(tilt_deg)
    cosine, sine = float(np.cos(angle)), float(np.sin(angle))
    return cosine * z_c - sine * y_c, x_c, sine * z_c + cosine * y_c


def detect_corridor_banners(rgb: np.ndarray, color: str,
                            min_aspect_ratio: float = 3.5) -> list[BannerDetection]:
    """Find long, saturated entry/exit banners while rejecting blue sky."""
    hsv = cv2.cvtColor(rgb, cv2.COLOR_BGR2HSV)
    if color == "green":
        lower, upper = (45, 110, 45), (88, 255, 255)
    elif color == "blue":
        lower, upper = (95, 130, 45), (135, 255, 255)
    else:
        raise ValueError("banner color must be green or blue")
    mask = cv2.inRange(hsv, np.array(lower, np.uint8), np.array(upper, np.uint8))
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (9, 3))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    height, width = mask.shape
    minimum_area = max(500.0, width * height * 0.001)
    results: list[BannerDetection] = []
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for contour in contours:
        x, y, w, h = cv2.boundingRect(contour)
        aspect = w / max(h, 1)
        area = cv2.contourArea(contour)
        fill = area / max(float(w * h), 1.0)
        if color == "blue" and y <= 2 and h > height * .15:
            continue  # sky normally touches the top edge; the arch does not
        if area < minimum_area or aspect < min_aspect_ratio or fill < 0.45:
            continue
        centre_y = y + h / 2
        location = "top" if centre_y < height * .45 else "bottom" if centre_y > height * .55 else "middle"
        results.append(BannerDetection((x, y, w, h), x + w / 2 - width / 2, location, aspect))
    return sorted(results, key=lambda item: item.bbox[2] * item.bbox[3], reverse=True)


class OakDCamera:
    """OAK-D Lite RGB/stereo source using the DepthAI v3 API used by oak_pi_stream.py."""
    def __init__(self, tilt_deg: float = 45.0):
        self.tilt_deg = tilt_deg
        self.pipeline = None
        self.device = None
        self.rgb_q = None
        self.depth_q = None
        self.detection_q = None
        self.labels: list[str] = []
        self._detections: list[SpatialDetection] = []

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

        # Match the working oak_pi_stream.py deployment pipeline: run YOLOv6
        # on CAM_A and attach stereo depth for X/Y/Z object coordinates.
        model = dai.NNModelDescription("yolov6-nano")
        spatial_nn = self.pipeline.create(dai.node.SpatialDetectionNetwork).build(rgb_camera, stereo, model)
        spatial_nn.setConfidenceThreshold(0.45)
        spatial_nn.setBoundingBoxScaleFactor(0.5)
        spatial_nn.setDepthLowerThreshold(100)
        spatial_nn.setDepthUpperThreshold(10_000)
        self.labels = list(spatial_nn.getClasses() or [])

        # requestFullResolutionOutput() and createOutputQueue() are v3 APIs.
        self.rgb_q = spatial_nn.passthrough.createOutputQueue(maxSize=2, blocking=False)
        self.depth_q = stereo.depth.createOutputQueue(maxSize=2, blocking=False)
        self.detection_q = spatial_nn.out.createOutputQueue(maxSize=2, blocking=False)
        self.pipeline.start()

    @staticmethod
    def _wait_for_packet(queue, deadline: float, name: str, on_wait: Callable[[], None] | None):
        while time.monotonic() < deadline:
            packet = queue.tryGet()
            if packet is not None:
                return packet
            if on_wait:
                on_wait()
            time.sleep(.02)
        raise TimeoutError(f"OAK-D {name} frame timed out")

    def read(self, color: str = "green", floor_height_m: float = 0.25,
             timeout_s: float = 2.0, on_wait: Callable[[], None] | None = None) -> CorridorScene:
        if self.pipeline is None:
            self.open()

        deadline = time.monotonic() + timeout_s
        rgb_packet = self._wait_for_packet(self.rgb_q, deadline, "RGB", on_wait)
        depth_packet = self._wait_for_packet(self.depth_q, deadline, "depth", on_wait)
        rgb = rgb_packet.getCvFrame()
        depth_mm = depth_packet.getFrame().astype(np.float32)

        detection_packet = self.detection_q.tryGet()
        if detection_packet is not None:
            height, width = rgb.shape[:2]
            self._detections = []
            for item in detection_packet.detections:
                label = self.labels[item.label] if 0 <= item.label < len(self.labels) else str(item.label)
                coords = item.spatialCoordinates
                body_xyz = camera_to_body_frd(
                    coords.x / 1000.0, coords.y / 1000.0, coords.z / 1000.0, self.tilt_deg
                )
                self._detections.append(SpatialDetection(
                    label=label,
                    confidence=float(item.confidence),
                    bbox=(max(0.0, item.xmin), max(0.0, item.ymin), min(1.0, item.xmax), min(1.0, item.ymax)),
                    xyz_m=body_xyz,
                ))

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
        depth_valid_fraction = float(np.isfinite(roi).mean()) if roi.size else 0.0

        banners = detect_corridor_banners(rgb, color)
        if banners:
            weights = np.array([item.bbox[2] * item.bbox[3] for item in banners], dtype=float)
            banner_offset = float(np.average([item.offset_px for item in banners], weights=weights))
        else:
            banner_offset = None

        # Matte black walls are deliberately not treated as clear merely
        # because stereo returned invalid disparity.  For the blue chicane,
        # range only high-value, low-saturation standee-like regions.
        white_obstacle_m = None
        if color == "blue":
            hsv = cv2.cvtColor(rgb, cv2.COLOR_BGR2HSV)
            white = cv2.inRange(hsv, np.array((0, 0, 145), np.uint8), np.array((179, 90, 255), np.uint8))
            candidates, _ = cv2.findContours(white, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            ranges = []
            for contour in candidates:
                x, y, w, h = cv2.boundingRect(contour)
                if h < height * .18 or w < width * .04 or cv2.contourArea(contour) < width * height * .004:
                    continue
                sample = valid[y:y + h, x:x + w]
                if np.isfinite(sample).any():
                    ranges.append(float(np.nanpercentile(sample, 20)))
            if ranges:
                white_obstacle_m = min(ranges)
        return CorridorScene(rgb, depth, sectors, banner_offset, list(self._detections), banners,
                             depth_valid_fraction, white_obstacle_m)

    def calibrate_ground_plane(self, count: int = 20,
                               on_wait: Callable[[], None] | None = None) -> dict:
        frames = [self.read(on_wait=on_wait).depth for _ in range(count)]
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
        self.detection_q = None
        self._detections = []


def annotate_scene(scene: CorridorScene) -> np.ndarray:
    """Return OAK RGB with distance sectors, banner alignment, and YOLO boxes."""
    image = scene.rgb.copy()
    height, width = image.shape[:2]
    for x in (width // 3, 2 * width // 3):
        cv2.line(image, (x, 0), (x, height), (255, 255, 255), 1)
    for index, distance in enumerate(scene.sectors):
        cv2.putText(image, f"{'LCR'[index]} {distance:.2f}m", (12 + index * width // 3, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)
    banner = "none" if scene.banner_offset is None else f"{scene.banner_offset:+.0f}px"
    cv2.putText(image, f"Banner {banner}", (12, height - 14), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)
    for item in scene.detections:
        x1, y1, x2, y2 = item.bbox
        start, end = (int(x1 * width), int(y1 * height)), (int(x2 * width), int(y2 * height))
        cv2.rectangle(image, start, end, (0, 255, 0), 2)
        text = f"{item.label} {item.confidence * 100:.0f}% F:{item.xyz_m[0]:.2f} R:{item.xyz_m[1]:+.2f}m"
        cv2.putText(image, text, (start[0] + 4, max(18, start[1] - 6)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
    for banner_item in scene.banners or []:
        x, y, w, h = banner_item.bbox
        cv2.rectangle(image, (x, y), (x + w, y + h), (255, 255, 0), 2)
        cv2.putText(image, banner_item.location, (x, max(18, y - 5)), cv2.FONT_HERSHEY_SIMPLEX, .45, (255, 255, 0), 1)
    return image
