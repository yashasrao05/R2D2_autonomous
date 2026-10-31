from __future__ import annotations

import os
import select
import shutil
import subprocess
import time
from typing import Callable

import cv2
import numpy as np


class DownwardCamera:
    """Pi Camera v2 reader for Jetson CSI, independent of OpenCV GStreamer support.

    The default external-GStreamer backend is the tested path: `gst-launch-1.0`
    writes fixed-size BGR frames through `fdsink` to this process. OpenCV's
    GStreamer and V4L2 readers are retained as automatic fallbacks.
    """
    def __init__(self, width: int = 1280, height: int = 720, fps: int = 30, sensor_id: int = 0,
                 backend: str = "auto", v4l2_device: int = 0, read_timeout_s: float = 2.0):
        if backend not in {"auto", "subprocess", "opencv", "v4l2"}:
            raise ValueError("backend must be auto, subprocess, opencv, or v4l2")
        self.width, self.height, self.fps, self.sensor_id = width, height, fps, sensor_id
        self.backend_request, self.v4l2_device, self.read_timeout_s = backend, v4l2_device, read_timeout_s
        self.cap = None
        self.process: subprocess.Popen[bytes] | None = None
        self.backend_name: str | None = None
        self._candidate_index = 0
        self._last_errors: list[str] = []

    @property
    def frame_bytes(self) -> int:
        return self.width * self.height * 3

    def _candidates(self) -> tuple[str, ...]:
        return ("subprocess", "opencv", "v4l2") if self.backend_request == "auto" else (self.backend_request,)

    def _gst_pipeline(self) -> str:
        return (
            f"nvarguscamerasrc sensor-id={self.sensor_id} ! "
            f"video/x-raw(memory:NVMM),width=1280,height=720,framerate={self.fps}/1,format=NV12 ! "
            f"nvvidconv ! video/x-raw,width={self.width},height={self.height},format=BGRx ! "
            "videoconvert ! video/x-raw,format=BGR ! appsink drop=true sync=false"
            " max-buffers=1"
        )

    def _open_subprocess(self) -> None:
        executable = shutil.which("gst-launch-1.0")
        if not executable:
            raise RuntimeError("gst-launch-1.0 is not installed")
        command = [
            executable, "-q", "nvarguscamerasrc", f"sensor-id={self.sensor_id}", "!",
            f"video/x-raw(memory:NVMM),width=1280,height=720,framerate={self.fps}/1,format=NV12", "!",
            "nvvidconv", "!", f"video/x-raw,width={self.width},height={self.height},format=BGRx", "!",
            "videoconvert", "!", "video/x-raw,format=BGR", "!",
            "queue", "leaky=downstream", "max-size-buffers=1", "!", "fdsink", "sync=false",
        ]
        self.process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, bufsize=self.frame_bytes * 2)
        if self.process.stdout is None:
            raise RuntimeError("could not open CSI GStreamer stdout pipe")
        self.backend_name = "external-gstreamer"

    def _open_opencv(self) -> None:
        self.cap = cv2.VideoCapture(self._gst_pipeline(), cv2.CAP_GSTREAMER)
        if not self.cap.isOpened():
            self.cap.release(); self.cap = None
            raise RuntimeError("OpenCV GStreamer pipeline could not open")
        self.backend_name = "opencv-gstreamer"

    def _open_v4l2(self) -> None:
        self.cap = cv2.VideoCapture(self.v4l2_device, cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self.cap.set(cv2.CAP_PROP_FPS, self.fps)
        if not self.cap.isOpened():
            self.cap.release(); self.cap = None
            raise RuntimeError(f"V4L2 device /dev/video{self.v4l2_device} could not open")
        self.backend_name = "v4l2"

    def _release_current(self) -> None:
        if self.cap is not None:
            self.cap.release(); self.cap = None
        if self.process is not None:
            if self.process.poll() is None:
                self.process.terminate()
                try:
                    self.process.wait(timeout=1.0)
                except subprocess.TimeoutExpired:
                    self.process.kill(); self.process.wait(timeout=1.0)
            if self.process.stdout is not None:
                self.process.stdout.close()
            self.process = None
        self.backend_name = None

    def _activate_next_backend(self) -> None:
        candidates = self._candidates()
        while self._candidate_index < len(candidates):
            candidate = candidates[self._candidate_index]
            self._candidate_index += 1
            try:
                if candidate == "subprocess": self._open_subprocess()
                elif candidate == "opencv": self._open_opencv()
                else: self._open_v4l2()
                return
            except Exception as exc:
                self._last_errors.append(f"{candidate}: {exc}")
                self._release_current()
        details = "; ".join(self._last_errors) or "no backend attempted"
        raise RuntimeError(f"downward camera unavailable ({details})")

    def open(self) -> None:
        if self.backend_name is None:
            self._activate_next_backend()

    def _read_subprocess(self, on_wait: Callable[[], None] | None = None) -> np.ndarray:
        if self.process is None or self.process.stdout is None:
            raise RuntimeError("external GStreamer reader is not running")
        if self.process.poll() is not None:
            raise RuntimeError(f"external GStreamer exited with code {self.process.returncode}")
        data = bytearray()
        deadline = time.monotonic() + self.read_timeout_s
        fd = self.process.stdout.fileno()
        while len(data) < self.frame_bytes and time.monotonic() < deadline:
            ready, _, _ = select.select([fd], [], [], min(.05, max(0.0, deadline - time.monotonic())))
            if not ready:
                if on_wait:
                    on_wait()
                continue
            chunk = os.read(fd, self.frame_bytes - len(data))
            if not chunk:
                break
            data.extend(chunk)
        if len(data) != self.frame_bytes:
            raise RuntimeError(f"external GStreamer produced {len(data)}/{self.frame_bytes} frame bytes")
        return np.frombuffer(data, dtype=np.uint8).reshape((self.height, self.width, 3)).copy()

    def read(self, on_wait: Callable[[], None] | None = None) -> np.ndarray:
        self.open()
        while self.backend_name is not None:
            try:
                if self.backend_name == "external-gstreamer": return self._read_subprocess(on_wait)
                if self.cap is None: raise RuntimeError("OpenCV capture is not open")
                ok, frame = self.cap.read()
                if not ok or frame is None: raise RuntimeError("camera returned no frame")
                return frame
            except Exception as exc:
                self._last_errors.append(f"{self.backend_name}: {exc}")
                self._release_current()
                self._activate_next_backend()
        raise RuntimeError("downward camera failed without a fallback backend")

    def close(self) -> None:
        self._release_current()


def pixel_to_ground_offset(pixel: tuple[float, float], shape: tuple[int, int] | tuple[int, int, int], altitude_m: float,
                           hfov_deg: float = 62.2, vfov_deg: float = 48.8,
                           camera_yaw_deg: float = 0.0) -> tuple[float, float]:
    """Return body-forward/body-right ground error for a nadir camera.

    Image right maps to body right.  With a nose-up image and zero camera yaw,
    image top maps to body forward.  Positive yaw rotates the camera clockwise
    when viewed from below and is compensated here.
    """
    h, w = shape[:2]; x, y = pixel
    right_camera = (x - w / 2) / (w / 2) * altitude_m * np.tan(np.radians(hfov_deg / 2))
    forward_camera = (h / 2 - y) / (h / 2) * altitude_m * np.tan(np.radians(vfov_deg / 2))
    yaw = np.radians(camera_yaw_deg)
    forward_body = forward_camera * np.cos(yaw) - right_camera * np.sin(yaw)
    right_body = forward_camera * np.sin(yaw) + right_camera * np.cos(yaw)
    return float(forward_body), float(right_body)
