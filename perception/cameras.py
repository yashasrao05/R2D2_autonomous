from __future__ import annotations
import cv2, numpy as np

class DownwardCamera:
    def __init__(self, width: int=1280,height: int=720,fps: int=30): self.width,self.height,self.fps=width,height,fps; self.cap=None
    def open(self) -> None:
        pipeline=(f"nvarguscamerasrc ! video/x-raw(memory:NVMM), width={self.width}, height={self.height}, framerate={self.fps}/1, format=NV12 ! "
                  "nvvidconv ! video/x-raw, format=BGRx ! videoconvert ! video/x-raw, format=BGR ! appsink drop=true")
        self.cap=cv2.VideoCapture(pipeline,cv2.CAP_GSTREAMER)
        if not self.cap.isOpened(): self.cap=cv2.VideoCapture(0); self.cap.set(cv2.CAP_PROP_FRAME_WIDTH,self.width); self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT,self.height)
        if not self.cap.isOpened(): raise RuntimeError("downward camera unavailable (GStreamer and V4L2 both failed)")
    def read(self) -> np.ndarray:
        if self.cap is None: self.open()
        ok,frame=self.cap.read()
        if not ok: raise RuntimeError("failed to read downward camera")
        return frame
    def close(self) -> None:
        if self.cap is not None: self.cap.release(); self.cap=None

def pixel_to_ground_offset(pixel: tuple[float,float], shape: tuple[int,int] | tuple[int,int,int], altitude_m: float, hfov_deg: float=62.2, vfov_deg: float=48.8) -> tuple[float,float]:
    h,w=shape[:2]; x,y=pixel; # forward/north, right/east for a nadir camera
    east=(x-w/2)/(w/2)*altitude_m*np.tan(np.radians(hfov_deg/2)); north=(h/2-y)/(h/2)*altitude_m*np.tan(np.radians(vfov_deg/2))
    return north,east
