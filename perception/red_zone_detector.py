from __future__ import annotations
from dataclasses import dataclass
import cv2, numpy as np
@dataclass
class RedZone: centroid: tuple[float,float]; area: float; vector: tuple[float,float]
def detect_red_zone(frame: np.ndarray, min_area: float=500.) -> RedZone | None:
    hsv=cv2.cvtColor(frame,cv2.COLOR_BGR2HSV); mask=cv2.bitwise_or(cv2.inRange(hsv,(0,80,60),(10,255,255)),cv2.inRange(hsv,(170,80,60),(180,255,255)))
    contours,_=cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    if not contours: return None
    c=max(contours,key=cv2.contourArea); area=cv2.contourArea(c)
    if area<min_area: return None
    m=cv2.moments(c); x,y=m["m10"]/m["m00"],m["m01"]/m["m00"]; h,w=frame.shape[:2]
    dx,dy=w/2-x,h/2-y; norm=(dx*dx+dy*dy)**.5 or 1
    return RedZone((x,y),area,(dx/norm,dy/norm))
