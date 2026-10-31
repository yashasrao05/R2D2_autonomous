from __future__ import annotations
from dataclasses import dataclass
import cv2, numpy as np
try: import depthai as dai
except ImportError: dai=None
@dataclass
class CorridorScene: rgb: np.ndarray; depth: np.ndarray; sectors: tuple[float,float,float]; banner_offset: float | None
class OakDCamera:
    def __init__(self, tilt_deg: float=45.): self.tilt_deg=tilt_deg; self.device=None; self.rgb_q=None; self.depth_q=None
    def open(self) -> None:
        if dai is None: raise RuntimeError("depthai is required")
        p=dai.Pipeline(); cam=p.createColorCamera(); cam.setPreviewSize(640,400); cam.setInterleaved(False)
        left=p.createMonoCamera(); right=p.createMonoCamera(); left.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P); right.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P); left.setBoardSocket(dai.CameraBoardSocket.LEFT); right.setBoardSocket(dai.CameraBoardSocket.RIGHT)
        stereo=p.createStereoDepth(); stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY); stereo.setSubpixel(True); stereo.setDepthAlign(dai.CameraBoardSocket.RGB)
        xo=p.createXLinkOut(); xo.setStreamName("rgb"); xd=p.createXLinkOut(); xd.setStreamName("depth"); cam.preview.link(xo.input); left.out.link(stereo.left); right.out.link(stereo.right); stereo.depth.link(xd.input)
        self.device=dai.Device(p); self.rgb_q=self.device.getOutputQueue("rgb",2,False); self.depth_q=self.device.getOutputQueue("depth",2,False)
    def read(self, color: str="green", floor_height_m: float=.25) -> CorridorScene:
        if self.device is None: self.open()
        rgb=self.rgb_q.get().getCvFrame(); depth=self.depth_q.get().getFrame().astype(np.float32)/1000
        # Near-ground pixels are stripped after plane calibration; unknown/invalid stays obstructed only at close range.
        valid=np.where((depth>floor_height_m)&np.isfinite(depth),depth,np.nan)
        h,w=valid.shape; roi=valid[int(.2*h):int(.8*h)]; thirds=np.array_split(roi,3,axis=1)
        sectors=tuple(float(np.nanpercentile(t,25)) if np.isfinite(t).any() else 99. for t in thirds)
        hsv=cv2.cvtColor(rgb,cv2.COLOR_BGR2HSV); lo,hi=((45,80,50),(85,255,255)) if color=="green" else ((100,100,50),(135,255,255)); mask=cv2.inRange(hsv,np.array(lo),np.array(hi)); m=cv2.moments(mask); offset=None if m["m00"]<500 else m["m10"]/m["m00"]-w/2
        return CorridorScene(rgb,depth,sectors,offset)
    def calibrate_ground_plane(self, count: int=20) -> dict:
        frames=[self.read().depth for _ in range(count)]; points=[]
        for d in frames:
            yy,xx=np.indices(d.shape); sel=np.isfinite(d)&(d>.2)&(d<20); points.append(np.c_[xx[sel],yy[sel],d[sel]])
        cloud=np.concatenate(points); best=None
        for _ in range(100):
            sample=cloud[np.random.choice(len(cloud),3,replace=False)]; normal=np.cross(sample[1]-sample[0],sample[2]-sample[0]); n=np.linalg.norm(normal)
            if n<1e-8: continue
            normal/=n; d=-normal@sample[0]; inliers=np.abs(cloud@normal+d)<.08
            if best is None or inliers.sum()>best[0].sum(): best=(inliers,normal,d)
        if best is None: raise RuntimeError("could not fit ground plane")
        _,normal,d=best; pitch=float(np.degrees(np.arctan2(normal[1],normal[2])))
        return {"normal":normal.tolist(),"offset":float(d),"pitch_offset_deg":pitch-self.tilt_deg,"floor_clearance_m":.25}
    def close(self) -> None:
        if self.device is not None: self.device.close(); self.device=None
