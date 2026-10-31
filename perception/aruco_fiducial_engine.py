from __future__ import annotations
from dataclasses import dataclass
import cv2, numpy as np

@dataclass
class FiducialDetection:
    matrix: np.ndarray | None; aruco_id: int | None; qr_data: str | None; corners: np.ndarray | None; crop: np.ndarray | None

class FiducialEngine:
    def __init__(self):
        self.qr=cv2.QRCodeDetector(); self.dictionaries=[]
        for name in ("DICT_6X6_250","DICT_6X6_1000","DICT_4X4_250","DICT_APRILTAG_36h11"):
            if hasattr(cv2.aruco,name): self.dictionaries.append(cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco,name)))
    def detect(self, frame: np.ndarray) -> FiducialDetection | None:
        gray=cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
        for dictionary in self.dictionaries:
            detector=cv2.aruco.ArucoDetector(dictionary,cv2.aruco.DetectorParameters()) if hasattr(cv2.aruco,"ArucoDetector") else None
            corners,ids,_ = detector.detectMarkers(gray) if detector else cv2.aruco.detectMarkers(gray,dictionary)
            if ids is not None: return FiducialDetection(None,int(ids[0][0]),None,corners[0].reshape(4,2),None)
        value,points,_=self.qr.detectAndDecode(frame)
        if points is not None: return FiducialDetection(None,None,value or None,points.reshape(4,2),None)
        return self._grid(gray)
    def _grid(self, gray: np.ndarray) -> FiducialDetection | None:
        threshold=cv2.adaptiveThreshold(gray,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,cv2.THRESH_BINARY,31,5)
        contours,_=cv2.findContours(255-threshold,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)
        for contour in sorted(contours,key=cv2.contourArea,reverse=True):
            if cv2.contourArea(contour)<800: break
            quad=cv2.approxPolyDP(contour,.04*cv2.arcLength(contour,True),True)
            if len(quad)!=4 or not cv2.isContourConvex(quad): continue
            points=quad.reshape(4,2).astype(np.float32); sums=points.sum(1); diffs=np.diff(points,axis=1).ravel()
            ordered=np.array([points[np.argmin(sums)],points[np.argmin(diffs)],points[np.argmax(sums)],points[np.argmax(diffs)]],np.float32)
            warp=cv2.warpPerspective(gray,cv2.getPerspectiveTransform(ordered,np.array([[0,0],[79,0],[79,79],[0,79]],np.float32)),(80,80))
            _,binary=cv2.threshold(warp,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
            cells=binary.reshape(8,10,8,10).mean(axis=(1,3))>127
            border=np.r_[cells[0],cells[-1],cells[1:-1,0],cells[1:-1,-1]]
            if border.mean() < .25:
                return FiducialDetection(cells[1:7,1:7].astype(np.uint8),None,None,ordered,warp)
        return None
    @staticmethod
    def hamming(a: np.ndarray, b: np.ndarray) -> int: return int(np.count_nonzero(a != b))
    @classmethod
    def best_hamming(cls, candidate: np.ndarray, reference: np.ndarray) -> int: return min(cls.hamming(np.rot90(candidate,k),reference) for k in range(4))
    @staticmethod
    def majority(matrices: list[np.ndarray]) -> np.ndarray:
        if not matrices: raise ValueError("no matrices to vote")
        return (np.stack(matrices).mean(axis=0)>=.5).astype(np.uint8)
