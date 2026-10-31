from __future__ import annotations
import threading, time, cv2, numpy as np
from flask import Flask, Response
class WifiHudStreamer:
    def __init__(self, port: int=5000): self.port=port; self.app=Flask(__name__); self.lock=threading.Lock(); self.left=None; self.right=None; self.app.add_url_rule('/stream','stream',self.stream)
    def update(self,left:np.ndarray|None,right:np.ndarray|None) -> None:
        with self.lock: self.left=None if left is None else left.copy(); self.right=None if right is None else right.copy()
    def _frame(self) -> bytes:
        with self.lock: left,right=self.left,self.right
        if left is None: left=np.zeros((400,640,3),np.uint8); cv2.putText(left,"Awaiting downward camera",(60,200),cv2.FONT_HERSHEY_SIMPLEX,.8,(255,255,255),2)
        if right is None: right=np.zeros_like(left); cv2.putText(right,"Awaiting OAK-D",(180,200),cv2.FONT_HERSHEY_SIMPLEX,.8,(255,255,255),2)
        if left.shape[0]!=right.shape[0]: right=cv2.resize(right,(int(right.shape[1]*left.shape[0]/right.shape[0]),left.shape[0]))
        return cv2.imencode('.jpg',np.hstack([left,right]),[cv2.IMWRITE_JPEG_QUALITY,80])[1].tobytes()
    def stream(self):
        def generate():
            while True:
                started=time.monotonic(); yield b'--frame\r\nContent-Type: image/jpeg\r\n\r\n'+self._frame()+b'\r\n'
                time.sleep(max(0.0, .1-(time.monotonic()-started)))
        return Response(generate(),mimetype='multipart/x-mixed-replace; boundary=frame')
    def start(self) -> None: threading.Thread(target=lambda:self.app.run(host='0.0.0.0',port=self.port,threaded=True,use_reloader=False),daemon=True).start()
