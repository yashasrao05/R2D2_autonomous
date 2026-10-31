from __future__ import annotations

import hmac
import logging
import threading
import time
from typing import Callable

import cv2
import numpy as np
from flask import Flask, Response, jsonify, request


_DASHBOARD = """<!doctype html>
<html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>R2D2 Competition Console</title><style>
:root{color-scheme:dark;--bg:#071015;--panel:#101d24;--line:#29404b;--ok:#35d07f;--bad:#ff5d68;--warn:#ffc857;--text:#e8f0f3;--muted:#95aab4}
*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--text);font:15px system-ui,sans-serif}.wrap{max-width:1440px;margin:auto;padding:18px}
h1{font-size:20px;margin:0}.top{display:flex;gap:14px;align-items:center;justify-content:space-between;margin-bottom:14px}.badge{padding:7px 10px;border:1px solid var(--line);border-radius:8px;font-weight:700}
.grid{display:grid;grid-template-columns:minmax(520px,2fr) minmax(330px,1fr);gap:14px}.panel{background:var(--panel);border:1px solid var(--line);border-radius:10px;padding:13px}.video{width:100%;border-radius:7px;background:#000;min-height:300px}
.cards{display:grid;grid-template-columns:repeat(4,1fr);gap:8px;margin:10px 0}.card{padding:9px;background:#0b161c;border-radius:7px}.label{font-size:11px;color:var(--muted);text-transform:uppercase}.value{font-size:17px;font-weight:700;margin-top:3px}
button,input{background:#14252e;color:var(--text);border:1px solid var(--line);border-radius:6px;padding:9px}button{cursor:pointer;font-weight:700}button:hover{border-color:#6b96aa}.controls{display:flex;gap:8px;flex-wrap:wrap;margin-top:10px}.controls input{flex:1;min-width:180px}
.ok{color:var(--ok)}.bad{color:var(--bad)}.warn{color:var(--warn)}ul{padding-left:20px}.logs{height:260px;overflow:auto;background:#050b0e;padding:8px;border-radius:7px;font:12px ui-monospace,monospace}.log{margin:0 0 5px;color:#c8d7dd}.action{border-left:4px solid var(--warn);padding:9px;background:#182128;margin:10px 0}.small{font-size:12px;color:var(--muted)}
@media(max-width:900px){.grid{grid-template-columns:1fr}.cards{grid-template-columns:repeat(2,1fr)}}
</style></head><body><div class="wrap">
<div class="top"><div><h1>R2D2 Competition Console</h1><div class="small">Read-only telemetry; profile selection cannot arm the aircraft.</div></div><div id="state" class="badge">CONNECTING</div></div>
<div class="grid"><div><div class="panel"><img class="video" src="/stream" alt="Camera stream"><div class="cards">
<div class="card"><div class="label">Profile</div><div class="value" id="profile">-</div></div><div class="card"><div class="label">Mission phase</div><div class="value" id="phase">-</div></div>
<div class="card"><div class="label">Pixhawk</div><div class="value" id="pixhawk">-</div></div><div class="card"><div class="label">TX15 SC</div><div class="value" id="tx15">-</div></div>
<div class="card"><div class="label">Mode / armed</div><div class="value" id="mode">-</div></div><div class="card"><div class="label">Battery</div><div class="value" id="battery">-</div></div>
<div class="card"><div class="label">GPS</div><div class="value" id="gps">-</div></div><div class="card"><div class="label">EKF</div><div class="value" id="ekf">-</div></div></div>
<div class="action" id="action">Waiting for status...</div></div><div class="panel" style="margin-top:14px"><div class="label">Event log</div><div id="logs" class="logs"></div></div></div>
<div><div class="panel"><div class="label">Official run selection</div><p>Select only while disarmed with SC DOWN. The selection locks for the run; SC UP remains the only autonomy trigger.</p><div class="controls"><input id="token" type="password" placeholder="Field control token"><button onclick="pick('segment1')">Segment 1</button><button onclick="pick('segment2')">Segment 2</button></div><p id="pickResult" class="small"></p></div>
<div class="panel" style="margin-top:14px"><div class="label">Readiness</div><div id="ready" class="value">CHECKING</div><ul id="blockers"></ul><div class="small" id="logPath"></div></div>
<div class="panel" style="margin-top:14px"><div class="label">Verified checks</div><ul id="checks"></ul></div></div></div></div>
<script>
let seq=0;const token=document.getElementById('token');token.value=localStorage.getItem('r2d2-token')||'';token.onchange=()=>localStorage.setItem('r2d2-token',token.value);
const esc=s=>String(s??'').replace(/[&<>"']/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]));
async function pick(profile){localStorage.setItem('r2d2-token',token.value);let r=await fetch('/api/profile',{method:'POST',headers:{'Content-Type':'application/json','X-R2D2-Token':token.value},body:JSON.stringify({profile})});let j=await r.json();document.getElementById('pickResult').textContent=j.message||j.error;}
function set(id,v,cls=''){let e=document.getElementById(id);e.textContent=v;e.className=(id==='state'?'badge ':'value ')+cls;}
async function refresh(){try{let r=await fetch('/api/status',{cache:'no-store'}),s=await r.json(),t=s.telemetry,x=s.tx15;set('state',s.service_state,s.service_state==='MISSION_ACTIVE'?'warn':s.service_state==='FAULT'?'bad':'');set('profile',s.profile||'NOT SELECTED');set('phase',s.mission_phase||'-');set('pixhawk',t.connected?(t.transport+' LINK'):'OFFLINE',t.connected?'ok':'bad');set('tx15',(x.state||'UNKNOWN')+' '+(x.pwm??'')+(x.rc_fresh?'':' STALE'),x.state==='DOWN'&&x.rc_fresh?'ok':'warn');set('mode',t.mode+' / '+(t.armed?'ARMED':'DISARMED'),t.armed?'warn':'ok');set('battery',t.battery_v?t.battery_v.toFixed(2)+' V':'-');set('gps',t.satellites+' sats / HDOP '+t.hdop,(t.satellites>=6&&t.hdop<1.4)?'ok':'warn');set('ekf',t.ekf_ok?'HEALTHY':'NOT READY',t.ekf_ok?'ok':'bad');document.getElementById('action').textContent=s.operator_action;set('ready',s.readiness_passed?'PASS':'BLOCKED',s.readiness_passed?'ok':'bad');document.getElementById('blockers').innerHTML=(s.blockers||[]).map(b=>'<li class="bad">'+esc(b)+'</li>').join('')||'<li class="ok">No blockers</li>';document.getElementById('checks').innerHTML=(s.readiness_checks||[]).map(c=>'<li class="'+(c.status==='PASS'?'ok':c.status==='FAIL'?'bad':'warn')+'">'+esc(c.name)+': '+esc(c.status)+' - '+esc(c.detail)+'</li>').join('');document.getElementById('logPath').textContent='Jetson log: '+s.log_path;let lr=await fetch('/api/logs?after='+seq,{cache:'no-store'}),logs=await lr.json(),box=document.getElementById('logs');for(const e of logs.events){seq=Math.max(seq,e.sequence);let p=document.createElement('p');p.className='log';p.textContent=e.time.slice(11,19)+' ['+e.source+'] '+e.message;box.appendChild(p)}while(box.children.length>200)box.removeChild(box.firstChild);if(logs.events.length)box.scrollTop=box.scrollHeight;}catch(e){set('state','DASHBOARD ERROR','bad')}}
refresh();setInterval(refresh,1000);
</script></body></html>"""


class WifiHudStreamer:
    def __init__(self, port: int = 5000, runtime=None, control_token: str | None = None,
                 profile_selector: Callable[[str], tuple[bool, str]] | None = None):
        self.port = port
        self.runtime = runtime
        self.control_token = control_token or ""
        self.profile_selector = profile_selector
        self.app = Flask(__name__)
        self.lock = threading.Lock()
        self.left = None
        self.right = None
        self.app.add_url_rule("/", "dashboard", self.dashboard)
        self.app.add_url_rule("/stream", "stream", self.stream)
        self.app.add_url_rule("/healthz", "healthz", self.healthz)
        self.app.add_url_rule("/api/status", "status", self.status)
        self.app.add_url_rule("/api/logs", "logs", self.logs)
        self.app.add_url_rule("/api/profile", "profile", self.select_profile, methods=["POST"])

    def update(self, left: np.ndarray | None, right: np.ndarray | None) -> None:
        with self.lock:
            if left is not None:
                self.left = left.copy()
            if right is not None:
                self.right = right.copy()

    def _frame(self) -> bytes:
        with self.lock:
            left = None if self.left is None else self.left.copy()
            right = None if self.right is None else self.right.copy()
        if left is None:
            left = np.zeros((400, 640, 3), np.uint8)
            cv2.putText(left, "Awaiting downward camera", (60, 200), cv2.FONT_HERSHEY_SIMPLEX, .8, (255, 255, 255), 2)
        if right is None:
            right = np.zeros_like(left)
            cv2.putText(right, "Awaiting OAK-D", (180, 200), cv2.FONT_HERSHEY_SIMPLEX, .8, (255, 255, 255), 2)
        if left.shape[0] != right.shape[0]:
            right = cv2.resize(right, (int(right.shape[1] * left.shape[0] / right.shape[0]), left.shape[0]))
        ok, encoded = cv2.imencode(".jpg", np.hstack([left, right]), [cv2.IMWRITE_JPEG_QUALITY, 80])
        if not ok:
            raise RuntimeError("could not encode HUD frame")
        return encoded.tobytes()

    def dashboard(self):
        return Response(_DASHBOARD, mimetype="text/html")

    def healthz(self):
        return jsonify({"ok": True})

    def status(self):
        if self.runtime is None:
            return jsonify({"service_state": "VIDEO_ONLY", "telemetry": {}, "tx15": {}})
        return jsonify(self.runtime.snapshot())

    def logs(self):
        after = request.args.get("after", default=0, type=int)
        return jsonify({"events": [] if self.runtime is None else self.runtime.events(after)})

    def _authorized(self) -> bool:
        supplied = request.headers.get("X-R2D2-Token", "")
        return bool(self.control_token) and hmac.compare_digest(supplied, self.control_token)

    def select_profile(self):
        if not self._authorized():
            return jsonify({"error": "unauthorized", "message": "Valid field control token required."}), 401
        if self.profile_selector is None:
            return jsonify({"error": "read_only", "message": "Profile selection is not enabled."}), 503
        profile = str((request.get_json(silent=True) or {}).get("profile", ""))
        if profile not in {"segment1", "segment2"}:
            return jsonify({"error": "invalid_profile", "message": "Select segment1 or segment2."}), 400
        accepted, message = self.profile_selector(profile)
        return jsonify({"accepted": accepted, "message": message}), 202 if accepted else 409

    def stream(self):
        def generate():
            while True:
                started = time.monotonic()
                yield b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + self._frame() + b"\r\n"
                time.sleep(max(0.0, .1 - (time.monotonic() - started)))
        return Response(generate(), mimetype="multipart/x-mixed-replace; boundary=frame")

    def start(self) -> None:
        logging.getLogger("werkzeug").setLevel(logging.WARNING)
        thread = threading.Thread(
            target=lambda: self.app.run(host="0.0.0.0", port=self.port, threaded=True, use_reloader=False),
            daemon=True,
            name="r2d2-dashboard",
        )
        thread.start()
