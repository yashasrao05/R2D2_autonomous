from __future__ import annotations
import json, time, math
from types import SimpleNamespace
from enum import Enum
from pathlib import Path
import cv2, numpy as np
from core.waypoint_loader import MissionPlan
from control.checkpoint_manager import CheckpointManager
from control.servo7_actuator import Servo7Actuator
from perception.aruco_fiducial_engine import FiducialEngine
from perception.cameras import pixel_to_ground_offset
from perception.red_zone_detector import detect_red_zone
from perception.oakd_corridor_avoidance import annotate_scene

class Phase(str,Enum): CALIBRATE="CALIBRATE"; SCAN_REFERENCE="SCAN_REFERENCE"; GREEN="GREEN"; SEARCH="SEARCH"; RELEASE="RELEASE"; BLUE="BLUE"; LAND="LAND"; COMPLETE="COMPLETE"

class MarkerVault:
    def __init__(self,path: str|Path="storage/marker_vault.json"): self.path=Path(path); self.items=json.loads(self.path.read_text()) if self.path.exists() else {"reference":None,"detections":[]}
    def save(self): self.path.parent.mkdir(parents=True,exist_ok=True); self.path.write_text(json.dumps(self.items,indent=2))
    def reference_matrix(self):
        r=self.items.get("reference"); return None if not r or not r.get("matrix") else np.array(r["matrix"],dtype=np.uint8)
    def set_reference(self,detection,lat,lon):
        self.items["reference"]={"matrix":detection.matrix.tolist() if detection.matrix is not None else None,"aruco_id":detection.aruco_id,"qr":detection.qr_data,"lat":lat,"lon":lon,"created_at":time.time()}; self.save()
    def add(self,detection,lat,lon,score=None):
        self.items["detections"].append({"matrix":detection.matrix.tolist() if detection.matrix is not None else None,"aruco_id":detection.aruco_id,"qr":detection.qr_data,"lat":lat,"lon":lon,"score":score,"seen_at":time.time()}); self.save()

class MissionStateMachine:
    def __init__(self, vehicle, fence, watchdog, mission: MissionPlan, downcam, oakd, payload: Servo7Actuator, checkpoint: CheckpointManager, calibration_path="config/calibration_cache.json", hud=None, operator_guard=None):
        self.vehicle,self.fence,self.watchdog,self.plan,self.downcam,self.oakd,self.payload,self.checkpoint,self.hud,self.operator_guard=vehicle,fence,watchdog,mission,downcam,oakd,payload,checkpoint,hud,operator_guard
        self.calibration_path=Path(calibration_path); self.engine=FiducialEngine(); self.vault=MarkerVault(); self.phase=Phase.CALIBRATE
    def _checkpoint(self, phase: Phase, wp=None): self.phase=phase; self.checkpoint.save(phase.value,None if wp is None else wp.index,phase_altitude=self.vehicle.status.relative_alt_m); self.vehicle.log(f"Checkpoint {phase.value}")
    def _guard(self, modes: tuple[str, ...] | None = ("GUIDED",)):
        if self.operator_guard: self.operator_guard.assert_autonomy(modes)
    def _goto(self, waypoint, altitude=None, timeout_s=90.):
        self._guard()
        target_alt=waypoint.alt if altitude is None else altitude; self.fence.goto_gps(waypoint,target_alt)
        deadline=time.monotonic()+timeout_s
        while time.monotonic()<deadline:
            self._guard(); self.watchdog.tick(); s=self.vehicle.status
            horizontal=math.hypot((s.lat-waypoint.lat)*111_319.49,(s.lon-waypoint.lon)*111_319.49*math.cos(math.radians(waypoint.lat)))
            if horizontal<=1.5 and abs(s.relative_alt_m-target_alt)<=.6:
                self._checkpoint(self.phase,waypoint); return
            if self.vehicle.dry_run: break
            time.sleep(.1)
        if self.vehicle.dry_run: self._checkpoint(self.phase,waypoint); return
        raise TimeoutError(f"did not reach waypoint {waypoint.index} within {timeout_s:.0f}s")
    def preflight_calibrate(self):
        self.vehicle.poll(.5); s=self.vehicle.status
        if not (s.ekf_ok and s.satellites>=6 and s.hdop<1.4 and s.battery_v>20.3): raise RuntimeError(f"preflight failed: ekf={s.ekf_ok}, sats={s.satellites}, hdop={s.hdop}, V={s.battery_v}")
        frames=[]
        for _ in range(20): frames.append(self.downcam.read())
        hsv=cv2.cvtColor(np.concatenate([f.reshape(-1,3) for f in frames]).reshape(-1,1,3),cv2.COLOR_BGR2HSV)
        barometer=[]
        for _ in range(50): self.vehicle.poll(.01); barometer.append(self.vehicle.status.relative_alt_m)
        calibration={"barometer_zero_m":float(np.median(barometer)),"barometer_samples_m":barometer,"lighting":{"value_median":float(np.median(hsv[:,:,2])),"saturation_median":float(np.median(hsv[:,:,1]))},"ground_plane":self.oakd.calibrate_ground_plane(20)}
        self.calibration_path.parent.mkdir(parents=True,exist_ok=True); self.calibration_path.write_text(json.dumps(calibration,indent=2)); self.vehicle.log("Calibration complete"); self._checkpoint(Phase.SCAN_REFERENCE)
    def _collect_reference(self, frames=10):
        matrices=[]; chosen=None
        deadline=time.monotonic()+60
        while len(matrices)<frames and time.monotonic()<deadline:
            frame=self.downcam.read(); d=self.engine.detect(frame)
            if d and d.matrix is not None: matrices.append(d.matrix); chosen=d
        if not matrices: raise RuntimeError("no 6x6 reference marker observed")
        chosen.matrix=self.engine.majority(matrices); self.vault.set_reference(chosen,self.vehicle.status.lat,self.vehicle.status.lon); self.vehicle.log("Reference fiducial stored")
    def phase1(self):
        self._guard(None)
        cancel = (lambda: self._guard(None)) if self.operator_guard else None
        if not self.vehicle.set_mode_confirmed("GUIDED", cancel=cancel): raise RuntimeError("GUIDED mode rejected or cancelled")
        if not self.vehicle.arm_confirmed(True, cancel=cancel): raise RuntimeError("arming rejected or cancelled")
        if not self.vehicle.takeoff_confirmed(5.0, cancel=cancel): raise RuntimeError("takeoff rejected or cancelled")
        self._goto(self.plan.takeoff_scan_qr,5.0); self._collect_reference(); self._checkpoint(Phase.GREEN,self.plan.takeoff_scan_qr)
    def _corridor(self,color:str,start,end,obstacles:bool=False):
        self._goto(start,1.5); deadline=time.monotonic()+45
        while time.monotonic()<deadline:
            self._guard()
            scene=self.oakd.read(color); left,center,right=scene.sectors; vy=0 if scene.banner_offset is None else float(np.clip(scene.banner_offset/320,-.5,.5)); vx=1.0
            if obstacles and center<2.2: vx=.4; vy=.6 if right>left else -.6
            if min(left,right)<.8: vy=.7 if right>left else -.7
            self.fence.send_body_velocity(vx,vy,0); self.watchdog.tick()
            if self.hud: self.hud.update(None, annotate_scene(scene))
            if self.vehicle.status.relative_alt_m>2:
                # Clamp inside-corridor altitude through the same GPS fence gate as every other position command.
                current=SimpleNamespace(lat=self.vehicle.status.lat,lon=self.vehicle.status.lon,alt=2.0,index=-1)
                self.fence.goto_gps(current,2.0)
            if self.vehicle.dry_run: break
        self.fence.send_body_velocity(0,0,0); self._goto(end,1.5)
    def phase2(self): self._corridor("green",self.plan.green_corridor_start,self.plan.green_corridor_end); self._checkpoint(Phase.SEARCH,self.plan.green_corridor_end)
    def _is_target(self,d):
        ref=self.vault.reference_matrix(); ref_id=(self.vault.items.get("reference") or {}).get("aruco_id")
        if d.aruco_id is not None and ref_id is not None: return d.aruco_id==ref_id,0
        if d.matrix is None or ref is None: return False,None
        score=self.engine.best_hamming(d.matrix,ref); return score<=3,score
    def phase3(self):
        for wp in self.plan.delivery_search_waypoints:
            self._guard()
            self._goto(wp,10.0); frame=self.downcam.read(); red=detect_red_zone(frame)
            if red: self.fence.send_body_velocity(red.vector[1],red.vector[0],0); self.vehicle.log("Red zone avoidance")
            d=self.engine.detect(frame)
            if d:
                matched,score=self._is_target(d); self.vault.add(d,self.vehicle.status.lat,self.vehicle.status.lon,score)
                if matched: self.vehicle.log("TARGET MATCH"); self._checkpoint(Phase.RELEASE,wp); return d,frame
            if self.hud:self.hud.update(frame,None)
        raise RuntimeError("delivery search completed without target match")
    def phase4(self,detection,frame):
        deadline=time.monotonic()+45
        while time.monotonic()<deadline:
            self._guard()
            if detection.corners is None: raise RuntimeError("target lacks corners for visual servoing")
            centre=tuple(detection.corners.mean(axis=0)); north,east=pixel_to_ground_offset(centre,frame.shape,self.vehicle.status.relative_alt_m)
            if np.hypot(north,east)<.25: break
            self.fence.send_body_velocity(float(np.clip(north,-.7,.7)),float(np.clip(east,-.7,.7)),0)
            frame=self.downcam.read(); detection=self.engine.detect(frame)
            if detection is None: raise RuntimeError("lost target during visual servoing")
        else: raise RuntimeError("visual servoing did not converge within 45 seconds")
        self.fence.send_body_velocity(0,0,0)
        current=SimpleNamespace(lat=self.vehicle.status.lat,lon=self.vehicle.status.lon,alt=5.0,index=-1)
        self.fence.goto_gps(current,5.0); self.payload.release_cycle(guard=lambda: self._guard()); self._checkpoint(Phase.BLUE)
    def phase5(self): self._corridor("blue",self.plan.blue_corridor_start,self.plan.blue_corridor_end,True); self._checkpoint(Phase.LAND,self.plan.blue_corridor_end)
    def phase6(self):
        self._goto(self.plan.land_podium,3)
        cancel=(lambda: self._guard(("GUIDED","LAND"))) if self.operator_guard else None
        if not self.vehicle.set_mode_confirmed("LAND", cancel=cancel): raise RuntimeError("LAND mode rejected or cancelled")
        self._checkpoint(Phase.COMPLETE,self.plan.land_podium)
    def run(self,resume=False):
        if resume: return self.resume()
        if self.operator_guard: self.operator_guard.require_initial_manual()
        self.preflight_calibrate()
        if self.operator_guard: self.operator_guard.wait_for_autonomy_authorization()
        self.phase1(); self.phase2(); target,frame=self.phase3(); self.phase4(target,frame); self.phase5(); self.phase6()
    def resume(self):
        saved=self.checkpoint.load(); phase=Phase(saved["state"]); idx=saved.get("last_cleared_waypoint")
        if phase in (Phase.GREEN,Phase.BLUE): idx=self.plan.green_corridor_start.index if phase==Phase.GREEN else self.plan.blue_corridor_start.index
        if self.operator_guard: self.operator_guard.require_initial_manual(); self.operator_guard.wait_for_autonomy_authorization()
        cancel = (lambda: self._guard(None)) if self.operator_guard else None
        if not self.vehicle.set_mode_confirmed("GUIDED", cancel=cancel) or not self.vehicle.arm_confirmed(True, cancel=cancel) or not self.vehicle.takeoff_confirmed(10, cancel=cancel):
            raise RuntimeError("hot-resume launch rejected or cancelled")
        if idx is not None: self._goto(self.plan.all_waypoints[idx],10)
        if phase==Phase.GREEN: self.phase2(); phase=Phase.SEARCH
        if phase==Phase.SEARCH: target,frame=self.phase3(); self.phase4(target,frame); phase=Phase.BLUE
        if phase==Phase.BLUE: self.phase5(); phase=Phase.LAND
        if phase==Phase.LAND: self.phase6()
