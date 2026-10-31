from __future__ import annotations
import json, time, math, os, tempfile
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
from safety.tx15_override import PilotOverride

class Phase(str,Enum):
    CALIBRATE="CALIBRATE"; SCAN_REFERENCE="SCAN_REFERENCE"; GREEN="GREEN"; SEARCH="SEARCH"; RELEASE="RELEASE"; BLUE="BLUE"; LAND="LAND"; COMPLETE="COMPLETE"
    HIGH_BYPASS_OUT="HIGH_BYPASS_OUT"; HIGH_BYPASS_RETURN="HIGH_BYPASS_RETURN"
    ENTRY_CORRIDOR="ENTRY_CORRIDOR"; RED_BYPASS="RED_BYPASS"; EXIT_CORRIDOR="EXIT_CORRIDOR"; RETURN_HOME="RETURN_HOME"

class MarkerVault:
    def __init__(self,path: str|Path="storage/marker_vault.json"):
        self.path=Path(path)
        self.items=json.loads(self.path.read_text()) if self.path.exists() else {"reference":None,"detections":[],"visited_decoys":[]}
        self.items.setdefault("visited_decoys", [])
    def save(self):
        self.path.parent.mkdir(parents=True,exist_ok=True)
        fd,temp=tempfile.mkstemp(prefix="marker-",suffix=".json",dir=self.path.parent)
        try:
            with os.fdopen(fd,"w",encoding="utf-8") as handle:
                json.dump(self.items,handle,indent=2); handle.flush(); os.fsync(handle.fileno())
            os.replace(temp,self.path)
        except BaseException:
            if os.path.exists(temp): os.unlink(temp)
            raise
    def reference_matrix(self):
        r=self.items.get("reference"); return None if not r or not r.get("matrix") else np.array(r["matrix"],dtype=np.uint8)
    def set_reference(self,detection,lat,lon):
        if self.items.get("reference") is not None: raise RuntimeError("reference marker is already latched")
        self.items["reference"]={"matrix":detection.matrix.tolist() if detection.matrix is not None else None,"aruco_id":detection.aruco_id,"qr":detection.qr_data,"lat":lat,"lon":lon,"created_at":time.time()}; self.save()
    def reset_for_new_mission(self): self.items={"reference":None,"detections":[],"visited_decoys":[]}; self.save()
    def add(self,detection,lat,lon,score=None,local_xy=None,matched=False):
        item={"matrix":detection.matrix.tolist() if detection.matrix is not None else None,"aruco_id":detection.aruco_id,"qr":detection.qr_data,"lat":lat,"lon":lon,"score":score,"local_xy":local_xy,"matched":matched,"seen_at":time.time()}
        self.items["detections"].append(item)
        if not matched: self.items["visited_decoys"].append(item)
        self.save()

class MissionStateMachine:
    PROFILE="full"
    requires_payload=True
    requires_delivery_search=True
    post_calibration_phase=Phase.SCAN_REFERENCE
    def __init__(self, vehicle, fence, watchdog, mission: MissionPlan, downcam, oakd, payload: Servo7Actuator, checkpoint: CheckpointManager, calibration_path="config/calibration_cache.json", hud=None, operator_guard=None,
                 corridor_distance_m: float = 7.5, delivery_altitude_m: float = 5.0,
                 delivery_altitude_tolerance_m: float = .3, delivery_center_tolerance_m: float = .15,
                 downcam_yaw_deg: float = 0.0, require_red_exclusion: bool = True,
                 minimum_search_waypoints: int = 5, status_callback=None):
        self.vehicle,self.fence,self.watchdog,self.plan,self.downcam,self.oakd,self.payload,self.checkpoint,self.hud,self.operator_guard=vehicle,fence,watchdog,mission,downcam,oakd,payload,checkpoint,hud,operator_guard
        self.calibration_path=Path(calibration_path); self.engine=FiducialEngine(); self.vault=MarkerVault(); self.phase=Phase.CALIBRATE
        self.corridor_distance_m=corridor_distance_m; self.delivery_altitude_m=delivery_altitude_m
        self.delivery_altitude_tolerance_m=delivery_altitude_tolerance_m; self.delivery_center_tolerance_m=delivery_center_tolerance_m
        self.downcam_yaw_deg=downcam_yaw_deg; self._last_velocity_refresh=0.0
        self.require_red_exclusion=bool(require_red_exclusion); self.minimum_search_waypoints=int(minimum_search_waypoints)
        self.status_callback=status_callback
    def _notify_status(self, state: str, detail: str | None = None):
        if self.status_callback:
            self.status_callback(state, self.phase.value, detail)
    def _checkpoint(self, phase: Phase, wp=None):
        self.phase=phase
        self.checkpoint.save(
            phase.value,None if wp is None else wp.index,
            phase_altitude=self.vehicle.status.relative_alt_m,mission_profile=self.PROFILE,
        )
        self.vehicle.log(f"Checkpoint {self.PROFILE}:{phase.value}")
        self._notify_status("COMPLETE" if phase == Phase.COMPLETE else "MISSION_ACTIVE")
    def _guard(self, modes: tuple[str, ...] | None = ("GUIDED",)):
        if self.operator_guard: self.operator_guard.assert_autonomy(modes)
    def _control_heartbeat(self):
        self._guard(); self.watchdog.tick()
        now=time.monotonic()
        if now-self._last_velocity_refresh>=.1:
            self.vehicle.resend_last_velocity(); self._last_velocity_refresh=now
    def _read_down(self): return self.downcam.read(on_wait=self._control_heartbeat)
    def _manual_fallback(self, reason: str) -> bool:
        if self.operator_guard:
            return self.operator_guard.yield_to_pilot(reason)
        if self.vehicle.status.mode == "ALT_HOLD":
            return True
        return self.vehicle.set_mode_confirmed("ALT_HOLD", 1.5) or self.vehicle.set_mode_confirmed("STABILIZE", 1.5)
    def _disarm_on_ground(self) -> bool:
        if not self.vehicle.status.armed:
            return True
        if self.vehicle.status.relative_alt_m >= .3:
            return False
        return self.vehicle.arm_confirmed(False, timeout_s=3.0)
    def _abort_launch(self, reason: str) -> None:
        self.vehicle.log(f"Launch aborted: {reason}", 2)
        if not self._disarm_on_ground() and self.vehicle.status.relative_alt_m < .3:
            self.vehicle.log("Ground disarm was not confirmed", 2)
        self._manual_fallback(reason)
    def _launch(self, altitude_m: float, climb_timeout_s: float = 15.0) -> None:
        """Run the tested GUIDED -> arm -> spool -> takeoff sequence."""
        self.vehicle.poll(.1); self._guard(None)
        if self.vehicle.status.armed and self.vehicle.status.relative_alt_m > .5:
            self._manual_fallback(f"launch requested while airborne at {self.vehicle.status.relative_alt_m:.1f}m")
            raise RuntimeError("refusing a ground launch while vehicle is already airborne")

        authorize = (lambda: self._guard(None)) if self.operator_guard else None
        stay_guided = (lambda: self._guard(("GUIDED",))) if self.operator_guard else None
        try:
            if not self.vehicle.set_mode_confirmed("GUIDED", cancel=authorize):
                self._abort_launch("GUIDED mode rejected or authorization removed")
                raise RuntimeError("GUIDED mode rejected or cancelled")
            self._guard(("GUIDED",))
            if not self.vehicle.arm_confirmed(True, cancel=stay_guided):
                self._abort_launch("arming rejected or authorization removed")
                raise RuntimeError("arming rejected or cancelled")

            spool_deadline = time.monotonic() + .5
            while time.monotonic() < spool_deadline:
                self._guard(("GUIDED",)); self.watchdog.tick(); time.sleep(.05)

            if not self.vehicle.takeoff_confirmed(altitude_m, cancel=stay_guided):
                self._abort_launch("takeoff command rejected or authorization removed")
                raise RuntimeError("takeoff rejected or cancelled")

            deadline = time.monotonic() + climb_timeout_s
            while time.monotonic() < deadline:
                self._guard(("GUIDED",)); self.watchdog.tick()
                if self.vehicle.status.relative_alt_m >= altitude_m - .2:
                    return
                self.vehicle.log(
                    f"Climb {self.vehicle.status.relative_alt_m:.1f}/{altitude_m:.1f}m",
                    throttle_s=2.0, key="launch-climb",
                )
                time.sleep(.05)
            self.vehicle.log("Climb timeout; requesting LAND", 2)
            self.vehicle.set_mode_confirmed("LAND", 3.0)
            raise TimeoutError(f"did not climb to {altitude_m:.1f}m within {climb_timeout_s:.0f}s")
        except PilotOverride:
            # assert_autonomy already commanded the tested fallback. If the
            # vehicle only just armed, also stop the motors while still grounded.
            if self.vehicle.status.relative_alt_m < .3:
                self._disarm_on_ground()
            raise
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
        self._notify_status("PREFLIGHT_CALIBRATION")
        self.vehicle.poll(.5); s=self.vehicle.status
        if not (s.ekf_ok and s.satellites>=6 and s.hdop<1.4 and s.battery_v>20.3): raise RuntimeError(f"preflight failed: ekf={s.ekf_ok}, sats={s.satellites}, hdop={s.hdop}, V={s.battery_v}")
        if self.require_red_exclusion and not self.fence.exclusions:
            raise RuntimeError("geofence has no red-zone exclusion polygon; refusing blind GPS transit")
        if self.requires_delivery_search and len(self.plan.delivery_search_waypoints)<self.minimum_search_waypoints:
            raise RuntimeError(
                f"delivery search has {len(self.plan.delivery_search_waypoints)} waypoints; "
                f"at least {self.minimum_search_waypoints} are required for field coverage"
            )
        self._validate_profile_plan()
        self.vehicle.verify_flight_failsafes()
        if self.requires_payload: self.payload.verify_configuration()
        frames=[]
        for _ in range(20): frames.append(self._read_down())
        hsv=cv2.cvtColor(np.concatenate([f.reshape(-1,3) for f in frames]).reshape(-1,1,3),cv2.COLOR_BGR2HSV)
        barometer=[]
        for _ in range(50): self.vehicle.poll(.01); barometer.append(self.vehicle.status.relative_alt_m)
        calibration={"barometer_zero_m":float(np.median(barometer)),"barometer_samples_m":barometer,"lighting":{"value_median":float(np.median(hsv[:,:,2])),"saturation_median":float(np.median(hsv[:,:,1]))},"ground_plane":self.oakd.calibrate_ground_plane(20,on_wait=self._control_heartbeat)}
        self.calibration_path.parent.mkdir(parents=True,exist_ok=True); self.calibration_path.write_text(json.dumps(calibration,indent=2)); self.vehicle.log("Calibration complete"); self._checkpoint(self.post_calibration_phase)
    def _validate_profile_plan(self):
        """Child profiles add route requirements without duplicating common preflight."""
        return None
    @staticmethod
    def _detection_key(d):
        if d.aruco_id is not None: return "aruco",int(d.aruco_id)
        if d.qr_data: return "qr",str(d.qr_data)
        if d.matrix is not None: return "matrix",np.asarray(d.matrix,dtype=np.uint8).tobytes().hex()
        return None
    def _collect_reference(self, frames=5):
        matrices=[]; chosen=None; run_key=None; consecutive=0
        deadline=time.monotonic()+60
        while consecutive<frames and time.monotonic()<deadline:
            self._control_heartbeat(); frame=self._read_down(); d=self.engine.detect(frame); key=self._detection_key(d) if d else None
            if key is None or key!=run_key:
                run_key=key; consecutive=1 if key is not None else 0; matrices=[]
            else: consecutive+=1
            if d and key is not None:
                chosen=d
                if d.matrix is not None: matrices.append(d.matrix)
        if chosen is None or consecutive<frames: raise RuntimeError(f"reference marker did not persist for {frames} consecutive frames")
        if matrices: chosen.matrix=self.engine.majority(matrices)
        self.vault.set_reference(chosen,self.vehicle.status.lat,self.vehicle.status.lon); self.vehicle.log(f"Reference fiducial stored after {frames}-frame vote")
    def phase1(self):
        self._launch(5.0)
        self._goto(self.plan.takeoff_scan_qr,5.0); self._collect_reference(); self._checkpoint(Phase.GREEN,self.plan.takeoff_scan_qr)
    def _corridor_distance(self,start_local,start_gps):
        s=self.vehicle.status; distances=[]
        if s.local_position_valid and time.monotonic()-s.last_local_position_monotonic<1.0:
            distances.append(math.hypot(s.local_north_m-start_local[0],s.local_east_m-start_local[1]))
        if s.lat and s.lon:
            distances.append(math.hypot((s.lat-start_gps[0])*111_319.49,(s.lon-start_gps[1])*111_319.49*math.cos(math.radians(start_gps[0]))))
        return max(distances) if distances else None
    def _corridor(self,color:str,start,end,obstacles:bool=False):
        self._goto(start,1.3); self.watchdog.tick(); s=self.vehicle.status
        if not s.local_position_valid or time.monotonic()-s.last_local_position_monotonic>=1.0:
            raise RuntimeError("fresh LOCAL_POSITION_NED is required before corridor entry")
        start_local=(s.local_north_m,s.local_east_m); start_gps=(s.lat,s.lon); deadline=time.monotonic()+30
        while time.monotonic()<deadline:
            self._control_heartbeat()
            scene=self.oakd.read(color, on_wait=self._control_heartbeat); left,center,right=scene.sectors; vy=0 if scene.banner_offset is None else float(np.clip(scene.banner_offset/320,-.5,.5)); vx=1.0
            obstacle_range=scene.white_obstacle_m if obstacles else None
            if obstacles and obstacle_range is not None and obstacle_range<2.2: vx=.4; vy=.6 if right>left else -.6
            if scene.depth_valid_fraction>.12 and min(left,right)<.8: vy=.7 if right>left else -.7
            self.fence.send_body_velocity(vx,vy,0); self.watchdog.tick()
            if self.hud: self.hud.update(None, annotate_scene(scene))
            if self.vehicle.status.relative_alt_m>2:
                # Clamp inside-corridor altitude through the same GPS fence gate as every other position command.
                current=SimpleNamespace(lat=self.vehicle.status.lat,lon=self.vehicle.status.lon,alt=2.0,index=-1)
                self.fence.goto_gps(current,2.0)
            distance=self._corridor_distance(start_local,start_gps)
            if distance is not None and distance>=self.corridor_distance_m: break
            if self.vehicle.dry_run and (distance is None or distance < .05):
                self.vehicle.status.local_north_m+=self.corridor_distance_m
                self.vehicle.status.last_local_position_monotonic=time.monotonic()
        else: raise TimeoutError(f"{color} corridor did not reach {self.corridor_distance_m:.1f}m EKF/GPS travel")
        self.fence.send_body_velocity(0,0,0); self._goto(end,1.3)
    def phase2(self): self._corridor("green",self.plan.green_corridor_start,self.plan.green_corridor_end); self._checkpoint(Phase.SEARCH,self.plan.green_corridor_end)
    def _is_target(self,d):
        reference=self.vault.items.get("reference") or {}; ref=self.vault.reference_matrix(); ref_id=reference.get("aruco_id"); ref_qr=reference.get("qr")
        if d.aruco_id is not None and ref_id is not None: return d.aruco_id==ref_id,0
        if d.qr_data and ref_qr: return d.qr_data==ref_qr,0
        if d.matrix is None or ref is None: return False,None
        score=self.engine.best_hamming(d.matrix,ref); return score<=3,score
    def _escape_red_zone(self,frame) -> bool:
        """Clear a visually detected red area; return whether an escape was needed."""
        red=detect_red_zone(frame)
        if red is None: return False
        clear_frames=0; escape_deadline=time.monotonic()+5
        while time.monotonic()<escape_deadline and clear_frames<3:
            # red.vector is image right/down away from the red centroid;
            # image down is body-backward for the nose-up nadir camera.
            self.fence.send_body_velocity(float(-red.vector[1]*.7),float(red.vector[0]*.7),0)
            self.vehicle.log("Red zone escape",4,throttle_s=1.0,key="red-zone-escape")
            frame=self._read_down(); red=detect_red_zone(frame)
            clear_frames=clear_frames+1 if red is None else 0
        self.fence.send_body_velocity(0,0,0)
        if red is not None: raise RuntimeError("could not clear visually detected red zone")
        return True
    def phase3(self):
        for wp in self.plan.delivery_search_waypoints:
            self._guard()
            self._goto(wp,10.0); observations=[]; frame=None
            for _ in range(5):
                frame=self._read_down(); d=self.engine.detect(frame)
                if d: observations.append((self._detection_key(d),d))
            if frame is None: continue
            if self._escape_red_zone(frame):
                continue
            d=None
            if observations:
                counts={key:sum(1 for observed,_ in observations if observed==key) for key,_ in observations}
                winner=max(counts,key=counts.get)
                if counts[winner]>=3: d=next(item for key,item in observations if key==winner)
            if d:
                matched,score=self._is_target(d); local=[self.vehicle.status.local_north_m,self.vehicle.status.local_east_m] if self.vehicle.status.local_position_valid else None
                self.vault.add(d,self.vehicle.status.lat,self.vehicle.status.lon,score,local,matched)
                if matched:
                    self._matched_waypoint=wp
                    self.vehicle.log("TARGET MATCH"); self._checkpoint(Phase.RELEASE,wp); return d,frame
            if self.hud:self.hud.update(frame,None)
        raise RuntimeError("delivery search completed without target match")
    def phase4(self,detection,frame,next_phase: Phase=Phase.BLUE):
        current=SimpleNamespace(lat=self.vehicle.status.lat,lon=self.vehicle.status.lon,alt=self.delivery_altitude_m,index=-1)
        self.fence.goto_gps(current,self.delivery_altitude_m); deadline=time.monotonic()+60; stable=0; last_seen=time.monotonic()
        while time.monotonic()<deadline:
            self._control_heartbeat()
            if detection.corners is None: raise RuntimeError("target lacks corners for visual servoing")
            centre=tuple(detection.corners.mean(axis=0)); forward,right=pixel_to_ground_offset(centre,frame.shape,max(self.vehicle.status.relative_alt_m,.1),camera_yaw_deg=self.downcam_yaw_deg)
            error=float(np.hypot(forward,right)); altitude_error=abs(self.vehicle.status.relative_alt_m-self.delivery_altitude_m)
            stable=stable+1 if error<self.delivery_center_tolerance_m and altitude_error<=self.delivery_altitude_tolerance_m else 0
            if stable>=5: break
            self.fence.send_body_velocity(float(np.clip(forward,-.7,.7)),float(np.clip(right,-.7,.7)),0)
            frame=self._read_down(); candidate=self.engine.detect(frame)
            if candidate is not None and self._is_target(candidate)[0]: detection=candidate; last_seen=time.monotonic()
            elif time.monotonic()-last_seen>.75: raise RuntimeError("lost matching target during delivery interlock")
        else: raise RuntimeError("delivery interlock did not achieve 5.0m +/-0.3m and <0.15m centering")
        self.fence.send_body_velocity(0,0,0)
        self.payload.release_cycle(guard=lambda: self._guard()); self._checkpoint(next_phase)
    def phase5(self): self._corridor("blue",self.plan.blue_corridor_start,self.plan.blue_corridor_end,True); self._checkpoint(Phase.LAND,self.plan.blue_corridor_end)
    def _land_at(self,waypoint):
        self._goto(waypoint,3)
        cancel=(lambda: self._guard(("GUIDED","LAND"))) if self.operator_guard else None
        if not self.vehicle.set_mode_confirmed("LAND", cancel=cancel): raise RuntimeError("LAND mode rejected or cancelled")
        if self.vehicle.dry_run:
            self.vehicle.status.armed = False
        deadline=time.monotonic()+45
        while time.monotonic()<deadline:
            self._guard(("LAND",)); self.watchdog.tick()
            if not self.vehicle.status.armed:
                self._checkpoint(Phase.COMPLETE,waypoint); self.vehicle.log(f"Mission {self.PROFILE} complete: touchdown/disarm"); return
            self.vehicle.log(
                f"Landing: {self.vehicle.status.relative_alt_m:.1f}m",
                throttle_s=2.0, key="landing-progress",
            )
            time.sleep(.1)
        self.vehicle.log("Landing timeout; Pixhawk remains in LAND", 2)
        raise TimeoutError("vehicle did not disarm within 45 seconds of LAND")
    def phase6(self): self._land_at(self.plan.land_podium)
    def _failsafe_land(self, reason: str) -> None:
        if not self.vehicle.status.armed:
            return
        self.vehicle.log(f"Mission fault: {reason}; requesting LAND", 2)
        try:
            self.vehicle.send_body_velocity(0,0,0)
        except Exception as exc:
            self.vehicle.log(f"Could not stop velocity setpoint: {exc}", 2)
        if self.vehicle.status.mode != "LAND" and not self.vehicle.set_mode_confirmed("LAND", 3.0):
            self.vehicle.log("Fault LAND mode was not confirmed", 2)
    def _mission_steps(self) -> None:
        self.phase1(); self.phase2(); target,frame=self.phase3(); self.phase4(target,frame); self.phase5(); self.phase6()
    def _run_active_mission(self) -> None:
        try:
            self._mission_steps()
        except PilotOverride:
            raise
        except BaseException as exc:
            self._failsafe_land(type(exc).__name__)
            raise
    def run(self,resume=False):
        if resume: return self.resume()
        self.vehicle.log(f"Mission profile selected: {self.PROFILE}")
        self.vault.reset_for_new_mission()
        if self.operator_guard:
            self._notify_status("WAITING_TX15_DOWN")
            self.operator_guard.require_initial_manual()
        self.preflight_calibrate()
        if self.operator_guard:
            self._notify_status("READY_WAITING_TX15_UP")
            self.operator_guard.wait_for_autonomy_authorization()
        self._notify_status("MISSION_ACTIVE")
        self._run_active_mission()
    def resume(self):
        saved=self.checkpoint.load(); phase=Phase(saved["state"]); idx=saved.get("last_cleared_waypoint")
        saved_profile=saved.get("mission_profile","full")
        if saved_profile!=self.PROFILE:
            raise RuntimeError(f"checkpoint belongs to {saved_profile}, not selected profile {self.PROFILE}")
        if phase in (Phase.GREEN,Phase.BLUE): idx=self.plan.green_corridor_start.index if phase==Phase.GREEN else self.plan.blue_corridor_start.index
        if self.operator_guard: self.operator_guard.require_initial_manual(); self.operator_guard.wait_for_autonomy_authorization()
        try:
            self._launch(10.0)
            if idx is not None: self._goto(self.plan.all_waypoints[idx],10)
            if phase==Phase.GREEN: self.phase2(); phase=Phase.SEARCH
            if phase==Phase.SEARCH: target,frame=self.phase3(); self.phase4(target,frame); phase=Phase.BLUE
            if phase==Phase.BLUE: self.phase5(); phase=Phase.LAND
            if phase==Phase.LAND: self.phase6()
        except PilotOverride:
            raise
        except BaseException as exc:
            self._failsafe_land(type(exc).__name__)
            raise
