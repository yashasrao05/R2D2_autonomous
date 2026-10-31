#!/usr/bin/env python3
from __future__ import annotations
import argparse, json, sys, time
from pathlib import Path
import yaml
from core.waypoint_loader import load_mission, load_waypoints, load_fence
from control.mavlink_interface import MavlinkVehicle
from control.checkpoint_manager import CheckpointManager
from control.servo7_actuator import Servo7Actuator
from safety.geofence_enforcer import GeofenceEnforcer, point_in_polygon
from safety.failsafe_watchdog import FailsafeWatchdog
from safety.tx15_override import PilotOverride, Tx15Config, Tx15SafetyGuard

def parser():
    p=argparse.ArgumentParser(description="SAEINDIA AeroTHON SkyScan companion")
    p.add_argument("--mode",required=True,choices=["calibrate","test-waypoints","test-aruco","test-corridor","test-redzone","test-servo7","test-geofence","test-tx15","mission"])
    p.add_argument("--wp-file",default="config/mission.waypoints"); p.add_argument("--fence-file",default="config/geofence.poly"); p.add_argument("--mapping",default="config/mission_mapping.yaml")
    p.add_argument("--dry-run",action="store_true"); p.add_argument("--resume",action="store_true"); p.add_argument("--reset",action="store_true"); p.add_argument("--save-ref",action="store_true"); p.add_argument("--color",choices=["green","blue"],default="green"); p.add_argument("--no-hud",action="store_true")
    p.add_argument("--tx15-live-fallback", action="store_true", help="Non-arming bench test: GUIDED on SC-up, then verify ALT_HOLD on SC-down.")
    p.add_argument("--tx15-timeout", type=float, default=0, help="TX15 test timeout in seconds; 0 waits indefinitely.")
    return p
def main():
    args=parser().parse_args(); cfg=yaml.safe_load(Path(args.mapping).read_text()) or {}; payload_cfg=cfg.get("payload",{})
    if args.mode=="test-waypoints":
        plan=load_mission(args.wp_file,args.mapping); print(json.dumps({"delivery_count":len(plan.delivery_search_waypoints),"roles":{k:getattr(plan,k).index for k in ("home","takeoff_scan_qr","green_corridor_start","green_corridor_end","blue_corridor_start","blue_corridor_end","land_podium")}},indent=2)); return
    if args.mode=="test-geofence":
        fence=load_fence(args.fence_file); print(json.dumps({"polygons":fence,"first_vertex_inside":point_in_polygon(fence[0][0],fence[0])},indent=2)); return
    vehicle=MavlinkVehicle(dry_run=args.dry_run); vehicle.connect()
    tx=cfg.get("tx15",{}); guard=Tx15SafetyGuard(vehicle,Tx15Config(channel=int(tx.get("rc_channel",8)),up_pwm=int(tx.get("sc_up_pwm",1700)),down_pwm=int(tx.get("sc_down_pwm",1300)),debounce_count=int(tx.get("debounce_count",3)),rc_timeout_s=float(tx.get("rc_timeout_s",2)),heartbeat_timeout_s=float(tx.get("heartbeat_timeout_s",3)),manual_mode=str(tx.get("manual_fallback_mode","ALT_HOLD")),backup_mode=str(tx.get("backup_fallback_mode","STABILIZE"))))
    if args.mode=="test-tx15":
        if args.dry_run:
            for _ in range(3): vehicle.status.rc_sequence += 1; guard.observe()
            assert guard.is_down()
            vehicle.status.rc_channels[guard.config.channel]=1800
            for _ in range(3): vehicle.status.rc_sequence += 1; guard.observe()
            guard.assert_autonomy(); print(f"TX15 Ch{guard.config.channel} debounce/manual override guard: OK"); return
        vehicle.request_message_intervals()
        timeout = None if args.tx15_timeout <= 0 else args.tx15_timeout
        print("TX15 test is NON-ARMING. Remove propellers before using --tx15-live-fallback.")
        guard.require_initial_manual(timeout)
        print(f"SC DOWN verified on channel {guard.config.channel}: {guard.pwm} PWM")
        if not args.tx15_live_fallback:
            print("Monitor-only mode. Move SC UP/DOWN and observe PWM/state; Ctrl+C exits. No flight-mode commands will be sent.")
            last = None
            while True:
                state = guard.poll(.1); current = (state, guard.pwm, guard.rc_fresh(), guard.heartbeat_fresh())
                if current != last: print(f"SC state={state}, pwm={guard.pwm}, rc_fresh={current[2]}, heartbeat_fresh={current[3]}"); last = current
                time.sleep(.05)
        guard.wait_for_autonomy_authorization(timeout)
        print("SC UP verified. Requesting GUIDED only; no arm or takeoff command will be sent.")
        if not vehicle.set_mode_confirmed("GUIDED", cancel=lambda: guard.assert_autonomy(None)):
            raise RuntimeError("GUIDED mode was rejected or SC was returned to manual")
        print("GUIDED confirmed. Flip SC DOWN now to verify manual fallback.")
        try:
            while True:
                guard.assert_autonomy(("GUIDED",)); time.sleep(.05)
        except PilotOverride as exc:
            if "TX15 manual override" not in str(exc):
                raise RuntimeError(f"test stopped before SC-down fallback: {exc}; Pixhawk mode is {vehicle.status.mode}") from exc
            if vehicle.status.mode not in {guard.config.manual_mode, guard.config.backup_mode}:
                raise RuntimeError(f"fallback did not confirm; current mode is {vehicle.status.mode}")
            print(f"TX15 fallback verified: Pixhawk is in {vehicle.status.mode}. No arm/takeoff command was sent.")
        return
    if args.mode=="test-servo7":
        Servo7Actuator(vehicle,**payload_cfg).release_cycle(); return
    if args.mode in {"test-aruco","test-redzone","test-corridor","calibrate","mission"}:
        from perception.cameras import DownwardCamera
        from perception.oakd_corridor_avoidance import OakDCamera
        from perception.aruco_fiducial_engine import FiducialEngine
        from perception.red_zone_detector import detect_red_zone
        down=DownwardCamera(); oak=OakDCamera(float(cfg.get("oakd_tilt_deg",45)))
        if args.mode=="test-aruco":
            frame=down.read(); d=FiducialEngine().detect(frame); print("none" if d is None else {"aruco_id":d.aruco_id,"qr":d.qr_data,"matrix":None if d.matrix is None else d.matrix.tolist()});
            if args.save_ref and d: from mission.state_machine import MarkerVault; MarkerVault().set_reference(d,vehicle.status.lat,vehicle.status.lon)
            return
        if args.mode=="test-redzone":
            print(detect_red_zone(down.read())); return
        if args.mode=="test-corridor":
            print(oak.read(args.color)); return
        from mission.state_machine import MissionStateMachine
        from streaming.wifi_hud_streamer import WifiHudStreamer
        plan=load_mission(args.wp_file,args.mapping); polygons=load_fence(args.fence_file); geofence=GeofenceEnforcer(vehicle,polygons,float(cfg.get("fence_margin_m",2)))
        geofence.upload(); hud=None
        if not args.no_hud:
            network=yaml.safe_load(Path("config/network.yaml").read_text()) if Path("config/network.yaml").exists() else {}
            stream_host=str(network.get("jetson_stream_host","10.233.44.10")); stream_port=int(network.get("stream_port",5000))
            hud=WifiHudStreamer(port=stream_port); hud.start(); vehicle.log(f"HUD http://{stream_host}:{stream_port}/stream")
        vehicle.request_message_intervals()
        machine=MissionStateMachine(vehicle,geofence,FailsafeWatchdog(vehicle),plan,down,oak,Servo7Actuator(vehicle,**payload_cfg),CheckpointManager(),hud=hud,operator_guard=guard)
        if args.reset: CheckpointManager().clear()
        if args.mode=="calibrate": machine.preflight_calibrate()
        else: machine.run(args.resume)
        return
    raise AssertionError("unreachable mode")
if __name__=="__main__":
    try: main()
    except KeyboardInterrupt: sys.exit(130)
    except Exception as exc: print(f"ERROR: {exc}",file=sys.stderr); sys.exit(2)
