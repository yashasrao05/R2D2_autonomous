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
    p.add_argument("--mode",required=True,choices=["service","competition-check","configure-servo7","sync-flightplan","flight-check","calibrate","sync-pixhawk","test-lifecycle","test-waypoints","test-aruco","test-corridor","test-redzone","test-servo7","test-geofence","test-tx15","mission"])
    p.add_argument("--wp-file",default="config/mission.waypoints"); p.add_argument("--fence-file",default="config/geofence.poly"); p.add_argument("--mapping",default="config/mission_mapping.yaml")
    p.add_argument("--coordinates", default="config/field_coordinates.yaml", help="Normalized field-coordinate YAML used by the lifecycle audit.")
    p.add_argument("--mission-profile", choices=["full","segment1","segment2"], default=None,
                   help="Mission sequence: original parent, official Run 1, or official Run 2.")
    p.add_argument("--lifecycle-seconds", type=float, default=5.0, help="Low-rate live telemetry sample duration (1-30 seconds).")
    p.add_argument("--transport", choices=["auto","ethernet","uart"], default=None, help="MAVLink transport; mapping defaults to Ethernet-only.")
    p.add_argument("--ethernet-endpoint", default=None, help="pymavlink UDP endpoint override.")
    p.add_argument("--uart-device", default=None, help="Jetson UART device override.")
    p.add_argument("--uart-baud", type=int, default=None, help="Jetson UART baud override.")
    p.add_argument("--dry-run",action="store_true"); p.add_argument("--resume",action="store_true"); p.add_argument("--reset",action="store_true"); p.add_argument("--save-ref",action="store_true"); p.add_argument("--color",choices=["green","blue"],default="green"); p.add_argument("--no-hud",action="store_true")
    p.add_argument("--tx15-live-fallback", action="store_true", help="Non-arming bench test: GUIDED on SC-up, then verify ALT_HOLD on SC-down.")
    p.add_argument("--tx15-timeout", type=float, default=0, help="TX15 test timeout in seconds; 0 waits indefinitely.")
    p.add_argument("--tx15-monitor-seconds", type=float, default=0, help="Exit the non-arming TX15 monitor after this many seconds; 0 runs until Ctrl+C.")
    p.add_argument("--servo7-pwm", type=int, default=2000, help="Servo 7 bench-test PWM (default: 2000).")
    p.add_argument("--servo7-seconds", type=float, default=65.0, help="Servo 7 bench-test pulse duration (default: 65 seconds).")
    p.add_argument("--servo7-neutral-pwm", type=int, default=None, help="Optional fixed neutral PWM; default captures the current RC12 input.")
    p.add_argument("--upload-manifest", default="config/fc_upload_manifest.yaml", help="Validated FC parameter/fence/mission upload bundle.")
    p.add_argument("--checkout-level", choices=["hover","hop"], default="hover", help="Live checkout: hover only, or hover plus one safe waypoint hop/return.")
    p.add_argument("--takeoff-altitude", type=float, default=2.0, help="Live checkout altitude in metres (1-5).")
    p.add_argument("--hover-seconds", type=float, default=10.0, help="Live checkout hover duration (3-30 seconds).")
    p.add_argument("--confirm-live-flight", action="store_true", help="Required acknowledgement for the arming flight-check mode.")
    return p
def main():
    args=parser().parse_args(); cfg=yaml.safe_load(Path(args.mapping).read_text()) or {}; payload_cfg=cfg.get("payload",{})
    profile_cfg=cfg.get("mission_profiles",{}); mission_profile=args.mission_profile or str(profile_cfg.get("default","full"))
    if args.mode=="service":
        from mission.supervisor import run_competition_service
        run_competition_service(args,cfg); return
    if args.mode=="test-waypoints":
        plan=load_mission(args.wp_file,args.mapping); print(json.dumps({"delivery_count":len(plan.delivery_search_waypoints),"roles":{k:getattr(plan,k).index for k in ("home","takeoff_scan_qr","green_corridor_start","green_corridor_end","blue_corridor_start","blue_corridor_end","land_podium")}},indent=2)); return
    if args.mode=="test-geofence":
        fence=load_fence(args.fence_file); print(json.dumps({"polygons":fence,"first_vertex_inside":point_in_polygon(fence[0][0],fence[0])},indent=2)); return
    mav_cfg=cfg.get("mavlink",{})
    vehicle=MavlinkVehicle(
        endpoint=args.ethernet_endpoint or str(mav_cfg.get("ethernet_endpoint","udpin:0.0.0.0:14550")),
        uart=args.uart_device or str(mav_cfg.get("uart_device","/dev/ttyTHS1")),
        baud=args.uart_baud or int(mav_cfg.get("uart_baud",115200)),
        transport=args.transport or str(mav_cfg.get("transport","ethernet")),
        dry_run=args.dry_run,
    ); vehicle.connect()
    if args.mode=="sync-flightplan":
        from safety.fc_flightplan_sync import sync_flightplan_to_fc
        print("FC flight-plan sync is NON-ARMING. Keep the aircraft disarmed at the configured Starting Box.")
        report=sync_flightplan_to_fc(vehicle,args.upload_manifest)
        print(json.dumps(report,indent=2)); return
    if args.mode=="flight-check":
        if not args.dry_run and not args.confirm_live_flight:
            raise RuntimeError("flight-check can ARM and TAKE OFF; repeat with --confirm-live-flight after clearing the aircraft")
        from safety.flight_checkout import run_flight_checkout
        tx=cfg.get("tx15",{}); checkout_guard=Tx15SafetyGuard(vehicle,Tx15Config(channel=int(tx.get("rc_channel",8)),up_pwm=int(tx.get("sc_up_pwm",1700)),down_pwm=int(tx.get("sc_down_pwm",1300)),debounce_count=int(tx.get("debounce_count",3)),rc_timeout_s=float(tx.get("rc_timeout_s",2)),heartbeat_timeout_s=float(tx.get("heartbeat_timeout_s",3)),manual_mode=str(tx.get("manual_fallback_mode","ALT_HOLD")),backup_mode=str(tx.get("backup_fallback_mode","STABILIZE"))))
        print("LIVE FLIGHT CHECK: this mode can arm, take off, hover, and land. Use an approved test area.")
        fc_managed=str((cfg.get("flight_controller") or {}).get("field_source","jetson_files"))=="mission_planner"
        report=run_flight_checkout(
            vehicle,checkout_guard,args.upload_manifest,args.checkout_level,args.takeoff_altitude,args.hover_seconds,
            fc_managed=fc_managed,waypoint_path=args.wp_file,mapping_path=args.mapping,
            fence_margin_m=float(cfg.get("fence_margin_m",2.0)),
        )
        print(json.dumps(report,indent=2)); return
    if args.mode=="competition-check":
        from safety.competition_readiness import run_competition_readiness
        print("Competition check is NON-ARMING. It does not move motors or Servo 7.")
        report=run_competition_readiness(
            vehicle,mav_cfg,args.coordinates,args.wp_file,args.mapping,args.fence_file,
            float(cfg.get("fence_margin_m",2.0)),args.lifecycle_seconds,
        )
        print(json.dumps(report,indent=2))
        if not report["passed"]: raise SystemExit(2)
        return
    if args.mode=="test-lifecycle":
        from safety.lifecycle_audit import run_lifecycle_audit
        print("Lifecycle audit is NON-ARMING and does not move Servo 7.")
        print("Place the RC12 winch control at its actual neutral position before continuing.")
        fc_managed=str((cfg.get("flight_controller") or {}).get("field_source","jetson_files"))=="mission_planner"
        visual_red=str((cfg.get("red_avoidance") or {}).get("mode","coordinate"))=="visual"
        report=run_lifecycle_audit(
            vehicle,mav_cfg,args.coordinates,args.lifecycle_seconds,
            require_local_containment=not fc_managed,
            require_coordinate_red_exclusion=not visual_red,
        )
        print(json.dumps(report,indent=2))
        if not report["passed"]: raise SystemExit(2)
        return
    if args.mode=="sync-pixhawk":
        from safety.pixhawk_field_sync import sync_field_to_pixhawk
        print("Pixhawk field sync is NON-ARMING. It writes and verifies home plus MAVLink2 polygon fences.")
        _geofence, report=sync_field_to_pixhawk(
            vehicle,args.coordinates,float(cfg.get("fence_margin_m",2.0))
        )
        print(json.dumps(report,indent=2)); return
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
            last = None; monitor_deadline = None if args.tx15_monitor_seconds <= 0 else time.monotonic() + args.tx15_monitor_seconds
            while monitor_deadline is None or time.monotonic() < monitor_deadline:
                state = guard.poll(.1); current = (state, guard.pwm, guard.rc_fresh(), guard.heartbeat_fresh())
                if current != last: print(f"SC state={state}, pwm={guard.pwm}, rc_fresh={current[2]}, heartbeat_fresh={current[3]}"); last = current
                time.sleep(.05)
            print("TX15 finite monitor complete. No flight-mode, arm, or takeoff command was sent.")
            return
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
    if args.mode=="configure-servo7":
        print("SERVO7 configuration is NON-ARMING. It writes only SERVO7_MAX and verifies read-back.")
        vehicle.poll(.3)
        actuator = Servo7Actuator(vehicle, **payload_cfg)
        actuator.configure_exact_lower_limit()
        actuator.verify_configuration()
        print(f"SERVO7_MAX verified at {actuator.lower_pwm}; payload drop remains {actuator.lower_seconds:.1f}s.")
        return
    if args.mode=="test-servo7":
        print("SERVO7 test is NON-ARMING. It captures RC12 neutral, then directly drives Servo 7.")
        print("Remove propellers, secure the winch, and hold RC12 at its actual neutral position.")
        vehicle.poll(.3)
        if vehicle.status.armed:
            raise RuntimeError("refusing Servo 7 bench test while Pixhawk reports ARMED")
        Servo7Actuator(vehicle,**payload_cfg).test_drop(args.servo7_pwm,args.servo7_seconds,args.servo7_neutral_pwm); return
    if args.mode in {"test-aruco","test-redzone","test-corridor","calibrate","mission"}:
        from perception.cameras import DownwardCamera
        from perception.oakd_corridor_avoidance import OakDCamera
        from perception.aruco_fiducial_engine import FiducialEngine
        from perception.red_zone_detector import detect_red_zone
        geofence=None
        if args.mode in {"calibrate","mission"}:
            from safety.pixhawk_field_sync import sync_field_to_pixhawk
            geofence,sync_report=sync_field_to_pixhawk(
                vehicle,args.coordinates,float(cfg.get("fence_margin_m",2.0))
            )
            print("Pixhawk field configuration verified: " + json.dumps(sync_report))
        down=DownwardCamera(); oak=OakDCamera(float(cfg.get("oakd_tilt_deg",45)))
        if args.mode=="test-aruco":
            frame=down.read(); d=FiducialEngine().detect(frame); print("none" if d is None else {"aruco_id":d.aruco_id,"qr":d.qr_data,"matrix":None if d.matrix is None else d.matrix.tolist()});
            if args.save_ref and d: from mission.state_machine import MarkerVault; MarkerVault().set_reference(d,vehicle.status.lat,vehicle.status.lon)
            return
        if args.mode=="test-redzone":
            print(detect_red_zone(down.read())); return
        if args.mode=="test-corridor":
            print(oak.read(args.color)); return
        from mission.profiles import build_mission
        from streaming.wifi_hud_streamer import WifiHudStreamer
        plan=load_mission(args.wp_file,args.mapping); hud=None
        if not args.no_hud:
            network=yaml.safe_load(Path("config/network.yaml").read_text()) if Path("config/network.yaml").exists() else {}
            stream_host=str(network.get("jetson_stream_host","10.233.44.10")); stream_port=int(network.get("stream_port",5000))
            hud=WifiHudStreamer(port=stream_port); hud.start(); vehicle.log(f"HUD http://{stream_host}:{stream_port}/stream")
        vehicle.request_message_intervals()
        checkpoint_path="storage/mission_checkpoint.json" if mission_profile=="full" else f"storage/mission_checkpoint_{mission_profile}.json"
        machine=build_mission(
            mission_profile,
            vehicle,geofence,FailsafeWatchdog(vehicle),plan,down,oak,
            Servo7Actuator(vehicle,**payload_cfg),CheckpointManager(checkpoint_path),hud=hud,operator_guard=guard,
            corridor_distance_m=float(cfg.get("corridor_distance_m",7.5)),
            delivery_altitude_m=float(cfg.get("delivery_altitude_m",5.0)),
            delivery_altitude_tolerance_m=float(cfg.get("delivery_altitude_tolerance_m",.3)),
            delivery_center_tolerance_m=float(cfg.get("delivery_center_tolerance_m",.15)),
            downcam_yaw_deg=float(cfg.get("downcam_yaw_deg",0.0)),
            require_red_exclusion=bool(cfg.get("require_red_exclusion",True)),
            minimum_search_waypoints=int(cfg.get("minimum_search_waypoints",5)),
            visual_red_avoidance=str((cfg.get("red_avoidance") or {}).get("mode","coordinate"))=="visual",
            red_check_interval_s=float((cfg.get("red_avoidance") or {}).get("check_interval_s",.25)),
            profile_config=profile_cfg.get(mission_profile,{}),
        )
        if args.reset: machine.checkpoint.clear()
        if args.mode=="calibrate": machine.preflight_calibrate()
        else: machine.run(args.resume)
        return
    raise AssertionError("unreachable mode")
if __name__=="__main__":
    try: main()
    except KeyboardInterrupt: sys.exit(130)
    except Exception as exc: print(f"ERROR: {exc}",file=sys.stderr); sys.exit(2)
