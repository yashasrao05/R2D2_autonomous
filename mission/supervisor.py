from __future__ import annotations

import os
import threading
import time
from pathlib import Path

import yaml

from control.checkpoint_manager import CheckpointManager
from control.mavlink_interface import MavlinkVehicle
from control.servo7_actuator import Servo7Actuator
from core.field_coordinates import load_field_layout
from core.mission_runtime import MissionRuntime
from core.waypoint_loader import load_mission
from mission.profiles import build_mission
from safety.competition_readiness import run_competition_readiness
from safety.fc_flightplan_sync import load_upload_manifest, sync_flightplan_to_fc
from safety.failsafe_watchdog import FailsafeWatchdog
from safety.geofence_enforcer import GeofenceEnforcer
from safety.tx15_override import PilotOverride, Tx15Config, Tx15SafetyGuard
from streaming.wifi_hud_streamer import WifiHudStreamer


def _guard(vehicle, config) -> Tx15SafetyGuard:
    tx = config.get("tx15", {})
    return Tx15SafetyGuard(vehicle, Tx15Config(
        channel=int(tx.get("rc_channel", 8)),
        up_pwm=int(tx.get("sc_up_pwm", 1700)),
        down_pwm=int(tx.get("sc_down_pwm", 1300)),
        debounce_count=int(tx.get("debounce_count", 3)),
        rc_timeout_s=float(tx.get("rc_timeout_s", 2)),
        heartbeat_timeout_s=float(tx.get("heartbeat_timeout_s", 3)),
        manual_mode=str(tx.get("manual_fallback_mode", "ALT_HOLD")),
        backup_mode=str(tx.get("backup_fallback_mode", "STABILIZE")),
    ))


class CompetitionSupervisor:
    """Boot-safe one-run supervisor. Only the physical TX15 switch can authorize flight."""

    def __init__(self, args, config: dict):
        self.args = args
        self.config = config
        self.mav_config = config.get("mavlink", {})
        self.profile_config = config.get("mission_profiles", {})
        self.payload_config = config.get("payload", {})
        self.runtime = MissionRuntime()
        self._selection = threading.Event()
        self._selection_lock = threading.Lock()
        self._selected_profile: str | None = None
        self.down = None
        self.oak = None
        self._preview_stop = threading.Event()
        self._preview_thread: threading.Thread | None = None

        self.vehicle = MavlinkVehicle(
            endpoint=args.ethernet_endpoint or str(self.mav_config.get("ethernet_endpoint", "udpin:0.0.0.0:14550")),
            uart=args.uart_device or str(self.mav_config.get("uart_device", "/dev/ttyTHS1")),
            baud=args.uart_baud or int(self.mav_config.get("uart_baud", 115200)),
            transport=args.transport or str(self.mav_config.get("transport", "ethernet")),
            dry_run=args.dry_run,
            logger=lambda message: self.runtime.log(message, source="flight"),
        )
        self.guard = _guard(self.vehicle, config)
        self.runtime.bind(vehicle=self.vehicle, guard=self.guard)

        network_path = Path("config/network.yaml")
        network = yaml.safe_load(network_path.read_text(encoding="utf-8")) if network_path.exists() else {}
        self.stream_host = str(network.get("jetson_stream_host", "10.233.44.10"))
        self.stream_port = int(network.get("stream_port", 5000))
        self.control_token = os.environ.get("R2D2_CONTROL_TOKEN", "")
        self.hud = WifiHudStreamer(
            port=self.stream_port,
            runtime=self.runtime,
            control_token=self.control_token,
            profile_selector=self.select_profile,
            stream_fps=float(network.get("stream_fps", 4)),
            jpeg_quality=int(network.get("stream_jpeg_quality", 65)),
            camera_width=int(network.get("stream_camera_width", 640)),
        )

    def select_profile(self, profile: str) -> tuple[bool, str]:
        if profile not in {"full", "segment1", "segment2"}:
            return False, "Select full, segment1, or segment2."
        with self._selection_lock:
            allowed, reason = self.runtime.can_select_profile()
            if not allowed:
                return False, reason
            self._selected_profile = profile
            self.runtime.update(profile=profile, profile_locked=True)
            self.runtime.log(f"Official mission profile locked: {profile}", source="operator")
            self._selection.set()
            return True, f"{profile} locked. Preparing field sync and cameras; keep TX15 SC DOWN."

    def _connect(self) -> None:
        self.runtime.transition("LINK_WAIT", "Waiting for Pixhawk 6X Mini Ethernet heartbeat")
        while self.vehicle.connected_transport is None:
            try:
                self.vehicle.connect(timeout=5.0)
            except Exception as exc:
                self.runtime.update(last_error=str(exc))
                self.runtime.log(f"Pixhawk link retry: {exc}", level="WARNING", source="supervisor")
                time.sleep(3.0)
        self.vehicle.request_message_intervals()
        self.runtime.update(last_error=None)

    def _readiness(self) -> bool:
        self.runtime.transition("BOOT_CHECKS", "Running non-arming competition readiness checks")
        report = run_competition_readiness(
            self.vehicle, self.mav_config, self.args.coordinates,
            self.args.wp_file, self.args.mapping, self.args.fence_file,
            float(self.config.get("fence_margin_m", 2.0)),
            min(max(float(self.args.lifecycle_seconds), 1.0), 5.0),
        )
        blockers = list(report["blockers"])
        if not self.control_token:
            blockers.append("dashboard_control: R2D2_CONTROL_TOKEN is missing from /etc/default/r2d2-autonomous")
        passed = report["passed"] and bool(self.control_token)
        self.runtime.update(
            readiness_passed=passed,
            readiness_checks=report["checks"],
            blockers=blockers,
        )
        self.vehicle.request_message_intervals()
        if not passed:
            self.runtime.transition("BLOCKED", "Competition readiness blocked; no profile can be selected")
        return passed

    def _mission_status(self, state: str, phase: str, detail: str | None) -> None:
        self.runtime.transition(state, detail, mission_phase=phase)

    def _preview_loop(self) -> None:
        try:
            while not self._preview_stop.is_set():
                scene = self.oak.read("green")
                self.hud.update(self.down.read(), scene.rgb)
                self._preview_stop.wait(.2)
        except Exception as exc:
            blocker = f"camera_preview: {type(exc).__name__}: {exc}"
            snapshot = self.runtime.snapshot()
            self.runtime.update(
                readiness_passed=False,
                blockers=[*snapshot.get("blockers", []), blocker],
                last_error=blocker,
            )
            self.runtime.transition("BLOCKED", "Camera preview failed; mission selection disabled")

    def _start_camera_preview(self) -> bool:
        self.runtime.transition("CAMERA_CHECK", "Opening and validating both mission cameras")
        from perception.cameras import DownwardCamera
        from perception.oakd_corridor_avoidance import OakDCamera
        self.down = DownwardCamera()
        self.oak = OakDCamera(float(self.config.get("oakd_tilt_deg", 45)))
        try:
            down_frame = self.down.read()
            oak_scene = self.oak.read("green")
            self.hud.update(down_frame, oak_scene.rgb)
        except Exception as exc:
            blocker = f"camera_hardware: {type(exc).__name__}: {exc}"
            snapshot = self.runtime.snapshot()
            checks = [*snapshot.get("readiness_checks", []), {
                "name": "camera_hardware", "status": "FAIL", "detail": str(exc),
            }]
            self.runtime.update(
                readiness_passed=False,
                readiness_checks=checks,
                blockers=[*snapshot.get("blockers", []), blocker],
                last_error=blocker,
            )
            self.runtime.transition("BLOCKED", "Mission camera check failed; no profile can be selected")
            self._close_cameras()
            return False
        snapshot = self.runtime.snapshot()
        self.runtime.update(readiness_checks=[*snapshot.get("readiness_checks", []), {
            "name": "camera_hardware", "status": "PASS",
            "detail": f"downward={self.down.backend_name}; OAK-D RGB/depth ready",
        }])
        self.runtime.log(
            f"Camera check passed: downward={self.down.backend_name}, OAK-D RGB/depth ready",
            source="supervisor",
        )
        self._preview_thread = threading.Thread(target=self._preview_loop, daemon=True, name="r2d2-camera-preview")
        self._preview_thread.start()
        return True

    def _stop_preview(self) -> None:
        self._preview_stop.set()
        if self._preview_thread is not None:
            self._preview_thread.join(timeout=5.0)
            if self._preview_thread.is_alive():
                raise RuntimeError("camera preview did not stop before mission control")
            self._preview_thread = None

    def _close_cameras(self) -> None:
        if self.down is not None:
            self.down.close()
            self.down = None
        if self.oak is not None:
            self.oak.close()
            self.oak = None

    def _build_and_run(self, profile: str) -> None:
        fc_config = self.config.get("flight_controller") or {}
        fc_managed = str(fc_config.get("field_source", "jetson_files")) == "mission_planner"
        if fc_managed:
            self.runtime.transition("FIELD_VERIFY", "Downloading and verifying the Mission Planner fence without changing FC state")
            polygons = self.vehicle.download_fence_polygons()
            geofence = GeofenceEnforcer(
                self.vehicle, polygons, float(self.config.get("fence_margin_m", 2.0))
            )
            sync_report = {
                "non_arming": True, "read_only": True, "source": "mission_planner",
                "inclusion_vertices": len(polygons[0]),
                "exclusion_vertices": sum(len(polygon) for polygon in polygons[1:]),
            }
        else:
            self.runtime.transition("FIELD_SYNC", "Writing and reading back FC parameters, home, fences, and mission")
            sync_report = sync_flightplan_to_fc(self.vehicle, self.args.upload_manifest)
            manifest = load_upload_manifest(self.args.upload_manifest)
            layout = load_field_layout(manifest["files"]["coordinates"])
            geofence = GeofenceEnforcer(
                self.vehicle, layout.pixhawk_fence_polygons(), manifest["parameters"]["FENCE_MARGIN"]
            )
        self.runtime.update(fence_sync=sync_report)
        self.runtime.log(
            "Mission Planner fence read-back passed" if fc_managed
            else "Pixhawk parameters/home/fence/mission read-back passed",
            source="supervisor",
        )

        if self.down is None or self.oak is None:
            raise RuntimeError("mission cameras were not retained after the boot hardware check")
        plan = load_mission(self.args.wp_file, self.args.mapping)
        checkpoint_path = f"storage/mission_checkpoint_{profile}.json"
        machine = build_mission(
            profile,
            self.vehicle, geofence, FailsafeWatchdog(self.vehicle), plan, self.down, self.oak,
            Servo7Actuator(self.vehicle, **self.payload_config), CheckpointManager(checkpoint_path),
            hud=self.hud, operator_guard=self.guard,
            corridor_distance_m=float(self.config.get("corridor_distance_m", 7.5)),
            delivery_altitude_m=float(self.config.get("delivery_altitude_m", 5.0)),
            delivery_altitude_tolerance_m=float(self.config.get("delivery_altitude_tolerance_m", .3)),
            delivery_center_tolerance_m=float(self.config.get("delivery_center_tolerance_m", .15)),
            downcam_yaw_deg=float(self.config.get("downcam_yaw_deg", 0.0)),
            require_red_exclusion=bool(self.config.get("require_red_exclusion", True)),
            minimum_search_waypoints=int(self.config.get("minimum_search_waypoints", 5)),
            visual_red_avoidance=str((self.config.get("red_avoidance") or {}).get("mode", "coordinate")) == "visual",
            red_check_interval_s=float((self.config.get("red_avoidance") or {}).get("check_interval_s", .25)),
            profile_config=self.profile_config.get(profile, {}),
            status_callback=self._mission_status,
        )
        if self.args.reset:
            machine.checkpoint.clear()
        machine.run(False)
        self.runtime.transition("COMPLETE", f"Official run {profile} complete and disarmed")

    def _monitor_terminal(self) -> None:
        while True:
            self.guard.poll(.2)
            time.sleep(.1)

    def run(self) -> None:
        self.hud.start()
        self.runtime.log(f"Field console http://{self.stream_host}:{self.stream_port}/", source="supervisor")
        self.runtime.log("Boot service is non-arming until profile lock and physical TX15 SC-UP", source="supervisor")
        self._connect()
        if not self._readiness():
            self._monitor_terminal()
        if not self._start_camera_preview():
            self._monitor_terminal()

        self.runtime.transition("IDLE_SELECT_PROFILE", "Ready for official profile selection; keep TX15 SC DOWN")
        while not self._selection.is_set():
            self.guard.poll(.1)
            time.sleep(.05)
        profile = self._selected_profile
        if profile is None:
            raise RuntimeError("profile selection event occurred without a selected profile")
        try:
            self._stop_preview()
            if not self.runtime.snapshot()["readiness_passed"]:
                raise RuntimeError("readiness was withdrawn before mission preparation")
            self._build_and_run(profile)
        except PilotOverride as exc:
            self.runtime.update(last_error=str(exc))
            self.runtime.transition("PILOT_CONTROL", f"Autonomy yielded to pilot: {exc}")
        except Exception as exc:
            self.runtime.update(last_error=f"{type(exc).__name__}: {exc}")
            self.runtime.transition("FAULT", f"Mission locked after {type(exc).__name__}: {exc}")
        self._close_cameras()
        self._monitor_terminal()


def run_competition_service(args, config: dict) -> None:
    CompetitionSupervisor(args, config).run()
