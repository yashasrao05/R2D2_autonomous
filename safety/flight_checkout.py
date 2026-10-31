from __future__ import annotations

import math
import time
from types import SimpleNamespace

from control.mavlink_interface import mavutil
from core.field_coordinates import load_field_layout
from core.waypoint_loader import load_mission
from safety.failsafe_watchdog import FailsafeWatchdog
from safety.fc_flightplan_sync import load_upload_manifest, sync_flightplan_to_fc
from safety.geofence_enforcer import GeofenceEnforcer
from safety.tx15_override import PilotOverride


def _horizontal_distance_m(lat: float, lon: float, target_lat: float, target_lon: float) -> float:
    north = (lat - target_lat) * 111_319.49
    east = (lon - target_lon) * 111_319.49 * math.cos(math.radians(target_lat))
    return math.hypot(north, east)


def _wait_position(vehicle, guard, watchdog, target, altitude_m: float,
                   horizontal_tolerance_m: float = 1.5, timeout_s: float = 30.0) -> None:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        guard.assert_autonomy(("GUIDED",))
        watchdog.tick()
        status = vehicle.status
        if (_horizontal_distance_m(status.lat, status.lon, target.lat, target.lon) <= horizontal_tolerance_m
                and abs(status.relative_alt_m - altitude_m) <= .6):
            return
        time.sleep(.1)
    raise TimeoutError(f"checkout did not reach waypoint {target.index} within {timeout_s:.0f}s")


def _land_and_wait(vehicle, guard, timeout_s: float = 45.0) -> None:
    def authorized() -> bool:
        guard.assert_autonomy(None)
        return False

    if not vehicle.set_mode_confirmed("LAND", timeout_s=3.0, cancel=authorized):
        raise RuntimeError("checkout LAND mode was rejected")
    if vehicle.dry_run:
        vehicle.status.armed = False
        vehicle.status.relative_alt_m = 0.0
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        guard.assert_autonomy(("LAND",))
        vehicle.poll(.1)
        if not vehicle.status.armed:
            vehicle.log("Checkout touchdown/disarm confirmed")
            return
        time.sleep(.1)
    raise TimeoutError("checkout LAND did not disarm within 45 seconds")


def run_flight_checkout(vehicle, guard, manifest_path: str, level: str = "hover",
                        takeoff_altitude_m: float = 2.0, hover_seconds: float = 10.0,
                        fc_managed: bool = False, waypoint_path: str = "config/mission.waypoints",
                        mapping_path: str = "config/mission_mapping.yaml",
                        fence_margin_m: float = 2.0) -> dict:
    """Explicit live checkout: arm/takeoff/hover, optional one-waypoint hop, then LAND."""
    if level not in {"hover", "hop"}:
        raise ValueError("checkout level must be hover or hop")
    if not 1.0 <= takeoff_altitude_m <= 5.0:
        raise ValueError("checkout takeoff altitude must be between 1m and 5m")
    if not 3.0 <= hover_seconds <= 30.0:
        raise ValueError("checkout hover duration must be between 3 and 30 seconds")
    if vehicle.status.armed:
        raise RuntimeError("refusing checkout because Pixhawk already reports ARMED")

    if fc_managed:
        polygons = vehicle.download_fence_polygons()
        plan = load_mission(waypoint_path, mapping_path)
        geofence = GeofenceEnforcer(vehicle, polygons, fence_margin_m)
        upload_report = {
            "non_arming": True, "read_only": True, "source": "mission_planner",
            "inclusion_vertices": len(polygons[0]),
            "exclusion_vertices": sum(len(polygon) for polygon in polygons[1:]),
        }
    else:
        upload_report = sync_flightplan_to_fc(vehicle, manifest_path)
        manifest = load_upload_manifest(manifest_path)
        plan = load_mission(manifest["files"]["waypoints"], manifest["files"]["mapping"])
        layout = load_field_layout(manifest["files"]["coordinates"])
        geofence = GeofenceEnforcer(
            vehicle, layout.pixhawk_fence_polygons(), manifest["parameters"]["FENCE_MARGIN"]
        )

    if not vehicle.dry_run:
        vehicle.request_message_intervals((
            mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT,
            mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED,
            mavutil.mavlink.MAVLINK_MSG_ID_GPS_RAW_INT,
            mavutil.mavlink.MAVLINK_MSG_ID_SYS_STATUS,
            mavutil.mavlink.MAVLINK_MSG_ID_EKF_STATUS_REPORT,
            mavutil.mavlink.MAVLINK_MSG_ID_RC_CHANNELS,
        ), interval_us=200_000)
        deadline = time.monotonic() + 15.0
        while time.monotonic() < deadline:
            vehicle.poll(.1)
            status = vehicle.status
            if status.battery_v > 0 and status.satellites >= 6 and status.hdop < 1.4 and status.ekf_ok:
                break
        else:
            status = vehicle.status
            raise RuntimeError(
                f"checkout preflight telemetry not ready: battery={status.battery_v:.1f}V, "
                f"sats={status.satellites}, hdop={status.hdop:.1f}, ekf={status.ekf_ok}"
            )

    status = vehicle.status
    if not (status.ekf_ok and status.satellites >= 6 and status.hdop < 1.4 and status.battery_v > 20.3):
        raise RuntimeError(
            f"checkout preflight failed: ekf={status.ekf_ok}, sats={status.satellites}, "
            f"hdop={status.hdop}, battery={status.battery_v:.1f}V"
        )
    vehicle.verify_flight_failsafes()
    if not vehicle.dry_run:
        arming_check = vehicle.read_parameter("ARMING_CHECK")
        if arming_check is None or int(round(arming_check)) == 0:
            raise RuntimeError(f"ARMING_CHECK must be enabled; read {arming_check}")

    vehicle.request_message_intervals()
    if vehicle.dry_run:
        for _ in range(guard.config.debounce_count):
            vehicle.status.rc_sequence += 1
            vehicle.status.last_rc_monotonic = time.monotonic()
            guard.observe()
    guard.require_initial_manual()
    vehicle.log(f"Checkout {level}: ready; SC UP will arm, take off to {takeoff_altitude_m:.1f}m, then LAND")
    if vehicle.dry_run:
        vehicle.status.rc_channels[guard.config.channel] = guard.config.up_pwm + 100
        for _ in range(guard.config.debounce_count):
            vehicle.status.rc_sequence += 1
            vehicle.status.last_rc_monotonic = time.monotonic()
            guard.observe()
    guard.wait_for_autonomy_authorization()

    watchdog = FailsafeWatchdog(vehicle)

    def authorized(expected=None) -> bool:
        guard.assert_autonomy(expected)
        return False

    try:
        if not vehicle.set_mode_confirmed("GUIDED", cancel=lambda: authorized(None)):
            raise RuntimeError("checkout GUIDED mode rejected")
        if not vehicle.arm_confirmed(True, cancel=lambda: authorized(("GUIDED",))):
            raise RuntimeError("checkout arming rejected")
        spool_deadline = time.monotonic() + .5
        while time.monotonic() < spool_deadline:
            guard.assert_autonomy(("GUIDED",)); watchdog.tick(); time.sleep(.05)
        if not vehicle.takeoff_confirmed(takeoff_altitude_m, cancel=lambda: authorized(("GUIDED",))):
            raise RuntimeError("checkout takeoff command rejected")

        climb_deadline = time.monotonic() + 15.0
        while time.monotonic() < climb_deadline:
            guard.assert_autonomy(("GUIDED",)); watchdog.tick()
            if vehicle.status.relative_alt_m >= takeoff_altitude_m - .2:
                break
            time.sleep(.1)
        else:
            raise TimeoutError("checkout did not reach takeoff altitude within 15 seconds")

        hover_lat, hover_lon = vehicle.status.lat, vehicle.status.lon
        stable_samples = 0
        total_samples = 0
        hover_deadline = time.monotonic() + (.2 if vehicle.dry_run else hover_seconds)
        while time.monotonic() < hover_deadline:
            guard.assert_autonomy(("GUIDED",)); watchdog.tick()
            drift = _horizontal_distance_m(vehicle.status.lat, vehicle.status.lon, hover_lat, hover_lon)
            altitude_error = abs(vehicle.status.relative_alt_m - takeoff_altitude_m)
            total_samples += 1
            stable_samples += int(drift <= 2.0 and altitude_error <= .8)
            if drift > 3.0:
                raise RuntimeError(f"checkout hover drift exceeded 3m ({drift:.1f}m)")
            time.sleep(.1)
        stability = stable_samples / max(total_samples, 1)
        if stability < .8:
            raise RuntimeError(f"checkout hover stability only {stability * 100:.0f}%")
        vehicle.log(f"Checkout hover passed: {stability * 100:.0f}% stable")

        completed = ["arm_takeoff_hover"]
        if level == "hop":
            target = plan.takeoff_scan_qr
            geofence.goto_gps(target, takeoff_altitude_m)
            _wait_position(vehicle, guard, watchdog, target, takeoff_altitude_m)
            vehicle.log(f"Checkout waypoint hop reached item {target.index}")
            home = SimpleNamespace(lat=plan.home.lat, lon=plan.home.lon, alt=takeoff_altitude_m, index=plan.home.index)
            geofence.goto_gps(home, takeoff_altitude_m)
            _wait_position(vehicle, guard, watchdog, home, takeoff_altitude_m)
            completed.append("safe_waypoint_hop_and_return")

        _land_and_wait(vehicle, guard)
        completed.append("land_disarm")
        return {
            "passed": True,
            "level": level,
            "takeoff_altitude_m": takeoff_altitude_m,
            "hover_seconds": hover_seconds,
            "tasks_completed": completed,
            "flightplan_write": upload_report,
        }
    except PilotOverride:
        raise
    except BaseException as exc:
        if vehicle.status.armed:
            vehicle.log(f"Checkout fault {type(exc).__name__}; requesting LAND", 2)
            vehicle.set_mode_confirmed("LAND", timeout_s=3.0)
        raise
