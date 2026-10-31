from __future__ import annotations

import socket
import time
from pathlib import Path
from typing import Any

from control.mavlink_interface import MavlinkVehicle, mavutil
from core.field_coordinates import load_field_layout


def _check(name: str, passed: bool | None, detail: str) -> dict[str, Any]:
    return {"name": name, "status": "SKIP" if passed is None else ("PASS" if passed else "FAIL"), "detail": detail}


def _selected_source_ip(destination: str, port: int) -> str:
    """Ask the OS which local IPv4 address routes to the Pixhawk; sends no packet."""
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.connect((destination, port))
        return str(sock.getsockname()[0])


def _read_parameters(vehicle: MavlinkVehicle, names: tuple[str, ...]) -> dict[str, float | None]:
    return {name: vehicle.read_parameter(name) for name in names}


def run_lifecycle_audit(
    vehicle: MavlinkVehicle,
    mavlink_config: dict[str, Any],
    coordinates_path: str | Path,
    sample_seconds: float = 5.0,
    require_local_containment: bool = True,
    require_coordinate_red_exclusion: bool = True,
) -> dict[str, Any]:
    """Non-arming Ethernet, telemetry, coordinate, failsafe, and winch-readiness audit."""
    if not 1.0 <= sample_seconds <= 30.0:
        raise ValueError("lifecycle sample must be between 1 and 30 seconds")
    checks: list[dict[str, Any]] = []

    layout = load_field_layout(coordinates_path)
    coordinate_audit = layout.audit(
        require_containment=require_local_containment,
        require_red_exclusion=require_coordinate_red_exclusion,
    )
    checks.append(_check(
        "field_coordinates",
        bool(coordinate_audit["flight_ready"]),
        "normalized spreadsheet geometry is flight-ready" if coordinate_audit["flight_ready"]
        else "; ".join(coordinate_audit["blockers"]),
    ))

    expected_transport = str(mavlink_config.get("transport", "ethernet"))
    if vehicle.dry_run:
        checks.append(_check("ethernet_route", None, "dry-run does not inspect the host network"))
        checks.append(_check("mavlink_transport", None, "dry-run has no physical transport"))
    else:
        pixhawk_ip = str(mavlink_config.get("pixhawk_ip", "192.168.168.2"))
        jetson_ip = str(mavlink_config.get("jetson_ip", "192.168.168.10"))
        port = int(mavlink_config.get("udp_port", 14550))
        try:
            source_ip = _selected_source_ip(pixhawk_ip, port)
            checks.append(_check(
                "ethernet_route", source_ip == jetson_ip,
                f"OS route to Pixhawk {pixhawk_ip} uses {source_ip}; expected Jetson {jetson_ip}",
            ))
        except OSError as exc:
            checks.append(_check("ethernet_route", False, f"cannot route to Pixhawk {pixhawk_ip}: {exc}"))
        checks.append(_check(
            "mavlink_transport",
            expected_transport == "ethernet" and vehicle.connected_transport == "ethernet",
            f"configured={expected_transport}, connected={vehicle.connected_transport}",
        ))

    vehicle.poll(.2)
    checks.append(_check("vehicle_disarmed", not vehicle.status.armed, f"armed={vehicle.status.armed}, mode={vehicle.status.mode}"))

    if vehicle.dry_run:
        checks.append(_check("telemetry_sample", None, "dry-run telemetry is simulated"))
    else:
        message_ids = (
            mavutil.mavlink.MAVLINK_MSG_ID_HEARTBEAT,
            mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT,
            mavutil.mavlink.MAVLINK_MSG_ID_GPS_RAW_INT,
            mavutil.mavlink.MAVLINK_MSG_ID_RC_CHANNELS,
        )
        # A 2 Hz sample is sufficient for lifecycle health and avoids a test
        # that permanently increases routine telemetry rates.
        vehicle.request_message_intervals(message_ids, interval_us=500_000)
        before = dict(vehicle.message_counts)
        deadline = time.monotonic() + sample_seconds
        while time.monotonic() < deadline:
            vehicle.poll(.1)
        observed = {name: vehicle.message_counts.get(name, 0) - before.get(name, 0) for name in (
            "HEARTBEAT", "GLOBAL_POSITION_INT", "GPS_RAW_INT", "RC_CHANNELS"
        )}
        missing = [name for name, count in observed.items() if count < 1]
        checks.append(_check(
            "telemetry_sample", not missing,
            f"{sample_seconds:.1f}s at requested 2 Hz: {observed}" + ("" if not missing else f"; missing {missing}"),
        ))

    if vehicle.dry_run:
        checks.extend([
            _check("pixhawk_ethernet_parameters", None, "dry-run cannot read Pixhawk parameters"),
            _check("flight_controller_failsafes", None, "dry-run cannot read Pixhawk parameters"),
            _check("winch_readiness", None, "dry-run cannot read Servo 7 or live RC12 neutral"),
        ])
    else:
        network_names = (
            "NET_ENABLE", "NET_NETMASK", "NET_P1_PORT", "NET_P1_PROTOCOL", "NET_P1_TYPE",
            "NET_IPADDR0", "NET_IPADDR1", "NET_IPADDR2", "NET_IPADDR3",
            "NET_P1_IP0", "NET_P1_IP1", "NET_P1_IP2", "NET_P1_IP3",
        )
        network = _read_parameters(vehicle, network_names)
        pixhawk_octets = [int(mavlink_config.get("pixhawk_ip", "192.168.168.2").split(".")[i]) for i in range(4)]
        jetson_octets = [int(mavlink_config.get("jetson_ip", "192.168.168.10").split(".")[i]) for i in range(4)]
        expected = {
            "NET_ENABLE": 1, "NET_NETMASK": 24,
            "NET_P1_PORT": int(mavlink_config.get("udp_port", 14550)),
            "NET_P1_PROTOCOL": 2, "NET_P1_TYPE": 1,
            **{f"NET_IPADDR{i}": value for i, value in enumerate(pixhawk_octets)},
            **{f"NET_P1_IP{i}": value for i, value in enumerate(jetson_octets)},
        }
        mismatches = [
            f"{name}={network[name]} expected {value}"
            for name, value in expected.items()
            if network.get(name) is None or int(round(float(network[name]))) != value
        ]
        checks.append(_check(
            "pixhawk_ethernet_parameters", not mismatches,
            "Pixhawk UDP-client settings match" if not mismatches else "; ".join(mismatches),
        ))

        failsafes = _read_parameters(vehicle, ("FS_GCS_ENABLE", "BATT_FS_LOW_ACT", "BATT_FS_CRT_ACT"))
        disabled = [name for name, value in failsafes.items() if value is None or int(round(value)) == 0]
        checks.append(_check(
            "flight_controller_failsafes", not disabled,
            f"values={failsafes}" + ("" if not disabled else f"; configure non-zero actions for {disabled}"),
        ))

        servo = _read_parameters(vehicle, (
            "SERVO7_FUNCTION", "SERVO7_MIN", "SERVO7_MAX", "SERVO7_TRIM", "RC12_TRIM"
        ))
        winch_problems = []
        if any(value is None for value in servo.values()):
            winch_problems.append("one or more Servo 7/RC12 parameters could not be read")
        else:
            function = int(round(servo["SERVO7_FUNCTION"]))
            if not (function in {0, 1} or 51 <= function <= 66 or 140 <= function <= 155):
                winch_problems.append(f"SERVO7_FUNCTION={function} blocks servo commands")
            if int(round(servo["SERVO7_MAX"])) < 2000:
                winch_problems.append(f"SERVO7_MAX={servo['SERVO7_MAX']:.0f} clips PWM 2000")
            if int(round(servo["SERVO7_MIN"])) > 1100:
                winch_problems.append(f"SERVO7_MIN={servo['SERVO7_MIN']:.0f} clips PWM 1100")
        neutral = None
        if not vehicle.status.armed:
            try:
                neutral = vehicle.capture_rc_channel_pwm(12)
            except Exception as exc:
                winch_problems.append(f"live RC12 neutral capture failed: {exc}")
        else:
            winch_problems.append("vehicle is armed; neutral capture refused")
        if neutral is not None and servo.get("SERVO7_TRIM") is not None and abs(neutral - int(round(servo["SERVO7_TRIM"]))) > 5:
            winch_problems.append(f"SERVO7_TRIM={servo['SERVO7_TRIM']:.0f} differs from live RC12 neutral {neutral}")
        checks.append(_check(
            "winch_readiness", not winch_problems,
            f"parameters={servo}, live_rc12_neutral={neutral}" + ("" if not winch_problems else "; " + "; ".join(winch_problems)),
        ))

    failed = [item["name"] for item in checks if item["status"] == "FAIL"]
    return {
        "non_arming": True,
        "passed": not failed,
        "failed_checks": failed,
        "checks": checks,
        "coordinate_audit": coordinate_audit,
    }
