from __future__ import annotations
import math, time
from dataclasses import dataclass
from typing import Callable

try:
    from pymavlink import mavutil
except ImportError:
    mavutil = None


@dataclass
class VehicleStatus:
    lat: float = 0.0; lon: float = 0.0; relative_alt_m: float = 0.0
    battery_v: float = 0.0; satellites: int = 0; hdop: float = 99.0
    ekf_ok: bool = False; armed: bool = False; mode: str = "UNKNOWN"; heading_deg: float = 0.0
    rc_channels: dict[int, int] | None = None; rc_sequence: int = 0
    last_rc_monotonic: float = 0.0; last_heartbeat_monotonic: float = 0.0


class MavlinkVehicle:
    """MAVLink2 adapter. All commands are no-ops except logging in dry-run mode."""
    def __init__(self, endpoint: str = "udpin:0.0.0.0:14550", uart: str = "/dev/ttyTHS1", baud: int = 115200,
                 dry_run: bool = False, logger: Callable[[str], None] | None = None):
        self.endpoint, self.uart, self.baud, self.dry_run = endpoint, uart, baud, dry_run
        self.logger = logger or print; self.connection = None; self.status = VehicleStatus()
        self._last_velocity = (0.0, 0.0, 0.0)
        self._command_acks: dict[int, int] = {}

    def connect(self, timeout: float = 12.0) -> None:
        if self.dry_run:
            self.status = VehicleStatus(12.9716, 77.5946, 0, 22.2, 12, 0.8, True, False, "GUIDED", 0.0, {8: 1000}, 1, time.monotonic(), time.monotonic())
            self.log("DRY-RUN MAVLink connection ready"); return
        if mavutil is None: raise RuntimeError("pymavlink is required; install requirements.txt")
        errors = []
        for device, kwargs in ((self.endpoint, {}), (self.uart, {"baud": self.baud})):
            try:
                self.connection = mavutil.mavlink_connection(device, autoreconnect=True, **kwargs)
                self.connection.wait_heartbeat(timeout=timeout)
                self.log(f"MAVLink heartbeat via {device}"); return
            except Exception as exc:
                errors.append(f"{device}: {exc}")
        raise ConnectionError("; ".join(errors))

    def poll(self, duration: float = 0.05) -> VehicleStatus:
        if self.dry_run or self.connection is None: return self.status
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            message = self.connection.recv_match(blocking=False)
            if message is None: time.sleep(.005); continue
            kind = message.get_type()
            if kind == "GLOBAL_POSITION_INT":
                self.status.lat, self.status.lon, self.status.relative_alt_m = message.lat / 1e7, message.lon / 1e7, message.relative_alt / 1000
                self.status.heading_deg = message.hdg / 100 if message.hdg != 65535 else self.status.heading_deg
            elif kind == "SYS_STATUS": self.status.battery_v = message.voltage_battery / 1000
            elif kind == "GPS_RAW_INT": self.status.satellites, self.status.hdop = message.satellites_visible, message.eph / 100
            elif kind == "EKF_STATUS_REPORT": self.status.ekf_ok = not bool(message.flags & 0b111000)
            elif kind == "HEARTBEAT":
                self.status.armed = bool(message.base_mode & 128); self.status.mode = mavutil.mode_string_v10(message)
                self.status.last_heartbeat_monotonic = time.monotonic()
            elif kind == "RC_CHANNELS":
                self.status.rc_channels = {index: getattr(message, f"chan{index}_raw") for index in range(1, 19) if getattr(message, f"chan{index}_raw", 0) not in (0, 65535)}
                self.status.rc_sequence += 1; self.status.last_rc_monotonic = time.monotonic()
            elif kind == "COMMAND_ACK": self._command_acks[message.command] = message.result
        return self.status

    def log(self, text: str, severity: int = 6) -> None:
        text = text[:50]; self.logger(text)
        if self.connection is not None and not self.dry_run:
            self.connection.mav.statustext_send(severity, text.encode("ascii", "replace"))

    def _target(self) -> tuple[int, int]:
        if self.connection is None: return 1, 1
        return self.connection.target_system, self.connection.target_component

    def set_mode(self, mode: str) -> None:
        self.log(f"MODE {mode}")
        if self.dry_run: self.status.mode = mode; return
        mapping = self.connection.mode_mapping();
        if mode not in mapping: raise ValueError(f"mode unsupported by vehicle: {mode}")
        self.connection.mav.set_mode_send(self._target()[0], mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mapping[mode])

    def request_message_intervals(self, message_ids: tuple[int, ...] | None = None, interval_us: int = 100_000) -> None:
        """Request 10 Hz position/RC/heartbeat telemetry required by the TX15 safety guard."""
        if self.dry_run: return
        if message_ids is None:
            message_ids = (mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, mavutil.mavlink.MAVLINK_MSG_ID_RC_CHANNELS)
        for message_id in message_ids:
            self.command_long(mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, message_id, interval_us)

    def _wait_for_ack(self, command: int, timeout_s: float, cancel: Callable[[], bool] | None = None) -> bool:
        if self.dry_run: return True
        deadline = time.monotonic() + timeout_s
        denied = {mavutil.mavlink.MAV_RESULT_DENIED, mavutil.mavlink.MAV_RESULT_FAILED, mavutil.mavlink.MAV_RESULT_UNSUPPORTED}
        while time.monotonic() < deadline:
            self.poll(.05)
            if cancel and cancel(): return False
            result = self._command_acks.pop(command, None)
            if result is not None: return result == mavutil.mavlink.MAV_RESULT_ACCEPTED
        return False

    def set_mode_confirmed(self, mode: str, timeout_s: float = 3.0, cancel: Callable[[], bool] | None = None) -> bool:
        if self.dry_run: self.set_mode(mode); return True
        mapping = self.connection.mode_mapping()
        if mode not in mapping: self.log(f"Unsupported mode {mode}", 3); return False
        mode_id = mapping[mode]; deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if cancel and cancel(): return False
            self._command_acks.pop(mavutil.mavlink.MAV_CMD_DO_SET_MODE, None)
            self.connection.mav.command_long_send(*self._target(), mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode_id, 0, 0, 0, 0, 0)
            wait_until = min(deadline, time.monotonic() + .5)
            while time.monotonic() < wait_until:
                self.poll(.05)
                if self.status.mode == mode: self.log(f"Mode confirmed {mode}"); return True
                if self._command_acks.pop(mavutil.mavlink.MAV_CMD_DO_SET_MODE, None) in {mavutil.mavlink.MAV_RESULT_DENIED, mavutil.mavlink.MAV_RESULT_FAILED, mavutil.mavlink.MAV_RESULT_UNSUPPORTED}: return False
        self.log(f"Mode timeout {mode}", 3); return False

    def arm(self) -> None:
        self.log("ARM");
        if not self.dry_run: self.connection.mav.command_long_send(*self._target(), mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
        self.status.armed = True

    def arm_confirmed(self, arm: bool = True, timeout_s: float = 6.0, cancel: Callable[[], bool] | None = None) -> bool:
        if self.dry_run: self.status.armed = arm; self.log("DRY ARM" if arm else "DRY DISARM"); return True
        value = 1 if arm else 0; command = mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM; self._command_acks.pop(command, None)
        self.connection.mav.command_long_send(*self._target(), command, 0, value, 0, 0, 0, 0, 0, 0)
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            self.poll(.05)
            if cancel and cancel(): return False
            if self.status.armed == arm: self.log("Arming confirmed" if arm else "Disarm confirmed"); return True
            if self._command_acks.pop(command, None) in {mavutil.mavlink.MAV_RESULT_DENIED, mavutil.mavlink.MAV_RESULT_FAILED, mavutil.mavlink.MAV_RESULT_UNSUPPORTED}: return False
        return False

    def takeoff(self, altitude_m: float) -> None:
        self.log(f"TAKEOFF {altitude_m:.1f}m")
        if not self.dry_run: self.connection.mav.command_long_send(*self._target(), mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, altitude_m)
        self.status.relative_alt_m = altitude_m

    def takeoff_confirmed(self, altitude_m: float, timeout_s: float = 3.0, cancel: Callable[[], bool] | None = None) -> bool:
        if self.dry_run: self.takeoff(altitude_m); return True
        command = mavutil.mavlink.MAV_CMD_NAV_TAKEOFF; self._command_acks.pop(command, None)
        self.connection.mav.command_long_send(*self._target(), command, 0, 0, 0, 0, 0, 0, 0, altitude_m)
        return self._wait_for_ack(command, timeout_s, cancel)

    def goto_gps(self, lat: float, lon: float, alt_m: float) -> None:
        self.log(f"GOTO {lat:.7f},{lon:.7f} @{alt_m:.1f}m")
        if not self.dry_run:
            self.connection.mav.set_position_target_global_int_send(int(time.time()*1000) & 0xffffffff, *self._target(),
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, 0b0000111111111000, int(lat*1e7), int(lon*1e7), alt_m, 0,0,0,0,0,0,0,0)
        else: self.status.lat, self.status.lon, self.status.relative_alt_m = lat, lon, alt_m

    def send_body_velocity(self, vx: float, vy: float, vz: float = 0.0) -> None:
        self._last_velocity = (vx, vy, vz)
        if self.dry_run: self.log(f"VEL x={vx:.2f} y={vy:.2f} z={vz:.2f}"); return
        self.connection.mav.set_position_target_local_ned_send(int(time.time()*1000) & 0xffffffff, *self._target(),
            mavutil.mavlink.MAV_FRAME_BODY_NED, 0b0000111111000111, 0,0,0,vx,vy,vz,0,0,0,0,0)

    def command_long(self, command: int, *params: float) -> None:
        values = list(params) + [0.0] * (7-len(params))
        self.log(f"COMMAND {command}")
        if not self.dry_run: self.connection.mav.command_long_send(*self._target(), command, 0, *values[:7])

    def set_parameter(self, name: str, value: float) -> None:
        self.log(f"PARAM {name}={value:g}")
        if not self.dry_run:
            self.connection.mav.param_set_send(*self._target(), name.encode("ascii")[:16], value, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)

    def send_fence_point(self, index: int, total: int, lat: float, lon: float) -> None:
        self.log(f"FENCE {index+1}/{total}")
        if not self.dry_run: self.connection.mav.fence_point_send(*self._target(), index, total, lat, lon)

    def rc_override(self, channel_12_pwm: int | None) -> None:
        self.log(f"RC12 {'release' if channel_12_pwm is None else channel_12_pwm}")
        if self.dry_run: return
        channels = [65535] * 18; channels[11] = 0 if channel_12_pwm is None else channel_12_pwm
        self.connection.mav.rc_channels_override_send(*self._target(), *channels)

    def land(self) -> None: self.set_mode("LAND")
    def rtl(self) -> None: self.set_mode("RTL")
