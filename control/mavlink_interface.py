from __future__ import annotations
import math, os, statistics, time
from dataclasses import dataclass
from typing import Callable

# Pymavlink chooses generated message signatures at import time. Force MAVLink
# 2 so RC9-18 and mission_type extensions are present on every entry point.
os.environ.setdefault("MAVLINK20", "1")

try:
    from pymavlink import mavutil
except ImportError:
    mavutil = None


@dataclass
class VehicleStatus:
    lat: float = 0.0; lon: float = 0.0; relative_alt_m: float = 0.0; absolute_alt_m: float = 0.0
    battery_v: float = 0.0; satellites: int = 0; hdop: float = 99.0
    ekf_ok: bool = False; armed: bool = False; mode: str = "UNKNOWN"; heading_deg: float = 0.0
    rc_channels: dict[int, int] | None = None; rc_sequence: int = 0; custom_mode: int | None = None
    last_rc_monotonic: float = 0.0; last_heartbeat_monotonic: float = 0.0
    local_north_m: float = 0.0; local_east_m: float = 0.0; local_down_m: float = 0.0
    local_position_valid: bool = False; last_local_position_monotonic: float = 0.0
    home_lat: float = 0.0; home_lon: float = 0.0; home_alt_m: float = 0.0
    last_home_position_monotonic: float = 0.0


class MavlinkVehicle:
    """MAVLink2 adapter. All commands are no-ops except logging in dry-run mode."""
    def __init__(self, endpoint: str = "udpin:0.0.0.0:14550", uart: str = "/dev/ttyTHS1", baud: int = 115200,
                 dry_run: bool = False, logger: Callable[[str], None] | None = None,
                 transport: str = "auto"):
        if transport not in {"auto", "ethernet", "uart"}:
            raise ValueError("transport must be auto, ethernet, or uart")
        self.endpoint, self.uart, self.baud, self.dry_run = endpoint, uart, baud, dry_run
        self.transport = transport
        self.connected_transport: str | None = None
        self.logger = logger or print; self.connection = None; self.status = VehicleStatus()
        self._last_velocity = (0.0, 0.0, 0.0)
        self._command_acks: dict[int, int] = {}
        self._last_stream_request_monotonic = 0.0
        self._last_log_monotonic: dict[str, float] = {}
        self._parameters: dict[str, float] = {}
        self._last_gcs_heartbeat_monotonic = 0.0
        self.message_counts: dict[str, int] = {}
        self.last_message_monotonic: dict[str, float] = {}

    @staticmethod
    def _is_autopilot_heartbeat(message) -> bool:
        if message is None:
            return False
        if mavutil is not None:
            autopilot_component = mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1
            if message.get_srcComponent() != autopilot_component or message.autopilot == mavutil.mavlink.MAV_AUTOPILOT_INVALID:
                return False
        return True

    def _apply_heartbeat(self, message) -> bool:
        if not self._is_autopilot_heartbeat(message):
            return False
        modes = self.connection.mode_mapping() or {}
        inverse_modes = {value: name for name, value in modes.items()}
        self.status.armed = bool(message.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
        self.status.custom_mode = message.custom_mode
        self.status.mode = inverse_modes.get(message.custom_mode, str(message.custom_mode))
        self.status.last_heartbeat_monotonic = time.monotonic()
        return True

    def _wait_for_autopilot_heartbeat(self, timeout: float):
        """Ignore companion/GCS heartbeats until Pixhawk's own heartbeat arrives."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            heartbeat = self.connection.recv_match(
                type="HEARTBEAT", blocking=True, timeout=min(.5, max(.01, deadline - time.monotonic()))
            )
            if self._is_autopilot_heartbeat(heartbeat):
                return heartbeat
        raise TimeoutError(f"no autopilot heartbeat received within {timeout:.1f}s")

    def connect(self, timeout: float = 12.0) -> None:
        if self.dry_run:
            now = time.monotonic()
            self.status = VehicleStatus(
                lat=12.9716, lon=77.5946, relative_alt_m=0.0,
                battery_v=22.2, satellites=12, hdop=0.8, ekf_ok=True,
                armed=False, mode="GUIDED", heading_deg=0.0,
                rc_channels={8: 1000}, rc_sequence=1, custom_mode=4,
                last_rc_monotonic=now, last_heartbeat_monotonic=now,
                local_position_valid=True, last_local_position_monotonic=now,
            )
            self.connected_transport = "dry-run"
            self.log("DRY-RUN MAVLink connection ready"); return
        if mavutil is None: raise RuntimeError("pymavlink is required; install requirements.txt")
        errors = []
        candidates = []
        if self.transport in {"auto", "ethernet"}:
            candidates.append(("ethernet", self.endpoint, {}))
        if self.transport in {"auto", "uart"}:
            candidates.append(("uart", self.uart, {"baud": self.baud}))
        for transport, device, kwargs in candidates:
            try:
                self.connection = mavutil.mavlink_connection(device, autoreconnect=True, **kwargs)
                heartbeat = self._wait_for_autopilot_heartbeat(timeout)
                # mavutil's wait_heartbeat() would bind target IDs to the
                # first UDP speaker. Set them only after selecting Pixhawk.
                self.connection.target_system = heartbeat.get_srcSystem()
                self.connection.target_component = heartbeat.get_srcComponent()
                if not self._apply_heartbeat(heartbeat):
                    raise ConnectionError("autopilot heartbeat was invalid")
                self.connected_transport = transport
                self.log(f"MAVLink heartbeat via {device}"); return
            except Exception as exc:
                errors.append(f"{device}: {exc}")
                failed_connection = self.connection
                self.connection = None
                self.connected_transport = None
                if failed_connection is not None:
                    close = getattr(failed_connection, "close", None)
                    if callable(close):
                        close()
        raise ConnectionError("; ".join(errors))

    def poll(self, duration: float = 0.05) -> VehicleStatus:
        if self.dry_run or self.connection is None: return self.status
        self.send_companion_heartbeat()
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            message = self.connection.recv_match(blocking=False)
            if message is None: time.sleep(.005); continue
            kind = message.get_type()
            # The companion can receive GCS/peripheral messages on UDP. Only
            # the selected Pixhawk's autopilot heartbeat is allowed to change
            # our authoritative flight-mode state (as in autonomous_test3.py).
            if message.get_srcSystem() != self._target()[0]:
                continue
            if kind != "HEARTBEAT":
                self.message_counts[kind] = self.message_counts.get(kind, 0) + 1
                self.last_message_monotonic[kind] = time.monotonic()
            if kind == "GLOBAL_POSITION_INT":
                self.status.lat, self.status.lon, self.status.relative_alt_m = message.lat / 1e7, message.lon / 1e7, message.relative_alt / 1000
                self.status.absolute_alt_m = message.alt / 1000
                self.status.heading_deg = message.hdg / 100 if message.hdg != 65535 else self.status.heading_deg
            elif kind == "SYS_STATUS": self.status.battery_v = message.voltage_battery / 1000
            elif kind == "GPS_RAW_INT": self.status.satellites, self.status.hdop = message.satellites_visible, message.eph / 100
            elif kind == "EKF_STATUS_REPORT":
                # MAV_EKF_STATUS_REPORT flags are positive validity flags, not
                # error bits. Require attitude, both velocity axes, vertical
                # absolute position, and at least one horizontal position source.
                flags = int(message.flags)
                required = 0b100111
                self.status.ekf_ok = (flags & required) == required and bool(flags & 0b011000)
            elif kind == "LOCAL_POSITION_NED":
                self.status.local_north_m = float(message.x)
                self.status.local_east_m = float(message.y)
                self.status.local_down_m = float(message.z)
                self.status.local_position_valid = True
                self.status.last_local_position_monotonic = time.monotonic()
            elif kind == "HOME_POSITION":
                self.status.home_lat = message.latitude / 1e7
                self.status.home_lon = message.longitude / 1e7
                self.status.home_alt_m = message.altitude / 1000
                self.status.last_home_position_monotonic = time.monotonic()
            elif kind == "STATUSTEXT":
                text = str(message.text).rstrip("\x00")
                now = time.monotonic(); key = f"pixhawk:{text}"
                if now - self._last_log_monotonic.get(key, -float("inf")) >= 1.0:
                    self._last_log_monotonic[key] = now
                    self.logger(f"[Pixhawk] {text}")
            elif kind == "HEARTBEAT":
                if self._apply_heartbeat(message):
                    self.message_counts[kind] = self.message_counts.get(kind, 0) + 1
                    self.last_message_monotonic[kind] = time.monotonic()
            elif kind == "RC_CHANNELS":
                self.status.rc_channels = {index: getattr(message, f"chan{index}_raw") for index in range(1, 19) if getattr(message, f"chan{index}_raw", 0) not in (0, 65535)}
                self.status.rc_sequence += 1; self.status.last_rc_monotonic = time.monotonic()
            elif kind == "PARAM_VALUE":
                name = str(message.param_id).rstrip("\x00")
                self._parameters[name] = float(message.param_value)
            elif kind == "COMMAND_ACK": self._command_acks[message.command] = message.result
        return self.status

    def send_companion_heartbeat(self) -> None:
        """Advertise the companion at 1 Hz so ArduPilot can detect its loss."""
        if self.dry_run or self.connection is None:
            return
        now = time.monotonic()
        if now - self._last_gcs_heartbeat_monotonic < 1.0:
            return
        self.connection.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_GCS,
            mavutil.mavlink.MAV_AUTOPILOT_INVALID,
            0, 0, mavutil.mavlink.MAV_STATE_ACTIVE,
        )
        self._last_gcs_heartbeat_monotonic = now

    def verify_flight_failsafes(self) -> None:
        """Fail preflight when companion/battery loss has no FC-level action."""
        if self.dry_run:
            return
        required = ("FS_GCS_ENABLE", "BATT_FS_LOW_ACT", "BATT_FS_CRT_ACT")
        values = {name: self.read_parameter(name) for name in required}
        missing = [name for name, value in values.items() if value is None]
        if missing:
            raise RuntimeError(f"could not verify failsafe parameters: {', '.join(missing)}")
        disabled = [name for name, value in values.items() if int(round(value)) == 0]
        if disabled:
            raise RuntimeError(
                "flight-controller failsafe action is disabled: " + ", ".join(disabled) +
                "; configure non-zero actions in Mission Planner before flight"
            )

    def log(self, text: str, severity: int = 6, *, throttle_s: float = 0.0, key: str | None = None) -> None:
        """Log locally and through STATUSTEXT, optionally rate-limited by key."""
        if throttle_s > 0:
            now = time.monotonic(); throttle_key = key or text
            if now - self._last_log_monotonic.get(throttle_key, -float("inf")) < throttle_s:
                return
            self._last_log_monotonic[throttle_key] = now
        # Keep the complete event in the Jetson log/dashboard. MAVLink
        # STATUSTEXT is the only destination that needs the short payload.
        self.logger(text)
        if self.connection is not None and not self.dry_run:
            self.connection.mav.statustext_send(severity, text[:50].encode("ascii", "replace"))

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
        """Request only the 10 Hz position/RC streams used by the safety guard."""
        if self.dry_run: return
        if message_ids is None:
            message_ids = (
                mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT,
                mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED,
                mavutil.mavlink.MAVLINK_MSG_ID_RC_CHANNELS,
            )
        for message_id in message_ids:
            # Stream maintenance is transport housekeeping, not operator
            # telemetry. Send it directly so retries do not emit STATUSTEXT.
            values = (message_id, interval_us, 0, 0, 0, 0, 0)
            self.connection.mav.command_long_send(
                *self._target(), mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0, *values
            )
        self._last_stream_request_monotonic = time.monotonic()

    def maintain_rc_telemetry(self, stale_after_s: float = 2.0, retry_interval_s: float = 1.0) -> None:
        """Re-request RC/position streams when Channel 8 telemetry becomes stale."""
        if self.dry_run:
            return
        now = time.monotonic()
        rc_stale = now - self.status.last_rc_monotonic > stale_after_s
        if rc_stale and now - self._last_stream_request_monotonic >= retry_interval_s:
            self.log("RC telemetry stale; re-requesting stream", 4, throttle_s=5.0, key="rc-stream-stale")
            self.request_message_intervals()

    def capture_rc_channel_pwm(self, channel: int, timeout_s: float = 3.0, samples: int = 5,
                               cancel: Callable[[], bool] | None = None) -> int:
        """Capture a stable, current RC input PWM before commanding an actuator."""
        if not 1 <= channel <= 18: raise ValueError("RC channel must be between 1 and 18")
        if samples < 1: raise ValueError("samples must be positive")
        if self.dry_run:
            return int((self.status.rc_channels or {}).get(channel, 1500))
        # Five fresh RC samples at 5 Hz are enough to establish neutral. Do
        # not request unrelated position streams during a bench actuator test.
        self.request_message_intervals(
            (mavutil.mavlink.MAVLINK_MSG_ID_RC_CHANNELS,), interval_us=200_000
        )
        deadline = time.monotonic() + timeout_s; observed = []; last_sequence = -1
        while time.monotonic() < deadline:
            self.poll(.1)
            if cancel and cancel(): raise RuntimeError("RC neutral capture cancelled")
            if self.status.rc_sequence == last_sequence:
                continue
            last_sequence = self.status.rc_sequence
            pwm = (self.status.rc_channels or {}).get(channel)
            if pwm is not None and 800 <= pwm <= 2200:
                observed.append(pwm)
                if len(observed) >= samples:
                    window = observed[-samples:]
                    if max(window) - min(window) <= 8:
                        return int(round(statistics.median(window)))
        raise TimeoutError(f"no stable RC{channel} input received for neutral capture")

    @staticmethod
    def _ack_rejected(result: int | None) -> bool:
        if result is None:
            return False
        return result in {
            mavutil.mavlink.MAV_RESULT_TEMPORARILY_REJECTED,
            mavutil.mavlink.MAV_RESULT_DENIED,
            mavutil.mavlink.MAV_RESULT_UNSUPPORTED,
            mavutil.mavlink.MAV_RESULT_FAILED,
            getattr(mavutil.mavlink, "MAV_RESULT_CANCELLED", -1),
        }

    def _wait_for_ack(self, command: int, timeout_s: float, cancel: Callable[[], bool] | None = None) -> bool:
        if self.dry_run: return True
        deadline = time.monotonic() + timeout_s
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
                if cancel and cancel(): return False
                if self.status.mode == mode: self.log(f"Mode confirmed {mode}"); return True
                if self._ack_rejected(self._command_acks.pop(mavutil.mavlink.MAV_CMD_DO_SET_MODE, None)): return False
        self.log(f"Mode timeout {mode}", 3); return False

    def arm(self) -> None:
        self.log("ARM");
        if not self.dry_run: self.connection.mav.command_long_send(*self._target(), mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
        self.status.armed = True

    def arm_confirmed(self, arm: bool = True, timeout_s: float = 6.0, cancel: Callable[[], bool] | None = None) -> bool:
        if self.dry_run: self.status.armed = arm; self.log("DRY ARM" if arm else "DRY DISARM"); return True
        if cancel and cancel(): return False
        value = 1 if arm else 0; command = mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM; self._command_acks.pop(command, None)
        self.log("ARM command" if arm else "DISARM command")
        self.connection.mav.command_long_send(*self._target(), command, 0, value, 0, 0, 0, 0, 0, 0)
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            self.poll(.05)
            if cancel and cancel(): return False
            if self.status.armed == arm: self.log("Arming confirmed" if arm else "Disarm confirmed"); return True
            if self._ack_rejected(self._command_acks.pop(command, None)): return False
        self.log("Arming timeout" if arm else "Disarm timeout", 3)
        return False

    def takeoff(self, altitude_m: float) -> None:
        self.log(f"TAKEOFF {altitude_m:.1f}m")
        if not self.dry_run: self.connection.mav.command_long_send(*self._target(), mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, altitude_m)
        self.status.relative_alt_m = altitude_m

    def takeoff_confirmed(self, altitude_m: float, timeout_s: float = 3.0, cancel: Callable[[], bool] | None = None) -> bool:
        if self.dry_run: self.takeoff(altitude_m); return True
        if cancel and cancel(): return False
        command = mavutil.mavlink.MAV_CMD_NAV_TAKEOFF; self._command_acks.pop(command, None)
        self.log(f"TAKEOFF command {altitude_m:.1f}m")
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
            mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED, 0b0000111111000111, 0,0,0,vx,vy,vz,0,0,0,0,0)

    def resend_last_velocity(self) -> None:
        """Refresh the active Guided velocity setpoint without generating a log."""
        vx, vy, vz = self._last_velocity
        if self.dry_run or self.connection is None:
            return
        self.connection.mav.set_position_target_local_ned_send(
            int(time.time() * 1000) & 0xffffffff, *self._target(),
            mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED, 0b0000111111000111,
            0, 0, 0, vx, vy, vz, 0, 0, 0, 0, 0,
        )

    def read_parameter(self, name: str, timeout_s: float = 1.5) -> float | None:
        """Read one flight-controller parameter, returning None on timeout."""
        if self.dry_run:
            return None
        key = name[:16]
        self._parameters.pop(key, None)
        self.connection.mav.param_request_read_send(*self._target(), key.encode("ascii"), -1)
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            self.poll(.05)
            if key in self._parameters:
                return self._parameters[key]
        return None

    def command_long(self, command: int, *params: float) -> None:
        values = list(params) + [0.0] * (7-len(params))
        self.log(f"COMMAND {command}")
        if not self.dry_run: self.connection.mav.command_long_send(*self._target(), command, 0, *values[:7])

    def command_long_confirmed(self, command: int, *params: float, timeout_s: float = 2.0,
                               cancel: Callable[[], bool] | None = None) -> bool:
        """Send COMMAND_LONG and require an ACCEPTED acknowledgement."""
        if self.dry_run:
            self.command_long(command, *params)
            return True
        self._command_acks.pop(command, None)
        self.command_long(command, *params)
        return self._wait_for_ack(command, timeout_s, cancel)

    def set_home_confirmed(self, lat: float, lon: float, absolute_alt_m: float, timeout_s: float = 5.0) -> bool:
        """Write and read back the Copter home/takeoff point using WGS84 altitude."""
        if self.dry_run:
            self.status.home_lat, self.status.home_lon, self.status.home_alt_m = lat, lon, absolute_alt_m
            self.log(f"DRY HOME {lat:.7f},{lon:.7f}")
            return True
        command = mavutil.mavlink.MAV_CMD_DO_SET_HOME
        self._command_acks.pop(command, None)
        self.connection.mav.command_int_send(
            *self._target(), mavutil.mavlink.MAV_FRAME_GLOBAL, command,
            0, 0, 0, 0, 0, 0, int(round(lat * 1e7)), int(round(lon * 1e7)), absolute_alt_m,
        )
        if not self._wait_for_ack(command, timeout_s):
            return False
        self.request_message_intervals((mavutil.mavlink.MAVLINK_MSG_ID_HOME_POSITION,), interval_us=500_000)
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            self.poll(.1)
            north = (self.status.home_lat - lat) * 111_319.49
            east = (self.status.home_lon - lon) * 111_319.49 * math.cos(math.radians(lat))
            if self.status.last_home_position_monotonic and math.hypot(north, east) <= 1.0:
                self.log(f"Home verified {lat:.7f},{lon:.7f}")
                return True
        self.log("Home write acknowledged but read-back did not match", 3)
        return False

    def upload_fence_polygons(self, outer: list[tuple[float, float]],
                              exclusions: list[list[tuple[float, float]]], timeout_s: float = 12.0) -> None:
        """Upload MAVLink2 polygon fences and verify every stored item by read-back."""
        if len(outer) < 3 or any(len(polygon) < 3 for polygon in exclusions):
            raise ValueError("every fence polygon must contain at least three vertices")
        items: list[tuple[int, int, float, float]] = []
        for lat, lon in outer:
            items.append((5001, len(outer), lat, lon))
        for polygon in exclusions:
            for lat, lon in polygon:
                items.append((5002, len(polygon), lat, lon))
        if self.dry_run:
            self.log(f"DRY FENCE upload {len(items)} items")
            return
        mission_type = mavutil.mavlink.MAV_MISSION_TYPE_FENCE
        target = self._target()
        self.connection.mav.mission_count_send(*target, len(items), mission_type)
        deadline = time.monotonic() + timeout_s
        sent: set[int] = set()
        message = None
        while time.monotonic() < deadline:
            self.send_companion_heartbeat()
            message = self.connection.recv_match(
                type=["MISSION_REQUEST_INT", "MISSION_ACK"], blocking=True, timeout=.5
            )
            if message is None or message.get_srcSystem() != target[0]:
                continue
            if message.get_type() == "MISSION_ACK":
                if int(message.type) != mavutil.mavlink.MAV_MISSION_ACCEPTED:
                    raise RuntimeError(f"Pixhawk rejected fence upload with MAV_MISSION_RESULT={message.type}")
                if len(sent) != len(items):
                    raise RuntimeError(f"Pixhawk acknowledged fence after requesting only {len(sent)}/{len(items)} items")
                break
            sequence = int(message.seq)
            if not 0 <= sequence < len(items):
                raise RuntimeError(f"Pixhawk requested invalid fence sequence {sequence}")
            command, vertex_count, lat, lon = items[sequence]
            self.connection.mav.mission_item_int_send(
                *target, sequence, mavutil.mavlink.MAV_FRAME_GLOBAL_INT, command,
                0, 0, float(vertex_count), 0, 0, 0,
                int(round(lat * 1e7)), int(round(lon * 1e7)), 0.0, mission_type,
            )
            sent.add(sequence)
        else:
            raise TimeoutError("timed out waiting for Pixhawk fence upload acknowledgement")

        self.connection.mav.mission_request_list_send(*target, mission_type)
        count_message = self.connection.recv_match(type="MISSION_COUNT", blocking=True, timeout=timeout_s)
        if count_message is None or int(count_message.count) != len(items):
            actual = None if count_message is None else int(count_message.count)
            raise RuntimeError(f"fence read-back count {actual}, expected {len(items)}")
        for sequence, expected in enumerate(items):
            self.connection.mav.mission_request_int_send(*target, sequence, mission_type)
            received = self.connection.recv_match(type="MISSION_ITEM_INT", blocking=True, timeout=timeout_s)
            if received is None or int(received.seq) != sequence:
                raise TimeoutError(f"no read-back for fence item {sequence}")
            command, vertex_count, lat, lon = expected
            if (int(received.command) != command or int(round(received.param1)) != vertex_count or
                    int(received.x) != int(round(lat * 1e7)) or int(received.y) != int(round(lon * 1e7))):
                raise RuntimeError(f"fence read-back mismatch at item {sequence}")
        self.log(f"Pixhawk fence verified: {len(items)} items")

    @staticmethod
    def decode_fence_items(items) -> list[list[tuple[float, float]]]:
        """Decode ordered MAVLink polygon fence items into one inclusion plus exclusions."""
        polygons: list[tuple[int, list[tuple[float, float]]]] = []
        index = 0
        while index < len(items):
            item = items[index]
            command = int(item.command)
            # Mission Planner may prepend one optional fence return point.  It
            # is metadata, not a polygon boundary, so it must not become a
            # false unsupported-command failure during the read-only audit.
            if command == 5000:
                index += 1
                continue
            if command not in (5001, 5002):
                raise RuntimeError(f"unsupported Mission Planner fence command {command}; use polygon fences")
            count = int(round(float(item.param1)))
            if count < 3 or index + count > len(items):
                raise RuntimeError(f"invalid polygon vertex count {count} at fence item {index}")
            vertices = []
            for offset in range(count):
                vertex = items[index + offset]
                if int(vertex.command) != command or int(round(float(vertex.param1))) != count:
                    raise RuntimeError(f"inconsistent polygon at fence item {index + offset}")
                vertices.append((int(vertex.x) / 1e7, int(vertex.y) / 1e7))
            polygons.append((command, vertices))
            index += count
        inclusions = [vertices for command, vertices in polygons if command == 5001]
        if len(inclusions) != 1:
            raise RuntimeError(f"expected exactly one polygon inclusion fence, found {len(inclusions)}")
        return [inclusions[0], *[vertices for command, vertices in polygons if command == 5002]]

    def download_fence_polygons(self, timeout_s: float = 12.0) -> list[list[tuple[float, float]]]:
        """Download Mission Planner polygon fences without changing FC state."""
        if self.dry_run:
            polygons = getattr(self, "_dry_fence_polygons", None)
            if polygons is None:
                raise RuntimeError("dry-run has no FC-managed fence snapshot")
            return [[tuple(vertex) for vertex in polygon] for polygon in polygons]
        mission_type = mavutil.mavlink.MAV_MISSION_TYPE_FENCE
        target = self._target()

        def receive(kind, deadline):
            while time.monotonic() < deadline:
                self.send_companion_heartbeat()
                message = self.connection.recv_match(type=kind, blocking=True, timeout=.5)
                if message is not None and message.get_srcSystem() == target[0]:
                    return message
            return None

        self.connection.mav.mission_request_list_send(*target, mission_type)
        count_message = receive("MISSION_COUNT", time.monotonic() + timeout_s)
        if count_message is None:
            raise TimeoutError("no Mission Planner fence count received from Pixhawk")
        count = int(count_message.count)
        if count < 3:
            raise RuntimeError(f"Pixhawk polygon fence has only {count} stored items")
        items = []
        for sequence in range(count):
            self.connection.mav.mission_request_int_send(*target, sequence, mission_type)
            item = receive("MISSION_ITEM_INT", time.monotonic() + timeout_s)
            if item is None or int(item.seq) != sequence:
                raise TimeoutError(f"no read-back for Pixhawk fence item {sequence}")
            items.append(item)
        self.connection.mav.mission_ack_send(
            *target, mavutil.mavlink.MAV_MISSION_ACCEPTED, mission_type
        )
        polygons = self.decode_fence_items(items)
        self.log(
            f"Mission Planner fence downloaded: {len(polygons[0])} inclusion vertices, "
            f"{sum(len(polygon) for polygon in polygons[1:])} exclusion vertices"
        )
        return polygons

    def upload_mission_items(self, waypoints, timeout_s: float = 15.0) -> None:
        """Upload the main AUTO mission and verify every item by MAVLink2 read-back."""
        items = list(waypoints)
        if not items:
            raise ValueError("mission upload requires at least one waypoint")
        if [int(item.index) for item in items] != list(range(len(items))):
            raise ValueError("mission waypoint indices must be sequential from zero")
        if self.dry_run:
            self.log(f"DRY MISSION upload/read-back {len(items)} items")
            return
        mission_type = mavutil.mavlink.MAV_MISSION_TYPE_MISSION
        target = self._target()

        def receive(kinds, deadline):
            while time.monotonic() < deadline:
                self.send_companion_heartbeat()
                message = self.connection.recv_match(type=kinds, blocking=True, timeout=.5)
                if message is not None and message.get_srcSystem() == target[0]:
                    return message
            return None

        self.connection.mav.mission_count_send(*target, len(items), mission_type)
        deadline = time.monotonic() + timeout_s
        sent: set[int] = set()
        while time.monotonic() < deadline:
            message = receive(["MISSION_REQUEST_INT", "MISSION_ACK"], deadline)
            if message is None:
                break
            if message.get_type() == "MISSION_ACK":
                if int(message.type) != mavutil.mavlink.MAV_MISSION_ACCEPTED:
                    raise RuntimeError(f"Pixhawk rejected mission upload with MAV_MISSION_RESULT={message.type}")
                if len(sent) != len(items):
                    raise RuntimeError(f"Pixhawk acknowledged mission after requesting only {len(sent)}/{len(items)} items")
                break
            sequence = int(message.seq)
            if not 0 <= sequence < len(items):
                raise RuntimeError(f"Pixhawk requested invalid mission sequence {sequence}")
            item = items[sequence]
            self.connection.mav.mission_item_int_send(
                *target, sequence, int(item.frame), int(item.command),
                int(bool(item.current)), int(bool(item.autocontinue)),
                *[float(value) for value in item.params],
                int(round(item.lat * 1e7)), int(round(item.lon * 1e7)), float(item.alt), mission_type,
            )
            sent.add(sequence)
        else:
            message = None
        if message is None or message.get_type() != "MISSION_ACK":
            raise TimeoutError("timed out waiting for Pixhawk mission upload acknowledgement")

        self.connection.mav.mission_request_list_send(*target, mission_type)
        count_message = receive("MISSION_COUNT", time.monotonic() + timeout_s)
        if count_message is None or int(count_message.count) != len(items):
            actual = None if count_message is None else int(count_message.count)
            raise RuntimeError(f"mission read-back count {actual}, expected {len(items)}")
        for sequence, expected in enumerate(items):
            self.connection.mav.mission_request_int_send(*target, sequence, mission_type)
            received = receive("MISSION_ITEM_INT", time.monotonic() + timeout_s)
            if received is None or int(received.seq) != sequence:
                raise TimeoutError(f"no read-back for mission item {sequence}")
            received_params = (received.param1, received.param2, received.param3, received.param4)
            if (
                int(received.frame) != int(expected.frame)
                or int(received.command) != int(expected.command)
                or int(received.x) != int(round(expected.lat * 1e7))
                or int(received.y) != int(round(expected.lon * 1e7))
                or not math.isclose(float(received.z), float(expected.alt), abs_tol=.05)
                or any(not math.isclose(float(actual), float(wanted), abs_tol=1e-4)
                       for actual, wanted in zip(received_params, expected.params))
            ):
                raise RuntimeError(f"mission read-back mismatch at item {sequence}")
        self.log(f"Pixhawk mission verified: {len(items)} items")

    def set_parameter(self, name: str, value: float) -> None:
        self.log(f"PARAM {name}={value:g}")
        if not self.dry_run:
            self.connection.mav.param_set_send(*self._target(), name.encode("ascii")[:16], value, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)

    def set_parameter_confirmed(self, name: str, value: float, timeout_s: float = 3.0) -> bool:
        """Set one FC parameter and require matching PARAM_VALUE read-back."""
        key = name[:16]
        if self.dry_run:
            self._parameters[key] = float(value)
            self.log(f"DRY PARAM verified {key}={value:g}")
            return True
        self._parameters.pop(key, None)
        self.set_parameter(key, value)
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            self.poll(.05)
            actual = self._parameters.get(key)
            if actual is not None and math.isclose(actual, float(value), rel_tol=0.0, abs_tol=1e-3):
                self.log(f"Parameter verified {key}={actual:g}")
                return True
        self.log(f"Parameter read-back failed {key}={value:g}", 3)
        return False

    def send_fence_point(self, index: int, total: int, lat: float, lon: float) -> None:
        self.log(f"FENCE {index+1}/{total}")
        if not self.dry_run: self.connection.mav.fence_point_send(*self._target(), index, total, lat, lon)

    def rc_override(self, pwm: int | None, channel: int = 12) -> None:
        if not 1 <= channel <= 18: raise ValueError("RC override channel must be between 1 and 18")
        self.log(f"RC{channel} {'release' if pwm is None else pwm}")
        if self.dry_run: return
        # MAVLink uses different release sentinels for extension channels:
        # CH1-8 release with 0; CH9-18 release with UINT16_MAX-1.
        release_value = 0 if channel <= 8 else 65534
        channels = [65535] * 18; channels[channel - 1] = release_value if pwm is None else pwm
        self.connection.mav.rc_channels_override_send(*self._target(), *channels)

    def land(self) -> None: self.set_mode("LAND")
    def rtl(self) -> None: self.set_mode("RTL")
