from __future__ import annotations

import json
import threading
import time
from collections import deque
from datetime import datetime, timezone
from pathlib import Path
from typing import Any


TERMINAL_STATES = {"COMPLETE", "FAULT", "PILOT_CONTROL"}


class MissionRuntime:
    """Thread-safe operator status plus a persistent, transition-only event log."""

    def __init__(self, log_directory: str | Path = "storage/logs", vehicle=None, guard=None):
        started = datetime.now(timezone.utc)
        self._lock = threading.RLock()
        self._events: deque[dict[str, Any]] = deque(maxlen=300)
        self._vehicle = vehicle
        self._guard = guard
        self._sequence = 0
        directory = Path(log_directory)
        directory.mkdir(parents=True, exist_ok=True)
        self.log_path = directory / f"mission-{started.strftime('%Y%m%dT%H%M%SZ')}.jsonl"
        self._status: dict[str, Any] = {
            "service_state": "BOOT",
            "mission_phase": None,
            "profile": None,
            "profile_locked": False,
            "readiness_passed": False,
            "readiness_checks": [],
            "blockers": ["boot checks have not completed"],
            "fence_sync": None,
            "last_error": None,
            "started_at": started.isoformat(),
            "updated_at": started.isoformat(),
        }
        self.log("Competition supervisor started", source="supervisor")

    def bind(self, *, vehicle=None, guard=None) -> None:
        with self._lock:
            if vehicle is not None:
                self._vehicle = vehicle
            if guard is not None:
                self._guard = guard

    def update(self, **values: Any) -> None:
        with self._lock:
            changed = {
                key: value for key, value in values.items()
                if self._status.get(key) != value
            }
            if not changed:
                return
            self._status.update(changed)
            self._status["updated_at"] = datetime.now(timezone.utc).isoformat()

    def transition(self, state: str, message: str | None = None, **values: Any) -> None:
        with self._lock:
            previous = self._status.get("service_state")
        self.update(service_state=state, **values)
        if previous != state:
            self.log(message or f"State {previous} -> {state}", source="supervisor")

    def log(self, message: str, level: str = "INFO", source: str = "companion") -> None:
        event = {
            "sequence": 0,
            "time": datetime.now(timezone.utc).isoformat(),
            "monotonic_s": round(time.monotonic(), 3),
            "level": str(level).upper(),
            "source": source,
            "message": str(message),
        }
        with self._lock:
            self._sequence += 1
            event["sequence"] = self._sequence
            self._events.append(event)
            with self.log_path.open("a", encoding="utf-8") as handle:
                handle.write(json.dumps(event, ensure_ascii=True) + "\n")
        print(message, flush=True)

    def events(self, after: int = 0) -> list[dict[str, Any]]:
        with self._lock:
            return [dict(item) for item in self._events if item["sequence"] > after]

    def _telemetry(self) -> dict[str, Any]:
        if self._vehicle is None:
            return {
                "connected": False, "transport": None, "armed": False,
                "mode": "UNKNOWN", "battery_v": 0.0, "satellites": 0,
                "hdop": 99.0, "ekf_ok": False, "latitude": 0.0,
                "longitude": 0.0, "relative_altitude_m": 0.0,
            }
        status = self._vehicle.status
        heartbeat_age = None
        if status.last_heartbeat_monotonic:
            heartbeat_age = round(max(0.0, time.monotonic() - status.last_heartbeat_monotonic), 2)
        return {
            "connected": self._vehicle.connected_transport is not None,
            "transport": self._vehicle.connected_transport,
            "armed": bool(status.armed),
            "mode": status.mode,
            "battery_v": round(float(status.battery_v), 2),
            "satellites": int(status.satellites),
            "hdop": round(float(status.hdop), 2),
            "ekf_ok": bool(status.ekf_ok),
            "latitude": round(float(status.lat), 7),
            "longitude": round(float(status.lon), 7),
            "relative_altitude_m": round(float(status.relative_alt_m), 2),
            "heartbeat_age_s": heartbeat_age,
        }

    def _operator_switch(self) -> dict[str, Any]:
        if self._guard is None:
            return {"state": "UNKNOWN", "pwm": None, "rc_fresh": False}
        return {
            "state": self._guard.state,
            "pwm": self._guard.pwm,
            "rc_fresh": self._guard.rc_fresh(),
        }

    def snapshot(self) -> dict[str, Any]:
        with self._lock:
            result = dict(self._status)
            result["log_path"] = str(self.log_path)
            result["last_sequence"] = self._sequence
        result["telemetry"] = self._telemetry()
        result["tx15"] = self._operator_switch()
        state = result["service_state"]
        if state == "IDLE_SELECT_PROFILE":
            result["operator_action"] = "Select segment1 or segment2 while TX15 SC is DOWN."
        elif state == "READY_WAITING_TX15_UP":
            result["operator_action"] = "Final area-clear check, then flip TX15 SC UP to authorize autonomy."
        elif state == "MISSION_ACTIVE":
            result["operator_action"] = "Monitor only. Flip TX15 SC DOWN to take control."
        elif state in TERMINAL_STATES:
            result["operator_action"] = "Run is locked. Keep SC DOWN; inspect logs and reboot/reset deliberately."
        else:
            result["operator_action"] = "Wait for the current check to finish."
        return result

    def can_select_profile(self) -> tuple[bool, str]:
        snapshot = self.snapshot()
        if snapshot["service_state"] != "IDLE_SELECT_PROFILE":
            return False, f"service is {snapshot['service_state']}, not IDLE_SELECT_PROFILE"
        if snapshot["profile_locked"]:
            return False, "mission profile is already locked for this run"
        if not snapshot["readiness_passed"]:
            return False, "competition readiness checks have not passed"
        if snapshot["telemetry"]["armed"]:
            return False, "Pixhawk is armed"
        if not (snapshot["tx15"]["rc_fresh"] and snapshot["tx15"]["state"] == "DOWN"):
            return False, "TX15 SC must be DOWN with fresh RC telemetry"
        return True, "ready"
