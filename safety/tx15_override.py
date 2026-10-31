from __future__ import annotations

import time
from dataclasses import dataclass


class PilotOverride(RuntimeError):
    """Raised when the pilot has explicitly taken back control."""


@dataclass(frozen=True)
class Tx15Config:
    channel: int = 8
    up_pwm: int = 1700
    down_pwm: int = 1300
    debounce_count: int = 3
    rc_timeout_s: float = 2.0
    heartbeat_timeout_s: float = 3.0
    manual_mode: str = "ALT_HOLD"
    backup_mode: str = "STABILIZE"


class Tx15SafetyGuard:
    """Debounced TX15 SC switch interlock and immediate manual-control fallback."""
    def __init__(self, vehicle, config: Tx15Config = Tx15Config()):
        self.vehicle, self.config = vehicle, config
        self.state = "UNKNOWN"; self._candidate = "UNKNOWN"; self._count = 0; self._last_sequence = -1

    def observe(self) -> str:
        status = self.vehicle.status
        if status.rc_sequence == self._last_sequence: return self.state
        self._last_sequence = status.rc_sequence
        pwm = (status.rc_channels or {}).get(self.config.channel)
        if pwm is None or not 800 <= pwm <= 2200: return self.state
        instant = "UP" if pwm > self.config.up_pwm else "DOWN"
        if instant == self._candidate: self._count += 1
        else: self._candidate, self._count = instant, 1
        if self._count >= self.config.debounce_count: self.state = instant
        return self.state

    def poll(self, duration_s: float = 0.05) -> str:
        """Match autonomous_test3.py: read telemetry, re-request stale RC, debounce SC."""
        self.vehicle.poll(duration_s)
        self.vehicle.maintain_rc_telemetry(self.config.rc_timeout_s)
        return self.observe()

    @property
    def pwm(self) -> int | None: return (self.vehicle.status.rc_channels or {}).get(self.config.channel)
    def rc_fresh(self) -> bool: return time.monotonic() - self.vehicle.status.last_rc_monotonic <= self.config.rc_timeout_s
    def heartbeat_fresh(self) -> bool: return time.monotonic() - self.vehicle.status.last_heartbeat_monotonic <= self.config.heartbeat_timeout_s
    def is_down(self) -> bool: return self.rc_fresh() and self.state == "DOWN" and self.pwm is not None and self.pwm < self.config.down_pwm
    def is_up(self) -> bool: return self.rc_fresh() and self.state == "UP" and self.pwm is not None and self.pwm > self.config.up_pwm
    def manual_requested(self) -> bool: return not self.is_up()

    def require_initial_manual(self, timeout_s: float | None = None) -> None:
        deadline = None if timeout_s is None else time.monotonic() + timeout_s
        self.vehicle.log(f"TX15 CH{self.config.channel}: set SC DOWN to arm mission")
        while not self.is_down():
            self.poll(.05)
            if deadline and time.monotonic() >= deadline: raise TimeoutError("TX15 SC switch did not reach manual/DOWN")
            time.sleep(.05)
        self.vehicle.log(f"TX15 CH{self.config.channel} manual interlock verified ({self.pwm})")

    def wait_for_autonomy_authorization(self, timeout_s: float | None = None) -> None:
        deadline = None if timeout_s is None else time.monotonic() + timeout_s
        self.vehicle.log(f"TX15 CH{self.config.channel}: flip SC UP to start autonomous mission")
        while not self.is_up():
            self.poll(.05)
            if deadline and time.monotonic() >= deadline: raise TimeoutError("TX15 SC switch did not authorize autonomy")
            time.sleep(.05)

    def yield_to_pilot(self, reason: str) -> None:
        self.vehicle.log(f"PILOT OVERRIDE: {reason}", 2)
        if self.vehicle.status.mode != self.config.manual_mode and not self.vehicle.set_mode_confirmed(self.config.manual_mode, 1.5):
            self.vehicle.set_mode_confirmed(self.config.backup_mode, 1.5)

    def assert_autonomy(self, expected_modes: tuple[str, ...] | None = ("GUIDED",)) -> None:
        self.poll(.01)
        if not self.heartbeat_fresh(): self.yield_to_pilot("Pixhawk heartbeat stale"); raise PilotOverride("Pixhawk heartbeat stale")
        if self.manual_requested(): self.yield_to_pilot(f"TX15 SC manual/stale ({self.pwm})"); raise PilotOverride("TX15 manual override")
        if expected_modes is not None and self.vehicle.status.mode not in expected_modes:
            self.vehicle.log(f"Autonomy yielded: mode is {self.vehicle.status.mode}", 4)
            raise PilotOverride(f"mode changed to {self.vehicle.status.mode}")
