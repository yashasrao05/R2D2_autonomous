from __future__ import annotations
import time
from .mavlink_interface import MavlinkVehicle


class Servo7Actuator:
    def __init__(self, vehicle: MavlinkVehicle, method: str = "set_servo", servo_number: int = 7, rc_channel: int = 12,
                 neutral_from_rc_channel: int | None = 12,
                 lower_pwm: int = 2000, neutral_pwm: int = 1500, retract_pwm: int = 1100,
                 lower_seconds: float = 65, retract_seconds: float = 8, settle_seconds: float = 1):
        if method not in {"rc_override", "set_servo"}: raise ValueError("payload method must be rc_override or set_servo")
        if not 1 <= servo_number <= 16: raise ValueError("servo_number must be between 1 and 16")
        if not 1 <= rc_channel <= 18: raise ValueError("rc_channel must be between 1 and 18")
        if neutral_from_rc_channel is not None and not 1 <= neutral_from_rc_channel <= 18:
            raise ValueError("neutral_from_rc_channel must be between 1 and 18")
        for name, pwm in (("lower_pwm",lower_pwm),("neutral_pwm",neutral_pwm),("retract_pwm",retract_pwm)):
            if not 800 <= pwm <= 2200: raise ValueError(f"{name} must be between 800 and 2200")
        self.vehicle, self.method, self.servo_number, self.rc_channel = vehicle, method, servo_number, rc_channel
        self.neutral_from_rc_channel = neutral_from_rc_channel
        self.lower_pwm, self.neutral_pwm, self.retract_pwm = lower_pwm, neutral_pwm, retract_pwm
        self.lower_seconds, self.retract_seconds, self.settle_seconds = lower_seconds, retract_seconds, settle_seconds
    def _set(self, pwm: int | None, guard=None) -> None:
        if self.method == "rc_override": self.vehicle.rc_override(pwm,self.rc_channel)
        elif pwm is not None:
            cancel = (lambda: guard()) if guard else None
            if not self.vehicle.command_long_confirmed(183, self.servo_number, pwm, cancel=cancel):
                raise RuntimeError(f"SERVO{self.servo_number} PWM {pwm} command was rejected or timed out")
    def _neutralize(self, pwm: int | None = None) -> None:
        if self.method == "rc_override":
            self._set(None)
        else:
            self._set(self.neutral_pwm if pwm is None else pwm)
    def _capture_neutral(self, guard=None) -> None:
        if self.method != "set_servo" or self.neutral_from_rc_channel is None:
            return
        cancel = (lambda: guard()) if guard else None
        self.neutral_pwm = self.vehicle.capture_rc_channel_pwm(
            self.neutral_from_rc_channel, cancel=cancel
        )
        self.vehicle.log(f"Captured RC{self.neutral_from_rc_channel} neutral PWM {self.neutral_pwm}")
    def _wait(self, duration_s: float, guard=None) -> None:
        deadline=time.monotonic()+duration_s
        while time.monotonic()<deadline:
            if guard: guard()
            time.sleep(min(.1, max(0, deadline-time.monotonic())))
    def release_cycle(self, guard=None) -> None:
        """Run the winch sequence, neutralising immediately if the pilot override guard raises."""
        self._capture_neutral(guard)
        self.vehicle.log(f"Payload lower: SERVO{self.servo_number} PWM {self.lower_pwm}")
        try:
            self._set(self.lower_pwm,guard)
            self._wait(self.lower_seconds,guard); self._neutralize(); self._wait(self.settle_seconds,guard)
            self.vehicle.log("Payload retract/lock"); self._set(self.retract_pwm,guard); self._wait(self.retract_seconds,guard)
            self._neutralize(); self._wait(self.settle_seconds,guard)
        except BaseException:
            try:
                self._neutralize()
            except Exception as exc:
                self.vehicle.log(f"SERVO{self.servo_number} neutral command failed: {exc}", 2)
            raise
        self.vehicle.log("Payload cycle complete")

    def test_drop(self, pwm: int = 2000, duration_s: float = 65.0, neutral_pwm: int | None = None) -> None:
        """Non-arming bench pulse, then fixed neutral or release back to RC input."""
        if not 800 <= pwm <= 2200: raise ValueError("test PWM must be between 800 and 2200")
        if not 0 < duration_s <= 120: raise ValueError("test duration must be greater than 0 and at most 120 seconds")
        self._capture_neutral()
        neutral = self.neutral_pwm if neutral_pwm is None else neutral_pwm
        if not 800 <= neutral <= 2200: raise ValueError("neutral PWM must be between 800 and 2200")
        self.vehicle.log(f"BENCH SERVO{self.servo_number}: PWM {pwm} for {duration_s:.1f}s")
        try:
            self._set(pwm)
            self._wait(duration_s)
        finally:
            self._neutralize(neutral)
            if self.method == "rc_override":
                self.vehicle.log(f"BENCH RC{self.rc_channel}: released to TX neutral")
            else:
                self.vehicle.log(f"BENCH SERVO{self.servo_number}: neutral PWM {neutral}")
