from __future__ import annotations
import time
from .mavlink_interface import MavlinkVehicle


class Servo7Actuator:
    def __init__(self, vehicle: MavlinkVehicle, method: str = "rc_override", lower_seconds: float = 8, retract_seconds: float = 8, settle_seconds: float = 1):
        if method not in {"rc_override", "set_servo"}: raise ValueError("payload method must be rc_override or set_servo")
        self.vehicle, self.method = vehicle, method; self.lower_seconds, self.retract_seconds, self.settle_seconds = lower_seconds, retract_seconds, settle_seconds
    def _set(self, pwm: int | None) -> None:
        if self.method == "rc_override": self.vehicle.rc_override(pwm)
        elif pwm is not None: self.vehicle.command_long(183, 7, pwm) # MAV_CMD_DO_SET_SERVO
    def _wait(self, duration_s: float, guard=None) -> None:
        deadline=time.monotonic()+duration_s
        while time.monotonic()<deadline:
            if guard: guard()
            time.sleep(min(.1, max(0, deadline-time.monotonic())))
    def release_cycle(self, guard=None) -> None:
        """Run the winch sequence, neutralising immediately if the pilot override guard raises."""
        self.vehicle.log("Payload lower"); self._set(1900)
        try:
            self._wait(self.lower_seconds,guard); self._set(1500); self._wait(self.settle_seconds,guard)
            self.vehicle.log("Payload retract/lock"); self._set(1100); self._wait(self.retract_seconds,guard)
            self._set(1500); self._wait(self.settle_seconds,guard)
        except BaseException:
            self._set(1500)
            if self.method == "rc_override": self._set(None)
            raise
        if self.method == "rc_override": self._set(None)
        self.vehicle.log("Payload cycle complete")
