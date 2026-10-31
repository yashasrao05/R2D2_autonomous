from __future__ import annotations
import time

class FailsafeWatchdog:
    def __init__(self, vehicle, timeout_s: float = 3.0): self.vehicle=vehicle; self.timeout_s=timeout_s; self.last_poll=time.monotonic(); self.level=0
    def tick(self) -> int:
        self.vehicle.poll(); self.last_poll=time.monotonic(); s=self.vehicle.status
        if s.battery_v and s.battery_v < 19.2 and self.level < 3: self.vehicle.log("CRITICAL battery: LAND", 2); self.vehicle.land(); self.level=3
        elif s.battery_v and s.battery_v < 19.5 and self.level < 2: self.vehicle.log("LOW battery: RTL", 3); self.vehicle.rtl(); self.level=2
        elif s.battery_v and s.battery_v < 19.8 and self.level < 1: self.vehicle.log("Battery warning", 4); self.level=1
        if not s.ekf_ok: self.vehicle.log("EKF unhealthy", 3, throttle_s=5.0, key="ekf-unhealthy")
        return self.level
    def healthy_companion(self) -> bool: return time.monotonic()-self.last_poll <= self.timeout_s
