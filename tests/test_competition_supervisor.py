from __future__ import annotations

import tempfile
import time
import unittest
from pathlib import Path
from types import SimpleNamespace

from core.mission_runtime import MissionRuntime
from streaming.wifi_hud_streamer import WifiHudStreamer


class RuntimeTests(unittest.TestCase):
    def runtime(self, directory):
        now = time.monotonic()
        status = SimpleNamespace(
            armed=False, mode="ALT_HOLD", battery_v=22.1, satellites=12,
            hdop=.8, ekf_ok=True, lat=13.0, lon=77.0, relative_alt_m=0.0,
            last_heartbeat_monotonic=now,
        )
        vehicle = SimpleNamespace(status=status, connected_transport="ethernet")
        guard = SimpleNamespace(state="DOWN", pwm=1000, rc_fresh=lambda: True)
        runtime = MissionRuntime(Path(directory) / "logs", vehicle, guard)
        runtime.update(readiness_passed=True, blockers=[])
        runtime.transition("IDLE_SELECT_PROFILE")
        return runtime

    def test_profile_gate_requires_ready_disarmed_idle_and_sc_down(self):
        with tempfile.TemporaryDirectory() as directory:
            runtime = self.runtime(directory)
            self.assertEqual(runtime.can_select_profile(), (True, "ready"))
            runtime._vehicle.status.armed = True
            self.assertIn("armed", runtime.can_select_profile()[1])

    def test_events_are_persisted_without_telemetry_spam(self):
        with tempfile.TemporaryDirectory() as directory:
            runtime = self.runtime(directory)
            runtime.log("complete local detail that is longer than MAVLink STATUSTEXT", source="test")
            lines = runtime.log_path.read_text(encoding="utf-8").splitlines()
            self.assertGreaterEqual(len(lines), 3)
            self.assertIn("complete local detail", lines[-1])


class DashboardTests(unittest.TestCase):
    def test_profile_selection_is_token_protected(self):
        calls = []
        dashboard = WifiHudStreamer(
            runtime=None, control_token="field-secret",
            profile_selector=lambda profile: (calls.append(profile) is None, f"{profile} locked"),
        )
        client = dashboard.app.test_client()
        self.assertEqual(client.post("/api/profile", json={"profile": "segment1"}).status_code, 401)
        response = client.post(
            "/api/profile", json={"profile": "segment1"},
            headers={"X-R2D2-Token": "field-secret"},
        )
        self.assertEqual(response.status_code, 202)
        self.assertEqual(calls, ["segment1"])

    def test_dashboard_has_no_remote_arm_or_start_endpoint(self):
        dashboard = WifiHudStreamer(control_token="field-secret")
        rules = {rule.rule for rule in dashboard.app.url_map.iter_rules()}
        self.assertNotIn("/api/start", rules)
        self.assertNotIn("/api/arm", rules)


if __name__ == "__main__":
    unittest.main()
