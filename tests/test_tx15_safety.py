from __future__ import annotations

import time
import unittest
import sys
from types import SimpleNamespace
from unittest.mock import patch

from control.mavlink_interface import MavlinkVehicle
from control.servo7_actuator import Servo7Actuator
from safety.tx15_override import PilotOverride, Tx15SafetyGuard

try:
    import cv2  # noqa: F401
except ImportError:
    # Mission launch logic does not use OpenCV. A tiny import stub lets these
    # safety tests run on development hosts without the flight vision stack.
    sys.modules["cv2"] = SimpleNamespace()

from mission.state_machine import MissionStateMachine
from perception.oakd_corridor_avoidance import OakDCamera


class FakeVehicle:
    def __init__(self):
        now = time.monotonic()
        self.status = SimpleNamespace(
            rc_sequence=0,
            rc_channels={8: 1000},
            last_rc_monotonic=now,
            last_heartbeat_monotonic=now,
            mode="GUIDED",
        )
        self.mode_results: dict[str, bool] = {}
        self.mode_requests: list[tuple[str, float]] = []
        self.logs: list[str] = []
        self.stream_maintenance_calls = 0

    def poll(self, _duration=0.05):
        return self.status

    def maintain_rc_telemetry(self, _timeout):
        self.stream_maintenance_calls += 1

    def set_mode_confirmed(self, mode, timeout_s=3.0):
        self.mode_requests.append((mode, timeout_s))
        result = self.mode_results.get(mode, True)
        if result:
            self.status.mode = mode
        return result

    def log(self, text, _severity=6, **_kwargs):
        self.logs.append(text)


class Tx15SafetyTests(unittest.TestCase):
    def setUp(self):
        self.vehicle = FakeVehicle()
        self.guard = Tx15SafetyGuard(self.vehicle)

    def sample(self, pwm: int, count: int = 1):
        for _ in range(count):
            self.vehicle.status.rc_channels[8] = pwm
            self.vehicle.status.rc_sequence += 1
            self.vehicle.status.last_rc_monotonic = time.monotonic()
            self.guard.observe()

    def test_debounce_and_tested_manual_thresholds(self):
        self.sample(1000, 3)
        self.assertTrue(self.guard.is_down())
        self.sample(1800, 3)
        self.assertTrue(self.guard.is_up())

        # The tested file debounces the mid band rather than treating one
        # noisy sample as an immediate switch transition.
        self.sample(1500)
        self.assertFalse(self.guard.manual_requested())
        self.sample(1500, 2)
        self.assertTrue(self.guard.manual_requested())

        self.sample(1800, 3)
        self.sample(1000)
        self.assertTrue(self.guard.manual_requested())

    def test_stale_rc_fails_closed(self):
        self.sample(1800, 3)
        self.vehicle.status.last_rc_monotonic = time.monotonic() - 3.0
        self.assertTrue(self.guard.manual_requested())

    def test_primary_then_backup_fallback(self):
        self.vehicle.mode_results = {"ALT_HOLD": False, "STABILIZE": True}
        self.assertTrue(self.guard.yield_to_pilot("bench test"))
        self.assertEqual(self.vehicle.mode_requests, [("ALT_HOLD", 1.5), ("STABILIZE", 1.5)])
        self.assertEqual(self.vehicle.status.mode, "STABILIZE")

    def test_manual_switch_executes_fallback_and_stops_autonomy(self):
        self.sample(1800, 3)
        self.sample(1000)
        with self.assertRaises(PilotOverride):
            self.guard.assert_autonomy()
        self.assertEqual(self.vehicle.status.mode, "ALT_HOLD")


class MavlinkVehicleTests(unittest.TestCase):
    def test_connect_ignores_non_autopilot_udp_heartbeat(self):
        mavlink = SimpleNamespace(
            MAV_COMP_ID_AUTOPILOT1=1,
            MAV_AUTOPILOT_INVALID=8,
            MAV_MODE_FLAG_SAFETY_ARMED=128,
        )
        non_autopilot = SimpleNamespace(
            get_srcSystem=lambda: 255, get_srcComponent=lambda: 190,
            autopilot=8, base_mode=0, custom_mode=0,
        )
        pixhawk = SimpleNamespace(
            get_srcSystem=lambda: 1, get_srcComponent=lambda: 1,
            autopilot=3, base_mode=0, custom_mode=4,
        )

        class Connection:
            target_system = 0
            target_component = 0

            def __init__(self):
                self.messages = [non_autopilot, pixhawk]
                self.mav = SimpleNamespace(statustext_send=lambda *_args: None)

            def recv_match(self, **_kwargs):
                return self.messages.pop(0) if self.messages else None

            def mode_mapping(self):
                return {"GUIDED": 4}

            def close(self):
                pass

        connection = Connection()
        fake_mavutil = SimpleNamespace(
            mavlink=mavlink,
            mavlink_connection=lambda *_args, **_kwargs: connection,
        )
        with patch("control.mavlink_interface.mavutil", fake_mavutil):
            vehicle = MavlinkVehicle(transport="ethernet", logger=lambda _text: None)
            vehicle.connect(timeout=1.0)
        self.assertEqual((connection.target_system, connection.target_component), (1, 1))
        self.assertEqual(vehicle.status.mode, "GUIDED")

    def test_dry_run_status_timestamps_are_in_the_correct_fields(self):
        vehicle = MavlinkVehicle(dry_run=True, logger=lambda _text: None)
        vehicle.connect()
        self.assertEqual(vehicle.status.custom_mode, 4)
        self.assertGreater(vehicle.status.last_rc_monotonic, 0)
        self.assertGreater(vehicle.status.last_heartbeat_monotonic, 0)

    def test_repeated_status_text_can_be_rate_limited(self):
        messages = []
        vehicle = MavlinkVehicle(dry_run=True, logger=messages.append)
        vehicle.log("repeated", throttle_s=5.0, key="test")
        vehicle.log("repeated", throttle_s=5.0, key="test")
        self.assertEqual(messages, ["repeated"])

    def test_rc12_release_uses_mavlink_extension_channel_sentinel(self):
        sent = []
        mav = SimpleNamespace(
            rc_channels_override_send=lambda *values: sent.append(values),
            statustext_send=lambda *_values: None,
        )
        vehicle = MavlinkVehicle(logger=lambda _text: None)
        vehicle.connection = SimpleNamespace(target_system=1, target_component=1, mav=mav)
        vehicle.rc_override(None, channel=12)
        channel_values = sent[0][2:]
        self.assertEqual(channel_values[11], 65534)
        self.assertTrue(all(value == 65535 for index, value in enumerate(channel_values) if index != 11))


class LaunchVehicle:
    def __init__(self, *, armed=False, altitude=0.0):
        self.status = SimpleNamespace(armed=armed, relative_alt_m=altitude, mode="ALT_HOLD")
        self.calls = []
        self.dry_run = False

    def poll(self, _duration=0.05):
        return self.status

    def set_mode_confirmed(self, mode, timeout_s=3.0, cancel=None):
        self.calls.append(("mode", mode))
        self.status.mode = mode
        return True

    def arm_confirmed(self, arm=True, timeout_s=6.0, cancel=None):
        self.calls.append(("arm", arm))
        self.status.armed = arm
        return True

    def takeoff_confirmed(self, altitude_m, timeout_s=3.0, cancel=None):
        self.calls.append(("takeoff", altitude_m))
        self.status.relative_alt_m = altitude_m
        return True

    def send_body_velocity(self, vx, vy, vz):
        self.calls.append(("velocity", vx, vy, vz))

    def log(self, _text, _severity=6, **_kwargs):
        pass


class LaunchGuard:
    def __init__(self, vehicle, fail_on_guided_check=None):
        self.vehicle = vehicle
        self.fail_on_guided_check = fail_on_guided_check
        self.guided_checks = 0
        self.fallback_reasons = []

    def assert_autonomy(self, modes=("GUIDED",)):
        if modes == ("GUIDED",):
            self.guided_checks += 1
            if self.guided_checks == self.fail_on_guided_check:
                self.vehicle.status.mode = "ALT_HOLD"
                raise PilotOverride("TX15 manual override")

    def yield_to_pilot(self, reason):
        self.fallback_reasons.append(reason)
        self.vehicle.status.mode = "ALT_HOLD"
        return True


class LaunchSequenceTests(unittest.TestCase):
    @staticmethod
    def machine(vehicle, guard):
        machine = MissionStateMachine.__new__(MissionStateMachine)
        machine.vehicle = vehicle
        machine.operator_guard = guard
        machine.watchdog = SimpleNamespace(tick=lambda: 0)
        return machine

    def test_confirmed_launch_order(self):
        vehicle = LaunchVehicle()
        guard = LaunchGuard(vehicle)
        self.machine(vehicle, guard)._launch(5.0)
        self.assertEqual(vehicle.calls[:3], [("mode", "GUIDED"), ("arm", True), ("takeoff", 5.0)])

    def test_override_during_ground_spool_disarms(self):
        vehicle = LaunchVehicle()
        guard = LaunchGuard(vehicle, fail_on_guided_check=2)
        with self.assertRaises(PilotOverride):
            self.machine(vehicle, guard)._launch(5.0)
        self.assertIn(("arm", False), vehicle.calls)
        self.assertFalse(vehicle.status.armed)

    def test_airborne_retrigger_is_rejected_before_arming(self):
        vehicle = LaunchVehicle(armed=True, altitude=2.0)
        guard = LaunchGuard(vehicle)
        with self.assertRaises(RuntimeError):
            self.machine(vehicle, guard)._launch(5.0)
        self.assertFalse(any(call[0] in {"arm", "takeoff"} for call in vehicle.calls))
        self.assertTrue(guard.fallback_reasons)

    def test_internal_mission_fault_stops_motion_and_requests_land(self):
        vehicle = LaunchVehicle(armed=True, altitude=5.0)
        vehicle.status.mode = "GUIDED"
        machine = self.machine(vehicle, LaunchGuard(vehicle))
        machine._failsafe_land("camera failure")
        self.assertIn(("velocity", 0, 0, 0), vehicle.calls)
        self.assertIn(("mode", "LAND"), vehicle.calls)


class OakWaitTests(unittest.TestCase):
    def test_camera_wait_is_bounded_and_services_safety_callback(self):
        callbacks = []
        empty_queue = SimpleNamespace(tryGet=lambda: None)
        with self.assertRaises(TimeoutError):
            OakDCamera._wait_for_packet(
                empty_queue, time.monotonic() + .04, "test", lambda: callbacks.append(True)
            )
        self.assertTrue(callbacks)


class ServoVehicle:
    def __init__(self):
        self.commands = []
        self.logs = []
        self.command_results = []
        self.captured_neutral_pwm = 1600

    def command_long_confirmed(self, command, *params, **_kwargs):
        self.commands.append((command, *params))
        return self.command_results.pop(0) if self.command_results else True

    def rc_override(self, pwm, channel=12):
        self.commands.append(("rc", channel, pwm))

    def capture_rc_channel_pwm(self, channel, **_kwargs):
        self.commands.append(("capture", channel))
        return self.captured_neutral_pwm

    def log(self, text, _severity=6, **_kwargs):
        self.logs.append(text)


class Servo7Tests(unittest.TestCase):
    def test_drop_uses_output_7_pwm_2000_then_captured_rc12_neutral(self):
        vehicle = ServoVehicle()
        actuator = Servo7Actuator(vehicle,method="set_servo")
        actuator.test_drop(duration_s=.001)
        self.assertEqual(vehicle.commands, [("capture", 12), (183, 7, 2000), (183, 7, 1600)])

    def test_drop_restores_neutral_when_wait_is_interrupted(self):
        vehicle = ServoVehicle()
        actuator = Servo7Actuator(vehicle,method="set_servo")
        actuator._wait = lambda _duration: (_ for _ in ()).throw(KeyboardInterrupt())
        with self.assertRaises(KeyboardInterrupt):
            actuator.test_drop(duration_s=.001)
        self.assertEqual(vehicle.commands[-1], (183, 7, 1600))

    def test_release_attempts_neutral_if_drop_ack_fails(self):
        vehicle = ServoVehicle()
        vehicle.command_results = [False, True]
        actuator = Servo7Actuator(vehicle, method="set_servo", lower_seconds=.001, retract_seconds=.001, settle_seconds=.001)
        with self.assertRaises(RuntimeError):
            actuator.release_cycle()
        self.assertEqual(vehicle.commands, [("capture", 12), (183, 7, 2000), (183, 7, 1600)])

    def test_65_second_drop_duration_is_allowed(self):
        vehicle = ServoVehicle()
        actuator = Servo7Actuator(vehicle)
        waits = []
        actuator._wait = waits.append
        actuator.test_drop()
        self.assertEqual(waits, [65.0])
        self.assertEqual(vehicle.commands, [("capture", 12), (183, 7, 2000), (183, 7, 1600)])

    def test_rc12_offset_is_honoured_by_releasing_override(self):
        vehicle = ServoVehicle()
        actuator = Servo7Actuator(vehicle, method="rc_override", rc_channel=12)
        actuator.test_drop(duration_s=.001)
        self.assertEqual(vehicle.commands, [("rc", 12, 2000), ("rc", 12, None)])


if __name__ == "__main__":
    unittest.main()
