from __future__ import annotations

import tempfile
import unittest
from types import SimpleNamespace
from pathlib import Path

import cv2
import numpy as np

from control.servo7_actuator import Servo7Actuator
from mission.state_machine import MarkerVault
from perception.cameras import pixel_to_ground_offset
from perception.oakd_corridor_avoidance import camera_to_body_frd, detect_corridor_banners


class GeometryTests(unittest.TestCase):
    def test_oak_45_degree_optical_axis_maps_forward_and_down(self):
        forward, right, down = camera_to_body_frd(0.0, 0.0, 2.0, 45.0)
        self.assertAlmostEqual(forward, 2 ** .5, places=6)
        self.assertAlmostEqual(right, 0.0, places=6)
        self.assertAlmostEqual(down, 2 ** .5, places=6)

    def test_down_camera_signs_match_body_frd(self):
        shape = (480, 640, 3)
        forward, right = pixel_to_ground_offset((500, 100), shape, 5.0)
        self.assertGreater(forward, 0.0)  # image top -> body forward
        self.assertGreater(right, 0.0)    # image right -> body right


class BannerTests(unittest.TestCase):
    def test_long_blue_banners_detect_top_and_bottom(self):
        image = np.zeros((360, 640, 3), np.uint8)
        cv2.rectangle(image, (60, 20), (580, 65), (255, 0, 0), -1)
        cv2.rectangle(image, (80, 300), (560, 335), (255, 0, 0), -1)
        found = detect_corridor_banners(image, "blue")
        self.assertEqual({item.location for item in found}, {"top", "bottom"})
        self.assertTrue(all(item.aspect_ratio > 3.5 for item in found))

    def test_blue_sky_like_low_saturation_region_is_rejected(self):
        image = np.full((360, 640, 3), (235, 205, 180), np.uint8)
        self.assertEqual(detect_corridor_banners(image, "blue"), [])


class PersistenceAndServoTests(unittest.TestCase):
    def test_reference_cannot_be_overwritten_by_decoy(self):
        with tempfile.TemporaryDirectory() as directory:
            vault = MarkerVault(Path(directory) / "vault.json")
            target = SimpleNamespace(matrix=None, aruco_id=100, qr_data=None)
            decoy = SimpleNamespace(matrix=None, aruco_id=200, qr_data=None)
            vault.set_reference(target, 1.0, 2.0)
            with self.assertRaisesRegex(RuntimeError, "already latched"):
                vault.set_reference(decoy, 3.0, 4.0)
            self.assertEqual(MarkerVault(vault.path).items["reference"]["aruco_id"], 100)

    def test_servo_limit_mismatch_fails_closed(self):
        class Vehicle:
            dry_run = False
            def read_parameter(self, name):
                return {"SERVO7_FUNCTION": 62, "SERVO7_MIN": 1100, "SERVO7_MAX": 1900,
                        "SERVO7_TRIM": 1500}[name]
        actuator = Servo7Actuator(Vehicle(), lower_pwm=2000)
        with self.assertRaisesRegex(RuntimeError, "SERVO7_MAX=1900"):
            actuator.verify_configuration()

    def test_timed_winch_is_scheduled_on_the_flight_controller(self):
        class Vehicle:
            dry_run = False
            def __init__(self): self.commands = []
            def read_parameter(self, name):
                return {"SERVO7_FUNCTION": 62, "SERVO7_MIN": 1100, "SERVO7_MAX": 2000,
                        "SERVO7_TRIM": 1507}[name]
            def capture_rc_channel_pwm(self, _channel, **_kwargs): return 1507
            def command_long_confirmed(self, command, *params, **_kwargs):
                self.commands.append((command, *params)); return True
            def log(self, *_args, **_kwargs): pass
        vehicle = Vehicle()
        actuator = Servo7Actuator(vehicle, autopilot_timed=True)
        actuator._wait = lambda _duration: None
        actuator.test_drop(duration_s=65.0)
        self.assertEqual(vehicle.commands[0], (184, 7, 2000, 1, 130.0))
        self.assertEqual(vehicle.commands[-1], (183, 7, 1507))


if __name__ == "__main__":
    unittest.main()
