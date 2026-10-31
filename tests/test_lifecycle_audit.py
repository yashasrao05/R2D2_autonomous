from __future__ import annotations

import unittest
import tempfile
from pathlib import Path
from types import SimpleNamespace

import yaml

from control.mavlink_interface import MavlinkVehicle
from control.servo7_actuator import Servo7Actuator
from core.field_coordinates import load_field_layout
from safety.lifecycle_audit import run_lifecycle_audit
from safety.pixhawk_field_sync import sync_field_to_pixhawk


class FieldCoordinateTests(unittest.TestCase):
    def test_spreadsheet_labels_are_normalized_without_inventing_containment(self):
        layout = load_field_layout("config/field_coordinates.yaml")
        self.assertEqual(layout.source["measurement_method"], "phone_gps_on_field")
        self.assertEqual(layout.points["home_takeoff_landing"].source_label, "Starting Box")
        self.assertEqual(layout.points["blue_corridor_entry"].source_label, "Danger Zone – Blue")
        self.assertIsNone(layout.flight_containment)
        audit = layout.audit()
        self.assertFalse(audit["flight_ready"])
        self.assertIn("whole-flight containment boundary was not supplied", audit["blockers"])
        self.assertAlmostEqual(audit["distances_m"]["home_to_qr"], 1.24, delta=.05)

    def test_production_waypoints_use_the_measured_field(self):
        from core.waypoint_loader import load_mission

        layout = load_field_layout("config/field_coordinates.yaml")
        plan = load_mission("config/mission.waypoints", "config/mission_mapping.yaml")
        self.assertEqual((plan.home.lat, plan.home.lon), layout.points["home_takeoff_landing"].pair)
        self.assertEqual((plan.takeoff_scan_qr.lat, plan.takeoff_scan_qr.lon), layout.points["qr_scan_target"].pair)
        self.assertEqual(len(plan.delivery_search_waypoints), 5)
        self.assertTrue(all(13.1704 < waypoint.lat < 13.1708 for waypoint in plan.all_waypoints))

    def test_non_arming_dry_lifecycle_reports_static_coordinate_blocker(self):
        vehicle = SimpleNamespace(
            dry_run=True,
            connected_transport="dry-run",
            status=SimpleNamespace(armed=False, mode="GUIDED"),
            poll=lambda _duration: None,
        )
        report = run_lifecycle_audit(
            vehicle,
            {"transport": "ethernet"},
            "config/field_coordinates.yaml",
            sample_seconds=1.0,
        )
        self.assertTrue(report["non_arming"])
        self.assertIn("field_coordinates", report["failed_checks"])
        self.assertTrue(all(item["name"] != "winch_actuation" for item in report["checks"]))


class WinchBenchSafetyTests(unittest.TestCase):
    def test_winch_bench_test_refuses_armed_vehicle(self):
        vehicle = SimpleNamespace(status=SimpleNamespace(armed=True))
        with self.assertRaisesRegex(RuntimeError, "vehicle is armed"):
            Servo7Actuator(vehicle).test_drop(duration_s=.001)


class PixhawkFieldSyncTests(unittest.TestCase):
    def test_supplied_delivery_polygon_is_refused_as_whole_flight_fence(self):
        vehicle = MavlinkVehicle(dry_run=True, logger=lambda _text: None)
        vehicle.connect()
        with self.assertRaisesRegex(RuntimeError, "whole-flight containment boundary"):
            sync_field_to_pixhawk(vehicle, "config/field_coordinates.yaml")

    def test_validated_takeoff_and_fence_are_written_in_dry_run(self):
        points = {
            "home_takeoff_landing": (13.0, 77.0),
            "qr_scan_target": (13.00001, 77.0),
            "green_corridor_entry": (13.00002, 77.0),
            "green_corridor_exit": (13.00003, 77.0),
            "blue_corridor_entry": (13.00004, 77.0),
            "blue_corridor_exit": (13.00005, 77.0),
        }
        data = {
            "points": {
                name: {"latitude": lat, "longitude": lon, "source_label": name}
                for name, (lat, lon) in points.items()
            },
            "polygons": {
                "red_no_fly_boundary": {"vertices": [
                    {"latitude": 13.0002, "longitude": 77.0002},
                    {"latitude": 13.0003, "longitude": 77.0002},
                    {"latitude": 13.0003, "longitude": 77.0003},
                ]},
                "delivery_area_boundary": {"vertices": [
                    {"latitude": 12.9999, "longitude": 76.9999},
                    {"latitude": 13.0001, "longitude": 76.9999},
                    {"latitude": 13.0001, "longitude": 77.0001},
                ]},
            },
            "flight_containment_boundary": [
                {"latitude": 12.9999, "longitude": 76.9999},
                {"latitude": 13.0004, "longitude": 76.9999},
                {"latitude": 13.0004, "longitude": 77.0004},
                {"latitude": 12.9999, "longitude": 77.0004},
            ],
        }
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "field.yaml"
            path.write_text(yaml.safe_dump(data), encoding="utf-8")
            vehicle = MavlinkVehicle(dry_run=True, logger=lambda _text: None)
            vehicle.connect()
            _fence, report = sync_field_to_pixhawk(vehicle, path)
        self.assertTrue(report["pixhawk_write_verified"])
        self.assertAlmostEqual(vehicle.status.home_lat, 13.0)
        self.assertAlmostEqual(vehicle.status.home_lon, 77.0)

    def test_boot_unit_runs_non_arming_supervisor_over_ethernet(self):
        unit = Path("deploy/r2d2-autonomous.service").read_text(encoding="utf-8")
        self.assertIn("User=r2d2", unit)
        self.assertIn("--mode service --transport ethernet", unit)
        self.assertNotIn("--mission-profile", unit)
        self.assertIn("WantedBy=multi-user.target", unit)


if __name__ == "__main__":
    unittest.main()
