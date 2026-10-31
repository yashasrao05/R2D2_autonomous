from __future__ import annotations

import tempfile
import unittest
from pathlib import Path

import yaml

from control.mavlink_interface import MavlinkVehicle
from safety.fc_flightplan_sync import sync_flightplan_to_fc
from safety.flight_checkout import run_flight_checkout
from safety.tx15_override import Tx15SafetyGuard


def _valid_bundle(directory: str) -> Path:
    root = Path(directory)
    points = {
        "home_takeoff_landing": (13.0, 77.0),
        "qr_scan_target": (13.0, 77.00001),
        "green_corridor_entry": (13.0, 77.00002),
        "green_corridor_exit": (13.0, 77.00003),
        "blue_corridor_entry": (13.0, 77.00009),
        "blue_corridor_exit": (13.0, 77.00010),
    }
    outer = [(12.9998, 76.9998), (13.0008, 76.9998), (13.0008, 77.0003), (12.9998, 77.0003)]
    red = [(13.00045, 77.00012), (13.00055, 77.00012), (13.00055, 77.00020), (13.00045, 77.00020)]
    delivery = [(12.99995, 77.000035), (13.00005, 77.000035), (13.00005, 77.000085), (12.99995, 77.000085)]

    field = {
        "points": {name: {"latitude": lat, "longitude": lon, "source_label": name}
                   for name, (lat, lon) in points.items()},
        "polygons": {
            "red_no_fly_boundary": {"vertices": [{"latitude": lat, "longitude": lon} for lat, lon in red]},
            "delivery_area_boundary": {"vertices": [{"latitude": lat, "longitude": lon} for lat, lon in delivery]},
        },
        "flight_containment_boundary": [{"latitude": lat, "longitude": lon} for lat, lon in outer],
    }
    (root / "field.yaml").write_text(yaml.safe_dump(field), encoding="utf-8")

    coordinates = [
        points["home_takeoff_landing"], points["qr_scan_target"],
        points["green_corridor_entry"], points["green_corridor_exit"],
        (13.0, 77.00004), (13.0, 77.00005), (13.0, 77.00006),
        (13.0, 77.00007), (13.0, 77.00008), points["blue_corridor_entry"],
        points["blue_corridor_exit"], points["home_takeoff_landing"],
    ]
    lines = ["QGC WPL 110"]
    for index, (lat, lon) in enumerate(coordinates):
        altitude = 0 if index in {0, 11} else 5
        lines.append(f"{index}\t{int(index == 0)}\t3\t16\t0\t0\t0\t0\t{lat:.7f}\t{lon:.7f}\t{altitude}\t1")
    (root / "mission.waypoints").write_text("\n".join(lines) + "\n", encoding="utf-8")

    mapping = {
        "fence_margin_m": 2.0,
        "delivery_altitude_m": 5.0,
        "mission_profiles": {
            "default": "full", "full": {},
            "segment1": {"scan_altitude_m": 5.0, "bypass_altitude_m": 10.0},
            "segment2": {"launch_altitude_m": 3.0, "red_bypass_altitude_m": 5.0, "return_altitude_m": 5.0},
        },
    }
    (root / "mapping.yaml").write_text(yaml.safe_dump(mapping), encoding="utf-8")
    rings = ["\n".join(f"{lat:.7f} {lon:.7f}" for lat, lon in polygon) for polygon in (outer, red)]
    (root / "fence.poly").write_text("\n\n".join(rings) + "\n", encoding="utf-8")
    manifest = {
        "schema_version": 1,
        "sources": {
            "coordinates": "field.yaml", "waypoints": "mission.waypoints",
            "mapping": "mapping.yaml", "fence": "fence.poly",
        },
        "fence_parameters": {
            "FENCE_ENABLE": 1, "FENCE_TYPE": 5, "FENCE_ACTION": 1,
            "FENCE_ALT_MAX": 15, "FENCE_MARGIN": 2,
        },
    }
    path = root / "upload.yaml"
    path.write_text(yaml.safe_dump(manifest), encoding="utf-8")
    return path


class FlightControllerSyncTests(unittest.TestCase):
    def test_valid_bundle_writes_and_readbacks_in_dry_run(self):
        with tempfile.TemporaryDirectory() as directory:
            manifest = _valid_bundle(directory)
            logs = []
            vehicle = MavlinkVehicle(dry_run=True, logger=logs.append)
            vehicle.connect()
            report = sync_flightplan_to_fc(vehicle, manifest)
        self.assertTrue(report["write_verified"])
        self.assertEqual(report["mission_items_verified"], 12)
        self.assertTrue(any("DRY MISSION upload/read-back 12 items" in item for item in logs))

    def test_current_placeholder_bundle_is_refused_before_write(self):
        vehicle = MavlinkVehicle(dry_run=True, logger=lambda _message: None)
        vehicle.connect()
        with self.assertRaisesRegex(RuntimeError, "unsafe"):
            sync_flightplan_to_fc(vehicle, "config/fc_upload_manifest.yaml")

    def test_dry_hop_checkout_completes_takeoff_hover_hop_return_land(self):
        with tempfile.TemporaryDirectory() as directory:
            manifest = _valid_bundle(directory)
            vehicle = MavlinkVehicle(dry_run=True, logger=lambda _message: None)
            vehicle.connect()
            report = run_flight_checkout(
                vehicle, Tx15SafetyGuard(vehicle), str(manifest),
                level="hop", takeoff_altitude_m=2.0, hover_seconds=3.0,
            )
        self.assertEqual(report["tasks_completed"], [
            "arm_takeoff_hover", "safe_waypoint_hop_and_return", "land_disarm",
        ])
        self.assertFalse(vehicle.status.armed)
        self.assertEqual(vehicle.status.mode, "LAND")


if __name__ == "__main__":
    unittest.main()
