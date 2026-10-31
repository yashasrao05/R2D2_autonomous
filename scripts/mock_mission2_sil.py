#!/usr/bin/env python3
"""Deterministic, non-arming Mission 2 software-in-the-loop bench test.

This test uses generated AprilTag frames and simulated OAK/local-EKF motion to
exercise the production MissionStateMachine from preflight through landing.  It
never opens MAVLink, a camera, or the payload output.
"""
from __future__ import annotations

import argparse
import json
import sys
import tempfile
import time
from pathlib import Path

import cv2
import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from control.checkpoint_manager import CheckpointManager
from control.mavlink_interface import MavlinkVehicle
from core.waypoint_loader import load_mission
from mission.profiles import build_mission
from mission.state_machine import MarkerVault, Phase
from perception.oakd_corridor_avoidance import BannerDetection, CorridorScene
from safety.failsafe_watchdog import FailsafeWatchdog
from safety.geofence_enforcer import GeofenceEnforcer


def tag_frame(marker_id: int, size: tuple[int, int] = (640, 480)) -> np.ndarray:
    """Generate a centred AprilTag 36h11 test image."""
    width, height = size
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    if hasattr(cv2.aruco, "generateImageMarker"):
        marker = cv2.aruco.generateImageMarker(dictionary, marker_id, 240)
    else:
        marker = np.zeros((240, 240), np.uint8)
        cv2.aruco.drawMarker(dictionary, marker_id, 240, marker, 1)
    frame = np.full((height, width, 3), 255, np.uint8)
    x, y = (width - 240) // 2, (height - 240) // 2
    frame[y:y + 240, x:x + 240] = cv2.cvtColor(marker, cv2.COLOR_GRAY2BGR)
    return frame


class SyntheticDownCamera:
    def __init__(self, frames: list[np.ndarray]):
        self.frames = frames
        self.index = 0

    def read(self, on_wait=None) -> np.ndarray:
        if on_wait:
            on_wait()
        frame = self.frames[min(self.index, len(self.frames) - 1)]
        self.index += 1
        return frame.copy()


class SilVehicle(MavlinkVehicle):
    def poll(self, duration=.05):
        self.status.local_position_valid = True
        self.status.last_local_position_monotonic = time.monotonic()
        return self.status


class SyntheticOak:
    def __init__(self, vehicle: MavlinkVehicle):
        self.vehicle = vehicle
        self.read_count = 0

    def calibrate_ground_plane(self, count=20, on_wait=None):
        if on_wait:
            on_wait()
        return {"normal": [0.0, .7071, .7071], "offset": 0.0,
                "pitch_offset_deg": 0.0, "floor_clearance_m": .25}

    def read(self, color="green", floor_height_m=.25, timeout_s=2.0, on_wait=None):
        if on_wait:
            on_wait()
        self.read_count += 1
        self.vehicle.status.local_north_m += 1.1
        self.vehicle.status.last_local_position_monotonic = time.monotonic()
        rgb = np.zeros((360, 640, 3), np.uint8)
        paint = (0, 177, 0) if color == "green" else (255, 0, 0)
        cv2.rectangle(rgb, (70, 20), (570, 70), paint, -1)
        banner = BannerDetection((70, 20, 500, 50), 0.0, "top", 10.0)
        white_range = 1.8 if color == "blue" and self.read_count % 3 == 0 else None
        return CorridorScene(rgb, np.full((360, 640), 4.0, np.float32), (3.0, 3.0, 3.0),
                             0.0, [], [banner], 1.0, white_range)


class MockPayload:
    def __init__(self): self.release_count = 0
    def verify_configuration(self): pass
    def release_cycle(self, guard=None):
        if guard: guard()
        self.release_count += 1


def run_sil(repo: Path, profile: str = "full") -> dict:
    logs: list[str] = []
    vehicle = SilVehicle(dry_run=True, logger=logs.append)
    vehicle.connect()
    vehicle.status.rc_channels = {8: 1800}

    neutral = np.full((480, 640, 3), 160, np.uint8)
    target = tag_frame(100)
    decoy = tag_frame(200)
    # Full/segment1: calibration, reference vote, decoy/target search votes,
    # then a stable delivery tail. Segment2 consumes only neutral red checks.
    frames = ([neutral] * 40 if profile == "segment2" else
              [neutral] * 20 + [target] * 5 + [decoy] * 5 + [target] * 12)
    down = SyntheticDownCamera(frames)
    oak = SyntheticOak(vehicle)
    payload = MockPayload()
    plan = load_mission(repo / "config/mission.waypoints", repo / "config/mission_mapping.yaml")
    vehicle.status.lat, vehicle.status.lon = plan.home.lat, plan.home.lon
    # SIL validates mission logic independently of event-day FC data. Production
    # config/geofence.poly deliberately remains invalid until the team records a
    # whole-flight containment polygon, so build a synthetic test-only envelope.
    latitudes = [waypoint.lat for waypoint in plan.all_waypoints]
    longitudes = [waypoint.lon for waypoint in plan.all_waypoints]
    pad = .0005
    synthetic_outer = [
        (min(latitudes) - pad, min(longitudes) - pad),
        (max(latitudes) + pad, min(longitudes) - pad),
        (max(latitudes) + pad, max(longitudes) + pad),
        (min(latitudes) - pad, max(longitudes) + pad),
    ]
    fence = GeofenceEnforcer(vehicle, [synthetic_outer], 2.0)

    with tempfile.TemporaryDirectory(prefix="mission2-sil-") as directory:
        temp = Path(directory)
        machine = build_mission(
            profile,
            vehicle, fence, FailsafeWatchdog(vehicle), plan, down, oak, payload,
            CheckpointManager(temp / "checkpoint.json"),
            calibration_path=temp / "calibration.json", hud=None, operator_guard=None,
            require_red_exclusion=False, minimum_search_waypoints=1,
        )
        machine.vault = MarkerVault(temp / "marker_vault.json")
        machine.run()
        vault = machine.vault.items
        checkpoint = machine.checkpoint.load()

    reference = vault.get("reference") or {}
    decoys = vault.get("visited_decoys") or []
    assertions = {
        "mission_complete": machine.phase == Phase.COMPLETE and checkpoint["state"] == "COMPLETE",
        "profile_checkpointed": checkpoint.get("mission_profile") == profile,
        "telemetry_not_flooded": len(logs) < 200,
        "landed_and_disarmed": not vehicle.status.armed,
    }
    if profile == "full":
        assertions.update({
            "reference_latched": reference.get("aruco_id") == 100,
            "decoy_rejected_and_recorded": any(item.get("aruco_id") == 200 for item in decoys),
            "payload_released_once": payload.release_count == 1,
            "both_corridors_traversed": oak.read_count >= 14,
        })
    elif profile == "segment1":
        assertions.update({
            "reference_latched": reference.get("aruco_id") == 100,
            "decoy_rejected_and_recorded": any(item.get("aruco_id") == 200 for item in decoys),
            "payload_released_once": payload.release_count == 1,
            "corridor_controller_skipped": oak.read_count == 0,
        })
    elif profile == "segment2":
        assertions.update({
            "initial_scan_skipped": not reference,
            "payload_never_called": payload.release_count == 0,
            "both_corridors_traversed": oak.read_count >= 14,
            "marker_search_skipped": not vault.get("detections"),
        })
    failed = [name for name, passed in assertions.items() if not passed]
    result = {"profile": profile, "passed": not failed, "assertions": assertions, "failed": failed,
              "oak_frames": oak.read_count, "down_frames": down.index,
              "local_distance_m": round(vehicle.status.local_north_m, 2), "log_messages": len(logs)}
    if failed:
        raise AssertionError(json.dumps(result, indent=2))
    return result


def main() -> int:
    parser = argparse.ArgumentParser(description="Non-arming parent/child mission-profile mock SIL")
    parser.add_argument("--repo", type=Path, default=REPO_ROOT)
    parser.add_argument("--profile", choices=["full","segment1","segment2","all"], default="full")
    args = parser.parse_args()
    profiles = ["full","segment1","segment2"] if args.profile == "all" else [args.profile]
    results = [run_sil(args.repo.resolve(), profile) for profile in profiles]
    print(json.dumps(results[0] if len(results) == 1 else results, indent=2))
    print("PASS: selected synthetic mission profile(s) completed without MAVLink, cameras, motors, or Servo 7 output")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
