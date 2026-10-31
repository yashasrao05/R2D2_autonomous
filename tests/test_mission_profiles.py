from __future__ import annotations

import unittest
from pathlib import Path
from types import SimpleNamespace

from mission.profiles import SegmentOneMission, SegmentTwoMission
from mission.state_machine import MissionStateMachine
from scripts.mock_mission2_sil import run_sil
from safety.geofence_enforcer import EARTH_M_PER_DEG, GeofenceEnforcer


REPO = Path(__file__).resolve().parents[1]


class MissionProfileArchitectureTests(unittest.TestCase):
    def test_two_official_profiles_are_children_of_unchanged_parent(self):
        self.assertTrue(issubclass(SegmentOneMission, MissionStateMachine))
        self.assertTrue(issubclass(SegmentTwoMission, MissionStateMachine))

    def test_parent_and_both_children_complete_sil_contracts(self):
        results = {profile: run_sil(REPO, profile) for profile in ("full", "segment1", "segment2")}
        self.assertTrue(all(result["passed"] for result in results.values()))
        self.assertTrue(results["segment1"]["assertions"]["corridor_controller_skipped"])
        self.assertTrue(results["segment2"]["assertions"]["payload_never_called"])
        self.assertTrue(results["segment2"]["assertions"]["initial_scan_skipped"])


class SharedRedZoneSafetyTests(unittest.TestCase):
    @staticmethod
    def fence(current_lon_offset_m=0.0):
        lat,lon=13.0,77.0; lon_scale=EARTH_M_PER_DEG
        outer=[(lat-10/EARTH_M_PER_DEG,lon-10/lon_scale),(lat+10/EARTH_M_PER_DEG,lon-10/lon_scale),
               (lat+10/EARTH_M_PER_DEG,lon+10/lon_scale),(lat-10/EARTH_M_PER_DEG,lon+10/lon_scale)]
        red=[(lat-1/EARTH_M_PER_DEG,lon-1/lon_scale),(lat+1/EARTH_M_PER_DEG,lon-1/lon_scale),
             (lat+1/EARTH_M_PER_DEG,lon+1/lon_scale),(lat-1/EARTH_M_PER_DEG,lon+1/lon_scale)]
        status=SimpleNamespace(lat=lat,lon=lon+current_lon_offset_m/lon_scale,heading_deg=0.0)
        vehicle=SimpleNamespace(status=status,log=lambda *_args,**_kwargs: None)
        return GeofenceEnforcer(vehicle,[outer,red],margin_m=.5),lat,lon,lon_scale

    def test_straight_gps_leg_through_red_is_rejected(self):
        fence,lat,lon,lon_scale=self.fence()
        with self.assertRaisesRegex(ValueError,"red/exclusion"):
            fence.assert_safe_leg((lat,lon-3/lon_scale),(lat,lon+3/lon_scale))

    def test_velocity_toward_red_is_replaced_with_outward_velocity(self):
        fence,_lat,_lon,_scale=self.fence(current_lon_offset_m=1.2)
        forward,right,_down=fence.safe_velocity(0.0,-1.0,0.0)
        self.assertAlmostEqual(forward,0.0,delta=.15)
        self.assertGreater(right,0.5)


if __name__ == "__main__":
    unittest.main()
