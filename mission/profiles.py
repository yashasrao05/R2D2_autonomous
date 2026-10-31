from __future__ import annotations

from mission.state_machine import MissionStateMachine, Phase


class SegmentOneMission(MissionStateMachine):
    """Official Run 1: identify, bypass both corridors, deliver, and return home."""

    PROFILE = "segment1"

    def __init__(self, *args, scan_altitude_m: float = 5.0,
                 bypass_altitude_m: float = 10.0, **kwargs):
        super().__init__(*args, **kwargs)
        self.scan_altitude_m = float(scan_altitude_m)
        self.bypass_altitude_m = float(bypass_altitude_m)

    def _high_route(self, waypoints, phase: Phase) -> None:
        self.phase = phase
        last = None
        for waypoint in waypoints:
            if last is not None and waypoint.index == last:
                continue
            self._goto(waypoint, self.bypass_altitude_m)
            last = waypoint.index

    def _mission_steps(self) -> None:
        self.phase = Phase.SCAN_REFERENCE
        self._launch(self.scan_altitude_m)
        self._goto(self.plan.takeoff_scan_qr, self.scan_altitude_m)
        self._collect_reference()
        self._checkpoint(Phase.HIGH_BYPASS_OUT, self.plan.takeoff_scan_qr)

        # Use the corridor entry/exit GPS anchors only as a high-altitude
        # bypass route. The OAK corridor controller is deliberately not run.
        self._high_route(
            (self.plan.green_corridor_start, self.plan.green_corridor_end),
            Phase.HIGH_BYPASS_OUT,
        )
        self._checkpoint(Phase.SEARCH, self.plan.green_corridor_end)
        target, frame = self.phase3()
        self.phase4(target, frame, next_phase=Phase.HIGH_BYPASS_RETURN)

        matched = getattr(self, "_matched_waypoint", None)
        search = self.plan.delivery_search_waypoints
        matched_position = search.index(matched) if matched in search else len(search) - 1
        reverse_search = list(reversed(search[:matched_position + 1]))
        self._high_route(
            reverse_search + [self.plan.green_corridor_end,
                              self.plan.green_corridor_start, self.plan.home],
            Phase.HIGH_BYPASS_RETURN,
        )
        self._checkpoint(Phase.LAND, self.plan.home)
        self._land_at(self.plan.home)

    def resume(self):
        raise RuntimeError(
            "segment1 is an official single-run profile; automatic resume is disabled "
            "to prevent an ambiguous second payload release"
        )


class SegmentTwoMission(MissionStateMachine):
    """Official Run 2: traverse both corridors, bypass red, and return home."""

    PROFILE = "segment2"
    requires_payload = False
    requires_delivery_search = False
    post_calibration_phase = Phase.ENTRY_CORRIDOR

    def __init__(self, *args, launch_altitude_m: float = 3.0,
                 red_bypass_altitude_m: float = 5.0,
                 return_altitude_m: float = 5.0, **kwargs):
        super().__init__(*args, **kwargs)
        self.launch_altitude_m = float(launch_altitude_m)
        self.red_bypass_altitude_m = float(red_bypass_altitude_m)
        self.return_altitude_m = float(return_altitude_m)

    def _validate_profile_plan(self):
        if not self.plan.delivery_search_waypoints:
            raise RuntimeError("segment2 requires at least one GPS red-bypass route waypoint")

    def _mission_steps(self) -> None:
        self.phase = Phase.ENTRY_CORRIDOR
        self._launch(self.launch_altitude_m)
        self._corridor(
            "green", self.plan.green_corridor_start,
            self.plan.green_corridor_end, obstacles=False,
        )
        self._checkpoint(Phase.RED_BYPASS, self.plan.green_corridor_end)

        # Reuse the existing delivery-search GPS route strictly as a red-zone
        # bypass. No marker recognition or payload behavior is called.
        self.phase = Phase.RED_BYPASS
        for waypoint in self.plan.delivery_search_waypoints:
            self._goto(waypoint, self.red_bypass_altitude_m)
            self._escape_red_zone(self._read_down())

        self._checkpoint(Phase.EXIT_CORRIDOR, self.plan.delivery_search_waypoints[-1])
        self.phase = Phase.EXIT_CORRIDOR
        self._corridor(
            "blue", self.plan.blue_corridor_start,
            self.plan.blue_corridor_end, obstacles=True,
        )
        self._checkpoint(Phase.RETURN_HOME, self.plan.blue_corridor_end)

        self.phase = Phase.RETURN_HOME
        self._goto(self.plan.home, self.return_altitude_m)
        self._checkpoint(Phase.LAND, self.plan.home)
        self._land_at(self.plan.home)

    def resume(self):
        raise RuntimeError(
            "segment2 is an official single-run profile; automatic resume is disabled; "
            "restart the run from the takeoff box"
        )


MISSION_PROFILES = {
    "full": MissionStateMachine,
    "segment1": SegmentOneMission,
    "segment2": SegmentTwoMission,
}


def build_mission(profile: str, *args, profile_config=None, **kwargs) -> MissionStateMachine:
    if profile not in MISSION_PROFILES:
        raise ValueError(f"unknown mission profile {profile!r}")
    options = dict(profile_config or {})
    return MISSION_PROFILES[profile](*args, **kwargs, **options)
