from __future__ import annotations

import math
from pathlib import Path
from typing import Any

import yaml

from core.field_coordinates import load_field_layout
from core.waypoint_loader import load_fence, load_mission
from safety.geofence_enforcer import EARTH_M_PER_DEG, GeofenceEnforcer, point_in_polygon
from safety.lifecycle_audit import run_lifecycle_audit


def _check(name: str, passed: bool | None, detail: str) -> dict[str, Any]:
    return {"name": name, "status": "SKIP" if passed is None else ("PASS" if passed else "FAIL"), "detail": detail}


def _distance_m(first: tuple[float, float], second: tuple[float, float]) -> float:
    north = (second[0] - first[0]) * EARTH_M_PER_DEG
    east = (second[1] - first[1]) * EARTH_M_PER_DEG * math.cos(math.radians(first[0]))
    return math.hypot(north, east)


def _polygon_matches(actual: list[tuple[float, float]], expected: list[tuple[float, float]], tolerance_m: float = 1.0) -> bool:
    if len(actual) != len(expected):
        return False
    return all(min(_distance_m(vertex, candidate) for candidate in actual) <= tolerance_m for vertex in expected)


def _route_legs(plan, profile: str):
    search = list(plan.delivery_search_waypoints)
    if profile == "segment1":
        outbound = [plan.home, plan.takeoff_scan_qr, plan.green_corridor_start, plan.green_corridor_end, *search]
        return_path = [*reversed(search), plan.green_corridor_end, plan.green_corridor_start, plan.home]
        return list(zip(outbound, outbound[1:])) + list(zip(return_path, return_path[1:]))
    if profile == "segment2":
        route = [plan.home, plan.green_corridor_start, plan.green_corridor_end, *search,
                 plan.blue_corridor_start, plan.blue_corridor_end, plan.home]
        return list(zip(route, route[1:]))
    route = [plan.home, plan.takeoff_scan_qr, plan.green_corridor_start, plan.green_corridor_end,
             *search, plan.blue_corridor_start, plan.blue_corridor_end, plan.land_podium]
    return list(zip(route, route[1:]))


def _audit_mission_geometry(coordinates_path: str | Path, waypoint_path: str | Path,
                            mapping_path: str | Path, margin_m: float) -> list[dict[str, Any]]:
    checks: list[dict[str, Any]] = []
    try:
        layout = load_field_layout(coordinates_path)
        plan = load_mission(waypoint_path, mapping_path)
    except Exception as exc:
        return [_check("mission_files", False, str(exc))]

    expected_roles = {
        "home": (plan.home.lat, plan.home.lon, "home_takeoff_landing"),
        "takeoff_scan_qr": (plan.takeoff_scan_qr.lat, plan.takeoff_scan_qr.lon, "qr_scan_target"),
        "green_corridor_start": (plan.green_corridor_start.lat, plan.green_corridor_start.lon, "green_corridor_entry"),
        "green_corridor_end": (plan.green_corridor_end.lat, plan.green_corridor_end.lon, "green_corridor_exit"),
        "blue_corridor_start": (plan.blue_corridor_start.lat, plan.blue_corridor_start.lon, "blue_corridor_entry"),
        "blue_corridor_end": (plan.blue_corridor_end.lat, plan.blue_corridor_end.lon, "blue_corridor_exit"),
        "land": (plan.land_podium.lat, plan.land_podium.lon, "home_takeoff_landing"),
    }
    mismatches = []
    for role, (lat, lon, field_name) in expected_roles.items():
        expected = layout.points.get(field_name)
        if expected is None:
            mismatches.append(f"{role}: normalized point {field_name} is missing")
            continue
        distance = _distance_m((lat, lon), expected.pair)
        if distance > 3.0:
            mismatches.append(f"{role}: mission file is {distance:.1f}m from {field_name}")
    checks.append(_check(
        "waypoint_role_alignment", not mismatches,
        "mission roles match normalized coordinates within 3m" if not mismatches else "; ".join(mismatches),
    ))

    search = plan.delivery_search_waypoints
    delivery = [point.pair for point in layout.polygons.get("delivery_area_boundary", [])]
    red = [point.pair for point in layout.polygons.get("red_no_fly_boundary", [])]
    search_problems = []
    if len(search) < 5:
        search_problems.append(f"only {len(search)} delivery-search waypoints; segment1 requires at least 5")
    for waypoint in search:
        point = (waypoint.lat, waypoint.lon)
        if len(delivery) < 3 or not point_in_polygon(point, delivery):
            search_problems.append(f"waypoint {waypoint.index} is outside the delivery-area boundary")
        if len(red) >= 3 and point_in_polygon(point, red):
            search_problems.append(f"waypoint {waypoint.index} is inside the red exclusion")
    checks.append(_check(
        "delivery_search_coverage", not search_problems,
        f"{len(search)} search waypoints are inside delivery area and outside red" if not search_problems
        else "; ".join(search_problems),
    ))

    if layout.flight_containment is None:
        checks.append(_check("profile_route_clearance", False, "cannot validate flight legs without whole-field containment"))
    else:
        polygons = layout.pixhawk_fence_polygons()
        fence = GeofenceEnforcer(type("AuditVehicle", (), {"status": None, "log": lambda *_a, **_k: None})(), polygons, margin_m)
        route_problems = []
        for profile in ("full", "segment1", "segment2"):
            for start, end in _route_legs(plan, profile):
                try:
                    fence.assert_safe_leg((start.lat, start.lon), (end.lat, end.lon))
                except ValueError as exc:
                    route_problems.append(f"{profile} leg {start.index}->{end.index}: {exc}")
        checks.append(_check(
            "profile_route_clearance", not route_problems,
            "all full/segment1/segment2 GPS legs remain inside containment and outside red margin"
            if not route_problems else "; ".join(route_problems),
        ))
    return checks


def _audit_exported_fence(coordinates_path: str | Path, fence_path: str | Path) -> dict[str, Any]:
    try:
        layout = load_field_layout(coordinates_path)
        actual = load_fence(fence_path)
        expected = layout.pixhawk_fence_polygons()
    except Exception as exc:
        return _check("geofence_export_alignment", False, str(exc))
    matches = len(actual) == len(expected) and all(
        _polygon_matches(actual[index], polygon) for index, polygon in enumerate(expected)
    )
    return _check(
        "geofence_export_alignment", matches,
        "geofence.poly matches normalized inclusion and red-exclusion polygons"
        if matches else f"geofence.poly has {[len(item) for item in actual]} vertices; normalized field requires {[len(item) for item in expected]}",
    )


def _audit_live_fence_parameters(vehicle, maximum_mission_altitude_m: float = 10.0) -> dict[str, Any]:
    if vehicle.dry_run:
        return _check("pixhawk_fence_parameters", None, "dry-run cannot read Pixhawk fence parameters")
    names = ("FENCE_ENABLE", "FENCE_TYPE", "FENCE_ACTION", "FENCE_ALT_MAX", "ARMING_CHECK")
    values = {name: vehicle.read_parameter(name) for name in names}
    problems = []
    if any(value is None for value in values.values()):
        problems.append("one or more parameters could not be read")
    else:
        if int(round(values["FENCE_ENABLE"])) != 1:
            problems.append("FENCE_ENABLE must be 1")
        if not int(round(values["FENCE_TYPE"])) & 4:
            problems.append("FENCE_TYPE must include polygon fences (bit 4)")
        if int(round(values["FENCE_ACTION"])) == 0:
            problems.append("FENCE_ACTION must not be Report Only")
        if float(values["FENCE_ALT_MAX"]) < maximum_mission_altitude_m + 2.0:
            problems.append(f"FENCE_ALT_MAX must be at least {maximum_mission_altitude_m + 2.0:.0f}m")
        if int(round(values["ARMING_CHECK"])) == 0:
            problems.append("ARMING_CHECK is disabled")
    return _check(
        "pixhawk_fence_parameters", not problems,
        f"values={values}" + ("" if not problems else "; " + "; ".join(problems)),
    )


def audit_flightplan_files(coordinates_path: str | Path, waypoint_path: str | Path,
                           mapping_path: str | Path, fence_path: str | Path,
                           margin_m: float = 2.0) -> dict[str, Any]:
    """Pure file/geometry gate used before any FC write is attempted."""
    layout = load_field_layout(coordinates_path)
    coordinate_audit = layout.audit()
    checks = [_check(
        "field_coordinates", bool(coordinate_audit["flight_ready"]),
        "normalized field geometry is flight-ready" if coordinate_audit["flight_ready"]
        else "; ".join(coordinate_audit["blockers"]),
    )]
    checks.extend(_audit_mission_geometry(coordinates_path, waypoint_path, mapping_path, margin_m))
    checks.append(_audit_exported_fence(coordinates_path, fence_path))
    failed = [item for item in checks if item["status"] == "FAIL"]
    return {
        "passed": not failed,
        "checks": checks,
        "blockers": [f"{item['name']}: {item['detail']}" for item in failed],
    }


def run_competition_readiness(vehicle, mavlink_config: dict[str, Any], coordinates_path: str | Path,
                              waypoint_path: str | Path, mapping_path: str | Path,
                              fence_path: str | Path, margin_m: float = 2.0,
                              sample_seconds: float = 2.0) -> dict[str, Any]:
    """Non-arming readiness gate for boot service, field setup, and route validity."""
    lifecycle = run_lifecycle_audit(vehicle, mavlink_config, coordinates_path, sample_seconds)
    checks = list(lifecycle["checks"])
    file_audit = audit_flightplan_files(coordinates_path, waypoint_path, mapping_path, fence_path, margin_m)
    # field_coordinates is already included by the lifecycle audit.
    checks.extend(file_audit["checks"][1:])
    altitude_candidates = [10.0]
    try:
        mapping = yaml.safe_load(Path(mapping_path).read_text(encoding="utf-8")) or {}
        altitude_candidates.extend(waypoint.alt for waypoint in load_mission(waypoint_path, mapping_path).all_waypoints)
        altitude_candidates.append(float(mapping.get("delivery_altitude_m", 5.0)))
        for settings in (mapping.get("mission_profiles") or {}).values():
            if isinstance(settings, dict):
                altitude_candidates.extend(float(value) for key, value in settings.items() if key.endswith("altitude_m"))
    except (OSError, TypeError, ValueError):
        pass  # mission_files already carries the actionable parse failure
    checks.append(_audit_live_fence_parameters(vehicle, max(altitude_candidates)))
    failed = [item for item in checks if item["status"] == "FAIL"]
    return {
        "non_arming": True,
        "passed": not failed,
        "failed_checks": [item["name"] for item in failed],
        "blockers": [f"{item['name']}: {item['detail']}" for item in failed],
        "checks": checks,
        "coordinate_audit": lifecycle["coordinate_audit"],
    }
