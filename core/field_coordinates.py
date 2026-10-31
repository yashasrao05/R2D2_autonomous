from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import math
from typing import Any

import yaml

from safety.geofence_enforcer import EARTH_M_PER_DEG, point_in_polygon


@dataclass(frozen=True)
class FieldPoint:
    latitude: float
    longitude: float
    source_label: str

    @property
    def pair(self) -> tuple[float, float]:
        return self.latitude, self.longitude


@dataclass(frozen=True)
class FieldLayout:
    points: dict[str, FieldPoint]
    polygons: dict[str, list[FieldPoint]]
    flight_containment: list[FieldPoint] | None
    source: dict[str, Any]

    def distance_m(self, first: str, second: str) -> float:
        a, b = self.points[first], self.points[second]
        north = (b.latitude - a.latitude) * EARTH_M_PER_DEG
        east = (b.longitude - a.longitude) * EARTH_M_PER_DEG * math.cos(math.radians(a.latitude))
        return math.hypot(north, east)

    def pixhawk_fence_polygons(self) -> list[list[tuple[float, float]]]:
        if self.flight_containment is None or len(self.flight_containment) < 3:
            raise ValueError("whole-flight containment boundary is required before Pixhawk fence upload")
        red = self.polygons.get("red_no_fly_boundary", [])
        if len(red) < 3:
            raise ValueError("red no-fly exclusion requires at least three vertices")
        return [
            [point.pair for point in self.flight_containment],
            [point.pair for point in red],
        ]

    def audit(self) -> dict[str, Any]:
        required = {
            "home_takeoff_landing", "qr_scan_target", "green_corridor_entry",
            "green_corridor_exit", "blue_corridor_entry", "blue_corridor_exit",
        }
        missing = sorted(required - self.points.keys())
        red = [point.pair for point in self.polygons.get("red_no_fly_boundary", [])]
        delivery = [point.pair for point in self.polygons.get("delivery_area_boundary", [])]
        transit_names = sorted(required)
        delivery_contains = {
            name: point_in_polygon(self.points[name].pair, delivery)
            for name in transit_names
        } if len(delivery) >= 3 and not missing else {}
        red_conflicts = [
            name for name in transit_names
            if len(red) >= 3 and point_in_polygon(self.points[name].pair, red)
        ] if not missing else []
        containment = None if self.flight_containment is None else [point.pair for point in self.flight_containment]
        containment_contains = {
            name: point_in_polygon(self.points[name].pair, containment)
            for name in transit_names
        } if containment and len(containment) >= 3 and not missing else {}
        blockers = []
        if missing:
            blockers.append("missing normalized roles: " + ", ".join(missing))
        if len(red) < 3:
            blockers.append("red no-fly boundary has fewer than three vertices")
        if len(delivery) < 3:
            blockers.append("delivery-area boundary has fewer than three vertices")
        if containment is None:
            blockers.append("whole-flight containment boundary was not supplied")
        elif not all(containment_contains.values()):
            outside = [name for name, inside in containment_contains.items() if not inside]
            blockers.append("flight containment excludes: " + ", ".join(outside))
        if red_conflicts:
            blockers.append("normalized route points overlap red exclusion: " + ", ".join(red_conflicts))
        return {
            "source": self.source,
            "point_count": len(self.points),
            "red_exclusion_vertices": len(red),
            "delivery_boundary_vertices": len(delivery),
            "delivery_boundary_contains_transit": delivery_contains,
            "red_exclusion_conflicts": red_conflicts,
            "distances_m": {
                "home_to_qr": round(self.distance_m("home_takeoff_landing", "qr_scan_target"), 2) if not missing else None,
                "green_corridor": round(self.distance_m("green_corridor_entry", "green_corridor_exit"), 2) if not missing else None,
                "blue_corridor": round(self.distance_m("blue_corridor_entry", "blue_corridor_exit"), 2) if not missing else None,
            },
            "flight_ready": not blockers,
            "blockers": blockers,
        }


def _point(value: dict[str, Any]) -> FieldPoint:
    latitude = float(value["latitude"])
    longitude = float(value["longitude"])
    if not -90 <= latitude <= 90 or not -180 <= longitude <= 180:
        raise ValueError(f"invalid coordinate {latitude}, {longitude}")
    return FieldPoint(latitude, longitude, str(value.get("source_label", "")))


def load_field_layout(path: str | Path) -> FieldLayout:
    data = yaml.safe_load(Path(path).read_text(encoding="utf-8")) or {}
    points = {name: _point(value) for name, value in (data.get("points") or {}).items()}
    polygons = {
        name: [_point(value) for value in (definition.get("vertices") or [])]
        for name, definition in (data.get("polygons") or {}).items()
    }
    raw_containment = data.get("flight_containment_boundary")
    containment = None if raw_containment is None else [_point(value) for value in raw_containment]
    return FieldLayout(points, polygons, containment, dict(data.get("source") or {}))
