from __future__ import annotations

import math
import time
from pathlib import Path
from typing import Any

from core.field_coordinates import load_field_layout
from safety.geofence_enforcer import EARTH_M_PER_DEG, GeofenceEnforcer


def _distance_m(first: tuple[float, float], second: tuple[float, float]) -> float:
    north = (second[0] - first[0]) * EARTH_M_PER_DEG
    east = (second[1] - first[1]) * EARTH_M_PER_DEG * math.cos(math.radians(first[0]))
    return math.hypot(north, east)


def sync_field_to_pixhawk(vehicle, coordinates_path: str | Path,
                          margin_m: float = 2.0, gps_timeout_s: float = 60.0,
                          takeoff_tolerance_m: float = 5.0) -> tuple[GeofenceEnforcer, dict[str, Any]]:
    """Write home and polygon fences only after all geometry and GPS checks pass."""
    if vehicle.status.armed:
        raise RuntimeError("refusing field configuration write while Pixhawk is armed")
    layout = load_field_layout(coordinates_path)
    audit = layout.audit()
    if not audit["flight_ready"]:
        raise RuntimeError("field coordinates are not safe to upload: " + "; ".join(audit["blockers"]))
    polygons = layout.pixhawk_fence_polygons()
    takeoff = layout.points["home_takeoff_landing"]

    if vehicle.dry_run:
        vehicle.status.lat, vehicle.status.lon = takeoff.pair
        vehicle.status.absolute_alt_m = 900.0
        vehicle.status.satellites, vehicle.status.hdop = 12, .8
    else:
        vehicle.request_message_intervals(interval_us=500_000)
        deadline = time.monotonic() + gps_timeout_s
        while time.monotonic() < deadline:
            vehicle.poll(.1)
            if (vehicle.status.lat and vehicle.status.lon and vehicle.status.absolute_alt_m and
                    vehicle.status.satellites >= 6 and vehicle.status.hdop <= 2.0):
                break
        else:
            raise TimeoutError("no flight-quality GPS fix available for takeoff/home write")

    actual = (vehicle.status.lat, vehicle.status.lon)
    offset_m = _distance_m(takeoff.pair, actual)
    if offset_m > takeoff_tolerance_m:
        raise RuntimeError(
            f"aircraft is {offset_m:.1f}m from configured Starting Box; "
            f"move it within {takeoff_tolerance_m:.1f}m before writing home"
        )
    if not vehicle.set_home_confirmed(takeoff.latitude, takeoff.longitude, vehicle.status.absolute_alt_m):
        raise RuntimeError("Pixhawk rejected or failed read-back of the Starting Box home coordinate")

    geofence = GeofenceEnforcer(vehicle, polygons, margin_m)
    geofence.upload()
    return geofence, {
        "takeoff": {"latitude": takeoff.latitude, "longitude": takeoff.longitude,
                    "absolute_altitude_m": vehicle.status.absolute_alt_m},
        "distance_from_takeoff_m": round(offset_m, 2),
        "inclusion_vertices": len(polygons[0]),
        "exclusion_vertices": sum(len(polygon) for polygon in polygons[1:]),
        "pixhawk_write_verified": True,
    }
