from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Iterable
import yaml


@dataclass(frozen=True)
class Waypoint:
    index: int
    current: bool
    frame: int
    command: int
    params: tuple[float, float, float, float]
    lat: float
    lon: float
    alt: float
    autocontinue: bool


@dataclass(frozen=True)
class MissionPlan:
    home: Waypoint
    takeoff_scan_qr: Waypoint
    green_corridor_start: Waypoint
    green_corridor_end: Waypoint
    delivery_search_waypoints: list[Waypoint]
    blue_corridor_start: Waypoint
    blue_corridor_end: Waypoint
    land_podium: Waypoint
    all_waypoints: list[Waypoint]


def _data_lines(path: Path) -> Iterable[str]:
    for raw in path.read_text(encoding="utf-8-sig").splitlines():
        line = raw.strip()
        if line and not line.startswith("#"):
            yield line


def load_waypoints(path: str | Path) -> list[Waypoint]:
    path = Path(path)
    lines = list(_data_lines(path))
    if not lines or lines[0] != "QGC WPL 110":
        raise ValueError(f"{path} is not a QGC WPL 110 file")
    result: list[Waypoint] = []
    for number, line in enumerate(lines[1:], 2):
        fields = line.split("\t") if "\t" in line else line.split()
        if len(fields) != 12:
            raise ValueError(f"{path}:{number}: expected 12 WPL fields, got {len(fields)}")
        try:
            result.append(Waypoint(int(fields[0]), bool(int(fields[1])), int(fields[2]), int(fields[3]),
                                   tuple(float(v) for v in fields[4:8]), float(fields[8]), float(fields[9]),
                                   float(fields[10]), bool(int(fields[11]))))
        except ValueError as exc:
            raise ValueError(f"{path}:{number}: invalid waypoint: {exc}") from exc
    result.sort(key=lambda wp: wp.index)
    if [w.index for w in result] != list(range(len(result))):
        raise ValueError("waypoint indices must be sequential starting at 0")
    return result


def load_mission(path: str | Path, mapping_path: str | Path = "config/mission_mapping.yaml") -> MissionPlan:
    waypoints = load_waypoints(path)
    mapping = yaml.safe_load(Path(mapping_path).read_text()) or {}
    roles = mapping.get("roles", {})
    def setting(name: str, default: int) -> int:
        return int(roles.get(name, default))
    n = len(waypoints)
    home, scan = setting("home_index", 0), setting("takeoff_scan_qr_index", 1)
    gs, ge = setting("green_corridor_start_index", 2), setting("green_corridor_end_index", 3)
    bs = n - 1 - setting("blue_corridor_start_from_end", 2)
    be = n - 1 - setting("blue_corridor_end_from_end", 1)
    land = n - 1 - setting("land_from_end", 0)
    if not (0 <= home < scan < gs <= ge < bs <= be < land < n):
        raise ValueError("mission mapping produces invalid role ordering; need at least 8 waypoints")
    return MissionPlan(waypoints[home], waypoints[scan], waypoints[gs], waypoints[ge],
                       waypoints[ge + 1:bs], waypoints[bs], waypoints[be], waypoints[land], waypoints)


def load_fence(path: str | Path) -> list[list[tuple[float, float]]]:
    """Return polygons. Blank/comment separator begins a second (exclusion) polygon."""
    raw = Path(path).read_text(encoding="utf-8-sig").splitlines()
    polygons: list[list[tuple[float, float]]] = [[]]
    active_command: int | None = None
    expected_vertices: int | None = None
    for line in raw:
        text = line.strip()
        if not text or text.startswith("#"):
            if polygons[-1]:
                polygons.append([])
            continue
        values = text.split("\t") if "\t" in text else text.split()
        if len(values) >= 12:  # QGC fence/rally WPL commands
            command = int(values[3])
            if command not in (5001, 5002):
                continue
            count = int(float(values[4])) if float(values[4]) > 0 else None
            # A command transition (inclusion vs exclusion) or a completed declared polygon starts a new ring.
            if polygons[-1] and (command != active_command or (expected_vertices is not None and len(polygons[-1]) >= expected_vertices)):
                polygons.append([])
            active_command, expected_vertices = command, count
            lat, lon = float(values[8]), float(values[9])
        elif len(values) >= 2:
            lat, lon = float(values[0]), float(values[1])
        else:
            raise ValueError(f"invalid fence line: {line}")
        polygons[-1].append((lat, lon))
    polygons = [p for p in polygons if p]
    if not polygons or len(polygons[0]) < 3:
        raise ValueError("geofence requires an outer polygon with at least three vertices")
    return polygons
