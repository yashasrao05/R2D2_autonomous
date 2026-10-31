from __future__ import annotations

from pathlib import Path
from typing import Any

import yaml

from core.waypoint_loader import load_mission
from safety.competition_readiness import audit_flightplan_files
from safety.pixhawk_field_sync import sync_field_to_pixhawk


ALLOWED_FENCE_PARAMETERS = {
    "FENCE_ENABLE", "FENCE_TYPE", "FENCE_ACTION", "FENCE_ALT_MAX", "FENCE_MARGIN",
}


def load_upload_manifest(path: str | Path) -> dict[str, Any]:
    manifest_path = Path(path)
    data = yaml.safe_load(manifest_path.read_text(encoding="utf-8")) or {}
    if int(data.get("schema_version", 0)) != 1:
        raise ValueError("FC upload manifest schema_version must be 1")
    source = data.get("sources") or {}
    required = {"coordinates", "waypoints", "mapping", "fence"}
    missing = sorted(required - source.keys())
    if missing:
        raise ValueError("FC upload manifest is missing sources: " + ", ".join(missing))
    root = manifest_path.parent
    files = {name: (root / str(source[name])).resolve() for name in required}
    for name, file_path in files.items():
        if not file_path.is_file():
            raise FileNotFoundError(f"manifest {name} file does not exist: {file_path}")

    raw_parameters = data.get("fence_parameters") or {}
    unknown = sorted(set(raw_parameters) - ALLOWED_FENCE_PARAMETERS)
    if unknown:
        raise ValueError("manifest contains unsupported FC parameters: " + ", ".join(unknown))
    missing_parameters = sorted(ALLOWED_FENCE_PARAMETERS - set(raw_parameters))
    if missing_parameters:
        raise ValueError("manifest is missing fence parameters: " + ", ".join(missing_parameters))
    parameters = {name: float(value) for name, value in raw_parameters.items()}
    if int(round(parameters["FENCE_ENABLE"])) != 1:
        raise ValueError("FENCE_ENABLE must be 1")
    if not int(round(parameters["FENCE_TYPE"])) & 4:
        raise ValueError("FENCE_TYPE must include inclusion/exclusion polygons (bit value 4)")
    if int(round(parameters["FENCE_ACTION"])) not in {1, 2, 3, 4, 5}:
        raise ValueError("FENCE_ACTION must select a real Copter breach action (1-5), not Report Only")
    if not 10.0 <= parameters["FENCE_ALT_MAX"] <= 1000.0:
        raise ValueError("FENCE_ALT_MAX must be between 10m and 1000m")
    if not 1.0 <= parameters["FENCE_MARGIN"] <= 10.0:
        raise ValueError("FENCE_MARGIN must be between 1m and 10m")
    return {"path": manifest_path.resolve(), "files": files, "parameters": parameters}


def sync_flightplan_to_fc(vehicle, manifest_path: str | Path) -> dict[str, Any]:
    """Non-arming write of fence parameters, home, polygons, and AUTO mission with read-back."""
    if vehicle.status.armed:
        raise RuntimeError("refusing FC flight-plan write while Pixhawk reports ARMED")
    manifest = load_upload_manifest(manifest_path)
    files = manifest["files"]
    parameters = manifest["parameters"]
    margin = float(parameters["FENCE_MARGIN"])
    file_audit = audit_flightplan_files(
        files["coordinates"], files["waypoints"], files["mapping"], files["fence"], margin,
    )
    if not file_audit["passed"]:
        raise RuntimeError("FC upload files are unsafe: " + "; ".join(file_audit["blockers"]))

    plan = load_mission(files["waypoints"], files["mapping"])
    mapping = yaml.safe_load(files["mapping"].read_text(encoding="utf-8")) or {}
    altitude_candidates = [waypoint.alt for waypoint in plan.all_waypoints]
    altitude_candidates.append(float(mapping.get("delivery_altitude_m", 5.0)))
    for settings in (mapping.get("mission_profiles") or {}).values():
        if isinstance(settings, dict):
            altitude_candidates.extend(float(value) for key, value in settings.items() if key.endswith("altitude_m"))
    highest_altitude = max(altitude_candidates)
    if parameters["FENCE_ALT_MAX"] < highest_altitude + 2.0:
        raise RuntimeError(
            f"FENCE_ALT_MAX={parameters['FENCE_ALT_MAX']:.1f}m is below mission maximum "
            f"{highest_altitude:.1f}m plus the required 2m clearance"
        )

    # Configure breach behavior before enabling the freshly uploaded fence.
    for name in ("FENCE_TYPE", "FENCE_ACTION", "FENCE_ALT_MAX", "FENCE_MARGIN"):
        if not vehicle.set_parameter_confirmed(name, parameters[name]):
            raise RuntimeError(f"Pixhawk did not confirm {name}={parameters[name]:g}")

    _geofence, field_report = sync_field_to_pixhawk(vehicle, files["coordinates"], margin)
    if not vehicle.set_parameter_confirmed("FENCE_ENABLE", parameters["FENCE_ENABLE"]):
        raise RuntimeError("Pixhawk did not persist FENCE_ENABLE=1")
    vehicle.upload_mission_items(plan.all_waypoints)

    return {
        "non_arming": True,
        "manifest": str(manifest["path"]),
        "parameters_verified": parameters,
        "home_and_fence": field_report,
        "mission_items_verified": len(plan.all_waypoints),
        "file_checks": file_audit["checks"],
        "write_verified": True,
    }
