#!/usr/bin/env python3
from __future__ import annotations

import inspect
import json
import os
import shutil
import sys
from importlib.metadata import PackageNotFoundError, version
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
os.environ.setdefault("MAVLINK20", "1")


def check(name, passed, detail):
    return {"name": name, "status": "PASS" if passed else "FAIL", "detail": detail}


def main() -> int:
    results = []
    results.append(check("python", sys.version_info >= (3, 10), sys.version.split()[0]))
    try:
        import numpy as np
        results.append(check("numpy", True, np.__version__))
    except Exception as exc:
        results.append(check("numpy", False, repr(exc)))
    try:
        import cv2
        aruco_ready = hasattr(cv2, "aruco") and hasattr(cv2.aruco, "ArucoDetector")
        results.append(check("opencv_contrib_aruco", aruco_ready, cv2.__version__))
    except Exception as exc:
        results.append(check("opencv_contrib_aruco", False, repr(exc)))
    try:
        import yaml
        results.append(check("pyyaml", True, getattr(yaml, "__version__", "available")))
    except Exception as exc:
        results.append(check("pyyaml", False, repr(exc)))
    try:
        import flask  # noqa: F401
        try:
            flask_version = version("flask")
        except PackageNotFoundError:
            flask_version = "available"
        results.append(check("flask", True, flask_version))
    except Exception as exc:
        results.append(check("flask", False, repr(exc)))
    try:
        import depthai as dai
        v3_ready = hasattr(dai, "Pipeline") and hasattr(dai.node, "Camera") and hasattr(dai, "NNModelDescription")
        results.append(check("depthai_v3_api", v3_ready, getattr(dai, "__version__", "available")))
    except Exception as exc:
        results.append(check("depthai_v3_api", False, repr(exc)))
    try:
        from pymavlink import mavutil
        rc_parameters = inspect.signature(mavutil.mavlink.MAVLink.rc_channels_override_send).parameters
        mission_parameters = inspect.signature(mavutil.mavlink.MAVLink.mission_item_int_send).parameters
        api_ready = "chan18_raw" in rc_parameters and "mission_type" in mission_parameters
        results.append(check(
            "pymavlink_mavlink2_api", api_ready,
            f"rc18={'chan18_raw' in rc_parameters}, mission_type={'mission_type' in mission_parameters}",
        ))
    except Exception as exc:
        results.append(check("pymavlink_mavlink2_api", False, repr(exc)))

    gst = shutil.which("gst-launch-1.0")
    linux_target = sys.platform.startswith("linux")
    results.append(check(
        "jetson_gstreamer", gst is not None or not linux_target,
        gst or ("not required on this non-Linux development host" if not linux_target else "gst-launch-1.0 not found"),
    ))

    try:
        from control.mavlink_interface import MavlinkVehicle
        required = {
            "set_mode_confirmed", "arm_confirmed", "takeoff_confirmed",
            "set_parameter_confirmed", "upload_fence_polygons", "download_fence_polygons",
            "upload_mission_items",
        }
        missing = sorted(name for name in required if not callable(getattr(MavlinkVehicle, name, None)))
        results.append(check("flight_api_names", not missing, "all current APIs present" if not missing else "missing " + ", ".join(missing)))
    except Exception as exc:
        results.append(check("flight_api_names", False, repr(exc)))

    failed = [item["name"] for item in results if item["status"] == "FAIL"]
    print(json.dumps({"passed": not failed, "failed": failed, "checks": results}, indent=2))
    return 0 if not failed else 2


if __name__ == "__main__":
    raise SystemExit(main())
