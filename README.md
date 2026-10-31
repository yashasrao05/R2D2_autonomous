# AeroTHON 2026 SkyScan Companion

Jetson-side Python companion for the SkyScan mission. Production configuration uses MAVLink2 over Ethernet at `udpin:0.0.0.0:14550` and intentionally does not fall back to UART. The expected direct-link addresses are Pixhawk `192.168.168.2/24` and Jetson `192.168.168.10/24`. UART remains available only through the explicit `--transport uart` option.

For the headless competition Jetson, upload the current repository to `/home/r2d2/R2D2_autonomous`, connect with PuTTY over Wi-Fi, keep the aircraft disarmed with propellers removed and SC DOWN, then run one command:

```bash
cd /home/r2d2/R2D2_autonomous
bash deploy/one_shot_headless_setup.sh --ethernet-interface YOUR_PIXHAWK_ETHERNET_INTERFACE
```

Add `--with-winch-test` only when the winch is mechanically secured; it performs the requested PWM 2000 test for 65 seconds. The script configures the dedicated Ethernet interface persistently, updates the venv, runs dependency/unit/SIL/live non-arming checks, tests both cameras without a GUI, installs/enables systemd, and starts the supervisor. It refuses to readdress the interface carrying the active PuTTY connection.

Install with `python3 -m pip install -r requirements.txt`. Put field exports in `config/mission.waypoints` and `config/geofence.poly`, verify the YAML mapping, then validate parsing before flight:

```bash
python3 runner.py --mode test-waypoints --wp-file config/mission.waypoints
python3 runner.py --mode test-geofence
python3 runner.py --mode test-lifecycle
python3 runner.py --mode test-tx15 --dry-run
python3 runner.py --mode mission --dry-run --wp-file config/mission.waypoints
python3 scripts/mock_mission2_sil.py
```

The complete boot, laptop-console, field-check, state-condition, and official-run procedure is in [FIELD_RUNBOOK.md](FIELD_RUNBOOK.md). The decisive non-arming gate is:

```bash
python3 runner.py --mode competition-check --transport ethernet --lifecycle-seconds 5
```

Production now treats the polygon fence already stored by Mission Planner as authoritative. The Jetson downloads it read-only over MAVLink2, validates every mission leg against it, and never overwrites it during boot or profile selection. Spreadsheet red-zone coordinates have been removed; downward-camera red detection is active during GPS travel, corridor traversal, delivery, and return. The waypoint roles use the team-recorded phone coordinates, with five provisional delivery-search samples inset from the measured delivery polygon.

To monitor the physical TX15 switch without changing flight mode, use `python3 runner.py --mode test-tx15`. For a propellers-removed fallback test, use `python3 runner.py --mode test-tx15 --tx15-live-fallback`; it never arms or takes off, but it deliberately requests `GUIDED` after SC-up and verifies `ALT_HOLD`/`STABILIZE` after SC-down.

`--dry-run` never arms or sends flight/actuator commands. Camera tests still require their respective cameras. Do a restrained, propeller-off integration test and validate ArduPilot parameters—including `SERVO7_FUNCTION`, fence configuration, battery thresholds, and DShot limits—before operational flight.

## Pixhawk 6X Mini / Jetson Ethernet lifecycle test

The coordinate spreadsheet contains team-recorded on-field phone measurements and is normalized in `config/field_coordinates.yaml`; each clean role name retains its original spreadsheet label. Its red-zone rows are no longer stored or used. The four rows called `Geo Fence` remain the delivery/search boundary because they do not contain the Starting Box, QR target, or corridor anchors. The live Mission Planner fence downloaded from the Pixhawk is the whole-flight containment source.

Give the Jetson Ethernet interface the static address `192.168.168.10/24`, connect the Pixhawk 6X Ethernet link, keep the aircraft disarmed, place the RC12 winch control at its actual neutral position, and run:

```bash
python3 runner.py --mode test-lifecycle --transport ethernet --lifecycle-seconds 5
```

This is non-arming. It verifies the OS route, confirms that the heartbeat arrived specifically over Ethernet, samples only four MAVLink messages at 2 Hz, checks the Pixhawk network/failsafe parameters, downloads the Mission Planner polygon fence read-only, validates every profile leg, reads Servo 7 limits, and captures RC12 neutral without moving the winch. It exits non-zero and lists every blocker instead of stopping after the first one.

### Mission Planner fence ownership

Do not run `sync-pixhawk` or `sync-flightplan` in the current production configuration. Mission Planner owns the Pixhawk fence, and `competition-check` downloads it without mutation. The four spreadsheet `Geo Fence` points cannot safely be uploaded as the global inclusion polygon: geometrically they exclude the recorded Starting Box, QR target, and most corridor anchors, which would cause a fence/pre-arm failure.

### Start automatically at Jetson boot

The unit is configured for the observed Jetson account and path, `r2d2` and `/home/r2d2/R2D2_autonomous`. Install and enable it without starting the aircraft software yet:

```bash
cd /home/r2d2/R2D2_autonomous
bash deploy/install_boot_service.sh
```

After `competition-check` passes, start and inspect it:

```bash
sudo systemctl start r2d2-autonomous.service
systemctl status r2d2-autonomous.service
journalctl -fu r2d2-autonomous.service
```

At subsequent boots, systemd starts a non-arming supervisor after `network-online.target`. It waits for Ethernet, runs readiness checks, downloads and verifies the Mission Planner fence, validates both cameras, and serves the laptop console. An authenticated operator may lock Segment 1 or Segment 2 only while disarmed with fresh SC-DOWN telemetry. The existing TX15 SC-UP authorization remains mandatory before arming.

## Parent mission and two official child runs

The original complete mission remains the `full` parent in `MissionStateMachine`. Both official runs inherit that parent, so MAVLink, Ethernet-only connection, FC fence enforcement, visual red avoidance, TX15 authorization and fallback, watchdog, confirmed launch, camera waiting, throttled telemetry, checkpoints, and fault landing remain shared rather than copied.

Run the original parent profile:

```bash
python3 runner.py --mode mission --mission-profile full
```

Official Segment 1 scans and records the initial QR/ArUco marker, flies the corridor anchors only at the configured 10 m bypass altitude without invoking either corridor controller, follows the red-safe delivery-search route, finds the matching marker, performs the payload sequence once, reverses the safe high route, and lands at the Starting Box:

```bash
python3 runner.py --mode mission --mission-profile segment1
```

Official Segment 2 skips marker acquisition and payload configuration entirely, traverses the green entry corridor, follows the GPS/visual red-bypass route, traverses the blue exit corridor with white-obstacle avoidance, and returns to the Starting Box:

```bash
python3 runner.py --mode mission --mission-profile segment2
```

Every straight Guided GPS leg is sampled at 0.5 m spacing and rejected before movement if it leaves the downloaded FC containment or enters any FC exclusion. Independently, downward-camera red detection checks every movement phase and commands an outward escape without using stored red-zone coordinates.

Check all three profiles without hardware:

```bash
python3 scripts/mock_mission2_sil.py --profile all
```

Select the official run from the already-running boot service (this cannot arm it):

```bash
bash deploy/select_mission_profile.sh full
# or
bash deploy/select_mission_profile.sh segment1
# or
bash deploy/select_mission_profile.sh segment2
```

After every lifecycle blocker relevant to the winch is corrected, remove propellers, mechanically secure the winch and payload, leave the Pixhawk disarmed, and run the separate actuator test:

```bash
python3 runner.py --mode test-servo7 --transport ethernet --servo7-pwm 2000 --servo7-seconds 65
```

The test captures the actual RC12 neutral, schedules a Pixhawk-side 65-second pulse on output 7, waits for the complete interval, and explicitly returns to the captured neutral. The flight-controller timer independently returns to `SERVO7_TRIM` if the Jetson or Ethernet link dies. The one-shot setup corrects the old `SERVO7_MAX=1900` value by writing and reading back `SERVO7_MAX=2000` while disarmed, so the FC cannot clip the requested 2000 command. `SERVO7_TRIM` must still match the lifecycle test's live RC12 neutral within 5 microseconds. `RC12_DZ=20` means a 20-microsecond dead zone, not a 20-percent neutral offset.

## Flight-blocking configuration checks

Mission preflight intentionally fails closed until all of these are true:

- The fence export has an outer containment polygon and a second exclusion polygon around the red no-fly banner.
- The mapped mission has at least five delivery-search waypoints (increase this for complete 40 m x 30 m camera coverage).
- `FS_GCS_ENABLE`, `BATT_FS_LOW_ACT`, and `BATT_FS_CRT_ACT` each select a non-zero flight-controller action.
- `SERVO7_FUNCTION` permits `MAV_CMD_DO_SET/REPEAT_SERVO`, `SERVO7_MAX >= 2000`, and `SERVO7_MIN <= 1100`.
- `SERVO7_TRIM` matches the captured RC12 neutral within 5 microseconds. The supplied export has `RC12_TRIM=1507`; verify the live neutral rather than assuming 1500.

The production payload configuration uses an ArduPilot-side timed servo cycle. Servo 7 is held at 2000 for 65 seconds and automatically returns to `SERVO7_TRIM` even if the companion process or MAVLink link fails. The companion also explicitly neutralizes the output when the cycle finishes or is interrupted.

## Camera and stream test

On the Jetson, test both cameras and publish the side-by-side MJPEG stream:

```bash
python3 scripts/test_cameras.py --camera both --display
```

Open [http://10.233.44.10:5000/](http://10.233.44.10:5000/) from a laptop on the same network. The boot service page combines live video, readiness, vehicle/SC state, mission phase, blockers, and event logs; `/stream` remains the raw MJPEG endpoint. The stream shows live Pi-camera ArUco/QR/custom-grid overlays plus OAK-D YOLOv6-nano spatial detections, object range, corridor sectors, and banner alignment. Use `--camera downward` or `--camera oakd` to isolate one device, `--no-aruco` to disable Pi-marker detection, and `--duration 30` for a timed 30-second test. The downward camera defaults to the proven external `gst-launch-1.0`/`fdsink` reader, which does not depend on a GStreamer-enabled OpenCV build. Use `--downward-backend subprocess` to require that path or `--csi-sensor-id 1` when the camera is connected as CAM1. Update `config/network.yaml` if the Jetson's Wi-Fi/LAN address changes.

## TX15 SC safety interlock

The default TX15 interlock uses Channel 8. The SC switch must stay below 1300 PWM for three consecutive `RC_CHANNELS` messages before the mission is made ready. After calibration, it must stay above 1700 PWM for three messages before arming can begin. During autonomy, a low/mid switch value, stale RC stream, stale Pixhawk heartbeat, or a mode change outside the expected autonomous mode immediately yields control to `ALT_HOLD`; if that mode is rejected, it attempts `STABILIZE`. Tune the thresholds or mode names only in `config/mission_mapping.yaml` and repeat `--mode test-tx15 --dry-run` after doing so.
