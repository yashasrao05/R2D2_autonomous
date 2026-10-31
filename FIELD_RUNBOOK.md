# R2D2 competition field runbook

This is the go/no-go procedure for the Jetson Orin Nano, Pixhawk 6X Mini, TX15, cameras, winch, operator laptop, and the two official mission segments. A command is flight-ready only when it exits with code `0`; a dry-run is useful for software validation but skips live hardware checks.

## Current configuration status

Production is headless, Ethernet-only, and uses the polygon fence already stored on the Pixhawk by Mission Planner. The Jetson downloads that fence read-only and rejects profile selection unless every mission leg remains inside it. Spreadsheet red-zone coordinates have been removed; red avoidance is downward-camera-only throughout movement. The spreadsheet `Geo Fence` rows remain the delivery/search boundary and are not uploaded as the whole-flight fence because they exclude the Starting Box, QR point, and most corridor anchors.

## One-shot PuTTY setup

Upload the current repository to `/home/r2d2/R2D2_autonomous`. Connect using PuTTY over the Jetson's Wi-Fi, not the Pixhawk Ethernet interface. Keep the aircraft disarmed, remove propellers, set TX15 SC DOWN, and hold RC12 at mechanical neutral. Identify the Pixhawk wired interface with `ip -br link`, then run:

```bash
cd /home/r2d2/R2D2_autonomous
bash deploy/one_shot_headless_setup.sh --ethernet-interface YOUR_PIXHAWK_ETHERNET_INTERFACE
```

To include the secured 65-second winch test in the same run:

```bash
bash deploy/one_shot_headless_setup.sh \
  --ethernet-interface YOUR_PIXHAWK_ETHERNET_INTERFACE \
  --with-winch-test
```

The script refuses to readdress the interface carrying PuTTY. It disables the obsolete `aerothon-mission.service` if present so two controllers cannot start at boot, updates the venv, runs dependency checks, 46 unit tests, all three mock mission profiles, live Ethernet/FC readiness, a finite non-arming TX15 monitor, and both cameras without a desktop. It installs, enables, and starts the boot service only after all earlier gates pass.

## One-time Jetson installation

The repository and venv must be at `/home/r2d2/R2D2_autonomous` and the account must be `r2d2`.

```bash
cd /home/r2d2/R2D2_autonomous
.venv/bin/python3 -m pip install -r requirements.txt
.venv/bin/python3 -B scripts/environment_check.py
bash deploy/install_boot_service.sh
```

The environment check rejects an incompatible Python/OpenCV/DepthAI/pymavlink installation, including the old eight-channel RC override API and a MAVLink library without `mission_type`. The installer runs the same check again, creates a random field-console token, prints it once, installs `r2d2-autonomous.service`, and enables it for boot without starting flight. Save the token on the operator laptop. To display it later on the Jetson:

```bash
sudo sed -n 's/^R2D2_CONTROL_TOKEN=//p' /etc/default/r2d2-autonomous
```

The Pixhawk Ethernet link is `192.168.168.2/24` and the Jetson Ethernet interface must be `192.168.168.10/24`. The laptop-facing address in `config/network.yaml` is `10.233.44.10`; change it if the Jetson's Wi-Fi address changes. Confirm the route before flight:

```bash
ip -br address
ip route get 192.168.168.2
ping -c 3 192.168.168.2
```

## Before fitting propellers

Keep the Pixhawk disarmed, SC DOWN, and RC12 at the actual mechanical-neutral input. Run:

```bash
cd /home/r2d2/R2D2_autonomous
.venv/bin/python3 runner.py --mode competition-check --transport ethernet --lifecycle-seconds 5
.venv/bin/python3 runner.py --mode test-tx15 --transport ethernet
.venv/bin/python3 runner.py --mode test-servo7 --transport ethernet --servo7-pwm 2000 --servo7-seconds 65
.venv/bin/python3 scripts/test_cameras.py --camera both --duration 30
```

The competition check validates all of the following without arming or moving Servo 7:

- the OS Ethernet route and the Pixhawk's UDP-client parameters;
- live heartbeat, GPS, RC, EKF-related telemetry, disarmed state, and configured failsafes;
- fence enable/action/type/maximum altitude and arming checks;
- the Mission Planner polygon fence by read-only MAVLink2 download;
- each named waypoint against the field coordinate with a 3 m tolerance;
- every straight leg for all three profiles at 0.5 m sampling against FC containment and any FC exclusions;
- delivery-search count/location and visual-only red-zone mode;
- Servo 7 limits plus the live RC12 neutral.

Required Pixhawk conditions include non-zero `FS_GCS_ENABLE`, `BATT_FS_LOW_ACT`, and `BATT_FS_CRT_ACT`; enabled polygon fencing with a non-report-only action; `FENCE_ALT_MAX >= 12`; `SERVO7_MAX >= 2000`; `SERVO7_MIN <= 1100`; and `SERVO7_TRIM` matching the measured RC12 neutral within 5 microseconds. Segment 2 does not actuate the winch, but the complete aircraft readiness report still checks it so both official profiles are known-good before competition.

## Pixhawk geofence ownership

Mission Planner owns the live Pixhawk polygon fence. The service and `competition-check` download it without writing FC state. Do not upload the four spreadsheet `Geo Fence` points as the global inclusion fence: they do not contain the recorded takeoff/QR/corridor route and would prevent a valid arming check. `sync-flightplan` remains a development utility but is not used by the current boot configuration.

## Prove arming, hover, and one safe task before the full mission

After `competition-check`, TX15, cameras, and propeller-off checks all pass, use the dedicated live checkout only under the team's approved flight-test process.

Arm, climb to 2 m, prove a stable 10-second hover, LAND, and confirm disarm:

```bash
.venv/bin/python3 runner.py --mode flight-check --checkout-level hover \
  --takeoff-altitude 2 --hover-seconds 10 \
  --upload-manifest config/fc_upload_manifest.yaml \
  --transport ethernet --confirm-live-flight
```

To prove two flight tasks, use `hop`: it performs the same hover, flies only to the validated QR waypoint, returns to home at 2 m, then lands and confirms disarm.

```bash
.venv/bin/python3 runner.py --mode flight-check --checkout-level hop \
  --takeoff-altitude 2 --hover-seconds 10 \
  --upload-manifest config/fc_upload_manifest.yaml \
  --transport ethernet --confirm-live-flight
```

Both levels re-upload/read-back the flight bundle, require battery/GPS/EKF/failsafes/arming checks, require SC DOWN first, and wait for the deliberate SC-UP authorization. Hover fails if drift exceeds 3 m or fewer than 80% of samples remain within 2 m horizontally and 0.8 m vertically. Any internal checkout fault requests LAND. SC DOWN remains an immediate pilot takeover and therefore does not fight the pilot by forcing LAND.

## Software and mock-flight gate

Run before every tagged field build:

```bash
.venv/bin/python3 -B -m unittest discover -s tests -v
.venv/bin/python3 scripts/mock_mission2_sil.py --profile all
```

Then do a restrained propeller-off integration test, followed by a controlled low-altitude flight test under the team's normal flight-safety process. Mock/SIL success is not a substitute for the live `competition-check`.

## Start and monitor the boot service

```bash
sudo systemctl start r2d2-autonomous.service
systemctl status r2d2-autonomous.service --no-pager
journalctl -fu r2d2-autonomous.service
```

Open `http://10.233.44.10:5000/` on the operator laptop. Enter the saved token. The page shows the side-by-side camera stream, link, arm/mode, SC PWM/state, battery, GPS, EKF, selected profile, mission phase, readiness checks, blockers, operator action, and recent events. Full structured logs are stored on the Jetson as `storage/logs/mission-*.jsonl`; routine telemetry is displayed from memory and is not written every poll.

Expected service states:

| State | Meaning / required action |
|---|---|
| `LINK_WAIT` | Waiting for the Pixhawk Ethernet heartbeat; flight is impossible. |
| `BOOT_CHECKS` | Non-arming configuration and hardware checks are running. |
| `CAMERA_CHECK` | Both cameras are opened; live preview begins only after success. |
| `BLOCKED` | A listed check failed. No profile can be selected; correct it and restart the service. |
| `IDLE_SELECT_PROFILE` | Disarmed and ready. Keep SC DOWN and lock Segment 1 or Segment 2. |
| `FIELD_VERIFY` | The Mission Planner fence is downloaded read-only and revalidated; FC state is not changed. |
| `WAITING_TX15_DOWN` | The mission's independent manual interlock is being rechecked. |
| `PREFLIGHT_CALIBRATION` | GPS/EKF/battery/failsafes/cameras/payload requirements are being rechecked. |
| `READY_WAITING_TX15_UP` | Profile is locked and all preparation passed. Clear the field, then SC UP authorizes flight. |
| `MISSION_ACTIVE` | Monitor. Moving SC DOWN immediately requests the tested manual fallback. |
| `COMPLETE` | Landed/disarmed and locked; no automatic second run. |
| `PILOT_CONTROL` | Autonomy yielded after SC/manual/link/mode intervention. |
| `FAULT` | Mission failed and is locked; if airborne, mission fault handling requests LAND. |

The web console has no arm or start endpoint. It only selects and locks a profile while the aircraft is disarmed and fresh RC telemetry confirms SC DOWN. The physical, debounced SC-UP transition is the only path into autonomous arm/takeoff.

## Official run procedure

1. Complete the field, software, hardware, camera, TX15, and winch gates above.
2. Place the aircraft in the configured Starting Box, with SC DOWN and the area clear.
3. Boot/start the service and wait for `IDLE_SELECT_PROFILE` with every check green.
4. Select exactly one official profile in the laptop console:
   - Segment 1: initial marker scan, high corridor bypass, red-safe search/drop, high return, land at home.
   - Segment 2: green corridor, red-safe bypass, blue obstacle corridor, return/land; no marker scan or payload.
5. Wait for verified fence read-back, live cameras, calibration, and `READY_WAITING_TX15_UP`.
6. Perform the final human field-clear/aircraft check. Flip SC UP once to authorize the selected run.
7. If intervention is required, flip SC DOWN. Do not attempt to select another profile during the same service session.
8. After `COMPLETE`, `PILOT_CONTROL`, or `FAULT`, keep SC DOWN, save the JSONL/journal logs, inspect the aircraft, and deliberately restart/reboot for a new official run.

The Jetson-local selector is available if the laptop UI is unavailable:

```bash
bash deploy/select_mission_profile.sh segment1
# or segment2
```

It calls the same authenticated API and therefore obeys the same disarmed, readiness, SC-DOWN, and one-run lock conditions.
