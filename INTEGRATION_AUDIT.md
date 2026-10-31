# Mission 2 integration audit

Audit basis: current companion code, supplied Pixhawk parameter export, official field PDF, and the five event-field marker/corridor photographs. This is a software/mock verification, not a claim of airworthiness.

## Critical findings

| Finding | Disposition |
|---|---|
| EKF flags were interpreted as error bits, so valid estimator flags could mark EKF unhealthy. | Fixed: positive validity bits are now required explicitly. |
| Start-tag collection ignored AprilTag/QR detections because it accepted only custom matrices. | Fixed: five consecutive identical AprilTag, QR, or custom-grid observations are required and atomically persisted. The reference cannot be overwritten. |
| Payload could be released immediately after requesting 5 m, with a 0.25 m visual threshold and no stable altitude gate. | Fixed: five consecutive samples must satisfy 5.0 m +/- 0.3 m and body-plane error below 0.15 m. A lost or different tag aborts. |
| A one-shot Servo 7 command could leave the winch running after a Jetson crash. | Fixed in production configuration: one FC-side `DO_REPEAT_SERVO` cycle holds the requested output for 65 s, then returns to `SERVO7_TRIM`; the companion also neutralizes explicitly. |
| Supplied `SERVO7_MAX=1900` clips the requested 2000 us command. | Flight-blocking preflight check added. Set `SERVO7_MAX` to at least 2000 and bench-confirm the physical direction. |
| Supplied `FS_GCS_ENABLE=0`, `BATT_FS_LOW_ACT=0`, and `BATT_FS_CRT_ACT=0` provide no FC action on companion/battery loss. | Companion now sends a 1 Hz GCS heartbeat and preflight rejects zero actions. Configure appropriate non-zero actions in Mission Planner. |
| GPS transit can cross the red no-fly zone; the visual escape vertical sign was also reversed. | Sign fixed. Spreadsheet red coordinates were removed by operator decision; downward-camera red detection now runs during every movement phase, while the Mission Planner containment fence remains an independent FC barrier. |
| Only two delivery-search waypoints are supplied for the 40 m x 30 m search area. | Production preflight requires at least five. Actual camera-footprint coverage should use more points and must be generated from event-day coordinates. |

## High findings

| Finding | Disposition |
|---|---|
| Corridor completion was a fixed 45 s timer. | Fixed: fresh `LOCAL_POSITION_NED` is mandatory and completion is latched at 7.5 m fused local/GPS travel, with a 30 s failure timeout. |
| OAK spatial coordinates remained in optical right/down/forward coordinates. | Fixed with the specified 45-degree optical-to-body FRD rotation. |
| Body velocity used `MAV_FRAME_BODY_NED`. | Changed to `MAV_FRAME_BODY_OFFSET_NED`; velocity setpoints are refreshed at 10 Hz during camera waits without `STATUSTEXT` spam. |
| Blue segmentation accepted any blue pixels, including sky, and merged top/bottom banners. | Fixed with saturation, fill, edge, and aspect-ratio filtering; top and bottom banners are retained separately and combined by area weighting. |
| Invalid black-wall stereo pixels were treated as 99 m clear. | Wall steering now requires adequate valid-depth density. Blue-chicane ranging preferentially uses bright, low-saturation standee regions. |
| CSI/OAK waits could starve MAVLink servicing. | External CSI reads service the safety callback every 50 ms; OAK waits do the same. GStreamer/OpenCV queues are capped at one newest frame. |
| Decoys were not remembered. | Rejected IDs are persisted with GPS and local coordinates; search observations require a 3-of-5 vote. |

## Medium/residual risks

- The supplied marker photos decode successfully as AprilTag 36h11 IDs 100, 200, 300, 400, and 500. The PDF calls them QR codes, so both decoders remain enabled.
- `RC12_DZ=20` means a 20-microsecond deadzone, not a 20 percent neutral offset. The export reports `RC12_TRIM=1507`; production captures live RC12 neutral and requires `SERVO7_TRIM` to be within 5 us for the FC-side timer fallback.
- Camera yaw is configured as `downcam_yaw_deg: 0.0`. Confirm the Pi camera's image top physically points toward the nose; otherwise enter the measured yaw.
- The OpenCV/V4L2 fallback can still block inside a driver call. The external GStreamer backend is the intended Jetson path and must be soak-tested on the actual image.
- OAK-D v3 pipeline construction, passive-stereo behavior, color thresholds, propwash, GNSS multipath, and controller tuning cannot be proven by desktop SIL. Run propellers-off camera/servo/fallback tests and a restrained netted/tethered test where permitted.
- Mission roles now use the recorded field coordinates. Production downloads and validates the Mission Planner fence read-only instead of overwriting it from the Jetson.

## Verification performed

- 46 unit/integration tests pass, plus all three parent/child mock-flight profiles.
- `scripts/mock_mission2_sil.py` passes the full production state machine with generated AprilTag frames: 39 downward frames, 14 OAK frames, a rejected/persisted decoy, both distance-gated corridors, one release, LAND, and disarm.
- The mock emitted 54 local log messages, below its 200-message anti-flood limit. No MAVLink connection, camera, motor, or Servo 7 output is opened by the harness.
