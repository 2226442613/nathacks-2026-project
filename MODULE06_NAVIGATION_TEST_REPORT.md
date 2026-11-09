# Module 06: observed heading, verified localization and navigation-only test

2026-11-09. Implementation and validation used simulation and loopback endpoints
only. No camera, physical robot, cleaning actuator or firmware upload was operated.

## Preservation and changes

Started on `main`, clean, at `258fc3583059ed4fe377f94fd19aa3da02620a13`.
Existing work was already committed. Before editing, additionally saved
`backups/navigation_only_before_20261109T102925Z/before.zip` and `sha256.json`.
All 180 archive entries were checked; all 44 protected configuration/data/firmware
files match the starting hashes (`verification_after.json`). This includes ignored
camera calibration and registered background data. No calibration was refitted,
overwritten or given new measurement attestations. Dashboard, ESP32 firmware,
robot settings and cleaning tool configuration are unchanged.

Changed files:

- `config/movement_timing.json`: operator-observed signed heading response only.
- `navigation/timing.py`: consume confirmed direction signs; contradictory data blocks.
- `vision/robot_tracking.py`, `vision/robot_cli.py`,
  `config/robot_pose_v1.schema.json`: scoped current-context verification and clear overlay.
- `navigation/adapters.py`: accept fresh VERIFIED Module 04 poses only.
- `navigation/controller.py`: dedicated navigation-only round trip; shared existing
  return verification; faults lock the session.
- `navigation/cli.py`, `main.py`: supervised test entry point and explicit confirmations.
- `navigation/simulation.py`: correct synthetic heading signs and optional TX/RX logging.
- `robot_comm/client.py`: require cleaning start before DONE and reject completion
  on an already failed connection.
- `tests/test_navigation_only.py`, `tests/test_navigation.py`,
  `tests/test_operator_measurements.py`: focused coverage and updated heading expectations.
- `README.md`, `MODULE06_REPORT.md`, this report: current instructions; old results remain history.

## Heading and movement

| Command | Signed Module 04 response | Approximate timing retained |
| --- | --- | --- |
| F | forward displacement positive | 16 cm/s; 62.5 ms/cm |
| B | forward displacement negative | 18.5 cm/s; 54.054 ms/cm |
| L | theta decreases | 4000 ms/90 degrees |
| R | theta increases | 3000 ms/90 degrees |

Positive heading correction selects R; negative selects L, with wraparound.
Historical R1500 and R3000 references remain unchanged. These are approximate
physical estimates, not invariant speeds or measured stopping upper bounds.

Timed MOVE, live pose monitoring, settling and short corrections remain. The
configured long-segment ceiling is 1000 ms, never automatically 10000 ms. Without
measured pose latency, STOP latency, maximum speed and coast bounds, current physical
configuration falls back to monitored pulses at most 500 ms. Distance/angle stopping
margins and longer early-braking segments require those actual measurements;
synthetic stopping data never fills physical settings. Camera loss or unsafe live
geometry triggers STOP even in the short-pulse fallback.

## Why UNVERIFIED appeared

The saved map is valid: 4 training and 3 independent validation points, RMS
1.04459768 cm, maximum 1.25286084 cm, tolerance 2 cm. Its camera hash matches
`config/camera_config.json`. Resolution 1280x720, marker black-square size 6 cm,
mounting height 16.6 cm, robot dimensions 26x15.2 cm and marker offsets match the
completed evidence in `data/robot_plane_points_003.json`.

The old tracker assigned UNVERIFIED unconditionally and its schema allowed no
VERIFIED result. Also, `track-robot` needs an explicit `--marker-plane` argument;
without it the method is approximate floor-ray intersection. The empirical-method
label describes the model and is not itself a validation failure.

The completed measurements verify **marker ground projection**, not independently
the robot ground-center conversion, heading alignment or the current camera pose.
Their original `heading_alignment_verified=false` and
`chassis_reference_conversion_supported=false` metadata are preserved. The old
`data/robot_plane_points.json` is the interrupted 4+1 snapshot; it cannot verify the
finished map.

New runtime verification checks the actual map/evidence points, calibration hash,
frame size, height, marker size/ID/dictionary, offsets, heading and chassis dimensions.
The operator must confirm the fixed camera and independently checked ground-center
offsets/marker-forward alignment. A live visible observation must also pass the
existing marker-scale consistency limit. Supplied moved/compensated/override
registration cannot confer VERIFIED. The overlay distinguishes mapping PASS from
current-context VERIFIED/UNVERIFIED. `actionable` and `motion_authorized` remain false.

This scoped verification relies on the operator's fixed-camera confirmation; the
navigation adapter does not continuously certify camera immobility. Fresh host
receipt times are not sensor exposure-time guarantees. If the camera is moved or
reframed, stop and recheck calibration/context before any new session.

Optional localization-only check, after completing those operator checks:

```bash
.venv/bin/python main.py track-robot --camera 0 --width 1280 --height 720 \
  --marker-plane config/robot_marker_plane.json \
  --verification-points data/robot_plane_points_003.json \
  --confirm-fixed-camera --confirm-ground-reference
```

## PC / ESP32 responsibilities

Normal task: record verified HOME -> navigate Y then X -> verify actual tool arrival
-> register TARGET once for this task -> ARRIVED -> wait for cleaning start and
the exact `DONE,CLEANING` -> settle and obtain a new verified pose -> return X then Y
-> restore original heading -> verify HOME -> READY.

The PC sends no pump, servo, sponge or cleaning-motion control commands. Existing
Module 05/ToolApproach calculations are read-only reachability/clearance checks
for the firmware's fixed cleaning preset, not a second physical cleaning controller.
All actual cleaning sequencing belongs to ESP32. The simulator models ESP32 behavior
solely for software tests. TARGET surface/fluid mapping remains FLOOR->GROUND,
WALL->WALL, RED_SPILL->BLOOD, NON_RED_SPILL->OTHER.

START may precede or follow OK,ARRIVED, but must precede DONE,CLEANING. Fault/ERR/STOP,
missing start, timeout, connection loss or unavailable post-clean pose blocks return
and READY. No automatic retry, replay, return or session recovery. A new session
requires explicit operator initiation after inspection. Dashboard only displays progress.

## Supervised navigation-only command

Startup localization fix: `navigate-test` continues to use Module04's
`build_robot_tracker`, the saved `config/robot_marker_plane.json`, and matching
`data/robot_plane_points_003.json` evidence. The previous adapter terminated its
capture worker on the first missing ArUco observation, even with a valid mapping.
It now allows up to two seconds for initial current, VERIFIED empirical
ground-center acquisition before TCP connection. Only initially missing/stale
observations are retried; mapping/context errors, unverified coordinates and
camera errors fail closed. After acquisition, tracking loss still fails immediately.
`wire.log` records marker-not-visible, stale pose, mapping/context errors,
localization verification failures, startup timeout and successful acquisition.
No floor-homography fallback is accepted. Calibration, firmware and motion logic
are unchanged. Focused regression tests use synthetic frames and loopback servers;
no hardware was operated.

`navigate-test` defaults to simulation. It records fresh HOME, preflights both the
outbound Y/X and return X/Y swept routes, verifies destination, restores starting
heading and verifies HOME. No TARGET or ARRIVED calls, target ingestion or cleaning
planner are used. Arrival uses 2.5 cm per axis / 7 degrees; existing HOME radial
3 cm / 10 degrees limits remain. Current chassis and measured LEFT 5 cm overhang
are included. Runtime geometry and fresh-frame checks continue throughout movement.

Before physical use: secure the unchanged camera, check black-square size/height,
actual robot center/heading, safe HOME/rotation space and the **whole** route with
no unmodeled obstacles. Confirm every parked tool, nozzle and any STOP-induced
servo homing stays within the conservative body/side outline (or the separately
configured floor-tool outline). If it does not, measure/configure its collision
geometry first. Unknown floor contact/cleaning reachability remains CONFIGURATION_REQUIRED;
this navigation-only workflow does not claim floor-cleaning coverage.

Use the actual camera index. Verify exact SIMPLE firmware CAPS before movement:
`CAPS,NATHACK_V2_SIMPLE,MOVE_DURATION_MS,FIXED_PWM_170,STOP,TARGET,ARRIVED`.
SIMPLE has no heartbeat watchdog or latched ESTOP; Wi-Fi STOP is not a hardware
emergency stop. Keep the operator's independent power cutoff available. STOP may
home firmware servos; account for that motion in the clearance confirmation.

After the checks, the following zsh commands ask for an operator-selected destination
in calibrated cm. They were **not run against hardware** during development:

```bash
cd /Users/williamwu/natHack
read -r "HG_ROBOT_IP?ESP32 IP: "
read -r "HG_TEST_X_CM?Safe destination X (cm): "
read -r "HG_TEST_Y_CM?Safe destination Y (cm): "
.venv/bin/python main.py navigate-test --mode physical \
  --host "$HG_ROBOT_IP" --port 3333 \
  --x-cm "$HG_TEST_X_CM" --y-cm "$HG_TEST_Y_CM" \
  --camera 0 --width 1280 --height 720 \
  --verification-points data/robot_plane_points_003.json \
  --authorize-motion --confirm-fixed-camera --confirm-ground-reference \
  --confirm-static-tools \
  --output-dir "data/navigation_test_$(date +%Y%m%d_%H%M%S)"
```

Omitted destination heading means the initial observed heading. Each run requires
a new output directory. `wire.log` preserves full TX/RX, `progress.jsonl` includes
`navigation_only=true`, and `summary.json` records arrival/home results. Navigation-only
READY does not pretend that a hospital cleaning task was completed; cleaning-done
remains false and the task uses an isolated `navigation-test:` identifier.

## Validation

Full regression: **946 passed**, one existing intentional legacy floor-only warning.
No dependency added. Focused tests include signed timing/wraparound, synthetic
feedback using the physical rates, actual saved-file compatibility, missing context,
marker/settings/height/frame mismatches, evidence preservation, stale/lost/unverified
poses, route clearance, early STOP / DONE races, missing STOP ACK, no fault recovery,
cleaning start/DONE ordering and link failure, no return on uncertain cleaning,
and full existing ground/wall cleaning regressions.

```bash
.venv/bin/python -m pytest -q
.venv/bin/python main.py navigate-test --x-cm 64 --y-cm 36 --output-dir data/nav_sim_new
```

Saved software-only evidence: `data/module06_navigation_only_20261109` (READY, 24
movements, no TARGET/ARRIVED), `data/module06_floor_20261109_final` (READY, 26),
`data/module06_wall_20261109_final` (READY, 7). These ignored generated logs are local
evidence; they are not measurements of physical navigation accuracy.
