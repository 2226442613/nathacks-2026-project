# Module 06 — closed-loop navigation MVP

Current heading/context verification and supervised navigation-only workflow:
[navigation test update](MODULE06_NAVIGATION_TEST_REPORT.md). The UNVERIFIED-only
tracker behavior described below is historical; calibration files remain unchanged.

Latest update: [hybrid monitored navigation](MODULE06_HYBRID_REPORT.md) adds
bounded longer segments, measured early STOP and short corrections. This document
retains the original implementation/test results as baseline history.

Implementation date: 2026-11-09. This report supersedes earlier assumptions about
the **current** robot's protocol. The older Module 07 safety contract and archived
firmware remain available and unchanged by default. No physical robot was
connected, moved, cleaned or flashed during this work.

## Scope and preserved state

Inspected the repository at `d07095732f24ca947d253062b5df0801cdebecde`, branch `main`,
with a clean working tree. There was no Module 06 controller to extend. Read the
actual `NatHack_Robot 3` firmware and supplied computer integration document.
Added a small navigation package using existing Module 01/03/04/05 interfaces.

Before edits, made and SHA-256 verified
`backups/module06_before_20261109T074731Z/before.zip` with `sha256.json`.
It includes the ignored camera calibration and registered legal background, as
well as tracked source, robot mapping, and both measured-points snapshots.
Modules 01–05 source, existing robot/camera/tool configurations, legal background,
marker mapping and measurements, and repository firmware were not edited.
Final archive verification checked all 135 entries; all 54 protected existing
configuration/data/vision/firmware files matched their pre-edit hashes. Results
are in the backup directory's `after_verification.json`.
The earlier Module 04 physical PASS remains operator evidence; this implementation
does not change its `UNVERIFIED` or physical authorization metadata.

## Files

- New: `navigation/{__init__,controller,geometry,timing,adapters,simulation,cli}.py`.
- New: `config/navigation_config.json`, `config/movement_timing.json`.
- New: `tests/test_navigation.py`, this report.
- Modified: `main.py` (register three commands).
- Modified: `robot_comm/{protocol,client,simulator,cli}.py` (explicit SIMPLE profile,
  asynchronous cleaning wait, optional pose-monitor callbacks; old default retained).
- Modified: `dashboard/{cli,model,service}.py`, `dashboard/static/app.js`
  (optional, display-only task progress).
- Modified: `README.md` (entry points and scope).

## Architecture and task lifecycle

`CameraPoses` calls the existing Module 04 tracker with the saved empirical
fixed-height marker-plane mapping and takes its **ground-center** position and
heading. It requires a currently visible observation and rejects lost/invalid
localization. It does not use marker pixel center or a retained last-known pose.
Every observation receives a new read sequence and monotonic receipt timestamp.
The tracker remains unfiltered (`smoothing_alpha=1`, already configured).

`ToolApproach` uses Module 05 reachability and measured contact/spray geometry.
The existing planner remains declarative and non-authorizing. SIMPLE firmware
cannot execute arbitrary Module 05 strokes: this MVP accepts only a target that
fits a **single fixed preset** inside both measured spray and sponge coverage.
It retains the original Module 05 approach and records the adapted preset approach.
Larger/multiple-pass targets are blocked rather than declared cleaned.

`Navigator` records the current home pose. Outbound stages are **Y then X**, then
final heading and actual tool-reach verification. It chooses F/B using the actual
heading and signed displacement. It can reverse instead of turning at an unsafe
wall-adjacent location. There are no diagonal shortcut commands.

Every pulse performs: current pose → signed error → calibrated duration → full
swept-geometry check → one MOVE → acceptance → asynchronous completion → settling
delay → new post-settling pose → error recalculation. An ACK is not completion;
completion is not arrival. The client prevents overlapping MOVE commands.
Camera monitoring also runs while waiting for motion/cleaning completion.

Cleaning requires `OK,TARGET`, then `ARRIVED`/`OK,ARRIVED`. The receive event cursor
accepts `EVENT,CLEANING_STARTED` before or after the ARRIVED ACK. Only the exact
`DONE,CLEANING` event establishes firmware cleaning completion. Fault/STOP events
take precedence over DONE. A timeout/disconnect is unknown, never an automatic retry.

After cleaning, obtain the actual settled pose and return **X then Y**, restore
the original heading, and verify radial home distance and heading. Neither elapsed
outbound timings nor inverse command replay are used. READY requires both cleaning
DONE and verified home; no nonexistent HOME command is sent. This does not certify
cleaning effectiveness.

State logs include planning, aligning/moving/correcting each axis, verifying
arrival, cleaning/waiting, returning/correcting, restoring home heading, verifying
home, COMPLETE/READY, or PAUSED/BLOCKED/ERROR. Any failure attempts priority STOP
and terminates the run. Restart/reconnect never automatically resumes a task.

## Timing, tolerances, drift and geometry

Duration estimate is `round(abs(error) * direction_ms_per_unit * correction_gain)`,
bounded by a configurable per-pulse maximum. Each direction uses the median of
its own **verified** measured samples. Turn direction uses the observed signed
Module 04 heading change, not an assumed L/R sign. PWM stays 170; it is not speed.

Initial **unvalidated demo settings**, editable in the separate navigation config:

| Setting | Default |
|---|---:|
| Per-axis error / heading deadband | 2.5 cm / 7° |
| Home radial distance / heading | 3 cm / 10° |
| Wall-adjacent reference / tolerance | Y=13 cm / 2.5 cm |
| Settle / maximum pose receipt age | 0.5 s / 0.5 s |
| Maximum pulse / minimum turn / minimum translation | 500 / 100 / 40 ms |
| Proportional correction gain | 0.8 |
| Attempts per alignment or axis / no-progress count | 24 / 3 |
| Minimum position / heading progress | 0.2 cm / 0.5° |
| Cross-axis drift threshold / ordered replans | 3 cm / 2 |
| Clearance margin / cleaning wait | 1.5 cm / 30 s |

These loose deadbands avoid endless micro-correction. A too-small effective pulse,
repeated lack of progress or exhausted attempt/replan budget blocks the task.
Home axis correction uses a smaller intermediate tolerance so the radial check
can still meet 3 cm. Actual tool coverage and collision clearance take precedence.

Full chassis (26×15.2 cm) and the configured permanent floor sponge are checked
at every observation, translation sweep and rotation sweep. Sampled sweeps include
additional inflation between samples. The geometry API also supports explicit
rectangular obstacles; the MVP CLI does not infer obstacles from camera imagery.
Timing-based predicted sweeps are estimates: real overshoot, tool protrusions and
camera latency require operator validation, not a collision-proof guarantee.
The operator's physical safety margin must cover unmodeled installed protrusions.

Cross-axis drift is logged from actual poses. Excess drift restarts the same
ordered route from the observed pose within the replan budget; unsafe sweeps stop.
Final pose is checked again after final heading adjustment.

Wall cleaning stays parallel to the wall and stationary during the firmware preset.
Measured tool geometry must agree with Y≈13. Tool-contact checking is tighter than
generic arrival: ±0.5 cm around the configured contact gap and ±2° parallel heading.
If generic navigation tolerance does not establish actual contact, the MVP blocks
instead of loosening coverage or inventing an additional maneuver. A rotation at
Y=13 can intersect the wall even if the current parallel pose is legal. Such a
route is rejected. The wall demo starts parallel and returns using reverse motion;
it is not evidence that arbitrary wall-to-floor rotations are safe.

## Current physical protocol

Explicitly select `--firmware-profile simple` for communication CLI. The navigation
physical entry point selects SIMPLE explicitly. Exact required CAPS:

```text
CAPS,NATHACK_V2_SIMPLE,MOVE_DURATION_MS,FIXED_PWM_170,STOP,TARGET,ARRIVED
```

TCP3333, ASCII, newline terminated. Fixed PWM170/20 kHz. Supported:

| Request | Reply / event |
|---|---|
| PING | PONG |
| CAPS | Exact line above |
| STATUS | STATUS,<cleaning state> |
| MOVE,<F\|B\|L\|R>,<1..10000 ms> | OK,MOVE, then DONE,MOVE only on normal expiry |
| TARGET,<GROUND\|WALL>,<BLOOD\|OTHER> | OK,TARGET |
| ARRIVED | EVENT,CLEANING_STARTED and OK,ARRIVED; later DONE,CLEANING |
| STOP / ABORT | OK,STOP; cancels movement, target and cleaning |

Busy/invalid requests return ERR replies. STOP/fault/disconnect interruption never
becomes successful MOVE completion. Cleaning hard-limit events are
`EVENT,CLEAN_MOTOR_TIMEOUT` and `EVENT,PUMP_TIMEOUT`; the host faults the session.
Known TCP/Wi-Fi disconnect stops motors and pumps and returns servos to 180°.
STOP can therefore cause servo homing motion; it is not a hardware emergency stop.
Normal ground cleaning pumps 1200 ms and runs F350/B350 three times, all at PWM170.
Independent cleaning motor/pump limits are 600/1700 ms. Wall spray lift90°,
return180°, sponge180→0→180 three times; no MOVE during wall cleaning.

**SIMPLE has no HEARTBEAT, SAFETY, ESTOP or RESET_ESTOP, no silent-link watchdog,
and no latched firmware emergency stop.** The SIMPLE client sends none of these
commands, including on failure; it attempts STOP. It has no heartbeat thread.
Do not use the old watchdog/latched-ESTOP bench procedure on this firmware.
The old `safe_v1_1` profile remains the default with its existing physical arming,
PWM170 authorization, duration limit and TARGET/ARRIVED restriction. SIMPLE is a
separate explicit operator-selected profile; CAPS is compatibility checking,
not a safety certification. Choosing physical navigation starts a task; it is
not a communication-only command.

## Movement measurements needed and calibration CLI

The operator confirmed the latest approximate **R3000 ms≈90°**. Both that reference
and the older R1500 ms≈90° remain unverified history in `movement_timing.json`.
Neither is silently selected as precise calibration. Physical samples are empty;
F/B/L and the signed Module 04 R heading change remain to be measured. SIMULATION
uses separate synthetic F40/B45 ms/cm and L/R3000/90 ms/degree; never physical data.

Provide F and B independently: actual duration and traveled centimeters, ideally
two samples each. For L/R provide duration and signed measured heading change, or
Module 04 before/after ground-center pose records. Record settling time, reliable
minimum pulses and a safe single-action limit on the actual surface/load.

```bash
cd /Users/williamwu/natHack
.venv/bin/python main.py movement-calibration show

# EXAMPLE FORMAT ONLY: replace duration/change with an actual measured sample.
# These commands record measurements and never connect or move the robot.
.venv/bin/python main.py movement-calibration add \
  --file config/movement_timing.json --direction R --duration-ms 3000 \
  --before-pose data/turn_before.json --after-pose data/turn_after.json \
  --confirm-measured

# Alternative when measured signed values are available:
.venv/bin/python main.py movement-calibration add \
  --file config/movement_timing_001.json --direction F --duration-ms 500 \
  --distance-cm 12 --confirm-measured
```

The final example's 12 cm is illustrative, not a measured rate. F is positive
body-forward distance; B negative. Angles are signed normalized Module 04 changes
below 180°. A physical right turn may have a negative heading change; check actual
poses rather than assuming it. `--angle-deg` is available instead of pose files.
Without `--confirm-measured` a sample stays unverified and cannot drive navigation.
Before/after pose pairs must share calibration/marker context and be chronological;
gross translation during turning or heading/cross drift during straight samples
is rejected. Adding measurements writes numbered, exclusive, atomically completed
JSON revisions; the source file and earlier samples are never overwritten. Use
the printed new revision path for subsequent additions and execution.

Tool measurements: floor sponge contact length/width and center offsets from
ground-center; actual nozzle impact center/coverage/orientation; installed status,
clearance and uncertainty. For wall use measured side/protrusion/compression,
contact width/height range, spray impact width/height range and safe distances.
Mechanical mount dimensions alone do not establish contact or spray coverage.
Keep unknown fields unconfigured. The existing physical tool template remains
untouched and currently prevents a physical cleaning task.

## Reproducible simulation and Dashboard

All output directories must be new. These commands open no camera or network:

```bash
.venv/bin/python main.py simulate-navigation \
  --surface FLOOR --output-dir data/module06_floor_new
.venv/bin/python main.py simulate-navigation \
  --surface WALL --output-dir data/module06_wall_new
```

Evidence: `progress.jsonl`, `summary.json`, `plan.json`, `commands.json`. The plant
deliberately executes 85% of the initial model's movement/rotation, exercising
fresh-pose corrections; tests separately inject overshoot, drift, loss and errors.

Optional display-only Dashboard:

```bash
.venv/bin/python main.py dashboard --mode demo \
  --calibration data/module03_demo/calibration.json \
  --navigation-events data/module06_floor_new/progress.jsonl
```

Step the existing demo to introduce target `demo:0:1`, matching the default task
key. Only Navigating/Aligning/Cleaning/Returning/Ready/Failed is presented; no
robot telemetry or control panel is added. A completed replay displays Ready;
an incrementally written log shows live progress. Cleaning DONE by itself leaves
the task IN_PROGRESS/Returning. Ready requires `home_verified` and cleaning DONE.
Physical progress must match Dashboard calibration identity and target record key.
The Dashboard remains non-actuating and never starts or resumes navigation.

## Physical prerequisites and later commands — NOT executed

First isolate actuator power for communication-only checks. Set the real host:

```bash
.venv/bin/python -m robot_comm robot-client --mode physical \
  --firmware-profile simple --host ROBOT_IP \
  --command PING --command CAPS --command STATUS --listen 1
```

This command does not request MOVE or cleaning. It cannot test a nonexistent
SIMPLE watchdog or ESTOP rejection. Verify CAPS against the actual flashed unit.

Before later operator-authorized powered navigation: current unobstructed camera,
saved matching calibration/resolution/marker geometry, current Module 04 poses,
separately verified direction timing, validated PHYSICAL tool file and margins,
safe home/rotation space, refreshed Module 03 target snapshot, and an operator
able to disconnect actuator power. Validate settling/latency and small bounded
pulses before a task. Do not run Dashboard LIVE and navigation against the same
camera simultaneously unless the camera source supports shared access.

```bash
# Later, powered task execution; replace placeholders and use a NEW output path.
.venv/bin/python main.py navigate-task --host ROBOT_IP \
  --targets data/current_targets.json --target-id 1 \
  --hardware config/cleaning_hardware_measured.json \
  --timing config/movement_timing_LATEST.json \
  --camera 0 --width 1280 --height 720 \
  --output-dir data/module06_physical_new
```

The target snapshot must be continuously refreshed externally (at most 2 seconds
old, active, matching calibration); it is checked again before cleaning. Changed
category/geometry blocks the task for replanning rather than silently substituting
an old target. This MVP does not add a camera-sharing/target-stream service.
The saved physical tool template is still empty; do not use synthetic demo hardware.
Physical runs log raw communication to a new `wire.log` as well as progress/result.

Freshness uses successful synchronous camera read receipt, not a sensor exposure
timestamp; OS/camera buffering and capture latency still need physical validation.
Independent 350 ms cleaning strokes belong to firmware; the PC checks estimated
normal and 600 ms fault-bound travel against the arena before ARRIVED, but cannot
camera-correct each internal stroke. It observes actual pose after DONE,CLEANING.

## Verification and evidence

Navigation tests cover axis/heading undershoot and overshoot, wraparound, deadband,
progress/attempt/minimum-pulse limits, ordered drift replanning, stale/lost/reused
frames, camera loss during active MOVE/cleaning waits, chassis/tool sweep collision,
wall parallel operation, direction-specific timing and immutable history, busy and
connection errors, ACK/DONE order, asynchronous faults, cleaning timeout/disconnect,
pump/motor hard limits, servo home, home verification and complete FLOOR/WALL cycles.
The old safety profile tests remain unchanged.

```bash
.venv/bin/python -m pytest -q tests/test_navigation.py
.venv/bin/python -m pytest -q
```

Navigation-specific: **49 passed**. Full suite: **870 passed, 1 warning in 44.18 s**.
The warning is the existing intentional legacy floor-only calibration test.
Dashboard progress behavior is covered by Python integration tests. The small
JavaScript display changes were reviewed, but no browser rendering test was run.
Local TCP tests bind loopback only; deterministic navigation simulations use no
TCP or camera. Native parser tests exercise the archived firmware with host stubs,
not the physical ESP32.

Final FLOOR simulation: READY, 42 pulses, home (32,29,0°), measured simulated final
(33.2643,30.0089,3.2385°); position error ≈1.62 cm. WALL: READY, 11 pulses, home
(32,13,0°), final (32.8689,13,0°), error ≈0.87 cm, no L/R commands. These are
synthetic simulation results, not physical accuracy claims.
Evidence directories: `data/module06_floor_final` and `data/module06_wall_final`.

The final Module 06 commit is reported in the chat; retrieve it using
`git log -1 --format='%H %s'`. Ignored backups and generated evidence remain local;
source, defaults, tests and this report are version controlled.
