# Module 06 — hybrid monitored navigation

Latest physical-data update: [operator movement and tool measurements](PHYSICAL_MEASUREMENTS_REPORT.md).
This document retains the original hybrid implementation and test results as history.

2026-11-09. This update builds on commit
`fd4f4235979e65ed5e0a37f81a1631aa27e2ad85` on `main` (clean before edits).
No robot/camera was connected and no physical movement, cleaning or firmware
upload was performed. The earlier Module 06 report remains the baseline history.

## Changes

- `navigation/controller.py`: longer bounded segments, live pose checks, measured
  early stopping margins, confirmed STOP handling and short near-target corrections.
- `navigation/adapters.py`: one background camera/Module 04 capture worker. A
  blocked camera read cannot block the controller's bounded fresh-pose wait.
- `navigation/timing.py`: optional direction-specific measured stopping bounds;
  existing timing samples and approximate R references remain untouched.
- `navigation/cli.py`: `movement-calibration add-stop`, and start/close the live
  capture adapter only during explicitly requested physical navigation.
- `navigation/simulation.py`: movement advances during monitoring, with simulated
  STOP latency, separate acceptance/completion, and injectable missing STOP ACK.
- `robot_comm/client.py`: additive `stop_and_wait(result=None)`; existing
  `send_command`, `send_stop`, default safety profile and wire protocol retained.
- `config/navigation_config.json`: default segment cap 1000 ms and fresh-pose wait
  100 ms, with the existing short-pulse cap 500 ms, position ±2.5 cm and heading ±7°.
- `tests/test_navigation.py`, `tests/test_hybrid_navigation.py`: regression and
  hybrid/STOP-race coverage; `README.md` and reports document the update.

Modules 01–05, Dashboard, robot geometry, camera/marker calibration, measured
navigation files, registered background and ESP32 firmware were not changed.
The SIMPLE CAPS, ASCII/LF framing, TCP3333, fixed PWM170/20 kHz, MOVE semantics,
and cleaning commands are unchanged. No unsupported safety command was added.

## Control behavior

With verified stopping data for the selected direction:

1. Read the current Module 04 ground-center pose and compute signed error.
2. Far from the target, send one estimated longer MOVE, capped at 1000 ms by default.
3. Monitor new poses while the timer runs. Check chassis/tool geometry, heading
   and cross-axis drift. Never extend the segment by repeating MOVE.
4. Near the calculated stopping margin, send STOP. Require a fresh `OK,STOP`
   and a PING/PONG response barrier that drains earlier replies.
5. Wait for settling and a newly captured pose. Use short bounded pulses for the
   remaining error, or proceed when within the existing deadband.

Long mode starts when remaining error exceeds stopping margin + twice the stage
tolerance. Its estimated duration uses the direction-specific timing model;
short mode retains the 0.8 correction gain and minimum effective-pulse checks.
Segment duration never automatically uses the firmware's 10000 ms maximum.
Explicit physical caps above 1000 ms require verified stopping data for the
current direction; configuration still must remain below 10000 ms.

Without stopping calibration, the controller uses only the existing short,
monitored pulses (default≤500 ms). It does not invent a physical stopping margin.
Missing movement-rate calibration still blocks that direction entirely. The
current physical timing file has no verified movement/stopping samples, so this
work does not enable physical navigation automatically.

Full commanded sweeps are prechecked, using measured upper speed and coast bounds
when available. Observed inter-frame sweeps are checked during movement. Excess
heading/cross-axis drift triggers confirmed STOP, then the existing bounded
alignment/replanning logic. Unsafe geometry remains a blocking condition.
The route remains Y→X→CLEAN→X→Y→restore heading→verify HOME→READY. Cleaning uses
the existing fixed preset and requires exact `DONE,CLEANING`; returning starts
from the observed post-cleaning pose, not reversed durations.

## Measured stopping margin

For each F/B/L/R direction independently, record verified upper bounds for:

| Field | Meaning |
|---|---|
| `pose_latency_ms` | End-to-end image/pose pipeline latency |
| `stop_latency_ms` | STOP transmission/processing delay until motor output shuts off |
| `maximum_speed_per_second` | Measured upper speed: cm/s for F/B, deg/s for L/R |
| `coast_units` | Measured residual travel/rotation after output shutdown |

The base margin is:

```text
maximum_speed × (pose_latency + stop_latency + maximum observation wait + 50ms poll interval)
+ residual coast
```

Host time elapsed since the current pose's receipt adds a further speed×age
allowance during monitoring. Multiple verified samples use the largest value of
each stopping bound; normal duration conversion still uses direction-specific
median timing samples. These are operator-measured bounds, not guarantees against
changed floor friction, load or network conditions. The100 ms capture-wait policy
and 50 ms client poll interval are software limits, not fabricated physical data.

Inspect/record data without connecting hardware:

```bash
cd /Users/williamwu/natHack
.venv/bin/python main.py movement-calibration show

# Replace ALL placeholders with independently measured upper bounds.
.venv/bin/python main.py movement-calibration add-stop \
  --file config/movement_timing_LATEST.json --direction F \
  --pose-latency-ms MEASURED_MS --stop-latency-ms MEASURED_MS \
  --maximum-speed MEASURED_CM_PER_SECOND --coast-units MEASURED_CM \
  --confirm-measured
```

Repeat independently for B/L/R, using deg/s and degrees for turns. Output is a new
numbered atomic revision; the source and prior observations are not overwritten.
Without `--confirm-measured` a stopping sample cannot enable long mode. Do not
reuse synthetic simulation stopping bounds for physical execution. The existing
R1500 and operator-confirmed R3000 approximate references remain unverified history.

## STOP and failure semantics

`OK,MOVE` is acceptance only. Normal expiry is `DONE,MOVE`. An intentional STOP
produces an aborted MOVE outcome plus confirmed stopping; it is not marked as
normal completion and does not wait for a DONE that firmware will not send.
If DONE wins the race, that history remains completed, but STOP must still be
acknowledged independently before continuing. Old queued DONE replies are drained
before another MOVE, preventing completion of a later action by an earlier event.

Motion requests are prohibited during STOP confirmation. Missing STOP ACK cannot
be replaced by PONG or DONE. A pre-existing unresolved priority STOP is treated as
ambiguous, rather than counting its delayed ACK as confirmation of a new STOP.
Missing ACK, connection failure or uncertain completion faults the task; the real
client closes/latches the session, and the controller prohibits automatic reuse.

Stale/lost localization or an unavailable new camera pose triggers STOP. A
successful STOP allows a PAUSED failure result; an unconfirmed STOP yields ERROR
and a motion lock. No MOVE retries, reconnects, task resumes or repeated cleaning
are automatic. STOP ACK confirms firmware processing, not mechanical rest;
settling and fresh-pose verification remain mandatory.

SIMPLE still lacks a heartbeat watchdog and latched firmware ESTOP. Capture and
STOP timing on actual hardware remain unvalidated. This is software control with
bounded moves, not a hardware emergency-stop system or collision guarantee.

## Simulation and tests

Use new output paths; these commands do not open a camera or real connection:

```bash
.venv/bin/python main.py simulate-navigation --surface FLOOR --output-dir data/hybrid_floor_new
.venv/bin/python main.py simulate-navigation --surface WALL --output-dir data/hybrid_wall_new
.venv/bin/python -m pytest -q tests/test_navigation.py tests/test_hybrid_navigation.py
.venv/bin/python -m pytest -q
```

The deterministic plant advances throughout each MOVE and applies 40 ms synthetic
STOP latency. All movement/latency bounds are explicitly SIMULATION-only.

| Same synthetic scenario | Short-pulse fallback | Hybrid | Result |
|---|---:|---:|---|
| Floor complete round trip | 42 MOVEs | 26 MOVEs (18 long segments; 3 early STOPs) | READY |
| Wall complete round trip | 11 MOVEs | 7 MOVEs (4 long segments) | READY |

Floor final simulated home error: 1.627 cm / 3.2385°. Wall: 0.869 cm / 0°.
The wall scenario finishes its long timers before the stopping threshold, then
uses short corrections; it does not require an early STOP. Both keep the existing
home tolerances and all boundary protections. Reduced segment counts are not
physical speed/accuracy measurements.

Evidence: `data/module06_hybrid_floor_final/` and
`data/module06_hybrid_wall_final/`, each with summary, plan, command trace and
pose/progress events. Earlier simulation evidence was not overwritten.

Full suite: **897 passed, 1 existing warning in 41.27 s**. This includes the 27 new
hybrid cases, all 49 earlier navigation cases and the existing module regressions.
The warning is the intentional legacy floor-only calibration test. Coverage
includes early STOP, delayed MOVE ACK, DONE/STOP orderings, missing ACK even after
DONE/PONG, stale/lost/blocked camera reads, old frames after settling, overshoot
correction, correction budgets, sweep boundaries and complete floor/wall lifecycle.
All TCP tests bind loopback only.

## Recovery and next work

Pre-edit verified backup:
`backups/module06_hybrid_before_20261109T085951Z/before.zip`, with SHA-256 manifest
and the starting commit. It preserves tracked state and existing JSON/JSONL
measurements/evidence, camera calibration and registered background. Recordings
were left in place. Post-edit protected-file verification is stored alongside it.
All 281 backup entries were verified; all 190 protected configuration/data/vision/
firmware files matched their pre-edit hashes (navigation policy settings excluded).
Generated evidence/backups are ignored by Git; implementation/tests/docs are
committed separately. The final commit hash is reported in the chat.

Still needed before physical validation: separate F/B/L/R timing, independently
measured latency/maximum speed/coast bounds, actual cleaning-tool geometry,
settling/clearance validation, and explicit operator authorization for any powered
test. This round performs no powered test and makes no physical navigation PASS claim.
