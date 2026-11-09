# Autonomous navigation MVP — 2026-11-09

## Delivered behavior

`auto-clean` now selects RED_SPILL before NON_RED_SPILL, chooses a collision-checked
front-facing parking pose with a nominal 5 cm bumper-to-contour gap, navigates with
bounded MOVE commands and fresh camera feedback, verifies observed arrival,
sends TARGET then ARRIVED, waits for DONE,CLEANING, obtains a new pose and continues
to the next target. It does not force a HOME trip between targets.

The original Y→X / X→Y round trip remains available with `--return-home`.
`--navigation-only` retains its supervised round-trip behavior and never invokes
cleaning. Dashboard remains display-only and its code/layout was not modified.

## Workflow assessment

PASS below means software validation, not physical-navigation validation.

| Stage | Result | Evidence / limit |
| --- | --- | --- |
| Single live camera capture / failure cleanup | PASS | Injected camera tests; production still uses the existing CameraManager/CameraPoses |
| ArUco robot pose / configured center and heading | PASS | Actual decoder driven by generated images; existing Module04 context, offset, scale and freshness regressions |
| Spill detection / confirmation / consistent coordinates | PASS | Same Module02/03 algorithms/settings; common current-frame registration in test-vision and auto-clean; transient misses tested |
| RED priority and multiple sequential targets | PASS | Two-target full camera simulation; three-target retry/dedup regression |
| Front-bumper gap to real contour | PASS | Finite segment/contour geometry, independent triangle calculation, observed arrival checks |
| Practical direct / one-corner F/B routing | PASS | Full swept-footprint checks, constrained-start escapes, obstacles, reverse and overshoot tests |
| Real MOVE acceptance / duration / DONE handling | PASS | Existing physical-mode client tested with local/in-memory TCP peers; no robot contacted |
| STOP, stale/lost localization and stationary reacquisition | PASS | Missing STOP ACK, lost marker, missing DONE and buffered-frame regressions |
| TARGET → ARRIVED → DONE,CLEANING → next target | PASS | Simulated firmware event sequence and fault regressions |
| Physical wall-cleaning reachability | BLOCKED | Existing measured contact/compression/nozzle/rotating-tool geometry is incomplete; no fabricated replacements |
| Actual complete physical navigation and cleaning | NOT VALIDATED | No physical camera/robot/actuator operation in this implementation task |

## Reproduced software bugs and fixes

1. **Box edge was reported as spill edge.** A triangular spill at
   [(80,50),(100,50),(100,90)] and robot (62,70,0°) formerly reported 5 cm from
   its bounding box. The finite bumper is actually 10.0176 cm from the contour.
   Distances now use actual outer polygon edges, including edge crossings and
   body overlap. Candidate generation solves the actual gap when a diagonal or
   concave edge is farther than its bounding box.
2. **Only one axis ordering and a greedy heading could reject a viable approach.**
   Synthetic starts (16,18,0°) and (20,17,0°), using the saved skip-run spill,
   reproduce the old Y-first rejection. Direct and both one-corner F/B alternatives
   now receive complete rotation/translation checks, with a choice based on the
   existing direction-specific timing estimates. Parking candidates also include
   the current heading and direct approach bearing. This is a small bounded
   enumeration, not a new global search/planning framework.
3. **Every automatic target was constrained by a compulsory return route.**
   The current requested workflow no longer preflights or executes HOME between
   targets. Existing standalone/explicit round-trip behavior is preserved.
4. **Exactly one confirmation streak was treated as sufficient observation time.**
   Blank → spill → blank could finish the old 3-frame scan empty, immediately
   before three stable spill frames. The scan now observes up to 12 frames,
   still requiring the unchanged three consecutive confirmations. It exits early
   once visible candidates are confirmed, and records current candidate/localization
   counts. Genuine empty/unusable observations remain empty, not invented targets.
5. **Display and navigation used different background-to-calibration assumptions.**
   A legal background is not proof of the original calibration view. Both wrappers
   now use the same diagnostic current-frame Module01 registration, preserving
   UNVERIFIED labels. Detector thresholds, actual transforms, calibration and
   the existing geometry hazard/warning checks were not relaxed or rewritten.
6. **Position and heading tolerances could combine into a failed facing check.**
   A robot inside the accepted parking-position and nominal-heading tolerances
   can still have a different bearing to the spill. A bounded final facing
   correction now uses the measured spill bearing, followed by a new spill scan
   and a new robot pose. Larger unexpected position/heading changes still fail.
7. **Failed targets were permanently retired; nearby distinct targets could merge.**
   Only completed targets are retired. Failed targets are reconsidered from later
   poses, and spatial identity includes polygon-bounds overlap. Post-cleaning color
   fluctuations cannot automatically re-run the same preset.
8. **Any successful task masked remaining rejections / batch exhaustion.**
   Results now distinguish PARTIAL and BATCH_LIMIT. Default batch cap is 100;
   completion is not a claim that stains disappeared. Non-return tasks never
   advertise `home_verified=true`.
9. **Stationary states received fictitious sweep inflation.** Zero translation
   or rotation has no unsampled interval. Actual moving intervals keep their
   translation allowance and radius-based angular coverage; genuine wall/obstacle
   collisions still fail.
10. **Two buffered pre-settling frames could cause a false stale-pose fault.**
    CameraPoses now waits explicitly for capture after the settling boundary.
    The wait remains bounded and still rejects stale/unverified results.

Historical `NO_REACHABLE_TARGET` run 073050 did not record the robot pose or the
failing leg. Run 074619's `NO_CONFIRMED_SPILL` did not preserve its input frames
or candidate counts. Those particular physical causes cannot be reconstructed;
the fixes above are demonstrated software reproductions, not invented historical
positions or proof of physical success.

## Validation

- Final full suite: **1072 passed, 3 warnings, 64.43 seconds**.
- New autonomy regressions: 26 tests; rotation/reacquisition combined focused
  check: **37 passed**.
- Full suite includes localhost TCP client/simulator tests, actual physical-mode
  MOVE/DONE timing guards, STOP races, cleaning fault handling, Dashboard tests
  and all existing vision/calibration regressions. The warnings concern existing
  legacy floor-only calibration fixtures.
- Full generated-camera end-to-end test renders ArUco frames from a synthetic
  plant, then uses the actual Module04 decoder and CameraPoses for every controller
  observation. Module02/03 detects both targets from those same frames. It completes
  RED then NON_RED with two ARRIVED/DONE,CLEANING cycles. Timed predictions are
  never supplied as the controller's observed pose.
- Optional saved real image `data/iphone_debug/frame_000200_input.png` was replayed
  through the real detector/localizer and then a simulated robot/firmware queue:
  **one target completed**. The robot motion is synthetic; historical image/current
  calibration association and physical spill accuracy are not certified by this replay.
- CLI simulation evidence: `data/autonomy_mvp_final_20261109/`, READY, completed=2,
  skipped=0. Observed simulated gaps: **5.3110 cm** and **6.5409 cm**; facing errors
  **1.6779°** and **0.8070°**, within the existing ±2.5 cm / ±7° verification limits.
- No physical device was opened, contacted, moved, cleaned or flashed.

Reproduce software tests with `.venv/bin/python -m pytest -q`. Tests that bind
localhost ports require an environment permitting local TCP listeners.

## Preserved inputs and remaining physical prerequisites

Started from clean branch `main`, commit `f407bed`. A verified backup of 42
configuration/background/measurement/firmware files is stored at
`backups/autonomy_before_20261109T151806Z/`; original hashes remain unchanged.
Saved Module04 mapping still has 4 training / 3 independent validation points,
RMS 1.045 cm and maximum 1.253 cm. Robot configuration remains 9 cm marker,
16.6 cm height, 26×15.2 cm body, configured center offsets 2 cm forward / 1 cm
local lateral, heading offset 0°. Its signed local convention was not silently
reinterpreted or its measurements changed.

Physical validation remains required for actual camera framing/marker visibility,
configured chassis-center reference and signed offsets, real speed variation,
stopping latency/coast, and tool clearance/cleaning coverage. The operator's
ground-reference/static-tool confirmations are still required. Missing wall/floor
contact geometry was not invented. SIMPLE still has no heartbeat watchdog or
latched ESTOP, and the repository's imported firmware is not proof of the flashed
version. Actual CAPS must match the selected SIMPLE profile.

Before the operator runs the command below, close other applications holding the
camera, including Dashboard LIVE/test-vision, and ensure the configured camera,
reference, chassis/tool clearance and current ESP32 address have been checked.
This command authorizes real navigation and cleaning; it was **not run** here.

```bash
.venv/bin/python main.py auto-clean --mode physical --host 172.20.10.12 --port 3333 --camera 0 --background data/iphone_reference.png --confirm-clean --authorize-motion --confirm-fixed-camera --confirm-ground-reference --confirm-static-tools
```

## Changed files

`navigation/automatic.py`, `navigation/controller.py`, `navigation/geometry.py`,
`navigation/adapters.py`, `vision/live_test.py`, `tests/test_autonomy_mvp.py`,
`tests/test_automatic_cleaning.py`, `README.md`, and this report.
