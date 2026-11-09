# Operator movement and cleaning-tool measurements

2026-11-09. Started on clean `main` at
`8817dda577358cc9f89bfd7ef2d56119250e6969`.
No physical robot/camera connection, motion, cleaning or firmware upload occurred.

## Measurements recorded

The values are **real-world approximate operator measurements**, not simulation
data or guaranteed constant motion rates.

| Direction | Reported measurement | Initial timing conversion |
|---|---|---:|
| F | 16 cm/s | 62.5 ms/cm |
| B | 18.5 cm/s | 54.054054 ms/cm |
| L | 4000 ms per 90° | 44.444444 ms/degree |
| R | 3000 ms per 90° | 33.333333 ms/degree |

`config/movement_timing.json` preserves the original sample list and R1500/R3000
reference history, adding distinct approximate physical `operator_estimates`.
F/B use a mathematically normalized one-second reference; this is explicitly not
a claim that a new one-second trial was performed. Old unverified references do
not get averaged into the latest reported estimates.

`MovementTiming.magnitude_rate(direction)` exposes all four conversions. F/B
signed body-forward displacements remain positive/negative. The operator confirmed
that the sign of the **Module 04 heading-number change** during L/R is unknown.
The turn magnitudes are retained as measured, while automatic signed turn selection
remains blocked until a signed camera observation is available. Physical L/R labels
alone do not establish how this camera coordinate frame's heading numbers change.
The existing before/after pose calibration CLI can resolve each direction later;
it writes a new immutable revision and preserves these operator estimates.

The hybrid controller remains unchanged: bounded moves, live pose checks,
confirmed early STOP, settling, short corrections, Y→X outbound and X→Y return.
The default segment maximum remains 1000 ms. The 3000/4000 ms records are calibration
references, not automatic requests to run a single segment for that duration.
No stopping latency, upper speed bound or coast distance was provided, so those
records remain absent. Approximate average speeds are not substituted for stopping
bounds. Until those are measured, the existing short monitored fallback remains.

## Side sponge

Recorded in `config/cleaning_hardware.json`:

- Chassis remains 26 × 15.2 cm.
- Sponge installed on LEFT; physical dimensions 8 × 13.5 cm, without assigning the
  two dimensions to unverified forward/vertical axes.
- Outermost LEFT edge extends 5 cm beyond the chassis boundary.
- Observed lowest edge is 5 cm above the floor; the planning lower limit is 5 cm.
- Frontmost edge is 6 cm behind the chassis front: observed forward edge = +7 cm
  from chassis center. This does not identify the sponge's effective contact center.
- Rotation about its center is recorded. Rotation-axis direction, dimension-axis
  assignment, effective contact width/center, maximum contact height, compression,
  safe clearance and rotation envelope remain unverified/unconfigured.

A conservative **static collision exclusion box** covers the full chassis length
and LEFT overhang: 26 × 20.2 cm, shifted 2.5 cm LEFT from chassis center. The panel
diagonal bounds either unknown projected dimension; the supplied front setback
allows its observed static span to fit within the full chassis-length box. Invalid
future measurements that no longer fit this bound require a separate envelope.

This coarse box participates in Module 05 floor-stroke/pose containment and Module
06 boundary, obstacle and chassis-rotation sweeps. It is not an effective cleaning
patch, floor-contact sponge, exact panel shape, or rotation-dependent sponge sweep.
It covers the reported static pose only; a different carry/servo pose must be
validated before physical execution. No sponge rotation axis was invented.

Under the existing Module 05 top-wall convention, a LEFT-side tool requires
heading 180° to face the wall, with the chassis parallel to it. Half-width 7.6 +
overhang 5 gives an **uncompressed** centerline reference 12.6 cm. The earlier measured
Y≈13 cm navigation reference was preserved. Neither value establishes actual
compression or contact force. The static collision check does not grant penetration
into the wall for an unvalidated compressed/rotating sponge model.

Wall plans stay CONFIGURATION_REQUIRED while contact/rotation geometry is unknown.
The measured below-5 cm blind region is reported even when upper coverage cannot
yet be calculated. No cleaning below the measured lower limit is claimed.

## Nozzle and independent floor tool

The nozzle observation records near-front-left placement, height 16.5 cm and water
landing approximately 10 cm ahead of the robot. The observation does not specify
the exact reference point for that distance. It therefore does not become a
ground-center-relative impact offset, a mount-corner coordinate, spray rectangle,
wall impact band or invariant trajectory.

Height 16.5 cm is also retained as floor-nozzle mounting metadata; exact forward/
lateral offsets and effective coverage remain null. The wall actuator/nozzle
profile remains independently unconfigured. This records one reported nozzle
observation, not additional independent nozzle hardware or a wall-mode solution.
Previously explicit calibrated effective offsets retain authority over raw nozzle
observations; changing observation metadata cannot silently move a coverage patch.

The separately configured `floor_sponge` section was preserved unchanged.
It currently has no measured dimensions/offsets. The elevated side sponge is never
substituted for it. Physical floor plans retain `reachability=UNKNOWN` and add
`configuration_status=CONFIGURATION_REQUIRED`, with explicit missing fields and
no approach or coverage segments. This preserves existing reachability enums and
Dashboard compatibility. Partial physical measurements do not set the global
`measurements_validated` flag or authorize execution.

## Files and verification

Source/configuration changes are in existing commit
`a6e303b817500c42174f0d87c60ea1e4012dbd23`, which appeared during this work and was
preserved without rewriting history:

- `config/movement_timing.json`, `config/cleaning_hardware.json`.
- `config/cleaning_hardware.schema.json`, `config/cleaning_plan_v1.schema.json`.
- `navigation/timing.py`, `navigation/geometry.py`, `navigation/cli.py`.
- `vision/cleaning_config.py`, `vision/cleaning_planner.py`.
- `tests/test_navigation.py`, `tests/test_cleaning_planner.py`.

The independently created merge `5b6f0cb` also appeared during verification. It
adds reference assets/docs/firmware without changing these tested runtime files;
it was preserved. This task neither edited nor uploaded that firmware.

The supplemental commit adds `tests/test_operator_measurements.py`, this report
and report links. It includes 18 new cases covering independent timing factors,
unresolved turn signs, no invented stopping bounds, bounded closed-loop use,
LEFT/RIGHT wall alignment, static overhang containment/sweeps, floor/wall tool
separation, lower blind zone, nozzle-offset authority and failure before opening
hardware when physical geometry is incomplete.

```bash
cd /Users/williamwu/natHack
.venv/bin/python main.py movement-calibration show
.venv/bin/python -m pytest -q tests/test_operator_measurements.py
.venv/bin/python -m pytest -q
```

Targeted planner/navigation/measurement tests: **182 passed**.
Full suite: **915 passed, 1 existing warning in 42.01 seconds**. The warning is
the intentional legacy floor-only calibration test.
No physical accuracy or cleaning effectiveness claim is made from these tests.

Pre-edit backup: `backups/operator_measurements_before_20261109T100359Z/before.zip`,
with SHA-256 manifest and starting commit; all 153 entries were verified. It includes
the old physical tool/timing configuration, protected camera calibration, marker
mapping, measured point snapshots, registered background and tracked modules.
Existing recordings/evidence were left in place. Protected-file verification is
stored alongside the backup: 140 existing files outside the intended edit list
matched their pre-edit hashes. The original floor-sponge settings and all prior
timing samples/references also matched. Camera/ArUco calibration, robot settings,
registered background, existing firmware and SIMPLE ASCII/TCP3333 protocol were
not edited by this task.
