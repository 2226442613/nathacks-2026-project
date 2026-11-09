# Offline investigation: auto_navigation_20261109_073050_381003

Evidence: `data/auto_navigation_20261109_073050_381003/{summary.json,wire.log,progress.jsonl}`.
No hardware was accessed. Original evidence was preserved.

## What the evidence proves

The recorded rejection is exactly:

`Chassis/tool swept area intersects wall, floor boundary or obstacle`

It originates in `Arena.check_segment`, when a sampled chassis/tool footprint fails
`Arena.safe`. Physical startup creates an arena without additional obstacles, so,
with the preserved configuration, this is a floor/wall containment failure.
`run_queue` previously retained only the last candidate's exception text. It did
not record the candidate, failing stage, sampled pose or footprint bounds.

**The robot's x, y and heading are absent from all three evidence files.**
`progress.jsonl` is empty. The localization diagnostics establish a fresh VERIFIED
marker observation but contain no coordinates. There is no saved camera frame.
Thus neither the historical candidate order nor the exact rejected outbound/return
segment can be recovered. No selected parking pose or physical bumper clearance
was recorded. No MOVE, TARGET or ARRIVED was transmitted in this run.

The background diagnostic was non-blocking (`hazard: null`) and was not changed.

## Recorded target and offline parking calculations

- Target 1: RED_SPILL, HIGH priority, FLOOR.
- Representative spill position: (68.999991, 38.494474) cm.
- Bounds: x=[66.235789, 71.882771], y=[35.613002, 41.312518] cm.
- Width 5.646982 cm; depth 5.699516 cm; area 27.414500 cm².
- The original summary retains the complete measured contour.
- Robot actual pose: **not recorded**, not inferred from another run.

With a 26 cm chassis, center-to-front distance is 13 cm. Parking offsets use
13+5=18 cm from the relevant spill bounding edge. All four calculated bumpers
span the spill's transverse extent, so their nearest contour-edge clearance is
also 5 cm. These are proposed mathematical poses, not observed arrival poses.

| Candidate | Center x, y (cm) | Heading | Bumper gap | Parked boundary | Cleaning preset sweep |
|---|---|---:|---:|---|---|
| West | 48.235789, 38.462760 | 0° | 5 cm | PASS | PASS |
| East | 89.882771, 38.462760 | 180° | 5 cm | PASS | PASS |
| North / wall side | 69.059280, 17.613002 | 90° | 5 cm | PASS | FAIL |
| South | 69.059280, 59.312518 | 270° | 5 cm | FAIL | Not eligible |

These calculations use the preserved current configuration; they cannot supply
the missing historical robot pose or prove which candidate supplied the final
exception.

## Exact recomputed boundary failures and thresholds

Floor: x=[0,121.6], y=[0,68.4] cm, wall at y=0. Chassis: 26×15.2 cm;
configured margin: 1.5 cm. Static left-tool exclusion envelope: 26×20.2 cm,
lateral center offset +2.5 cm. Translation checks add 0.25 cm; rotation checks
add maximum footprint radius × sin(1°). Containment floating tolerance is 1e-7 cm.
No boundary or margin was relaxed.

- **South:** padded chassis max_y=73.812518 cm, exceeding the 68.4 cm floor by
  5.412518 cm. This candidate was filtered before route checks in the original
  implementation.
- **North:** parking itself fits. Cleaning checks each of the three F/B cycles
  against the independent 600 ms segment limit (normal segment 350 ms).
  Segment 1 normally advances 16×0.350=5.6 cm. Segment 2's backward bound travels
  18.5×0.600=11.1 cm. Its endpoint y=12.113002 cm gives chassis
  min_y=12.113002−13−1.5−0.25=−2.636998 cm, violating min_y≥−1e-7 cm.
  The actual sweep loop first rejects at center (69.059280,14.579669,90°), with
  min_y=−0.170331 cm. This cleaning rejection remains enforced.
- **West/East:** both parking and the complete cleaning collision sweep pass.
  Their outbound/return checks require the unrecorded robot pose. The evidence
  does not establish which route or sampled bounds failed.

Arrival verification remains |gap−5|≤2.5 cm, facing error≤7°, using fresh measured
poses. Those checks were never reached in this run.

## Minimal fixes and their limits

1. Planner bug reproduced separately: synthetic start (18,18,0°), Y target 38 cm.
   The old nearest-heading tie selected 90°. At 31° the left-tool min_x becomes
   −0.066218 cm and fails containment. The alternative turn to 270° is safe;
   backward movement then increases Y. Both preflight and execution now consider
   both calibrated cardinal headings, retain all sweep checks, and prefer the
   shorter turn among safe choices. Existing collision exception/STOP behavior
   is preserved. **This counterexample is not the recorded robot pose and does
   not prove this bug caused the historical run.**
2. Each target now prints/records its robot pose, spill bounds, all attempted
   parking poses, bumper clearance, failed stage, sampled bounds and floor limits.
   Rejected targets retain every candidate reason in the summary. Unsafe parking
   candidates are logged but still rejected. No additional planning architecture
   or hardware operation was introduced.

Changed code: `navigation/automatic.py`, `navigation/controller.py`,
`navigation/geometry.py`. New regression tests: `tests/test_skipped_target.py`.
This report is also new. Pre-existing work was preserved in the verified archive
`backups/skipped_target_before_20261109T133758Z/before.zip`.

Verification: **251 passed, 21 deselected** (11.31 seconds). Focused offline tests cover recorded target geometry,
cleaning boundary rejection, safe alternate F/B heading, obstacle rejection,
candidate diagnostic preservation, live-feedback freshness, camera loss,
MOVE/STOP handling and simulated cleaning/HOME lifecycle. No real camera or TCP
robot connection was opened. Network-binding integration tests were deselected.
