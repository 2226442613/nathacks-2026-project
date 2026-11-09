# Module 03 physical geometry extension — implementation report

Historical geometry-extension report. The later non-blocking framing policy in
`MODULE03_FRAMING_REPORT.md` supersedes its unresolved-registration behavior.

## Files

New implementation: `vision/target_geometry.py`.
Updated integration: `vision/localization.py`, `vision/localization_runtime.py`,
`vision/spill_cli.py`.
Updated configuration/contracts: `config/localization_config.json`,
`config/target_message_v1.schema.json`.
Tests: new `tests/test_target_geometry.py`, updated `tests/test_localization_cli.py`.
Documentation: `README.md`, this report.
Generated synthetic artifacts: `data/module03_geometry_demo/` (fixtures, image/video
JSONL and event streams, full sample localization, v1 message and preview).

Real `config/camera_config.json`, `data/iphone_reference.png` and its metadata were
read only. Their before/after SHA-256 checks matched during this task. Camera
capture, detector segmentation, tracking, calibration algorithms and robot modules
were not redesigned.

## Transformation, bounds and area

Reuse each exact Module 02 component-label mask. Construct exposed pixel-cell
edges at half-pixel coordinates; retain every outer ring, hole and disconnected
island, plus a lossless mask row-run representation. This avoids collapsing
one-pixel targets and distinguishes the footprint from a display bounding box.
Each vertex maps current → calibration exactly once, then uses the public
Module 01 floor/wall mapping API. Validate registration/physical homography
horizons, finite coordinates and physical surface bounds. Ambiguous/unresolved
ownership retains image geometry and publishes null physical geometry.

Bounds are min/max over complete mapped rings. FLOOR dimensions are width/depth
(X/Y); WALL dimensions are width/height (X/H). Area adds projected outer polygon
areas and subtracts projected holes using the shoelace formula. It measures the
projected union of raster cells, not unknown hidden object area. Projective
mapping preserves the straight cell edges away from a horizon; no global pixel
scale is assumed. Calibration, sampling and segmentation still limit accuracy.

## Internal and transmitted representation

Local records preserve dense float64 physical rings, image rings and mask runs.
Messages carry optional additive v1 `geometry_cm`, `last_known_geometry_cm`,
`geometry_status` and `geometry_reason`. Original v1 required fields remain.
`contours` supports outer/hole rings; `contour` is available only for one outer
ring with no holes. No masks/full internal rings are sent in the compact message.

Default physical simplification tolerance: 0.2 cm, capped at one quarter of the
smallest projected pixel edge. Douglas–Peucker selects existing vertices.
Fallback preserves full rings for collapsed/orientation-changing outlines, >1%
area changes or proper compact-segment crossings. Area and bounds always use
full rings. Default geometry-section limits: 512 vertices, 32768 bytes.
Over-budget contours become null with explicit CONTOURS_OMITTED_LIMIT; summaries
and local full geometry survive. Limits do not apply to complete local records.

Geometry trust follows calibration and registration quality. Complete boundaries
must fit inside the independently measured validation hull to be VERIFIED; a
verified centroid alone is insufficient. UNVERIFIED reliable estimates remain
available. Framing overrides and small motion do not prove centimeter accuracy.

## Example and persistence

Complete generated v1 payload: `data/module03_geometry_demo/target_message_v1.json`.
Its FLOOR example retains schema_version=1, target_id=1, center X=32.142857,
Y=44 cm, UNVERIFIED. Geometry has X bounds 28.392857..35.892857 cm and Y bounds
37..51 cm, width 7.5 cm, depth 14 cm and estimated area 83.4 cm². The file
contains the actual compact contour, calibration/frame/registration metadata,
timestamps and freshness fields. These are synthetic fixture results.

UPSERT updates stable target geometry; INACTIVE clears current geometry and
position while preserving explicitly last-known geometry, timestamps and full
historical localization records. Fresh-target readers continue filtering inactive
and expired observations. No message authorizes motion.

## Validation

`.venv/bin/python -m pytest -q`: **429 passed, 1 warning in 5.55 s**.
The warning is the existing legacy-floor-only calibration notice.
32 new test cases cover floor/wall rectangles and irregular shapes, perspective
mapping, exact bounds/area, camera transform direction/no double application,
holes, islands, diagonal contacts, single pixels, thin strips, simplification,
point/byte budgets, ambiguity, invalid calibration/horizons/nonfinite values,
trust scope, v1 compatibility, multiple targets, updates/disappearance and CLI
preview/event integration. Existing Modules 01–03 regression tests pass.
Compile check passed. Actual synthetic image and video CLI runs succeeded; the
saved image preview was visually inspected. Schema JSON parses and required v1
fields were checked; no external JSON Schema validator dependency was added.

## Live preview command

Only declare this reference if it actually matches the saved calibration pose:

```bash
cd /Users/williamwu/natHack
.venv/bin/python main.py detect-spills --camera 0 --width 1280 --height 720 --fps 30 \
  --background data/iphone_reference.png --confirm-clean \
  --calibration config/camera_config.json --localize --localization-boundaries \
  --localization-reference data/iphone_reference.png \
  --output data/module03_geometry_live.jsonl --target-events data/module03_geometry_targets.jsonl
```

The README also contains a reproducible headless synthetic preview command.
Real iPhone boundary accuracy, centimeter accuracy and live throughput require
field measurements. Full dense geometry increases local JSONL size and processing
cost; synthetic tests are not a live performance guarantee. No cleaning paths,
ArUco, reachability, navigation, Wi-Fi transport, scheduling or hardware control
were implemented.
