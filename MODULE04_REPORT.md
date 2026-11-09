# Module 04 — ArUco robot tracking implementation report

Code-level implementation is complete. Physical printed-marker and robot validation
is pending. No movement commands, navigation, network transport or later modules
were implemented.

## Files

New:

- `vision/robot_tracking.py`: configuration, marker detector, typed observations and
  poses, height-aware/approximate position, plane-aware heading, tracker,
  freshness predicate, dynamic body exclusion and preview.
- `vision/robot_plane.py`: separate empirical fixed-height mapping with raw measured
  training/hold-out points, validation and calibration/height binding.
- `vision/robot_cli.py`: marker generation, standalone image/video/camera tracking,
  measured plane fitting and guarded output files.
- `config/robot_config.json`, `config/robot_pose_v1.schema.json`.
- `tests/test_robot_tracking.py`, `tests/test_robot_cli.py`.
- `tools/generate_robot_demo.py`: reproducible protected synthetic fixtures.
- This report and generated `data/module04_print/`, `data/module04_demo/` artifacts.

Modified: `main.py` dispatch, `vision/__init__.py` public APIs,
`vision/spill_cli.py` optional robot exclusion/integration and `README.md`.
No dependency changes. Saved Module 01 calibration and legal baseline were not
written; existing Module 03 target schema/events and default behavior remain.
The pre-existing root `aruco_id0.png` was also preserved.

## Marker generation and printing

Actual default dictionary **DICT_4X4_50**, ID **0**. Other dictionaries/IDs are
configurable. The generated ready-to-print file is
`data/module04_print/aruco_id0_6cm.png` with sizing metadata in `.png.json`.
It is a 1600×1600 PNG: 1200 px black square plus 200 px white quiet zone per side.

Reproduce to a NEW output:

```bash
.venv/bin/python main.py generate-aruco --dictionary DICT_4X4_50 --marker-id 0 \
  --marker-size-cm 6 --side-pixels 1200 --quiet-zone-pixels 200 \
  --output data/aruco_id0_new.png
```

Print the complete image at **8×8 cm**, preserving its square aspect ratio and
white border. Disable fit-to-page/cropping. Measure the black square along both
axes: each must be **60 mm**. Adjust printer scale if necessary. The white border
should be 10 mm each side at that scale. PNG pixel dimensions/DPI do not verify
physical dimensions; the sidecar deliberately says physical_size_verified=false.
Mount the marker flat, facing up. Use the actual measured black-square side in
robot settings if it differs from the intended 6 cm.

## Detector, robot API and heading

`RobotMarkerDetector.detect(frame)` uses OpenCV ArucoDetector and subpixel corner
refinement. It ignores unrelated IDs, requires exactly one configured marker and
rejects invalid/duplicate observations. It preserves canonical TL/TR/BR/BL corner
order and the projective center (diagonal intersection, not corner mean).
[OpenCV's official ordered-corner documentation](https://docs.opencv.org/4.x/d5/dae/tutorial_aruco_detection.html)
was checked against the installed OpenCV 4.14.0 API.

`RobotTracker.update(frame, timestamp=None, registration=None)` returns `RobotPose`.
`to_dict()` is JSON serializable; the separate schema is ROBOT_POSE v1. Stable robot
ID defaults to 0. Current marker observations include timestamp, pixels, pose,
registration and quality. Missing/ambiguous markers immediately clear current
position, heading and corners, retain explicitly last-known historical values,
and produce LOST then EXPIRED. Defaults: 5 missed frames, 0.5 s observation TTL.
Gain/loss transition events are emitted. Reacquisition resets smoothing. The
public `robot_pose_is_current(message, timestamp=None)` also checks freshness of
serialized historical observations; never treat the last VISIBLE line as current
without checking its timestamp/valid_until.

Heading uses native Module 01 floor X/Y: 0° toward +X, positive toward +Y,
normalized [0,360). In the present top-boundary setup, +X is along the wall and +Y
away from it. It maps the marker canonical +u axis through the appropriate plane
before atan2, then adds marker_to_robot_heading_deg. Raw image angle is not used
as physical heading. Configurable smoothing uses shortest wrapped angular changes,
including 359°→1°. The preview arrow is explicitly the decoded **marker +u image
axis**; numeric theta is the plane-aware robot heading estimate.

## Height and mounting offsets

Module 01 has planar homographies but no intrinsics/extrinsics. No camera matrix
or robot height is fabricated.

Default Option C: floor-ray intersection of the elevated marker, explicitly
APPROXIMATE_FLOOR_RAY_INTERSECTION and UNVERIFIED. It has height parallax and is not
claimed as true robot ground-center. Setting a height number alone cannot fix it.
The default height is null (unknown), not silently zero.

Optional Option B: a separately measured fixed-height marker-plane homography
maps elevated marker pixels to their vertically projected floor X/Y. Fit at least
four broadly distributed training points and validate against at least three
non-collinear independent hold-out points. Both sets need >=1% floor-area hull
coverage; all hold-out errors must be within tolerance (default 2 cm). Invalid,
reused, nonfinite, collinear or out-of-bounds measurements are rejected. Raw
points are re-fit and revalidated on load; output binds calibration SHA-256,
frame dimensions and mounting height. Changing height or calibration identity
rejects the mapping. The original floor calibration is never edited.

Measured-point JSON is `{training:[{pixel:[u,v],ground_cm:[x,y]},...],
validation:[...]}`. Ground coordinates must be independently measured **vertical
marker-center projections**, not homography-derived coordinates or robot centers
with unaccounted offsets. The README explains collection and includes an explicitly
illustrative template. After collecting actual measurements:

```bash
marker_height_cm=YOUR_MEASURED_HEIGHT
.venv/bin/python main.py calibrate-robot-plane --calibration config/camera_config.json \
  --points data/robot_plane_measurements.json --mounting-height-cm "$marker_height_cm" \
  --confirm-measured --tolerance-cm 2 --output data/robot_marker_plane.json
```

Use `--marker-plane data/robot_marker_plane.json --mounting-height-cm "$marker_height_cm"`
with track-robot. Fixed camera pose and flat, parallel mounting are required.
Reliable bounded image alignment maps pixels backward exactly once; large/unknown
alignment falls back with UNVERIFIED warnings. A floor/background alignment can
still have elevated-plane parallax. A frozen optional --alignment-reference must
show the corresponding static camera view, not rely on moving-robot features.
Even an independently fitted empirical map remains UNVERIFIED for actual robot
operation pending printed-marker/robot field validation.

Reference point is robot ground-center. `marker_offset_forward_cm` and
`marker_offset_left_cm` describe robot-center → marker-center in robot-local
coordinates. Subtract their rotated vector using the computed robot heading.
The explicit default zeros assume a centered marker whose +u axis is robot forward;
measure and change them when that is not true. Height is floor → marker plane.

## Dynamic exclusion integration

`RobotTracker.exclusion_mask(pose, frame_size, timestamp=None)` creates a fresh,
approximate body mask for the existing
`SpillDetector.detect(frame, exclusion_mask=mask)` API. It expands the marker image
box by a configurable margin (default 80 px); optional measured body dimensions
union a projected ground footprint. It does not claim full body segmentation.
Tune margins against the oblique robot silhouette; overlarge masks hide nearby
spills, undersized masks leave robot parts visible. Missing/stale marker masks are
zero immediately. Visible robot parts may become foreground after marker loss.

`detect-spills --track-robot --robot-settings config/robot_config.json` enables
same-frame exclusion and separate robot_pose/robot_exclusion frame fields. It
preserves target IDs, target geometry/events and the explicit legal-baseline system.
The frozen initial reference is not silently rebound by later R/Y recapture.
`--robot-marker-plane PATH` selects the measured map; its height must match settings.
Other floor regions continue detecting spills. No circular segmentation dependency.

## Exact standalone commands

Use the existing iPhone camera index 0 and calibration. Output files are guarded
against overwrite; the timestamped live filename is convenient for repeated runs:

```bash
cd /Users/williamwu/natHack
robot_run="data/robot_pose_$(date +%Y%m%d_%H%M%S).jsonl"
.venv/bin/python main.py track-robot --camera 0 --width 1280 --height 720 --fps 30 \
  --calibration config/camera_config.json --robot-settings config/robot_config.json \
  --output "$robot_run"
```

Optional --marker-size-cm and --mounting-height-cm override settings with measured
values. They are not an automatic parallax correction. One GUI shows corners,
ID, center, X/Y/theta, tracking/trust, mapping limitation, arrow and measured FPS.
Q/Escape/window close exits. No extra debug windows open by default.

Actual synthetic image/video fixtures and successful output already exist under
`data/module04_demo/`. To repeat to NEW output names:

```bash
.venv/bin/python main.py track-robot --image data/module04_demo/robot.png \
  --calibration data/module04_demo/calibration.json --headless \
  --output data/module04_image_check.jsonl
.venv/bin/python main.py track-robot --video data/module04_demo/robot.avi \
  --calibration data/module04_demo/calibration.json --headless \
  --output data/module04_video_check.jsonl
```

Optional `--annotated-dir NEW_DIRECTORY` saves previews.
`.venv/bin/python -m tools.generate_robot_demo --output-dir data/module04_demo_new`
regenerates a fresh fixture set without changing real camera files.

## JSON example

Abbreviated real generated **synthetic** example (rounded display values):

```json
{
  "schema_version": 1,
  "message_type": "ROBOT_POSE",
  "robot_id": 0,
  "marker_id": 0,
  "surface": "FLOOR",
  "position_cm": {"x": 9.9875, "y": 7.4875},
  "heading_deg": 0.0000036,
  "tracking_status": "VISIBLE",
  "localization_status": "UNVERIFIED",
  "actionable": false,
  "motion_authorized": false,
  "suitable_for_navigation": false
}
```

Complete JSON including actual timestamp, corners, center, coordinate frame,
calibration identity, marker/mounting settings, freshness, history and quality:
`data/module04_demo/robot_pose_v1.json`. It is historical fixture data, not a current
physical robot observation. The schema JSON parses; required-field/loss invariants
are tested. No external JSON Schema validator dependency was added.

## Test results and performance

`.venv/bin/python -m pytest -q`: **511 passed, 1 warning in 8.08 s**.
The warning is the existing legacy floor-only notice. Python compile checks passed.
52 new tests cover generation, dictionary/ID selection, rotations, perspective,
canonical corners/projective center, occlusion/missing/low-contrast markers,
duplicate IDs, four cardinal headings, mounting rotation/offsets, wrap smoothing,
missing calibration, independent elevated-plane correction and bad context/data,
small alignment direction/no double application, large-motion uncertainty,
tracking loss/reacquisition/expiration, stale-mask/freshness checks, body exclusion,
JSON contract, Module 03 floor-axis consistency, real static/video CLI, mocked GUI,
spill integration, print metadata/output protection and timestamp validation.
All existing Modules 01–03 tests pass. Actual static/video CLI runs succeeded:
the generated 20-frame video includes tracking gained/lost events. Preview was
visually inspected.

A local 100-frame warm synthetic 800×600 detector+pose benchmark measured
approximately **0.635 ms median**, about **1487 frames/s aggregate for this isolated
simple fixture**. It excludes camera capture, GUI, JSON serialization and disk I/O.
It is not live-camera FPS, real-scene throughput or physical accuracy. Detailed
scope is in `data/module04_demo/performance.json`.

## Physical validation checklist and navigation readiness

1. Print and ruler-check the black square in both axes, including white quiet zone.
2. Mount flat; measure marker height, robot-center offsets and rotation; configure
   actual body dimensions or tune the conservative pixel mask against its silhouette.
3. Fix the iPhone pose/crop/resolution; confirm the floor/wall calibration and
   actual saved dimensions. Register a legal background explicitly as needed.
4. Check detection at different floor positions, distances, oblique angles and
   0/90/180/270° rotations. Verify robot forward and native floor +X/+Y conventions.
5. Independently measure ground-center/heading errors. If using the empirical map,
   collect spread training points and independent hold-outs at unchanged height.
6. Test partial/full occlusion, loss/reacquisition, stale output and body-mask removal;
   verify nearby real spills remain outside the exclusion mask.
7. Test small/large camera shifts, glare, lighting, motion blur and print visibility.
   Measure actual iPhone end-to-end FPS separately.

Code interfaces are ready for future reachability/navigation/transport integration.
Physical readiness is not established. All pose messages are non-actionable and
not suitable_for_navigation until a later policy and real validation authorize
usage. No Module 05–09 logic, cleaning paths, motor controls or network commands
are present in this change.
