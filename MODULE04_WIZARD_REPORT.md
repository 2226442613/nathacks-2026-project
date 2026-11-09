# Module 04 interactive robot-plane calibration

The new `calibrate-robot-interactive` command captures raw ArUco centers from a
live camera, takes ruler-measured coordinates in the OpenCV window, and delegates
fitting and independent validation to the existing `MarkerPlaneMapping.fit`.
No terminal coordinate transcription, new GUI framework, camera algorithm,
robot-pose schema, navigation, or actuator interface was introduced.

## Start the wizard

Run from `/Users/williamwu/natHack`:

```bash
.venv/bin/python main.py calibrate-robot-interactive \
  --camera 0 --width 1280 --height 720 \
  --calibration config/camera_config.json \
  --robot-settings config/robot_config.json \
  --mounting-height-cm 16.6 \
  --points-output data/robot_plane_points.json \
  --output config/robot_marker_plane.json \
  --tolerance-cm 2
```

Both output paths must be new. For a second session choose new names for both.
`--fps` is also supported and defaults to 30. Height may be omitted when measured
height is configured; an explicit height must agree with a configured height.
The requested and actual frame dimensions, camera source, arena identity, and
robot settings are checked. Source files changing during collection prevent
fitting/saving under the old identity; restart with the intended fixed inputs.

## Measurement convention and physical procedure

1. Keep the camera, lens/zoom/crop, resolution, arena and marker height fixed.
   The printed marker must be flat at the measured 16.6 cm height. Keep the full
   robot on the floor and the marker clearly visible. Move the robot by hand.
2. Measure the **vertical projection of the MARKER CENTER onto the floor**, using
   a plumb reference. Use native Module 01 floor X/Y, centimeters. With the current
   top shared edge, X runs along the wall; Y runs away from the wall. These are
   **not chassis-center coordinates**. Press A to attest to that convention.
3. In TRAINING mode, press C, type measured X, Tab, type measured Y. The selected
   input is marked `>`. The live panel shows the decoded ID, raw center pixels,
   corners/center, detection state, next point number and collected counts.
4. Let the robot stop. Press Space when a stable detection window is available.
   No image clicks or pixel transcription are necessary. The capture message
   reports retained frames and pixel variation. A failed capture adds no point;
   C clears the window and fields for retry. Each new capture clears its window.
5. Move to another measured location, repeat. Use at least four training points;
   8–12 broadly spread points are a useful practical target, not a code minimum.
6. Press V for independent VALIDATION mode. This clears inputs and the window.
   Capture at least three additional, non-collinear, broadly distributed locations.
   Five to eight hold-outs are useful. They must not be fitting positions.
7. Press F. The points are saved before fitting, including on failure. Review
   independent validation errors; `[` and `]` page through rows. Each row gives
   signed dX/dY (mapped minus measured) and Euclidean distance in cm. The panel
   also gives RMS, maximum and tolerance pass/fail. Full rows are printed in the
   terminal and included in the points metadata; no terminal input is needed.
8. If rejected, inspect the listed validation measurements, height, camera pose
   and point distribution. U undoes the most recently captured point across both
   groups; subsequent captures require fresh frames. Correct an earlier point by
   undoing later points too, or start a new session. Fitting/edits invalidate any
   previous in-memory map/report. No failed map can be saved.
9. After PASS, press M to explicitly save the map. P toggles a read-only live
   test: marker-ground X/Y, offset-compensated chassis X/Y, heading, freshness,
   UNVERIFIED trust and the configured oriented chassis rectangle. Capture is
   disabled while testing. Numeric pose uses the existing tracker; the floor
   footprint is projected through the existing floor calibration. A displayed
   rectangle does not validate body clearance or authorize movement.
10. Q, Esc or closing the window safely ends capture. Collected points are
    preserved automatically on exit, including camera-read errors. The camera
    and GUI are released. Only M saves a passing mapping.

The wizard deliberately does not accept chassis-center measurements or offer a
heading-based conversion: the marker-to-vehicle heading alignment is unverified.
The configured +2 cm forward and +1 cm left offsets are retained and applied by
the tracker **after** mapping, using its computed robot heading. The existing 0°
heading offset is preserved as an unverified assumption. Height alone never
corrects parallax; the new measured plane mapping is required.

## Keyboard controls

| Key | Action |
| --- | --- |
| A | Confirm independent measurements of marker vertical ground projection |
| Digits, `.`, Backspace/Delete | Edit selected X/Y field |
| Tab | Switch X/Y field |
| C | Clear coordinate fields and discard current capture window |
| T / V | Select training/validation; clear fields and capture window |
| Space | Capture robust current center with measured X/Y |
| U | Undo last captured point, invalidate fit |
| S | Save a new points snapshot |
| F | Preserve points, fit with existing calibrator, show hold-out report |
| `[` / `]` | Previous/next validation report page |
| M | Explicitly save passing map, never overwrite |
| P | Toggle read-only live ground-center test after M |
| Q / Esc / close window | Preserve points and exit |

## Stability and independence

The collection uses the existing `RobotMarkerDetector`, never held-last robot
positions. It requires at least 8 distinct detected camera frames over at least
0.3 seconds in a recent window of at most 1.2 seconds / 90 observations. The most
recent observation must be no more than 0.25 seconds old at capture.

A repeated exact frame SHA-256 or missing/invalid marker breaks the window.
Monotonic receive timestamps must increase. A robust median center rejects
radial outliers; at least 8 inliers and 80% retention are required. Maximum
inlier radius and end-to-end median drift must each be <=2 pixels. Diagnostics
include inlier/outlier counts, radius, per-axis range, drift, span, frame hashes,
UTC receive times, monotonic receive times, and detector observation timestamps.

Receive timestamps are recorded immediately after synchronous camera reads.
The existing camera API supplies no sensor exposure timestamp; these are honest
host receive times, not invented hardware timestamps. Exact duplicate frames
are excluded, but a software-only check cannot prove source-camera freshness.
No image frames are written to the points JSON, only capture metadata/hashes.

Observations cannot share frames across saved points. New points must differ
from every training and validation position by >=0.1 cm and >=0.5 pixel. These
wizard guards supplement the existing calibrator's cross-set duplicate checks.
All coordinates must be finite and inside the declared frame/floor.
Each group's physical convex hull must span at least 1% of the floor area
(83.1744 cm² for 121.6×68.4). The existing checks were not weakened.

Suggested training marker projections are (25,18), (60,18), (95,18), (25,48),
(95,48). Suggested validation positions are (60,35), (45,30), (80,40), **(35,48),
(85,22)**. The first three alone span only 12.5 cm² and cannot pass the existing
1% rule. The two extra suggestions broaden the hold-out set. None are fabricated
observations or auto-filled coordinates. The UI filters suggestions using a
conservative circle enclosing the chassis and marker offset for every heading;
unknown body geometry produces no clearance claim. Check actual floor edges,
protrusions and obstacles physically as well.

## Output formats and non-overwriting behavior

Points use the existing required arrays without changing their point format:

```json
{
  "training": [{"pixel": [320.25, 295.10], "ground_cm": [25.0, 18.0]}],
  "validation": [{"pixel": [550.50, 390.25], "ground_cm": [60.0, 35.0]}],
  "metadata": {
    "measurement_reference": "MARKER_GROUND_PROJECTION",
    "attested_measured": true,
    "frame_size": [1280, 720],
    "calibration_id": "sha256:<actual calibration file digest>",
    "robot_configuration": {"marker_id": 0, "marker_size_cm": 6.0},
    "mounting_height_cm": 16.6,
    "heading_alignment_verified": false,
    "chassis_reference_conversion_supported": false,
    "capture_quality": [],
    "independent_validation": null,
    "created_at_utc": "<actual UTC creation time>"
  }
}
```

This is a shortened structural illustration, **not measured data** or a complete
fit-ready set. Actual robot_configuration contains all unchanged settings;
capture_quality contains each point's group/index, frame identities and quality
record. After F, independent_validation contains the real per-point errors and
aggregate report. The original `calibrate-robot-plane --points` loader accepts
these arrays and ignores supplemental top-level metadata; compatibility is tested.

S, F and safe exit preserve new snapshots: the requested points path first,
then `_001`, `_002`, etc. Identical unchanged state reuses its already saved
snapshot. Paths are printed. Exclusive file creation prevents overwriting even
if another process creates the path after startup. A saved mapping is never
replaced; start another session with new paths when replacing/refining a map.
The wizard does not have a resume/import command; saved snapshots remain usable
with the existing noninteractive fitting command.

The mapping uses the **unchanged existing structure**:

```json
{
  "schema_version": 1,
  "mapping_type": "FIXED_HEIGHT_MARKER_PLANE",
  "calibration_id": "sha256:<actual calibration file digest>",
  "frame_size": [1280, 720],
  "floor_size_cm": [121.6, 68.4],
  "height_cm": 16.6,
  "tolerance_cm": 2.0,
  "attested_measured": true,
  "training": [],
  "validation": [],
  "homography": [["<3x3 fitted numeric matrix>"]],
  "validation_report": {
    "residuals_cm": [],
    "maximum_error_cm": "<measured number>",
    "rms_error_cm": "<measured number>",
    "tolerance_cm": 2.0,
    "attested_measured": true,
    "physical_validation_scope": "fixed height, measured points, frozen camera pose only"
  }
}
```

Placeholders illustrate the structure; actual output contains real arrays,
numbers and a 3×3 numeric matrix. Only training points fit the matrix. Every
hold-out's Euclidean error must be <=2 cm with the command above; errors are not
training residuals. Loading a mapping re-fits/revalidates its stored raw points
and checks the camera calibration digest, frame size and mounting height.

The only calibrator extension is a ValueError subclass carrying the existing
failed hold-out residuals and signed component errors, so the wizard can explain
a tolerance failure. Fitting method, rejection threshold, successful mapping
schema and existing error text are unchanged.

## Track with the saved map

```bash
.venv/bin/python main.py track-robot \
  --camera 0 --width 1280 --height 720 \
  --calibration config/camera_config.json \
  --robot-settings config/robot_config.json \
  --mounting-height-cm 16.6 \
  --marker-plane config/robot_marker_plane.json
```

Expected mapping method: `EMPIRICAL_FIXED_HEIGHT_PLANE`. Pose remains UNVERIFIED
and non-actionable. Fixed height, fixed camera, printed marker size, heading
alignment, chassis-center compensation and real hold-out accuracy still require
physical field checks. The current 6 cm black-square setting is retained; actual
approximately 9 cm print size still needs ruler confirmation. Size discrepancy
warnings do not substitute for measurement or height-plane calibration.

## Files and verification

Created:
- `vision/robot_calibration_wizard.py`: capture window, session state, keyboard UI,
  diagnostics, exclusive persistence, optional read-only chassis preview.
- `tests/test_robot_calibration_wizard.py`: 31 regressions, including simulated
  complete live UI workflow, missing/held/replayed/unstable rejection, counts,
  independence, undo, safe snapshots, source identity/height, original CLI
  compatibility, validation failure residuals, offset compensation and cleanup.
- `MODULE04_WIZARD_REPORT.md`: this operator guide and delivery report.

Modified:
- `vision/robot_cli.py`: new command and dispatch.
- `vision/robot_plane.py`: failed-validation diagnostic exception only.
- `main.py`: route the new command through the existing robot CLI.
- `README.md`: command and guide link.

Full suite: `.venv/bin/python -m pytest -q` — **637 passed**, **1 existing expected
legacy floor-only calibration warning**, 11.10 seconds. The command's help was
checked and the actual-arena wizard panel was rendered and visually inspected.
The arena calibration, robot settings and legal-background image/metadata hashes
were checked unchanged. No measured points or map were fabricated in the real
output locations. No physical camera capture or hardware accuracy claim is made.
