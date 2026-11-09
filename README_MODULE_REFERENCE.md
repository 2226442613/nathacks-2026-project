# 历史模块操作参考

> 本文件保留重写项目前的完整 README，供查阅旧版标定、诊断与 API 细节。
> 其中的项目状态、默认参数和协议结论可能已经过时；当前入口以 [README.md](README.md)、实际源码和运行时 CAPS 为准。
> 以下为原文归档，不是当前实机操作指南。

---

# Hospital Guardian — Modules 01–05

**Final architecture: fixed oblique iPhone Continuity Camera → macOS camera device → CameraManager/OpenCV on an M5 MacBook Air.** The camera covers the calibrated floor, vertical wall and robot. Local Python performs vision; future movement commands will use Wi-Fi to the robot controller, and a separate hardware team handles cleaning. Raspberry Pi/CSI cameras, Pi OS and Ethernet camera streaming have been permanently abandoned for this project; no such setup or dependency is required.

Module 01 provides camera input and independent floor/wall calibration. Module 02 registers an initial legal scene, detects subsequent new visual regions and classifies their foreground as red/non-red in **image coordinates only**, with local image/video workflows retained for testing. Camera input remains generic and replaceable. No separate iPhone streaming server, browser dashboard, autonomous navigation, robot control or cleaning execution is included. Module 05 adds offline contact/spray reachability and declarative cleaning coverage plans, always non-actionable. Module 04 adds optional single-ArUco robot tracking and dynamic body exclusion, with explicit height/parallax estimates. Module 03 adds optional mask-based floor/wall localization, bounded camera-motion compensation and persistent target messages; it does not authorize robot movement. This demonstration cannot identify real blood, vomit, pathogens or infectious material.

## Install on an M5 MacBook Air

Use an ARM64 Python 3.12 distribution. If Homebrew is not installed, follow its official installer instructions at https://brew.sh first (it may request Apple's Command Line Tools). Then, from this repository:

```bash
brew install python@3.12
python3.12 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install -r requirements.txt
python --version
uname -m
pytest -v
```

Expect Python 3.12.x and `arm64`. Python 3.11 is also supported. Avoid launching Terminal under Rosetta. Dependencies use macOS wheels; no CUDA, Docker, or ML framework is needed. Install only `opencv-contrib-python`, not `opencv-python` alongside it, because both supply `cv2`. Do not substitute a headless OpenCV package: native windows are required.

## Connect and preview

Make the iPhone Continuity Camera available to macOS, then find its OpenCV device index with the preview commands below. Secure the phone at a fixed oblique angle with the floor, wall and their shared edge visible. Close competing camera apps and avoid automatic reframing, zoom or crop changes between calibration, background capture and detection.

**Connecting an iPhone by USB does not guarantee video is routed exclusively over USB.** macOS manages the Continuity Camera transport. This application selects an OS camera index and does not determine or promise its underlying transport. CameraManager retains AVFoundation-first capture on macOS with fallback, configurable indices and requested frame settings. Generic local USB devices and prerecorded video files remain supported, without another camera implementation or a phone streaming server.

```bash
python main.py preview --camera 0
python main.py preview --camera 1 --width 1280 --height 720 --fps 30
python main.py preview --video path/to/video.mp4
```

Try different indices (`0`, `1`, `2`, etc.) to locate the iPhone; the Mac's built-in camera may occupy index 0. Verify the displayed scene rather than assuming a particular index names the phone. Startup logging reports actual capture properties when available. The overlay shows measured loop FPS, actual frame resolution and source. Requested capture settings are best effort; 1280×720 is requested but actual frames must be checked. FPS includes capture, processing, display and event-loop time; it is not the requested FPS. No background frame queue is maintained; a one-frame capture buffer is requested, but AVFoundation may ignore it.

Press **Q** (or Escape) to quit, **S** to save the original frame as a uniquely named PNG in `data/screenshots/`. Ctrl+C also releases capture and closes windows. Closing the window exits. Video EOF or decoder failure ends playback; there is no loop mode.

On first access, allow camera use for the application launching Python (Terminal, VS Code, or Codex). Check **System Settings → Privacy & Security → Camera** if denied, then restart the launching application after changing permission. For an unavailable device, first confirm the iPhone is available to macOS, check the index/permission and close competing camera apps. For a black screen, verify the phone's view in a native camera application, improve lighting and try the default resolution/FPS. A frame-read failure or Continuity Camera interruption produces a terminal camera error and releases capture; restore the connection, preview again, then restart detection. Verify background/calibration geometry before reusing them. Physical camera/permission behavior still needs field validation.

## Calibrate both planes (recommended setup)

Keep the camera fixed at its intended oblique angle; avoid a wide-angle lens and a nearly edge-on view of either plane. Use a rectangular floor and a rectangular vertical wall whose **complete bottom edge coincides with one complete floor boundary**. This version does not support a partial or offset shared edge. Measure floor width/depth and wall width/height in centimeters. The wall width must equal the chosen floor boundary length. Calibration needs **six unique clicks on the same frozen frame**: four floor corners, then two wall top corners. The two wall base points are inherited exactly from the chosen floor boundary, so you never click them again. Capture is released during selection.

```bash
python main.py calibrate --camera 1 --dual-plane --floor-boundary top \
  --arena-width-cm 120 --arena-depth-cm 80 \
  --wall-width-cm 120 --wall-height-cm 60 --grid

python main.py calibrate --image path/to/frame.jpg --calibration-source 1 \
  --dual-plane --floor-boundary top

python main.py verify-calibration --camera 1 --grid
```

Dimensions above are examples; supply your measurements or answer the terminal prompts before selection. Invalid dimensions or a wall width that disagrees with its shared edge are reported and can be corrected in the terminal. GUI windows are closed while terminal input is needed.

1. Select floor **F1 TL → F2 TR → F3 BR → F4 BL**, then press **Enter**. These labels define the floor's physical coordinates; an oblique image need not look rectangular. **R** clears the four floor selections. Invalid floor geometry stays in the selection window with an error, so you can reset and retry.
2. The wall stage shows the floor polygon and its highlighted shared edge. For the default `top` boundary without reversal, **W4 (wall BL) = F1** and **W3 (wall BR) = F2**, copied exactly from the saved floor coordinates. Labels explicitly identify inherited points; these two points are locked.
3. Click **only W1 (wall TL), then W2 (wall TR)**. The tool automatically constructs `[W1, W2, W3, W4]`. Additional clicks after the two top points are ignored. **R** resets only these two manually selected wall tops; **F** returns to edit the floor. Press **Enter** to validate and preview. Invalid wall geometry stays in the same window for correction; it is not saved.
4. Before saving, a review window displays **all six points**, the floor polygon in green, wall polygon in magenta, and shared boundary in cyan. Shared points have combined floor/inherited-wall labels. Separate windows show the rectified floor and wall. `--grid` adds ten physical divisions per axis. Check both outlines and measured points on each plane.
5. In review, press **Enter to save** if no quality warnings are present. With quality warnings, an additional report shows residuals and units; **Enter is blocked and A explicitly accepts the displayed warnings and saves**. **F** edits the floor, **W** edits the two wall tops, **R** clears all six selections, and **D** reopens the dimension prompts. When editing, previous points remain visible: press R to clear that stage or Enter to keep its points. Changing geometry recomputes quality and resets acceptance. Neither correction nor inheritance silently changes the chosen boundary or X direction.
6. **Q**, Escape, or closing a window before the final save cancels and leaves an existing configuration unchanged. Review/correction uses the original frozen frame; you do not need to restart the application. Saving closes the review; use `verify-calibration` for live verification afterward.

The wall's H=0 is its bottom edge at the floor; H increases **upward**, regardless of camera tilt. Wall image corners map to `(0,Hmax), (W,Hmax), (W,0), (0,0)`. The wall front view keeps high points at the image top; the floor view keeps +Y downward. Corner order must remain clockwise in the camera image; the tool cannot identify physical corners automatically.

### Shared geometry and wall-base projection

`--floor-boundary` identifies a boundary in **floor coordinates**, not a screen direction:

| Boundary | Wall X=0 floor position | Wall X=W floor position |
| --- | --- | --- |
| `top` | `(0, 0)` | `(floor_width, 0)` |
| `bottom` | `(0, floor_depth)` | `(floor_width, floor_depth)` |
| `left` | `(0, 0)` | `(0, floor_depth)` |
| `right` | `(floor_width, 0)` | `(floor_width, floor_depth)` |

Pixel endpoint inheritance follows the same convention: `top` uses **W4=F1, W3=F2**; `right` uses **W4=F2, W3=F3**; `bottom` uses **W4=F4, W3=F3**; `left` uses **W4=F1, W3=F4**. `--wall-x-reversed` swaps W4/W3's inherited endpoints and therefore the physical wall X direction. W1/W2 must still be selected as the corresponding wall top-left/top-right, with +H upward. The tool does not reorder your points to hide an incorrect orientation. Choose the boundary/reversal flags appropriate to your physical setup; geometry edits do not change these CLI flags.

New interactive calibrations reuse shared coordinates **exactly**, without rounding or a pixel/world round trip. This invariant is unchanged. The legacy **3 px endpoint** rejection threshold is also unchanged for older files/explicit four-corner constructors; it is not a tunable quality threshold. Wall width and boundary length must agree numerically (1e-6 tolerance), and both visible plane interiors must lie on opposite sides of the edge. Degenerate, concave, crossing, non-finite or incorrectly oriented polygons, non-invertible/invalid matrices, non-positive dimensions, camera/source inconsistency and missing required data remain hard errors. Warning acceptance cannot bypass these checks.

### Calibration quality: pixel diagnostics versus measured centimeters

The old `Floor and wall disagree along their shared boundary` rejection compared the two inverse projections of **five equally spaced physical X positions** on the common wall base, including endpoints. It measured Euclidean separation **in image pixels**, rejecting any sample above **3 px**. For ideal rectangular planes imaged by a distortion-free camera, corresponding points on the common edge should project identically; the comparison is mathematically meaningful. But matching the two endpoints does not determine the entire one-dimensional projective mapping. Manual corner errors, a non-rectangular measured patch, or lens distortion can leave different interior parameterizations even when both homographies are finite and invertible. It is a quality diagnostic, not proof that either matrix is singular. Equal widths 108.5 cm, floor depth 43 cm and wall height 29 cm are valid dimensions; those dimensions alone cannot diagnose the image residual without the six pixel positions.

The five-sample maximum and RMS discrepancies are now reported in **px**. A maximum **above 3 px** gives a quality warning by default; it does not prevent review. Configure this diagnostic with `--shared-edge-warning-px`. No centimeter threshold is applied to this pixel residual, and the inherited coordinates are never moved to hide a residual. Even a large finite residual is a warning requiring explicit acceptance; reselect or remeasure if the displayed disagreement is unsuitable for your demo. A low pixel residual is not an independently established physical accuracy result.

Independent physical accuracy can optionally be checked using points measured with a ruler on a known plane, **not the corner points used to fit the matrices**. Save a JSON array such as this schema illustration (replace with your own measured points):

```json
[
  {"surface": "FLOOR", "pixel": [300, 300], "expected_cm": [54.25, 21.5]},
  {"surface": "WALL", "pixel": [300, 150], "expected_cm": [54.25, 14.5]}
]
```

Pass the file with `--validation-points path/to/measurements.json`. Each point's Euclidean difference between transformed and independently measured coordinates is reported in **cm**. The maximum point error determines the physical quality band:

| Maximum independent error | Default status | Save behavior |
| --- | --- | --- |
| ≤ 2 cm | GOOD | Enter saves if no other warning |
| > 2 cm and ≤ 5 cm | WARNING | A required to accept warnings and save |
| > 5 cm | POOR | Strong warning; A required, remeasurement recommended |
| No independent points | NOT_ASSESSED | No physical accuracy claim; pixel warnings still apply |

Configure these separate centimeter bands using `--physical-good-cm` and `--physical-warning-cm`. Expected coordinates must be finite and inside the declared plane: floor `(X,Y)` or wall `(X,H)` with upward H. Validation points assess existing matrices; they do not refit them. They must come from the same frozen frame/camera geometry and are not arbitrary 3D measurements.

For the 108.5 × 43 cm floor and 108.5 × 29 cm wall setup:

```bash
python main.py calibrate --camera 0 --dual-plane --floor-boundary top \
  --arena-width-cm 108.5 --arena-depth-cm 43 \
  --wall-width-cm 108.5 --wall-height-cm 29 --grid \
  --shared-edge-warning-px 3 --physical-good-cm 2 --physical-warning-cm 5
```

Use the actual camera index, or replace `--camera 0` with `--image path/to/frame.jpg --calibration-source 0` for offline selection. The physical thresholds only evaluate independently measured points when supplied; they do not convert the pixel residual into centimeters. With warnings, review the report window/terminal, choose F/W/D to correct, Q to cancel, or **A** to explicitly accept and save. Enter alone cannot approve a warning.

Warnings, thresholds, pixel sample/max/RMS residuals, independent point errors and the acceptance flag are saved in an optional **`quality`** metadata block within JSON v2. Core camera/plane/shared-geometry fields and coordinate APIs are unchanged. Old v2 files without this block still load; good default calibrations without measurements keep the original minimal schema. On load, geometry and quality are recomputed from corners/measurements rather than trusting stored error summaries; recorded warnings are surfaced again with their acceptance state. Programmatic `save()` refuses unaccepted warnings; callers must inspect `quality_report` and explicitly call `accept_quality_warnings()` before saving. `set_quality_validation()` resets acceptance when adding/changing measurements or thresholds.

`wall_to_floor_base(x, h)` returns the floor coordinate directly below that known wall point, after checking wall bounds. It is **a projection, not a reachable robot destination**. Later modules must consider wall spill height, mechanism reach, and a suitable distance from the wall. `RobotReachGeometry` stores maximum reachable height and required wall distance as inputs only; this module makes no reachability or navigation decisions.

### Legacy floor-only workflow

Commands without `--dual-plane` retain the original floor-only behavior and write version 1. `--arena-height-cm` remains an alias for floor depth. Use this only when you intentionally want a floor-only calibration.

```bash
python main.py calibrate --camera 1
python main.py calibrate --camera 1 --arena-width-cm 120 --arena-height-cm 80
python main.py calibrate --image path/to/frame.jpg --calibration-source 1
```

The dimensions above are examples; use your measurements. If omitted, dimensions are requested in the terminal after corner confirmation, with the selection window closed so GUI events are not blocked. For offline images, `--calibration-source 1` explicitly associates the image with camera 1: use it only if that image came from exactly the same camera, resolution, lens/zoom/crop, and mounting geometry. Without this option, the image path is stored as the source and live-camera verification refuses to use it.

1. Click **top-left → top-right → bottom-right → bottom-left**. Use the same physical corner convention throughout the project. The tool validates clockwise convex geometry but cannot infer your intended physical top-left corner.
2. Markers and connecting edges show the selection. Press **R** to reset; **Q** or Escape cancels without saving.
3. Press **Enter** after all four points. Enter measured width/height in the terminal if not supplied by CLI.
4. A valid calibration saves to `config/camera_config.json`, replacing the previous calibration, and displays a static bird's-eye view. Invalid geometry/dimensions produce an error; rerun calibration to correct them.
5. Press **Q** to close the bird's-eye view.

No placeholder calibration is shipped. The config directory is created automatically when saving, and the local calibration is gitignored. Calibration replaces the selected configuration only after all requested planes validate.

## Verify and reuse

```bash
python main.py verify-calibration --camera 1
python main.py verify-calibration --camera 1 --config config/camera_config.json --pixels-per-cm 5
```

Verification loads and validates saved data, overlays the saved polygons on live frames, and displays each calibrated plane's rectified view. Confirm that the polygons match the physical corners, straight lines remain straight, and independent measured points map to expected centimeter coordinates. Source mismatch is rejected. Actual frame dimensions must match the calibration; mismatch emits a warning and refuses to apply either transform. The program cannot detect a moved camera at the same resolution. Recalibrate both planes after moving the camera, wall or floor, changing lens/zoom/crop, resolution, or capture orientation. Do not reuse a calibration based only on matching resolution.

`load_calibration()` reads both versions. Version 1 returns a floor-only object with an explicit warning that the wall is unavailable; it is never promoted to a dual-plane calibration. `DualPlaneCalibration.load()` requires a complete valid version 2 file and rejects floor-only files. Version 2 stores shared camera metadata, floor width/depth/corners/homography, wall width/height/corners/homography, floor boundary and wall X reversal. Its schema is unchanged by the six-point UX: all four wall corner coordinates, including both inherited base endpoints, are explicitly saved. Previously valid version 2 files load normally; no exact-equality restriction has been retroactively imposed on old files. Missing or invalid wall data causes the whole version 2 file to be rejected. Rerun `calibrate --dual-plane` to replace an old file; no geometry is invented during migration.

Rectified-view scale defaults to 5 pixels/cm and is capped at 2048 pixels on its longest side to bound memory. Physical aspect ratio is preserved to integer pixel rounding; endpoints correspond to `(width-1, height-1)`. Extremely thin or tiny regions may round to a two-pixel dimension. Yellow labels show the floor's top-left origin and +X/+Y axes, or the wall's bottom-left origin and +X/+H axes. Axis arrows are inset for legibility. Coordinate conversion uses the matrix directly and does not generate a rectified frame.

## CLI reference

Run commands from the repository root. Every command has `--help`:

```bash
python main.py --help
python main.py preview --help
python main.py calibrate --help
python main.py verify-calibration --help
```

| Option | Commands | Default / meaning |
| --- | --- | --- |
| `--camera INDEX` | all | 0; physical camera index |
| `--video PATH` | preview, verify-calibration | Local file instead of camera |
| `--image PATH` | calibrate | Local image instead of camera |
| `--width`, `--height` | all | Requested camera resolution, 1280 × 720; ignored for files/images |
| `--fps` | all | Requested camera FPS, 30 |
| `--config PATH` | all | `config/camera_config.json`; read by verify, written by calibrate, unused by preview |
| `--pixels-per-cm SCALE` | all | 5; used by calibrate/verify bird's-eye, unused by preview |
| `--screenshots PATH` | preview, verify-calibration | `data/screenshots` |
| `--arena-width-cm`, `--arena-height-cm` | calibrate | Prompted if omitted |
| `--arena-depth-cm` | calibrate | Alias for `--arena-height-cm`, now interpreted as floor depth |
| `--calibration-source INDEX` | calibrate | Explicit camera association for an offline image |
| `--dual-plane` | calibrate | Select floor then wall; save version 2 (otherwise floor-only version 1) |
| `--wall-width-cm`, `--wall-height-cm` | calibrate | Prompted for dual-plane calibration if omitted |
| `--floor-boundary` | calibrate | `top` (default), `right`, `bottom`, or `left` |
| `--wall-x-reversed` | calibrate | Reverse wall X along the shared floor boundary |
| `--shared-edge-warning-px` | calibrate, dual-plane | 3 px; quality warning threshold, not endpoint tolerance |
| `--physical-good-cm`, `--physical-warning-cm` | calibrate, dual-plane | 2 cm and 5 cm; independent measured-point quality bands |
| `--validation-points PATH` | calibrate, dual-plane | Optional independent surface/pixel/expected_cm JSON array |
| `--grid` | all | Ten divisions per axis in rectified views; unused by preview |

Camera and video/image options are mutually exclusive. All paths may be relative to the current directory or explicitly configured. Failures log a terminal error and return exit code 1; normal completion and Ctrl+C return 0. HighGUI operations must run on the main Python thread.

## Public API for later modules

Dual-plane usage:

```python
from vision import DualPlaneCalibration, SurfaceType, RobotReachGeometry

calibration = DualPlaneCalibration.load("config/camera_config.json")
# For every incoming frame, validate actual dimensions and keep the physical camera fixed.
calibration.validate_resolution(frame.shape[1], frame.shape[0])
x, y = calibration.pixel_to_floor(u_floor, v_floor)
x_wall, height = calibration.pixel_to_wall(u_wall, v_wall)
u, v = calibration.wall_to_pixel(x_wall, height)
floor_base = calibration.wall_to_floor_base(x_wall, height)
location = calibration.localize(SurfaceType.WALL, u_wall, v_wall)
# location.wall_xh_cm, location.floor_xy_cm (base projection), location.height_above_floor_cm
robot_geometry = RobotReachGeometry(max_reachable_height_cm=40, required_wall_distance_cm=12)
# Geometry inputs only; no reachability decision or robot target is calculated.
floor_view = calibration.floor.birdseye(frame, grid=True)
wall_view = calibration.wall.birdseye(frame, grid=True)
```

The caller must know which plane a point lies on; `localize()` requires an explicit `SurfaceType.FLOOR` or `SurfaceType.WALL`. It does not classify surfaces. `floor_to_pixel(x,y)` is the floor inverse. `WallCalibration` reuses the existing `GroundCalibration` solver, validation, inverse transformation, and rectification rather than maintaining a second implementation. `wall.pixel_to_wall()`/`wall.wall_to_pixel()` name the height convention explicitly. `SurfaceCoordinates` preserves surface and centimeter coordinates; `RobotReachGeometry` validates finite nonnegative geometry inputs without evaluating them. Only wall-base projection bounds its input to the calibrated wall (with 1e-6 numerical tolerance); generic matrix transforms can extrapolate.

For six-point construction in code, `DualPlaneCalibration.wall_base_corners(floor, boundary, wall_x_reversed=False)` returns the exact inherited **[W4 BL, W3 BR]** pixels. `DualPlaneCalibration.from_floor_and_wall_top(floor, [W1, W2], wall_width_cm, wall_height_cm, floor_boundary, wall_x_reversed=False)` constructs all four wall corners and runs the existing homography and shared-geometry validation. Existing constructors and coordinate/persistence APIs remain available.

Both dual-plane constructors accept optional keyword `quality_thresholds=CalibrationQualityThresholds(...)` and `validation_points=(PhysicalValidationPoint(...), ...)`. `load_validation_points(path)` reads the independent-point JSON. `quality_report` exposes separate pixel consistency, physical accuracy (or NOT_ASSESSED), warnings and current acceptance. Acceptance affects saving only and cannot make invalid geometry usable.

Existing floor-only API remains available:

```python
from vision import CameraManager, CameraError, GroundCalibration, CalibrationError

calibration = GroundCalibration.load("config/camera_config.json")
with CameraManager(source=1, width=1280, height=720, fps=30) as camera:
    frame = camera.read()  # BGR NumPy array; None at local-file EOF/decode failure
    if frame is not None:
        calibration.validate_resolution(frame.shape[1], frame.shape[0])
        x_cm, y_cm = calibration.pixel_to_world(520, 310)
        u, v = calibration.world_to_pixel(x_cm, y_cm)
        rectified = calibration.birdseye(frame, pixels_per_cm=5)  # optional
```

- `CameraManager(source, width, height, fps)`: integer camera index or string/Path local video; `open()`, `read()`, `release()`, `settings` (actual width/height/FPS or None), context manager cleanup. File EOF returns None; camera failure raises `CameraError`. File paths cannot be network URLs.
- `GroundCalibration(corners_pixels, width_cm, height_cm, frame_width, frame_height, source=0, requested_fps=30)`: validates geometry and calculates forward/inverse homographies. Construct with the actual captured frame dimensions.
- `pixel_to_world(u, v)` / `world_to_pixel(x, y)`: return float pairs; reject nonfinite inputs and projective horizons. Points outside the arena can be extrapolated; consumers should check arena bounds if needed.
- `validate_resolution(width, height)`: raises `CalibrationError` and warns on mismatch. Future consumers must call this on incoming frame geometry and ensure the physical setup/source is unchanged.
- `save(path)`, `load(path)`, `to_dict()`: versioned JSON with camera metadata, arena dimensions, four pixel corners and actual homography. Loading validates schema and consistency and recomputes the inverse.
- `birdseye(frame, pixels_per_cm=5, max_side=2048)`: annotated BGR visualization, with a bounded output size (max_side 2–4096).

Pixel `(u,v)` uses image coordinates. Floor world `(x,y)` uses **centimeters**, origin at the calibrated floor's top-left; +X points right, +Y follows floor depth downward in its rectified view. For width W and depth D, floor corners map to `(0,0), (W,0), (W,D), (0,D)`. Wall world `(x,h)` uses centimeters with H=0 on the shared base and +H upward. A single ordinary RGB camera **cannot recover arbitrary 3D height or depth**. Wall height is valid only for points known to lie on the calibrated wall; use the separate floor transform only for points known to lie on the floor. Elevated robot ArUco markers lie on neither plane and introduce parallax requiring future compensation. Lens distortion is not calibrated here; substantial distortion reduces accuracy.

## Tests and verification limits

```bash
source .venv/bin/activate
pytest -v
```

Tests require no camera or GUI: known rectangle, inverse/perspective round trips, invalid dimensions/corners/matrices/JSON, persistence, resolution mismatch, bounded bird's-eye mapping, mocked AVFoundation fallback and camera failures, generated MJPG video reading/EOF, screenshots, and mocked UI selection/reset/save/preview/cleanup. GUI mocks test control flow, not native window rendering or real input events. The suite deliberately does not access a physical camera. Actual iPhone capture, camera permission dialogs, native window interactions, physical measurement accuracy and latency must be verified before declaring hardware acceptance.

Additional dual-plane tests cover a synthetic oblique projective camera, wall H=0/upward height, both inverses, all four floor boundaries and reversed X, wall-base projection, explicit surface dispatch, future robot geometry inputs, shared-edge consistency, version 2 round trips, missing/invalid wall records, legacy floor-only handling, matching camera geometry, bounded upright wall views with optional grid, dual-selection/verification GUI control flow, and preserving saved configuration when wall calibration is cancelled or invalid. No physical 3D accuracy is claimed from these synthetic checks.

Six-point UX regressions additionally verify exact/subpixel endpoint reuse for every boundary and both X directions, complete wall homography construction, unchanged v2 JSON, two-click wall-top selection with locked base points, invalid-geometry correction in the same window, explicit review before save, floor/wall/reset/dimension corrections, and cancellation without replacing an existing file. GUI control flow is mocked; physical iPhone and native window interactions still require field verification.

Module 02 reuses CameraManager frames and calibration pixel polygons without modifying the Module 01 capture or calibration implementation.

## Module 02 — registered legal scene and new objects

**Everything registered in the initial scene is legal. Only subsequent visual changes are candidates.** Existing cardboard, logos (including red logos), text, folds, tape, shadows and objects remain background. A foreground component is RED_SPILL when its red evidence meets the full-component threshold or the guarded interior rule below; otherwise it is NON_RED_SPILL, including cream, gray and other low-saturation objects. These are compatibility labels for visual demo targets, not material or biological recognition. UNKNOWN remains supported by the API/tracker but is no longer the routine classification of non-red regions.

### Register once, then monitor

From `/Users/williamwu/natHack`, use the existing iPhone CameraManager. Camera 0 must actually be the calibrated iPhone: check the preview before capture, stop preview before detection, and keep position/crop/resolution fixed. The saved Module 01 calibration is source 0 at 1280×720. Actual floor/wall dimensions come from calibration, never detector constants.

```bash
cd /Users/williamwu/natHack

# 1. Preview; Q/Escape exits. Confirm the intended initial legal scene.
.venv/bin/python main.py preview --camera 0 --width 1280 --height 720 --fps 30

# 2. Explicit registration, using a NEW path to preserve the previous reference.
.venv/bin/python main.py background-reference --camera 0 --width 1280 --height 720 --fps 30 \
  --reference-frames 15 --aggregation median --output data/legal_scene.png --confirm-clean

# 3–4. Continuous live monitoring with the clean operator GUI; Q/Escape or Ctrl+C exits.
.venv/bin/python main.py detect-spills --camera 0 --width 1280 --height 720 --fps 30 \
  --background data/legal_scene.png --confirm-clean --calibration config/camera_config.json \
  --settings config/spill_config.json --output data/legal_live.jsonl

# 5. Detailed diagnostics: pending candidates, contours, evidence and mask windows.
.venv/bin/python main.py detect-spills --camera 0 --width 1280 --height 720 --fps 30 \
  --background data/legal_scene.png --confirm-clean --calibration config/camera_config.json \
  --debug --show-masks --debug-dir data/legal_debug --max-frames 30 --output data/legal_debug.jsonl

# Explicit re-registration ONLY when the operator intends all currently visible items to become legal.
.venv/bin/python main.py background-reference --camera 0 --width 1280 --height 720 --fps 30 \
  --reference-frames 15 --aggregation median --output data/legal_scene.png --confirm-clean --overwrite
```

Camera/video registration defaults to 15 frames, image registration to one. `--reference-frames 1` remains supported explicitly. Aggregation accepts 1–31 frames, median by default or `--aggregation mean`. All selected frames must contain the intended legal scene: do not move test markers through the registration interval. PNG plus `.png.json` metadata records dimensions, BGR ordering, aggregation, registration time, source and capture start frame. Older version-1 reference metadata and references without sidecars remain readable when explicitly trusted. Existing baseline or sidecar files cause an error unless `--overwrite` is supplied; the Python equivalent is `reference.save(path, overwrite=True)`. No baseline is captured implicitly on detection startup; missing baseline requires explicit registration. Startup reports LOADED/REGISTERED and dimensions. `--confirm-clean` now explicitly trusts everything in the baseline as legal; the current monitoring scene may contain new test targets.

Normal detection never writes the reference or calibration. A target remains new even after hours of being stationary. Only an intentional baseline registration can legalize it. Baseline recapture does not recalibrate the camera; significant camera movement also requires Module 01 verification/recalibration.

### Algorithm, camera checks and settings

1. Validate uint8 BGR input, baseline/calibration dimensions and exclusions.
2. Compare distributed ORB background features using RANSAC. Reliable movement within the small-alignment limit warps the baseline into the **current original image coordinate system**, not the incoming image. Warp borders are excluded. Saved homographies and pixel polygons remain unchanged.
3. Fit bounded per-channel exposure/white-balance gain/offset using sub-threshold background samples. Fit at most 4096 samples. No adaptive background model or reference learning.
4. Threshold max BGR difference, apply floor/wall union ROI and exclusions, then opening/closing and connected components. A narrow outer ROI margin suppresses edge artifacts. The shared floor/wall interior edge is retained to avoid splitting one object into duplicate targets.
5. Keep sufficiently large components within configured area limits. Classify **only original BGR component pixels**, using both red HSV hue ranges, RGB-supported dim-red evidence and guarded interior statistics. Brown holes and unrelated background inside the bounding box never enter the red fraction. No surface-specific color transform, shape/material gate or rectified color image is used. Boundary/truncation information remains diagnostic.
6. Associate components one-to-one; confirm only consecutive, category-consistent, stationary observations with sufficiently stable box overlap and area. Maintain IDs, reset streaks after movement/gaps/category changes, and expire missing tracks. Return exact component masks without learning targets into the baseline.

`config/spill_config.json` remains settings version 1. Missing new fields receive defaults. Recommended starting values:

| Setting | Default | Meaning |
| --- | --- | --- |
| `difference_threshold` | 30 intensity levels | Minimum corrected max-channel BGR difference |
| `illumination_correction`, `illumination_gain_correction` | true | Bounded per-channel offset and gain correction |
| `illumination_max_shift`, `illumination_max_gain_change` | 25, 0.15 | Offset bound and gain bound 1±0.15 |
| `blur_kernel`, `opening_kernel`, `closing_kernel` | 3, 3, 5 px | Odd kernels; 1 disables each operation |
| `min_area_pixels`, `max_area_fraction`, `max_area_pixels` | 80, 0.35, 0 | Area bounds; absolute maximum 0 disables that cap |
| `roi_margin_pixels` | 3 px | Erode the floor/wall union's outer edge, not the shared seam |
| `exclusion_padding_pixels` | 3 px | Padding around existing static/dynamic exclusions |
| `red_hue_low_max`, `red_hue_high_min` | 10, 170 | OpenCV red hue wraparound bands |
| `red_min_saturation`, `red_min_value` | 100, 50 | Existing strict per-pixel red evidence |
| `red_shadow_enabled` | true | Supplement strict HSV with bounded dim/desaturated red support |
| `red_shadow_min_saturation`, `red_shadow_min_value` | 65, 25 | Supplementary HSV lower bounds; hue bands stay unchanged |
| `red_shadow_min_normalized_excess` | 0.08 | Supplement requires `(R-max(B,G))/(R+G+B) ≥ 0.08` |
| `red_fraction_threshold` | 0.70 | Full-component red fraction, or eligible guarded interior fraction |
| `red_interior_erosion_pixels` | 1 px | Interior color samples only; 0 disables; geometry/mask never eroded |
| `red_interior_min_pixels`, `red_interior_min_fraction` | 25, 0.50 | Interior must retain both ≥25 pixels and ≥50% of component |
| `red_interior_min_full_ratio` | 0.50 | Interior can rescue only components already ≥50% red overall |
| `alignment_enabled`, `alignment_max_displacement_pixels` | true, 2 px | Reliable baseline-to-current small-motion correction; bounds maximum corner displacement |
| `unverified_max_foreground_fraction` | 0.15 | Legacy parsed setting; no longer gates detection in operator-managed mode |
| `tracker.confirmation_frames`, `max_missed_frames` | 3, 2 | Stable observations before confirmation and bounded missing-track retention |
| `tracker.stationary_distance_pixels`, `confirmation_min_iou`, `confirmation_max_area_ratio` | 4 px, 0.6, 1.35 | Confirmation anchor radius, successive box overlap and larger/smaller area ratio |

Existing `max_aspect_ratio`, `min_fill_ratio`, `min_solidity`, `min_mean_color_difference`, `non_red_max_red_fraction`, `color_min_saturation`, `color_min_value`, `min_color_fraction`, and `max_edge_density` remain parsed for old-settings compatibility but no longer control classification. `boundary_fraction`/`boundary_margin_pixels` only describe shared-boundary overlap. Tune effective parameters using saved raw inputs/masks, not the obsolete shape/material knobs. Do not disable geometry checks just to hide a framing warning.

MATCHED means reliable background registration within configured limits, not measured physical accuracy. Framing diagnostics are non-blocking for camera, image, video and headless detection. A transform exceeding the 2 px safe-correction limit or major-motion bounds is retained as a MOVED warning; unreliable matches remain UNVERIFIED. Detection continues against the unchanged legal baseline. Reliable bounded small shifts still align the background; unsupported transforms are not applied. Broad changes can cause false foreground detections after camera movement; the operator decides whether to recapture or recalibrate. Existing component size and segmentation filters remain active. Invalid input types/dimensions and actual camera failures remain errors.

### Operator view, debug and output

The default window shows original live video, optional floor/wall outlines, **confirmed-only** target boxes with IDs/categories, confirmed count, measured processing FPS, baseline status and geometry status. It shows no pending boxes, scores or debug contours. `--no-polygons` hides only outlines. `--debug` reveals pending candidates/contours/areas/reasons in records and detailed overlay counts without opening extra mask windows. `--show-masks` explicitly enables five diagnostic windows and the detailed view. `--headless` calls no GUI functions; normal headless runs skip annotation entirely. No images are exported unless `--debug-dir` is given. Image mode processes once (use `--confirmation-frames 1` for explicit single-image confirmation); video mode processes sequentially without playback pacing.

Each debug frame exports eight PNGs: original `input`, `annotated`, `raw_difference` (before exposure correction, after optional alignment), corrected `difference`, `raw_foreground` (threshold/ROI/exclusions before morphology), filtered `foreground`, HSV `red` intersection, and allowed `analysis` mask. Compare input against the actual registered baseline to locate print/seam/shadow/reflection artifacts. Existing red pixels alone never create a target.

Debug export also writes `frame_NNNNNN_diagnostics.json` (the same synchronized JSONL frame record) and, when component extraction ran, `frame_NNNNNN_components.npz` containing the full original-coordinate `component_labels` array. `labels == detection['component_id']` reconstructs the exact mask, including holes. Only an explicitly confirmed baseline capture temporarily pauses extraction; framing warnings do not pause it.

JSONL preserves `frame_index`, `detections`, geometry, photometric diagnostics and frame diagnostics. Each detection retains `target_id`, `category`, `pixel_center`, `bounding_box`, `contour`, `area_pixels`, `detection_score`, `classification_reason`, `boundary_overlap`, `confirmed`, `consecutive_frames`, observation timestamps and `component_id`. For compatibility JSON includes pending observations explicitly marked `confirmed=false`; consumers must filter `confirmed` for operational events. `detection_score` is normalized difference strength, not material probability. `diagnostics.detection_paused`/`alignment_applied` describe registration handling; original pixel coordinates feed the optional Module 03 localizer. `DetectionResult.mask_for(target)` returns the exact full-resolution component mask including holes for that frame. Missing observations disappear immediately from current output, while private tracks expire after the missed-frame allowance.

NumPy booleans/integers/floats/arrays and nested containers are converted strictly to native JSON. NaN/Infinity and unsupported values fail with a path; serialization errors log frame/source/output and traceback. Expected errors and programming failures exit nonzero, release CameraManager and close windows. Category UNKNOWN is not used to disguise camera or geometry errors.

### Floor/wall red classification diagnosis

Pipeline trace: original BGR frame → background-only alignment/exposure compensation for segmentation → corrected BGR difference → ROI/morphology → connected-component labels → exact original-coordinate component pixels → HSV plus normalized RGB evidence → full-mask/interior red statistics → RED_SPILL/NON_RED_SPILL. The incoming frame itself is never rectified or color-normalized for classification. Floor and wall share the same code and thresholds; ROI polygons only limit segmentation and provide diagnostic membership.

Code inspection confirmed that the earlier classifier already used original frame colors and exact component pixels, not bounding-box/background/difference colors. No floor/wall-specific color transform was found. Two failure mechanisms were reproduced synthetically: `(B,G,R)=(5,5,45)` has red hue but failed the old `V>=50` gate (zero red pixels), and a 4-pixel-wide red target acquired a brown segmentation halo, reducing its full-component red ratio to 0.667 despite a fully red interior. The existing archived debug frames predate this reported wall failure, and no corresponding wall-failure sample has been identified; these reproductions do **not** establish which mechanism caused the physical report.

The repair preserves existing strict HSV evidence and hue ranges `[0,10]`/`[170,179]`. Supplementary dim/desaturated pixels must remain in those same hue bands, meet `S>=65,V>=25`, and have normalized RGB red excess ≥0.08. Yellow/orange outside the configured hue bands cannot pass this supplement. Full-component ratio ≥0.70 still passes directly. Otherwise, a 1 px interior sample may pass only if it retains ≥25 pixels and ≥50% of the component, its red ratio is ≥0.70, and the entire component is already ≥50% red. Small or thin interiors failing those support bounds fall back to the full-component statistic. This interior never changes component labels, contour, area, center or tracking geometry; a tiny red core in a mostly non-red component cannot override the overall result. All parameters are in the existing detector settings.

`--debug`, `--show-masks` or `--debug-dir` requests optional top-level `color_diagnostics` records. Normal operator JSON/GUI stays compact. Each debug target includes:

- `target_id`, `component_id`, `surface_roi` (FLOOR/WALL/BOUNDARY/UNKNOWN), per-plane pixel counts, area, box and center. Membership is overlap with the saved pixel ROI, not verified physical surface ownership, especially after a framing override.
- Full-component `red_pixel_count`, `total_component_pixel_count`, `red_pixel_ratio`, strict/supplementary red counts and hue-in-range count.
- `mean_hsv` and `median_hsv` from original component pixels (H 0–179, S/V 0–255). Hue mean/median are arithmetic diagnostics: red wrapping around 0/179 can make them misleading; the classifier uses per-pixel hue bands, not the mean hue.
- Interior counts/eligible ratio, configured red threshold, threshold settings, classification basis, final category and full classification reason.
- `color_source=original_bgr_frame` and `coordinate_system=original_image_pixels`.

For the actual failing wall object, compare the source with its exact label mask first. A mask covering large cardboard/shadow regions indicates segmentation contamination; high hue-band support but low strict S/V counts indicates a color gate problem. Debug output keeps the raw ratio visible even when interior evidence rescues the category. A substantially incorrect component or colors outside the configured hue ranges still need operator inspection; this is not automatic material recognition or a guarantee across arbitrary lighting.

Run one camera command at a time, preserving the current baseline/calibration:

```bash
cd /Users/williamwu/natHack
live=(.venv/bin/python main.py detect-spills --camera 0 --width 1280 --height 720 --fps 30
  --background data/iphone_reference.png --confirm-clean
  --calibration config/camera_config.json --settings config/spill_config.json)

# Normal live operator view.
"${live[@]}" --output data/wall_live.jsonl

# Per-target red diagnostics in JSONL, with no extra mask windows.
"${live[@]}" --debug --output data/wall_color.jsonl

# Synchronized original frames, masks, component labels and per-frame diagnostics.
capture_dir="data/wall_red_$(date +%Y%m%d_%H%M%S)"
"${live[@]}" --debug --show-masks --debug-dir "$capture_dir" --max-frames 60 --output "$capture_dir.jsonl"
```

With the camera fixed, move the same clearly red target between floor and wall, then repeat under mild shadow/white-balance changes. Capture any NON_RED result and compare its counts/HSV/interior basis; add yellow and orange controls. The commands load the existing baseline; no registration command is involved. Framing warnings are non-blocking as described below, and R→Y deliberately replaces the legal scene, independently of color diagnostics. Physical iPhone validation of this color repair remains pending; synthetic success is not a physical accuracy claim.

### Operator-managed framing and explicit baseline capture

Camera framing is now operator-managed. MOVED/UNVERIFIED diagnostics never stop detection, pause it, reset IDs or require C. This also applies to headless, image and video workflows. Normal operator mode shows a compact `CALIBRATION: ESTIMATED | UNVERIFIED` indicator; detailed geometry and legacy state diagnostics are available in debug mode and JSONL.

| Key | Behavior |
| --- | --- |
| **C** | Optional legacy key; no acknowledgement is required to keep detecting. |
| **R** | Requests explicit baseline replacement. Shows that all visible objects will become legal; does not write or stop detection while awaiting confirmation. |
| **Y** after R | Captures a fresh median baseline (15 frames by default; `--reregister-frames N` supports 3–31), writes it explicitly, resets tracks and resumes detection. Only this requested capture pauses target extraction. |
| **N** after R | Cancels without saving or resetting tracks. |
| **Q**, Escape, Ctrl+C or window close | Releases capture and closes windows; unfinished capture never replaces the baseline. |

No baseline is replaced automatically and stationary targets never become legal by waiting. Repeated R cannot confirm a write. Re-registration does not alter homographies or establish centimeter accuracy. Original ROI/exclusion polygons remain diagnostic estimates after movement; `--no-roi` is available for explicit image-wide inspection. The operator decides when Module 01 recalibration is necessary.

JSONL retains `diagnostics.framing`, geometry quality, registration reasons and calibration warnings. Re-registration metadata continues to record `calibration_may_be_invalid=true`; the normal GUI uses the compact UNVERIFIED indicator. These warnings do not suppress Module 03 estimates. No target message authorizes motion.

Reference saving stages and flushes PNG/metadata before replacing anything. Each destination replacement is atomic; ordinary image-commit failures roll back metadata and leave the previous image active. A SHA-256 in metadata rejects mismatched PNG/metadata after an interruption between the two renames. This is not a single atomic transaction across two files under power loss; an interrupted pair is rejected with an explicit recovery error. Failed interactive saves log the traceback, retain the previous in-memory baseline and resume non-blocking detection. Existing references without the checksum remain compatible. Interactive replacement requires a PNG baseline.

Launch against the existing iPhone baseline (no recapture occurs merely by launching):

```bash
cd /Users/williamwu/natHack
.venv/bin/python main.py detect-spills --camera 0 --width 1280 --height 720 --fps 30 \
  --background data/iphone_reference.png --confirm-clean \
  --calibration config/camera_config.json --reregister-frames 15 \
  --output data/iphone_interactive.jsonl
```

### Offline images and videos

```bash
# Generate deterministic fixtures in a separate demo directory, not the real baseline.
.venv/bin/python tools/generate_spill_demo.py --output-dir data/legal_demo

# 6. Image workflow (the generated clean image is already the known legal baseline).
.venv/bin/python main.py detect-spills --image data/legal_demo/spills.png \
  --background data/legal_demo/clean.png --confirm-clean --confirmation-frames 1 \
  --headless --output data/legal_demo/image.jsonl

# 7. Video workflow; the demo includes a masked moving-object lane and transient artifacts.
.venv/bin/python main.py detect-spills --video data/legal_demo/spills.avi \
  --background data/legal_demo/clean.png --confirm-clean --exclusion-mask data/legal_demo/exclude.png \
  --headless --output data/legal_demo/video.jsonl

.venv/bin/python -m pytest -q
```

Do not apply the real 1280×720 calibration to 400×320 generated fixtures. Manual exclusions retain existing `--exclusion-mask`/`--exclusion-polygons` behavior; dynamic exclusions remain available through `detect(frame, exclusion_mask=...)`. `--no-roi` is explicit full-image analysis. Public Module 01/02 imports and transformation APIs remain intact; the detector never calls world-coordinate mappings.

### Validation, performance and physical checklist

Module 02 baseline before Module 03: **344 passed**, with one expected legacy floor-only calibration warning (`.venv/bin/python -m pytest -q`). Python compilation checks also passed. Interactive tests additionally cover both motion limits, explicit C/R/Y/N/Q decisions, no pre-decision detections, live event-loop progress, override session lifetime, calibration preservation/warnings, re-registration/track resets, capture cancellation, staged-write/rename failure rollback and checksum rejection. These use mocked cameras and GUI events; physical iPhone/native-window validation has not been performed by these tests. The operator subsequently saved a new Module 01 calibration; Module 03 preserves its current file and accepted quality warnings (see below).

Synthetic regressions cover legal red logos/text/folds/tape, new red/yellow/cream/blue/dark objects, foreground-only red ratios and holes, separate floor/wall IDs, noise/exposure/white balance/small shadows, long-lived stationary targets, removal, ROI/exclusions/margins, reliable small-shift alignment and original pixels, non-blocking registration warnings, quiet operator GUI, explicit median registration/overwrite protection, JSONL and prior Module 01/camera/offline workflows. Old tests expecting low-saturation or irregular objects to become UNKNOWN were updated to the required binary semantics; no legacy test modules were removed.

Avoidable overhead removed: whole-frame Canny/shape-likelihood calculations, full-frame HSV classification, unbounded gain-fitting samples, redundant baseline copies and annotation in normal headless processing. Normal operation exports no PNGs. On this development host, 20 warmed repeated calls on the saved 1280×720 reference measured median **38.18 ms**, mean **37.83 ms**, detector-only throughput **26.44 FPS**. This is an offline identical-image measurement, excluding camera capture, GUI, tracking and JSON I/O; it is not iPhone live FPS or an accuracy benchmark. The GUI/JSON processing FPS remains measured from actual loop elapsed time, never the requested capture FPS. Physical performance must still be measured, particularly with debug windows/exports enabled.

Physical acceptance still to perform:

1. Register the intended legal scene with existing red logos/text/tape; monitor and record **0 confirmed targets**.
2. Add a new red marker: after the stable streak, record **1 RED_SPILL**.
3. Add a separated yellow/blue marker, including on the wall: record **2 targets**, separate stable IDs and correct categories.
4. Keep both stationary long enough to verify neither disappears into the baseline; then remove one and check current output disappears and its private track expires.
5. Inspect mild lighting changes/noise and collect raw frames/masks for any artifact; shift the phone noticeably and verify continued preview/detection, compact UNVERIFIED status and diagnostic fallback estimates. Baseline-relative false foregrounds after movement are an accepted limitation.
6. Re-register only on purpose; confirm `--overwrite` is required for an existing path. Recalibrate as well if the physical camera pose changed.

Remaining limits: insufficiently contrasting objects can be invisible; strong/local lighting, shadows/reflections and compression can still create changes; removal of something registered in the baseline can also create a changed region. Touching objects may merge, crossing/occlusion can switch IDs, low-texture UNVERIFIED cannot establish framing, and area/edge bounds can exclude genuine large or edge targets. Strong global scene changes can create false foregrounds or be filtered by existing component-area limits; framing diagnostics do not pause detection. This is deterministic change detection, not semantic recognition. Synthetic success does not establish physical iPhone accuracy, macOS native GUI acceptance or clinical suitability. Saved calibration remains unchanged; no Pi, ML, robot control or ArUco implementation is added. Module 03 consumes this pixel/contour/mask contract when explicitly enabled; the physical demonstration still requires the checklist above.

## Module 03 — surface localization and target events

Enable explicitly with `detect-spills --localize`. Without this flag, Module 02 commands, categories, tracking and JSONL behavior remain unchanged. The localizer adds `localizations`, `target_events` and `target_message_schema_version` to frame records. Confirmed observations only are localized. It reuses **`DualPlaneCalibration.pixel_to_floor`, `pixel_to_wall`, `wall_to_floor_base` and `GroundCalibration.pixel_to_world`**, and Module 01's `to_dict`/`from_dict` validation; there is no second physical homography implementation or calibration refit.

At task start the operator's v2 calibration had an accepted approximately 6.312 px shared-edge warning. During implementation an external process/operator saved a newer calibration and baseline; the latest read calibration has an accepted **approximately 4.331 px** warning and dimensions 121.8×66.5 cm floor / 121.8×31.5 cm wall. Runtime always reads the file's dimensions, never these documentation examples. That is a pixel consistency residual, not a measured centimeter error. It remains usable for approximate geometry, and its warning/acceptance metadata is retained in results. Neither the saved calibration nor the legal reference is overwritten by enabling Module 03.

### Coordinate frames and motion direction

The detector supplies a reliable **reference → current** image transform `A`. Its background may be warped forward for segmentation, but target masks/contours and anchors stay in original **current-image** pixels. Module 03 warps the calibrated ownership polygons forward with `A`, selects the anchor from actual component pixels, then applies **`A⁻¹` once to the anchor** before calling the existing calibrated pixel→cm function. It never transforms an already corrected point again. `pixel_anchor` and `calibration_pixel_anchor` expose both points; `camera_registration.current_to_calibration` exposes the direction explicitly.

Reliable motion within the detector's established limits (including default maximum corner displacement **2 px**) is compensated automatically, with no operator prompt. Similarity registration comes from the existing ORB/RANSAC guard; the registration API also validates finite, invertible, orientation-preserving bounded projective transforms. A single image registration across two physical planes can still have parallax errors: this is not a replacement for physical validation.

The calibration view must actually correspond to the camera pose at which Module 01 corners were selected. `--localization-reference IMAGE` is an **operator declaration of that view**, not an accuracy attestation. Keep this image fixed even if R→Y later replaces the legal background. Module 03 freezes it in memory, reuses Module 02 registration when it is the same baseline, and independently checks the frozen view after baseline changes/overrides. R never silently rebinds the original homographies to the new camera pose.

`--localization-reference` is optional. Without a declared association, the initial baseline is a nominal calibration view. Unknown association, missing reference or uncertain/over-limit registration uses current-frame pixel coordinates with loaded homographies, retaining FLOOR/WALL ownership from the exact mask and loaded calibration polygons. Structured `camera_registration` retains the original status/reason/matrix and adds `fallback_active`, `mapping_mode` and `registration_warning`. `overlap_coordinate_frame=current_frame_loaded_calibration_polygons` identifies nominal ownership. All fallback positions, contours, bounds, sizes and areas are UNVERIFIED estimates even if a measured report exists. Reliable bounded alignment still uses the forward-warped polygons and inverse mapping exactly once. Neither path mutates calibration. Missing/invalid calibration, incompatible image dimensions, invalid matrices or impossible transformations remain INVALID; actual cross-plane ambiguity remains BOUNDARY/UNKNOWN. Camera source-index differences are warnings, not proof of mathematically invalid calibration.

For the current top shared boundary, FLOOR coordinates are `x,y` centimeters, with the Module 01 top-left origin on the shared edge, +X along the floor edge, +Y away from the wall and Y=0 at the edge. WALL coordinates are `x,h`, with H=0 at the shared base and +H upward. Wall X reversal is preserved by the existing wall homography. `coordinate_frame` includes origins, axes, actual dimensions, shared edge, X reversal and the wall base's endpoints in native floor coordinates. Legacy/non-top calibrations retain their native Module 01 X/Y convention with a warning; they are never silently rebased into a conflicting frame. Wall H is a target height, never a floor driving Y coordinate.

### Exact-mask ownership and anchor policy

Thresholds are in `config/localization_config.json`:

| Parameter | Default | Policy |
| --- | --- | --- |
| `ownership_fraction` | 0.85 | At least 85% component overlap for clear plane ownership |
| `significant_other_fraction` | 0.10 | ≥10% exclusive overlap with each plane means BOUNDARY |
| `max_outside_fraction` | 0.15 | >15% outside both polygons means UNKNOWN |
| `boundary_margin_pixels` / `boundary_fraction` | 2 px / 0.10 | ≥10% of pixels in the shared-edge uncertainty band means BOUNDARY |
| `world_boundary_tolerance_cm` | 0.5 cm | Maximum minor world-boundary extrapolation; not a physical accuracy claim |

Counts/fractions use exact `component_labels == component_id`, including holes. Both ROI counts include their shared pixels and need not sum to one. The opposite-plane test uses exclusive pixels. Clear ownership also requires insignificant exclusive overlap with the other plane. `surface_confidence` is the winning overlap fraction, a **heuristic, not a calibrated probability**. No bounding-box-center ownership inference occurs.

For a clear plane, the anchor is the actual member pixel nearest the centroid of component pixels inside that plane. For an irregular shape or hole this guarantees an anchor on the relevant mask. Boundary/unknown objects retain a visual representative but have null cm coordinates. This is a representative visual target point, **not a robot approach pose or cleaning trajectory**. Localizer batching crops label arrays by the known component bounds and caches transformed polygons; it does not generate full-size masks separately for every target.

### Physical quality and verification

Visual usability, registration status and physical localization status are separate fields:

- `UNVERIFIED`: default, including mathematically computable approximate coordinates and geometrically MATCHED cameras without measured evidence. Actual ambiguous ownership can have null coordinates; uncertain registration alone does not suppress estimates.
- `INVALID`: missing/inconsistent calibration, wrong dimensions, singular/non-finite matrices, invalid masks/anchors or excessive extrapolation. Includes a reason, no physical coordinates, and can still describe a usable visual target.
- `VERIFIED`: only an explicitly attested independent measurement report tied to **this calibration hash and this calibration-view pixel hash**, recomputed and checked on load. Requires ≥3 non-collinear samples per available plane, physical convex-hull coverage ≥5%, and every Euclidean error ≤the stated tolerance (default 2 cm). A target must also lie **inside that plane's measured hull**, with MATCHED registration, no significant compensation (>0.1 px numerical tolerance), and no framing override. This status is scoped to that evidence/tolerance; it is not universal accuracy or motion authorization.

Any compensated motion, override or current-frame fallback keeps estimates UNVERIFIED, even when a reference-view report passed. A fabricated `passed=true` flag without valid measurement data cannot confer verification. Accepted pixel warnings remain visible independently of measured cm errors. Frame-level Module 02 flags remain conservative; per-target Module 03 status contains the scoped physical evidence. All version-1 target messages have **`actionable=false` and `motion_authorized=false`**, including VERIFIED records. No navigation, reachability or actuator logic is implemented.

### Actual live and offline commands

First verify the iPhone index and that your reference image really shows the current saved calibration pose. If so, keep a distinct frozen calibration-view copy before later legal-background re-registration:

```bash
cd /Users/williamwu/natHack
calibration_view="data/calibration_view_$(date +%Y%m%d_%H%M%S).png"
cp data/iphone_reference.png "$calibration_view"

# Live detection + localization + append-only target event export; normal GUI has no mask windows.
.venv/bin/python main.py detect-spills --camera 0 --width 1280 --height 720 --fps 30 \
  --background data/iphone_reference.png --confirm-clean --calibration config/camera_config.json \
  --localize --localization-reference "$calibration_view" \
  --output data/module03_live.jsonl --target-events data/module03_targets.jsonl

# Existing generated 600x440 synthetic image and video fixtures, with their own calibration.
.venv/bin/python main.py detect-spills --image data/module03_demo/targets.png \
  --background data/module03_demo/baseline.png --confirm-clean \
  --calibration data/module03_demo/calibration.json --localize \
  --localization-reference data/module03_demo/baseline.png --confirmation-frames 1 --headless \
  --output data/module03_demo/image.jsonl
.venv/bin/python main.py detect-spills --video data/module03_demo/targets.avi \
  --background data/module03_demo/baseline.png --confirm-clean \
  --calibration data/module03_demo/calibration.json --localize \
  --localization-reference data/module03_demo/baseline.png --headless \
  --output data/module03_demo/video.jsonl
```

To generate a **fresh** fixture set, use `.venv/bin/python -m tools.generate_localization_demo --output-dir data/module03_demo_new` (choose a new directory; existing fixture baselines are protected). Add `--debug --show-masks --debug-dir PATH --max-frames 60` to a camera command only when detailed exports are wanted. `--localization-settings PATH` selects custom ownership settings. Module 03 is off unless `--localize` is present. If `--target-events` is omitted, it appends to `OUTPUT.targets.jsonl` when `--output` is provided, otherwise `data/localization_targets.jsonl`. Input paths cannot be reused as outputs.

### Structured results and future Module 07 payload

The generated `data/module03_demo/image.jsonl` contains real synthetic-run examples: FLOOR approximately `{x:32.143,y:44.0}`, WALL approximately `{x:73.929,h:33.0}`, and BOUNDARY with null coordinates. All are UNVERIFIED. This validates code geometry, not the actual cardboard's accuracy.

Each `LocalizationResult` includes target/category/surface, current and calibration pixel anchors, cm coordinates, units/frame metadata, overlap counts/ratios, representative confidence, UTC ISO timestamp and first observation time, calibration ID/version/quality metadata, registration matrix/quality, physical status/reason/warnings, visual usability and any measured-report identity. `to_dict()` preserves strict NumPy-safe/non-finite-rejecting JSON behavior. `localize_detections()` consumes exact component labels directly; it never reconstructs a filled bounding box.

`LocalizationResult.target_message()` produces **schema_version 1 / message_type TARGET_DETECTION**; the formal schema is `config/target_message_v1.schema.json`. An abbreviated WALL example (full generated messages additionally include frame origins/axes/dimensions, registration and freshness metadata):

```json
{
  "schema_version": 1,
  "message_type": "TARGET_DETECTION",
  "record_key": "run:0:3",
  "target_id": 3,
  "category": "NON_RED_SPILL",
  "surface": "WALL",
  "position_cm": {"x": 73.929, "h": 33.0},
  "units": "cm",
  "localization_status": "UNVERIFIED",
  "detection_state": "LOCALIZED_ESTIMATE",
  "active": true,
  "actionable": false,
  "motion_authorized": false
}
```

FLOOR messages contain only x/y position keys; WALL only x/h; BOUNDARY/UNKNOWN have no single physical coordinate. `detection_state` distinguishes DETECTED, LOCALIZED_ESTIMATE, LOCALIZED_VERIFIED and INACTIVE. There are no image/mask payloads or network calls. `valid_until`/`valid_for_seconds` give observations a two-second freshness window; a future consumer must reject expired, inactive or unverified movement data. No physical coordinate is an executable goal in schema v1.

`TargetEventStore` appends UPSERT and INACTIVE snapshots without overwriting unrelated targets. Keys are `session_id:tracking_epoch:target_id`: resets/re-registration/restarts cannot alias an old ID to an unrelated object. Updates preserve first/last seen times. Disappearance, confirmation loss, explicit tracking reset and graceful exit mark observations inactive; framing warnings alone do not reset tracks. Inactive messages set `position_cm=null` while retaining `last_known_position_cm` and its observation time. A producer restart retires old active observations; expiration also prevents a crashed producer's old location from becoming current.

Read-only consumption for integration/Module 07 does not require image recognition and must not instantiate another producer on the same stream:

```python
from vision import TargetEventStore
last_known = TargetEventStore.read("data/module03_targets.jsonl")
fresh_active_messages = TargetEventStore.read_current_messages("data/module03_targets.jsonl")
# last_known is historical state; fresh_active_messages filters active AND non-expired data.
# Stream consumers also read INACTIVE events to retire previously displayed targets.
```

The stream is intended for one producer per file, not concurrent writers. No database, Wi-Fi transport, motor command, ArUco or Module 04 is added. Saved Module 02 contour-only JSONL cannot reconstruct holes reliably; direct image/video processing or synchronized `components.npz` exact labels are needed for accurate replay. Plain old JSONL without exact masks is not silently localized from boxes.

### Independent centimeter field validation

1. Fix the camera at the calibrated pose and preserve its calibration-view image. Check that chosen floor/wall measurement positions lie inside the actual saved dimensions.
2. Use a compact, symmetric test marker so its measured physical center consistently corresponds to the mask representative. Measure x/y for floor, x/h for wall independently with a ruler. Example starting triangles (check against the actual dimensions loaded from your current calibration): floor `(20,10),(80,10),(50,30)`; wall `(20,5),(80,5),(50,20)` cm. Add more widely spread points for broader validated hulls. The original example diagonal triples alone are collinear and cannot establish area coverage.
3. At each position, record `calibration_pixel_anchor` from a confirmed Module 03 result and the independently measured expected coordinates. Do not derive the expected cm values from the homography itself. Create `data/module03_measurements.json`, a JSON array of `{ "surface":"FLOOR", "pixel":[u,v], "expected_cm":[x,y] }` or WALL with `[x,h]`, replacing u/v/x/y/h with actual numbers.
4. Export independent per-axis absolute and Euclidean errors, maximum/RMS errors and coverage without rewriting calibration:

   ```bash
   .venv/bin/python main.py validate-localization --calibration config/camera_config.json \
     --reference "$calibration_view" --points data/module03_measurements.json \
     --tolerance-cm 2 --confirm-measured --output data/module03_physical_report.json
   ```

5. Review the report's `physical_validation_passed`, individual errors and coverage. Report files are never silently overwritten. Only after actual independent measurement/attestation, add `--localization-validation data/module03_physical_report.json` to the live command. Reports are recomputed against the current calibration on load, bound to the declared image hash and scoped to their measured hull. Motion/override or a changed calibration/reference prevents automatic VERIFIED labeling.
6. Test known floor/wall positions, near-edge/boundary cases, multiple objects, stable IDs, removal/inactive events, a recoverable small shift and a large shift/C/R decision. Record actual cm errors and observed live FPS. Synthetic software checks do not constitute this physical acceptance.

### Module 03 validation and limits

The full suite covers perspective floor/wall coordinates, wall height/X reversal, shared-edge ambiguity, irregular masks/holes, missing/degenerate calibration, invalid anchors/dimensions, accepted quality warnings, NumPy JSON, small-shift inverse direction/no double application, stable coordinates, live no-prompt recovery, significant motion/override/re-registration trust handling (later updated to UNVERIFIED fallback estimates), independent measurement evidence/hulls, multiple persistent IDs/epochs, updates/inactivity/timestamp preservation, freshness, and Module 01/02 compatibility. The complete run passed **397 tests**, with one expected legacy floor-only warning; Python compilation checks passed. No physical camera was accessed by these tests. Saved calibration/reference files were not written by this implementation; external updates during the task were preserved and the latest files were also checked read-only.

On the development host, 100 warmed synthetic frames with three targets measured approximately **0.235 ms median for localization only** (600×440, no image recognition, capture, rendering, serialization or file writes). This is not live iPhone FPS. Full detection, feature registration and optional debug exports remain separate costs.

Limits: registration over two physical planes may retain parallax; actual lens distortion, changed geometry, occlusion, imprecise clicks/dimensions and marker-anchor measurement can cause cm error. Neither the earlier 6.312 px nor latest 4.331 px accepted consistency residual is independently measured cm accuracy. Unknown surface can legitimately yield null coordinates; unknown pose alone uses diagnostic UNVERIFIED estimates. Legacy non-top boundaries retain native axes rather than silently changing Module 01 conventions. Independent reports verify only their measured reference/hull/tolerance; no general clinical/navigation accuracy is claimed. Code interfaces are ready for separate Module 04/07 integration review; measured physical acceptance remains pending, and no later module is implemented here.

### Module 03 physical object boundaries

`--localize` now records the complete foreground footprint as well as its
representative point. `--localization-boundaries` optionally adds original outlines
(yellow outer rings, magenta holes), dimensions and estimated cm² to the preview.
Floor geometry uses X/Y and width/depth; wall geometry uses X/H and width/height.
The existing `--localization-boundaries` flag is the geometry display option;
no separate `--show-geometry` flag is required. It labels confirmed targets with
ID, category, surface, center, W and D/H, estimated area (`cm^2` in the OpenCV
font), localization trust and geometry trust. BOUNDARY, UNKNOWN and invalid
localization show the unavailable-geometry reason instead of dimensions.
Text wraps to the frame width and uses separated rows with dark backing. If the
screen cannot fit every target, a notice reports how many labels could not fit;
all target contours are still drawn, and complete records remain in JSONL.
Without this option, the default overlay is unchanged.
No navigation, cleaning paths or hardware control is included.

The exact Module 02 component-label mask is reused without changing segmentation.
A pixel at (u,v) is represented by its square cell [u−0.5,u+0.5] ×
[v−0.5,v+0.5]. Exposed cell edges form dense, implicitly closed rings, including
holes, disconnected islands and diagonal contacts. This convention gives even a
single pixel or one-pixel-wide strip a nonzero footprint. It differs from an
OpenCV contour through boundary pixel centers, which can collapse thin objects.
`image_geometry.contours` preserves these exact image-space cell boundaries;
`mask_runs_y_xstart_xend_exclusive` losslessly reconstructs the original mask.
A display bounding box never substitutes for this mask.

Every ring vertex maps through **current → calibration** registration exactly once,
then through the existing Module 01 floor/wall pixel mapping API. Horizon
crossings, non-finite output, collapsed footprints and extents outside the
configured physical surface tolerance are rejected. Physical bounds are the
min/max of all transformed vertices, before simplification. Polygon shoelace area
adds outer rings and subtracts holes. Because projective transforms preserve
straight lines away from a horizon, this is the area of the transformed union of
pixel cells; no global pixel-to-cm scale is used. It is an estimate of the visible
surface footprint, not a measurement of hidden portions, true 3D object area or
sub-centimeter physical accuracy.

Full float64 physical rings are stored in `full_physical_contours_cm`. Local JSONL
event records also retain the full image geometry and lossless mask runs; these
can be large for complicated components. The robot-facing v1 message receives
only compact `geometry_cm`, with bounds, dimensions, area, `contours` (outer/hole
rings), trust and payload status. `contour` is a convenience alias only when there
is exactly one outer ring and no holes; multi-ring consumers must use `contours`.
Areas/extents always come from the complete geometry, not the compact outline.

The configurable `contour_tolerance_cm` defaults to 0.2 cm. Simplification uses
closed Douglas–Peucker in physical units, conservatively capped at a quarter of
the smallest projected pixel edge to protect thin structures. It retains original
float64 vertices and falls back to the full ring if orientation changes, fewer
than three vertices survive, area changes by more than 1%, or compact rings develop
proper crossings. It often only removes collinear or near-collinear vertices;
it never increases tolerance to meet a transmission budget. Default limits are
512 total compact vertices and 32768 bytes for the **geometry section**, configurable
as `contour_max_points` and `contour_max_bytes`. Over-budget contours are omitted
explicitly (`payload_status: CONTOURS_OMITTED_LIMIT`, null contour fields); numerical
summaries remain available and complete rings/masks remain in local records.
There is no byte cap on full internal/persisted geometry or on the entire message.

The new v1 fields are additive and optional in the schema, preserving existing
centroid consumers: `geometry_cm`, `last_known_geometry_cm`, `geometry_status`,
`geometry_reason`. Consumers of the extension must inspect `geometry_status` and
`geometry_cm.trust_state` separately from the existing centroid localization
status: a verified center does not verify a boundary outside the measured hull.
The entire boundary must lie inside the independently measured coverage hull to
be marked VERIFIED. Otherwise reliable mapping remains available as UNVERIFIED
estimates. Small-motion compensation and framing overrides cannot verify physical
accuracy. Unresolved registration uses diagnostic current-frame geometry marked
UNVERIFIED; BOUNDARY and UNKNOWN retain null physical geometry with reasons and image geometry. Geometry inherits the
record's calibration ID, frame, registration and quality warnings.

UPSERT events update geometry under the existing stable session/epoch/target key.
INACTIVE events clear current `geometry_cm`, keep `last_known_geometry_cm` and
retain historical full localization geometry and observation timestamps. Existing
freshness checks continue to exclude disappeared and stale targets. Version 1
messages remain non-actionable and never authorize motion.

Operator-managed live geometry preview (no calibration-view declaration required):

```bash
cd /Users/williamwu/natHack
.venv/bin/python main.py detect-spills --camera 0 --width 1280 --height 720 --fps 30 \
  --background data/iphone_reference.png --confirm-clean \
  --calibration config/camera_config.json --localize --localization-boundaries \
  --output data/module03_geometry_live.jsonl --target-events data/module03_geometry_targets.jsonl
```

A reproducible synthetic preview without a camera:

```bash
.venv/bin/python main.py detect-spills --image data/module03_geometry_demo/targets.png \
  --background data/module03_geometry_demo/baseline.png --confirm-clean \
  --calibration data/module03_geometry_demo/calibration.json --localize \
  --localization-reference data/module03_geometry_demo/baseline.png \
  --localization-boundaries --confirmation-frames 1 --headless \
  --debug-dir data/module03_geometry_demo/debug --output data/module03_geometry_demo/image.jsonl
```

See `data/module03_geometry_demo/target_message_v1.json` for a complete generated
v1 payload and `debug/frame_000000_annotated.png` for its preview. Synthetic tests
verify raster geometry and API behavior; real iPhone calibration accuracy,
segmentation boundaries, motion/registration quality and live throughput still
require field measurements. Physical verification follows the independently
measured-point workflow above; it cannot establish the accuracy of segmentation.

## Module 04 — one ArUco robot

Live shared-camera robot and spill display (read only):

```bash
.venv/bin/python main.py test-vision --camera 0
```

One camera supplies each frame to Modules 02/03 and 04. The window shows the ArUco
marker/center, robot heading and ground coordinates, plus RED/NON_RED spill boxes
and coordinates after the existing confirmation streak. Pending regions are also
shown. Terminal coordinates print about once per second. Wall positions show their
floor-base X/Y projection and a separate height. Detection/localization errors stay
visible while the window remains open; Q exits. Existing calibration, marker-plane,
robot settings and registered background are read without modification. No ESP32
connection, navigation or cleaning is involved; no motion authorization is requested.

Module 04 adds `generate-aruco`, `track-robot` and `calibrate-robot-plane`.
It uses the existing CameraManager and installed OpenCV ArUco implementation;
no new dependencies or motor/navigation/network modules are involved.
Settings are in `config/robot_config.json`. Defaults are DICT_4X4_50, marker ID 0,
robot ID 0 and an intended **6 cm black square**, with mounting height unknown
(`null`). The zero mounting offsets and zero heading rotation are explicit demo
assumptions, not measured hardware facts. Edit them to match the real installation.
Other dictionaries/IDs work through the same settings.

### Print and measure the marker

Ready file: `data/module04_print/aruco_id0_6cm.png`, with print metadata in its
`.png.json` sidecar. The 1600×1600 PNG contains a 1200×1200 black marker square
and a 200 px white quiet zone on each side. To reproduce, choose a fresh output:

```bash
.venv/bin/python main.py generate-aruco --dictionary DICT_4X4_50 --marker-id 0 \
  --marker-size-cm 6 --side-pixels 1200 --quiet-zone-pixels 200 \
  --output data/aruco_id0_new.png
```

Print the **entire image 8 cm × 8 cm** without cropping/stretching or fit-to-page.
The black square should then be **6 cm × 6 cm**, with 1 cm white margins on each
side. PNG pixel count or printer DPI does not prove physical size. Measure both
black-square axes with a ruler; adjust the print scale until each is 60 mm.
Keep the surrounding white quiet zone and mount the marker flat, upward-facing.
For another intended black-square size S, total image width is
`S × total_pixels / black_square_pixels`; measure the black square again.
The generator refuses existing outputs and requires at least one module of white
quiet zone. The pre-existing repository-root `aruco_id0.png` is untouched.

### API, corners and heading

`RobotMarkerDetector.detect(frame)` decodes only the configured ID. Exactly one
matching marker is required; unrelated IDs are ignored, duplicate configured IDs
are ambiguous and emit no observation. `MarkerObservation` retains canonical
TL/TR/BR/BL corners, the projective diagonal-intersection center, an orientation
axis, UTC observation timestamp and image-quality measurements. Corner order rotates with the marker code;
it is not re-sorted by image position. This uses OpenCV's canonical ordered-corner
API ([official ArUco documentation](https://docs.opencv.org/4.x/d5/dae/tutorial_aruco_detection.html)).

`RobotTracker.update(frame, timestamp=None, registration=None)` returns a typed
`RobotPose`; `to_dict()` emits strict JSON. Timestamps are UTC; supplied numeric
timestamps must be finite and nondecreasing. Heading is in native Module 01 floor
X/Y: 0° points +X, positive angles go toward +Y, normalized to [0,360). In the
current top-boundary setup, +X runs along the wall and +Y away from it. Native axes
are retained with a warning for other boundary configurations. The marker's
canonical left-to-right **+u** axis is mapped through the plane before atan2;
raw image rotation is never substituted for physical heading.

`marker_to_robot_heading_deg` adds the angle from marker +u to robot forward.
Mounting offsets describe **robot center → marker center** in robot-local forward
and left directions. With robot heading θ, the ground-center estimate is
`marker_projection − (forward_offset*cosθ − left_offset*sinθ,
forward_offset*sinθ + left_offset*cosθ)`. Height is measured from floor to the
marker plane. Optional position/heading smoothing uses `smoothing_alpha`; angular
updates use the shortest wrapped difference, so 359°→1° does not swing through 180°.
The preview arrow is explicitly labeled **decoded marker +u in the image**;
numeric theta is the plane-aware robot heading, not the arrow's image angle.

### Elevated marker strategy and quality

There are no camera intrinsics/extrinsics in Module 01. Module 04 never fabricates
them or silently sets mounting height to zero.

1. Default practical fallback: apply the existing floor API to marker pixels and
   mapped orientation vectors, then account for configured mounting offsets.
   This is labeled `APPROXIMATE_FLOOR_RAY_INTERSECTION`: an elevated marker's ray
   intersects the floor at a displaced point. It is **not the true ground-center**
   and is always UNVERIFIED, even when a height value is supplied. Known height
   alone cannot correct this parallax without more calibration.
2. Optional empirical fixed-height plane: independently measure the floor
   projection of marker centers at a fixed mounting height. Fit a separate
   homography from marker-plane pixels to floor X/Y; validate with independent
   points. The original Module 01 calibration file remains untouched.

For the empirical workflow, record `data/robot_plane_measurements.json`:

```json
{
  "training": [
    {"pixel": [210, 310], "ground_cm": [20, 20]},
    {"pixel": [510, 310], "ground_cm": [80, 20]},
    {"pixel": [500, 500], "ground_cm": [80, 50]},
    {"pixel": [220, 500], "ground_cm": [20, 50]}
  ],
  "validation": [
    {"pixel": [310, 370], "ground_cm": [40, 30]},
    {"pixel": [410, 370], "ground_cm": [60, 30]},
    {"pixel": [350, 440], "ground_cm": [50, 40]}
  ]
}
```

**Replace every illustrative number with actual independent measurements.** Pixel
centers are available in robot JSONL; ground_cm must be the vertically projected
marker center measured on the floor (use a plumb reference), not coordinates
computed by the homography being tested. If marker offsets are nonzero, do not
substitute the robot center for that projection. Keep the camera and marker height
fixed; use broad coverage and additional hold-out points. At least four training
points and three non-collinear independent validation points are required. Each
set must cover at least 1% of floor area. Every hold-out Euclidean error must be
within the configured tolerance; collinear, reused, nonfinite and out-of-bounds
points are rejected. The result binds calibration SHA-256, image size and height;
raw measurements are revalidated on load.

```bash
# Set this to the ruler-measured marker mounting height, in cm.
marker_height_cm=YOUR_MEASURED_HEIGHT
.venv/bin/python main.py calibrate-robot-plane \
  --calibration config/camera_config.json --points data/robot_plane_measurements.json \
  --mounting-height-cm "$marker_height_cm" --tolerance-cm 2 --confirm-measured \
  --output data/robot_marker_plane.json
# Add these to track-robot when using that map:
# --marker-plane data/robot_marker_plane.json --mounting-height-cm "$marker_height_cm"
```

Even a passing empirical map remains UNVERIFIED for actual robot operation until
printed-marker/robot field validation. Errors can increase outside measured
coverage. Small reliable camera transforms map current → calibration once;
unsupported/absent alignment falls back without blocking, with warning metadata.
A background plane registration is not guaranteed exact at elevated marker height.
`--alignment-reference IMAGE` optionally supplies a frozen calibration-pose view;
use static background features, not a robot/marker that can move. Camera movements,
changed height/tilt, print size, lens distortion and mounting error can invalidate
physical accuracy. No record is automatically suitable for navigation or actionable.

### Live and offline commands

Live iPhone tracking needs no legal baseline or explicit alignment reference:

```bash
cd /Users/williamwu/natHack
robot_run="data/robot_pose_$(date +%Y%m%d_%H%M%S).jsonl"
.venv/bin/python main.py track-robot --camera 0 --width 1280 --height 720 --fps 30 \
  --calibration config/camera_config.json --robot-settings config/robot_config.json \
  --output "$robot_run"
```

Optional `--marker-size-cm` and `--mounting-height-cm` override settings with measured
values; they do not themselves perform height compensation. One readable window
shows ID/corners/center, X/Y/theta estimates, tracking/trust, mapping limitation,
marker-axis arrow and measured processing FPS. Q/Escape/window close exits.
`--headless` avoids all GUI calls. `--annotated-dir NEW_DIRECTORY` saves previews.
New output files/directories are required to protect existing data.

Synthetic examples already generated in `data/module04_demo/`:

```bash
.venv/bin/python main.py track-robot --image data/module04_demo/robot.png \
  --calibration data/module04_demo/calibration.json --headless \
  --output data/module04_image_check.jsonl
.venv/bin/python main.py track-robot --video data/module04_demo/robot.avi \
  --calibration data/module04_demo/calibration.json --headless \
  --output data/module04_video_check.jsonl
```

Create a new fixture directory with
`.venv/bin/python -m tools.generate_robot_demo --output-dir data/module04_demo_new`.
Synthetic floor dimensions are deliberately fixture-specific; real setup dimensions
always come from its saved calibration. Root real calibration/baseline are untouched.

### Tracking, freshness, JSON and spill exclusion

Robot identity stays fixed. Current positions/headings/corners are null immediately
on marker loss. `last_known_pose` retains historical values with their observation
time; `tracking_status` is LOST, then EXPIRED by missed-frame or elapsed-age limits.
The default TTL is 0.5 s and missed-frame expiration 5. Reacquisition emits
TRACKING_GAINED and resets smoothing; first loss emits TRACKING_LOST.
`robot_pose_is_current(message, timestamp=None)` checks raw JSONL freshness too:
a historical VISIBLE line alone is not evidence of a current observation.

`config/robot_pose_v1.schema.json` describes the separate v1 ROBOT_POSE contract.
The message includes coordinate frame, calibration identity, complete marker and
mounting settings, registration, measurement method, quality warnings, freshness,
missed frames and optional gain/loss event. `actionable`, `motion_authorized` and
`suitable_for_navigation` are always false. Module 03 target-message/event schemas
are unchanged. `data/module04_demo/image_final.jsonl` is a complete synthetic pose
example; it is a historical observation, not a live pose.

`RobotTracker.exclusion_mask(pose, frame_size, timestamp=None)` is a clean integration
API for the existing `SpillDetector.detect(frame, exclusion_mask=mask)` mechanism.
It expands the current marker's image box by configurable `exclusion_padding_pixels`
(default 80 px) to approximate the **body**, not just the marker. Optionally supply
both physical body_length_cm/body_width_cm to union the projected ground footprint.
The image expansion remains important for elevated body surfaces. These are
conservative approximations: measure the actual robot silhouette in the oblique
view and tune the margin. Too large can hide nearby spills; too small can leave
robot parts classified as foreground. There is no full robot-body segmentation.
The mask is zero immediately on marker loss or stale observations, never retained
indefinitely. After loss, visible robot parts can reappear as foreground; this is
an explicit limitation. With synthetic timestamps, pass the same clock explicitly
to exclusion_mask; default freshness uses the real current clock.

Enable optional integration without changing default spill behavior:

```bash
.venv/bin/python main.py detect-spills --camera 0 --width 1280 --height 720 --fps 30 \
  --background data/iphone_reference.png --confirm-clean \
  --calibration config/camera_config.json --localize --localization-boundaries \
  --track-robot --robot-settings config/robot_config.json \
  --output data/spills_with_robot.jsonl
```

This adds separate `robot_pose`/`robot_exclusion` frame fields, applies a fresh body
mask before spill extraction, and preserves the existing target events. The robot
reference camera view is frozen from the initial baseline and is not silently
rebound by later R/Y registration. Baseline registration remains explicit. Other
floor regions continue to detect spills; no circular tracking/segmentation dependency
is introduced. `--robot-marker-plane PATH` selects an empirical map; set its measured
height in robot_config.json to match. No new window opens by default.

Module 04 verification: **511 tests passed**, one existing legacy-floor warning.
Synthetic image/video CLI runs and preview inspection passed. A warmed 100-frame
800×600 marker-detector/pose-only benchmark measured ~0.635 ms median; this excludes
capture, rendering, JSON and disk I/O and is not physical iPhone throughput. See
`MODULE04_REPORT.md` for the full implementation report and physical validation checklist.

## Module 05 — declarative cleaning contact planning

Module 05 is an **offline planner, never a robot controller**. It reads existing
Module 03 target localization/messages/events and optional Module 04 robot poses,
then writes v1 CLEANING_PLAN JSONL and optional PNG diagrams. No movement, pump,
servo, rotating-sponge or network commands are issued. No new dependencies.

Two deliberately separate configurations:

| File | Meaning |
| --- | --- |
| `config/cleaning_hardware.json` | PHYSICAL template; real measurements remain null, installed states unknown, measurements_validated=false. Missing necessary geometry returns UNKNOWN. |
| `config/cleaning_demo.json` | SIMULATION only; illustrative dimensions and coverage models, never real hardware measurements. |
| `config/cleaning_hardware.schema.json` | Configuration field contract; `CleaningHardware` also checks interval order, mounting side, angle limits, finite values and context. |
| `config/cleaning_plan_v1.schema.json` | Separate non-actionable version-1 plan contract. Existing target/pose schemas are unchanged. |

`--mode physical` is the default and never borrows demo dimensions. Simulation
requires `--mode simulation`. A configuration whose geometry_source disagrees
with selected mode is rejected. Physical model parameters require explicit measured
validation; a complete physical geometry plan still cannot authorize execution.
The planner preferences (pass overlap, lead-in, retreat, stroke budget) are software
proposal parameters, not claims about measured hardware performance.

### Geometry and coverage

`CleaningPlanner(calibration, hardware, mode=...).plan(target, robot_pose=None)`
returns `CleaningPlan`. `ApproachPose` and `CoverageSegment` are reusable structures.
All positions and footprints use centimeters and the Module 04 ground-center,
forward/left and native Module 01 floor heading convention: 0=+X, 90=+Y.
Arena dimensions are loaded from calibration, never hardcoded demonstration sizes.

Target ingestion prefers Module 03 full_physical_contours_cm, otherwise compact
physical rings. Holes and disconnected outer rings survive. Missing contours are
UNKNOWN; a bounding box never substitutes for shape. Source simplification/trust,
calibration/frame identity, camera warnings and stable record keys are preserved.
Invalid/nonfinite/self-crossing contours, incompatible calibration or out-of-arena
geometry do not produce a coverage path. Module 03 camera-registration fallback
warnings alone do not block theoretical planning with available UNVERIFIED geometry.

Floor planning considers 0/90/180/270-degree forward strokes. It transforms the
arena/target into each robot-oriented frame, inflates the chassis by configured
safety margin plus position uncertainty, and accounts for the fixed front sponge's
forward/lateral contact offset. Overlapping rows use the contact width; actual
outer contours intersected with each row determine strokes. Narrow or wide targets
can need one or multiple passes. Body and fixed sponge stay within the arena;
the sponge remains present during wetting passes too. The nozzle is evaluated
independently with its measured effective impact rectangle and offset, not assumed
to spray wherever the sponge touches. Known rectangular footprint orientations
at multiples of 90 degrees are supported. Other effective orientations or custom
tool/body polygon definitions are explicitly unsupported, not silently converted
to a bounding box; use a measured guaranteed conservative aligned rectangle.
Mechanical fixed spray azimuth/elevation and mounting position are distinct metadata
from effective footprint orientation/center. No nozzle is treated as independently
rotatable or elevating in floor mode.

Coverage is **continuous polygon clipping** against unions of rectangular swept
tool regions. Rectangle sets are partitioned into disjoint vertical slabs, so
areas are not double-counted. Outer polygon area is added, holes subtracted;
contact, spray and their intersection are reported separately. This is not a grid
sample or pixel-to-cm scale. Numerical area tolerance 1e-8 cm² is an arithmetic
comparison tolerance, not physical accuracy. FULLY_REACHABLE means full coverage
of the *provided estimated polygons under the configured effective rectangle model*,
not proven physical cleaning. Candidate stroke budgets do not truncate paths and
then claim full coverage. Tools may pass over clean holes between target portions;
hole area is excluded from all target coverage metrics. Obstacle avoidance is not
provided.

Wall planning clips full X/H geometry against the fixed side sponge's independently
measured minimum/maximum contact heights and along-wall contact width. The band is
never derived from diameter. Low and high blind zones are both reported. Independently
calibrated nozzle impact-height coverage, horizontal width, safe distance, lift
limits and rotation ranges determine spray reach. Lift height is **not** impact
height. A spray-only region is not contact-cleanable. Measured effective rectangles
must be conservative *guaranteed* coverage, not unproven bounding boxes of a circular
sponge, irregular spray pattern or coupled actuator envelope.

For the current top-wall frame, RIGHT side sponge uses heading 0; LEFT uses 180.
Chassis wall gap is protrusion minus configured contact compression, and must
preserve required wall clearance plus safety/uncertainty inflation. Robot center
Y is chassis half-width plus that gap, independent of target H. Horizontal strokes
follow the wall with fixed sponge height. Wall X conversion reuses
DualPlaneCalibration.wall_to_floor_base. Non-top shared-edge wall approach planning
currently returns UNKNOWN rather than silently redefining coordinates. Actual
rotation/contact force and full robot/tool collision envelopes still need physical
validation. Floor contact is supported in native floor coordinates.

### Configuring hardware later

Fill the PHYSICAL template with real measured chassis dimensions, clearances and
localization uncertainty; front sponge contact size/offset; fixed floor nozzle's
effective guaranteed footprint/offset/orientation; wall sponge side, protrusion,
compression, safe clearance and minimum/maximum effective contact heights; and
wall nozzle measured impact coverage, safe distances, lift and rotation limits.
Set coverage_calibrated only after actual footprint/reach measurements. The
provisional 20 cm wall nozzle height appears only in the illustrative demo, never
as a physical default and never as the side sponge limit.

Nozzle mounting height/position/direction and sponge diameter remain explicit
metadata, possibly unknown. They are not used to infer coverage when a measured
effective footprint is the authoritative input. No specific wall nozzle lift/angle
solution is fabricated: plans retain calibrated limits/required region and require
later actuator-setting validation. Do not mark illustrative dimensions as measured.
The algorithms read new values without code changes within their declared model.

### Plan states and workflow

States: FULLY_REACHABLE, PARTIALLY_REACHABLE, UNREACHABLE, UNKNOWN.
Missing hardware/evidence is UNKNOWN; a known absent required mechanism or zero
joint coverage can be UNREACHABLE. Geometric reachability, target/robot localization
trust and execution authorization are separate. All plans have actionable=false,
execution_authorization=NONE, physical_execution_authorization=NONE and
route_to_approach_validated=false, even with complete geometry or VERIFIED inputs.

Plans retain exact source rings, contact/spray/combined regions, uncovered target
geometry, total/contact/spray/joint areas and cleanable fraction. Wall plans also
include lower/upper blind-zone areas. The configuration snapshot and hash, calibration
identity, timestamp, source quality and freshness support later review.

Segments are declarative APPROACH, ALIGN, SPRAY, CONTACT_CLEAN, RETREAT and
REQUEST_VERIFICATION. Actual wetting/contact strokes have physical start/end poses
and waypoints. The approach starts the proposed operation; the first contact start
is separately recorded in desired_contact_geometry. APPROACH/transitions/RETREAT
have **route=null**, requiring Module 06 routing; rotation clearance, obstacle
collisions, motor speed, water flow, servo settings and duration are not assumed.
An unproven retreat endpoint is left null with a reason. REQUEST_VERIFICATION is a
future visual-inspection request, not a cleaning-success claim.

Inactive targets return UNKNOWN with no path unless --allow-inactive explicitly
requests historical analysis. Stale targets/robot poses remain clearly marked
as offline evidence; no stale last-known robot position is substituted as current.
The latest Module 04 observation is used, including LOST/EXPIRED. Optional --as-of
is a timezone-aware offline evaluation time, not an assertion that historical data
is live. Every result remains an offline, non-actionable geometric proposal.

### Reproducible offline commands

Fixtures already exist in `data/module05_demo/`, generated through real Module 03/04
APIs with synthetic masks/marker image and a read-only copy of saved calibration.
No connected camera, printed marker or completed robot is needed. Run from the repo:

```bash
cd /Users/williamwu/natHack
.venv/bin/python main.py plan-cleaning \
  --targets data/module05_demo/targets.jsonl \
  --robot-poses data/module05_demo/robot_pose.jsonl \
  --calibration data/module05_demo/calibration.json \
  --mode simulation --hardware config/cleaning_demo.json \
  --output data/module05_plans_new.jsonl --visualize-dir data/module05_preview_new

.venv/bin/python main.py plan-cleaning \
  --targets data/module05_demo/targets.jsonl \
  --calibration data/module05_demo/calibration.json \
  --mode physical --hardware config/cleaning_hardware.json \
  --output data/module05_physical_unknown_new.jsonl
```

The physical command should produce UNKNOWN until required real measurements are
configured. Outputs/directories must be NEW; input and existing experimental files
are protected. --target-id selects an ID; --allow-inactive enables explicit historical
analysis; --as-of accepts an ISO timestamp with timezone. JSON, JSONL, raw localized
frame records, v1 target messages and event snapshots are accepted.

For a new fixture directory:
`.venv/bin/python -m tools.generate_cleaning_demo --output-dir data/module05_demo_new`.
Optional --calibration chooses the source calibration to copy; no root calibration
or baseline is overwritten. Generated demo masks are synthetic, not field evidence.

Planning PNGs show the arena, true target rings/holes, proposed chassis/tool footprints,
contact/wetting strokes, joint/partial/uncovered regions and wall blind/impact bands.
They do not interfere with camera windows. The generated examples are under
`data/module05_demo/preview_delivery/`; full plan sample is
`data/module05_demo/cleaning_plan_v1.json`.

Module 05 verification: **606 tests passed**, one existing legacy-floor warning.
The 95 new planning regressions include geometry, blind zones, separate spray/contact,
unknown hardware, source contracts, freshness, output protection and non-authorization.
See `MODULE05_REPORT.md` for the full report and physical measurement checklist.

### Interactive Module 04 marker-plane calibration

The operator can now enter measured **marker-center vertical floor projection**
X/Y directly in a live OpenCV wizard; raw pixels are captured automatically.

```bash
.venv/bin/python main.py calibrate-robot-interactive \
  --camera 0 --width 1280 --height 720 \
  --calibration config/camera_config.json --robot-settings config/robot_config.json \
  --mounting-height-cm 16.6 --tolerance-cm 2 \
  --points-output data/robot_plane_points.json --output config/robot_marker_plane.json
```

Use new output paths. A confirms the measurement reference; type X, Tab, Y;
Space captures a stable raw marker center. T/V select fitting/independent
validation points; C clears fields/window; U undoes; S saves a points snapshot;
F fits and reports independent errors; M explicitly saves a passing mapping;
P toggles a read-only marker/chassis/footprint test; Q preserves points and exits.
No terminal coordinate input or manual pixel copying is required. Heading remains
unverified, mounting offsets are applied after mapping, and no motion is authorized.
See [MODULE04_WIZARD_REPORT.md](MODULE04_WIZARD_REPORT.md) for the complete physical
procedure, keys, point layouts, JSON structures, stability limits, tracking command,
non-overwriting snapshot behavior, tests and remaining field checks.

To continue the saved 4-training/1-validation session, run the same wizard with
`--resume --points-output data/robot_plane_points.json` and an unused mapping
`--output` path. It validates the saved calibration identity, resolution, height,
robot configuration and capture metadata, then opens VALIDATION at point #2.
Keep the camera pose and marker mounting unchanged. The original points file is
never replaced: before the first updated save, a verified `*_resume_backup.json`
copy is created, and each resumed capture atomically saves a numbered points
snapshot such as `robot_plane_points_001.json`. For a later restart, pass the
latest snapshot explicitly as `--points-output` with `--resume`. Fitting,
independent validation and trust thresholds are unchanged; old reports do not
restore a mapping. Files from ordinary new sessions still require unused paths.

### Module 07 — ESP32 communication and local simulation

**Current robot:** the inspected Robot 3 source is `NATHACK_V2_SIMPLE`. Select
`--firmware-profile simple` explicitly; it has no heartbeat, SAFETY or latched
ESTOP. The examples below retain the older `safe_v1_1` default and its arming
rules. See [Module 06](MODULE06_REPORT.md) for the current SIMPLE integration.

Module 07 uses ASCII TCP on port 3333. `MOVE,F,1000` means **1000 milliseconds at
fixed PWM170, 8-bit/20 kHz**, not a power or velocity parameter. `send_command`
reports acceptance; `wait_for_move(result)` / CLI `--wait-move` wait for DONE while
heartbeats continue. No automatic retries, reconnect or motion recovery occur.

In two terminals:

```bash
.venv/bin/python main.py robot-simulator --port 3333
.venv/bin/python main.py robot-client --port 3333 --command PING --command STATUS --listen 1
```

For software-only movement, cleaning actuator sequence and ESTOP tests:

```bash
.venv/bin/python main.py robot-client --port 3333 --demo --listen 1
```

`--log-file NEW_PATH` saves plain-text logs. Simulation is default and restricted
to loopback. In the default `safe_v1_1` profile, physical mode requires an explicit host, remains disarmed, and cannot
run full cleaning sequences. The user reports a newer firmware with the agreed
safety patch; the local downloaded reference is older and cannot establish its
current behavior for that older profile. Its simulator also has documented differences in
startup locking, heartbeat timing and actuator fault protection. The old PWM
firmware patch is incompatible. Arming requires a validated PWM ceiling >=170 and a
separate `--validated-max-duration-ms` below 10000; reset never rearms the client.
See [the current protocol and short validation steps](MODULE07_DURATION_PROTOCOL.md).
No firmware upload or real robot connection was performed. `python -m robot_comm`
offers the same commands without importing OpenCV or vision modules.

### Module 06 — pose-feedback navigation and return

Automatic cleaning MVP (no manually supplied target coordinates):

```bash
.venv/bin/python main.py auto-clean --output-dir data/auto_clean_demo_new
```

Default mode uses actual Module 02 detection/confirmation and Module 03 localization
on synthetic images, Module 05 planning, and simulated Module 06/07 execution.
RED/HIGH is selected before NON_RED/LOW. Each task waits for ESP32 cleaning DONE,
returns to the same initial HOME and verifies it before re-scanning. Unreachable
targets are recorded/skipped; any execution fault stops the whole batch. At most
10 tasks run by default (`--max-tasks`). Completed locations are suppressed for this
session; DONE does not certify visual stain removal. Evidence is in summary.json,
progress.jsonl and the full wire.log. Existing output directories are rejected.

Floor targets use the detected spill bounding edge, with the robot facing the
spill: the 26 cm chassis center parks 13 + 5 = **18 cm beyond that edge**.
The existing bounded MOVE/STOP feedback controller selects a safe cardinal approach.
After a fresh spill scan and robot pose, the finite front bumper must be within
the existing 2.5 cm tolerance of the 5 cm goal and facing within 7 degrees before
TARGET/ARRIVED. This is approximate camera geometry, not precision distance sensing.
Physical floor tasks use this parking pose and the ESP32 GROUND preset. The PC
checks chassis/known side-tool containment across the preset's forward/backward
motion (including its existing 600 ms independent segment bound), without demanding
unmeasured sponge contact or spray coverage. Coverage and cleaning effectiveness
remain unverified; no missing dimensions are filled in. Wall tasks keep the separate
parallel contact approach and require its measured wall geometry. Existing offline
Module05 coverage planning and its checks are unchanged.

Physical execution uses the live camera and real TCP client, with a real-time clock.
`OK,MOVE` only accepts the command; each segment waits for `DONE,MOVE` or an
acknowledged early STOP while continuing fresh ArUco observations. A physical DONE
before the requested duration (allowing 1 ms firmware timer quantization) faults
and locks the session. Elapsed time, predicted poses and simulator responses never
establish physical arrival. `[ARRIVAL]` logs the fresh pose/frame, spill bounds and
measured bumper-gap estimate immediately before the cleaning handoff.

Transient marker loss keeps camera capture running and immediately invalidates
the cached pose. During MOVE it still triggers acknowledged STOP. After DONE or
confirmed STOP and the existing 0.5 s settling pause, a missing pose gets a short
stationary reacquisition window (default 1 s, `reacquisition_seconds` in navigation
settings, at most 2 s). Only a new VERIFIED frame can resume corrections; obsolete
turn calculations are discarded. Persistent loss, camera read/context errors and
uncertain STOP exit without another MOVE. `POSE_DIAGNOSTIC` logs frame-read status,
host pose age, marker visibility and ArUco decode-quality evidence. ArUco supplies
no calibrated probability; `confidence_probability` stays null. Healthy observations
clear historical missing-marker reasons, so read timeouts no longer cite startup loss.

Module02's background registration is image-only: a new legal background is not
silently declared to be the original calibration photograph. Module03 maps current
spill pixels through the loaded Module01 calibration once, in the same current-frame
convention as Module04. Background `MOVED`, `UNVERIFIED` and fit-limit warnings remain
diagnostics; they do not authorize motion or reject an otherwise valid navigation
coordinate. Target estimates remain UNVERIFIED. Invalid/singular/non-finite/reflected
or horizon-crossing transforms, incompatible frame sizes, calibration IDs, source
metadata, missing robot localization and unsafe collision geometry still stop.
The `ASSOCIATION` log separates the calibration/mapping ID from the background image
ID. Current camera pose/crop/zoom association still requires the existing operator
fixed-camera confirmation; runtime software does not pretend to physically verify it.
Calibration, background images/metadata and measured marker mapping are not rewritten.

Full physical GROUND-preset command for the current ESP32 (operator-supervised,
first task only; the confirmations attest unchanged calibrated framing and checked
parked/STOP-homing tool outline):

```bash
.venv/bin/python main.py auto-clean --mode physical --host 172.20.10.12 --port 3333 \
  --camera 0 --background data/iphone_reference.png --max-tasks 1 \
  --authorize-motion --confirm-clean --confirm-fixed-camera \
  --confirm-ground-reference --confirm-static-tools
```

**Automatic navigation-only physical test** (replace `ESP32_IP`):

```bash
.venv/bin/python main.py auto-clean --navigation-only --mode physical \
  --host ESP32_IP --confirm-navigation-test
```

No target coordinates or output path are needed. Defaults reuse camera 0 at
1280x720, `data/iphone_reference.png`, the saved verified marker-plane mapping,
robot settings and physical timing. The confirmation explicitly authorizes movement
and attests the current legal baseline/calibration association, unchanged camera,
checked chassis reference/heading, clear route and parked tools contained within
the known collision outline. **Isolate cleaning-actuator power:** SIMPLE STOP can
home firmware servos even though this mode never sends TARGET or ARRIVED.

The operator-confirmed black-square side is **9.0 cm** in `config/robot_config.json`.
Navigation defaults to `data/robot_plane_points_marker9.json`, an explicit nominal-size
metadata correction derived from the completed 4+3 session; it preserves every
measurement/capture and records the original file hash and 6→9 correction. Original
points, camera calibration and `config/robot_marker_plane.json` remain unchanged.
Empirical center/heading mapping does not use nominal marker size, so the validated
map is compatible; size still controls the existing live scale-consistency gate.
The historical 6 cm evidence intentionally fails strict context matching with the
current 9 cm settings. For explicit `track-robot` verification, use the new evidence
path. All height/frame/ID/offset, camera/ground-reference and live scale checks remain.

The highest-priority reachable confirmed spill selects a front-clearance floor
destination or parallel wall destination, with chassis/side-tool rotation clearance. Existing Module 06
preflights Y/X outbound and X/Y return and checks live verified poses throughout.
This is a navigation observation pose, not a cleaning-contact/coverage claim. No
cleaning measurements are invented. Unknown tools outside the known parked outline
must be measured/configured before confirming; missing installed side geometry blocks.
One target is tested by default, followed by verified HOME and original heading.

The saved background's historical framing warning is preserved and logged; this
explicit navigation-test confirmation attests its current calibration association,
while detection-quality warnings alone do not block navigation. Low texture,
UNVERIFIED background matching, compensation and exceeding the detector's 2 px
small-alignment threshold remain visible warnings; they do not authorize motion
or relabel Module 03 estimates VERIFIED. Invalid calibration/resolution/transforms,
paused detection and significant framing changes still stop the run. Existing
limits are retained: 8 px shift, 2 degrees rotation, 3% scale; corner displacement
is also checked against the existing 8 px shift bound. Normal cleaning keeps its
strict matched-registration requirement.

Each `[GEOMETRY]` log prints the detector's exact reason, inlier count, center/corner
displacement, rotation, scale, configured limits, frame sizes and registration state;
`hazard=null` means a nonfatal detection warning. The old failed runs recorded only
a generic error, so their actual displacement cannot be reconstructed from those
logs. Robot pose verification, chassis/tool checks, bounded MOVE and STOP handling
remain unchanged. Startup may wait up to 2 seconds
for the first verified marker pose before connecting TCP; subsequent observation
waits stay bounded to 100 ms. Faults stop the run, with no automatic resume.
Evidence uses a unique `data/auto_navigation_*` directory. Software-only test:
`.venv/bin/python main.py auto-clean --navigation-only --output-dir data/auto_nav_sim_new`.

Physical use additionally requires `--mode physical --host ESP32_IP --port 3333
--background REGISTERED_BASELINE.png --confirm-clean --authorize-motion
--confirm-fixed-camera --confirm-ground-reference --confirm-static-tools` and the
existing measured hardware/timing/verified-pose configuration. The current incomplete
physical contact/spray/clearance template remains CONFIGURATION_REQUIRED and is
rejected before camera/network access when neither surface is configured. No demo
measurements are used physically. No firmware, calibration or UI changes are needed.

[Latest operator measurements](PHYSICAL_MEASUREMENTS_REPORT.md): F 16 cm/s,
B 18.5 cm/s, L 4000 ms/90°, R 3000 ms/90°. These approximate physical estimates are
kept separate from stopping bounds. Latest physical heading observation: **L decreases
Module 04 theta; R increases theta**. LEFT-side
sponge overhang is included in static collision checks; unknown floor contact,
rotating wall contact and spray coverage remain CONFIGURATION_REQUIRED.

The [hybrid navigation update](MODULE06_HYBRID_REPORT.md) uses bounded longer
segments (default≤1000 ms), live Module 04 monitoring, confirmed early STOP and
short near-target corrections. Long mode requires independently measured stopping
bounds; unknown physical latency/speed/coast values stay unconfigured. Use
`movement-calibration add-stop` to record measured bounds in new immutable revisions.

Software-only complete task demos (use new output directories):

```bash
.venv/bin/python main.py simulate-navigation --surface FLOOR --output-dir data/module06_floor_new
.venv/bin/python main.py simulate-navigation --surface WALL --output-dir data/module06_wall_new
.venv/bin/python main.py movement-calibration show
```

Outbound Y→X; each segment requires timer DONE or a confirmed intentional STOP,
settling and a new Module 04 ground-center pose before correction. After exact DONE,CLEANING, return X→Y,
restore the starting heading and verify HOME before READY. Full chassis/tool
sweeps, deadbands and finite correction budgets can block unsafe routes.
The Dashboard can read `--navigation-events PATH/progress.jsonl` for high-level,
display-only progress. It cannot start navigation.

Both R1500 and the newly confirmed **R3000 ms≈90°** references are retained;
unknown stopping bounds and cleaning contact/spray geometry still require measurements.
No simulated rates fill physical configuration. No hardware was operated.
See [the navigation-only update](MODULE06_NAVIGATION_TEST_REPORT.md) for the verified
pose gate, supervised `navigate-test` command and prerequisites. It sends no TARGET
or ARRIVED. During normal tasks, ESP32 owns all cleaning actions; the PC waits for
`DONE,CLEANING`, obtains a fresh pose and only then returns. The Dashboard remains display-only.
See [the Module 06 report](MODULE06_REPORT.md) for calibration commands, physical
prerequisites, protocol limitations, evidence and automated test results.

### Hospital Cleaning Management Dashboard

Install the optional dashboard dependencies separately; the original module CLI
does not load Flask or the dashboard runtime:

```bash
.venv/bin/python -m pip install -r requirements-dashboard.txt
```

Start the offline-capable hospital-facing demo:

```bash
.venv/bin/python main.py dashboard --mode demo --port 5050
```

Open **http://127.0.0.1:5050**. Use Play/Pause, Next step and Restart to present the
simulated detection → priority → cleaning-management workflow. RED_SPILL maps to
HIGH; NON_RED_SPILL maps to LOW. Active work is separate from the pending priority
queue. Select targets, filter status/priority, inspect actual Module 05 proposed
coverage, and review explicitly simulated/manual completion history. No hardware
control or robot engineering panels are exposed.

LIVE can be selected explicitly in the UI, or started with:

```bash
.venv/bin/python main.py dashboard --mode live --camera 0 \
  --width 1280 --height 720 --calibration config/camera_config.json \
  --background data/iphone_reference.png --port 5050
```

LIVE directly shares the `test-vision` Module 02/03/04 processing and OpenCV
annotations: robot marker/pose, confirmed and pending spill boxes/coordinates,
FPS and diagnostics. One camera producer sends the annotated frames over
`/api/stream.mjpg`; browser refresh and additional tabs reuse it. There are no
extra JavaScript overlays in LIVE. DEMO and task history remain unchanged.
It never creates a new baseline or sends robot commands. If the camera disconnects,
capture is released; reconnect it and select **Retry monitoring**. Switching to
DEMO or stopping the server releases capture too. Close standalone camera commands
before starting LIVE so they do not compete for the same device.
Missing camera/setup is handled gracefully; incomplete physical cleaning geometry
remains UNKNOWN. Optional dependency: Flask (in requirements-dashboard.txt); all frontend
assets are local. See [DASHBOARD_REPORT.md](DASHBOARD_REPORT.md) for architecture,
file list, API, limitations, tests and full operation instructions.

### Module 02: real-camera evidence and read-only diagnostics

An experimental `detector.brightening_rejection_enabled` settings boolean is
available and **defaults to false**. It rejects mostly all-channel-brighter,
low-chromatic-change non-red components before tracking, while retaining their
masks and diagnostic rejection reasons. It can also reject real bright/pale
objects. Use only in a separately validated restricted scene; the current
configuration is unchanged. See [the reflection follow-up report](MODULE02_REFLECTION_FOLLOWUP_REPORT.md)
for same-recording red/blue results, scope, commands and remaining live validation.

`--diagnostic-dir` creates a **new** evidence directory with baseline/configuration
snapshots, raw frames, stage masks/contours, photometric statistics, source hashes,
and lossless FFV1 video when the installed codec supports it. Existing directories
are refused. This uses the production detector/configuration. It disables R/Y
baseline replacement for the whole diagnostic session; normal detection controls
are unchanged. `--diagnostic-panels` saves a nine-panel view and shows it when the
GUI is enabled. Saving full-resolution frames reduces observed capture cadence;
per-frame host observation times are retained for delay measurements.

```bash
.venv/bin/python main.py detect-spills --camera 0 --width 1280 --height 720 \
  --background data/iphone_reference.png --confirm-clean \
  --calibration config/camera_config.json --localize --localization-boundaries \
  --diagnostic-dir data/module02_field_evidence/live_sequence_02 \
  --diagnostic-panels --max-frames 600 \
  --output data/module02_field_evidence/live_sequence_02.jsonl \
  --target-events data/module02_field_evidence/live_sequence_02.targets.jsonl
```

Choose a new directory on each run. Stop other camera consumers first. Click the
OpenCV window for keyboard focus. After the hand leaves the image, press A for an
empty scene, B for red only, C after removal, D for blue only, E for both, and F
after a slight lighting change. Wait about five seconds per stage, then Q to end.
Stage keys mark subsequent frames. Hand-entry transitions need separate review;
stage labels alone are not independent object-box ground truth.

```bash
.venv/bin/python -m tools.analyze_spill_diagnostics \
  --run data/module02_field_evidence/live_sequence_02
```

Optional `--labels FILE.json` supplies independently reviewed inclusive frame
intervals and approximate object boxes in original image pixels:

```json
{
  "schema_version": 1,
  "coordinate_system": "original_image_pixels",
  "segments": [
    {"stage": "A", "start_frame": 0, "end_frame": 9, "objects": []},
    {"stage": "B", "start_frame": 10, "end_frame": 39,
     "objects": [{"id": "red-1", "category": "RED_SPILL",
                  "bounding_box": [600, 450, 40, 35]}]}
  ]
}
```

These are illustrative boxes, not measurements of the current scene. The evaluator
reports false positives, matched classification accuracy, recall, per-object ID
fragmentation/duplicates, labeled-window delay and empty-scene disappearance.
Without annotations it reports stage counts and leaves accuracy/false positives
unclaimed. See [MODULE02_REAL_CAMERA_REPORT.md](MODULE02_REAL_CAMERA_REPORT.md)
for the verified real-camera repair, A–F results, remaining reflections and replay.
