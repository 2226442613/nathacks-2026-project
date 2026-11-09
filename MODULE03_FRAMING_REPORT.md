# Non-blocking camera framing and Module 03 fallback

## Root cause and changed checks

`CameraRegistration.from_reference_to_current()` in `vision/localization.py`
produces `Registration exceeds safe small-motion limit` when projected frame
corner displacement exceeds the existing default 2 px correction limit. It
correctly rejects that transform as unsupported and returns MOVED with no
current-to-calibration matrix. Previously `SurfaceLocalizer.localize()` then
returned UNKNOWN/null coordinates before evaluating mask ownership. A Module 02
framing override did not bypass this independent Module 03 gate.

The 2 px alignment validity threshold is unchanged. The new localizer uses a
clearly documented diagnostic identity fallback when no reliable supported
registration is available. It evaluates the exact target mask against loaded
calibration polygons; camera warnings alone no longer change FLOOR to UNKNOWN.
The same on-mask anchor and full boundary pipeline call the existing Module 01
homography APIs. Reliable bounded registration still warps ownership polygons
forward and maps anchors/contours backward once. Large/unreliable transforms are
not applied or silently accepted as reliable.

Module 02 no longer raises FramingMismatch for motion or pauses foreground
emission on uncertain registration/broad change. It keeps comparing to the
immutable baseline with the existing segmentation, size filters and binary red
classification. The compatibility exception class and framing_override API
argument remain available, but warnings do not require them. Source-index
provenance differences are also warnings; actual image-size/data incompatibility
still fails. Framing warnings do not reset tracking IDs.

## Structured output and trust

Fallback retains the original registration status/reason/matrix metadata and adds:

- `fallback_active: true`
- `mapping_mode: CURRENT_FRAME_FALLBACK`
- `registration_warning: <original reason>`

Ownership uses `overlap_coordinate_frame=current_frame_loaded_calibration_polygons`.
The applied coordinates are current-frame pixels, not coordinates corrected by
an unsupported transform. Positions, full/compact contours, bounds, width,
depth/height and area all remain available as UNVERIFIED estimates. Warnings are
also recorded in localization warnings/reasons and v1 target-message metadata.
Even a bound independent measurement report cannot mark a fallback VERIFIED.

Actual cross-plane ambiguity remains BOUNDARY/UNKNOWN with null physical geometry.
Missing/invalid calibration, incompatible dimensions, singular/nonfinite
homographies, invalid masks or impossible/nonfinite transformations remain rejected.
Version-1 fields, geometry representation, stable IDs, append-only updates,
freshness and inactive/last-known handling are retained. All messages remain
`actionable=false` and `motion_authorized=false`.

## GUI and baseline behavior

Normal mode uses compact `CALIBRATION: ESTIMATED | UNVERIFIED` and the existing
optional geometry overlay. No framing prompt, C requirement, large OVERRIDE_ACTIVE
or CALIBRATION MAY BE INVALID banner is shown. Detailed state/quality information
remains in debug mode and JSONL. R/Y confirmation to intentionally replace the
legal baseline is retained; pending confirmation does not stop detection. Only
explicitly confirmed baseline capture temporarily pauses extraction. Cancellation
or capture failure resumes without implicitly changing the baseline. Stationary
targets are never automatically absorbed.

## Files changed

Implementation:
`vision/spill_detection.py`, `vision/spill_cli.py`, `vision/framing_session.py`,
`vision/localization.py`, `vision/localization_runtime.py`.

Tests: new `tests/test_operator_managed_localization.py`; updated
`tests/test_framing_override.py`, `tests/test_legal_scene.py`,
`tests/test_spill_extensions.py`, `tests/test_localization.py`,
`tests/test_localization_cli.py`, `tests/test_target_geometry.py`.
Old expectations that required framing rejection/pause were replaced with the
new explicit operator-managed policy while retaining invalid-data rejection,
background durability, capture confirmation and camera resource checks.

Documentation: `README.md`, this report; the older
`MODULE03_GEOMETRY_REPORT.md` now links to this superseding policy.
No Module 01 calibration math, geometry algorithms, message schema, baseline
storage algorithm, camera manager, or later robot module was changed.

## Verification

`.venv/bin/python -m pytest -q`: **459 passed, 1 warning in 8.59 s**.
The warning is the existing legacy-floor-only calibration notice. Python compile
checks passed.

New regressions cover the exact NON_RED_SPILL-inside-FLOOR, over-small-motion-limit
scenario, WALL fallback, no localization-reference requirement, unknown baseline
association, fallback never VERIFIED despite measured evidence, preserved physical
geometry/events/IDs, genuine boundary ambiguity, source-index warning, compact
normal GUI versus debug diagnostics, and explicit camera disconnection failure.
Existing inverse-direction/no-double-alignment, invalid calibration/nonfinite
geometry, legal-baseline, color classification and full Modules 01–03 tests pass.
Camera/GUI regression runs use mocks; no physical iPhone accuracy or live FPS is
claimed.

Before/after SHA-256 values match for saved inputs during this task:

- `config/camera_config.json`: `02d58dbc2a12ad01c451bf77541a4379ccf6d839d895124bb2f8088f6edcf197`
- `data/iphone_reference.png`: `a012af06643e5158cc4e3e26c34dcd1a9313b4fb3508e9d080d00df1613c3d5a`
- `data/iphone_reference.png.json`: `f97aa100deb4af9e9b7c24fd58ee51b4df9910ab106843320b1460cb834f8573`

## Live iPhone command

No C or localization-reference declaration is required:

```bash
cd /Users/williamwu/natHack
.venv/bin/python main.py detect-spills --camera 0 --width 1280 --height 720 --fps 30 \
  --background data/iphone_reference.png --confirm-clean \
  --calibration config/camera_config.json --localize --localization-boundaries \
  --output data/module03_operator_live.jsonl \
  --target-events data/module03_operator_targets.jsonl
```

The operator decides when to recalibrate. Moving the camera can produce baseline
false foregrounds and inaccurate diagnostic cm estimates; fallback does not
recover the true camera pose. Geometry still describes the visible segmented
raster footprint, not hidden or true three-dimensional extent. Actual field
measurements remain necessary to assess physical accuracy. No ArUco, navigation,
Wi-Fi transport or robot hardware control was implemented.
