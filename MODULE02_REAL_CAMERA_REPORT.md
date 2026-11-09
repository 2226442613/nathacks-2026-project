# Module 02 real-camera diagnosis and partial repair

Follow-up: [restricted-scene brightening experiment](MODULE02_REFLECTION_FOLLOWUP_REPORT.md)
adds a default-off option and reports zero unmatched observations in the same
180 labeled stable frames. It also documents a later live-background registration
discovered during audit. The historical results below remain the default behavior;
the follow-up is not new physical-camera acceptance.

## Result and limits

The actual iPhone feed reproduced the problem on an operator-confirmed empty arena.
A verified background-registration defect was repaired without threshold changes.
The subsequent real red/blue A–F recording contains 593 frames, not synthetic
targets. Same-frame before/after comparison substantially improves detection, but
some wall-reflection false positives remain. **Empty-arena confirmed count is not
yet reliably zero.** This is not a claim that all physical-camera issues are solved.

## Protection and Git

No Git repository existed at task start. Git was initialized and the initial
recoverable code snapshot committed as **dabd749**:
`Snapshot original modules and isolated dashboard before camera diagnostics`.
No remote was configured or contacted. .gitignore excludes virtual environments,
caches, credentials, local camera calibrations, real captures, logs, archives and
large generated data. Only small existing synthetic integration fixtures are
whitelisted. Firmware test-placeholder credentials are test stubs, not real secrets.

Before detection edits, a separately verified 121-file archive and SHA-256 manifest
were created at:

- `backups/module02_diagnostics_20261109T015848Z/before_diagnostics.zip`
- `backups/module02_diagnostics_20261109T015848Z/sha256.json`

This archive includes the actual current camera/robot configuration and registered
background/metadata, which are not copied into Git as real camera data. It also
contains all relevant source, tests and configuration. ZIP integrity and individual
archived SHA-256 values were checked. It is the pre-repair snapshot, not a recovered
historic working detector. No reset, destructive checkout or cleaning was performed.

Baseline full suite: **746 passed, 1 existing warning, 21.06 s**.

## Real evidence and verified responsible stage

The read-only cached Dashboard request on 5051 failed because no server was
listening. The existing standalone CameraManager then successfully acquired six
1280×720 iPhone frames with the original detector, before any detection edits:

- `data/module02_field_evidence/prechange_camera_attempt.jsonl`
- `data/module02_field_evidence/prechange_camera_attempt/frame_*_input.png`

The operator explicitly confirmed the arena was empty, with no manual phone
movement or zoom adjustment after background registration. The old detector's
confirmed counts were `[0,0,0,0,10,13]`; candidate counts were
`[0,89,16,16,15,13]`.

Distributed ORB/RANSAC matches measured approximately 10.9–11.3 pixels of center
displacement and 3.0–4.0% scale change relative to the saved baseline, with 258–445
inliers. This is measured image framing change, not proof of physical phone
movement. Its underlying device cause—stabilization, focus, capture startup or
another camera behavior—has not been established.

The original code discarded a reliable image fit when the framing exceeded the
small-motion physical trust limits. The detector then subtracted an unaligned
background. Cardboard texture, printed marks and glossy edges became foreground
components. Persisting artifacts passed unchanged temporal confirmation and became
mostly NON_RED_SPILL. Some large false components also absorbed the real red target
in the subsequent old-version replay, preventing a separate red detection.

An offline counterfactual changed only the background image alignment on those
same six raw frames. It reduced the final confirmed count from 13 to 1. The
production repair reproduces `[0,0,0,0,1,1]` on the same frames. The surviving
approximately 137-pixel component at `[248,135,18,10]` is in a wall-glare region and
was deliberately not hidden by increasing the 80-pixel area threshold.

## Minimal repair and unchanged safety

- `FrameGeometryGuard` retains a separate `image_alignment_transform` after the
  existing finite, distributed, RANSAC consensus checks. Singular fits are rejected.
- `SpillDetector` may warp the **immutable baseline image** with that image-only
  fit before subtraction, including when physical framing status is MOVED.
- Warped invalid borders are eroded and excluded from the analysis mask.
- Existing motion thresholds still produce MOVED. The original `transform` and
  `DetectionResult.reference_to_current` remain unavailable outside physical trust
  limits. A separate internal background transform is never physical authorization.
- Module 03 keeps its original current-frame UNVERIFIED fallback. Physical geometry
  APIs, masks/contours, JSON target-message schema and robot action prohibitions
  remain unchanged. No large image fit becomes a VERIFIED chassis/target pose.

No difference threshold, illumination limit, morphology kernel, contour-area limit,
color rule, tracker matching rule or confirmation count was changed. Tracking IDs
continue through the original SpillTracker. The baseline PNG and metadata are
never learned from, updated or overwritten. No replacement Dashboard detector exists.

## Investigation of other stages

| Stage | Evidence and disposition |
| --- | --- |
| Exposure/white balance | Per-channel gains/offsets change during warmup; their existing bounded compensation is recorded. Sensor AE/AWB controls are not exposed or locked here, so AE versus AWB cannot be separately attributed. Early transient candidates remain possible. |
| Reflections | A localized wall-glare residual persists after alignment. It changes with illumination. No generic reflection rejection was added because this clip does not establish a safe rule that preserves all non-red targets. |
| Camera registration | Proven dominant defect: a strong image fit was discarded solely by physical motion limits before subtraction. Image-only alignment is now separated from physical trust. |
| Morphology/minimum area | Before/after masks and component decisions are saved. Existing settings retained; area suppression is not the repair. |
| Color | Original component-pixel red/non-red rules retained. Both actual test colors classify correctly after repair. Old broad artifacts could merge with a red object before classification. |
| Temporal confirmation | Unchanged three-frame stationary confirmation suppresses some warmup artifacts, but cannot reject a persistent reflection. |
| Duplicate IDs | No true test object has duplicate confirmed observations or an ID change in the reviewed stable windows. Reflection/hand transients still create other candidates and IDs. |
| Lighting | The F-stage mild lighting change was recorded. Its reviewed window has two real targets and no extra confirmed target; this one condition does not establish general lighting robustness. |

## New diagnostic mode

`--diagnostic-dir NEW_DIRECTORY` uses the existing standalone pipeline and its
effective configuration. Existing run directories are rejected. R/Y background
replacement is disabled during diagnosis; original non-diagnostic controls remain.
It saves:

- Exact registered PNG/metadata, calibration and effective detector/tracker settings
  snapshots, with source hashes and units.
- Raw BGR frames, actual raw/corrected differences, gain/offset-predicted background,
  pre/post-morphology masks, analysis/red masks.
- All filtered component contours, accepted/rejected area decisions, tracked
  candidate/confirmed contours in structured metadata, and lifecycle ID statistics.
- Optional nine-panel visualization via `--diagnostic-panels` (also GUI display
  unless headless).
- Lossless FFV1 video when available; PNG frames remain the authoritative fallback.
- Host observation timestamps, operator A–F markers and a completion/error manifest.

The video uses nominal requested FPS. Use recorded host observation timestamps,
not nominal video duration, for live detection delays. Saving diagnostics reduces
capture cadence; this run sampled at about 5 Hz. Camera sensor and transport delay
are not measured.

The offline evaluator accepts independently reviewed original-pixel boxes. Matching
is one-to-one with IoU >= 0.25; classification accuracy is conditional on matched
objects. Counts alone are not labeled false positives without ground truth. Stage
keys identify review windows, not automatic ground-truth boxes.

## Actual A–F validation and same-frame comparison

The operator performed the six requested stages. Final recording length was
**593 frames** (Q ended recording before the 600-frame safety cap):
`data/module02_field_evidence/live_sequence_01/`.

All 593 decoded FFV1 frames were compared pixel-for-pixel to their raw PNGs.
The original pre-repair source archive was extracted into an isolated temporary
directory and ran its actual standalone CLI over that exact lossless video, using
the saved baseline, calibration and configuration. No project files were restored
in place. Original output is `live_before_replay.jsonl`.

Stable windows were independently reviewed using raw images and approximate boxes:
30 frames per stage, 180 labeled frames total. Hand-entry/exit transitions were
excluded from these count/accuracy metrics. The full 593-frame output is retained.

| Stage | Expected real objects | Old confirmed total/frame | New confirmed total/frame | Old FP observations | New FP observations |
| --- | ---: | ---: | ---: | ---: | ---: |
| A empty | 0 | 12–16 | 1 | 421 | 30 |
| B red | 1 | 6–15 | 1–2 | 334 | 26 |
| C red removed | 0 | 12–20 | 1 | 477 | 30 |
| D blue | 1 | 8–21 | 2–3 | 462 | 32 |
| E red + blue | 2 | 13–19 | 3 | 466 | 30 |
| F mild lighting change, both retained | 2 | 12–18 | 2 | 416 | 0 |

An FP observation is an unmatched confirmed detection in one labeled frame, not a
unique object. Total FP observations: **2576 → 148**, approximately **94.3% fewer**.
Matched ground-truth target-frame recall: **51.1% → 100%** across 180 expected
target-frame observations. Classification accuracy among matched observations is
100% in both; the old version's red misses must not be concealed by that conditional
accuracy figure. New physical target IDs were 30 (first red placement), 118 (blue)
and 138 (red placed again); each remained stable in its reviewed visible windows,
including blue through D/E/F. No duplicate confirmed real-object observations were
found in those windows. IDs need not have identical numeric values across detector
versions, since different false candidates affect allocation order.

Latency from independently reviewed first stationary, unoccluded object frames:

- First red: frame 137 → 139, **0.388 s**, two observation intervals.
- Blue: frame 369 → 371, **0.399 s**, two observation intervals.
- Second red placement: frame 469 → 471, **0.398 s**, two observation intervals.

The first red was visible while moving earlier; this is stationary-confirmation
latency, not a claim of total appearance/sensor latency. Raw-frame review images
are in `appearance_review.jpg`. After reviewed clear removal at frame 268, the
first red target has no stale confirmed observation. The C-stage empty window
still contains the unrelated wall-reflection false positive.

Evidence files under `data/module02_field_evidence/` (ignored by Git):

- `live_sequence_labels.json`, `latency_review.json`, `appearance_review.jpg`
- `live_before_replay.jsonl`, `live_before/frames.jsonl`
- `live_before_summary.json`, `live_after_summary.json`, `live_comparison.json`
- `after_empty/frame_000005_panels.png`, empty-clip before/after summaries

The legacy before output was converted to the new evaluator's wrapper format
using its actual old detections and the corresponding live host observation times.
It was not presented as a diagnostic run created by an older CLI option.

## Files and preservation

Changed production files: `vision/frame_geometry.py`, `vision/spill_detection.py`,
`vision/spill_cli.py`. Added `vision/spill_diagnostics.py` and
`tools/analyze_spill_diagnostics.py`.

Tests: added `tests/test_spill_diagnostics.py`; updated three expectations in
`tests/test_legal_scene.py` and `tests/test_framing_override.py` that previously
expected unaligned background artifacts. The ID test now inserts a real synthetic
red object instead of treating shifted texture as its tracked object.

Documentation: README.md and this report. .gitignore was created/expanded for the
initial snapshot. Camera/robot configs, registered background/metadata, all Dashboard
files, ESP32 firmware/protocol, Module 01 calibration, Module 03/04/05 implementations
and main.py are unchanged. Hashes of all 42 protected backed-up files in these
categories match the pre-edit archive.

Final full suite: **754 passed, 1 existing legacy floor-only calibration warning,
22.80 s**. Eight new tests include actual private empty/colored recording regressions
when available (these skip if their private recordings are absent), no-op diagnostic
output parity, lossless pixel equality, R/Y protection/A–F labels, GUI/headless
behavior, image/physical trust separation, real-object preservation and evaluator
metrics. Dashboard replay parity tests continue to pass without Dashboard edits.

## Exact diagnostic/replay commands

Use a fresh directory and stop competing camera consumers. From
`/Users/williamwu/natHack`:

```bash
.venv/bin/python main.py detect-spills --camera 0 --width 1280 --height 720 \
  --background data/iphone_reference.png --confirm-clean \
  --calibration config/camera_config.json --localize --localization-boundaries \
  --diagnostic-dir data/module02_field_evidence/live_sequence_02 \
  --diagnostic-panels --max-frames 600 \
  --output data/module02_field_evidence/live_sequence_02.jsonl \
  --target-events data/module02_field_evidence/live_sequence_02.targets.jsonl
```

Focus the OpenCV window. After hands leave: A empty, B red, C removed, D blue,
E both, F slightly changed lighting; wait ~5 s each, Q to finish. R/Y never registers
a baseline in this mode. Remove --diagnostic-panels for lower visualization cost;
add --headless for batch capture (stage keys then unavailable).

Replay the completed actual recording with the current detector:

```bash
.venv/bin/python main.py detect-spills \
  --video data/module02_field_evidence/live_sequence_01/capture.avi \
  --background data/module02_field_evidence/live_sequence_01/reference.png --confirm-clean \
  --calibration data/module02_field_evidence/live_sequence_01/calibration.json \
  --settings data/module02_field_evidence/live_sequence_01/settings.json \
  --localize --localization-boundaries --headless \
  --diagnostic-dir data/module02_field_evidence/replay_02 \
  --output data/module02_field_evidence/replay_02.jsonl

.venv/bin/python -m tools.analyze_spill_diagnostics \
  --run data/module02_field_evidence/live_sequence_01 \
  --labels data/module02_field_evidence/live_sequence_labels.json

.venv/bin/python -m pytest -q
```

For before-code replay, extract selected source from the pre-repair archive into a
temporary directory and run its main.py against the same absolute video/reference/
calibration/settings paths. Do not reset the working project or overwrite its
physical configurations. This is how the recorded comparison was produced.

## Remaining work

The dominant registration defect is supported by actual same-frame evidence and
physical red/blue validation. The remaining reflection false positives are real
and retained. Broader lighting angles, glossy/dry conditions, repeated trials,
additional non-red materials, long runs and different focus/framing states are
still required before claiming robust zero-false-positive operation. Exposure/AWB
and device-level framing behavior need separate controlled measurements; no camera
setting was secretly changed. No motor or navigation authorization is introduced.
