# Module 02: restricted-scene brightening experiment

## Outcome and scope

An **opt-in, default-off** component rejection policy removes all 148 residual
false-positive observations in the existing 180 independently labeled stable
frames. Actual red/blue targets retain 180/180 matches and correct classifications.
This is a replay of the existing physical recording, **not a new live-camera
acceptance test or a general zero-false-positive guarantee**. Unlabeled transitions
are not assigned accuracy/false-positive metrics.

The operator explicitly excluded white, pale, transparent and reflective objects
from the current test scope. This experiment can reject genuine bright/pale new
objects. General baseline-relative detection therefore retains its original
behavior unless the experiment is explicitly enabled. No production configuration
was enabled or overwritten.

## Repository and evidence audit

- Starting Git tree was clean at `5d978e1`, preceded by `dabd749`.
- Read the real-camera and Dashboard isolation reports; ZIP integrity of the
  121-entry original diagnostic backup passed.
- A discrepancy in the handoff was found: current `data/iphone_reference.png`
  and its sidecar differ from the historical backup and recording. The current
  sidecar records `2026-11-09T02:32:02.944802+00:00`,
  `registration_trigger=interactive_R_then_Y`,
  `physical_coordinates_verified=false`, and `calibration_may_be_invalid=true`.
  This metadata does not independently establish who registered it or why.
- Current reference SHA-256:
  `92ff9535b8bfdef4ebbbb49b510e58d532d4e1ea1c1b1b72eb10719698c0d5d0`.
  Historical recording reference:
  `f98dabf8ab08ec08b53c4613fe724d1cbbf30d68e1ec935bfa8de2e09700882f`.
- Current files were preserved. Replay uses the reference/settings/calibration
  **inside `live_sequence_01`**, not the newly registered live reference.
- Before edits, saved code, relevant configs, current reference and sidecar with
  hashes to `backups/module02_reflection_followup/session_start.zip`.
- Historical empty-scene regression previously skipped when the live reference
  changed. It now uses `after_empty`'s immutable reference/settings/calibration
  snapshots, so it runs again against the correct historical inputs.

## Measured residuals and experimental policy

`tools/analyze_reflection_residuals.py` streams saved frame records and measures
signed BGR changes against each frame's predicted background, using actual
connected-component masks, including holes. It applies the configured input blur
before comparison. Saved predicted PNG quantization contributes less than one
8-bit level of residual error. Approximate independent boxes and the existing
one-to-one IoU >= 0.25 matching policy separate matched and unmatched observations.

Across the stable windows, unmatched components have 94.8%–100% of component
pixels brighter in **all three channels**, and median channel-change spreads of
17–29 levels. True red/blue targets have much larger median spreads (114–142
levels) and at most about 7.2% all-channel-brighter pixels. These measurements
support a candidate image-change distinction; they do not identify a material or
prove a device-level AE/AWB/focus/stabilization cause.

New `DetectorConfig.brightening_rejection_enabled` defaults to `false`. When true,
after unchanged segmentation/area filtering/red evidence and before tracking:

1. Never reject a component classified red by the existing red rules.
2. For other components, reject if at least 90% of pixels have strictly positive
   corrected residuals in all BGR channels and median per-pixel channel spread is
   at most the existing difference threshold (30 levels in this recording).
3. Retain the segmentation masks and label IDs for diagnosis; record the rejection
   reason and measured values in diagnostic component decisions.

This is a heuristic selected from this recording, not independently validated on
a held-out trial. It is not a new physical trust signal. It is not area removal,
baseline learning or an increase in segmentation thresholds. No threshold,
morphology, red classification or tracker configuration was changed. A unit test
explicitly demonstrates that a genuine neutral bright patch is also rejected
when enabled; this limitation is intentional and documented.

## Same-frame replay

The actual CLI replayed all 593 lossless source frames with the experimental
settings into a fresh diagnostic directory. All 593 saved raw input PNG SHA-256
hashes match the corresponding original recording PNGs. Original calibration
and effective settings were used, adding only the experimental boolean.

| Stable stage (30 frames each) | Physical objects | Default confirmed/frame | Experimental confirmed/frame |
| --- | ---: | ---: | ---: |
| A empty | 0 | 1 | 0 |
| B red | 1 | 1–2 | 1 |
| C removed | 0 | 1 | 0 |
| D blue | 1 | 2–3 | 1 |
| E blue + red | 2 | 3 | 2 |
| F lighting change, both | 2 | 2 | 2 |

- False-positive observations: **148 → 0** across these 180 labeled frames.
- True target-frame matches: **180/180 → 180/180**; matched classification 100%.
- Experimental true IDs: red-first 23, blue 94, red-second 105. No fragmentation
  in labeled windows; blue remains the same ID through D/E/F.
- First stationary/unoccluded → confirmed frames remain 137→139, 369→371,
  469→471. Using **original capture host timestamps**, delays remain 0.388 s,
  0.399 s and 0.398 s. Replay wall-clock time is not capture latency.

Evidence, ignored by Git:

- `data/module02_field_evidence/reflection_review_01/{summary,observations}.json`
- `data/module02_field_evidence/brightening_experiment_settings.json`
- `data/module02_field_evidence/brightening_experiment_01/`
  contains COMPLETE 593-frame manifest, raw images/masks, component rejection
  reasons, `labeled_summary.json` and `verification.json`.
- `data/module02_field_evidence/brightening_experiment_01.jsonl`

## Verification and preservation

Full suite: **760 passed, 1 existing legacy floor-only calibration warning,
33.65 s**. Initial sandbox run could not bind local TCP; full rerun with permission
passed. Six added tests cover signed residuals, duplicate matching, exact masks,
source preservation/no overwrite, opt-in behavior/known bright-target limitation,
red preservation, and the private physical red/blue stable windows (the last test
skips when private evidence is absent). The original historical empty regression
now runs instead of skipping after live background re-registration.

Camera/robot config, current background/sidecar, Dashboard, Module 01/03/04/05,
communication and firmware remain unchanged. Image/physical trust separation is
preserved. No motor control, navigation or camera acquisition was performed.

## Reproduction and next validation

Measure original saved residuals into a **new** output directory:

```bash
.venv/bin/python -m tools.analyze_reflection_residuals \
  --run data/module02_field_evidence/live_sequence_01 \
  --labels data/module02_field_evidence/live_sequence_labels.json \
  --output data/module02_field_evidence/reflection_review_NEW
```

To reproduce the experiment, copy the recording's settings to a new file and add
`"brightening_rejection_enabled": true` within `detector`. Use the original
report's replay command with that settings path and fresh output paths. Keep
`config/spill_config.json` unchanged. Do not use the current live background for
historical replay.

Next required evidence is a separate real red/blue trial with empty/removal and
lighting stages, using the currently registered reference only after checking
that it represents the clean current scene. Do not assume object positions or
camera availability. The experiment remains off pending that validation; the
default detector still has the previously measured reflection false positives.
