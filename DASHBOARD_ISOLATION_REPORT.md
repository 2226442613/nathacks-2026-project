# Dashboard isolation investigation and fixes

## Outcome

The original Module 01–05/07 source files were left unchanged. Dashboard LIVE
continues to call the existing BackgroundReference, SpillDetector, SpillTracker,
SurfaceLocalizer, LocalizationRegistration and TargetEventStore APIs. No alternative
detector, threshold tuning, tracking rules or physical-geometry implementation was
introduced. Dashboard presentation and its task queue remain additional consumers.

This is a proven integration correction, not recovery of an exact historical
project version. The workspace has no Git repository or verified complete
pre-Dashboard source archive.

## Recoverable backup taken before edits

- `backups/dashboard_isolation_20261109T014856Z/current_code_and_config.zip`
- `backups/dashboard_isolation_20261109T014856Z/sha256.json`

The archive includes 118 files: all relevant source, tests, configuration,
firmware, tools, root documentation/dependency files, and the current legal
baseline PNG/metadata. ZIP integrity and every archived SHA-256 were checked.
The virtual environment, caches and large unrelated experimental recordings were
not included. This is the current pre-fix backup, not a pre-Dashboard backup.
Individual archived files can be recovered without resetting the project.

## Evidence about Dashboard development

The available local operation record is:
`/Users/williamwu/.codex/sessions/2026/11/06/rollout-2026-11-06T20-51-02-01a1b421-951e-7f10-9205-8c0c0d15e866.jsonl`.

Dashboard implementation records around 2026-11-09 00:42–01:10 UTC show these
changes to pre-existing project files:

| Existing file | Attributable change |
| --- | --- |
| `main.py` | Register the additional dashboard subcommand and dispatch it to `run_dashboard`; no existing command branch replaced. |
| `requirements.txt` | Append `Flask>=3.0,<4`. |
| `README.md` | Add dashboard startup and operating instructions. |

New dashboard package/assets, its tests, preview images and DASHBOARD_REPORT.md
were added. Reads of `vision/` and `config/` in these records are not writes.
No available Dashboard-development mutation record changes the detector, tracker,
camera manager, localization implementation, detection configuration, robot
configuration or firmware. Source modification times also place the detector and
spill configuration before Dashboard development. This is evidence from available
records, not a claim that a full historical tree was recovered.

In particular, the whitelist redesign that removed shape/material screening and
the later non-blocking framing policy both predate Dashboard development. They
were not reverted or retuned in this change.

## Proven differences and changes

1. **Web dependencies leaked into the original CLI.** `main.build_parser()`
   registers Dashboard, whose CLI module eagerly imported Werkzeug, Flask and the
   Dashboard service. A subprocess that deliberately made web packages unavailable
   could not even parse an original module command. Imports now occur only inside
   `run_dashboard`. Parser registration remains additive and lightweight.
2. **Dashboard imposed extra baseline restrictions.** Its adapter separately
   required a metadata sidecar and rejected a different baseline camera index.
   Original `detect-spills` uses BackgroundReference.load, which supports explicitly
   trusted legacy PNG baselines and validates sidecars/checksums when present.
   Dashboard now uses that same loader rather than duplicating baseline validation.
   It never creates, re-registers or replaces a baseline.
3. **Dashboard rejected a different calibration source index.** Original detection
   continues with a source-index quality warning and unverified association.
   Dashboard now applies that same warning and registration-association behavior.
   It does not claim that matching image sizes or camera indices verify a pose.
4. **Base dependency installation acquired a web requirement.** The historical
   Flask addition is removed from requirements.txt and retained in the new optional
   requirements-dashboard.txt, which includes the original requirements. README
   documents the separate install. Existing installed packages are not uninstalled.

Actual incompatible dimensions, invalid metadata/checksums and invalid geometry
remain rejected by their existing APIs. The adapter still requires a usable arena
configuration for its arena/planning view; this does not affect standalone CLI
handling of incomplete localization configuration.

## Files modified in this isolation change

- `dashboard/cli.py`: delay web/service imports until Dashboard execution.
- `dashboard/live.py`: remove duplicated baseline checks and match original
  camera-source warning/registration semantics.
- `requirements.txt`: restore its original three non-web dependency entries.
- `README.md`: document optional web installation.

New files: requirements-dashboard.txt, tests/test_dashboard_isolation.py,
this report, and data/dashboard_isolation_validation replay evidence.
No edit was necessary in main.py or any vision, robot_comm, firmware or config file.
All other Dashboard backend, frontend, demo and queue files remain unchanged.
RED_SPILL → HIGH and NON_RED_SPILL → LOW remain unchanged. There are no Robot
Status or ESP32 Monitor panels in the frontend. No motor/navigation code was added.

## Before/after and standalone/LIVE verification

Seven new regression cases were added. Before the adapter/import fixes, six initial
cases produced **2 passed, 4 failed**: normal and unverified-baseline replay already
matched; both source-index differences, legacy PNG input and missing web packages
exposed the documented integration problems. After the fixes all seven pass,
including the added saved physical-camera-frame replay.

The comparison runs the actual standalone `main.main` detection command and the
actual LIVE adapter over the same decoded pixels. A replay camera supplies identical
frames; there is no mock detector, tracker, registration or localizer. The comparison
is taken before task presentation/planning filtering and covers:

- Every per-frame tracked candidate, confirmation state, category and stable ID.
- Entire localization records: contours, surface, coordinates, bounds, physical
  dimensions/areas, trust state and warnings.
- UPSERT/INACTIVE target events, including geometry and lifecycle.

Observation time is frozen for comparison. Independently generated producer
session IDs and their record-key prefixes are excluded from event equality, because
different runs must have different session identities. Target IDs are compared
without normalization. No detection or geometric values are excluded.

Inputs include the existing 16-frame synthetic Module 03 video and eight saved
1280×720 iPhone input frames. The saved iPhone frames predate the current baseline
and do not provide ground truth for present false-positive rates; their use proves
integration parity, not physical accuracy. Sparse scenes, source differences,
unverified association, a legal legacy PNG, FLOOR, WALL, BOUNDARY, both categories,
and stable per-surface IDs are exercised.

Additionally, the pre-fix backup was extracted to an isolated temporary directory
and its standalone CLI compared with the post-fix standalone CLI on the same
16-frame recording. Detections and complete localization records are exactly
equal. Confirmed counts in both runs:

`[0,0,0,0,0,3,3,3,3,3,3,3,3,0,0,0]`

Both runs contain RED_SPILL/NON_RED_SPILL and FLOOR/WALL/BOUNDARY. Evidence:

- `data/dashboard_isolation_validation/standalone_before.jsonl`
- `data/dashboard_isolation_validation/standalone_after.jsonl`
- `data/dashboard_isolation_validation/comparison.json`

The before/after baseline here is the backup taken at the start of this task, not
an invented pre-Dashboard version. Simultaneous independent physical camera runs
are not expected to receive identical frames; LIVE retains its existing processing
cadence. No live-camera accuracy claim is made.

## Preservation and full tests

All 54 backed-up vision, configuration, communication, firmware and reference files
match their pre-edit SHA-256 hashes. main.py is also unchanged in this task.

The older protected-file snapshot taken before Dashboard development still matches
camera_config.json and robot_config.json. The current baseline PNG and metadata
differ from that older snapshot. Current metadata records explicit 15-frame median
registration at **2026-11-09T01:38:10.464867+00:00**, after the available Dashboard
implementation records. This investigation does not attribute that registration
to Dashboard or undo it. Both files are preserved exactly as found for this task.

All 14 CLI command help/registration checks pass: preview, calibrate,
verify-calibration, background-reference, detect-spills, validate-localization,
generate-aruco, track-robot, calibrate-robot-plane, calibrate-robot-interactive,
plan-cleaning, robot-simulator, robot-client and dashboard. These are parser checks,
not claims of physical camera/robot execution.

Full regression suite: **746 passed, 1 existing legacy floor-only calibration
warning, 21.01 seconds**. No new failures. Local TCP simulator tests require the
existing permission to bind localhost; no ESP32 or physical robot is contacted.

## Commands

```bash
cd /Users/williamwu/natHack
.venv/bin/python -m pytest -q
.venv/bin/python -m pytest -q tests/test_dashboard_isolation.py
```

Original standalone live detection:

```bash
.venv/bin/python main.py detect-spills --camera 0 --width 1280 --height 720 \
  --background data/iphone_reference.png --confirm-clean \
  --calibration config/camera_config.json --settings config/spill_config.json \
  --localize --localization-boundaries \
  --target-events /tmp/hospital-guardian-isolation-targets.jsonl
```

Optional Dashboard installation and launch (stop the existing process on 5051
before restarting it; do not run competing physical-camera consumers):

```bash
.venv/bin/python -m pip install -r requirements-dashboard.txt
.venv/bin/python main.py dashboard --mode live --camera 0 --width 1280 --height 720 \
  --background data/iphone_reference.png --calibration config/camera_config.json \
  --settings config/spill_config.json --port 5051
```

## Remaining uncertainty

There is no verified exact pre-Dashboard source snapshot or identified last
successful physical-camera detector version. Available evidence does not show
Dashboard retuning standalone detection. These integration fixes do not establish
that the reported real-camera false positives are resolved. Current baseline/view
agreement, lighting and earlier detector-policy changes require separately labeled
physical observations to diagnose without guessing or arbitrary threshold changes.
