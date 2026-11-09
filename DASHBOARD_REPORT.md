# Hospital Guardian — Cleaning Management Dashboard

Implemented a local, offline-capable demonstration dashboard with a Flask backend,
plain JavaScript/CSS/SVG frontend, reusable task model, deterministic scenario and
read-only live computer-vision integration. No robot communication client, motor
control, physical cleaning execution or autonomous navigation is imported by the
dashboard package. Existing ESP32 protocol and modules remain intact.

## Start and open

From `/Users/williamwu/natHack`:

```bash
.venv/bin/python main.py dashboard --mode demo --host 127.0.0.1 --port 5050
```

Open **http://127.0.0.1:5050**. The server binds localhost only, runs one process,
uses no development reloader, and defaults to DEMO without opening a camera.
Stop with Ctrl+C; camera/ticker/server resources are released. The delivered
preview was left running in DEMO for inspection.

New dependency: `Flask>=3.0,<4` in requirements.txt, installed in the project venv
(Flask 3.1.3 in this session). For another environment, install requirements once:

```bash
.venv/bin/python -m pip install -r requirements.txt
```

After dependencies are installed, the demo needs no internet, phone, ESP32, arena
or physical robot. Fonts use the system stack; icons, room illustration, scripts
and styles are all served locally. No npm/build step or CDN is required.

## Interface and operator workflow

The dark navy layout includes branding, DEMO/LIVE and monitoring availability,
state-derived metrics, a large monitoring scene, a priority-first task workspace,
separate active task, task details, calibrated surface map/proposed coverage and
recent activity. Responsive layouts were checked at desktop 1440×1000 and narrow
390×844 viewports; no horizontal page overflow was measured at either width.
The browser's temporary viewport override is reset after testing.

- Select cards or target labels to review a target and its actual backend category,
  friendly label, priority, surface, location, estimated size, plan and trust.
- All/High/Low filter priorities; the status selector supports Pending, In progress,
  Completed, Failed, Unreachable and All statuses. Pending tasks can also be
  explicitly flagged unreachable and returned to the queue after review. Default is the pending queue.
  Metrics remain derived from the complete current store, not the filtered list.
- The current in-progress task remains separate and is never silently preempted.
- Play/Pause, Next step and Restart control the repeatable demo. Restart clears its
  own tasks/history and restores default filters. LIVE history is a separate store.
- Task buttons change management state only. DEMO actions say simulated; LIVE
  controls record a manual start/completion/failure. LIVE completion asks for a
  manual confirmation and never claims physical verification or operates equipment.
- A target disappearing is marked not currently visible, with location labeled
  last observed; it is never automatically completed or declared cleaned.
- Unknown surface/location/size remains unavailable. Missing Module 05 physical
  measurements show UNKNOWN / configuration or assessment required.

No robot status/pose, battery, ESP32/Wi-Fi/TCP details, raw transport logs, PWM,
firmware panels, teleoperation or actuator controls are present.

## Priority, identity and state model

`dashboard/model.py::PRIORITY_RULES` is the single classification-to-priority and
friendly-label mapping. The frontend receives the mapping/presentation fields;
it never classifies colors or assigns its own priority.

| Backend classification | Priority | Friendly display |
| --- | --- | --- |
| RED_SPILL | HIGH | Red Spill |
| NON_RED_SPILL | LOW | Blue / Non-Red Spill |

No BLUE_SPILL backend category was introduced. The UI explicitly says color-based
priority is a demo rule and does not identify blood or any medical substance.

Queue order: HIGH before LOW, then detection timestamp in actual UTC chronological
order, then stable arrival sequence. A later HIGH reorders pending work but never
interrupts an IN_PROGRESS task. Original Module 03 target_id is preserved; producer
record_key (session/epoch/ID) provides deduplication and prevents ID-reset collisions.
Repeated current observations update the existing task without duplicate events.
A stable record key cannot change its target ID.

Supported transitions:

- PENDING → IN_PROGRESS / FAILED / UNREACHABLE
- IN_PROGRESS → COMPLETED / FAILED / UNREACHABLE
- FAILED / UNREACHABLE → PENDING after operator review
- COMPLETED is terminal within the session; a demo restart creates a clean scenario.

Only one task can be in progress. Completion source is SIMULATED in DEMO and MANUAL
in LIVE. VERIFIED completion is not offered or accepted; verification remains
requested/pending. Observed events and manually triggered LIVE events are labeled
OBSERVED and MANUAL respectively. Every event has a timestamp.

The in-memory store retains up to 200 tasks and 160 activity events per mode;
planning caches are bounded. Terminal inactive records may be retired at capacity;
active/pending work is never silently evicted. No database or disk task persistence
was added. Restarting the process loses management history, intentionally suitable
for this hackathon demonstration rather than a production hospital system.

## Deterministic demo

All generated targets and completion actions are explicitly labeled DEMO/SIMULATED.
The illustrated room is an SVG scene, not a fake live camera feed. Target positions
and irregular physical contours are generated deterministically as proportions of
the **loaded** arena dimensions, never hardcoded arena measurements. The source
category remains the actual existing category string.

Scenario starts at a clean step 0, then:

1. Floor non-red/blue target #1 appears; LOW task created.
2. Floor red target #2 appears later; HIGH moves above #1.
3. Red task #2 starts; blue stays pending.
4. Red task completes with SIMULATED source; verification requested.
5. Non-red wall target #3 appears (only when wall calibration exists).
6. Blue floor task #1 starts.
7. Blue floor task completes, simulated.
8. Wall task starts.
9. Wall task completes, simulated.

Play advances approximately every three seconds. Pause and Next step permit judge
controlled presentation; Restart reproduces target IDs, coordinates and ordering.
Demo detection timestamps are fixed scenario times, clearly labeled as such in
details. Operator-triggered management actions use their actual action times.
Manual changes pause the script and label it adjusted; incompatible scripted
transitions are skipped rather than interrupting another task or inventing success.

Demo plans are calculated by the existing Module 05 planner using the explicitly
SIMULATION hardware configuration in config/cleaning_demo.json. No example tool
measurement becomes a physical hardware fact. Floor coverage segments are displayed
as proposed dashed coverage, never as a validated navigation route. Wall surface
maps use X/H; floor X/Y approach poses are not misplotted as wall height paths.

## LIVE integration and limitations

Start directly in LIVE, or explicitly choose LIVE in the page:

```bash
.venv/bin/python main.py dashboard --mode live --camera 0 \
  --width 1280 --height 720 --port 5050 \
  --calibration config/camera_config.json \
  --background data/iphone_reference.png \
  --settings config/spill_config.json \
  --localization-settings config/localization_config.json \
  --hardware config/cleaning_hardware.json
```

All parser flags above are implemented. The camera/source/resolution must agree
with the existing calibration and registered scene. If another program owns the
phone camera, stop it before enabling LIVE. Reuse of a matching resolution alone
does not prove camera pose; existing registration/quality rules still apply.

One shared `LiveMonitor` per server uses the existing:

1. CameraManager acquisition (no HighGUI windows).
2. Registered BackgroundReference image + confirmed-clean metadata/checksum.
3. Shared `vision.live_test.LiveVision`: the actual `test-vision` processor/renderer,
   including Module 04 robot tracking and its existing robot exclusion mask.
4. Existing SpillDetector settings/ROI/legal baseline, SpillTracker confirmation and IDs.
5. LocalizationRegistration / SurfaceLocalizer for original target mask ownership,
   contour geometry, FLOOR/WALL/UNKNOWN and existing trust state.
6. In-memory TargetEventStore events consumed by the management TaskStore.
7. Existing Module 05 PHYSICAL planner, with no illustrative hardware substitution.

It never re-registers/replaces the legal reference and never appends to existing
experimental target-event files. Only confirmed current detections become tasks;
inactive events do not complete them. LIVE displays the same OpenCV marker/pose,
spill boxes/contours, candidate/confirmed labels, coordinates, FPS and diagnostics
as `test-vision`; the browser does not redraw these overlays. The shared annotated
JPEGs stream over `/api/stream.mjpg` (MJPEG); state still refreshes about every
0.9 seconds. Multiple tabs and refreshes do not open additional cameras. The
producer targets up to 30 Hz, subject to camera/vision processing time, and
planning recomputation is throttled to at most once per two seconds per changing
target. Temporary feed loss clears the image and marks observations stale for
review after three seconds; task history remains intact.

Detection/setup errors use the original test-vision on-frame diagnostics. Camera
read failures release capture and show the specific error with a retry action.
The page remains usable. Retry stops the old
worker before starting another; an unresponsive worker cannot create a duplicate
camera session. DEMO and LIVE histories never mix.

LIVE pipeline tests exercised the actual detector/tracker/localizer with a mocked
camera producing synthetic frames, including successful non-red FLOOR localization,
stable IDs, contours, and unavailable-camera cleanup. **A physical iPhone feed was
not opened or validated during implementation.** The existing physical Module 05
hardware template is incomplete, so its UNKNOWN reachability is expected rather
than invented. Camera color classification is not medical material identification.

### Shared test-vision LIVE update (2026-11-09)

Modified files: `vision/live_test.py`; `dashboard/live.py`, `dashboard/service.py`,
`dashboard/app.py`, `dashboard/cli.py`, `dashboard/static/app.js`;
`tests/test_dashboard_isolation.py`, new `tests/test_dashboard_live_stream.py`;
`README.md` and this report. No additional dependencies or calibration/settings
changes were made. Existing uncommitted project work was preserved.

Focused validation: **86 passed**, using:

```bash
.venv/bin/python -m pytest -q tests/test_dashboard_live_stream.py \
  tests/test_dashboard.py tests/test_dashboard_isolation.py \
  tests/test_live_vision.py tests/test_live_detection_regressions.py
```

Recorded/synthetic replay compares OpenCV `test-vision` output and Dashboard JPEG
input pixel-for-pixel, including robot marker/pose and spill annotations. Tests
also verify exact encoded MJPEG payloads, multiple subscriptions/browser refresh,
single camera worker, disconnect cleanup, mode switch, server shutdown and reopen.
All camera acquisition in these tests is mocked or recorded-file playback.
No automatic physical cleaning verification or Module 06 navigation exists.

## Architecture and files

Frontend:
- `dashboard/templates/index.html`: single-page hospital operations interface.
- `dashboard/static/styles.css`: responsive dark healthcare styling.
- `dashboard/static/app.js`: periodic state, filters, selection, controls and SVG
  target/coverage overlays; unchanged parts of queue/details/history are cached.
- `dashboard/static/favicon.svg`: local icon.

Backend:
- `dashboard/model.py`: centralized priority, bounded queue/history, state rules.
- `dashboard/demo.py`: deterministic synthetic observations and scenario labels.
- `dashboard/service.py`: mode separation, metrics, demo clock, planning bridge,
  stale monitoring state, singleton worker coordination and cleanup.
- `dashboard/live.py`: read-only existing Module 02/03 pipeline adapter.
- `dashboard/app.py`: Flask routes and local static UI.
- `dashboard/cli.py`, `__main__.py`, `__init__.py`: startup and package.
- `tests/test_dashboard.py`: 36 automated regressions.
- `DASHBOARD_REPORT.md`: this guide.
- `data/dashboard_preview/overview.jpg`, `desktop.jpg`, `mobile.jpg`, `wall.jpg`: actual browser previews.

Existing files modified: main.py registers/dispatches `dashboard`; requirements.txt
adds Flask; README.md documents startup. No Modules 01–05/07 algorithms, hardware
settings, firmware/protocol or experimental data were rewritten.

API:
- GET /health, /api/state?priority=HIGH&status=PENDING
- GET /api/tasks/<producer record key>, /api/frame.jpg, /api/stream.mjpg
- POST /api/mode with {"mode":"DEMO"} or {"mode":"LIVE"}
- POST /api/demo/play, pause, step, reset
- POST /api/tasks/<key>/status with only {"status":"IN_PROGRESS"}, etc.
- POST /api/monitor/retry

POST uses JSON and same-origin checks; assets are local and a self-only content
policy is sent. There are no physical-control or plan-execution endpoints. Internal
browser JSON does not modify the separate ESP32 plain-text protocol. This local
hackathon server has no authentication, production deployment or patient database.

## Verification

Full suite: `.venv/bin/python -m pytest -q` — **739 passed**, **1 existing legacy
floor-only calibration warning**, 16.53 seconds. The full run requires permission
for the already existing localhost Module 07 socket tests; dashboard tests do not
open a physical camera or connect to hardware.

The 36 dashboard regressions cover startup, hardware-free demo, centralized priority
and labels, FIFO and late HIGH ordering, active-task preservation, stable IDs and
no duplicate events/tasks, all status transitions, simulation/manual sources,
unknown physical planning, camera failure/staleness, actual Module 02/03 processing
with mock capture, contours and surface coordinates, selection/details/filters,
activity, reset/reproducibility/play/pause, singleton camera coordination, bounded
state, same-origin/static assets, no control endpoints/imports and CLI compatibility.

Browser verification covered desktop/narrow layouts, no horizontal overflow,
HIGH/LOW filtering, active-versus-pending separation, target selection and planning
maps, simulated completion/history filtering, play/pause and restart/filter reset.
Screenshots are actual rendered application output. Protected calibration,
robot-config and legal-background image/metadata hashes were checked unchanged.
