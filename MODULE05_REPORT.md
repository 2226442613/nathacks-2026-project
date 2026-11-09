# Module 05 — cleaning contact reachability and coverage planning

Software implementation is complete. Physical cleaning acceptance is **pending real
hardware measurements and testing**. This module never operates actuators or
provides movement authorization.

## 1. Files and existing interfaces

New implementation:

- `vision/cleaning_config.py` — validated, explicit hardware geometry and unknowns.
- `vision/cleaning_geometry.py` — physical footprints, polygon validation/clipping,
  disjoint rectangle unions/intersections/differences and area accounting.
- `vision/cleaning_planner.py` — target adapter, typed approach/segments/plans,
  independent contact/spray reachability, floor and wall coverage.
- `vision/cleaning_cli.py` — read-only JSON/JSONL ingestion and offline CLI.
- `vision/cleaning_visualization.py` — optional floor/wall PNG planning diagrams.
- `config/cleaning_hardware.json` — unconfigured PHYSICAL template.
- `config/cleaning_demo.json` — explicitly illustrative SIMULATION geometry.
- `config/cleaning_hardware.schema.json`, `config/cleaning_plan_v1.schema.json`.
- `tests/test_cleaning_planner.py`, `tests/test_cleaning_cli.py`.
- `tools/generate_cleaning_demo.py`, this report and `data/module05_demo/` fixtures.

Modified: `main.py`, `vision/__init__.py`, `README.md`.
No requirements/dependency changes or modifications to Modules 01–04 algorithms,
serialization schemas, saved camera calibration, legal baseline or experimental
robot records. No Module 06–09 logic was implemented.

Inspected/reused:

- Module 01 `GroundCalibration`, `DualPlaneCalibration`, `to_dict/from_dict` validity,
  floor/wall dimensions and `wall_to_floor_base` for wall X association.
- Module 03 exact full physical rings, compact geometry fallback, localization
  status, v1 target messages and append-only event snapshots with stable keys.
- Module 04 ground-center / forward-left frame, heading convention, ROBOT_POSE
  records and `robot_pose_is_current` freshness handling.
- Existing strict `json_native/json_line`, main CLI and test conventions.

The legacy Module 01 `RobotReachGeometry` upper-height/distance placeholder is not
used as a cleaning-reachability decision: it cannot describe distinct spray/contact
mechanisms or the fixed sponge's lower blind zone. It remains unchanged.

## 2. Architecture and hardware configuration

`CleaningHardware` validates source mode, installed true/false/null, finite positive
dimensions, nonnegative clearances, minimum/maximum ordering, mounting side,
rotation ranges, required coordinate conventions and planner limits. Custom tool
polygons are explicitly rejected in this rectangular model rather than silently
replaced by their bounding boxes. Source target polygons can be complex.

PHYSICAL template: every unmeasured hardware quantity remains null, installed
states unknown, `measurements_validated=false`. No null is filled from simulation.
Required missing values yield UNKNOWN and no approach/path. SIMULATION is a
separate file and must be explicitly selected. Mismatched source/mode is rejected.
A physical template completed later with measurements uses the same algorithms.

Configuration sections:

| Section | Required geometric model / scope |
| --- | --- |
| Body | Chassis length/width, ground-center forward-left reference, safety margin, position uncertainty |
| Front floor sponge | Fixed contact length/width and forward/lateral contact-center offset |
| Fixed floor nozzle | Independently calibrated effective impact rectangle, offset and rectangle orientation; fixed mounting/direction metadata remain distinct |
| Side wall sponge | LEFT/RIGHT, forward offset, guaranteed contact width, independently measured minimum/maximum contact heights, protrusion, controlled compression and minimum chassis-wall clearance |
| Adjustable wall nozzle | Mounting/impact offsets, safe distance interval, lift/rotation limits and independently calibrated guaranteed impact-height/width reach |
| Planning | Pass overlap, geometric lead-in/retreat preferences and maximum stroke budget; these are proposal parameters, not measured robot performance |

Nozzle mount height/physical direction and sponge diameter are retained metadata;
coverage is not inferred from them when a measured effective footprint is the
input. Optional unused metadata can remain unknown; required coverage/clearance
measurements cannot. In particular, nozzle lift is not wall impact height.

Illustrative simulation values include chassis 12×8 cm, safety margin 1 cm,
position uncertainty 0.5 cm, front sponge 4×6 cm with forward offset 4 cm, side
sponge guaranteed H=5..13 cm and wall nozzle guaranteed impact H=0..20 cm. These are
**not project measurements**. The provisional 20 cm nozzle estimate never fills
the physical template or defines the side sponge's maximum contact height.

Effective rectangle models must be conservative **guaranteed coverage**, not the
bounding box of a circular sponge or irregular/coupled spray envelope. If actual
coverage is not such a rectangle, supply a measured safe inscribed effective
rectangle or leave coverage unconfigured. Arbitrary tool polygons/non-cardinal
effective floor spray orientations are not currently supported; the planner
explicitly returns an error/UNKNOWN instead of claiming approximate full coverage.
This does not restrict target contour shapes.

## 3. Floor approach and coverage algorithm

For each cardinal heading 0/90/180/270:

1. Transform target and arena into robot-oriented longitudinal/lateral coordinates.
2. Inflate chassis half-extents by configured safety margin + position uncertainty.
3. Intersect feasible center ranges with the active tool's containment envelope.
   The fixed front sponge stays present during SPRAY passes too.
4. Place overlapping contact-width rows; intersect the actual target rings with
   each row, then generate forward strokes for real outer-component X intervals.
   Holes/disconnected components are retained, not replaced by one target box.
5. Account for tool forward/lateral offsets when converting tool coverage into
   ground-center poses. Start/end are before/after target contact where feasible.
6. Generate separate nozzle wetting strokes using its independently calibrated
   impact footprint; evaluate the intersection of spray and contact coverage.
7. Choose the candidate maximizing joint target area, then contact area and compact
   stroke count; a fresh pose may break ties by heading. Stroke limits never cause
   silent truncation with a FULLY_REACHABLE label.

Both endpoint chassis footprints are contained. For fixed-heading straight strokes
in a convex rectangular arena this also contains intermediate chassis footprints;
fixed sponge/tool envelopes are checked too. This proves arena containment under
the configured model, not collision freedom against objects. Paths between strokes,
rotations and obstacle avoidance are delegated to later routing.

A target wider than a sponge receives multiple passes. The robot center is not
assumed to coincide with target or tool center. Tools may traverse clean holes
between target portions; those holes are never counted as dirty area. Contact and
spray strokes are independent; unsupported nozzle geometry remains UNKNOWN.

## 4. Wall contact, height clipping and approach

The actual wall X/H rings intersect:

- Fixed side sponge minimum..maximum contact-height band and horizontally feasible
  contact region under chassis containment.
- Independent measured wall nozzle impact-height/width reach at a valid safe spray
  distance, with known lift/rotation limits.

The joint intersection defines cleanable area. Below sponge minimum and above
sponge maximum are independent blind zones. The full contour, not center H or
height ratio alone, determines partial coverage. Diameter never sets the band.
Spray-only and sponge-only regions do not count as completed contact cleaning.
Unknown nozzle coverage/lift limits or unknown sponge interval yield UNKNOWN.

For the currently calibrated top shared boundary:

- RIGHT sponge faces the wall at heading 0; LEFT at 180.
- Body wall gap = protrusion − configured contact compression.
- Gap must preserve required wall clearance plus safety/uncertainty inflation.
- Center Y = chassis half-width + body gap, independent of target height.
- Horizontal contact strokes keep Y, heading and sponge height constant.
- Native wall X is associated through Module 01 wall_to_floor_base.

Non-top shared-edge wall approach planning currently returns UNKNOWN rather than
redefining axes. Native Module 01 floor planning remains supported. Invalid
coordinate definitions rejected by Module 01 remain rejected.

The nozzle's measured impact band is a **reachable envelope**, not one instantaneous
lift setting. Plans retain required regions, safe distance and calibrated lift/
rotation ranges, with `specific_lift_angle_solution=null`. No fake actuator
solution is inferred from impact height. Future hardware calibration must supply
settings/control behavior. Force, pressure, water flow and cleaning duration are
not assumed. Controlled contact/compression requires separate physical validation.

## 5. Areas, shape and confidence

Continuous clipping uses source outer/hole polygons against rectangular swept
regions. Rectangle unions are partitioned into disjoint vertical slabs; overlap
is not double-counted. Polygon shoelace area adds outer rings and subtracts holes.
Concave targets, holes and disconnected intersections have analytic regressions.
Clipped uncovered regions are signed outer/hole fragments over disjoint slabs, not
a canonical merged GIS polygon; source rings and coverage rectangles are retained
for reconstruction. This is not centroid testing, global image scaling or coarse grid sampling.

Reported metrics include total, contact-reachable, spray-reachable and joint area
in cm², estimated joint fraction, uncovered target regions and wall lower/upper
blind-zone area. Unknown metrics stay null; known zero coverage is zero.

A 1e-8 cm² numerical comparison tolerance is an arithmetic tolerance, not physical
measurement accuracy. FULLY_REACHABLE means the entire **provided estimated polygon
under the configured guaranteed effective rectangle model** is covered. Source
simplification, camera registration warnings, calibration quality and UNVERIFIED
localization remain in the plan. Synthetic/model coverage is not proof of actual
water/sponge performance or physically exact target boundaries.

## 6. Approach, segments and robot-ready JSON

Reusable structures: `ApproachPose`, `CoverageSegment`, `CleaningPlan`.
The v1 CLEANING_PLAN contract is separate from Module 03/04 messages. It includes:

- Stable target ID and original session/epoch record key.
- Surface, cleaning mode, reachability and geometric_reachability alias.
- Approach ground-center pose/reference/tool/contact model/clearances/validity.
- Source rings, contact/spray/joint regions, areas, fractions and uncovered geometry.
- Configuration snapshot/hash, calibration/frame identity and timestamp.
- Separate target/robot localization trust, activity and freshness metadata.
- Reasons, warnings and tool requirements.

Every plan has `actionable=false`, `execution_authorization=NONE`,
`physical_execution_authorization=NONE`, `route_to_approach_validated=false`.
No source mode or VERIFIED input automatically grants authorization.

Segments: APPROACH, ALIGN, SPRAY, CONTACT_CLEAN, RETREAT, REQUEST_VERIFICATION.
Wetting/contact strokes include physical start/end and waypoint trajectories.
APPROACH is a goal request with route=null, not a straight-line navigation path.
The approach starts the operation; first contact start is separately retained in
desired_contact_geometry. Wall strokes explicitly specify no vertical sponge
motion. RETREAT remains a goal requiring later routing; an unsafe/unproven endpoint
is null with a reason. No motor speed, acceleration, pump flow, duration or servo
command is emitted. REQUEST_VERIFICATION is a declarative later inspection request.

Complete generated sample: `data/module05_demo/cleaning_plan_v1.json`.
It is a synthetic wall plan with PARTIALLY_REACHABLE, approximately 0.5543 joint
fraction and execution NONE. Source geometry comes from Module 03 APIs with the
saved calibration; illustrative hardware comes only from cleaning_demo.json.

## 7. Ingestion, freshness and compatibility

The CLI accepts standalone v1 target messages, raw localizations, localized frame
JSONL and target-event snapshots. Full event geometry is preferred to compact
message contours. Missing contour payload is UNKNOWN; bounds cannot fill it.
Event reading is pure/read-only: it never constructs a producer that might retire
existing active targets. Latest event state per stable key is retained.

Inactive targets have no path unless --allow-inactive explicitly requests historical
analysis. That override never enables execution. Missing/stale/incompatible robot
poses do not prevent offline target geometry analysis, but no last-known pose is
silently substituted as current. The latest actual Module 04 record is used, even
LOST/EXPIRED. Existing robot freshness logic is reused. --as-of is an explicit
historical evaluation time, not a live-data assertion. All plans remain offline
geometric proposals. Geometrically reachable UNVERIFIED targets stay UNVERIFIED.

The earlier Module 03 non-blocking registration fallback is preserved: available
estimated polygons can be theoretically planned under SIMULATION despite camera
registration warnings. Invalid homographies/coordinate definitions remain handled
by Module 01 validation; target calibration/frame mismatches produce UNKNOWN.

## 8. Exact offline commands

Fixtures already exist and require no camera, printed marker, completed robot,
pump or sponge motor. Choose NEW output files/directories; existing experiments
are never overwritten:

```bash
cd /Users/williamwu/natHack
.venv/bin/python main.py plan-cleaning \
  --targets data/module05_demo/targets.jsonl \
  --robot-poses data/module05_demo/robot_pose.jsonl \
  --calibration data/module05_demo/calibration.json \
  --mode simulation --hardware config/cleaning_demo.json \
  --output data/module05_plans_new.jsonl --visualize-dir data/module05_preview_new
```

Expected simulated examples: target 1 FLOOR FULLY_REACHABLE; target 2 WALL
PARTIALLY_REACHABLE (~55.4%); target 3 low WALL UNREACHABLE (spray-only).
All remain UNVERIFIED and non-actionable.

Physical template demonstration:

```bash
.venv/bin/python main.py plan-cleaning \
  --targets data/module05_demo/targets.jsonl \
  --calibration data/module05_demo/calibration.json \
  --mode physical --hardware config/cleaning_hardware.json \
  --output data/module05_physical_unknown_new.jsonl
```

Expected: all UNKNOWN, no approach/path, explicit missing parameters.

New reproducible fixtures:

```bash
.venv/bin/python -m tools.generate_cleaning_demo --output-dir data/module05_demo_new
```

The generator copies calibration read-only into a NEW directory and creates
synthetic targets through actual Module 03 geometry/events and a Module 04 marker
pose. No saved real reference or experimental record is replaced.

Other implemented options: --target-id, --allow-inactive, --as-of,
--robot-poses, --visualize-dir. `plan-cleaning --help` was checked. Outputs are JSONL;
PNG visualization is headless and independent of camera preview windows.

Visual examples: `data/module05_demo/preview_delivery/`. Green=both mechanisms,
blue=spray-only, amber=contact-only, red=neither, gray=unknown. Floor diagrams retain
holes, tool strokes, front sponge/nozzle footprints, approach and chassis. Wall
diagrams show both sponge limits/blind zones, nozzle impact band, colored target
subsets and proposed robot approach on the floor. The diagrams were inspected.

## 9. Automated verification

`.venv/bin/python -m pytest -q`: **606 passed, 1 warning in 9.53 s**.
The warning is the existing legacy-floor-only calibration notice.
95 new tests cover the requested floor, wall and integration cases; all prior
511 Module 01–04 regressions pass. Python compile checks passed.

Coverage includes small/multiple-pass/boundary targets, tool offsets and rotated
chassis, irregular/holed/disconnected target area, contact versus spray insufficiency,
unknown physical dimensions, invalid configs, all five illustrative wall height
cases plus crossing both limits, area-weighted irregular wall clipping, independent
nozzle reach, both mounting sides, wall clearance/protrusion, horizontal/no-vertical
sponge strokes, safe containment during wetting too, typed JSON, stable IDs,
Module 03 full/event/compact ingestion, actual Module 04 pose input and stale/lost
handling, inactive overrides, source units/nonfinite/invalid geometry, stroke limits,
custom unsupported footprint rejection, schema-required fields/config snapshots,
non-actionable simulation/physical plans, output protection, optional visualization,
non-blocking registration fallback and wall H not being used as floor goal Y.

Both simulation and physical-template CLI commands were actually run against the
generated records; physical mode returned UNKNOWN as intended. JSON schemas parse
and required-field/contract invariants are tested; no external schema-validator
package was added. Synthetic success does not establish actual cleaning success.

## 10. Remaining measurements, limitations and physical checklist

Still required from hardware testing:

- Actual chassis envelope, reference center, wheel/tool protrusions and clearances.
- Localization/heading uncertainty, including marker-height parallax and calibration.
- Front sponge true contact shape/size/offset/compression on the real floor.
- Fixed floor nozzle mechanical mounting/direction and guaranteed wetting footprint.
- Wall sponge side, mounting position, protrusion/compression, guaranteed contact
  width and **separately measured lower and upper contact heights**.
- Safe chassis-wall distance, allowable sponge contact force/compression, rotational
  envelope, surface friction and force-limiting behavior.
- Wall nozzle lift/rotation limits, mounting, safe spray distance and measured
  impact coverage/settings; lift and impact height must be calibrated separately.
- Motor/control response, speed/acceleration, water flow, cleaning duration and
  cleanliness verification criteria for later modules, never assumed here.

Validation procedure:

1. Measure real hardware and replace only PHYSICAL template nulls with real data.
   Mark model/coverage validation only after measurement; never promote demo values.
2. Establish conservative guaranteed effective contact/spray rectangles. If coverage
   is irregular/coupled, do not use its optimistic bounding box as guaranteed reach.
3. Verify Module 01/03 geometry and independently measure Module 04 ground pose and
   heading accuracy; preserve UNVERIFIED until evidence supports each claim.
4. Dry-check proposed chassis/tool envelopes and front-sponge offsets at boundaries;
   independently inspect obstacles and route/rotation/withdrawal feasibility.
5. Validate fixed wall sponge lower/upper blind zones, correct side orientation,
   safe gap/compression and force behavior. It must never be commanded vertically.
6. Measure nozzle impact regions at actual stand-off/lift/rotation; separately verify
   wetting and physical sponge contact, including regions reachable by spray alone.
7. Use later approved control policies for any physical experiment. Module 05 emits
   no hardware operations; measure actual cleaning effectiveness and visually verify.

Known limits: cardinal strokes; rectangular body/effective tool model; top-wall
approach support; no obstacle-free route, transit/rotation dynamics, actuator-setting
solver, force/duration/water model or physical cleanliness guarantee. Source mask,
calibration and compact-contour approximation remain. Unconfigured physical geometry
is UNKNOWN. These are explicit software-model limits, not hidden real measurements.
