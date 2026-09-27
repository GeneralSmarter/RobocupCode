# Front Matrix and Three-Rear-VL53L1X Implementation Plan

Status: completed implementation plan retained as context. The migration and
hunt firmware described here were implemented; see
`FRONT_MATRIX_THREE_REAR_IMPLEMENTATION_STATUS.md` and current source. The
operator later reported that the current robot has been run physically and
works. Statements below are preserved as the plan and evidence requirements at
the time, not as an active backlog or a claim about current physical status.

## 1. Outcome

Replace the rear-facing SEN0628 matrix path with:

- one front-facing SEN0628 8x8 matrix ToF used for front obstacle evidence and
  weight-sized-object perception; and
- three full-field VL53L1X sensors at the rear, aimed straight back and 30
  degrees to either side.

Keep every physical pose, hardware identity, field of view, threshold, timing
value, and row-selection policy in one source-controlled configuration block.
Sensor drivers and navigation code must consume that configuration rather than
embedding mount measurements or angles.

Weight hunting uses the matrix to classify and steer toward the closest
confirmed static weight at the normal full forward ceiling. Once the weight is
within a configurable physical gap of the matrix, initially `30 mm`, visual
steering ends and chassis control blends directly into the saved route
continuation without stopping. The pickup mechanism continues feeding the
weight in the background while the robot maintains forward progress. Because
this version has no internal ToF, the event is an assumed funnel handoff, not
proof that the weight is secured or permission to increment a payload count.
Material and inductive classification are intentionally not part of this
implementation.

In this document, **matrix-proximity handoff** means only the control transition
from visual hunting to route-plus-feed motion. **Saved route continuation**
means the same pending waypoint when a route leg was interrupted, or the
following waypoint when the robot had already completed a search waypoint.
Neither term implies that collection succeeded. The active implementation also
does not include an internal/chamber ToF, inductive classification, payload
counting, or the later pickup/rejection subsystem.

Add a separate `MatrixWeightFollowTest` program for perception/controller
development. It slowly scans in place until it confirms a weight, latches that
track, and then follows the same weight as an operator moves it. This diagnostic
reuses the production matrix frames, classifier, tracker, navigation motion
interface, and final motor writer; it must not duplicate or quietly fork the
production perception rules.

Forward and reverse use the same live-evidence policy after the matrix is
moved. A direction is available when its configured, direction-relevant
sensors are connected, fresh, valid, and clear; prior physical testing is not
an enable latch. The project assumption is that every obstacle capable of
obstructing or damaging the robot during reverse is tall enough to intersect
the rear VL53L1X fields of view at `Z = 180 mm`; lower objects are not
reverse-blocking hazards for this robot. Physical characterization should
calibrate thresholds and record the limits of that assumption without adding
a reverse-only validation flag. The front matrix supplements the existing
four-ray front fan; it must not weaken current runtime collision rejection.

## 2. Coordinate and angle contract

Use the existing robot frame everywhere. The sensor measurements and footprint
are referenced to the supplied centre of rotation:

- origin: centre of rotation, centred laterally and `122.85 mm` from the front
  edge (the current drive-wheel midpoint reference);
- `+X`: forward;
- `+Y`: robot-left;
- `+Z`: upward;
- yaw `0 deg`: forward;
- positive yaw: counter-clockwise/left when viewed from above;
- pitch `0 deg`: parallel to the chassis reference plane; and
- all distances: millimetres in configuration, metres only after an explicit
  conversion inside navigation.

The declared rectangular robot footprint is:

```text
overall width  = 228.000 mm
overall length = 245.700 mm
front extent   = 122.850 mm
rear extent    = 122.850 mm
left extent    = 114.000 mm
right extent   = 114.000 mm
```

Therefore the initial `ROBOT_FOOTPRINT_GEOMETRY` value is
`{122.85, 122.85, 114.0, 114.0}` in front/rear/left/right order. Keep planner
inflation and hard clearance separate from this physical rectangle.

Interpret "rear at 0 and 30 degrees either side" as angles relative to the
straight-rear axis. The resulting robot-frame yaws are `150`, `180`, and `210`
degrees (`210 deg` may be normalized to `-150 deg` only at calculation time).

## 3. Initial configurable sensor layout

These are optical-centre poses, not chassis-envelope measurements.

| Sensor | X mm | Y mm | Z mm | Yaw deg | Pitch deg | Initial role |
| --- | ---: | ---: | ---: | ---: | ---: | --- |
| Rear left VL53L1X | -107.860 | +95.340 | +180.000 | 150.0 | 0.0 | rear obstacle ray |
| Rear centre VL53L1X | -112.860 | 0.000 | +180.000 | 180.0 | 0.0 | rear obstacle ray |
| Rear right VL53L1X | -107.860 | -95.340 | +180.000 | 210.0 | 0.0 | rear obstacle ray |
| Front left inner VL53L0X | +109.298 | +29.063 | +180.000 | +20.0* | 0.0 | front navigation fan |
| Front right inner VL53L0X | +109.298 | -29.063 | +180.000 | -20.0* | 0.0 | front navigation fan |
| Front left outer VL53L0X | +66.453 | +103.571 | +180.000 | +60.0* | 0.0 | front navigation fan |
| Front right outer VL53L0X | +66.453 | -103.571 | +180.000 | -60.0* | 0.0 | front navigation fan |
| Front SEN0628 8x8 | +67.140 | 0.000 | +96.000 | 0.0* | 0.0* | front matrix safety/perception |

All single-zone sensors are recorded at `Z = 180 mm` with level (`0 deg`)
pitch. `*` The supplied measurements did not state the front-fan yaws or matrix
yaw/pitch. The table preserves the current front-fan yaw convention and assumes
a level, forward-facing matrix. Treat starred values as configurable initial
assumptions and refine them from measurement; they are not a separate motion
enable latch.

Also record matrix roll and logical row/column direction. Do not retain the
current implicit 180-degree rear-mount correction after moving the matrix.

## 4. Configuration design

### 4.1 Add a complete mount type

Add a reusable configuration type in `RobotTypes.h`, for example:

```cpp
struct SensorMountPose {
  float xMm;
  float yMm;
  float zMm;
  float yawDeg;
  float pitchDeg;
  float rollDeg;
};
```

Add sensor-specific configuration types that contain a `SensorMountPose` plus
only the hardware and optical values required by that device:

- rear VL53L1X: XSHUT channel, I2C bus, assigned address, full-field horizontal
  and vertical FoV, timing budget, sample period, valid/status policy, and
  stop/clear thresholds;
- front matrix: I2C bus/address, 8x8 dimensions, horizontal/vertical FoV,
  logical row/column transforms, frame timing, stale timeout, and configurable
  safety/perception row masks; and
- front fan: mount pose and existing VL53L0X hardware identity.

### 4.2 Make `RobotConfig.h` the single layout owner

Move all sensor poses out of `Globals.cpp`. Define the following compile-time
arrays/objects together in `RobotConfig.h` (or a focused
`RobotSensorConfig.h` included by it):

- `FRONT_FAN_CONFIG[4]`;
- `FRONT_MATRIX_CONFIG`;
- `REAR_TOF_CONFIG[3]`; and
- `ROBOT_FOOTPRINT_GEOMETRY`.

Changing a bracket measurement should require editing only these declarations.
Runtime state must refer to a sensor ID/config index, not duplicate its angle or
position.

Prefer source-controlled compile-time geometry for the first implementation.
Do not add live geometry editing over Bluetooth: an accidental runtime change
could invalidate collision checking. If persistent field calibration is added
later, require a versioned schema, bounds, CRC, preview, explicit confirmation,
and disarmed-only writes.

### 4.3 Full-field VL53L1X contract

The three rear sensors must use the full 16x16 receiver ROI. Either leave the
VL53L1X at its documented full-field default or explicitly call
`setROISize(16, 16)`. Do not inherit the current object-sensor `16x10` ROI.
Make the full-field choice visible in configuration and cover it with a source
contract test.

Use short-distance mode initially because the safety horizon is close to the
robot, but keep distance mode and timing budget configurable. Threshold values
must be calibrated; do not automatically inherit the old rear-matrix values.

### 4.4 Compile-time configuration checks

Add assertions/tests for:

- exactly three rear configurations;
- rear yaws exactly `150/180/210 deg` for the initial layout;
- mirrored left/right positions and angles;
- finite geometry values before a sensor can be trusted;
- unique XSHUT channels;
- unique I2C addresses among sensors on the same bus;
- valid address and timing ranges;
- full `16x16` rear VL53L1X ROI;
- matrix row/column masks within an 8x8 frame;
- a finite positive `MATRIX_PICKUP_HANDOFF_GAP_MM`, initially `30 mm`, plus
  bounded final-approach prediction distance/time values; and
- the configured physical rectangle exactly matching the declared
  `228.0 x 245.7 mm` body before any separately measured protrusion is added.

Do not infer the collision footprint from optical centres. The supplied front
outer (`|Y| = 103.571 mm`) and rear side (`|Y| = 95.340 mm`) optical centres
fit inside the declared `114.0 mm` side extents. The front matrix optical centre
at `X = 67.140 mm` also lies inside the declared `122.850 mm` front extent. If
any sensor board, lens guard, funnel, bracket, wiring guard, intake, or bumper
can make contact outside the declared rectangle, expand the collision footprint
or model that protrusion explicitly before navigation testing.

## 5. Rear VL53L1X subsystem

### 5.1 Replace object-sensor identities with rear identities

The four current VL53L1X instances belong to the old LOW/UPPER object layout.
Replace that hardware ownership with:

```text
REAR_TOF_LEFT
REAR_TOF_CENTRE
REAR_TOF_RIGHT
REAR_TOF_COUNT
```

Use three existing XSHUT channels and three unique addresses on the existing
VL53L1X bus; leave the fourth channel explicitly disabled/reserved. Record the
chosen pin-to-physical-sensor mapping in `REAR_TOF_CONFIG`, rather than in
switch statements.

Replace the four named global VL53L1X variables with an indexed driver/state
array or a small table of pointers. Keep configuration immutable and runtime
state separate.

### 5.2 Cooperative initialization and polling

Create a focused `RearTofArray` sensor module. It should:

1. hold every rear XSHUT low at startup;
2. raise and initialize one device at a time;
3. assign the configured address;
4. set short mode, full `16x16` ROI, timing budget, and continuous period;
5. use bounded initialization and report a missing device instead of blocking
   boot forever;
6. poll `dataReady()` cooperatively;
7. read at most one ready sensor per scheduler call;
8. retain range status, validity, distance, signal, ambient, timestamp, and
   sequence per sensor; and
9. reconnect in bounded steps without delays in the control path.

Reuse the useful nonblocking mechanics from `ObjectDetection.cpp`, but do not
keep object-classification responsibilities in the rear driver.

### 5.3 Rear observation API

The existing `getRearObstacleRay()` API assumes eight rays share one matrix
origin. That is incorrect for three sensors at different positions. Replace it
with an observation carrying its own origin:

```cpp
struct RangeRayObservation {
  float originXmm;
  float originYmm;
  float yawDeg;
  uint16_t distanceMm;
  uint32_t sequence;
  unsigned long acquiredMs;
};

bool getRearTofRay(RearTofId id, RangeRayObservation &observation);
```

Populate origin and yaw from `REAR_TOF_CONFIG`, never from duplicated runtime
fields. Transform each ray from its actual optical centre when adding map
evidence.

### 5.4 Runtime rear aggregation

Retain one aggregate rear state for the final motor-safety and recovery APIs,
but derive it from the three individual channels:

- a too-close, invalid-close, stale, timed-out, disconnected, or blocked
  required sensor blocks reverse immediately;
- `hasTrustedRearCoverage()` is false unless all three required sensors are
  initialized, valid/current, and within a configured maximum sample skew;
- clear requires all three channels beyond their clear thresholds for the
  configured number of fresh confirmations;
- repeated calls over the same sample must not increment confirmation counts;
- the aggregate distance is the minimum projected clearance, for telemetry and
  reverse rollout only; and
- an aggregate clear value must never hide an invalid individual ray.

`hasTrustedRearCoverage()` is a live aggregate only. It must not depend on a
build flag, manual approval latch, or `*_PHYSICALLY_VALIDATED` constant. It
reports whether the configured rear channels currently provide the required
coverage: initialized, fresh, valid, within sample-skew limits, and clear.
Physical tests tune those limits and document known blind regions; they do not
switch reverse on or off.

Apply the same rule in both directions at the final motion-safety layer in
`MotorControl.cpp`: evaluate current direction-relevant evidence for manual,
test, recovery, and autonomous commands. Do not add either a reverse-only
pre-validation veto or a mode-specific bypass of live collision evidence.

## 6. Front SEN0628 matrix subsystem

### 6.1 Move and rename the driver

Refactor `RearObstacleSensor.cpp` into a front-matrix module while retaining
its useful cooperative I2C packet state machine and bounded 32-byte payload
chunks. Rename all matrix-specific constants from `REAR_MATRIX_*` to
`FRONT_MATRIX_*` and remove rear-policy ownership from the matrix driver.

The matrix driver should publish a frame containing:

- all 64 raw/logical distances;
- per-cell validity/unknown state;
- frame sequence and acquisition timestamp;
- configured row/column transform; and
- sensor pose at acquisition.

Do not hardcode a 180-degree roll. Represent the physical mounting transform
with configurable `flipRows`, `flipColumns`, and/or a documented 0/90/180/270
degree grid rotation, then prove its mapping with unit tests and a stationary
target moved through all four corners.

### 6.2 Separate safety evidence from weight perception

Create two consumers of the immutable front-matrix frame:

1. `FrontMatrixObstacleEvidence` selects configured cells/rows, projects them
   from the matrix pose, and publishes front obstacle rays plus a conservative
   scalar front state for the motor-safety gate.
2. `MatrixWeightDetection` reconstructs the full depth pattern, classifies and
   tracks compact low candidates, and publishes advisory perception. It cannot
   command motors. Only the hunt controller may request a pickup motion through
   `Navigation.h`.

The matrix has a dual role. Normal valid returns remain obstacle evidence. An
active, confirmed weight may be designated as the one collectible target
inside the configured funnel corridor; this is not a general rule that lower
returns are clear. Side obstacles, unselected clusters, upper/tall evidence,
and the wall behind a weight remain in the collision model. A weight against a
wall may be hunted only when the capture approach and required post-handoff
feed corridor remain geometrically feasible, for example through a measured
oblique or tangential approach. The robot must still stop before its body or
funnel violates the configured wall-contact envelope. Do not override a
rejected footprint merely because a hunt is active.

#### 6.2.1 Reconstruct evidence by height, not fixed row number

Use the supplied matrix optical height (`Z = 96 mm`), configured cell angles,
mount transform, and robot attitude at acquisition. For every valid cell,
derive at least horizontal range, lateral position, and hit height:

```text
horizontal_range = measured_range * cos(vertical_angle)
lateral_position = horizontal_range * tan(horizontal_angle)
hit_height = sensor_height + measured_range * sin(vertical_angle)
```

Apply the full configured `SE(3)` mount transform in production rather than
using the simplified equations directly. The sensor is level relative to the
chassis, but the chassis is not level while entering or occupying a ramp. Use
fresh IMU pitch/roll at acquisition when available; otherwise degrade ramp and
height classification to `UNKNOWN` outside a measured small-attitude bound.

Do not use a permanent "lower four rows" rule. A 70 mm-high weight occupies
different rows as range changes. Classify lower/upper evidence from reconstructed
height bands and preserve the source-cell mask for audit and tracking.

#### 6.2.2 Column profile and object evidence

For each column, form a robust vertical range profile from fresh valid cells.
A significant increase in projected range when moving upward through a column
is evidence for a low foreground surface with farther background above it. The
farther upper return may be open space or a wall behind the weight; it does not
need to be a no-return value. This is what permits detection of a weight placed
against a wall.

Use configurable, measured thresholds for:

- the minimum lower-to-upper depth separation;
- the maximum within-surface range spread;
- the expected physical weight width and height bands;
- the maximum ramp-plane residual;
- the maximum static-target world speed; and
- the minimum number of fresh supporting cells/frames.

Publish explicit evidence rather than a premature binary result:

```text
MATRIX_EVIDENCE_NONE
MATRIX_EVIDENCE_WEIGHT_CANDIDATE
MATRIX_EVIDENCE_RAMP_LIKE
MATRIX_EVIDENCE_WALL_LIKE
MATRIX_EVIDENCE_DYNAMIC_LOW_OBJECT
MATRIX_EVIDENCE_MIXED_OR_OCCLUDED
MATRIX_EVIDENCE_UNKNOWN
```

Initial classification rules:

- **Weight candidate:** a compact near protrusion with an upward depth step,
  reconstructed dimensions consistent with a weight, and no contradictory
  tall foreground surface at the same range.
- **Weight against wall:** the same compact near protrusion remains visible in
  front of a broader/farther wall return; the wall is retained separately as a
  hard bound on the pickup trajectory.
- **Ramp-like:** a broad connected region whose reconstructed points fit a
  smooth sloped plane or whose vertical profile changes gradually rather than
  at a compact height boundary. Treat ramps as walls/blocked for navigation and
  never start a hunt from ramp-like evidence in this implementation.
- **Side-approached wall:** broad physical width, continuation to a matrix
  boundary, or a connected near surface spanning more than the calibrated
  weight-width band. Treat it as wall-like even if its vertical profile contains
  a low-object-style depth step.
- **Unknown:** partial, invalid, contradictory, or under-resolved evidence.

Physical width must be derived from the cluster's angular edges at its range,
not from a fixed number of columns. A 50 mm object may occupy one column at a
longer range and several columns at a shorter range.

#### 6.2.3 Motion/distraction evidence

Track candidate centres in the world frame using the frame's acquisition pose.
This rejects moving Spheros, feet during troubleshooting, and transient hands
without confusing the robot's own motion with target motion. Include the
following pseudocode in the implementation design:

```text
onFreshMatrixFrame(frame):
    observations = classifyCompactLowClusters(frame)

    for observation in observations:
        worldPoint = robotToWorld(
            observation.robotPoint,
            frame.robotPoseAtAcquisition)

        track = associateByWorldDistanceAndShape(worldPoint, observation)

        if track exists:
            dt = frame.acquiredTime - track.lastAcquiredTime
            apparentSpeed = distance(worldPoint, track.lastWorldPoint) / dt

            if apparentSpeed > MAX_STATIC_TARGET_SPEED:
                track.movingFrameCount += 1
            else:
                track.movingFrameCount = 0
                track.staticFrameCount += 1

            if track.movingFrameCount >= MOVING_CONFIRM_FRAMES:
                track.evidence = DYNAMIC_LOW_OBJECT
            else if track.staticFrameCount >= STATIC_CONFIRM_FRAMES:
                track.evidence = WEIGHT_CANDIDATE

        updateTrackOnlyOnceForThisFrame(track, observation, worldPoint)

    expireTracksNotObservedWithin(TRACK_STALE_TIMEOUT)
```

Do not implement material classification here. A future inductive sensor may
confirm plastic/metal and provide another payload-presence signal, but it is
outside this sensor-change and hunt implementation.

#### 6.2.4 Target selection and centre estimate

At acquisition, choose the closest confirmed static weight candidate by
projected front-surface range. Then latch that target using world position,
range, width, and source-cell continuity. A farther target must not steal the
lock. Do not continuously switch to a newly closer observation after hunting
has begun unless the original lock has been explicitly abandoned before the
matrix proximity handoff.

The 8x8 array has no single centre column: the robot centreline is coordinate
`3.5`, between columns 3 and 4. Estimate target centre from the weighted average
of adjacent supporting columns rather than one noisy minimum cell:

```text
columnWeight = validLowerSupport * rangeConsistency * temporalPersistence
targetColumn = sum(columnIndex * columnWeight) / sum(columnWeight)
columnError = targetColumn - 3.5
```

Keep uncertainty/source masks. If only one column supports the object, report a
quantized bearing rather than claiming sub-column precision.

#### 6.2.5 Full-speed visual hunt

Replace the current locked world-coordinate pickup goal with a nonblocking
pickup-tracking mode exposed through `Navigation.h`. Mission code supplies the
latest latched target observation; it must not write motors directly or
repeatedly cancel/recreate ordinary point goals.

Request the configured full hunt ceiling from the beginning. Do not add
distance-based slowing. The existing reachable-wheel, braking-clearance,
authority, lease, watchdog, and independent obstacle caps remain able to lower
or veto the command. Initially set `WEIGHT_HUNT_MAX_SPEED_TPS` equal to the
normal planner forward ceiling.

Use a broad steering deadband because the funnel spans the robot width:

```text
if abs(columnError) <= WEIGHT_HUNT_CENTER_DEADBAND_COLUMNS:
    requestedTurn = 0
else:
    requestedTurn = clamp(
        WEIGHT_HUNT_STEERING_GAIN * filteredColumnError,
        -WEIGHT_HUNT_MAX_TURN_TPS,
        +WEIGHT_HUNT_MAX_TURN_TPS)

requestPickupTrackingMotion(
    forward = WEIGHT_HUNT_MAX_SPEED_TPS,
    turn = requestedTurn)
```

Constrain turn so both wheel targets stay in the measured reachable set. Count
and filter only fresh frames. Do not chase single-frame strongest-column
changes.

#### 6.2.6 Matrix-proximity handoff and fluid pickup

This version has no collection-chamber or internal ToF. Define
`MATRIX_PICKUP_HANDOFF_GAP_MM = 30` as the initial physical forward gap from the
front matrix optical centre/plane to the locked weight's closest consistent
front surface. Keep it configurable and distinguish it from raw slant range.
Compute the gap from the locked target's supporting-cell geometry rather than
one noisy minimum cell.

The matrix may stop seeing the weight before the physical gap reaches `30 mm`.
With a `96 mm` sensor height, a `70 mm` weight, and an approximately `30 deg`
lowest downward cell, the geometric blind transition is roughly:

```text
minimum directly visible horizontal gap
    ~= (96 mm - 70 mm) / tan(30 deg)
    ~= 45 mm
```

Therefore arm a bounded final-approach predictor while the locked target is
still fresh, centred, static, close, and consistently approaching. Retain the
target's world-frame front-surface position and acquisition pose. On every
control update, derive the current matrix optical pose from odometry and
compute `predictedTargetGapMm`. Use a direct fresh matrix-derived gap whenever
available; after the expected close-range disappearance, use the projected gap
only within configured maximum forward distance and time bounds.

Transition when:

```text
bestAvailableTargetGapMm <= MATRIX_PICKUP_HANDOFF_GAP_MM  // initially 30
AND finalApproachWasArmedFromFreshConfirmedTrack
AND abs(lastReliableColumnError) <= MATRIX_HANDOFF_MAX_ERROR_COLUMNS
AND authorizedForwardCommand > 0
AND approach range/progress was consistently closing
```

Unexpected target loss before the final-approach arm point is ordinary target
loss, not a proximity handoff. A close-range disappearance after arming is
expected but does not by itself prove the weight was picked up; odometry must
project the locked surface through the `30 mm` handoff plane.

At the handoff:

- stop matrix steering and publish `WEIGHT_FUNNEL_HANDOFF_ASSUMED`;
- atomically blend chassis control into the saved route continuation without a
  neutral command, navigation cancellation, encoder/PID reset, or route pause;
- maintain positive forward progress and bounded curvature for at least
  `PICKUP_MIN_FORWARD_FEED_DISTANCE_MM`, initially the existing `150 mm` final
  push distance, while ordinary route navigation continues; and
- allow the pickup mechanism to keep operating without taking chassis control;
  publish the handoff/feed state for the future pickup/rejection subsystem to
  consume when that separate subsystem is implemented.

The `150 mm` feed distance is a minimum forward-feed contract, not a maximum
post-loss carry, payload confirmation, or reason to stop. During that distance,
route navigation is active but its requested motion is constrained to positive
forward progress and the configured feed-curvature bound. If the saved route
would immediately require reverse, an in-place pivot, or a sharper turn, retain
that route goal and first follow a short capture-aligned feed corridor; then
release unrestricted route steering after the feed contract is complete. This
temporary corridor is not a waypoint and must not advance the route index.

If live obstacle evidence temporarily vetoes forward motion, stop as required
but keep the pickup state `FEEDING_UNCONFIRMED`; resume its remaining feed
distance when forward motion becomes available. A bounded timeout may report
`PICKUP_FEED_INTERRUPTED` or `PICKUP_STATE_UNKNOWN`, but must not fabricate a
secured payload.

Do not increment payload count, declare material, or publish
`WEIGHT_PICKUP_CONFIRMED` in this matrix-only version. Rearm acquisition of
another weight only after the configured forward feed distance/time indicates
that the funnel entrance should be clear. An internal ToF can later upgrade
this assumed handoff to observed in-funnel confirmation without changing the
fluid chassis transition.

#### 6.2.7 Hunt state machine

```text
SEARCH
  -> CANDIDATE_CONFIRM
  -> TRACK_FULL_SPEED
  -> FINAL_APPROACH_PREDICT
  -> MATRIX_30MM_HANDOFF
  -> WEIGHT_FUNNEL_HANDOFF_ASSUMED

parallel after handoff:
  ROUTE_BLEND + PICKUP_FEEDING_UNCONFIRMED
  -> ROUTE + PICKUP_FEED_DISTANCE_COMPLETE_UNCONFIRMED
```

Required failure behavior:

- target lost before final-approach arming: bounded reacquisition, then neutral
  failure;
- target lost after arming but outside the bounded prediction distance/time:
  neutral `FINAL_APPROACH_ESTIMATE_EXPIRED` failure;
- target dynamic: abandon it as a distraction;
- ramp/wall evidence without a compact target: do not hunt;
- side obstacle or unsafe swept footprint: safety veto and neutral failure;
- no collision-free capture approach and post-handoff feed corridor exists,
  including a head-on wall reached before the `30 mm` handoff or before the
  required feed: neutral inaccessible-target result rather than a collision
  bypass;
- matrix loss after the `30 mm` handoff: expected; continue the configured
  positive-forward feed while route navigation runs; and
- feed interrupted beyond its distance/time bound: keep payload state unknown
  and report it without claiming collection.

#### 6.2.8 Telemetry and success measures

For every fresh frame record the evidence class, source-cell mask, reconstructed
width/height, vertical depth step or ramp residual, target world position,
apparent velocity, chosen track ID, weighted centre, column error, filtered
error, direct/predicted target gap, final-approach arm point, `30 mm` handoff,
route-blend transition, remaining forward-feed distance, requested speed/turn,
and terminal reason.

Measure pickup success rate, false-hunt rate, dynamic-distraction rejection,
centre error at handoff, direct-versus-predicted gap error, time/distance to
handoff, uninterrupted forward-feed rate, pickup outcome observed by external
inspection, false-assumed-handoff rate, and side-clearance margin.

#### 6.2.9 En-route acquisition and waypoint resumption

Weight acquisition is not restricted to the final search/standoff range around
a `SEARCH` waypoint. That range controls when the robot performs a deliberate
scan for targets it has not already seen; it is not a permission boundary for
an obvious weight encountered while travelling between waypoints.

When a fresh, confirmed static weight is seen during an ordinary route leg:

1. select the closest eligible candidate and snapshot a `RouteResumeContext`
   containing the route index, segment start/end, along-segment progress,
   interrupt pose, and target track ID;
2. require the target to fit the configurable opportunistic-intercept envelope
   and detour budget, measured from the active route segment or interrupt pose,
   not from the destination waypoint;
3. cancel the route goal once, confirm/latch the target, and enter the same
   full-speed visual hunt used at a search waypoint;
4. at `WEIGHT_FUNNEL_HANDOFF_ASSUMED`, atomically blend into the same pending
   route waypoint while the pickup mechanism continues feeding in parallel;
   constrain route motion to the capture-aligned feed contract before allowing
   reverse, a pivot, or sharper route curvature; and
5. preserve the route index. An en-route assumed handoff does not advance the
   pending waypoint or increment a payload count.

A fleeting, dynamic, ramp-like, wall-like, out-of-envelope, or unconfirmed
observation does not interrupt the route. If the target is lost or rejected
before final-approach arming, stop the hunt cleanly and resume the saved route
context rather than ending the match. After the matrix proximity handoff, route
navigation is already active; the concurrent pickup state reports feed progress
or unknown outcome without taking chassis ownership.

This replaces the current endpoint-centred deviation check. In particular,
`WEIGHT_SEARCH_MAX_ROUTE_DEVIATION_M` must not be evaluated as distance from the
next waypoint: halfway along a long segment that can reject a nearby target
even when the actual pickup detour is small.

### 6.3 Unknown/no-return policy

The SEN0628 uses a no-return value in current code. Initially classify a
no-return zone as `UNKNOWN`, not `CLEAR`. The front matrix may veto motion on a
valid close return while the existing front fan remains the independent source
of known clearance.

Do not make the matrix a required source of front clearance until stationary
evidence proves the device's status semantics for:

- open space;
- black/dull targets;
- partial-cell targets;
- floor and ramps;
- sunlight/ambient IR;
- stale/repeated frames; and
- values at and beyond maximum range.

If the protocol cannot distinguish "healthy and no target inside the safety
horizon" from "invalid measurement", retain the matrix as supplemental
blocking evidence rather than treating no-return as free space.

### 6.4 Front map rays

For obstacle mapping, aggregate selected non-floor rows per matrix column into
up to eight horizontal rays. Each ray starts at the configured front-matrix
origin and uses the configured matrix yaw/FoV. Preserve the full 64-cell frame
for weight detection.

Only mark free space along a ray when that exact zone has trustworthy clear
evidence. Never paint the whole 60-degree cone free from one return.

### 6.5 Separate spin-and-follow test program

Create a separate test firmware target, `MatrixWeightFollowTest`, with a thin
entry point and no route mission. Extract/reuse the production sensor,
`MatrixWeightDetection`, target-tracking, navigation motion, telemetry, and
`MotorControl.cpp` modules so the test cannot pass with a different classifier
or steering convention. It must not auto-start motion after boot; an explicit
test start begins this state machine:

```text
IDLE
  -> SLOW_SCAN
  -> ACQUIRE_STATIC_WEIGHT
  -> FOLLOW_LOCKED_TRACK
  -> BOUNDED_REACQUIRE
  -> STOPPED
```

- `SLOW_SCAN`: request a configurable low turn rate and continuously process
  fresh matrix frames.
- `ACQUIRE_STATIC_WEIGHT`: require ordinary production weight confirmation,
  then stop the scan and latch its track ID. A foot, Sphero, ramp, or wall must
  not acquire the test.
- `FOLLOW_LOCKED_TRACK`: use weighted column error for steering and range error
  relative to `WEIGHT_FOLLOW_TEST_STANDOFF_MM` for low-speed forward/reverse
  motion. Reverse remains subject to the same live rear evidence as every other
  reverse command.
- Once acquired, allow that same track to move within a separate configured
  diagnostic follow-speed bound. This is necessary because the operator is
  deliberately moving a real weight; it must not weaken the production rule
  that rejects a moving object during initial weight acquisition.
- `BOUNDED_REACQUIRE`: on brief loss, stop translation and scan only a bounded
  angle toward the last bearing. Stop neutral on timeout rather than latching a
  different object unexpectedly.

Keep scan rate, standoff, forward/reverse caps, steering cap, allowed locked
track speed, reacquisition angle, and timeout in configuration. Log raw 8x8
frames, evidence class, track ID, centre/range errors, requested/authorized
motion, and every transition. This program is a diagnostic for classifier and
tracking development; it never performs the production `30 mm` pickup handoff.

### 6.6 Future internal-ToF integration guide

As part of this implementation, create
`RobotCode/docs/INTERNAL_WEIGHT_TOF_INTEGRATION_GUIDE.md`. The guide is a
future hardware/software migration document, not a dependency of the
matrix-only build. It must explain how to add an internal ToF later without
reintroducing a stop between hunting and route navigation.

The guide must cover:

- intended optical pose and visibility envelope through the funnel;
- device model, voltage, bus, address/XSHUT allocation, timing budget, and
  nonblocking polling ownership;
- raw/status/freshness telemetry and calibrated distance-to-intake reference;
- empty, entering, feeding, secured/cleared, invalid, stale, and jam-like data
  collection;
- how fresh internal-ToF evidence upgrades
  `WEIGHT_FUNNEL_HANDOFF_ASSUMED` to an observed in-funnel event while the
  robot remains moving;
- moving confirmation counts based only on new samples; no stationary
  confirmation requirement and no chassis stop on first detection;
- how the internal sensor hands evidence to the separate pickup/rejection
  subsystem without commanding chassis motors;
- the future inductive-sensor boundary, explicitly kept out of the initial ToF
  integration;
- simulator 2.5D geometry, fault injection, telemetry schema changes, and
  firmware/simulator/unit acceptance tests; and
- migration and rollback steps that leave the current `30 mm` matrix/odometry
  handoff available until the internal sensor path is selected in configuration.

## 7. Front VL53L0X fan geometry update

Update the existing fan geometry to the measured X/Y positions while retaining
the current yaws only after confirming them physically:

```text
right outer: X=66.453,  Y=-103.571, yaw=-60 deg (assumed)
right inner: X=109.298, Y=-29.063,  yaw=-20 deg (assumed)
left inner:  X=109.298, Y=+29.063,  yaw=+20 deg (assumed)
left outer:  X=66.453,  Y=+103.571, yaw=+60 deg (assumed)
```

Retain the four individual rays as the primary proven front navigation fan
during the matrix migration. Re-run fan origin/angle calibration after the
mechanical changes; changing only the X/Y constants without measuring the
effective optical origin and yaw is not acceptance evidence.

## 8. Navigation, safety, and recovery integration

### 8.1 Range IDs and naming

Remove matrix-specific meaning from `RANGE_FAKE_REAR`. Migrate to explicit
names such as:

```text
RANGE_REAR_AGGREGATE
RANGE_FRONT_MATRIX_AGGREGATE
```

Keep individual rear states outside legacy front/right/left aggregate logic.
If a compatibility alias is needed during migration, make it temporary and
cover its removal with a test/documentation task.

### 8.2 Planner map

Change `PlannerMap.cpp` to consume ray observations with per-ray origins.
Update it in three separate patches:

1. introduce the generic observation type without changing behavior;
2. express the existing rear matrix through that type and prove identical map
   output in the simulator; and
3. switch the source to the three configured rear rays and add the front
   matrix rays.

This sequence separates an API refactor from the behavioral sensor change.

### 8.3 Final motion safety

Maintain these invariants:

- forward and reverse motion use the same current, direction-relevant evidence
  policy;
- any trustworthy close matrix return vetoes forward/turn motion whose swept
  footprint enters it;
- reverse motion uses the live aggregate all-three rear state, without a
  physical-validation latch;
- unknown, stale, or contradictory required evidence blocks motion;
- turns still require swept-footprint evidence, not merely one rear scalar;
  and
- `MotorControl.cpp` remains the only periodic motor-output owner.

The three rear rays do not observe the floor, prove true side coverage, or
fully observe an in-place pivot sweep. Record those limitations just as for
the front sensors, and use physical tests to characterize detection and tune
the configured swept-footprint model. Do not turn that characterization into
a reverse-only pre-validation gate.

## 9. Simulator changes

Update the firmware/WASM bridge and browser simulator from one eight-ray rear
origin to three independently configured origins/yaws. The simulator should
read the same compiled geometry constants as firmware where practical.

Use a deterministic 2.5D sensor model rather than treating every 2D footprint
as infinitely tall. Keep collision footprints separate from optical
visibility. Give each sensor ray a 3D origin and yaw/pitch; give every object a
ground footprint plus vertical geometry:

- weights are `50 mm` diameter, `70 mm` tall cylinders;
- mobile elements use their configured physical diameter/height;
- speed bumps use their configured maximum `25 mm` profile;
- ramps are sloped wedges with a configurable rise (maximum `100 mm`), run,
  uphill direction, and height function across the footprint; and
- panels and arena boundaries use their configured wall height.

For each ray, return the nearest surface that it intersects in both horizontal
footprint and height. If a ray passes above a low object, continue it to the
wall, boundary, or next object behind it. In particular, the level fan sensors
at `Z = 180 mm` must clear legal weights, ramps, speed bumps, and mobile
elements, while still seeing the `400 mm` walls. They must not return a low
object merely because their top-down line crosses its footprint.

For the front matrix, raycast all 64 configured cell directions. A weight
fixture should produce near returns only in cells whose rays intersect its
`0-70 mm` vertical extent, with upper cells returning the farther background.
A ramp fixture should intersect the actual sloped surface and produce the
range/height gradient expected by the ramp classifier, rather than behaving as
a full-height rectangular wall.

Add:

- one raycast per rear VL53L1X optical axis, with configurable full FoV only
  for visualization/target interception, not as invented free-cone evidence;
- front matrix columns/cells at the configured matrix pose;
- independent rear-left, rear-centre, rear-right, and matrix fault injection;
- stale, dropout, invalid-status, repeated-frame, and sample-skew cases; and
- a sensor-layout overlay showing optical centres, centre rays, FoV bounds,
  ray/object hit heights, and the inflated collision footprint.

Add reduced geometric fixtures proving that upper rays clear a weight and ramp
but hit a wall behind them; lower matrix cells hit a weight while upper cells
see the background; ramp-cell distances follow the wedge gradient; and rotating
or mirroring the ramp changes that gradient consistently. Keep simulator claims
limited to deterministic geometry and software behavior. This model cannot
validate reflectance, ambient light, multi-path, spot size, partial-cell mixing,
or actual optical return reliability.

## 10. Telemetry and stationary diagnostics

Add read-only diagnostics that work while disarmed:

- `SENSOR LAYOUT`: compiled X/Y/Z/yaw/pitch/roll, device, bus, address, XSHUT,
  and trust flags for every sensor;
- `REAR TOF`: each rear distance, status, valid/stale/blocked, age, sequence,
  signal/ambient, aggregate minimum, sample skew, and trust reason;
- `MATRIX`: frame sequence/age, row/column orientation, active masks, 8x8 raw
  values, reconstructed cluster geometry, track/motion evidence, selected
  target, centre error, and derived obstacle/weight summaries;
- `HUNT`: hunt phase, locked track ID, requested/authorized speed and turn,
  direct/predicted matrix gap, final-approach arm state, `30 mm` handoff,
  route-blend state, remaining forward-feed distance, and failure reason;
- `FOLLOW TEST`: scan/acquire/follow/reacquire phase, latched track ID,
  standoff/range error, column error, target speed, and requested/authorized
  motion; and
- telemetry schema/build identity so saved logs can be tied to the exact
  geometry.

Diagnostics must not arm, create a navigation goal, or write motors.

## 11. Test-first implementation order

### Phase A - Record the baseline

- Save current source/WASM build identity and full software test results.
- Save representative clean and website-default navigation traces.
- Add no hardware behavior in this phase.

Acceptance: the existing baseline is reproducible and unrelated dirty-tree
changes are identified and preserved.

### Phase B - Configuration and pure geometry

- Add the mount/config types and measured table.
- Add bus-aware address/XSHUT validation.
- Add pure transforms for sensor origin plus ray endpoint.
- Add mirror/sign tests for all eight front/rear single-zone sensors and the
  matrix.
- Set the declared body footprint to `228.0 x 245.7 mm` about the supplied
  centre of rotation and verify any funnel/bracket protrusion separately.

Acceptance: tests prove the listed positions and `150/180/210` rear yaws map to
the expected robot-frame endpoints; no runtime behavior changes.

### Phase C - Rear driver and runtime health policy

- Implement the cooperative three-VL53L1X driver with full `16x16` ROI.
- Implement per-sensor state, sequence, staleness, skew, hysteresis, and
  aggregate trust.
- Update telemetry and fault tests.

Acceptance: every required single-sensor failure makes the affected live rear
coverage unavailable; three fresh, valid, sufficiently synchronized, clear
sensors make it available. No separate physical-validation flag participates
in that result.

### Phase D - Front matrix driver migration

- Generalize/rename the matrix I/O state machine.
- Add configurable grid orientation.
- Publish immutable full frames.
- Add height reconstruction, column profiles, cluster geometry, ramp/wall/weight
  evidence, world-frame motion tracking, and closest-static-target latching.
- Add pure frame fixtures for a weight, weight against a wall, broad side wall,
  ramp, multiple weights, moving low object, invalid cells, and target loss.
- Keep the matrix supplemental to the existing fan initially.

Acceptance: missing/stale matrix frames fail according to the configured mode;
weight logic cannot command motion; ramp/broad-wall evidence never starts a
hunt; a weight against a wall remains a compact separable target; and moving
distractions do not become confirmed static weights.

### Phase E - Full-speed hunt and matrix-proximity handoff

- Add the nonblocking pickup-tracking navigation API and keep
  `MotorControl.cpp` as the sole periodic output owner.
- Implement full-speed weighted-column steering, target latching, broad
  deadband, reachable-wheel turn limits, direct matrix gap, and the bounded
  last-track/odometry predictor through the close vertical blind region.
- At the initial `30 mm` predicted physical gap, publish
  `WEIGHT_FUNNEL_HANDOFF_ASSUMED` and atomically blend into route navigation
  without a stop, goal-cancel gap, PID reset, or route pause.
- Continue at least the configured `150 mm` positive-forward feed distance in
  parallel with route navigation, subject to ordinary live motion evidence.
- Do not implement payload confirmation/counting, chamber state, or material
  rejection in this matrix-only version.
- Create `RobotCode/docs/INTERNAL_WEIGHT_TOF_INTEGRATION_GUIDE.md` with the
  required future non-stopping internal-ToF migration guidance from section
  6.6.

Acceptance: the hunt requests the normal full forward ceiling without a range
slowdown; never switches away from its latched closest target; stops on side or
footprint vetoes; distinguishes unexpected target loss from bounded close-range
prediction; crosses the `30 mm` handoff without a neutral chassis command;
continues route-plus-feed motion; and never labels an assumed handoff as
secured payload.

### Phase F - Separate spin-and-follow diagnostic program

- Create the separate `MatrixWeightFollowTest` firmware target with a thin,
  non-autostarting entry point.
- Reuse production matrix acquisition, classification, tracking, navigation
  motion, and final motor-output modules.
- Implement slow scan, static acquisition, latched moving-weight follow,
  bounded reacquisition, explicit stop, and diagnostic telemetry.
- Keep its low-speed motion constants independent from the production
  full-speed hunt constants without duplicating perception thresholds.

Acceptance: with no target the robot requests only the configured slow scan;
a confirmed static weight stops the scan and becomes the sole latched track;
moving that weight laterally and radially produces bounded follow commands; a
pre-acquisition moving distraction never locks; target loss ends neutral after
bounded reacquisition; and stopping/cancelling cannot leave a motion command
owner active.

### Phase G - Map, planner, safety, and simulator integration

- Migrate map input to per-ray origins.
- Add the three rear rays and selected front matrix rays.
- Update recovery and final motion safety to the new aggregate rear ID.
- Implement the 2.5D object/sensor geometry, 64 matrix cell rays, sloped ramp
  surface, and fault injection.
- Preserve exact historical regression fixtures.

Acceptance: clean/default simulator suites pass; sensor faults terminate
neutral; rear ray endpoints match firmware geometry; upper fan rays clear legal
weights and ramps; matrix cell patterns meet the height/ramp fixtures; and no
collision bypass or unknown-as-clear path is introduced.

### Phase H - Stationary physical characterization

This phase requires explicit permission in the task that executes it before
opening serial or controlling hardware. Keep the robot disarmed with neutral
motor outputs throughout.

Collect raw data for:

- each sensor independently and all sensors ranging together;
- flat walls at multiple gaps and lateral offsets;
- the shortest obstacle classified as a reverse-blocking hazard, placed at
  rear-centre, intermediate lateral positions, both corners, and FoV
  boundaries throughout the stopping envelope;
- representative lower non-blocking objects, to document that they are outside
  the rear sensing contract rather than accidentally treating them as covered;
- dark, reflective, curved, and partial-FoV targets;
- weights alone and against front/side walls at multiple bearings/ranges;
- broad side-wall approaches, ramps, two or more weights, and a moving
  low-height distraction;
- matrix centre/track behavior at the full hunt update rate;
- direct matrix gap versus measured physical gap approaching the close vertical
  blind region;
- last-visible range, odometry-projected `30 mm` crossing, lateral offsets,
  approach yaw, matrix dropout, and odometry-error sensitivity;
- uninterrupted route blending and configured positive-forward feed distance;
- no-target/open-space behavior;
- floor returns on flat ground and maximum permitted ramp pitch;
- cross-talk and ambient light;
- disconnect, stale frame, frozen frame, and invalid status; and
- measured update period, inter-sensor skew, I2C time, and control-loop gap.

Acceptance: every required rear footprint position is detected before the
chosen stopping envelope, every loss of required evidence blocks, and the
front matrix row/status policy is supported by saved logs rather than visual
intuition. Weight/ramp/wall fixtures meet their declared evidence oracles,
moving distractions are rejected, and the direct/predicted gap transition is
characterized against measured physical gaps without claiming payload
confirmation.

### Phase I - Motion and pickup characterization

This phase separately requires explicit permission, confirmed safe setup,
`END_MATCH`, disarmed state, neutral `1500/1500` outputs, valid required
sensors, STOP/disarm cleanup, and saved regression logs.

Proceed from wheels-up output checks to minimum-speed straight reverse, then
bounded curved reverse and turn-sweep cases. Validate actual stopping distance,
loop/watchdog timing, false stops, sensor loss, and neutral cleanup. Use the
results to tune geometry, timing, and clearance thresholds and to document
where the stated reverse-obstacle-height assumption does or does not hold.
These results do not set an enable flag; reverse availability continues to be
computed from live rear evidence before, during, and after characterization.

After rear acceptance and with separately explicit hunt-test permission, run
the pickup workflow from a stationary safe setup: isolated weight, lateral
offsets, weight against a wall, side-wall constraint, multiple candidates,
pre-arm loss, expected close-range disappearance, expired prediction, temporary
forward veto, and repeated full-speed matrix handoffs. Record side clearance,
centre error and true physical gap at handoff, prediction error, command
continuity, route-blend curvature, forward-feed distance, externally observed
pickup outcome, and false-assumed-handoff rate. Do not add an automatic speed
reduction unless measured pickup evidence justifies it.

## 12. Required automated verification

Run after each relevant phase and run the full gate before accepting the
implementation:

```powershell
arduino-cli compile --fqbn teensy:avr:teensy40 RobotCode
Push-Location RobocupSimulator; npm test; Pop-Location
python -m pytest SerialCommandUI TOFReturnSignalExperiment
python -m compileall SerialCommandUI TOFReturnSignalExperiment
```

Add or update contracts covering:

- sensor configuration and coordinate signs;
- per-ray origin transforms;
- front matrix grid rotation;
- no ROI reduction on the rear VL53L1X sensors;
- all-three rear freshness/skew requirement;
- per-sensor and aggregate hysteresis;
- invalid/no-return fail-closed behavior;
- one fresh frame/sample counting once;
- cell-to-height reconstruction with pose/attitude at acquisition;
- compact weight, weight-against-wall, broad side-wall, and ramp evidence;
- world-frame static/moving target tracking and closest-target latching;
- en-route interruption, route-segment detour accounting, pre-handoff route
  resumption, and same-waypoint continuation after an assumed handoff;
- weighted centre coordinate `3.5`, deadband, and reachable-wheel turn bounds;
- spin-and-follow diagnostic scan/acquire/latch/follow/reacquire transitions;
- no range-based hunt slowdown below the normal planner safety ceiling;
- direct matrix gap, bounded last-track/odometry gap prediction, and the initial
  `30 mm` physical handoff threshold;
- no neutral command, route pause, PID reset, or payload-confirmed result at the
  matrix proximity handoff;
- route-blended positive-forward feed distance and interrupted-feed state;
- capture-aligned feed-corridor constraint when the saved route initially asks
  for reverse, a pivot, or excessive curvature;
- unexpected matrix disappearance cannot trigger handoff, while expected
  close-range disappearance can only use bounded armed prediction;
- presence and required sections of
  `RobotCode/docs/INTERNAL_WEIGHT_TOF_INTEGRATION_GUIDE.md`;
- no inductive/material-classifier dependency in the hunt build;
- cooperative nonblocking polling;
- final-writer rear and front safety gates;
- simulator/firmware sensor-count and geometry parity;
- 2.5D simulator height filtering for upper fan rays, weight cylinders, and
  sloped ramps; and
- telemetry visibility for every configured sensor.

## 13. Decisions required before implementation completion

1. Confirm the four front-fan yaws.
2. Confirm front matrix pitch, roll, and connector/grid orientation.
3. Document the assumed minimum height and geometry of an obstacle that can
   obstruct reverse, and characterize detection of that representative
   obstacle throughout the rear stopping envelope.
4. Confirm whether the declared `228.0 x 245.7 mm` footprint includes the front
   matrix housing, funnel, guards, brackets, and bumpers; expand or explicitly
   model any contact-capable protrusion.
5. Choose the three XSHUT channels and bus-local I2C addresses; reserve the
   unused fourth channel explicitly.
6. Determine matrix safety/perception row masks from recorded frames.
7. Calibrate rear stop/clear thresholds using measured stopping distance and
   sensor uncertainty.
8. Decide whether the front matrix remains supplemental blocking evidence or
   can prove known-clear central space based on status-characterization data.
9. Measure direct matrix gap versus true physical gap and confirm/calibrate the
   initial `30 mm` handoff, final-approach arm gap, maximum prediction distance,
   maximum prediction time, and odometry uncertainty.
10. Measure the maximum acceptable centre error and minimum uninterrupted
    forward-feed distance, initially `150 mm`, then characterize whether the
    funnel retains the weight during route-blended curvature and whether a
    weight against a wall has a collision-free approach/feed corridor.
11. Calibrate weight width/height, lower-to-upper depth separation, ramp-plane,
    static-motion, centre-deadband, prediction, forward-feed, and hunt timeout
    thresholds from saved raw frames and pickup runs.
12. Choose the opportunistic route-intercept detour envelope and define which
    typed post-handoff feed failures continue the route versus abandon the
    current pickup attempt.
13. Choose the separate follow-test standoff, scan/translation/turn caps,
    allowed locked-track speed, bounded reacquisition angle, and timeout.

Track unresolved calibration items explicitly in telemetry and test notes.
They do not create an additional direction-enable latch: runtime forward and
reverse availability is determined from the current configured sensor
evidence. Keep the matrix in supplemental-veto mode until a later design
decision explicitly changes its collision-model authority.
