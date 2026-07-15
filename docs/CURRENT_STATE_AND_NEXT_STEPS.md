# Current State And Next Steps

Status: current handoff after object detection/search work, the 2026-07-06/07
offset wall/gap navigation rework, and the 2026-07 full codebase audit.

This file records what the robot can do now, what is still only scaffold, and
the next smallest useful steps. Read it alongside `ROBOT_CODEBASE_AUDIT.md`,
`../README.md`, `../../HANDOFF.md`, and `../../NAVIGATION_DESIGN.md`.

P0 disposition: P0-01, P0-02, P0-07, and P0-08 are fixed in software, with
physical validation still applicable to the motion findings. P0-03 is partly
fixed and depends on real sensor coverage. P0-04 fake rear is intentionally
deferred; P0-05 sensor safety proof is open. P0-06 phase 1 is implemented, but
coherent sensor snapshots and physical watchdog timing remain follow-ups.
Operator decision, 2026-07-14: obstacle testing may continue with the current
`RANGE_FAKE_REAR` channel. Treat this as explicit temporary test scaffolding,
not proof of rear safety or competition readiness.

## 1. Current Firmware Baseline

- Active firmware: `RobotCode`.
- Board/build target: Teensy 4.0. Current verified upload path is Arduino CLI
  for `teensy:avr:teensy40`.
- Navigation baseline: accepted V7 local planner.
- Main control path remains single and scheduled. Phase-1 runtime sensor reads
  are nonblocking, with a motor lease and loop-deadline telemetry; coherent
  sensor snapshots and physical watchdog timing remain follow-ups:
  - `updateTOFSensors()`
  - `updateLocalMapFromSensors()`
  - odometry update
  - local planner update
  - motor controller update
  - Bluetooth telemetry/commands
- High VL53L0X fan remains the travel-safety authority.
- Object VL53L1X sensors are perception/candidate sensors only; they do not
  override high-fan safety.

Latest robot/source state, 2026-07-08:

- Firmware compiles successfully with Arduino CLI. Upload and physical
  validation of the latest obstacle changes are still pending.
- Last reported build label: `V7-local-planner`.
- Latest accepted offset wall/gap navigation evidence is `arn7`/`arn7d2`, not
  `rvs2`. `arn7d2` ended in `waypoint_reached` with no reverse recovery and
  approximate pose `x=1.977 y=0.053 theta=-28.94`.
- Last reported search scan setting: `searchTurnUs=280`.
- Normal global slow pivot pulses are back at `1800/1200 us`.
- Weight-search scan/confirm turns use their own runtime offset via
  `SEARCHTURN`; default is `280 us`.
- The robot was left stopped, disarmed, and motor-neutral.
- `RobocupSimulator` now has deterministic tests for G0/G5 wall bypass,
  offset wall/gap reverse survey, front-against-wall escape, and wide-panel
  bypass speed recovery. Current verification is Teensy compile PASS,
  simulator 25/25 PASS, Python 76/76 PASS, and `compileall` PASS.

## 2. Current Navigation State

Navigation is considered good enough to support object work, with the
2026-07-06/07 bounded side-escape rework as the current baseline. Do not
redesign navigation unless object integration or physical regression testing
exposes a direct problem.

Accepted navigation behaviour:

- Four high fan VL53L0X sensors are active and valid in open space.
- V7 local planner can drive point goals.
- Fake-rear-ToF reverse recovery is allowed for current obstacle testing by
  explicit operator decision. It is not proof of rear safety; log the fake-rear
  assumption and rear clearance on every physical run.
- Reverse recovery now exits through `post_reverse_escape` when a close
  inner/outer side fan ray remains after forward space returns.
- The reverse-survey branch (`rvs1`/`rvs2`) is historical only. `rvs2` reached
  `waypoint_reached` in serial logs, but physical observation showed multiple
  retries and scraping, so it is not accepted as the current baseline.
- The accepted offset wall/gap model is:
  `SIDE_ESCAPE -> ROUTE_REJOIN -> FINAL_POINT_APPROACH -> waypoint_reached`.
  Obstacle avoidance should be a temporary overlay that clears the obstacle and
  then returns control to the actual waypoint, not a sticky replacement route.
- Current source uses proactive side-escape memory, urgent escape-turn
  commitment, committed side-sign steering, longer post-reverse latching,
  corridor-squeeze suppression during active side escape, bounded route
  rejoin, restored finish/overshoot/missed guards, scoped point-alignment
  bypass, and a temporary side-escape rejoin speed cap.
- Accepted evidence after the rework:
  - `arn7`: `TEST AVOID 1.00` passed with `waypoint_reached`, no reverse
    recovery, final pose about `x=1.028 y=0.049 theta=-80.50`.
  - `arn7d2`: `TEST AVOID 2.00` passed with `waypoint_reached`, no reverse
    recovery, final pose about `x=1.977 y=0.053 theta=-28.94`.
- Do not reintroduce broad detour-plane completion. The `arn4b`
  route-line-detour finish ended around `x=0.948 y=0.463`, too far from the
  real waypoint.
- Current watch item: the post-clear movement may be slower than necessary
  because `PLANNER_SIDE_ESCAPE_REJOIN_MAX_SPEED_TPS` is applied while
  `sideEscapeRejoinActive(...)` is true. If it feels slow in physical testing,
  remove that condition from the cap and rerun the narrow offset wall/gap
  checks described in the handoff before expanding test coverage. Do not tune
  this with physical autonomous testing while P0-04/P0-05 remain open.
- P0-07 remediation removed `corridor_squeeze_straight` and the reverse
  front-footprint grace. Zero accepted rollouts now mean zero commanded motion.
- The simulator keeps contact-free traversal checks for `g0_right`, `g5`, and
  `wide_panel_bypass`; the front-against-wall case now safe-stops when strict
  footprint collision rejects reverse. `final_blocked_reached` remains a
  stop-only result inside the existing `160 mm` gate and requires a clear
  current inflated footprint. Physical upload/validation remains out of scope.
- 400 mm competition gap has one accepted physical pass.
- `TEST HUNT` can use the same point-goal planner path as `TEST GOTO`.
- `TEST DISARM` cancels test-owned goals and leaves motors neutral.

Current limitations:

- No real rear sensor; `RANGE_FAKE_REAR` is intentionally deferred test
  scaffolding. Current obstacle testing may use it by explicit operator
  decision, but it cannot close P0-04/P0-05.
- Encoder distance scale is probably not final.
- `TICKS_PER_METRE` is currently `9125.0` in code.
- Geometry-derived estimate from `H_Motor.pdf` plus the 20T 8M pulley is about
  `8287.5 counts/m`, assuming the current A-channel `CHANGE` interrupt counts
  both edges of the `663 PPR` gearbox output signal.
- Physical rough-distance samples suggested a higher practical value:
  - `TEST DRIVE 0.50`: `4034.5` average ticks over roughly `0.440 m`
    -> about `9169 ticks/m`.
  - `TEST DRIVE 1.00`: `8730` average ticks over roughly `0.960 m`
    -> about `9094 ticks/m`.
  - The working midpoint was set to `9125.0 ticks/m`.
- This encoder scale should still be refined with measured repeats before
  relying on long route accuracy.

## 3. Current Object Sensor Hardware

Object sensors are four VL53L1X ToFs in paired LOW/UPPER columns.

Robot-frame convention:

- `+X` forward.
- `+Y` robot-left.
- Left/right names are from the robot's perspective, not from looking at the
  robot from the front.

Current object sensor layout:

| Logical sensor | XSHUT | I2C | X mm | Y mm | Z mm | Yaw |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| `object_left_low` | 7 | `0x34` | 91.4 | +60.6 | 55.0 | -20 deg |
| `object_left_upper` | 5 | `0x35` | 91.4 | +60.6 | 120.0 | -20 deg |
| `object_right_low` | 6 | `0x36` | 91.4 | -60.6 | 55.0 | +20 deg |
| `object_right_upper` | 4 | `0x37` | 91.4 | -60.6 | 120.0 | +20 deg |

Verified:

- All four object sensors enumerate on their XSHUT pins and unique addresses.
- `TEST OBJECT` prints role, XSHUT, address, distance, validity, stale,
  connected, age, timeout count, invalid count, range status, signal, and
  ambient.
- Moving upper sensors to `120 mm` made the LOW-valid/UPPER-clear classifier
  work for a centered standing weight.

## 4. Current Object Classifier

Current constants:

```cpp
OBJECT_TOF_ENABLED = true
OBJECT_TOF_ROI_WIDTH = 16
OBJECT_TOF_ROI_HEIGHT = 4
OBJECT_TOF_ROI_CENTER_SPAD = 199
OBJECT_TOF_VALID_MIN_MM = 40
OBJECT_TOF_VALID_MAX_MM = 4000
OBJECT_CANDIDATE_MIN_MM = 60
OBJECT_CANDIDATE_MAX_MM = 1500
OBJECT_UPPER_CLEAR_DELTA_MM = 80
OBJECT_UPPER_STRONG_SIGNAL_MCPS = 4.0
OBJECT_CANDIDATE_CONFIRM_READS = 3
OBJECT_TARGET_STALE_TIMEOUT_MS = 1000
```

Object ToF FOV notes:

- The object sensors currently use a centered `16 x 4` ROI: full horizontal
  width and minimum vertical height.
- For the mounted VL53L1X boards, `width` is left-right across the readable
  sensor label and `height` is up-down. The centered `16 x 4` ROI is intended
  to reduce floor/chassis/vertical clutter while keeping left-right object
  coverage.
- If the ROI centre is shifted later with `setROICenter()`, remember the
  VL53L1X lens inverts the image; centered sizing does not need that mental
  inversion.

Current search/hunt constants:

```cpp
OBJECT_PICKUP_OVERSHOOT_MM = 150.0
WEIGHT_SEARCH_SWEEP_DEG = 30.0
WEIGHT_SEARCH_STANDOFF_M = 0.25
WEIGHT_INTERRUPT_COOLDOWN_MS = 1000
WEIGHT_SEARCH_CONFIRM_TURN_MIN_DEG = 5.0
WEIGHT_SEARCH_CONFIRM_TURN_MAX_DEG = 35.0
WEIGHT_SEARCH_HUNT_TIMEOUT_MS = 5000
WEIGHT_SEARCH_MAX_ROUTE_DEVIATION_M = 0.85
```

Classifier shape:

- LOW valid and near, UPPER invalid/clear/weak -> `weight_sized`.
- LOW valid and UPPER valid with strong signal near same distance ->
  `tall_obstacle`.
- No LOW near return -> `none`.
- Mixed unclear geometry -> `unknown`.
- Weak upper edge returns below `4.0 MCPS` are treated as clear.

Verified behaviour:

- Centered/intersection weight after upper sensor move:
  - both LOW sensors valid;
  - both UPPER sensors clear;
  - `weight_sized`, confirmed.
- Farther placement after widening range:
  - dry target around `range_mm=368-371`;
  - `weight_sized`, confirmed;
  - fresh target around `world_x_m ~= 0.593`;
  - weak upper returns no longer blocked detection.

Known classifier risks:

- `OBJECT_UPPER_STRONG_SIGNAL_MCPS = 4.0` was chosen from a small number of
  live samples. It needs more data against walls, ramps, tall panels, and
  awkward weight angles.
- The classifier has not yet been tested against a plastic dummy, fallen
  weight, wall-adjacent weight, or opponent/robot-shaped panel.
- Single-side detections work mathematically, but centering/steering confidence
  still needs more lateral-offset trials.

## 5. Current Hunt Target Integration

Navigation uses one turn-sign contract throughout current firmware and tools:
`+X` is robot-forward, `+Y` is robot-left, and positive yaw, heading, chassis
turn, and planner curvature are CCW/left. Forward-positive wheels use
`left=forward-turn`, `right=forward+turn`, so positive angular velocity is
`(right-left)/trackWidth`. The installed BNO raw yaw is CW/right-positive and
is converted once by `navigationHeadingDeg()`; mapping, odometry, planning,
stuck checks, and diagnostics use that navigation heading.

Implemented:

- Confirmed `weight_sized` candidates publish `objectTargetEstimate`.
- The target is computed from valid LOW sensor ray endpoints.
- Source mask:
  - `1` = left LOW used.
  - `2` = right LOW used.
  - `3` = both LOW sensors used.
- The target is transformed into world coordinates using current odometry pose
  and heading.
- `OBJECT_PICKUP_OVERSHOOT_MM = 150.0` biases the target forward in the robot
  frame. It is pickup carry-through for weight slide, not a precise lateral
  target.
- `NAV_OWNER_TEST_HUNT` applies a small speed boost in the final `150 mm`
  carry-through zone, bounded by normal forward-space safety.
- `NAV_OWNER_TEST_HUNT` completes near the biased target in a wider pickup zone
  so it does not do the normal point-goal heading cleanup after contact.
- `TEST HUNT TARGET` prints the target without moving.
- `TEST HUNT` requires `TEST ARM`, locks the fresh target at command time, and
  starts:

```cpp
startNavigationPoint(targetWorldX, targetWorldY, NAV_OWNER_TEST_HUNT)
```

Verified:

- `TEST HUNT` refuses to move while test motion is disarmed.
- Close/centered pickup run completed by itself and stopped in `END_MATCH`.
- Farther pickup run after the 500 mm gate and weak-upper filter completed by
  itself and stopped in `END_MATCH`.
- During approach the object can disappear from object ToFs, consistent with
  the weight moving under/into the pickup zone. This is acceptable because the
  target is locked at command time.

Current pickup bias:

```cpp
OBJECT_PICKUP_OVERSHOOT_MM = 150.0
```

- Documentation now treats `150 mm` as the current intended test value. Keep it
  visible because it is a physical pickup tuning constant, not a classifier
  threshold.
- The overshoot is forward-only in the robot frame.
- The final carry-through zone gets a hunt-only speed boost from the normal
  `baseTargetSpeed` up to at most `2850 ticks/s`.
- The latest far-range run is consistent with this larger pickup bias and
  completed successfully.

## 6. Current Operator Commands

No-motion object checks:

```text
TEST OBJECT
TEST HUNT TARGET
STATUS
```

Guarded pickup run:

```text
TEST HUNT TARGET
TEST ARM
TEST HUNT
STATUS
TEST DISARM
```

Guarded waypoint-style search test:

```text
SEARCHTURN 280
TEST ARM
TEST SEARCH
STATUS
TEST DISARM
```

Turn/search tuning helpers:

```text
SEARCHTURN <120-300>
SEARCHTURN RESET
TEST TURNLADDER is disabled; do not use it as a movement command.
```

Safety expectations:

- `TEST HUNT` must refuse if `TEST ARM` has not been sent.
- `TEST DISARM` must stop/cancel test-owned motion and leave motors neutral.
- High fan safety remains active during `TEST HUNT`.

## 7. What Is Not Implemented Yet

Not implemented:

- `APPROACH_OBJECT` state in the main state machine.
- Payload/intake confirmation event.
- Count of carried target weights.
- Dummy/target classification.
- Return-home trigger based on payload/time.
- Real rear sensor.
- Broad detection dataset/confusion matrix.
- Separate left/right encoder scale calibration.

Current object hunting has three paths: guarded manual tests (`TEST HUNT` and
`TEST SEARCH`), explicit route waypoints with action `SEARCH`, and
opportunistic route-only interrupts on fresh confirmed `weight_sized` targets.
It is not yet a full autonomous match strategy.

## 8. Immediate Priority List

### P0: Validate Search-Hunt Completion Fix

Goal: once the robot has driven over the weight during a search-owned hunt, the
planner must finish the pickup instead of entering a final point-align turn.

Known evidence:

- A visible-weight `TEST SEARCH` found the target, started hunt, drove through
  the pickup/boost phase, but then selected `point_align_turn_fast` and ended
  around `-44.6 deg`.
- `TEST HUNT` already has a pickup-zone finish path, so the likely fix is in
  the owner/finish ordering for `NAV_OWNER_OBJECT_HUNT` after the target has
  passed behind or under the robot.

Code status, 2026-07-05:

- `LocalPlanner.cpp` now checks `huntPickupZoneReached()` before the generic
  point-goal alignment turn can run.
- It also suppresses point alignment during the final hunt carry-through band
  while the robot is still roughly aligned with the hunt route.
- Arduino CLI compile for `teensy:avr:teensy40` passed.
- Physical upload/validation is still pending.

Pass:

- Search-owned object hunts emit pickup-zone completion and stop or resume
  route as appropriate.
- They do not try to tidy final heading after the weight is under/behind the
  chassis.
- Motors end neutral on both success and failure.

### P1: Re-run TEST SEARCH With Final Search Turn

Goal: confirm the accepted search-specific turn speed works with and without a
visible target.

Current settings:

- Normal slow pivots: `1800/1200 us`.
- Search scan/confirm offset: `SEARCHTURN 280` by default.
- Raw search pulse pair at that offset: `1780/1220 us`.

No-weight run:

```text
ZERO
SEARCHTURN RESET
TEST ARM
TEST SEARCH
STATUS
TEST DISARM
```

Then repeat with a visible weight roughly `400-450 mm` ahead and slightly
off-centre.

Pass:

- No-weight case sweeps centre, `+30 deg`, `-60 deg`, returns toward centre,
  and ends/resumes cleanly.
- Visible-weight case performs confirm turn if needed, locks a fresh target,
  hunts once, and completes without point-align cleanup.

### P2: Route SEARCH And Interrupt Test

Goal: prove route-level search works both as an explicit `SEARCH` waypoint
and as a confirmed-weight interrupt during normal waypoint travel.

Actions:

- Put one explicit `SEARCH` action into a short route waypoint.
- Use a known open area with one weight near that waypoint.
- Confirm the robot drives to a nominal `250 mm` standoff, aligns to the
  bearing from its current pose to the search waypoint, then scans
  `+/-WEIGHT_SEARCH_SWEEP_DEG` around that bearing.
- Put a confirmed visible weight along a normal route segment and confirm the
  route is interrupted, the weight is hunted once, and the same waypoint is
  reassigned afterward.

Pass:

- Explicit `SEARCH` no target: the weight-location waypoint is skipped and
  the route continues.
- Explicit `SEARCH` target found: one pickup attempt, then the route
  continues to the next waypoint.
- Opportunistic route target found: one pickup attempt, then the original
  waypoint is resumed.
- Opportunistic confirm-loss: original waypoint is resumed after cooldown.
- Unsafe scan: route skips or safe-stops with telemetry.
- Hunt timeout/stuck/deviation: motors neutral and visible safe-stop for
  operator inspection.

### P3: Build A Small Detection Dataset

Goal: stop tuning thresholds from one or two successes.

Test matrix:

- Standing target weight:
  - centered;
  - left offset;
  - right offset;
  - near, mid, far within the `60-1500 mm` candidate range.
- Empty floor.
- Wall/panel.
- Ramp or sloped surface if available.
- Fallen/tilted weight.
- Plastic dummy if available.
- Weight near wall/background.

For each sample, log:

```text
TEST OBJECT
TEST HUNT TARGET
physical placement note
whether target should be hunted
```

Pass:

- Standing target weights reliably produce `weight_sized`.
- Tall panels/walls reliably produce `tall_obstacle` or no target.
- False positives are understood before opportunistic route interruption is
  enabled outside explicit `SEARCH` waypoints.

### P4: Far And Offset Pickup Repeatability

Goal: prove `TEST HUNT` works before autonomous route interruption.

Runs:

- 5 centered pickups.
- 5 left-offset pickups.
- 5 right-offset pickups.
- At least 3 farther detections beyond the early `400-500 mm` range.

Record:

- target range;
- source mask;
- final odometry;
- physical pickup success/failure;
- whether the robot needed a stop.

Pass:

- High success rate without contact with non-weight obstacles.
- No unsafe planner stops or oscillation.
- Source mask and lateral target make sense.

### P5: Encoder Distance Calibration

Goal: improve odometry before long autonomous search routes.

Known geometry:

- Motor PDF: `663 PPR for gearbox shaft`.
- Current code counts A-channel `CHANGE`, using B for direction.
- Belt: `880-8M`, so pitch is `8 mm`, belt teeth count is `110`.
- Drive pulley: `20 teeth`.
- Pulley travel per revolution: `20 * 8 = 160 mm`.
- Likely geometry estimate: `1326 / 0.160 = 8287.5 counts/m`.
- Current code: `9125.0 counts/m`.

Actions:

- Drive a measured 1 m straight line several times.
- Record encoder deltas and actual measured distance.
- Compute left and right ticks/m separately.
- Decide whether to replace the single `TICKS_PER_METRE` or introduce
  left/right scale factors.

Pass:

- Measured straight distance error is reduced.
- Heading drift does not get worse.
- Object target world coordinates stay more reliable during longer approaches.

### P5b: Slow Pivot Retune Outcome

Goal: keep the final slow-turn decision visible for future tuning.

The raw ladder command is disabled because blocking pulses cannot be
continuously supervised. Historical ladder measurements are retained only as
calibration notes, not as a runnable current command.
- Global slow turns were restored to `1800/1200`.
- Search scan/confirm turns now use the runtime `SEARCHTURN <offset_us>`
  setting, defaulting to `280 us`, so search speed can be tested without
  changing normal planner turns.
- Physical `TEST SEARCH` comparison found `200 us` too weak/draggy, `240 us`
  only okay, and `280 us` the only acceptable search scan speed.

### P6: Harden Route Search V2

Goal: route navigation can search before known weight locations, interrupt for
confirmed weights mid-route, pick up one weight, and resume correctly.

Implemented:

- Waypoint action `SEARCH` is treated as a weight-location waypoint.
- The robot drives to a nominal `WEIGHT_SEARCH_STANDOFF_M = 0.25` standoff
  before the waypoint. If already inside that distance, it starts search
  immediately.
- At standoff, it aligns to the bearing from the current robot pose to the
  search waypoint, then checks centre, sweeps `+WEIGHT_SEARCH_SWEEP_DEG`,
  sweeps `-2 * WEIGHT_SEARCH_SWEEP_DEG`, then returns toward centre.
- `WEIGHT_SEARCH_SWEEP_DEG = 30.0` is the global first-pass tuning value.
- A confirmed fresh `weight_sized` target first triggers a confirm turn toward
  the robot-frame object bearing, then refreshes the object estimate while
  stopped before starting `NAV_OWNER_OBJECT_HUNT`.
- `NAV_OWNER_OBJECT_HUNT` uses the same forward-only pickup carry, boost, and
  hunt finish behavior as `TEST HUNT`, but does not count as a test goal and
  does not end the match on success.
- During normal route travel, a fresh confirmed `weight_sized` target can
  interrupt the active route goal. The robot cancels that route goal, confirms
  and hunts the target, then resumes the same `currentWaypointIndex`.
- `TEST SEARCH` requires `TEST ARM` and runs the same search sequence using the
  current robot pose and heading; it does not use the 250 mm route standoff.
- Route search attempts at most one pickup per explicit search waypoint.
- If no confirmed target is found at a `SEARCH` standoff scan, the route
  skips that weight-location waypoint and advances to the next waypoint.
- If scan turns are unsafe/blocked, search is skipped and the route advances.
- If autonomous hunt times out, exceeds `WEIGHT_SEARCH_MAX_ROUTE_DEVIATION_M`,
  or fails safety/stuck checks, the robot safe-stops.

Rules:

- Do not let object sensors weaken high-fan travel safety.
- Do not refresh the target after the weight disappears under the intake.
- Add a timeout and max distance for every approach.

Pass:

- Route is interrupted only for fresh confirmed `weight_sized` targets.
- Robot completes pickup, resumes the correct waypoint, or cleanly aborts.
- No runaway if candidate disappears before the target is locked.

### P7: Payload / Intake Confirmation

Goal: know whether pickup actually happened.

Possible signals:

- intake switch;
- load cell;
- current spike;
- inductive sensor;
- operator-marked event during early testing.

Interface should be event-based:

```text
OBJECT_FULLY_INGESTED
OBJECT_LIKELY_TARGET
OBJECT_LIKELY_DUMMY
OBJECT_CLASSIFICATION_UNCERTAIN
PAYLOAD_FAULT
```

Pass:

- Navigation can stop after approach and wait for payload result.
- Strategy can decide keep/reject/continue.

### P8: Match Strategy

Goal: turn pickup into an actual competition behavior.

Actions:

- Define coverage/search route.
- Define carried weight count.
- Define return-home deadline.
- Define unload behavior.
- Define dummy rejection strategy.

Pass:

- Robot can search, collect, return, and unload within match constraints.

## 9. Recommended Next Work Order

1. Upload the 2026-07-05 route-search build.
2. Re-run `TEST SEARCH` at default `SEARCHTURN 280` with no weight and with one
   visible weight to confirm manual search still works.
3. Put action `SEARCH` into a short route and prove standoff alignment,
   no-target skip, and one-target pickup.
4. Place a confirmed weight on a normal route segment and prove opportunistic
   interrupt, pickup, and original-waypoint resume.
5. Collect a small false-positive dataset with `TEST OBJECT` and
   `TEST HUNT TARGET`.
6. Run 5-10 repeated pickup tests from centre/left/right offsets.
7. Calibrate encoder counts per metre over a measured 1 m run.
8. Add payload confirmation.
9. Add match strategy.

## 10. Do Not Do Yet

- Do not make object sensors travel-safety sensors.
- Do not loosen high-fan planner safety to make pickup easier.
- Do not expand opportunistic interruption outside normal route travel until
  false positives have been tested.
- Do not retune motor bases from a single pickup run.
- Do not change navigation architecture unless a reproduced object-hunt failure
  requires it.
