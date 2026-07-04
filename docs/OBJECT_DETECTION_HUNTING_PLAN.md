# Object Detection And Hunting Plan

Status: implementation plan and handoff after object sensing, `TEST HUNT`,
`TEST SEARCH`, and route-search V1 were added.

The goal is to add object detection, approach, pickup handoff, and hunting
behaviour without weakening the current VL53L0X navigation fan or replacing the
V7 local planner. This file is the decision guide for future work: if a change
does not move one of these success gates forward, it should wait.

## Success Definition

Object detection and hunting is successful when the robot can:

1. Keep the current V7 travel safety behaviour intact.
2. Read four VL53L1X object ToFs as a separate subsystem:
   `object_left_low`, `object_left_upper`, `object_right_low`,
   `object_right_upper`.
3. Produce reliable, logged weight-sized candidates from LOW/UPPER geometry.
4. Reject common false positives: wall, ramp, robot body, fallen weight, empty
   floor, stale samples, and broad high-fan obstacles.
5. Interrupt a coverage route only after a persistent candidate is confirmed.
6. Enter an explicit, slow, time-limited `APPROACH_OBJECT` mode.
7. Centre the intake using left/right object columns without oscillation.
8. Hand final pickup to the intake/payload subsystem through events, not direct
   motor or arm side effects inside navigation.
9. Resume search, reject, or return home based on payload events, target count,
   match time, and confidence.

## Architecture Rules

- The four VL53L0X high fan sensors remain the authority for travel safety.
- The four VL53L1X object sensors generate object candidates only.
- Object approach may slow or pause navigation, but it must not silently disable
  front-blocked, diagonal-clearance, stale-sensor, or stuck protection.
- Any mode that allows closer motion toward an object must be explicit,
  bounded by timeout, and visible in telemetry.
- Return-signal features are confidence evidence only. Geometry and internal
  payload sensing decide action.
- Sensor bring-up must preserve a reduced configuration where V7 navigation can
  run without object sensors connected.

## Implementation Milestones

### M0: Disabled Base Scaffold - complete

Success:

- Firmware compiles with object architecture present.
- `OBJECT_TOF_ENABLED` can hold the subsystem disabled until hardware is ready.
- Startup and V7 tests do not wait for missing object hardware.
- `TEST OBJECT` reports that object sensing is disabled/not configured.
- Plan and README clearly describe how to enable the subsystem later.

### M1: Object Sensor Bring-Up

Success:

- Four VL53L1X objects exist in firmware.
- XSHUT pins, addresses, timing budget, distance mode, and sample cadence are
  configured in one visible block.
- Each sensor reports distance, validity, age, timeout count, signal, ambient,
  and LOW/UPPER role.
- One failed object sensor is degraded telemetry, not proof of clear space.

Required physical measurements:

- X/Y/Z offset from the wheel-midpoint frame.
- LOW or UPPER height.
- Left/right column sign.
- Inward yaw direction.
- XSHUT pin and I2C address.
- Any body, intake, wiring, or carried-object obstruction in the field of view.

Current measured object-ToF layout:

| Sensor | XSHUT | X mm | Y mm | Z mm | Yaw |
| --- | ---: | ---: | ---: | ---: | ---: |
| `object_left_upper` | 5 | 91.4 | +60.6 | 120.0 | -20 deg |
| `object_right_upper` | 4 | 91.4 | -60.6 | 120.0 | +20 deg |
| `object_left_low` | 7 | 91.4 | +60.6 | 55.0 | -20 deg |
| `object_right_low` | 6 | 91.4 | -60.6 | 55.0 | +20 deg |

The view is expected to be clear of intake, wires, robot body, and carried
weights. First enabled sanity check connected all four sensors. These labels
were captured before the front-view left/right correction:

- `object_left_low`: XSHUT6, address `0x34`, connected, last status `2`
  (`SignalFail`).
- `object_left_upper`: XSHUT4, address `0x35`, connected, last status `4`
  (`OutOfBoundsFail`).
- `object_right_low`: XSHUT7, address `0x36`, connected, last status `2`
  (`SignalFail`), non-valid returns around `179-208 mm`.
- `object_right_upper`: XSHUT5, address `0x37`, connected, last status `4`
  (`OutOfBoundsFail`).

Next M1 success gate is to place known targets in the view and get valid
`range_status=0` returns from each channel.

Follow-up sample with a weight placed about `200 mm` from the right side:

- Candidate changed to `tall_obstacle`, range about `255-256 mm`.
- `object_left_low` returned valid `255-256 mm`, `range_status=0`, strong
  signal around `18 MCPS`.
- `object_left_upper` returned valid `261-270 mm`, `range_status=0`, signal
  around `2.2-2.5 MCPS`.
- `object_right_low` saw non-valid returns around `155-245 mm`,
  `range_status=2` (`SignalFail`).
- `object_right_upper` remained invalid, `range_status=2/4`.

This proves target returns can be read, but the logical left/right mapping or
the physical target placement needs one more isolation test before hunting
logic trusts direction.

Correction: the original left/right notes were taken while looking at the robot
from the front. Firmware now uses robot-frame left/right, so the live logical
XSHUT mapping is `left_upper=5`, `right_upper=4`, `left_low=7`, and
`right_low=6`.

Post-correction verification:

- Firmware compiled and uploaded with the swapped robot-frame mapping.
- `TEST OBJECT` table confirmed `object_left_low` on XSHUT7, `object_left_upper`
  on XSHUT5, `object_right_low` on XSHUT6, and `object_right_upper` on XSHUT4.
- Current target sample detected confirmed `weight_sized` at about `284-285 mm`
  with `direction_hint=-1`, driven by the corrected `object_left_low` channel.

Intersection placement sample:

- Target at the apparent left/right beam intersection produced strong valid
  low returns on both sides: about `138-148 mm`.
- Upper channels were also valid on most samples: about `171-179 mm`, with one
  `WrapTargetFail` sample on `object_left_upper`.
- Classifier returned `tall_obstacle`, not `weight_sized`, because the
  low-hit/upper-clear height gate was not satisfied.
- This confirms the intersection can be seen strongly, but a valid collection
  target still needs the upper beams clear or a revised height model.

Upper sensors moved to `120 mm` from floor:

- Firmware geometry/docs updated and uploaded with upper Z = `120.0 mm`.
- Repeating the centered/intersection-style sample now produced confirmed
  `weight_sized`, `direction_hint=0`, range about `169-173 mm`.
- Both low sensors were valid (`range_status=0`) with strong return signal.
- Both upper sensors stayed invalid/clear (`SignalFail`/`OutOfBoundsFail`).
- This satisfies the stationary centered weight signature and keeps the
  low-hit/upper-clear classifier viable with the revised physical layout.

Initial local-planner target integration:

- Confirmed `weight_sized` candidates now publish an `objectTargetEstimate`.
- The target is computed from valid LOW ray endpoints in the robot frame:
  left LOW contributes bit `1`, right LOW contributes bit `2`, and a centered
  target normally has source mask `3`.
- The target is transformed into odometry/world coordinates using current
  heading and pose, then exposed in `STATUS`, CSV, `TEST OBJECT`, and
  `TEST HUNT TARGET`.
- The published pickup target is intentionally biased `150 mm` forward from the
  estimated weight point in the robot frame. The bias is pickup carry-through,
  not a precise lateral target.
- `TEST HUNT` adds a small speed boost in the final `150 mm` carry-through
  zone after the estimated weight point. The boost is still bounded by the
  normal forward observation/braking cap.
- `TEST HUNT` uses a pickup-zone finish rule near the biased target, so it can
  complete without performing the normal point-goal final heading cleanup.
- Candidate range gate was widened during testing. Current code treats object
  readings as valid from `40-4000 mm`, while confirmed hunt candidates must be
  in the `60-1500 mm` range.
- Upper height veto now requires a valid upper range plus at least `4.0 MCPS`
  return signal. Weak upper edge returns are treated as clear so they do not
  turn a good low-weight signature into `tall_obstacle`.
- `TEST HUNT` requires `TEST ARM`, locks the fresh biased target at command
  time, and starts
  `startNavigationPoint(targetWorldX, targetWorldY, NAV_OWNER_TEST_HUNT)`.
- Dry-run verification with a centered confirmed weight estimated about
  `250 mm` forward and `0 mm` lateral (`world_x_m ~= 0.250`,
  `world_y_m ~= 0.000`, source mask `3`).
- Safety verification: `TEST HUNT` refused to move while test motion was
  disarmed.
- First armed `TEST HUNT` run locked a centered target at about
  `world_x_m=0.252`, `world_y_m=0.000` from pose zero. The local planner drove
  under `NAV_OWNER_TEST_HUNT`, selected safe arcs, completed the point goal, and
  entered `END_MATCH` without an external stop. Final odometry was about
  `x=0.217`, `y=-0.009`, within the point-goal tolerance. Object telemetry was
  `none` after the approach, consistent with the weight disappearing under the
  pickup/intake zone.
- After widening the candidate range and filtering weak upper returns, a
  far-range `TEST HUNT` locked a target at about `world_x_m=0.593`,
  `world_y_m=0.003`. The planner drove straight, completed on its own, and
  stopped at about `x=0.555`, `y=0.009`; object telemetry again went to `none`
  after the approach.

Opposite-side follow-up sample:

- Candidate changed to confirmed `weight_sized`, range about `211-214 mm`.
- `object_left_low` returned valid `211-212 mm`, `range_status=0`, signal
  around `14.5-19.8 MCPS`.
- `object_right_low` returned valid `214 mm`, `range_status=0`, signal around
  `15.4-16.0 MCPS`.
- Both upper sensors stayed invalid/clear (`SignalFail` or `WrapTargetFail`),
  which matches the intended low-hit/upper-clear weight profile.
- Direction hint was `0` because both low channels saw the target; direction
  needs narrower placement, differential signal, or a steering scan before it
  can reliably choose left/right.

### M2: Stationary Candidate Classification

Success:

- `TEST OBJECT` prints object-column rows and an object-candidate summary.
- Candidate generation requires several consistent frames.
- LOW near plus UPPER clear/far can become `weight_sized_candidate`.
- LOW near plus UPPER near becomes `tall_obstacle_like`.
- Clear/invalid/stale cases stay `none` or `unknown`.
- Tests cover target weight, plastic dummy, wall, robot-shaped panel, fallen
  weight, empty floor, and object against wall.

### M3: Logged Detection Dataset

Success:

- Logs include raw object rows and candidate events.
- At least 50 to 100 valid samples are collected per object/range/angle/sensor
  combination before thresholds are trusted.
- Evaluation is split by physical object, not random samples from the same
  object.
- False positive rate on walls and ramps is low enough for route interruption.

### M4: Hunt Interruption

Success:

- `FOLLOW_PATH` can run a fixed coverage route.
- Waypoint action `SEARCH` treats the waypoint as a likely weight location.
  The robot drives to a nominal `250 mm` standoff before the waypoint, aligns
  to the waypoint bearing, then runs a short
  `WEIGHT_SEARCH_SWEEP_DEG = 30 deg` left/right scan.
- During normal route travel, a fresh confirmed `weight_sized` target can
  interrupt the active route waypoint. After a successful hunt, the same
  `currentWaypointIndex` is resumed.
- A confirmed fresh `weight_sized` target first gets a short confirm turn
  toward the robot-frame object bearing, followed by a stopped refresh before
  `NAV_OWNER_OBJECT_HUNT` starts.
- `NAV_OWNER_OBJECT_HUNT` uses the same pickup carry-through and boost as
  `TEST HUNT`, but route state resumes after success.
- `TEST SEARCH` runs the same scan/hunt sequence from the current robot pose
  after `TEST ARM`; it remains current-heading based and does not use route
  standoff alignment.
- Search scan/confirm turns use `NAV_OWNER_WEIGHT_SCAN` and the runtime
  `SEARCHTURN` offset instead of changing normal planner slow turns. Final
  accepted default is `280 us`, producing `1780/1220 us` pulses; allowed range
  is `120-300 us`.
- Normal global slow turns remain `1800/1200 us`.
- If no confirmed target is found at an explicit `SEARCH` standoff, or a
  scan turn is unsafe, the route advances to the next waypoint.
- If an opportunistic interrupt loses the target during confirmation, the
  original waypoint is resumed after cooldown.
- If the autonomous hunt times out, strays beyond the configured deviation
  limit, or fails planner safety/stuck checks, the robot safe-stops.

Current M4 validation item:

- A visible-weight `TEST SEARCH` successfully detected and approached the
  target, but after the pickup/boost phase it selected `point_align_turn_fast`
  and ended around `-44.6 deg`.
- On 2026-07-05, `LocalPlanner.cpp` was changed so `huntPickupZoneReached()`
  is checked before generic point-goal alignment, and point alignment is
  suppressed during the final hunt carry-through band while the robot is still
  roughly aligned with the hunt route.
- Arduino CLI compile for `teensy:avr:teensy40` passed after the 2026-07-05
  hunt-finish, route-interrupt, and 250 mm standoff changes. Upload and
  physical validation are still pending.

### M5: Slow Approach And Centre

Success:

- Approach speed is capped below normal route speed.
- Left/right columns steer toward intake centre.
- The controller handles single-column, dual-column, and range-disagreement
  cases.
- High-fan safety and stuck detection still stop the robot.
- Physical tests from multiple lateral offsets centre without contact or
  oscillation.

### M6: Payload Handoff

Success:

- Intake/payload reports events such as `OBJECT_FULLY_INGESTED`,
  `OBJECT_LIKELY_TARGET`, `OBJECT_LIKELY_DUMMY`,
  `OBJECT_CLASSIFICATION_UNCERTAIN`, `EJECTION_COMPLETE`, and `PAYLOAD_FAULT`.
- Chassis motion pauses during intake/ejection.
- Navigation resumes only after completion or timeout.
- Dummy rejection and target retention are strategy decisions, not raw sensor
  branches.

### M7: Match Strategy

Success:

- Search route, return-home trigger, and unload timing are strategy parameters.
- The robot avoids ending the match with more than three target weights onboard.
- Return home is triggered by time remaining, carried payload, confidence loss,
  or repeated recovery.
- Delivered target weights are prioritised when time allows.

## Next-Step Heuristic

When deciding the next step, choose the smallest action that advances the first
incomplete milestone. Prefer this order:

1. Make the subsystem observable.
2. Make the data trustworthy.
3. Make candidate decisions conservative.
4. Add slow motion around a single confirmed object.
5. Integrate payload and strategy.

Do not tune hunting behaviour from a single physical success. Save the log,
record the physical observation, and repeat enough to know whether it was a
capability or a lucky pass.
