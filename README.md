# RobotCode

Current RoboCup robot firmware for Teensy 4.0.

V7 uses a scheduled, safety-supervised local planner. Runtime sensor polling is
phase-1 nonblocking, with a motor command lease and loop-deadline telemetry;
coherent sensor snapshots and physical watchdog timing remain follow-ups. The
four forward navigation ToFs are VL53L0X sensors and build a rolling local
confidence map plus thresholded persistent arena memory; a footprint-aware
receding-horizon controller selects a safe
differential-drive arc toward the active waypoint.  There is no fixed
reverse/turn/bypass/rejoin script and no outer-fan wall-follow fallback. The
four-ray front fan has no true side coverage. Reverse safety uses three
independent rear VL53L1X channels; the front SEN0628 matrix supplies
supplemental collision vetoes and weight perception. Invalid, stale, or blind
direction-relevant safety evidence is never treated as clear.

Read [ROBOT_CODEBASE_AUDIT.md](docs/ROBOT_CODEBASE_AUDIT.md) before planning
new navigation or mission work. It records the full 2026-07 audit, including
the stop-ship safety findings that supersede the older navigation test plans.

P0 status: P0-01 turn convention, P0-02 motion authority/disarm, P0-07 hard
collision override, and P0-08 field-GOTO command suppression are fixed in
software. P0-03 is partially fixed and still depends on real sensor coverage.
P0-04's temporary rear channel has been removed in software; physical rear
coverage and stop/clear calibration remain open P0-05 evidence. P0-06 phase 1
is implemented, with physical watchdog timing still to follow. Do not treat
simulator results as proof of rear safety or competition readiness.

The migration baseline and latest validation results are recorded in
[FRONT_MATRIX_THREE_REAR_IMPLEMENTATION_STATUS.md](docs/FRONT_MATRIX_THREE_REAR_IMPLEMENTATION_STATUS.md).

The field GOTO desktop UI is preview-only. Field clicks do not send `TEST ARM`
or `TEST GOTO`; the SE(2) transform, status preflight, bounds preview, and
explicit confirmation flow are not yet implemented.

## Read First

- [Current state and next steps](docs/CURRENT_STATE_AND_NEXT_STEPS.md)
- [Robot codebase audit](docs/ROBOT_CODEBASE_AUDIT.md)
- [Code cleanup record and LocalPlanner split plan](docs/CODE_CLEANUP_AND_LOCAL_PLANNER_SPLIT.md)

## Files

- `RobotCode.ino` - Arduino setup/loop only.
- `Robot.h` - shared hardware/runtime globals and function prototypes.
- `RobotTypes.h` - shared enums and structs.
- `RobotConfig.h` - calibration values, pins, geometry, timing, and planner constants.
- `Navigation.h` - the small mission-facing driving API.
- `NavigationAdmin.h` and `NavigationTest.h` - operator/system and simulator
  adapters kept out of normal mission code.
- `src/core/` - shared runtime storage, helpers, and controller scheduling.
- `src/mission/` - mission controller, the sole route owner, and weight search.
- `src/motion/` - encoder, odometry, progress, and final motor-output code.
- `src/sensors/` - navigation/object/rear ranging and IMU code.
- `src/operator/` - Bluetooth commands, telemetry, and stationary diagnostics.
- `src/navigation/Navigation.cpp` - goal/result façade and private goal
  storage.
- `src/navigation/NavigationController.cpp` - point/turn orchestration and final
  authorized planner command publication.
- `src/navigation/PlannerContext.cpp/.h` and `PlannerTypes.h` - the single static owner and
  internal data types for mutable planning state.
- `src/navigation/PlannerMap.cpp/.h` - rolling confidence map and persistent arena evidence.
- `src/navigation/PlannerCollision.cpp/.h` - footprint, clearance, and turn-sweep queries.
- `src/navigation/ObstacleContext.cpp/.h` - retained obstacle envelope and local bypass goal.
- `src/navigation/ForwardTrajectoryPlanner.cpp/.h` - forward arc rollout, rejection, scoring,
  speed capping, and cooperative planning epochs.
- `src/navigation/RecoveryPlanner.cpp/.h` - evidence-driven reverse and opt-in emergency
  scan/relocate/retry behavior.
- `src/navigation/PlannerProgress.cpp/.h` - route-frame progress helpers.
- `src/navigation/PlannerDebug.cpp/.h` - read-only diagnostics for Bluetooth and the simulator.
- `src/operator/AvoidanceDiagnostics.cpp` - stationary fan-clearance diagnostics for
  `TEST SIDE`; it is not an autonomous planner.
- `src/motion/MotorControl.cpp` - the single motor-output owner and wheel-speed PID.
- `src/core/Helpers.cpp` - angle wrapping, cumulative encoder control snapshots, count
  reads, and pose print.

## Control Flow

The active V7 firmware has one scheduled control path:

```text
RobotCode.ino loop()
  -> handleBluetoothCommands()
  -> updateMissionController()
       -> RouteMission (owns the route, index, pauses, and actions)
            -> WeightSearch (only while search is active)
            -> Navigation.h public goals/results
  -> updateRobotController()
       -> updateTOFSensors()
       -> updateObjectTOFSensors()
       -> updateLocalMapFromSensors()
       -> updateOdometry()
       -> updateNavigationController()
            -> ObstacleContext
            -> ForwardTrajectoryPlanner or RecoveryPlanner
       -> updateMotorController()
```

Keep `src/motion/MotorControl.cpp` as the only periodic motor-output owner. New
behaviours should request driving through `Navigation.h`. Only bounded diagnostic/manual
owners should call `setAuthorizedMotionCommand()` directly; no module may write
motor pulses outside `MotorControl.cpp`. `src/operator/Bluetooth.cpp` still contains command
handling and telemetry formatting so command names, CSV fields, and test
workflows stay in one visible interface file during hardware validation.

Mission and future pickup code use navigation as a nonblocking black box:

```cpp
if (navigationGetStatus().state == NAVIGATION_IDLE) {
  navigationGoTo(targetX, targetY);
}

NavigationStatus status = navigationGetStatus();
if (status.state == NAVIGATION_REACHED) {
  navigationClearResult();
  // Perform the location action or submit the next goal.
}
```

Only `RouteMission.cpp` reads the onboard route. Navigation receives one point
at a time and cannot advance, skip, or reinterpret mission actions.

## Bluetooth CH9143 Link

Connect the robot-side CH9143 board to the Robocup `Serial2` connector and plug
the matched board into the computer. Open the computer COM port at `115200`.
The robot boots into `WAITING_FOR_START`; navigation does not begin until you
send `START`. Normal robot `Serial.print()` debug output is mirrored to both
USB serial and the CH9143 terminal.

Available commands:

- `HELP` - show commands.
- `BUILD` - print the firmware build label.
- `START` - begin navigation from the waiting state.
- `STATUS` - print state, pose, ToF readings, encoder counts, and flags.
- `STREAM ON` / `STREAM OFF` - enable or disable one status line per second.
- `CSV ON` / `CSV OFF` - enable or disable CSV telemetry rows.
- `CAL` - print the calibration summary again.
- `SPEED <ticks/s>` - set a temporary base target speed for this boot.
- `SPEED RESET` - restore the default base target speed.
- `SEARCHTURN <offset_us>` / `SEARCHTURN RESET` - set or reset the search-only
  slow pivot pulse offset used by `TEST SEARCH` and `SEARCH` scan/confirm
  turns. Current allowed range is `120` to `300 us`.
- `FBASE <left_us> <right_us>` - set temporary forward motor base pulses.
- `FBASE RESET` - restore the default forward motor base pulses.
- `ESCAPE ON` / `ESCAPE OFF` / `ESCAPE STATUS` - enable, disable, or report
  reverse-repositioning policy. Physical motion still requires all three rear
  VL53L1X channels to be initialized, valid, fresh, coherent, and non-blocked.
- `EMERGENCY ON` / `EMERGENCY OFF` / `EMERGENCY STATUS` - enable, disable,
  or report the ultimate scan/relocate/retry recovery policy for this boot.
  Policy changes are rejected while a navigation goal is active.
- `TEST ARM` / `TEST DISARM` - enable or disable manual test motion commands.
- `TEST DRIVE <metres>` - drive a fixed distance at the current heading.
- `TEST GOTO <x> <y>` - go to one temporary absolute waypoint using normal
  local-planner navigation.
- `TEST AVOID <metres>` - create one temporary waypoint straight ahead and use
  normal navigation/avoidance to reach it.
- `TEST ESCAPE <metres>` - no-path/recovery diagnostic. It remains neutral on
  the physical build until trusted rear coverage is installed.
- `TEST FAN` or `FAN` - print the high forward ToF fan sector table.
- `TEST REAR` or `REAR` - print all three rear VL53L1X states and aggregate.
- `TEST MATRIX` or `MATRIX` - print the 8x8 front frame and weight evidence.
  `TEST OBJECT` remains a command alias only; there is no old object-ToF path.
- `TEST HUNT TARGET` - print the confirmed static matrix track without moving.
- `TEST HUNT` - after `TEST ARM`, latch that track, request the normal 2600
  ticks/s forward ceiling, steer from the matrix, perform the configurable
  assumed 30 mm funnel handoff, and maintain at least 150 mm unconfirmed feed.
  It never confirms payload or material.
- `TEST FOLLOW START|STOP|STATUS` - explicit-start bounded matrix-follow
  diagnostic. It never performs the pickup handoff.
- `TEST SEARCH` - after `TEST ARM`, run the same short waypoint-style weight
  search using the current robot pose as the temporary search waypoint.
- `TEST SIDE <seconds>` - sample the avoidance side choice once per second
  without moving the motors.
- `TEST TURN <degrees>` - turn a fixed signed angle.
- `TEST TURNPULSE <signed_seconds>` - run a calibrated turn pulse and log its
  one-second coast for turn calibration.
- `TEST TURNLADDER LEFT|RIGHT` - disabled. It is not a runnable movement
  command because raw blocking pulses cannot be continuously supervised.
- `MARK <note>` - print and CSV-log a test note. Commas are replaced with
  semicolons in CSV event detail.
- `MANUAL ARM` / `MANUAL DISARM` - enable or disable live manual drive.
- `DRIVE <forward> <turn>` - live manual drive command, both values `-100` to `100`; positive turn is left/CCW.
- `HOME` - request the return-home state.
- `ZERO` - stop and reset yaw, pose, local map, encoder references, and PID state.
- `STOP` - stop motors and set `END_MATCH`.

`TEST DRIVE` accepts `0.01` to `1.50` metres. `TEST AVOID` and `TEST ESCAPE`
accept `0.10` to `2.00` metres. `TEST GOTO` accepts coordinates from `-10.00`
to `10.00` metres. `TEST SIDE` accepts `1` to `30` seconds and does not
require `TEST ARM` because it never drives the motors. `TEST TURN` accepts
signed angles from `-360` to `360` degrees, excluding angles inside the turn
tolerance. `TEST TURNLADDER` is disabled. Motion tests
require `TEST ARM` first and finish by stopping the motors and returning to
`END_MATCH`.

Normal global slow pivot pulses remain `1800/1200 us`. Weight-search scan and
confirm turns use the runtime `SEARCHTURN` offset instead, defaulting to
`280 us`, so search can be tuned without changing normal planner turns.

`TEST SIDE` includes each candidate side's `passable` flag and inner/outer
`sweep_clearance_mm` values. It is a stationary fan diagnostic, not the V7
motion policy; the planner evaluates a family of footprint-safe arcs instead.

The local planner logs `planner_safe_stop` when it cannot prove a safe arc.
It evaluates short differential-drive trajectories against the local map and
the measured footprint, then naturally returns toward the active target as
soon as the target direction is safe. `ESCAPE ON` / `ESCAPE OFF` controls the
reverse-repositioning policy, but cannot bypass the trusted-rear capability or
the final motor-safety gate. Physical reverse requires all three installed rear
channels to be live, coherent, valid, and clear.

All four forward-facing fan rays form a diagonal footprint guard. A sudden
close valid endpoint pauses motion for confirmation, while a confirmed close
endpoint or a very small nonzero invalid ToF return is treated as unsafe rather
than clear. An outer-fan fault pauses a turn for fresh sensing and then
forbids it if the fault persists.

For point goals, collision proof and preferred running room are separate. The
hard footprint allowance is deliberately small enough for a pre-aligned 400 mm
straight passage; the planner still prefers 50 mm or more of clearance when it
has a choice. A 400 mm passage cannot support an in-place turn, so the planner
rejects sustained steering once it has observed both nearby boundaries.

After two coherent geometric no-path results spanning 250 ms, the simulator
may enter evidence-driven reverse repositioning. Reverse candidates must keep
their complete inflated swept footprint inside cells with persistent clear
evidence; unknown, occupied, contradictory, and out-of-map cells are blocked.
Candidates first maximize endpoint obstacle clearance, then score forward
continuation, orientation toward unexplored cells, swept clearance, and
efficiency. There is no fixed retreat, route-rejoin phase, side memory, or
post-reverse escape. When clearance gain plateaus, the robot stops and gives
the normal forward planner one freshly revalidated takeover attempt.

An opt-in ultimate recovery layer is available behind
`PLANNER_EMERGENCY_SCAN_ENABLED`, which defaults to `false`. After an eligible
ordinary-recovery exhaustion, it stops for fresh fan and rear frames, attempts
a slow 12-sector scan, unwinds any incomplete scan before movement, and may
reverse at most 0.8 m through the existing trusted-rear and known-clear
rollout. From a sufficiently clear pose it performs one full scan, preserves
all map evidence, resets only temporary planner context, and retries the
original point goal once. Sensor, authority, turn-sweep, footprint, relocation,
and time failures remain neutral terminal failures.

The physical firmware and WASM simulator use the same runtime policy setter.
Use `EMERGENCY ON` before `START` or a test goal on the robot; Visual Lab
scenario imports use `emergencyScanEnabled`. Both reject changes during an
active goal, and both default to the shared `PLANNER_EMERGENCY_SCAN_ENABLED`
value.

The WASM simulator supplies three independently faultable rear rays and all 64
front-matrix rays. Production rear motion depends on every rear channel being
connected, valid, fresh within the sample-skew bound, and non-blocked. There is
no temporary rear compatibility alias.

Use the smallest test that exercises the feature being changed:

- command/telemetry changes: `ZERO`, `BUILD`, `STATUS`, `MARK`.
- straight drive calibration: `TEST DRIVE`.
- turn calibration: `TEST TURN`.
- waypoint controller changes: `TEST GOTO`.
- obstacle side-choice changes: `TEST SIDE`.
- obstacle movement changes: `TEST AVOID`, then full `START` only as an
  acceptance test.

`MANUAL ARM` enables live drive commands for the desktop serial UI. `DRIVE`
uses signed percentages: positive forward drives forward, negative forward
reverses, positive turn turns left, and negative turn turns right. The firmware
stops motors if live drive commands stop arriving for about `350 ms`.

The canonical navigation frame is `+X` forward and `+Y` robot-left.
`+yaw`, `+heading`, `+turn`, and `+curvature` are counter-clockwise/left. Wheel
speed is forward-positive, with `left = forward - turn`,
`right = forward + turn`, and `omega = (right - left) / trackWidth`. The
installed BNO055's zero-relative raw yaw is clockwise/right-positive, so only
`navigationHeadingDeg()` converts it to the canonical sign; navigation code
must not consume raw IMU yaw directly.

When `CSV ON` is active, firmware emits the frozen schema-v3 header and
metadata followed by sequence-numbered 61-field `telemetry` and `event` rows.
The 10 Hz telemetry includes raw fan ranges/validity/ages, motor outputs,
wheel targets and rates, IMU/navigation yaw, planner command/clearance/stop
state, loop timing, queue depth, and dropped-row diagnostics. All rows are
assembled in bounded RAM buffers and drained cooperatively so logging cannot
block the motor-control loop.

At startup the sketch prints the current calibration summary: motor pulse
widths, encoder signs, ticks per metre, PID gains, waypoint tolerance, and ToF
safety thresholds. Copy that header into test notes so each run can be traced
back to the exact settings used.

For reliability runs, prefer the saved-log runner in
`SerialCommandUI/run_navigation_regression.py`. It sends a marked test command,
saves the raw serial transcript, parsed CSV rows, summary JSON, command
sequence, and final `STATUS`. Use it before changing planner constants,
clearance margins, or default speed.

## High Forward ToF Fan

The V7 local planner uses the high fan configured right-to-left by XSHUT/header number:

| Index | Name | Angle | Model | XSHUT | Address |
| --- | --- | ---: | --- | ---: | --- |
| 0 | `right_outer` | `-60` | VL53L0X | 0 | `0x30` |
| 1 | `right_inner` | `-20` | VL53L0X | 1 | `0x31` |
| 2 | `left_inner` | `+20` | VL53L0X | 2 | `0x32` |
| 3 | `left_outer` | `+60` | VL53L0X | 3 | `0x33` |

Legacy `front`, `left`, and `right` telemetry remains available. `left` and
`right` are aggregate fan readings using the nearest valid reading on that side,
so existing avoidance can compare left-side and right-side clearance while the
full fan is being validated. `front` is a virtual safety reading using the
nearest valid `right_inner` or `left_inner` reading, since there is no physical
0-degree front sensor in this layout. Very small nonzero readings below the
normal calibrated window are treated as unsafe aggregate clearance rather than
clear space. Use `TEST FAN` for the full sector table, and `TEST SIDE <seconds>`
for a no-motion check of the side choice that avoidance would make from the
current fan clearances.

### Footprint-aware avoidance geometry

Avoidance geometry is configured once in `Robot.h`, relative to the midpoint
between the drive wheels: `+X` is forward and `+Y` is left. The same block
contains the chassis extents and four fan sensor origins/angles, so future
mechanical changes do not require editing the avoidance algorithm. The current
turn-footprint margin is `50 mm`; validate it with `TEST SIDE` before changing
avoidance motion.

### Calibrating fan geometry

Treat each ToF reading as starting at an **effective beam origin**, not at the
robot centre. The effective origin intentionally includes any small fixed range
offset inside the sensor, which is exactly what the clearance maths needs.

1. Square the robot to a long flat front wall and record each raw fan range at
   three known gaps from the robot's front face (for example 300, 500, and
   700 mm). Use `CSV ON`, `MARK`, and `FAN`; the +/-60 degree outer rays need a
   wall wide enough to intercept them at every gap.
2. With `D = front_extent + front_face_gap` and readings `r1`, `r2` from two
   positions, fit the beam angle using
   `theta = acos((D2 - D1) / (r2 - r1))`. Then average
   `x = D - r * cos(theta)` over all positions.
3. Put a flat wall parallel to the relevant robot side. Express its coordinate
   as `Y` from the wheel-midpoint origin (right is negative, left positive),
   then fit the sideways origin with `y = Y - r * sin(theta)`.

Enter the resulting effective `x`, `y`, and signed angle in
`FAN_SENSOR_GEOMETRY` in `Robot.h`; do not change avoidance code for a
mechanical remount.

## Default START Route

`START` follows a calibration route with a longer first leg so obstacle
avoidance has room to bypass and rejoin:

- `(1.20, 0.00)` pause
- `(1.20, 0.80)` pause
- `(0.00, 0.80)` pause
- `(0.00, 0.00)` home

Waypoint actions are `MissionAction` enum values owned by
`src/mission/RouteMission.cpp`:

- `MISSION_ACTION_PAUSE` - run the normal short route settle.
- `MISSION_ACTION_HOME` - mark the final home waypoint.
- `MISSION_ACTION_SEARCH` - treat this waypoint as a likely weight location. The robot
  first drives to a nominal `250 mm` standoff before the waypoint, aligns to
  the bearing from its current pose to the waypoint, then checks centre and
  sweeps `+/-30 deg` using `WEIGHT_SEARCH_SWEEP_DEG`. It hunts at most one
  fresh confirmed `weight_sized` target. If no target is found at the
  standoff scan, the waypoint is marked searched and the route continues to
  the next waypoint.

During normal route travel, a fresh confirmed `weight_sized` target can also
interrupt the current route waypoint. The robot cancels the route goal,
confirms/locks the target, hunts once, then resumes the original waypoint.
This opportunistic interrupt is route-only; tests, return-home, active search,
and active hunts are not interrupted by object detections.

Normal waypoint travel assigns a local target up to `0.35 m` ahead on the line
to the waypoint. Every 40 ms, the local planner evaluates footprint-safe
differential-drive arcs against the confidence map and chooses the best one.
The 0.8 s rollout is only a safety/prediction horizon: a point goal completes
immediately on entering the `60 mm` arrival circle, so the controller does
not deliberately crawl merely because a hypothetical arc would continue past
the target. Scripted `PAUSE` and `HOME` route actions use short `250 ms`
settles for route tests.

## Note

The active V7 firmware has one scheduled navigation path and one periodic
motor-output owner. Historical V2-V4 sketches remain in the workspace for
comparison, but they are not part of the V7 build.

## Front Matrix Weight Stage

See [CURRENT_STATE_AND_NEXT_STEPS.md](docs/CURRENT_STATE_AND_NEXT_STEPS.md) for
the current object/search checklist, and
[ROBOT_CODEBASE_AUDIT.md](docs/ROBOT_CODEBASE_AUDIT.md) for the broader safety
and architecture concerns around object detection.

The SEN0628 publishes pose- and attitude-stamped immutable 64-cell frames.
Collision evidence and weight perception are separate consumers: valid close
matrix cells may veto motion, while unknown/no-return cells do not establish
known-clear space. Weight tracking is advisory until the navigation interface
latches a confirmed static track.

Stationary characterization and later motion calibration still require fresh,
explicit hardware permission. The provisional 30 mm handoff, 150 mm feed,
classification dimensions, rear stop/clear distances, and prediction bounds
must be replaced or confirmed from saved physical frames before acceptance.
See [INTERNAL_FUNNEL_TOF_INTEGRATION_GUIDE.md](docs/INTERNAL_FUNNEL_TOF_INTEGRATION_GUIDE.md)
for the future payload-confirmation boundary.
