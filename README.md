# RobotCode

Current RoboCup robot firmware for Teensy 4.0.

V7 uses a non-blocking, safety-supervised local planner.  The four forward
navigation ToFs are VL53L0X sensors and build a short-lived local confidence
map; a footprint-aware receding-horizon controller selects a safe
differential-drive arc toward the active waypoint.  There is no fixed
reverse/turn/bypass/rejoin script and no
outer-fan wall-follow fallback.  The present fan has no rear or true side
coverage, so invalid or blind space is never treated as clear.

See [NAVIGATION_PLANNER_TEST_PLAN.md](docs/NAVIGATION_PLANNER_TEST_PLAN.md) for the
required hardware-led validation sequence, including the G0-G5 gap ladder and
track-width calibration gate.

## Read First

- [Current state and next steps](docs/CURRENT_STATE_AND_NEXT_STEPS.md)
- [Navigation planner test plan](docs/NAVIGATION_PLANNER_TEST_PLAN.md)
- [Object detection and hunting plan](docs/OBJECT_DETECTION_HUNTING_PLAN.md)

## Files

- `RobotCode.ino` - Arduino setup/loop only.
- `Robot.h` - shared configuration, globals, enums, structs, and prototypes.
- `Globals.cpp` - hardware objects, runtime state, and waypoint list.
- `Encoders.cpp` - encoder interrupt handlers.
- `StateMachine.cpp` - high-level robot states.
- `LocalPlanner.cpp` - local confidence map, safe-arc rollout, goal control,
  scheduled sensor/odometry/planner updates, and planner telemetry.
- `Navigation.cpp` - route point-goal assignment.
- `ObstacleAvoidance.cpp` - stationary fan-clearance diagnostics for `TEST SIDE`.
- `StuckRecovery.cpp` - wheel/yaw progress detection.
- `Odometry.cpp` - encoder/IMU pose integration.
- `Imu.cpp` - BNO055 connection and yaw helpers.
- `TofSensors.cpp` - SX1509 and ToF setup/read logic.
- `MotorControl.cpp` - the single motor-output owner and wheel-speed PID.
- `Helpers.cpp` - angle wrapping, cumulative encoder control snapshots, count
  reads, and pose print.
- `Bluetooth.cpp` - CH9143 Bluetooth serial command and telemetry link on `Serial2`.

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
  the default front-blocked reverse recovery. It boots enabled.
- `TEST ARM` / `TEST DISARM` - enable or disable manual test motion commands.
- `TEST DRIVE <metres>` - drive a fixed distance at the current heading.
- `TEST GOTO <x> <y>` - go to one temporary absolute waypoint using normal
  local-planner navigation.
- `TEST AVOID <metres>` - create one temporary waypoint straight ahead and use
  normal navigation/avoidance to reach it.
- `TEST ESCAPE <metres>` - create one temporary waypoint straight ahead and
  explicitly exercise the front-blocked escape system: forward motion remains
  stopped while the virtual front is blocked, then after the no-path debounce
  the planner may run trusted fake-rear-ToF reverse arcs until forward arcs
  become valid again.
- `TEST FAN` or `FAN` - print the high forward ToF fan sector table.
- `TEST OBJECT` or `OBJECT` - print the object/weight ToF table and current
  object-candidate summary. This never moves the motors.
- `TEST HUNT TARGET` - print the estimated pickup target without moving.
- `TEST HUNT` - after `TEST ARM`, lock the fresh confirmed weight target and
  give the local planner a pickup goal `150 mm` forward of the estimated weight
  point. Hunt goals add a small speed boost during that final carry-through
  zone and complete near the biased target without requiring the normal
  point-goal final heading.
- `TEST SEARCH` - after `TEST ARM`, run the same short waypoint-style weight
  search using the current robot pose as the temporary search waypoint.
- `TEST SIDE <seconds>` - sample the avoidance side choice once per second
  without moving the motors.
- `TEST TURN <degrees>` - turn a fixed signed angle.
- `TEST TURNPULSE <signed_seconds>` - run a calibrated turn pulse and log its
  one-second coast for turn calibration.
- `TEST TURNLADDER LEFT|RIGHT` - run a raw slow-turn pulse ladder for new
  chassis slow-pivot calibration. This requires `TEST ARM` and directly pivots
  the robot.
- `MARK <note>` - print and CSV-log a test note. Commas are replaced with
  semicolons in CSV event detail.
- `MANUAL ARM` / `MANUAL DISARM` - enable or disable live manual drive.
- `DRIVE <forward> <turn>` - live manual drive command, both values `-100` to `100`.
- `HOME` - request the return-home state.
- `ZERO` - stop and reset yaw, pose, local map, encoder references, and PID state.
- `STOP` - stop motors and set `END_MATCH`.

`TEST DRIVE` accepts `0.01` to `1.50` metres. `TEST AVOID` and `TEST ESCAPE`
accept `0.10` to `2.00` metres. `TEST GOTO` accepts coordinates from `-10.00`
to `10.00` metres. `TEST SIDE` accepts `1` to `30` seconds and does not
require `TEST ARM` because it never drives the motors. `TEST TURN` accepts
signed angles from `-360` to `360` degrees, excluding angles inside the turn
tolerance. `TEST TURNLADDER` tests offsets `120,160,200,240,280,300 us` from
neutral with `250 ms` pulses and `500 ms` stopped/coast windows. Motion tests
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
soon as the target direction is safe. Front-blocked reverse recovery is enabled
by default and can be toggled with `ESCAPE ON` / `ESCAPE OFF`. `TEST ESCAPE`
is the safe diagnostic alternative to an emergency-block override: it never
drives forward into a blocked virtual front, but it lets persistent
`front_blocked` evidence enter the reverse-recovery/replan path. There
are no `adaptive_bypass_plan`,
`adaptive_rejoin`, fixed reverse, or fixed-angle turn events in V7.

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

If forward planning remains impossible for 250 ms, V7 enters reverse recovery.
For this testing build, `RANGE_FAKE_REAR` is a trusted rear ToF reading fixed
at 4000 mm clear. The planner samples reverse arcs, commands the best one, and
tries the normal forward planner first again on every planner tick. It leaves
recovery as soon as a valid forward arc is available. Stuck-wheel detection
still aborts to protect the drivetrain.

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

When `CSV ON` is active, rows start with `row_type,event,detail`. Regular
once-per-second samples use `row_type=telemetry`. Event rows use
`row_type=event` for front blocked/clear transitions, ToF timeout/stale
transitions, planner safe stops, log marks, and manual test motion starts/ends.
CSV rows include the aggregate legacy `front/left/right` readings plus raw fan
fields `fan0_mm` through `fan3_mm`, `fan0_valid` through `fan3_valid`,
`front_virtual_mm`, raw fan ages, and planner candidate/command/clearance/
reason fields.

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
   700 mm). Use `CSV ON`, `MARK`, and `FAN`; the ±60 degree outer rays need a
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

Waypoint actions:

- `"PAUSE"` - run the normal short route settle.
- `"HOME"` - mark the final home waypoint.
- `SEARCH` - treat this waypoint as a likely weight location. The robot
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

## Weight ToF Stage

See [OBJECT_DETECTION_HUNTING_PLAN.md](docs/OBJECT_DETECTION_HUNTING_PLAN.md) for
the success gates and next-step checklist for this stage.

The object stage is separate from the VL53L0X navigation fan: four VL53L1X
object/weight ToFs are mounted at fixed distances from the robot centreline
and angled 20 degrees inward toward the intake/centreline. The current logical
layout is left/right LOW and UPPER columns.

The architecture is enabled with
`OBJECT_TOF_ENABLED = true`. `TEST OBJECT` prints the object ToF table without
moving the robot. If the subsystem is disabled again, the reserved object XSHUT
pins are held low so unconfigured VL53L1X devices do not interfere with the
navigation fan on the I2C bus.

Current measured object wiring is in the robot's frame: `XSHUT4=object_right_upper`,
`XSHUT5=object_left_upper`, `XSHUT6=object_right_low`, and
`XSHUT7=object_left_low`. Both columns are `91.4 mm` forward of the
wheel-midpoint frame and `60.6 mm` from centreline. LOW sensors are `55 mm`
from the floor and UPPER sensors are `120 mm`; left yaw is `-20 deg`, right yaw
is `+20 deg`.

Before folding this into autonomous collection, collect `TEST OBJECT` and/or
`TOFReturnSignalExperiment/TOFReturnSignalExperiment.ino` samples on the
VL53L1X channels outside the stable navigation behaviour. The logs should
include range status, distance, return signal rate, ambient rate, and derived
signal features for walls, ramps, steel weights, plastic weights, and
lighting/angle changes. If the classes overlap, keep return signal as a
confidence feature only, not as a safety or material decision.
