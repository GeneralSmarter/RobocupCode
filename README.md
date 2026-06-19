# RobotCode

Current RoboCup robot firmware for Teensy 4.0.

The intent of this version is to preserve the Step 23 behavior while making
future navigation work easier to test and extend.

## Files

- `RobotCode.ino` - Arduino setup/loop only.
- `Robot.h` - shared configuration, globals, enums, structs, and prototypes.
- `Globals.cpp` - hardware objects, runtime state, and waypoint list.
- `Encoders.cpp` - encoder interrupt handlers.
- `StateMachine.cpp` - high-level robot states.
- `Priority.cpp` - emergency, stuck, obstacle, and return-home arbitration.
- `Navigation.cpp` - waypoint navigation and waypoint actions.
- `ObstacleAvoidance.cpp` - obstacle-avoid sequence and turn choice.
- `DriveControl.cpp` - distance driving, heading hold, wall-follow fallback.
- `Turning.cpp` - IMU-based turn routines and turn motor commands.
- `StuckRecovery.cpp` - stuck detection and recovery behavior.
- `Odometry.cpp` - encoder/IMU pose integration.
- `Imu.cpp` - BNO055 connection and yaw helpers.
- `TofSensors.cpp` - SX1509 and ToF setup/read logic.
- `MotorControl.cpp` - motor write and wheel-speed PID helpers.
- `Helpers.cpp` - angle wrapping, encoder/PID reset, count reads, pose print.
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
- `HGAIN <gain>` - set a temporary heading correction gain for this boot.
- `HGAIN RESET` - restore the default heading correction gain.
- `FBASE <left_us> <right_us>` - set temporary forward motor base pulses.
- `FBASE RESET` - restore the default forward motor base pulses.
- `TEST ARM` / `TEST DISARM` - enable or disable manual test motion commands.
- `TEST DRIVE <metres>` - drive a fixed distance at the current heading.
- `TEST GOTO <x> <y>` - go to one temporary absolute waypoint using normal
  waypoint navigation and priorities.
- `TEST AVOID <metres>` - create one temporary waypoint straight ahead and use
  normal navigation/avoidance to reach it.
- `TEST FAN` or `FAN` - print the high forward ToF fan sector table.
- `TEST TURN <degrees>` - turn a fixed signed angle.
- `MARK <note>` - print and CSV-log a test note. Commas are replaced with
  semicolons in CSV event detail.
- `MANUAL ARM` / `MANUAL DISARM` - enable or disable live manual drive.
- `DRIVE <forward> <turn>` - live manual drive command, both values `-100` to `100`.
- `HOME` - request the return-home state.
- `ZERO` - stop and reset yaw, pose, encoders, and PID state.
- `STOP` - stop motors and set `END_MATCH`.

`TEST DRIVE` accepts `0.01` to `1.50` metres. `TEST AVOID` accepts `0.10` to
`2.00` metres. `TEST GOTO` accepts coordinates from `-3.00` to `3.00` metres.
`TEST TURN` accepts signed angles from `-360` to `360` degrees, excluding
angles inside the turn tolerance. Motion tests require `TEST ARM` first and
finish by stopping the motors and returning to `END_MATCH`.

Use the smallest test that exercises the feature being changed:

- command/telemetry changes: `ZERO`, `BUILD`, `STATUS`, `MARK`.
- straight drive calibration: `TEST DRIVE`.
- turn calibration: `TEST TURN`.
- waypoint controller changes: `TEST GOTO`.
- obstacle avoidance changes: `TEST AVOID`, then full `START` only as an
  acceptance test.

`MANUAL ARM` enables live drive commands for the desktop serial UI. `DRIVE`
uses signed percentages: positive forward drives forward, negative forward
reverses, negative turn turns left, and positive turn turns right. The firmware
stops motors if live drive commands stop arriving for about `350 ms`.

When `CSV ON` is active, rows start with `row_type,event,detail`. Regular
once-per-second samples use `row_type=telemetry`. Event rows use
`row_type=event` for front blocked/clear transitions, ToF timeout/stale
transitions, obstacle-avoidance start/end, stuck-recovery start/end, log marks,
and manual test motion starts/ends.

At startup the sketch prints the current calibration summary: motor pulse
widths, encoder signs, ticks per metre, PID gains, heading gain, waypoint
tolerance, and ToF safety thresholds. Copy that header into test notes so each
run can be traced back to the exact settings used.

## High Forward ToF Fan

The V6 high fan is configured right-to-left by XSHUT/header number:

| Index | Name | Angle | Model | XSHUT | Address |
| --- | --- | ---: | --- | ---: | --- |
| 0 | `right_outer` | `-45` | VL53L0X | 0 | `0x30` |
| 1 | `right_inner` | `-20` | VL53L1X | 1 | `0x31` |
| 2 | `left_inner` | `+20` | VL53L1X | 2 | `0x32` |
| 3 | `left_outer` | `+45` | VL53L0X | 3 | `0x33` |

Legacy `front`, `left`, and `right` telemetry remains available. `left` and
`right` are aggregate fan readings using the nearest valid reading on that side,
so existing avoidance can compare left-side and right-side clearance while the
full fan is being validated. `front` is a virtual safety reading using the
nearest valid `right_inner` or `left_inner` reading, since there is no physical
0-degree front sensor in this layout. Very small nonzero readings below the
normal calibrated window are treated as unsafe aggregate clearance rather than
clear space. Use `TEST FAN` for the full sector table.

## Default START Route

`START` follows a calibration route with a longer first leg so obstacle
avoidance has room to bypass and rejoin:

- `(1.20, 0.00)` pause
- `(1.20, 0.80)` pause
- `(0.00, 0.80)` pause
- `(0.00, 0.00)` home

## Note

This is a structural refactor, not a behavior rewrite. Known V3 issues from
`NAVIGATION_DESIGN.md` are intentionally left visible for the next control
upgrade rather than silently changed during modularisation.

## Planned ToF Experiment

Before the paired-height object sensing work is folded into the navigation
modules, run `TOFReturnSignalExperiment/TOFReturnSignalExperiment.ino` on a
VL53L1X channel outside the stable navigation build.

That experiment logs range status, distance, return signal rate, ambient rate,
and derived signal features for walls, ramps, steel weights, plastic weights,
and lighting/angle changes. If the dataset is useful, the future V5/V7 range
sensor modules should carry signal and ambient fields next to each distance
reading. If the classes overlap, keep return signal as a confidence feature
only, not as a safety or material decision.
