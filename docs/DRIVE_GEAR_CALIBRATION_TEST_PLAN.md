# Drive Gear Calibration Test Plan

Use this whenever drive gears, drive pulleys, track wheels, motors, encoders,
track tension, or the normal running surface changes. The goal is to find the
new encoder scale and the lowest reliable motor commands before trusting
planner, search, or gap tests again.

## Values To Produce

| Value | Code location | Runtime helper | Why it changes |
| --- | --- | --- | --- |
| `ticks_per_mm` | measurement only | none | Useful human unit for gear comparisons. |
| `TICKS_PER_METRE` | `RobotCode/RobotConfig.h` | none | Converts encoder counts into distance. |
| `LEFT_BASE_US` / `RIGHT_BASE_US` | `RobotCode/RobotConfig.h` | `FBASE <left> <right>` | Full-speed feed-forward balance for straight travel. |
| `DEFAULT_BASE_TARGET_SPEED` | `RobotCode/RobotConfig.h` | `SPEED <ticks/s>` | Normal planner speed in encoder ticks/s. |
| `PLANNER_MIN_DRIVABLE_SPEED_TPS` | `RobotCode/RobotConfig.h` | manual-drive sweep only | Lowest planner speed that really moves both tracks. |
| `SEARCHTURN` default or slow turn pulses | `RobotCode/RobotConfig.h` | `SEARCHTURN`, `TEST TURNLADDER` | Gear torque/speed changes can alter pivot reliability. |
| `EFFECTIVE_TRACK_WIDTH_M` | `RobotCode/RobotConfig.h` | arc test | Usually unchanged by gears, but recalc if turn arcs changed. |

Do not change planner clearance, body geometry, or ToF thresholds during this
process. Keep this calibration about the drivetrain only.

## Gear Swap Record

Create a unique gear id before testing, for example:

```text
gear_18t_20260706
```

Record:

```text
Gear id:
Drive gear/pulley teeth:
Driven gear/pulley teeth:
Belt pitch or wheel diameter:
Track tension:
Battery condition:
Floor surface:
Robot mass/payload:
Firmware build label:
```

If only the final drive pulley tooth count changed, use this as the first
estimate:

```text
estimated_ticks_per_m = old_ticks_per_m * old_drive_teeth / new_drive_teeth
estimated_default_tps = old_default_tps * estimated_ticks_per_m / old_ticks_per_m
estimated_min_tps     = old_min_tps * estimated_ticks_per_m / old_ticks_per_m
```

Larger final drive gears move farther per encoder count, so counts/mm should
go down. Smaller final drive gears make counts/mm go up.

## Stage 0: Stationary Sanity

Use open floor, motors powered, robot on the ground but with no motion command
armed.

```text
BUILD
CAL
ZERO
CSV ON
MARK <gear_id>_stationary
STATUS
TEST FAN
CSV OFF
```

Pass:

- `CAL` prints the expected old values before you start tuning.
- `STATUS` shows motors neutral at `1500/1500`.
- All required fan sensors are valid in open space.
- Encoder counts are stable while stopped.

Stop here if a sensor is invalid, a motor is not neutral, or encoders count
while the robot is not moving.

## Stage 1: Encoder Counts Per Millimetre

Use a straight tape line. Measure actual travel from a consistent robot
reference point, preferably the wheel-midpoint projection. Do not assume the
commanded distance is the actual distance.

Run at least three `0.50 m` and three `1.00 m` tests. Add `1.25 m` or `1.50 m`
if the floor is long enough.

```powershell
cd "C:\Users\marco\Documents\Robot Algorithm\SerialCommandUI"

python .\run_navigation_regression.py --mark <gear_id>_drive050_r1 --test drive --distance 0.50 --timeout 18 --layout "open floor; tape measured"
python .\run_navigation_regression.py --mark <gear_id>_drive050_r2 --test drive --distance 0.50 --timeout 18 --layout "open floor; tape measured"
python .\run_navigation_regression.py --mark <gear_id>_drive050_r3 --test drive --distance 0.50 --timeout 18 --layout "open floor; tape measured"

python .\run_navigation_regression.py --mark <gear_id>_drive100_r1 --test drive --distance 1.00 --timeout 22 --layout "open floor; tape measured"
python .\run_navigation_regression.py --mark <gear_id>_drive100_r2 --test drive --distance 1.00 --timeout 22 --layout "open floor; tape measured"
python .\run_navigation_regression.py --mark <gear_id>_drive100_r3 --test drive --distance 1.00 --timeout 22 --layout "open floor; tape measured"
```

For each saved CSV, compute deltas from the `test_drive_start` row to the
`test_drive_end` row.

```powershell
$csvPath = "logs\navigation_regressions\<saved_run>.csv"
$actualM = 0.965
$rows = Import-Csv $csvPath
$start = $rows | Where-Object { $_.event -eq "test_drive_start" } | Select-Object -First 1
$end = $rows | Where-Object { $_.event -eq "test_drive_end" } | Select-Object -Last 1
$leftTicks = [math]::Abs([double]$end.enc_l - [double]$start.enc_l)
$rightTicks = [math]::Abs([double]$end.enc_r - [double]$start.enc_r)
$avgTicks = ($leftTicks + $rightTicks) / 2.0
$ticksPerM = $avgTicks / $actualM
$ticksPerMm = $ticksPerM / 1000.0
[pscustomobject]@{
  LeftTicks = $leftTicks
  RightTicks = $rightTicks
  AvgTicks = $avgTicks
  ActualM = $actualM
  TicksPerM = $ticksPerM
  TicksPerMm = $ticksPerMm
  LeftRightDiffPct = 100.0 * [math]::Abs($leftTicks - $rightTicks) / $avgTicks
}
```

Choose the new encoder scale from the median of the good `ticksPerM` values.
Reject or repeat any run with wheel slip, contact, `planner_safe_stop`,
`wheel_mismatch`, or a visibly bad start pose.

Pass:

- `0.50 m` and `1.00 m` medians agree within about `3%`.
- Left/right tick deltas agree within about `5%` on straight runs.
- Heading drift is small and repeatable.
- No safe-stop or stuck flags.

Output:

```text
new_ticks_per_mm = median_ticks_per_m / 1000
new_TICKS_PER_METRE = median_ticks_per_m
```

## Stage 2: Forward Base Pulse Balance

`LEFT_BASE_US` and `RIGHT_BASE_US` are full-speed feed-forward balance values,
not the minimum pulse that moves the motor. Use runtime `FBASE` to sweep them
before editing code.

Start conservative. If the old pair was close, test around it. Current old
baseline is `1935/1870`.

```text
FBASE 1915 1850
ZERO
CSV ON
MARK <gear_id>_fbase1915_1850_drive030
TEST ARM
TEST DRIVE 0.30
STATUS
CSV OFF
TEST DISARM
```

Try small steps, for example:

```text
FBASE 1900 1850
FBASE 1915 1860
FBASE 1935 1870
FBASE 1950 1880
FBASE 1965 1890
```

Use the serial UI for this sweep, or set `FBASE` manually before running the
regression logger. The logger does not currently have a `--fbase` option.

Pass:

- `TEST DRIVE 0.30` reaches the waypoint.
- No `planner_safe_stop`, `drive_stuck`, or `wheel_mismatch`.
- Final heading drift is the smallest repeatable value, not just the best
  single run.
- Left/right encoder deltas are close.
- Mid-run `motor_l_us` and `motor_r_us` are not stuck against the PID correction
  limit.

Then verify the best pair with:

```text
FBASE <best_left> <best_right>
ZERO
CSV ON
MARK <gear_id>_best_fbase_drive100
TEST ARM
TEST DRIVE 1.00
STATUS
CSV OFF
TEST DISARM
```

Output:

```text
new_LEFT_BASE_US =
new_RIGHT_BASE_US =
```

## Stage 3: Minimum Drivable Speed

This finds `PLANNER_MIN_DRIVABLE_SPEED_TPS`: the lowest encoder tick/s command
that actually moves both tracks smoothly. Because the planner refuses speeds
below the compiled minimum, use manual drive for the first sweep.

For each candidate speed, set `SPEED <candidate>`, then use the serial UI
manual-drive controls so `DRIVE 100 0` is resent continuously for about
`2 seconds`.

Candidate list:

```text
600, 800, 1000, 1200, 1400, 1500, 1700, 1900
```

Per candidate:

```text
SPEED <candidate_tps>
FBASE <best_left> <best_right>
ZERO
CSV ON
MARK <gear_id>_minspeed_<candidate_tps>
MANUAL ARM
DRIVE 100 0
DRIVE 0 0
STATUS
MANUAL DISARM
CSV OFF
```

If you are typing commands manually instead of using the UI controls, resend
`DRIVE 100 0` at least every `0.2 s`; the firmware stops live drive after about
`350 ms` without a fresh command.

Accept the lowest candidate that:

- starts both tracks without a push;
- sustains smooth movement for the whole pulse;
- has both encoder deltas clearly above noise;
- does not set `drive_stuck` or `wheel_mismatch`;
- stops cleanly when commanded.

Set the compiled minimum with margin:

```text
new_PLANNER_MIN_DRIVABLE_SPEED_TPS =
  accepted_minimum_tps + max(100, accepted_minimum_tps * 0.15)
```

Round to a simple value such as `1200`, `1400`, `1500`, or `1700`. Do not set
the planner minimum at the barely-moving threshold.

After editing and uploading, verify with:

```powershell
python .\run_navigation_regression.py --mark <gear_id>_mindrive_verify --speed <new_min_tps> --test drive --distance 0.30 --timeout 18 --layout "open floor; minimum speed verify"
```

## Stage 4: Default Speed Ladder

Pick a default speed after encoder scale and minimum speed are known. To keep
the old physical speed as a starting point:

```text
candidate_default_tps = old_default_tps * new_TICKS_PER_METRE / old_TICKS_PER_METRE
```

Run a ladder. Keep the fastest run below `3000 ticks/s`, which is the current
runtime `SPEED` command limit.

```powershell
python .\run_navigation_regression.py --mark <gear_id>_speed1800_drive030 --speed 1800 --test drive --distance 0.30 --timeout 18 --layout "open floor"
python .\run_navigation_regression.py --mark <gear_id>_speed2200_drive030 --speed 2200 --test drive --distance 0.30 --timeout 18 --layout "open floor"
python .\run_navigation_regression.py --mark <gear_id>_speed2600_drive030 --speed 2600 --test drive --distance 0.30 --timeout 18 --layout "open floor"
python .\run_navigation_regression.py --mark <gear_id>_speed3000_goto100 --speed 3000 --test goto --x 1.00 --y 0.00 --timeout 24 --layout "open floor"
```

Use only speeds that are at least `new_PLANNER_MIN_DRIVABLE_SPEED_TPS`.

Pass for a new default:

- Three repeat runs at that speed pass.
- The robot physically looks stable.
- No contact, safe-stop, stuck, or wheel-mismatch event.
- `max_forward_tps` reaches the requested speed in the summary JSON.
- Stopping distance and final heading are acceptable.

Output:

```text
new_DEFAULT_BASE_TARGET_SPEED =
```

If the new default is near the minimum speed, the gear set is too slow for the
current planner assumptions. If the new default must be near `3000` to be
useful, the gear set may be too tall or may need code support above the current
runtime speed limit.

## Stage 5: Turn And Search-Speed Checks

Gear changes can alter pivot torque and coast. Run these after the forward
values look sane.

```text
ZERO
CSV ON
MARK <gear_id>_turnpulse_pos
TEST ARM
TEST TURNPULSE 0.50
STATUS
TEST DISARM
CSV OFF

ZERO
CSV ON
MARK <gear_id>_turnpulse_neg
TEST ARM
TEST TURNPULSE -0.50
STATUS
TEST DISARM
CSV OFF
```

Then run the slow ladder if slow turns or search turns feel weak or too sharp:

```text
ZERO
TEST ARM
TEST TURNLADDER LEFT
STATUS
TEST TURNLADDER RIGHT
STATUS
TEST DISARM
```

For normal point turns:

```powershell
python .\run_navigation_regression.py --mark <gear_id>_turn90 --test turn --angle 90 --timeout 18 --layout "open floor"
python .\run_navigation_regression.py --mark <gear_id>_turn_minus90 --test turn --angle -90 --timeout 18 --layout "open floor"
```

For search turns, compare only if object/search work depends on the new gears:

```text
SEARCHTURN 240
ZERO
TEST ARM
TEST SEARCH
STATUS
TEST DISARM

SEARCHTURN 280
ZERO
TEST ARM
TEST SEARCH
STATUS
TEST DISARM
```

Pass:

- `TEST TURN 90` and `TEST TURN -90` settle inside tolerance without crawling.
- `TEST TURNPULSE` coast is not dramatically larger than the old baseline.
- `TEST TURNLADDER` has a symmetric reliable offset for left and right.
- `TEST SEARCH` scan turns are strong enough without overshooting wildly.

Update `SEARCHTURN` default or `TURN_*_SLOW_US` only after this stage. Do not
change global slow-turn pulses just to fix search if `SEARCHTURN` is enough.

## Stage 6: Effective Track Width Check

Changing gears should not change the physical centreline spacing, but it can
change skid and the effective turning response. Re-run this only if arc paths,
gap approaches, or turn telemetry now disagree with the old value.

Formula:

```text
left_distance_m  = left_delta_ticks / new_TICKS_PER_METRE
right_distance_m = right_delta_ticks / new_TICKS_PER_METRE
yaw_rad = abs(yaw_delta_deg) * pi / 180
effective_track_width_m = abs((right_distance_m - left_distance_m) / yaw_rad)
```

Run both directions and use the median. If the result differs from the current
`0.224 m` by more than about `5%`, update `EFFECTIVE_TRACK_WIDTH_M`, compile,
upload, and repeat clear `TEST GOTO` plus the affected gap tests.

## Stage 7: Commit Constants And Verify

Update only the constants supported by the evidence:

```cpp
const int LEFT_BASE_US  = <best_left>;
const int RIGHT_BASE_US = <best_right>;

const float TICKS_PER_METRE = <median_ticks_per_m>;
const float EFFECTIVE_TRACK_WIDTH_M = <only_if_remeasured>;

const float DEFAULT_BASE_TARGET_SPEED = <accepted_default_tps>;
const float PLANNER_MIN_DRIVABLE_SPEED_TPS = <accepted_min_tps_with_margin>;
```

Also check dependent values:

- `PLANNER_CORRIDOR_SQUEEZE_SPEED_TPS` must be at least the new minimum.
- `PLANNER_REVERSE_RECOVERY_MIN_SPEED_TPS` already follows
  `PLANNER_MIN_DRIVABLE_SPEED_TPS`.
- `PLANNER_ESCAPE_MIN_SPEED` already follows `PLANNER_MIN_DRIVABLE_SPEED_TPS`.
- `STUCK_ENCODER_SPEED_MIN` should stay well below the real moving speed, but
  high enough to catch a commanded motor that is not moving.

Compile and upload:

```powershell
cd "C:\Users\marco\Documents\Robot Algorithm"
arduino-cli compile --fqbn teensy:avr:teensy40 RobotCode
arduino-cli upload -p COM13 --fqbn teensy:avr:teensy40 RobotCode
```

Post-upload proof:

```powershell
cd "C:\Users\marco\Documents\Robot Algorithm\SerialCommandUI"

python .\run_navigation_regression.py --mark <gear_id>_post_drive030 --test drive --distance 0.30 --timeout 18 --layout "open floor"
python .\run_navigation_regression.py --mark <gear_id>_post_drive100 --test drive --distance 1.00 --timeout 24 --layout "open floor; tape measured"
python .\run_navigation_regression.py --mark <gear_id>_post_goto050 --test goto --x 0.50 --y 0.00 --timeout 18 --layout "open floor"
python .\run_navigation_regression.py --mark <gear_id>_post_turn90 --test turn --angle 90 --timeout 18 --layout "open floor"
python .\run_navigation_regression.py --mark <gear_id>_post_turn_minus90 --test turn --angle -90 --timeout 18 --layout "open floor"
```

If those pass, rerun the smallest obstacle/gap gate that matters for the next
work item. Do not jump straight to the hardest gap at a new speed.

## Result Template

```text
Gear id:
Accepted? yes/no

Old TICKS_PER_METRE:
New TICKS_PER_METRE:
New ticks_per_mm:

Old LEFT_BASE_US / RIGHT_BASE_US:
New LEFT_BASE_US / RIGHT_BASE_US:

Old DEFAULT_BASE_TARGET_SPEED:
New DEFAULT_BASE_TARGET_SPEED:

Old PLANNER_MIN_DRIVABLE_SPEED_TPS:
New PLANNER_MIN_DRIVABLE_SPEED_TPS:

Search turn/default turn changes:
Effective track width change:

Evidence logs:
- <summary json or csv path>
- <summary json or csv path>

Physical notes:
- measured actual distances:
- contact/no-contact:
- wobble/drift:
- battery/surface:
```

## Rejection Rules

Reject the gear setup or repeat the test before editing constants if:

- encoder scale varies by more than `3%` between repeated clean runs;
- left/right encoder deltas differ by more than `5%` on straight tests;
- a candidate minimum speed only moves after a push;
- the desired default speed is too close to the minimum speed;
- `wheel_mismatch`, `drive_stuck`, or `planner_safe_stop` appears in open-floor
  drivetrain tests;
- the robot turns or coasts much farther than the old baseline and cannot be
  corrected by the existing turn ladder/search-turn controls.

The safe answer after a bad gear swap is to back down speed or reject the gear
ratio, not to loosen navigation safety margins.
