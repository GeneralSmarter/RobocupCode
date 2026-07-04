# V7 Local Planner Robot Test Plan

## What changed

V7 replaces the fixed reverse, fixed-angle turn, bypass, and rejoin script
with a scheduled local planner.  It maintains a 3 m x 3 m local confidence
map at 50 mm resolution, inflates obstacles by the measured chassis footprint
and safety uncertainty, and evaluates short differential-drive arcs to the
active waypoint.

The four physical fan sensors remain the real forward obstacle evidence.  All
four navigation fan sensors are now VL53L0X. This test build also adds
`RANGE_FAKE_REAR`, a trusted fake rear ToF fixed at
4000 mm clear, so reverse recovery is deliberately testing the planner shape
that a real rear sensor would enable. A stale/invalid virtual-front channel
stops forward navigation, and an invalid outer channel forbids a turn into that
side. After persistent failure of every forward arc, V7 may enter reverse
recovery: it samples reverse arcs, commands the best one, and retries forward
planning first on every planner tick.

The configured effective track width is currently `0.224 m` after the
2026-07-02 arc calibration on the rebuilt chassis.  The rebuilt body is about 181 mm
wide, so a 400 mm opening is now physically plausible when the approach is
nearly straight, but it is still not a place to tune clearance from a single
run.

## New telemetry

`STATUS` now includes the selected planner command, safe-arc count, swept
clearance, stop reason, and `fakeRear=<mm>/<valid>/<blocked>`. CSV adds:

- raw fan ages;
- fake rear distance/valid/blocked;
- planner candidate count;
- selected forward and turn targets;
- selected curvature;
- minimum swept clearance;
- clearance-derived speed cap;
- local-goal distance;
- planner reason, replan reason, and safe-stop reason.

`planner_safe_stop` is an intentional safety event, not a failed attempt to
run the retired reverse/turn recovery script.

## Logging rule for reliability work

For any run that may influence constants, default speed, or planner behaviour,
use the serial regression logger instead of relying on terminal scrollback:

```powershell
cd "C:\Users\marco\Documents\Robot Algorithm\SerialCommandUI"
python run_navigation_regression.py --mark v7_g5_default_repeat --test avoid --distance 1.00 --layout "G5 460 mm right gap"
python run_navigation_regression.py --mark v7_speed2600_g5_repeat_1 --speed 2600 --test avoid --distance 1.00 --layout "G5 460 mm right gap"
```

Each run saves:

- the exact raw serial transcript;
- parsed V7 CSV rows;
- a JSON summary with terminal event, candidate count, clearance, speed cap,
  rejection details, command sequence, and final `STATUS`.

Still record the physical observation separately: measured layout, whether
there was contact, and whether the motion looked stable. A saved CSV without
the physical contact result is not enough to accept a speed or clearance
change.

## Operator / Codex split

For live robot testing, use a setup-then-run loop:

- The user sets up physical conditions: robot start pose, obstacle/gap layout,
  rear clearance for escape tests, motor power, and any requested sensor cover.
- Codex runs the serial commands itself, preferably with
  `SerialCommandUI/run_navigation_regression.py` for movement tests.
- Codex reads the serial output and saved logs directly. The user should not
  need to paste serial output unless Codex cannot open the port.
- The user reports physical facts that serial cannot know: contact/no contact,
  approximate real-world offset, visible wobble/judder, and whether the setup
  was accidentally different from requested.
- Codex then chooses the next test or code change from the combined evidence.

When Codex asks for “ready,” it means the physical setup is complete and it is
safe for Codex to send the next serial command sequence.

## Completed pre-weight navigation gates

These are the gates that needed to pass before starting weight detection:

1. Reverse-recovery logger support passed on 2026-07-03. The regression runner
   logs `reverse_recovery_start`, `reverse_recovery_step`, and
   `reverse_recovery_end`, captures a post-recovery window, and disarms before
   final `STATUS`.
2. All-VL53L0X stationary sanity passed on 2026-07-03. `TEST FAN` reported all
   four fan sectors as `VL53L0X`; `STATUS` showed all fan sensors valid,
   `frontVirtual=8190`, `blocked=0`, `fakeRear=4000/1/0`, and motors neutral.
3. Clear-space current-firmware sanity passed on 2026-07-03 with
   `v7_all_l0x_clear_goto050`: terminal `test_goto_end,route_line_reached`, no
   `planner_safe_stop`, no candidate rejects, all fan sensors valid, and motors
   neutral.
4. Default-on reverse-recovery physical testing passed on 2026-07-03 with
   `v7_reverse_recovery_physical_2_fixed_logger`: `planner_safe_stop`,
   `reverse_recovery_start`, repeated `reverse_recovery_step,arc_selected`,
   `front_clear,confirmed`, and `reverse_recovery_end,forward_path_found`.
   Operator observation: clean.
5. The 400 mm competition-width gap confirmation passed on 2026-07-03 with
   `v7_gap400_confirm_avoid100_1`: terminal `test_avoid_end,waypoint_reached`,
   no safe-stop, no candidate rejects, no reverse recovery, all fan sensors
   valid, and motors neutral. Operator instruction: treat it as an absolute
   success.

Navigation is therefore cleared enough to move to the weight-detection stage.
Optional future confidence tests can repeat the 400 mm gap or the
avoid-then-return-through-gap sequence, but they are not blockers.

The next stage uses four VL53L1X object/weight ToFs at fixed centreline
offsets, angled 20 degrees inward toward the intake/centreline. Keep that
subsystem separate from the VL53L0X navigation fan until its classification
and interaction rules are measured.

## Stage 0: stationary safety and telemetry

Place the robot in a genuinely open area.  Keep the wheels clear or the robot
on the floor with `TEST ARM` **not** sent.

```text
BUILD
ZERO
CSV ON
MARK v7_stationary_smoke
STATUS
TEST SIDE 5
STATUS
CSV OFF
```

Pass criteria:

- `BUILD` says `V7-local-planner`.
- All four `fanValid` values are `1` in open space.
- Fan ages advance rather than remaining stale.
- `TEST SIDE` keeps motors stopped.
- `STATUS` includes `plannerCandidates`, `plannerV`, `plannerW`, and
  `plannerStop`.

If any fan is invalid, do not run movement tests.  Return the full `STATUS`
and the CSV event rows.

## Stage 1: clear route and goal controller

Use an obstacle-free area at least 1.5 m ahead.

```text
ZERO
CSV ON
MARK v7_clear_goto
TEST ARM
TEST GOTO 0.50 0.00
STATUS
CSV OFF
```

Expected behaviour: the robot begins without a blocking pivot, reaches the
target within the existing 60 mm tolerance, and emits `test_goto_end`.  CSV
should show nonzero `planner_candidates`, a forward command no larger than
`planner_speed_cap_tps`, and no `planner_safe_stop`.

For the current drivetrain, also verify that the first selected command starts
with `planner_w_tps` near zero for a straight-ahead target and that every
nonzero `planner_v_tps` is at least 1500.  In clear space, the 0.50 m straight
test should retain the full 2600 ticks/s command into the arrival circle; it
must not slow to 1500 merely because the planner's 0.8 s prediction extends
past a goal it would already have stopped at.  A `tof_close_revalidating`
pause is allowed only for one sensor cycle; a single sudden close fan return
must not become a map obstacle or a `diagonal_clearance` safe-stop unless the
next reading confirms it.

Then verify both direction conventions in clear space:

```text
ZERO
TEST ARM
TEST TURN 90
STATUS
ZERO
TEST ARM
TEST TURN -90
STATUS
```

Pass criteria: both turns settle inside the 3 degree tolerance, with no
oscillation or prolonged low-duty crawl.  The telemetry should transition from
`plannerReason=calibrated_turn_fast` to `calibrated_turn_slow` within the last
30 degrees, then emit one `turn_brake_start`/`turn_brake_end` pair.  That pair
is a safety-gated, direction-calibrated counter-turn pulse that cancels
residual rotation (currently 20 ms for positive/right and 40 ms for
negative/left); it is not a corrective manoeuvre.  The calibrated fast/slow/brake pulses are all
written through the single motor-output path.  During a turn, one invalid
outer-side or sweep-sensor sample causes an immediate motor stop and up to
120 ms of stationary revalidation; valid sensing resumes the same turn, while
a persistent invalid sample aborts.  A turn refuses to start when any fan is
invalid or its diagonal sweep is unsafe; that is the intended degraded-mode
policy.

### Stage 1b: no-forward-path reverse recovery

Use a large open area with generous clear space behind the robot. This build
uses `RANGE_FAKE_REAR` as a trusted 4000 mm-clear rear ToF, so the firmware is
expected to believe reverse space exists. Place a broad front panel close
enough that every forward arc is rejected.

```text
ZERO
CSV ON
MARK v7_reverse_recovery
TEST ARM
TEST ESCAPE 1.00
STATUS
CSV OFF
```

Front-blocked reverse recovery is enabled by default. Use `ESCAPE OFF` only
when you want a diagnostic run to safe-stop without attempting reverse
recovery. Use `ESCAPE ON` to restore normal behaviour and `ESCAPE STATUS` to
check it.

`TEST ESCAPE` is deliberately not a raw emergency-block override. Forward
motion remains stopped while the virtual front is blocked. Expected behaviour:
after at least 250 ms with persistent `front_blocked` or no safe forward arc,
the log emits `reverse_recovery_start,no_forward_path`, then
`reverse_recovery_step,arc_selected` as reverse arcs are selected. The planner
tries normal forward planning first on every planner tick. When forward arcs
become valid again, it emits `reverse_recovery_end,forward_path_found` and
continues toward the point goal. It should not abort merely because one reverse
step failed to produce a forward option.

### Current escape evidence

On 2026-07-03, `run_navigation_regression.py` was updated for
`reverse_recovery_start`, `reverse_recovery_step`, and
`reverse_recovery_end`, and no longer treats recovery start as a terminal
condition for non-escape tests. The fixed logger disarms before the final
`STATUS`.

The accepted physical repeat `v7_reverse_recovery_physical_2_fixed_logger`
made no contact and was reported clean by the operator. It showed the current
intended chain: `planner_safe_stop,front_blocked`,
`reverse_recovery_start,no_forward_path`, repeated
`reverse_recovery_step,arc_selected`, `front_clear,confirmed`, and
`reverse_recovery_end,forward_path_found`.

Older 2026-07-02 blind-backtrack testing proved the bounded reverse idea was
safe, but that one-shot behaviour has been replaced in current source.
Current firmware should be judged by `reverse_recovery_*` events, not
`blind_backtrack_*` events.

The A/B diagnostic `v7_escape_off_blocked_front_1` also passed: `ESCAPE OFF`
produced front-blocked safe-stop only, no `blind_backtrack_start`, no forward
or turn command, and `ESCAPE ON` was restored afterward.

### Turn coast calibration

Run this whenever motor bases, tyres, battery condition, or surface changes:

```text
ZERO
CSV ON
TEST ARM
TEST TURNPULSE 0.50
CSV OFF
```

`TEST TURNPULSE` applies the calibrated full turn command for the requested
signed duration (0.10–0.80 s), then logs yaw at 100 ms intervals during a one
second motor-off coast.  Positive duration uses the positive turn convention;
negative duration verifies the opposite direction.  Record the yaw at the end
of the drive phase and the final yaw.  This characterises the drivetrain and
helps flag a changed battery, tyre, or surface condition before retuning the
simple continuous approach command.

### Slow pivot pulse ladder

Run this before changing the global slow turn pulses:

```text
ZERO
TEST ARM
TEST TURNLADDER LEFT
STATUS
TEST TURNLADDER RIGHT
STATUS
TEST DISARM
```

`TEST TURNLADDER LEFT|RIGHT` directly applies raw pivot pulses at offsets
`120,160,200,240,280,300 us` from neutral `1500 us`. Each rung drives for
`250 ms`, stops/coasts for `500 ms`, and logs drive/final/coast yaw deltas.
Use the smallest symmetric offset that moves reliably in both directions before
changing `TURN_*_SLOW_US`. These slow pulse constants are global: changing
them affects `TEST TURN`, search scan turns, confirm turns, and point-alignment
slow-zone turns.

2026-07-04 ladder result on the rebuilt chassis:

| Offset | Left final yaw | Right final yaw | Notes |
| ---: | ---: | ---: | --- |
| 120 us | `+0.13 deg` | `-0.62 deg` | too weak / near noise |
| 160 us | `+1.19 deg` | `-2.75 deg` | moves, but left is marginal |
| 200 us | `+2.44 deg` | `-5.37 deg` | first clear symmetric candidate |
| 240 us | `+4.50 deg` | `-7.19 deg` | solid but only okay for search |
| 280 us | `+9.25 deg` | `-11.06 deg` | accepted search-specific default |
| 300 us | `+10.81 deg` | `-13.88 deg` | legacy global slow offset |

The `+/-200 us` result was tested as a global slow-pivot retune
(`1700/1300 us`) but real `TEST TURN` behaviour felt poor. The global slow
pivot pair was restored to `1800/1200 us`.

Weight-search scan and confirm turns now use a separate runtime offset via
`SEARCHTURN <offset_us>`, defaulting to `280 us` (`1780/1220 us` pulses). The
physical `TEST SEARCH` comparison found `200 us` too weak/draggy, `240 us`
only okay, and `280 us` the only acceptable search scan speed.

### Current rebuilt-chassis Stage 1 evidence

After the 2026-06-30 chassis rebuild and motor-side correction, short
no-reset drivetrain smoke found a real side imbalance.  A streamed
`DRIVE 60 0` pulse with the old bases produced about `339/649` left/right
encoder ticks and about `-8 deg` heading drift.  Runtime `FBASE` sweeps
selected `1935/1870 us` as the best non-overfit compromise; this is now the
compiled default.

Post-upload smoke with the compiled bases:

| Mark | Command | Result |
| --- | --- | --- |
| `new_chassis_postupload_fbase1935_1870_drive_020` | `TEST DRIVE 0.20` | `waypoint_reached`, no safe-stop, all fans valid, final heading about `-2.5 deg` |
| `new_chassis_postupload_fbase1935_1870_goto_020` | `TEST GOTO 0.20 0.00` | `waypoint_reached`, no safe-stop, all fans valid, final heading about `-2.6 deg` |
| `new_chassis_speed2300_drive_030_repeatability` | `SPEED 2300`, `TEST DRIVE 0.30` | full-speed sample observed, `waypoint_reached`, no safe-stop, all fans valid, final heading about `-0.1 deg` |
| `new_chassis_speed2500_drive_030_repeatability` | `SPEED 2500`, `TEST DRIVE 0.30` | full-speed sample observed, `waypoint_reached`, no safe-stop, all fans valid, final heading about `-1.3 deg` |
| `new_chassis_speed2600_drive_030_repeatability` | `SPEED 2600`, `TEST DRIVE 0.30` | full-speed sample observed, `waypoint_reached`, no safe-stop, all fans valid, final heading about `+1.9 deg` |

This accepts the smoke gate only.  Do not retune forward bases again from one
short run; use a longer straight repeatability set and track-width/stopping
tests first.

After these runs, a stale sign convention was fixed: positive turn is
robot-left in the current navigation frame, and the turn-side observability
check, manual-drive side validity check, desktop UI buttons, and README now
match that convention.  Recompile/upload passed after the fix.

## Stage 2: early obstacle steering, G0

Use the established `TEST AVOID 1.00` start pose.  Put a panel far enough
ahead that the planner can see it before the 180 mm emergency threshold.  The
right side must be genuinely open; there is no outer boundary in G0.

```text
ZERO
CSV ON
MARK v7_gap_G0_right
TEST ARM
TEST AVOID 1.00
STATUS
CSV OFF
```

Pass requires no contact, no fixed reverse, no fixed 60 degree turn, and a
single continuous planner response: nonzero signed curvature while the panel
is relevant, followed by a return toward the original temporary target.  The
target-line rejoin is now a map/arc decision, so there should be no
`adaptive_bypass_plan` or `adaptive_rejoin` event.

A `planner_safe_stop,no_safe_trajectory` is a **safety pass but traversal
failure**.  Record fan positions, panel dimensions, final pose, fan ages, and
the final planner CSV row before changing the algorithm.

## Stage 3: right-side reliability ladder

Run each new geometry once.  Keep obstacle shape, start pose, target, and gap
length fixed; change only the right-side outer boundary.

| Stage | Right-side space | Question |
| --- | ---: | --- |
| G1 | 700 mm | Does map-based steering avoid premature target-line re-entry? |
| G2 | 600 mm | Does the selected arc remain stable with a nearby boundary? |
| G3 | 520 mm | Does clearance speed limiting engage before emergency stop? |
| G4 | 480 mm | Does footprint uncertainty produce an appropriate safe stop? |
| G5 | 460 mm | Can it enter only if the measured geometry truly fits? |

For every run:

```text
ZERO
CSV ON
MARK v7_gap_G#_right_run1
TEST ARM
TEST AVOID 1.00
STATUS
CSV OFF
```

Record: measured space, front-panel position, panel width, gap length, final
pose, contact/no-contact, `planner_safe_stop` events, minimum swept clearance,
and whether the goal was reached.  Do not narrow the gap after a single result
without inspecting its CSV.

### Current Stage 3 evidence

As of 2026-06-26, the G1-G5 ladder has produced contact-free traversal passes
after these fixes:

- navigation/map frame uses `navigationHeadingDeg()`;
- angled inner-fan ranges are projected forward before being used as rollout
  observation limits;
- clearance-escape side choice uses both inner and outer fan rays;
- high-speed clearance escape applies a temporary speed cap and stronger
  escape-turn preference when the front range is tightening.

Important: those physical passes were on the previous chassis. On 2026-06-30
the body was rebuilt to extents `front=123 mm`, `rear=138 mm`,
`left/right=90.5 mm`, effective track width `195 mm`, and new fan origins.
The geometry has been updated in firmware and the desktop tools, and the
new-chassis simulator sweeps show no contact failures, but physical validation
must restart from smoke tests before treating G3-G5 as re-proven.

Do not treat this as a reason to loosen footprint margins. The first
repeatability gate has passed: one logged G5 repeat at the previous default and
three logged/contact-free `SPEED 2600` G5 repeats reached the waypoint.
`DEFAULT_BASE_TARGET_SPEED` is now `2600`. The route-line finish patch was
validated on 2026-06-27 with two contact-free open-floor `SPEED 3000 TEST GOTO
1.00 0.00` repeats and one corrected default-speed G5 regression ending in
`waypoint_reached`.

Suggested acceptance table:

| Gate | Command | Acceptance |
| --- | --- | --- |
| G5 previous-default repeat | `TEST AVOID 1.00` at 2300 default | Passed once, contact-free |
| G5 2600 repeat | `SPEED 2600`, then `TEST AVOID 1.00` | Passed three times, contact-free |
| Open-floor route-line finish | `SPEED 3000`, then `TEST GOTO 1.00 0.00` | Passed twice, contact-free, `route_line_reached` |
| G5 new-default confirmation | `TEST AVOID 1.00` after upload with default 2600 | Passed once, contact-free, `waypoint_reached` |
| 2600 rejection | Any front block, safe stop, stall, or contact | Inspect CSV before changing planner logic; consider backing default down to 2500 |

## Stage 4: degraded sensors

With motors stopped, cover or disconnect one fan sector at a time and run
`STATUS` plus `TEST SIDE 3`.  Then, only in a very open area, test a small
`TEST GOTO 0.20 0.00`.

- An inner fan fault must prevent forward travel (`front_invalid`).
- A right outer fault must pause a right turn for stationary revalidation,
  then forbid it if the fault persists; the equivalent applies to the left.
- The robot may continue only with an evidenced straight path and valid inner
  safety sensing.  It must never report the failed side as clear.

Run this stage before more speed tuning because the fan harness was recently
rewired. A channel that times out, stays stale, or moves to the wrong logical
sector is a safety blocker.

Recommended no-motion sequence for each covered/disconnected sector:

```text
CSV ON
MARK degraded_right_outer
STATUS
TEST SIDE 3
STATUS
CSV OFF
```

Only after the no-motion result is correct, move to an open area and run:

```text
ZERO
CSV ON
MARK degraded_small_goto_right_outer
TEST ARM
TEST GOTO 0.20 0.00
STATUS
CSV OFF
```

### Current Stage 4 evidence

On 2026-06-30, covered-sector tests passed on the rebuilt chassis:

| Covered sector | Stationary result | Movement/sweep result |
| --- | --- | --- |
| `right_outer` | right side not passable; `TEST SIDE` chose left | manual turn command refused by `turn_sweep` |
| `right_inner` | virtual front collapsed, `blocked=1`; `TEST SIDE` chose left | `TEST GOTO 0.20 0.00` safe-stopped with `front_blocked` before motion |
| `left_inner` | virtual front collapsed, `blocked=1`; `TEST SIDE` chose right | `TEST GOTO 0.20 0.00` safe-stopped with `front_blocked` before motion |
| `left_outer` | left side not passable; `TEST SIDE` chose right | manual turn command refused by `turn_sweep` |

After uncovering all sensors, `new_chassis_post_degraded_recovery_drive_020`
passed at `SPEED 2600` with all fan sensors valid and no planner safe-stop.
It ended with about `-5.7 deg` heading drift, so sensor recovery is accepted
but longer straight repeatability remains a drivetrain/control item.

## Track-width calibration gate

Before accepting G3-G5, measure an arc under a repeated manual command using
the SerialCommandUI.  Log the signed left/right distance and IMU yaw change,
then calculate:

```text
effective_track_width = abs((right_distance - left_distance) / yaw_change_radians)
```

Repeat in both directions and use a measured, documented value in
`EFFECTIVE_TRACK_WIDTH_M`.  Recompile, repeat Stage 1, then rerun only the
affected gap geometry.  Do not tune planner clearance or curvature from a
single physical run.

## Report back after a run

Provide the exact command sequence, physical layout dimensions, whether there
was contact, the final `STATUS`, and the complete CSV segment from the `MARK`
through `CSV OFF`.  The next change should address one observed failure mode,
not add a batch of compensating constants.
