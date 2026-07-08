# Obstacle Rework Recommendation

Date: 2026-07-06

## Fresh regression run

I ran the offset-wall avoidance regression as `arn1` using:

```text
cd C:\Users\marco\Documents\Robot Algorithm\SerialCommandUI
python run_navigation_regression.py --mark arn1 --test avoid --distance 1.00 --layout "offset wall current; Marco observed straight-in/shimmy/scrape edge" --timeout 45
```

Artifacts:

- `C:\Users\marco\Documents\Robot Algorithm\SerialCommandUI\logs\navigation_regressions\20260706_205515_arn1.raw.txt`
- `C:\Users\marco\Documents\Robot Algorithm\SerialCommandUI\logs\navigation_regressions\20260706_205515_arn1.csv`
- `C:\Users\marco\Documents\Robot Algorithm\SerialCommandUI\logs\navigation_regressions\20260706_205515_arn1.summary.json`

Terminal result: timeout, then runner disarmed the test. Final status was motor-neutral, `state=END_MATCH`, all fan sensors valid, `blocked=0`, but `plannerStop=aborted` because the runner ended the test.

Summary from the saved JSON:

- `terminal_event`: `timeout`
- reverse recovery starts: 6
- reverse recovery ends: 5
- planner safe stops: 16
- min clearance: `-24.8 mm`
- max forward command: `2600 ticks/s`
- max turn command: `1181.8 ticks/s`

This matches Marco's observation: the robot did not fail because it turned into the wall. It first walked forward into the wall pocket, then repeatedly reversed and re-entered.

## Key evidence

Early run, before first front block:

```text
x=0.000 y=0.000 theta=0.00 front=466 left=466 right=231 v=1920.9 w=624.3
x=0.128 y=0.032 theta=19.94 front=344 left=413 right=344 v=1852.0 w=0.0
x=0.168 y=0.046 theta=19.69 front=292 left=367 right=292 v=1566.0 w=-508.9
x=0.185 y=0.052 theta=19.50 front=275 left=330 right=275 v=1500.0 w=-975.0
x=0.214 y=0.059 theta=10.75 front=249 left=278 right=249 v=1500.0 w=0.0
x=0.265 y=0.062 theta=7.69 front=198 left=207 right=198 v=1500.0 w=975.0
front_blocked at x=0.265 y=0.062 theta=7.69 front=178
```

Interpretation:

- The robot knew from the start that right-side space was restricted and left was more open.
- It did begin a leftward escape arc.
- As the front wall got close, the generic arc scoring still allowed straight or opposite/right-turn commands.
- Because `PLANNER_MIN_DRIVABLE_SPEED_TPS` is `1500`, the speed cap could not slow the robot below a real forward crawl. It kept moving into the wall pocket until the front stop band tripped.

Repeated recovery evidence:

```text
reverse_recovery_start x=0.295 y=0.073 theta=26.31 front=158
post_reverse_escape_start side=left right=0.35 left=8.19 extra=0.08
reverse_recovery_start x=0.317 y=0.137 theta=29.19 front=134
post_reverse_escape_start side=left right=0.34 left=0.78 extra=0.16
reverse_recovery_start x=0.315 y=0.210 theta=21.37 front=130
post_reverse_escape_start side=left right=0.37 left=0.81 extra=0.16
reverse_recovery_start x=0.322 y=0.286 theta=22.94 front=121
post_reverse_escape_start side=left right=0.36 left=0.63 extra=0.24
```

This is the shimmy: reverse recovery works, but the forward planner keeps accepting a path that is still too close to the same pocket.

## Root cause

The current side-escape lane exists, but it is not committed strongly enough.

There are two specific issues:

1. `candidateScore()` has only a soft `escapeTurnScore`. Under urgent side escape, generic progress/heading scoring can still choose straight or opposite-turn arcs. That is exactly what the early telemetry shows.

2. `post_reverse_escape` resets side-escape adaptation when side evidence clears. In this layout, that can erase the widening memory too early. The robot then rediscovers the same obstacle and starts again from a shallow lane instead of keeping the learned wider bypass for the same wall.

## Source changes made

Compiled source changes are now in:

- `LocalPlanner.cpp`
- `RobotConfig.h`

Patch intent:

1. Add hard escape commitment once side-escape urgency is high.

New constants in `RobotConfig.h`:

```cpp
const float PLANNER_ESCAPE_FORCE_TURN_URGENCY = 0.60;
const float PLANNER_ESCAPE_FORCE_TURN_MIN_RATIO = 0.15;
```

New filter in `selectTrajectory()`:

```cpp
if (clearanceEscapeLocalGoalActive &&
    escapeUrgency >= PLANNER_ESCAPE_FORCE_TURN_URGENCY &&
    desiredEscapeTurnSign != 0.0f) {
  float turnRatio = turn / max(1.0f, forward);
  bool committedToEscapeSide =
    turnRatio * desiredEscapeTurnSign > 0.0f &&
    fabs(turnRatio) >= PLANNER_ESCAPE_FORCE_TURN_MIN_RATIO;
  if (!committedToEscapeSide) {
    continue;
  }
}
```

Effect: once the robot is urgently escaping left, straight/right arcs are no longer allowed to win just because they make short-term waypoint progress. If no left-committed arc is safe, the planner safe-stops and enters reverse recovery earlier, before burying the front sensor.

2. Preserve side-lane adaptation after `post_reverse_escape` reports `side_clear`.

Removed the premature `resetSideEscapeAdaptation()` from the `post_reverse_escape_end,side_clear` path. Adaptation still resets when a new navigation goal starts, finishes, or the committed side flips.

Effect: a same-wall retry should keep the learned wider lane instead of shrinking back to the first shallow lane.

## Verification status

- Arduino CLI compile passed after the source changes.
- Upload did not clearly complete. Arduino CLI reported:

```text
No Teensy boards were found on any USB ports of your computer.
Please press the PROGRAM MODE BUTTON on your Teensy to upload your sketch.
New upload port: COM13 (serial)
```

Serial still reports `BUILD V7-local-planner`, so the build label cannot prove whether the patched firmware is running. Treat the patch as compiled but not physically validated yet.

## Next action

Press the Teensy program button, upload again, then rerun the same short mark:

```text
cd "C:\Users\marco\Documents\Robot Algorithm"
arduino-cli upload -p COM13 --fqbn teensy:avr:teensy40 RobotCode

cd "C:\Users\marco\Documents\Robot Algorithm\SerialCommandUI"
python run_navigation_regression.py --mark arn2 --test avoid --distance 1.00 --layout "offset wall current; escape-turn hard filter" --timeout 45
```

Expected improvement:

- no early straight/right crawl once front is under about 300 mm and side escape is urgent;
- fewer reverse-recovery cycles;
- if the same pocket is reached again, `side_escape_widen` should not collapse back to a shallow lane after `side_clear`.

Accept only with physical no-contact/no-scrape observation plus saved CSV evidence.

## ARN3 result after longer escape latch

Date: 2026-07-06

Run command:

```text
cd C:\Users\marco\Documents\Robot Algorithm\SerialCommandUI
python run_navigation_regression.py --mark arn3 --test avoid --distance 1.00 --layout "offset wall current; longer escape latch plus debug" --timeout 45
```

Artifacts:

- `C:\Users\marco\Documents\Robot Algorithm\SerialCommandUI\logs\navigation_regressions\20260706_211127_arn3.raw.txt`
- `C:\Users\marco\Documents\Robot Algorithm\SerialCommandUI\logs\navigation_regressions\20260706_211127_arn3.csv`
- `C:\Users\marco\Documents\Robot Algorithm\SerialCommandUI\logs\navigation_regressions\20260706_211127_arn3.summary.json`

Result: the avoid test completed with `test_avoid_end,waypoint_reached`.

Summary:

- reverse recovery starts: 0
- reverse recovery ends: 0
- front blocked events: 0 in summary/key scan
- candidate reject events: 0
- planner safe stops: 10, all `no_safe_trajectory`, mostly `diagonal_clearance`
- final reported pose: `x=1.054 y=0.029 theta=-46.56`
- final planner stop: `none`
- final reason: `best_safe_arc`

Important behavior change from ARN2:

- The robot no longer drove straight into the wall pocket and did not enter reverse-recovery shimmy.
- The longer escape latch held the around-wall behavior long enough to clear the first obstacle area.
- It made a large side excursion after clearing the wall, reaching roughly `x=2.98 y=0.86` before returning toward the waypoint. That may be acceptable for proving obstacle escape but is too wide/energetic for final tuning.

Caution:

- The test script declares success by waypoint reached, not by physical no-contact/no-scrape observation. Marco should visually confirm whether the robot actually cleared without contact.
- The next likely tuning target is post-clear return behavior: reduce over-wide excursion and smooth rejoining the nominal route after the wall is safely behind it.

## Post-clear jerk diagnosis and fix

Date: 2026-07-06

Marco observed that ARN3 cleared the wall well, then behaved weirdly/jerkily after the clear. The ARN3 CSV confirms this.

Evidence:

- First target-plane crossing: about `x=1.001 y=0.427 theta=8.31`, still far from the nominal route line.
- The strict route finish gate required lateral error <= `0.16 m`, so it did not finish.
- The missed-route abort also did not fire because the robot was outside the normal lateral corridor.
- The robot then chased the exact waypoint from a large side offset, driving out to about `x=2.98 y=0.86` before looping back.
- During that loop it repeatedly hit `diagonal_clearance` immediate stops and sharp command sign flips, which matches the observed jerky motion.

Root cause:

The side-escape patch successfully created an around-wall detour, but after the wall was cleared the route-completion logic treated the detour as an ordinary waypoint miss. It kept trying to recover to the exact original point instead of accepting that the forward task had been completed after a deliberate obstacle detour.

Patch added:

- `clearanceEscapeRouteDetourActive` latch in `LocalPlanner.cpp`, set whenever a clearance side escape is used and reset on goal start/end.
- `routeLineDetourReached()` finish gate.
- New constants in `RobotConfig.h`:

```cpp
const float PLANNER_SIDE_ESCAPE_ROUTE_FINISH_LATERAL_M = 0.55;
const float PLANNER_SIDE_ESCAPE_ROUTE_FINISH_HEADING_DEG = 70.0;
```

Behavior intent:

After a side-escape detour, if the robot crosses the original target plane while within a reasonable detour corridor and not facing wildly away, finish the point goal as `route_line_detour_reached` instead of chasing the exact point and creating a post-clear S-turn loop.

Verification:

- Firmware compile passed with Arduino CLI after the patch.
- Next hardware run should use mark `arn4` and confirm it stops shortly after the clean wall clear instead of roaming out to `y≈0.86`.

## ARN4/ARN4b detour finish verification

Date: 2026-07-06

First attempted run after upload:

- `20260706_212149_arn4.*`
- Invalid as a motion test: upload/reset startup text consumed the session and the runner timed out before producing telemetry rows.

Valid rerun after the board was ready:

```text
python run_navigation_regression.py --mark arn4b --test avoid --distance 1.00 --layout "offset wall current; detour finish gate rerun after upload" --timeout 45
```

Artifacts:

- `C:\Users\marco\Documents\Robot Algorithm\SerialCommandUI\logs\navigation_regressions\20260706_212253_arn4b.raw.txt`
- `C:\Users\marco\Documents\Robot Algorithm\SerialCommandUI\logs\navigation_regressions\20260706_212253_arn4b.csv`
- `C:\Users\marco\Documents\Robot Algorithm\SerialCommandUI\logs\navigation_regressions\20260706_212253_arn4b.summary.json`

Result:

- terminal event: `test_avoid_end,route_line_detour_reached`
- reverse recovery starts: 0
- final pose: `x=0.948 y=0.463 theta=1.00`
- final stop: `none`
- final sensors: open fan/front readings, motors neutral
- maximum side excursion stayed around `y≈0.46`, not the ARN3 `y≈0.86` loop.

Caution:

- There were still 6 `no_safe_trajectory` / `diagonal_clearance` safe-stop events during the wall-edge pass.
- Summary min clearance was `-18.2 mm`, which means the model still thinks the diagonal envelope can clip in at least one frame. Physical observation is required before accepting this as no-scrape.

Conclusion:

The detour finish gate fixed the post-clear runaway/loop behavior. The remaining tuning issue is local edge smoothness near the wall, especially the diagonal-clearance guard and command reversals during the closest pass.

## ARN6 failure analysis and bounded rejoin fix

Date: 2026-07-06

Observed failure:

- `arn6` aborted with `test_avoid_abort,point_align_sweep_sensor_invalid`.
- Final pose was approximately `x=2.738 y=0.976 theta=66`, far beyond the `x=1.000 y=0.000` target.
- This confirmed the side-escape detour latch had become a broad navigation override rather than a temporary wall-clear mode.

Root cause in code:

- `clearanceEscapeRouteDetourActive` stayed true after the obstacle was cleared.
- While true, `updatePointGoal()` forced `buildRouteLineLocalGoal(...)` even after the robot was near/past the target plane.
- The same flag suppressed normal `route_line_reached`, `route_line_overshoot_reached`, and `route_line_missed` guards.
- `sideEscapeShouldBypassPointAlignment()` also delayed normal point-alignment until too late.

Implemented cleanup:

- Added `PLANNER_SIDE_ESCAPE_FINAL_APPROACH_M = 0.30`.
- Added `sideEscapeRejoinActive(...)` so route-line rejoin after an escape is only active before the final approach window.
- Added `clearSideEscapeDetourIfActive(...)` to explicitly end the temporary detour state.
- Scoped point-alignment bypass to active route rejoin only, not the whole side-escape memory lifetime.
- Re-enabled the normal route-line finish/overshoot/missed guards during post-detour navigation; only active local clearance escape suppresses them now.
- Scoped the rejoin speed cap to active clearance/post-reverse/rejoin modes instead of any stale detour latch.

Compile verification:

```text
arduino-cli compile --fqbn teensy:avr:teensy40 RobotCode
```

Result: compile passed.

Expected next test mark: `arn7`.
