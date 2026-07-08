# Navigation Rework Session Summary - 2026-07-06

## Context

Marco observed that the robot was not using its sensor information intelligently around an offset wall. It drove straight into the obstacle, shimmied forward/backward, scraped close to the edge, and only eventually escaped. The goal for this session was to make obstacle avoidance proactive, preserve the real waypoint target, and avoid both post-wall jerk loops and runaway detours.

Project paths:

- Firmware: `C:/Users/marco/Documents/Robot Algorithm/RobotCode`
- Regression runner: `C:/Users/marco/Documents/Robot Algorithm/SerialCommandUI/run_navigation_regression.py`
- Main changed files:
  - `RobotConfig.h`
  - `LocalPlanner.cpp`
- Main diagnostic note:
  - `docs/OBSTACLE_REWORK_RECOMMENDATION.md`

## High-level result

The final tested firmware behavior is good for the offset wall case:

- `TEST AVOID 1.000` passed with `waypoint_reached`.
- `TEST AVOID 2.000` passed with `waypoint_reached`.
- The robot cleared the wall without reverse recovery.
- The robot returned to the actual waypoint tolerance instead of accepting a broad detour-plane finish.
- The previous runaway behavior, where the robot drove far beyond the waypoint, was removed.

Latest successful regression logs:

- 1 m avoid:
  - `C:/Users/marco/Documents/Robot Algorithm/SerialCommandUI/logs/navigation_regressions/20260706_215425_arn7.summary.json`
  - Result: `test_avoid_end,waypoint_reached`
  - Final pose: `x=1.028 y=0.049 theta=-80.50`
- 2 m avoid:
  - `C:/Users/marco/Documents/Robot Algorithm/SerialCommandUI/logs/navigation_regressions/20260706_215617_arn7d2.summary.json`
  - Result: `test_avoid_end,waypoint_reached`
  - Final pose: `x=1.977 y=0.053 theta=-28.94`

## Main diagnosis

The original wall behavior was not just a clearance constant problem. It was a planner-policy problem.

The robot could see enough side/front ToF information, but the planner was too reactive:

1. It let the waypoint objective pull it straight toward the wall pocket.
2. It waited too long before committing to a side escape.
3. Reverse recovery and side escapes could start, but the route objective could then retake control too early.
4. After clearing the wall, strict route-line completion and direct point-chase could create large sideways excursions or jerk loops.

The clean final architecture should be:

```text
SIDE_ESCAPE -> ROUTE_REJOIN -> FINAL_POINT_APPROACH -> waypoint_reached
```

Not:

```text
SIDE_ESCAPE -> sticky detour override forever
```

## Changes made

### 1. Proactive side-escape memory

Added/used state around a committed side escape:

- `lastClearanceEscapeSideSign`
- `lastClearanceEscapeMs`
- `clearanceEscapeLocalGoalActive`
- side-escape evidence from the fan/side sensors

Added config:

```cpp
const float PLANNER_PROACTIVE_ESCAPE_EXTRA_LATERAL_M = 0.12;
const unsigned long PLANNER_PROACTIVE_ESCAPE_HOLD_MS = 1800;
```

Purpose:

- Detect when one side is becoming restricted and the other side is more open.
- Commit early to a bypass lane before the robot gets pinned against the wall.
- Hold the escape choice long enough that ordinary waypoint attraction does not immediately undo it.

Achieved:

- Stopped the most reactive straight-into-wall behavior.
- Gave the robot a stable open-side preference.

### 2. Escape turn commitment filter

Added config:

```cpp
const float PLANNER_ESCAPE_FORCE_TURN_URGENCY = 0.60;
const float PLANNER_ESCAPE_FORCE_TURN_MIN_RATIO = 0.15;
```

Changed `selectTrajectory()` so that, during urgent side escape, candidates that are straight or turning away from the committed escape side are rejected.

Purpose:

- Prevent the generic point-progress scorer from choosing arcs that look good for the waypoint but walk the chassis back into the wall pocket.

Achieved:

- Made the side escape a real commitment rather than a soft preference.
- Reduced wall-hugging and shimmying.

### 3. Corrected escape turn sign source

Earlier logic inferred the desired escape turn direction from the body-frame bearing to the local goal. That was flawed. Once the chassis was already angled toward the bypass lane, the local goal could appear on the opposite side of the robot, making the planner steer back toward the wall.

Changed logic so urgent side escape uses the committed side sign directly:

```cpp
float committedSideSign = postReverseEscapeActive
  ? postReverseEscapeSideSign
  : lastClearanceEscapeSideSign;
```

Purpose:

- Keep turning toward the selected open side while escaping.

Achieved:

- Fixed the behavior where the robot initially aimed around the wall, then turned back toward the waypoint/wall pocket.

### 4. Extended post-reverse escape latch

Changed config:

```cpp
const unsigned long PLANNER_POST_REVERSE_ESCAPE_MIN_MS = 1800;
const unsigned long PLANNER_POST_REVERSE_ESCAPE_TIMEOUT_MS = 5000;
```

Purpose:

- If reverse recovery is used, keep the robot committed long enough to actually exit the bad pose.

Achieved:

- Helped prevent rapid reverse/forward shimmy cycles.
- Latest successful runs did not need reverse recovery.

### 5. Disabled corridor squeeze during active side escape

Changed the straight/corridor squeeze logic so it does not override active clearance escape.

Purpose:

- Corridor squeeze is useful for aligned narrow gaps, but harmful when the robot is escaping a wall edge.

Achieved:

- Reduced wall-hugging during side escape.

### 6. Added debug telemetry for escape candidate rejection

Expanded candidate reject telemetry with escape commitment details, including:

- `escape_commit`
- `urg`
- `side`

Purpose:

- Make future regression logs explain whether the committed escape filter is actually active.

Achieved:

- Improved diagnosis without needing to guess from motion alone.

### 7. Rejected broad detour-plane finish as the final solution

An intermediate fix added a broad route-detour finish gate using:

```cpp
PLANNER_SIDE_ESCAPE_ROUTE_FINISH_LATERAL_M = 0.55;
PLANNER_SIDE_ESCAPE_ROUTE_FINISH_HEADING_DEG = 70.0;
```

That allowed `route_line_detour_reached`, but it was a bad semantic fix because the robot could stop far from the actual waypoint.

Observed issue:

- `arn4b` finished around `x=0.948 y=0.463` for a 1 m target.
- That was approximately 0.466 m from the actual waypoint.

Decision:

- Do not accept broad target-plane crossing as success.
- Preserve the actual waypoint semantics.

Status:

- This broad detour-finish approach was removed/replaced by bounded rejoin plus normal `waypoint_reached`.

### 8. Bounded side-escape route rejoin

Added config:

```cpp
const float PLANNER_SIDE_ESCAPE_FINAL_APPROACH_M = 0.30;
```

Added helper:

```cpp
static bool sideEscapeRejoinActive(float routeLengthM, float routeUx, float routeUy) {
  if (!clearanceEscapeRouteDetourActive || routeLengthM <= 0.0f) {
    return false;
  }

  float alongM = routeLineAlongM(robotX, robotY, routeUx, routeUy);
  return alongM < routeLengthM - PLANNER_SIDE_ESCAPE_FINAL_APPROACH_M;
}
```

Added cleanup:

```cpp
static void clearSideEscapeDetourIfActive(const char* reason) {
  if (!clearanceEscapeRouteDetourActive) {
    return;
  }
  clearanceEscapeRouteDetourActive = false;
  sendBluetoothEvent("side_escape_final_approach", reason);
}
```

Changed local-goal selection from the bad sticky override:

```cpp
if (routeLineEligible || (clearanceEscapeRouteDetourActive && routeFrameValid)) {
```

to the bounded version:

```cpp
if (routeLineEligible || detourRejoinActive) {
```

Purpose:

- Let the robot rejoin the original route after an obstacle.
- Prevent the detour latch from extending the mission past the waypoint.
- Hand back to normal point approach in the last 0.30 m.

Achieved:

- Fixed the `arn6` failure where the robot drove to about `x=2.738 y=0.976` for a 1 m target.
- Preserved the good wall-clear behavior while restoring waypoint completion.

### 9. Restored normal finish / overshoot / missed guards

Removed broad suppression of route-line guards while `clearanceEscapeRouteDetourActive` was true.

Bad intermediary behavior:

```cpp
if (!clearanceEscapeRouteDetourActive && ... routeLineGoalReached(...))
```

Cleaned behavior:

```cpp
if (!clearanceEscapeLocalGoalActive && routeLineEligible &&
    routeLineGoalReached(...))
```

Purpose:

- Only active local clearance escape should suppress route finish guards.
- A stale or broad detour flag should not disable safety rails.

Achieved:

- Prevented runaway detour behavior.
- Normal finish/missed semantics are back after the actual obstacle escape.

### 10. Scoped point-alignment bypass

Changed point-alignment bypass so it only applies while active route rejoin is happening:

```cpp
bool sideEscapeBypassAlignment =
  detourRejoinActive && sideEscapeShouldBypassPointAlignment();
```

Purpose:

- During wall escape/rejoin, avoid snapping back toward the point too early.
- Near/past the final approach window, allow normal point alignment again.

Achieved:

- Prevented point alignment from being delayed until the target was far behind.

### 11. Rejoin speed cap

Added config:

```cpp
const float PLANNER_SIDE_ESCAPE_REJOIN_MAX_SPEED_TPS = 1700.0;
```

Currently used to limit speed during:

```cpp
clearanceEscapeLocalGoalActive || postReverseEscapeActive || sideEscapeRejoinActive(...)
```

Purpose:

- Reduce the violent post-wall S-turn/jerk loop seen after early successful wall-clear runs.

Achieved:

- Made wall clear and rejoin much more controlled.
- Contributed to the successful `arn7` and `arn7d2` results.

Current concern:

- This cap is probably too broad because `sideEscapeRejoinActive(...)` stays active for most of longer avoid routes.
- In the 2 m test, most telemetry was capped around 1500-1700 ticks/s and only a few rows reached 2600 ticks/s.

Log analysis from `arn7d2`:

```text
x 0.00-0.35: avg_v=1597, max_v=1700
x 0.35-0.70: avg_v=1602, max_v=1700
x 0.70-1.20: avg_v=1638, max_v=1700
x 1.20-1.70: avg_v=1640, max_v=1700
x 1.70-2.10: avg_v=1762, max_v=2600
```

Most likely cleanup later:

```cpp
if (clearanceEscapeLocalGoalActive || postReverseEscapeActive) {
  requestedCap = min(requestedCap, PLANNER_SIDE_ESCAPE_REJOIN_MAX_SPEED_TPS);
}
```

That removes `sideEscapeRejoinActive(...)` from the speed cap condition. The robot would stay gentle while actively escaping the wall, then recover normal route speed once it is simply rejoining/following the route.

## Regression timeline

### ARN1

Change:

- Initial proactive side-escape memory.

Result:

- Failed.
- Robot still drove into/near the wall and got trapped in recovery-like behavior.

Lesson:

- Side escape as a soft preference was not enough.

### ARN2

Change:

- Added urgent side-escape hard filter against straight/opposite-turn candidates.

Result:

- Failed.
- Better lateral behavior, but still got pulled back toward the wall/waypoint pocket.

Lesson:

- The desired turn sign source was wrong.

### ARN3

Change:

- Desired escape turn sign now follows committed side sign.
- Escape latch was lengthened.
- Corridor squeeze disabled during side escape.

Result:

- Passed wall clearing.
- No reverse recovery.
- But post-clear route produced large side excursion and jerky behavior.

Lesson:

- Wall avoidance was now good enough; post-clear rejoin/finish semantics were the next problem.

### ARN4 / ARN4b

Change:

- Tried broad detour finish gate.

Result:

- `arn4b` technically finished, but stopped too far from the real waypoint.

Lesson:

- Broad route-plane finish is not acceptable. The robot must return to the original waypoint.

### ARN5b

Change:

- Removed broad detour finish.
- Added rejoin speed cap.

Result:

- Passed, but finished via route-line overshoot around `x=1.211 y=0.029` for a 1 m target.

Lesson:

- Better, but still not preserving exact waypoint enough.

### ARN6

Change:

- Forced route-line navigation while detour latch was active and suppressed guards.

Result:

- Failed badly.
- Robot drove far beyond the target, around `x=2.738 y=0.976` for a 1 m target.

Lesson:

- Sticky detour latch was the wrong abstraction.
- Needed bounded phase handoff instead.

### ARN7

Change:

- Bounded side-escape rejoin.
- Restored normal guards.
- Scoped point-alignment bypass.

Result:

- Passed 1 m avoid.
- Final: `x=1.028 y=0.049`.
- Terminal event: `waypoint_reached`.

Lesson:

- This is the first behavior that satisfied the core requirement: clear wall and reach the real waypoint.

### ARN7D2

Change:

- Same firmware, 2 m test.

Result:

- Passed 2 m avoid.
- Final: `x=1.977 y=0.053`.
- Terminal event: `waypoint_reached`.
- No reverse recovery.
- No safe-stop events in the summary.

Lesson:

- Generalizes to a longer target, but speed cap feels conservative.

## Current known issues / watch items

### 1. It moves slowly after wall-clear

Most likely cause:

```cpp
sideEscapeRejoinActive(...)` is included in the 1700 ticks/s speed cap condition.
```

This means long avoid goals can stay capped until the last 0.30 m before the target.

Suggested later cleanup:

```diff
- if (clearanceEscapeLocalGoalActive || postReverseEscapeActive ||
-     sideEscapeRejoinActive(routeLengthM, routeUx, routeUy)) {
+ if (clearanceEscapeLocalGoalActive || postReverseEscapeActive) {
    requestedCap = min(requestedCap, PLANNER_SIDE_ESCAPE_REJOIN_MAX_SPEED_TPS);
  }
```

Expected effect:

- Still controlled during actual wall escape.
- Normal speed during route rejoin once clear.

Risk:

- Could reintroduce some post-clear jerk if final approach is still aggressive, but bounded rejoin should now carry most of that responsibility cleanly.

### 2. `side_escape_final_approach` can fire more than once

In the 2 m log, the final-approach event fired multiple times. That implies the detour state can be cleared, then reactivated later by fresh side-escape evidence.

Current status:

- Not blocking. The run still passed cleanly.

Possible later improvement:

- Add a small cooldown or only allow one route-detour latch per waypoint unless a new hard obstacle/recovery condition appears.

Do not fix this unless it becomes observable bad behavior. Avoid adding more state machine complexity prematurely.

### 3. Final heading is not controlled

The robot finishes by position tolerance:

- ARN7: final heading about `-80.50 deg`.
- ARN7D2: final heading about `-28.94 deg`.

Current status:

- Acceptable for `TEST AVOID`, because the goal is reaching the point after avoiding the wall.

If final heading matters later:

- Treat it as a separate requirement and add a final heading goal, not as part of obstacle rejoin.

### 4. Physical clearance still needs observation

ARN7 had reported minimum model clearance around `-16.3 mm`, while ARN7D2 had minimum model clearance around `22.0 mm`.

Current status:

- The physical result looked good to Marco.
- Still worth watching for scrape/contact during repeated tests.

## Recommended next steps

1. Keep the current bounded rejoin behavior as the baseline.
2. If speed remains too slow, remove `sideEscapeRejoinActive(...)` from the rejoin speed cap condition.
3. Retest:

```bash
python run_navigation_regression.py --mark arn8 --test avoid --distance 1.00 --layout "offset wall current; uncapped route rejoin" --timeout 40
python run_navigation_regression.py --mark arn8d2 --test avoid --distance 2.00 --layout "offset wall current; uncapped route rejoin 2m" --timeout 55
```

4. Do not add broad finish gates again. The robot must finish at the actual waypoint.
5. Do not add more sticky global flags unless they have explicit entry/exit conditions.

## Final design principle from this session

Obstacle avoidance should be a temporary policy overlay, not a replacement for waypoint navigation.

The clean model is:

```text
avoid the wall while near the wall;
rejoin the route while before the target;
return to ordinary waypoint logic for the final point.
```

That is what made ARN7 and ARN7D2 work.
