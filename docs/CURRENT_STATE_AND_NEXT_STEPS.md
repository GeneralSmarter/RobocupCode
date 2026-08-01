# Current State And Next Steps

Status: current active firmware documentation as of 2026-07-30.

This file is the short handoff for new work. For the deeper safety audit, read
`ROBOT_CODEBASE_AUDIT.md`. For the implemented cleanup and split record, read
`CODE_CLEANUP_AND_LOCAL_PLANNER_SPLIT.md`. Historical physical-validation
workstreams live under `archive/`.

## Current Architecture

`RobotCode/` is the active Teensy 4.0 firmware. Navigation is now a
nonblocking black box for mission and future pickup code:

```cpp
if (navigationGoTo(x, y)) {
  // poll navigationGetStatus() until REACHED or FAILED
}
```

Normal mission and pickup code should include only `Navigation.h`, submit one
goal, poll `navigationGetStatus()`, then call `navigationClearResult()` after
handling the result. Navigation owns planning, map state, obstacle context,
recovery, and typed failure reasons. It does not own route advancement,
mission transitions, or pickup actions.

Implementation ownership:

- `src/mission/` owns the route, route index, mission actions, and weight
  search flow.
- `src/navigation/` owns goal storage, point/turn control, planner context,
  map, collision checks, obstacle context, forward rollout, recovery, and
  read-only planner diagnostics.
- `src/motion/` owns odometry, encoder/progress helpers, and
  `MotorControl.cpp`, the only periodic motor-output writer.
- `src/sensors/` owns navigation ToF, object ToF, rear obstacle, and IMU
  sampling.
- `src/operator/` owns Bluetooth commands, telemetry, and stationary
  diagnostics.
- `src/core/` owns shared runtime storage, scheduling helpers, and controller
  ordering.

Root headers remain the visible shared/public surface: `Robot.h`,
`RobotTypes.h`, `RobotConfig.h`, `Navigation.h`, `NavigationAdmin.h`,
`NavigationTest.h`, `MotionSafety.h`, and `TurnConvention.h`.

## Verified Software Baseline

Latest software validation:

- warning-enabled Teensy 4.0 compile: PASS;
- firmware/WASM and simulator suite: 47/47 PASS;
- Python suite: 141 PASS, one intentional skip;
- Python `compileall`: PASS;
- retained custom and `heading-back` scenarios pass under clean and
  website-default sensing;
- rebuilt-WASM Visual Lab clear GOTO smoke test passed with
  `waypoint_reached`, neutral final state, and no simulator contact.

No upload, serial connection, arming, or physical movement was performed during
the architecture cleanup. Simulator evidence is deterministic software
evidence only.

## Current Capability

- `START` runs the onboard calibration route from `RouteMission.cpp`.
- `navigationGoTo()`, `navigationGoToPickup()`, `navigationTurnBy()`, and
  `navigationScanTurnBy()` are the supported mission-facing driving commands.
- Bluetooth test/admin commands use `NavigationTest.h` and
  `NavigationAdmin.h`; those headers are not for normal mission code.
- The Visual Lab simulator runs the firmware planner through WASM. The legacy
  JavaScript planner has been removed, so there is one current planner
  implementation.
- Weight search can run explicit search scans, route search actions, and
  route-only opportunistic interrupts for fresh confirmed `weight_sized`
  targets.

## Current Safety Boundaries

- `MotorControl.cpp` remains the sole periodic servo writer.
- All non-neutral motion must go through authorized motion commands and the
  final motor safety gate.
- Invalid, stale, blind, or unknown direction-relevant safety evidence must not
  be treated as clear.
- The front fan has no true side or low central coverage. Physical clearance
  remains an uncertainty even when software tests pass.
- Physical reverse requires trusted SEN0628 rear coverage through
  `hasTrustedRearCoverage()`. The legacy name `RANGE_FAKE_REAR` is now only a
  compatibility slot for that rear obstacle state.
- Field GOTO in the desktop UI is preview-only. It must not send `TEST ARM` or
  `TEST GOTO` until an arena-to-odom `SE(2)` transform, bounds preview, status
  checks, and explicit confirmation flow exist.
- Do not upload firmware, open serial, arm, or move the robot without explicit
  current-task permission and the repository safety gates.

## Immediate Next Work

The next feature work should be outside navigation:

1. Treat navigation as a stable driving service.
2. Add pickup/payload state machines against `Navigation.h`, not planner
   internals.
3. Build the smallest measured scoring loop: search, approach, confirm
   payload, return, dock/unload, and repeat.
4. Keep object sensors as perception/candidate evidence only; they must not
   weaken travel safety.
5. Collect a real object/dummy/wall/ramp dataset before tuning material
   classification.
6. Continue improving physical safety evidence for low central, side, rear,
   watchdog timing, and estimator health before broader autonomy.

## Do Not Do Yet

- Do not reopen the local planner unless a reproduced current failure proves
  the black-box navigation behavior is the blocker.
- Do not reintroduce direct route access, planner-private headers, or motor
  writes from mission/pickup code.
- Do not rely on old `LocalPlanner.cpp`, `StateMachine.cpp`, or workstream
  instructions as current file paths.
- Do not treat simulator completion as physical readiness.
