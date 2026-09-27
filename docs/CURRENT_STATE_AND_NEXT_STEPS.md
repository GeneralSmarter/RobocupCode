# Current State and Retained Work

Status: current active firmware summary as of 2026-08-11.

Read `../../HANDOFF.md` for the authoritative workspace status and evidence.
Read `README.md` in this directory for documentation classification. Older
audits and implementation plans are context, not an automatic backlog.

## Current architecture

`RobotCode/` is the active Teensy 4.0 firmware. Navigation is a nonblocking
driving service: mission code submits one goal through `Navigation.h`, polls
its typed status, handles the result, and clears it. Navigation owns private
planning, map, obstacle, collision, recovery, and goal state; mission code owns
route progress, search actions, competition retry policy, and what happens
after a result.

Implementation ownership:

- `src/mission/` — route, mission transitions, weight search, competition
  policy.
- `src/navigation/` — goal façade, planner, map, collision, obstacle context,
  recovery, and diagnostics.
- `src/motion/` — odometry, encoders, progress monitoring, and the sole
  periodic motor writer.
- `src/sensors/` — four-ray forward fan, front 8x8 matrix, three-rear ToF
  array, object perception, and IMU.
- `src/operator/` — Bluetooth commands, telemetry, and stationary diagnostics.
- `src/core/` — runtime storage and scheduling.

## Current capability

- `START` runs the onboard route from `RouteMission.cpp`.
- Route travel supports explicit weight-search actions and opportunistic
  interrupts for fresh confirmed static matrix tracks.
- Matrix pickup tracking can steer to the target, publish an assumed funnel
  handoff, maintain the configured feed, and resume route context. An
  independent internal-funnel bottom VL53L1X can confirm a present,
  unclassified payload from fresh consecutive near readings during that feed.
  It does not confirm material or payload count, and it cannot affect motion.
- `COMPETITION ON` can retry route and return-home navigation failures after a
  neutral handoff and continue after failed optional searches. It never
  bypasses safety gates or `STOP`.
- The browser simulator uses the firmware planner through generated WASM; there
  is no separate current JavaScript navigation implementation.

## Current sensing

- Forward navigation: four VL53L0X fan rays at ±20 and ±60 degrees.
- Supplemental front safety/perception: SEN0628 8x8 matrix.
- Reverse evidence: three independent rear VL53L1X channels with fail-closed
  aggregation.
- Payload evidence only: one internal-funnel bottom VL53L1X at the provisional
  `(-38.8, 0, 25) mm` mount. It is not navigation or safety evidence.
- Position: wheel-encoder translation and BNO055 heading. TOF currently affects
  map/safety evidence but does not correct pose.

## Physical and software status

The operator reports that the current robot has been run physically and works.
This statement supersedes old blanket claims that the current architecture has
never been physically run. It is not standing authorization for a serial
session, arming, or motion test. Firmware upload is separately pre-authorized
(operator decision, 2026-08-12): upload during physical testing, but not during
code-only work.

Non-hardware checks for the payload integration, re-run 2026-09-21:

- Teensy 4.0 compile: PASS, 169,784 bytes FLASH code.
- Python tests: 151 passed, 2 skipped.
- Python `compileall`: PASS.
- Focused firmware-backed pickup/payload simulator tests: PASS, 8 passed and
  0 failed.
- Full current-worktree simulator suite: 64 passed and 13 failed. All 13 are
  general planner scenarios outside the pickup/payload path; this is an open
  broad-suite regression state, not a full-suite pass.

The current RobotCode/bridge source hash and recorded artifact source hash
match across 115 source files:
`47ae3c8ea9c1188dd5ad72debf0d3454219590250d198f8c15f74365c079ab26`.
The build metadata was generated at `2026-09-20T12:02:22.5277848Z`. That
timestamp advances on every rebuild even when the source hash is unchanged, so
only the hash is evidence of parity.

## Retained future initiatives

Only these three initiatives are retained. Starting either remaining software
initiative or the payload sensor's physical characterization still requires an
explicit user request:

1. `TOF_DOMINANT_WALL_ANCHORED_LOCALIZATION_PLAN.md`.
2. `IMU_AIDED_POSITION_TRACKING_PLAN.md`, as the prediction layer for item 1.
3. `INTERNAL_FUNNEL_TOF_INTEGRATION_GUIDE.md` (software implemented;
   physical characterization and acceptance remain).

All other future improvements in audits, report notes, completed plans, and
archived workstreams are context to consult only when the user asks for related
work.

## Invariants

- Keep `MotorControl.cpp` as the only periodic motor-output writer.
- Preserve the `Navigation.h` black-box boundary for mission code.
- Never treat invalid, stale, blind, unknown, or incoherent required evidence
  as clear.
- Do not claim current simulator/source parity until WASM is rebuilt with a
  matching source hash.
- Do not perform physical actions without explicit permission in the current
  task.
