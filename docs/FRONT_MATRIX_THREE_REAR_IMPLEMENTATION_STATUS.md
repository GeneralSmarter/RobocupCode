# Front matrix and three-rear migration implementation status

Date: 2026-08-09 (Pacific/Auckland)

> **Completed implementation record.** This file preserves the migration-time
> software evidence and its then-unrun physical gates. The operator later
> reported on 2026-08-11 that the current robot has been run physically and
> works. Do not reinterpret the historical “no physical action during this
> task” wording as the current project status, and do not treat remaining gates
> here as an automatic backlog.

## Baseline identity

- Git source commit: `c8a81822f567f7d982f46d3a862046fa84c7e676`
- Pre-migration WASM source hash:
  `15a261392de6ce3959c4ca5c97f555fcb089849f05dd6ce06b872cc5ab46c6ad`
- The worktree was already dirty/untracked and was preserved.
- Pre-migration Teensy compile: pass (`155256` code bytes; `201824` RAM1
  variable bytes).
- Pre-migration Python checks: 141 passed, 1 skipped; compileall passed.
- Pre-migration simulator: 48 tests, 43 passed, 5 pre-existing failures.

## Implemented software

- Central measured footprint, fan mounts, three rear VL53L1X mounts/wiring,
  front matrix transform, addresses, masks, full rear ROI, and compile-time
  uniqueness/mirroring checks.
- Cooperative rear scheduling and reconnect, per-channel state and freshness,
  sample-skew checks, per-sequence hysteresis, and fail-closed aggregation.
- Cooperative front SEN0628 acquisition in bounded 32-byte I2C chunks with
  immutable 64-cell pose/attitude/grid-transform frames.
- Supplemental collision veto plus height-aware classification, tracking,
  static confirmation, dynamic rejection, closest selection, and latching.
- Per-origin mapping, final live safety checks, and pickup swept-footprint
  validation with only the latched capture object spatially excluded.
- Full-speed hunt, bounded prediction, configurable 30 mm assumed handoff,
  route-authority-preserving blend, configurable 150 mm unconfirmed feed,
  typed outcomes, and no payload confirmation/counting.
- Explicit-start bounded matrix-follow diagnostic and separate compile profile.
- WASM/2.5D simulator support for independent rear faults, 64 matrix cells,
  cylindrical objects, low bumps, sloped ramps, walls, per-ray height,
  stale/repeated evidence, and a layout overlay.

## Failure analysis and resolution

The initial migrated simulator run had 18 failures. Each group was reproduced
independently before changing production behavior:

- The open-route speed failure was a configuration mismatch: production and
  the simulator still exposed a 3000 ticks/s ceiling while the migration
  contract requires 2600 ticks/s. Both now use the same 2600 ticks/s ceiling.
- Five emergency-recovery failures were stale direction contracts. In-place
  scans do not require rear clearance, a rear fault blocks relocation before
  reverse is published, and an impossible wall does not start an emergency
  scan before any evidence-backed recovery attempt. The tests now distinguish
  those cases and require neutral terminal output.
- The obstacle family combined four causal defects: a selected side could have
  no feasible final bypass pose; a local target could be footprint-infeasible;
  the controller could leave the lateral stage or release the retained
  obstacle before a safe route-facing transition existed; and noisy near-ties
  could let a 3 mm range perturbation override a materially shorter geometric
  side. The planner now validates both side endpoints, searches forward for a
  safe staged pose, returns a typed no-path when none exists, holds the lateral
  stage to one map cell, checks turn safety before direct release, uses a
  geometry-first quarter-cell tie band, and gives broad obstacles an additional
  10 mm tracking allowance. A bounded safety-checked alignment is available
  only after geometric forward planning fails.
- Several obstacle assertions encoded maneuvers rather than safety outcomes.
  The reduced boundary case now chooses the open side initially instead of
  switching unnecessarily; separate obstacle contexts may choose independent
  sides; an unproved reverse corridor must fail neutral; and rear blindness
  blocks reverse without blocking independently proven forward travel.
- The pressed-wall fixture cannot prove 100 mm of swept rear corridor from the
  three sparse rear rays. It now verifies the implemented fail-closed result:
  a bounded evidence-backed reverse segment followed by neutral
  `recovery_time`, with no collision.

No collision or unknown-as-clear exception was added. `MotorControl.cpp`
remains the only periodic motor-output owner.

## Verification and remaining gates

Final software verification on 2026-08-09:

- Teensy 4.0 production compile: pass (`166776` code bytes; `204160` RAM1
  variable bytes).
- Explicit-start matrix-follow profile compile: pass with the same memory
  identity.
- Python contract/unit suite: 143 passed, 1 skipped.
- Python compile checks: pass.
- Current WASM source/build identity:
  `9e6fe092c7c568b46979b155784499b4f2c54b7dba339c2257dec4e4f7f79056`.
- Current WASM artifact SHA-256:
  `85dea3c569967896e3ba7f26bf5fa66db4794627d76120ef12e4386bb2d6dec8`.
- Simulator: 56 total, 56 passed. This includes clean and website-default
  sensing for the exact custom, boundary-pocket, soft-edge, and heading-right
  regressions, plus all matrix hunt/handoff/feed and rear/front parity cases.

The deterministic software acceptance gate is complete. This does not establish
physical readiness.

No upload, serial connection, arming, stationary hardware capture, or physical
movement was performed. Rear stop/clear thresholds, matrix orientation/masks,
classification, prediction, handoff, feed, and follow limits remain provisional
until separately authorized characterization.
