# Robot Codebase Audit

Full audit report for the RoboCup robot workspace. Generated 2026-07-14 from the completed Codex audit goal.

## Bottom Line

This is a serious local-navigation prototype, but it is not competition-ready. The current system is mostly a set of methods that worked in selected cases, not yet the best verified architecture for the 2026 task.

The most important recommendation is to stop reverse/autonomous physical testing until the P0 safety issues are fixed, then re-center the project around scoring-loop evidence: collect, confirm payload, return, dock, unload, and repeat under the two-minute match constraint.

## Scope And Evidence

- Read all first-party planning, status, and test docs, plus the 2026 RoboCup challenge PDF.
- Read and audited all first-party source/config across `RobotCode`, simulator, serial tools, waypoint planner, TOF sketches, historical versions, and course templates.
- Active `RobotCode` is about 8.5k source/config lines. `LocalPlanner.cpp` alone is 2,945 lines and carries most of the autonomy complexity.
- Generated no source edits during the audit. Existing dirty worktrees were preserved. Compile/test commands may have generated caches and output files.
- No physical robot or serial commands were sent.

## Verification Run

| Check | Result |
| --- | --- |
| `arduino-cli compile --fqbn teensy:avr:teensy40 RobotCode` | PASS |
| `RobotCode` compile with warnings enabled | PASS |
| Simulator Node tests | 25/25 PASS |
| Python pytest for `SerialCommandUI` and `TOFReturnSignalExperiment` | 76/76 PASS |
| Python `compileall` | PASS |
| Historical/standalone Teensy sketches | 9/9 compiled |
| `ruff` | 4 unused imports found |

Approximate compiled memory: code 117,744 bytes; RAM1 variables 114,208 bytes; free locals 279,008 bytes; RAM2 12,416 bytes.

## Reconstructed Plan

The plan appears to target the 2026 autonomous RoboCup challenge: a Teensy 4 robot operating for two minutes in an unknown 4.9m x 2.4m arena with 400mm walls, gaps wider than 400mm, ramps up to 100mm and 30 percent grade, bumps up to 25mm, target weights of 50mm x 70mm, and Spheros at 73mm diameter moving up to 7m/s.

Scoring should strongly shape strategy: delivered target weights score double mass, onboard targets score single mass, delivered dummy weights carry no penalty, onboard dummy weights score -0.5, and only three targets onboard count. That means the first architecture question should be whether to collect all cylinder-shaped weights and dump them in base before spending complexity on material classification.

The current default `START` route is still a small rectangle ending `HOME`. Full mission behavior is incomplete: no real match timer, base transform, docking/unload, payload confirmation/count, arena boundary model, dynamic opponent policy, or production scoring loop.

## What Is Sound

- The code has a single scheduled control path for normal autonomy and one normal periodic motor writer. That is the right instinct for a small embedded robot.
- Geometry and config are explicit, which makes the robot auditable instead of magical.
- Continuous encoder totals and a rolling local map are better than pretending the robot has a complete global map in an unknown field.
- Short-horizon replanning is appropriate for unknown obstacles and a crowded arena.
- Telemetry and regression tooling are unusually strong for a student robotics codebase. That work should be preserved and tightened, not discarded.
- The separation between object sensing and safety is directionally good, although the current implementation still lets stale/fake data leak into safety decisions.

## Stop-Ship Findings

### Current P0 remediation disposition (2026-07-14 review)

- **P0-01 â€” fixed in software.** The canonical turn contract and offline
  invariants are aligned; physical sign validation remains required.
- **P0-02 â€” fixed in software.** Authority, revoke-before-cancel transitions,
  final-writer checks, and disarm paths are covered by compile/contract tests;
  runtime/HIL and physical transition validation remain required.
- **P0-03 â€” partially fixed.** Continuous direction-aware supervision,
  command lease, and manual/test coverage are implemented, but the supervisor
  still depends on real rear/side/central sensor coverage.
- **P0-04 â€” intentionally deferred.** `RANGE_FAKE_REAR` remains test
  scaffolding. Operator decision, 2026-07-14: it may be used for current
  obstacle testing, but it is not proof of rear clearance, competition
  readiness, or closure of P0-04/P0-05.
- **P0-05 â€” still open.** Low central/rear/side coverage, status-aware
  tri-state evidence, and the unknown-blocked safety proof are incomplete.
- **P0-06 â€” partially fixed.** Phase 1 runtime polling, loop telemetry,
  disabled blocking turn-ladder, and motor lease/watchdog are implemented;
  coherent sensor snapshots and physical watchdog timing remain follow-ups.
- **P0-07 â€” fixed in software.** The hard collision fallback and reverse grace
  are removed; strict footprint rejection now stops. Physical navigation
  acceptance remains separate from this code-level closure.
- **P0-08 â€” fixed by disabling the unsafe path.** The field UI is preview-only
  and sends neither `TEST ARM` nor `TEST GOTO`; SE(2) command support remains
  intentionally unimplemented.

Operator decision, 2026-07-14: physical obstacle testing may continue with the
current `RANGE_FAKE_REAR` fake rear channel. This is a temporary test
assumption only; it does not prove rear safety, does not close P0-04/P0-05, and
does not represent competition-ready sensing.

### P0-01: Turn sign and kinematics contradict each other

Original audit evidence: `MotorControl.cpp` treated positive turn as left
faster/right slower and called the calibrated branch `rightTurn`. `Helpers.cpp`
negated the IMU into nav CCW-positive. `LocalPlanner.cpp` documented positive
physical-right/opposite-nav but computed omega as `(left-right)/track`,
increasing nav heading. Turn observability also checked opposite sides in
different paths.

Recommended fix: Define one canonical convention: `+yaw` and `+turn` are CCW/left in navigation. Use `omega=(right-left)/track` for differential-drive nav heading. Add wheels-up invariant tests that prove left/right wheel commands, IMU sign, odometry integration, and candidate sensing all agree.

Remediation status (2026-07-14): fixed in software. `TurnConvention.h`
defines `+X` forward, `+Y` left, positive yaw/heading/turn/curvature as CCW/left,
`left=forward-turn`, `right=forward+turn`, and
`omega=(right-left)/trackWidth`. The installed BNO raw yaw remains
CW/right-positive and is negated exactly once at `navigationHeadingDeg()`.
Firmware mixing, calibrated pivots, planner rollouts and swept-side selection,
odometry/stuck diagnostics, simulator physics, desktop manual controls, and
their labels now use that contract. Compile-time and offline simulator/Python
invariants cover the sign chain. This does not claim physical validation; no
serial, upload, or robot motion was run for the remediation.

### P0-02: Motion authority and disarm can be bypassed (historical evidence)

Evidence: `RobotCode.ino` requests stop when run is false, then always calls the controller. `LocalPlanner` can clear that stop for an old active goal. `ZERO`, `TEST ARM`, `MANUAL ARM`, and related commands do not consistently cancel or suspend existing goals.

Follow-up: exercise the authority transitions and stale-goal cancellation with runtime/HIL fault injection, then perform gated physical validation.

### P0-03: Manual safety is one-shot, not continuous (historical evidence)

Evidence: Obstacle checks happen on `DRIVE` parse. A new obstacle can appear after the command starts and the robot may continue until the next command or watchdog timeout. Reverse has no real rear sensing.

Remediation: a direction-aware continuous supervisor now covers manual, route,
test, and recovery motion. Follow-up is real rear/side/central coverage and
physical validation.

### P0-04: Fake rear sensor makes reverse unsafe (intentionally deferred)

Evidence: `RobotConfig.h` reports a fake 4000mm rear reading and `TofSensors.cpp` continuously marks it valid. `LocalPlanner` paints rear free and can erase remembered obstacles, while reverse is enabled by default.

Current testing note, 2026-07-14: the operator explicitly permits obstacle
testing with the existing fake rear channel for now. Label those runs as
fake-rear tests and do not use them as proof of real rear coverage.

Recommended fix: Make fake rear invalid in competition builds through a compile-time assert. Either install real rear/side sensing or remove reverse from autonomous recovery.

### P0-05: Sensing cannot prove safety (still open)

Evidence: High fan rays lack a central 0-degree ray, the high mount can miss legal low hazards, pivot safety is inferred from forward endpoints, and 8190/8191 no-return readings can be accepted as valid. A 7m/s moving element can travel about 0.84m in a 120ms sensing delay.

Recommended fix: Add verified overlapping low central coverage, real rear/side coverage or behavior constraints, status-aware tri-state sensor readings, and a rule that unknown is blocked for safety.

### P0-06: Blocking sensor paths can leave motors holding stale commands (phase 1 remediation)

Original evidence: `ObjectDetection` refresh used eight `delay(15)` calls, four sequential L0X reads could each block up to 100ms, and `TEST TURNLADDER` blocked for seconds and wrote motors directly. Phase 1 removed those runtime paths, added loop/lease protection, and disabled the ladder; coherent snapshots and physical watchdog timing remain open.

Remediation: phase 1 removed runtime delays, added readiness polling, loop
deadline telemetry, and an independent motor lease/watchdog. Follow-up is
coherent sensor snapshots plus measured physical watchdog timing.

### P0-07: Hard collision override exists

Evidence: `LocalPlanner` `corridor_squeeze_straight` can command forward after all rollouts were rejected or footprint collision was detected.

Recommended fix: Never override hard collision. If corridor squeezing is needed, prove the swept footprint analytically or fix inflation/resolution so the planner can represent the corridor correctly.

Remediation status (2026-07-14): fixed in software. The synthetic
`corridor_squeeze_straight` command and reverse front-footprint grace were
removed. Zero accepted forward or reverse rollouts now stop, simulator/source
contracts reject reintroduction of those bypasses, and bounded
`final_blocked_reached` additionally requires a clear current inflated
footprint and leaves motion neutral. No physical serial, upload, or robot test
was run for this remediation.

### P0-08: Field GOTO UI uses the wrong frame and auto-arms (historical evidence)

Original evidence: `serial_command_ui_field_goto.py` treated clicks as absolute top-left field coordinates with +Y down and sent `TEST GOTO` while auto-arming. The current UI is preview-only and sends neither `TEST ARM` nor `TEST GOTO`.

Current disposition: the field UI is disabled for command transmission and is
preview-only. It sends neither `TEST ARM` nor `TEST GOTO`; SE(2) command support
remains a future, explicitly confirmed feature.

## High Priority Findings

### P1-09: Unknown map cells are treated as free

Evidence: Collision checks only mark thresholded obstacles/outside as occupied. Zero evidence is not the same as free space. Footprint raster steps can also tunnel through inflated edges.

Recommended fix: Represent `UNKNOWN`, `FREE`, and `OCCUPIED` explicitly. Use polygon-cell intersection or a distance transform for footprint checks.

### P1-10: Object classifier is fail-open, stale, and asynchronous

Evidence: Upper invalid readings can mean clear. `UNKNOWN` is almost unreachable. Candidate confirmation can count repeated calls rather than fresh samples, and current pose is used instead of acquisition pose.

Recommended fix: Use tri-state classification with frame IDs, sequence numbers, age/skew limits, pose-at-acquisition, source metadata, and clustering. Missing data should produce `UNKNOWN`, not clear.

### P1-11: Planner commands can be physically infeasible

Evidence: Minimum sustained wheel speed is 1500, but forward 1500 with turn ratio 0.65 yields an inner wheel command around 525. The model lacks acceleration, deadband, and current velocity constraints.

Recommended fix: Add per-wheel inverse models, a reachable velocity set, acceleration slew, and PID reset/freeze behavior that does not reset odometry baselines.

### P1-12: Motor mode is tied to goal type

Evidence: Calibrated pivots are only used for `NAV_GOAL_TURN`. Point alignment, manual turning, and test turn pulses can exercise different motor paths.

Recommended fix: Make motor command mode explicit and ensure tests hit the same controller path that competition autonomy uses.

### P1-13: Success criteria are too loose

Evidence: Route lateral/overshoot tolerances are broad, `final_blocked_reached` can report success within 160mm, and regression end-state checks are loose.

Recommended fix: Return typed results: `EXACT_REACHED`, `PLANE_CROSSED`, `SAFE_PARTIAL`, `BLOCKED`, `FAULT`, and `TIMEOUT`. Let mission logic decide whether a partial is acceptable.

### P1-14: Mission state has liveness and reset bugs

Evidence: `startedMs` is unused, route failure can park forever, return-home failure is not handled, stuck flags persist, and `HOME` while disarmed can be ineffective.

Recommended fix: Replace scattered booleans/latches with explicit mission and planner-phase state machines with reset ownership and timeout transitions.

### P1-15: IMU init and runtime health are too optimistic

Evidence: Boot retries can be infinite and runtime `getEvent`/calibration/stale/finite checks are not enforced deeply enough.

Recommended fix: Use bounded init into a safe fault mode and track runtime sample health in the motion supervisor.

### P1-16: Full mission is not implemented

Evidence: `APPROACH_OBJECT`, `COLLECT_SORT`, and `UNLOAD` states exist but are unused. The default route is a calibration rectangle, not a scoring loop.

Recommended fix: Build the mission around timer, return deadline, base transform, payload confirmation/count, return/dock/unload, search, slow approach, and optional classification.

### P1-17: Simulator gives false confidence

Evidence: Simulator reverse min speed, candidate gating, scoring, sensor selection, map behavior, stuck handling, pose defaults, and collision model differ from firmware.

Recommended fix: Extract a platform-neutral shared core for planner logic or at least enforce a shared config/schema and golden differential traces tied to timestamps and `SE(2)` poses.

### P1-18: Regression and UI tools have safety gaps

Evidence: Regression runner lacks `try/finally` stop behavior, weak preflight, weak oracles, and disconnect paths that do not reliably `STOP`/disarm. Manual button and keyboard signs differ.

Recommended fix: Add preflight build/status/ack checks, `try/finally STOP`, stronger oracles, contact-as-failure exits, and consistent deadman behavior.

### P1-19: Waypoint planner is not firmware-compatible

Evidence: The web planner uses a top-left field frame and exports actions that differ from firmware route format and nav frame assumptions.

Recommended fix: Treat it as concept-only until route JSON has versioned frame metadata, a firmware adapter, and preview checks.

### P1-20: Docs and config have drifted

Evidence: Before this documentation reconciliation, positive turn direction,
inner sensor fault handling, nonblocking claims, blind-space claims, and stream
interval docs disagreed with code. Historical documents remain explicitly
labelled; operator-facing P0 status is now recorded above.

Recommended fix: Keep docs generated from config where possible and add a release checklist that rejects stale physical-test claims.

### P1-21: Repo reproducibility is weak

Evidence: Outer repo is dirty/ahead with active untracked projects, tracked pyc, no root `.gitignore`, a large temporary Arduino mirror, unpinned PlatformIO deps, no Python dependency manifest, and no simulator lockfile.

Recommended fix: Add lockfiles/manifests, ignore generated caches, archive temp mirrors, and stamp physical logs with commit SHA, config hash, and build label.

### P1-22: Historical sketches are unsafe to keep upload-ready

Evidence: Some older sketches auto-start, block movement, use blind reverse, or old pin topologies.

Recommended fix: Archive them behind a `DO NOT UPLOAD` marker and make the active build path unambiguous.

### P1-23: TOF material classifier evidence is not yet scientific

Evidence: Embedded prototypes appear evaluated on similar data, confidence is not a proper nearest-distance/margin, and object geometry/orientation can dominate material signal.

Recommended fix: Keep raw logging, move model evaluation to host-side grouped cross-validation, hold out physical objects/trials, and add out-of-distribution behavior.

## Recommended Architecture

### Phase 0: Safety And Truth

- Canonicalize frames and turn signs across firmware, simulator, UI, and docs.
- Add exclusive motion authority at the final motor writer.
- Make fake rear sensing a compile error in competition builds.
- Install/prove central low and rear/side coverage, or constrain behavior so missing coverage is never required for safety.
- Use timestamped async sensor frames, tri-state occupancy, and non-overridable collision checks.
- Add an independent motor command lease/watchdog and physical kill path.

### Phase 1: Pure Testable Core

- Extract or isolate frame transforms, differential-drive kinematics, snapshot health, occupancy, swept-polygon collision, reachable velocities, and typed planner results.
- Replace scattered booleans/latches with explicit planner-phase state.
- Preferred architecture: rolling grid plus local A*/D* for short local topology, then acceleration-constrained velocity window or simple path follower, with an independent safety kernel vetoing motion.
- Do not rewrite blindly. Compare a simplified current sampler against the proposed core on the same trace corpus first.

### Phase 2: Scoring Mission

- Implement match timer and return deadline.
- Implement base/start transform, arena boundaries, payload confirmation/count, return, dock, unload, and search.
- Use slow explicit approach for candidate objects.
- Treat material classification as optional until collect-and-deposit-all has been measured against it.

### Phase 3: Evidence

- Add C++ unit/property tests for signs, transforms, truth tables, map/collision, and state transitions.
- Add shared-core differential traces against the simulator.
- Run HIL loop-jitter tests with all eight ToF sensors active.
- Test stale sensor, timeout, disconnect, parser, deadman, power, and watchdog behavior.
- Tie physical logs to commit SHA, config hash, and build label.

## Strategy Recommendation

The highest-leverage strategic question is not the classifier. It is the scoring loop. Since delivered dummy weights carry no penalty while delivered target weights score double mass, the baseline should be collect all cylinder-shaped weights, return to base, and dump. Only keep the material classifier if timed trials show it improves score after accounting for false positives, false negatives, approach time, and carrying capacity.

That changes the engineering priority stack: safety kernel first, reliable payload/base loop second, search coverage third, classification fourth. Right now too much complexity sits in local planner heuristics before the system has proven the simple scoring cycle.

## Files Cited Most Often

- `C:/Users/marco/Documents/Robot Algorithm/RobotCode/MotorControl.cpp`
- `C:/Users/marco/Documents/Robot Algorithm/RobotCode/Helpers.cpp`
- `C:/Users/marco/Documents/Robot Algorithm/RobotCode/LocalPlanner.cpp`
- `C:/Users/marco/Documents/Robot Algorithm/RobotCode/RobotConfig.h`
- `C:/Users/marco/Documents/Robot Algorithm/RobotCode/TofSensors.cpp`
- `C:/Users/marco/Documents/Robot Algorithm/RobotCode/ObjectDetection.cpp`
- `C:/Users/marco/Documents/Robot Algorithm/SerialCommandUI/serial_command_ui_field_goto.py`
- `C:/Users/marco/Documents/Robot Algorithm/NAVIGATION_DESIGN.md`
- `C:/Users/marco/Documents/Robot Algorithm/00 - ROBOCUP INFO/Robocup - ALL FILES/2026 Robocup challenge v1.0.pdf`

## Final Assessment

This codebase is promising because it has real embedded discipline: config is visible, telemetry exists, tests exist, and the local planner has clearly been exercised against actual cases. But the dangerous parts are exactly where robotics systems usually get hurt: sign conventions, stale sensing, unowned motor authority, fake free space, blocking calls, and success conditions that are easier than the real mission.

My recommendation is to freeze feature tuning until the P0 issues are fixed. After that, build the smallest full scoring loop and measure it. The code will become simpler once the mission evidence, not the planner heuristics, starts pulling the design forward.
