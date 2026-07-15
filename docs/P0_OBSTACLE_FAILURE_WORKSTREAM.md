# P0 Obstacle Failure Remediation Workstream

## Purpose

This document turns the failed 2026-07-14 `TEST AVOID 1.20` physical run into
separate, reviewable Codex tasks. Start a new task for each numbered work item
and paste its prompt verbatim. Complete the tasks in order unless a task
explicitly says otherwise.

The failed run did not reach the requested goal. It travelled about 2.73 m,
ended about 2.35 m from the goal, changed bypass side, made a large right-hand
circle, recorded three command-lease trips, and ended only when the host timed
out. The operator observed no contact or scrape. That is a contact-free
failure, not a successful avoidance result.

Primary evidence:

- `logs/navigation_regressions/20260714_145715_p0_fake_rear_avoid12.raw.txt`
- `logs/navigation_regressions/20260714_145715_p0_fake_rear_avoid12.csv`
- `logs/navigation_regressions/20260714_145715_p0_fake_rear_avoid12.summary.json`

## Shared rules for every task

- Read `AGENTS.md`, `HANDOFF.md`, `RobotCode/README.md`, this workstream, and
  `RobotCode/docs/ROBOT_CODEBASE_AUDIT.md` before changing code.
- Preserve unrelated dirty-worktree changes. Inspect overlapping changes
  before editing.
- Keep `MotorControl.cpp` as the only periodic motor-output owner.
- Preserve the public convention: positive yaw/turn is left/CCW. Fix the
  lowest layer that contradicts physical truth; do not compensate by adding
  an opposite sign in the planner.
- Do not tune planner scoring or add new navigation features unless required
  by a demonstrated failure in this workstream.
- Do not upload firmware unless the current build label/status is not running
  the source that must be tested. Ask before every upload.
- Before any motor-moving command, obtain a fresh operator confirmation of the
  physical setup. Start wheels-up or on blocks when possible.
- Do not execute a full `START` route or a field `GOTO` command.
- Use the regression logger for floor movement and save logs/marks for every
  movement test.
- Stop immediately on unexpected authority, non-neutral motors, watchdog not
  ready, invalid/stale critical sensors, a lease trip, excessive loop delay,
  or unexpected motion.
- `RANGE_FAKE_REAR` is explicitly permitted test scaffolding only. Mark every
  applicable run as fake-rear testing. It proves neither rear safety nor
  P0-04/P0-05 closure.
- End every task with: files changed, tests run, exact results, residual risks,
  and the single recommended next workstream task.

## Task status board

| ID | Task | Depends on | Completion gate | Status |
|---|---|---|---|---|
| 01 | Re-prove and correct physical turn truth | None | Both signed forward arcs agree with physical and telemetry signs | Complete |
| 02 | Make obstacle-bypass state deterministic | 01 | Active bypass side cannot change implicitly | Complete |
| 03 | Add recovery liveness and divergence bounds | 02 | Detours fail safely within explicit time/distance/progress bounds | Complete |
| 04 | Make regression logging nonblocking and fail-fast | 01 | Logging cannot cause a lease trip; runner abort/cleanup is proven | Complete |
| 05 | Run integrated software verification | 02, 03, 04 | Full verification suite passes with new regression coverage | Complete |
| 06 | Run staged physical revalidation | 05 | Signed arcs, open-floor movement, then one obstacle run pass all gates | Not started |

Update only the `Status` column and add dated evidence beneath the applicable
task when a task is completed. Use `Blocked` rather than `Complete` when its
completion gate is not met.

## Task 01 â€” Re-prove and correct physical turn truth

### Copy-ready task prompt

```text
Workstream Task 01: re-prove and correct physical turn truth after the failed
2026-07-14 TEST AVOID 1.20 run.

Read AGENTS.md, HANDOFF.md, RobotCode/README.md,
RobotCode/docs/ROBOT_CODEBASE_AUDIT.md, and
RobotCode/docs/P0_OBSTACLE_FAILURE_WORKSTREAM.md first. Inspect the failed run
logs named there.

Goal: determine exactly why ordinary forward arcs respond with the opposite
physical/heading sign even though the software wheel-mixing equation is
left=forward-turn, right=forward+turn. Do not patch planner signs. Isolate the
physical motor-side mapping, forward direction, encoder signs, calibrated
pivot path, ordinary arc path, raw IMU yaw, and navigation yaw.

Add only the diagnostic telemetry or narrowly scoped test commands needed to
make that chain observable. Add offline contract tests that distinguish the
calibrated NAV_GOAL_TURN pulse path from the ordinary forward-arc PID path.
Compile and run all relevant software tests.

Physical work is allowed only after asking me to confirm the setup before the
first motor-moving command and again after any setup change. Begin wheels-up
or on blocks. Use very small signed tests. Ask before firmware upload, and
upload only if BUILD/STATUS proves the required source is not running. Save a
marked regression log for every floor movement.

Completion requires physical evidence for both signs: positive forward arc is
left/CCW with positive navigation yaw, negative forward arc is right/CW with
negative navigation yaw, encoder signs match forward-positive wheel motion,
and motors neutralize afterward with authority NONE and zero new lease trips.
If the exact cause cannot be proven, stop with Task 01 blocked; do not guess a
sign correction or proceed to obstacle testing.
```

### Required handoff evidence

- A table for each tested motion containing requested forward/turn, requested
  left/right wheel targets, actual pulses, encoder deltas/rates, raw IMU yaw
  delta, navigation yaw delta, and observed physical direction.
- Exact build label and source revision/config identity.
- Log paths for every movement.
- The exact lowest-layer correction, or a clearly stated unresolved conflict.

### Task 01 evidence â€” 2026-07-14

Result: **Complete.** The physical defect was a complete left/right drivetrain
channel swap below the canonical chassis mixer. No planner sign was changed.
Wheels-up isolation proved that the software-left motor/encoder channel drove
the physical-right wheel forward, while the software-right channel drove the
physical-left wheel forward. The correction therefore belongs in the physical
motor/encoder mapping and its per-channel calibration, not in planner scoring
or `left=forward-turn`, `right=forward+turn`.

| Mark / setup | Requested forward / turn (tps) | Requested L / R (tps) | Actual L / R pulses (us) | Encoder evidence / rates (tps) | Raw IMU / nav yaw delta | Physical observation / result |
|---|---:|---:|---:|---|---:|---|
| `t01_leftchan_up1`, wheels-up, pre-fix | `500 / -500` | `1000 / 0` | about `1692â†’1679 / 1500` | software L `+81` drive-phase ticks (`~600`), R `0` | `0.00 / 0.00 deg` | Physical **right** wheel forward. Proved software-left channel was physically right. Full CSV also exposed 5 loop misses; not an acceptance run. |
| `t01_rightchan_up1`, wheels-up, pre-fix | `500 / +500` | `0 / 1000` | `1500 / 1667â†’1641` | software L `0`, R `+120` drive-phase ticks (`~1048`) | `0.00 / 0.00 deg` | Physical **left** wheel forward. Proved software-right channel was physically left. One cumulative loop miss; not an acceptance run. |
| `t01_pivot_p15_up1`, wheels-up, post-fix calibrated `NAV_GOAL_TURN` | `0 / +800` | `-800 / +800` | `1200 / 1800` | L negative / R positive; active rates about `-2000 / +2300` | approximately `+0.06 / -0.06 deg` because chassis was blocked from rotating | Physical left wheel backward and right wheel forward: the positive calibrated pivot wheel invariant is left/CCW. The yaw goal correctly aborted `turn_progress_stalled` on blocks; this was path isolation, not a floor acceptance run. |
| `t01_arc_p250_floor1`, low-speed floor diagnostic | `800 / +250` | `550 / 1050` | `1592 / 1703` | sampled delta `0 / +38`; left wheel did not break away | `-0.31 / +0.31 deg` | Barely moved; inconclusive and rejected. One cumulative loop miss led to the neutral-only `LOOP RESET` diagnostic gate. |
| `t01_arc_p200_floor2`, low-speed floor diagnostic | `1000 / +200` | `800 / 1200` | `1633 / 1733` | sampled delta `0 / +38`; left wheel still did not break away | `-0.75 / +0.75 deg` | Barely moved; inconclusive and rejected. Timing window itself was later isolated with `LOOP RESET`. |
| `t01_arc_p200_floor3`, accepted positive floor arc | `1700 / +200` | `1500 / 1900` | `1735 / 1839` near drive end | total forward-positive delta `+379 / +614`; active end rates about `+700 / +1100` | `-6.62 / +6.62 deg` final | Operator: short forward-left/CCW. Zero loop misses, zero lease trips, neutral `1500/1500`, authority `NONE`; **PASS**. |
| `t01_arc_n200_floor1`, accepted negative floor arc | `1700 / -200` | `1900 / 1500` | `1791 / 1772` near drive end | sampled 120â€“600 ms forward-positive delta `+498 / +329`; active end rates about `+1150 / +700` | `+5.37 / -5.37 deg` final | Operator: forward-right/CW. Zero loop misses, zero lease trips, neutral `1500/1500`, authority `NONE`; **PASS**. |

Marked movement evidence:

- `SerialCommandUI/logs/navigation_regressions/20260714_153609_t01_leftchan_up1.*`
- `SerialCommandUI/logs/navigation_regressions/20260714_154233_t01_rightchan_up1.*`
- `SerialCommandUI/logs/navigation_regressions/20260714_154948_t01_pivot_p15_up1.*`
- `SerialCommandUI/logs/navigation_regressions/20260714_160851_t01_arc_p250_floor1.*`
- `SerialCommandUI/logs/navigation_regressions/20260714_161228_t01_arc_p200_floor2.*`
- `SerialCommandUI/logs/navigation_regressions/20260714_161420_t01_arc_p200_floor3.*`
- `SerialCommandUI/logs/navigation_regressions/20260714_161501_t01_arc_n200_floor1.*`

Exact lowest-layer correction in `RobotConfig.h`:

- motor outputs: physical left/right are pins `1/0`, not `0/1`;
- forward bases moved with their physical channels to left/right `1870/1935 us`;
- reverse bases moved with their physical channels to left/right `1190/1130 us`;
- encoder channels: physical left is `4/5`, physical right is `2/3`;
- encoder signs moved with those channels to left/right `+1/-1`.

The public contract remains `+turn=left/CCW`,
`left=forward-turn`, `right=forward+turn`, and
`omega=(right-left)/trackWidth`. Raw installed-BNO yaw is CW/right-positive and
is negated exactly once into navigation yaw.

Tested identity:

- firmware build: `V7-turn-truth-t01d`;
- dirty-worktree base revision: `e4db81154454298da79cd8ea78d13ce2ff831b1b`;
- `RobotConfig.h` SHA-256: `F9A96233F314B8FB6D114DB6D51E4206C4580A51DC5EFC0B4EACCA33101FC2FD`;
- `MotorControl.cpp` SHA-256: `096029FC4C06A2CFDED5A9360B6C9F05A9D188A3840ED534FDD38EF50E3829B3`;
- `Bluetooth.cpp` SHA-256: `35944557DDBD37F05F2B7D750036CA3189F1A5E1BB9117E2A43D6581F870289D`;
- `TurnConvention.h` SHA-256: `3C5075859F625170352520C13EDB4920D4A213B6434753442A71562E23C80F12`;
- regression runner SHA-256: `9D93298394DE539275C4F0D9E34031A0E3313733ADB5E22C5093D3398750A35B`.

Software verification after the correction:

- Teensy 4.0 Arduino CLI compile: PASS;
- simulator: 25/25 PASS;
- Python: 81/81 PASS (one transient local Tk resource-load failure passed on immediate unchanged rerun);
- Python `compileall`: PASS.

Task-01 files changed: `RobotConfig.h`, `Robot.h`, `Globals.cpp`,
`Odometry.cpp`, `MotorControl.cpp`, `Bluetooth.cpp`,
`SerialCommandUI/run_navigation_regression.py`,
`SerialCommandUI/test_turn_sign_contract.py`, and this workstream. Residual
risks: `RANGE_FAKE_REAR` and P0-05 sensor coverage remain open; the synchronous
full CSV transport remains Task 04 work; the compact Task-01 logger is not a
replacement for that refactor. No obstacle test, full `START`, field `GOTO`, or
Task 02 work was performed. Recommended next workstream task: **Task 02 â€” Make
obstacle-bypass state deterministic.**

## Task 02 â€” Make obstacle-bypass state deterministic

### Copy-ready task prompt

```text
Workstream Task 02: make obstacle-bypass state deterministic. Task 01 must be
complete first; verify its dated evidence in
RobotCode/docs/P0_OBSTACLE_FAILURE_WORKSTREAM.md.

Read the shared workstream rules and inspect the failed 2026-07-14 obstacle
log. Diagnose and fix the planner behavior where the run entered side_escape
with side=-1/right, then re-read changing sensor geometry and entered reverse
recovery with side=1/left.

Implement a clear distinction between observed preferred side and the latched
active bypass side. Latch the active side when a bypass starts. Ordinary
sensor refreshes must not reverse it during side escape, reverse recovery,
post-reverse escape, or route rejoin. A side change must require a neutral
stop, explicit abort/reset transition, and fresh stable evidence. Make every
phase transition explicit and ensure reverse recovery cannot skip the intended
post_reverse_escape transition because another bypass flag remains active.

Add deterministic firmware/source-contract and simulator tests for initial
side selection, noisy/tied/changing evidence, reverse entry and exit,
post-reverse escape, route rejoin, abort/reset, and a prohibited mid-manoeuvre
side flip. Add telemetry for observed side, latched side, phase, transition
reason, and reset reason.

This is a software task. Do not upload firmware or move the robot. Do not tune
arc scores or clearance constants unless a new test proves that is necessary.
Completion requires all new tests and existing verification to pass and a
trace proving that the original right-side decision remains latched through
the recovery sequence.
```

### Task 02 evidence â€” 2026-07-14

Result: **Complete.** Task 01 was confirmed Complete before implementation.
Its dated evidence identifies the lowest-layer left/right channel swap, proves
both signed forward arcs with encoder, raw/navigation-yaw, and physical
direction agreement, identifies the tested build and source hashes, and
records passing firmware, simulator, Python, and compileall verification.

The Task-02 defect was shared mutable side state. `side=-1` was selected at
`side_escape` entry in the failed run, but later sensor geometry overwrote the
same variable/memory with `side=1`; reverse startup then widened and continued
on the newly observed left side. A second control-flow defect let the active
front-wall flag suppress the intended post-reverse local-goal transition.

The firmware and simulator now keep separate observed and latched side state.
Observed preference requires two consistent refreshes; a tied front-wall view
uses the repeated blocked-turn observation as its deterministic fallback.
Once latched, ordinary observation refreshes cannot change the active side.
An attempted opposite-side write is rejected and reported. Goal completion,
abort, new-goal setup, and controller initialization neutralize/reset the
manoeuvre state with an explicit reset reason before a later side can be
latched from fresh stable evidence.

Reverse handling now has explicit phases:
`SIDE_ESCAPE -> FRONT_WALL_REVERSE -> POST_REVERSE_ESCAPE -> ROUTE_REJOIN`.
Post-reverse selection uses the active latch and is evaluated once the reverse
survey is ready, regardless of the pre-existing front-wall bypass flag. If the
side is already clear, the stopped survey exits explicitly to route rejoin.

Golden simulator trace for the exact failed-run side sequence:

| Transition | Observed preferred side | Latched active side | Result |
|---|---:|---:|---|
| `idle -> side_escape` | `-1` (right) | `-1` (right) | Initial stable right evidence latched. |
| changing evidence before reverse | `+1` (left) | `-1` (right) | Observation changes; active side does not. |
| `side_escape -> front_wall_reverse` | `+1` | `-1` | Reverse entry preserves right. |
| `front_wall_reverse -> post_reverse_escape` | `+1` | `-1` | Post-reverse escape preserves right. |
| `post_reverse_escape -> route_rejoin` | `+1` | `-1` | Rejoin preserves right. |
| stopped `explicit_abort_reset`, then two left observations | `+1` | `0 -> +1` | Side changes only after explicit neutral reset and fresh stable evidence. |

Telemetry events now expose `observed`, `latched`, phase, transition `reason`,
and reset `reason` through `obstacle_bypass_phase`,
`obstacle_bypass_reset`, `obstacle_bypass_latch` (simulator), and
`obstacle_bypass_side_rejected`.

Focused and full software verification:

- Teensy 4.0 Arduino CLI compile: PASS (`FLASH code 122868`, RAM1 variables
  `116352`, RAM2 variables `12416`).
- Simulator: **27/27 PASS**, including the exact right-to-left flip sequence,
  stable/noisy/tied evidence, reverse/post-reverse/rejoin, explicit reset,
  existing G0, G5, offset-wall/gap, front-wall safe-stop, and wide-panel cases.
- Firmware/source contract: **6/6 PASS** inside the full Python run.
- Python: **87/87 PASS**.
- Python `compileall`: PASS.

Task-02 files changed: `RobotCode/LocalPlanner.cpp`,
`RobocupSimulator/simulator-core.js`,
`RobocupSimulator/simulator-core.test.js`,
`SerialCommandUI/test_obstacle_bypass_contract.py`, and this workstream.
No arc scoring, clearance, collision, or watchdog constants were changed. No
firmware upload, serial access, or physical movement was performed.

Residual risks: Task 02 proves deterministic side ownership and phase
transitions in software only. It does not add the Task-03 time, distance,
progress, repeat-recovery, or divergence bounds; synchronous logging remains
Task 04; `RANGE_FAKE_REAR` and P0-05 coverage remain unresolved. Recommended
next workstream task: **Task 03 â€” Add recovery liveness and divergence bounds.**

## Task 03 â€” Add recovery liveness and divergence bounds

### Copy-ready task prompt

```text
Workstream Task 03: add recovery liveness and divergence bounds. Confirm Tasks
01 and 02 are complete first.

The failed TEST AVOID 1.20 run kept finding locally safe arcs while global
goal distance and lateral displacement grew. Inspect the route-line finish,
missed-route, clearance-local-goal, reverse-recovery, and bypass-phase code.

Implement fail-closed bounds that remain active even while a clearance or
post-reverse local goal exists. Track and expose at least: mission/global goal
distance, active local-goal distance, route along-progress, signed lateral
error, phase elapsed time, cumulative bypass/recovery distance, recovery
count, and best progress since bypass start.

Define typed safe-abort reasons for bounded goal-distance growth, excessive
lateral deviation, recovery time/distance exhaustion, repeated recovery, and
failure to improve route/global progress. Use conservative constants derived
from robot geometry and the small test envelope, with comments explaining
their units and rationale. The result must neutralize motion and preserve the
existing collision veto. Do not turn a partial or timeout into success.

Add simulator/unit regressions reproducing a looping/diverging detour, a
legitimate bounded bypass, a blocked safe stop, and route rejoin. Prove the
diverging case terminates inside every declared bound. This is a software-only
task: no upload and no physical movement.
```

### Task 03 evidence â€” 2026-07-14

Result: **Complete.** Tasks 01 and 02 were confirmed Complete before
implementation from their dated evidence above. Task 01 retains the canonical
signed drivetrain/yaw contract after the lowest-layer channel correction;
Task 02 retains the observed-versus-latched bypass side and explicit
`SIDE_ESCAPE -> FRONT_WALL_REVERSE -> POST_REVERSE_ESCAPE -> ROUTE_REJOIN`
phase contract.

The failed-run escape path previously disabled route-line missed/finish guards
whenever a clearance local goal existed. A locally reachable moving target
could therefore keep producing footprint-safe arcs even while distance from
the original mission goal and route grew. Task 03 adds an independent
original-goal liveness monitor before forward/reverse arc publication. It
remains active in every non-idle bypass phase and while clearance,
post-reverse, or reverse-recovery state exists. It does not modify candidate
scores, clearance constants, hard footprint rejection, or any success gate.

The monitor and CSV/STATUS telemetry now expose:

- mission/global goal distance and active local-goal distance;
- route along-progress and signed lateral error (positive is route-left);
- current bypass-phase elapsed time;
- cumulative bypass/recovery path distance and recovery count; and
- best improvement from bypass start using either global-distance reduction
  or route along-progress.

Declared small-test recovery envelope:

| Typed stop result | Bound | Rationale |
|---|---:|---|
| `recovery_divergence` | goal distance may grow at most `0.45 m` beyond its best value | Matches the existing missed-route overshoot window. |
| `recovery_displacement` | absolute signed lateral error at most `0.75 m` | Allows the `0.50 m` front-wall bypass plus chassis-width/clearance margin. |
| `recovery_time` | at most `12.0 s` in one bypass phase | Bounds settle, reverse, post-reverse, and rejoin independently. |
| `recovery_distance` | at most `2.40 m` cumulative bypass/recovery travel | Twice the nominal `1.20 m` test envelope is already a failed detour. |
| `recovery_repeated` | at most `2` reverse-recovery entries | A third recovery is a repeated-failure result, not another retry. |
| `recovery_no_progress` | `0.05 m` global/route improvement in every `6.0 s` window | Stops locally safe circling or stationary replanning that does not improve the mission. |

Every violation calls the single failure exit with its typed
`PlannerStopReason`, neutralizes the motion command before emitting the abort,
and leaves `completed=false`, `failed=true`. A timeout, partial route result,
or local-goal result cannot reach the success event path.

Deterministic evidence:

- The scripted large-loop regression keeps a local goal only `0.20 m` ahead
  while circling away from the original `TEST AVOID` goal. It aborts as
  `recovery_divergence`, before the `12 s` phase bound and before the `2.40 m`
  cumulative-distance bound, with neutral motion and no success event.
- Focused cases independently cross each of the six declared limits and prove
  the exact typed abort, failure-only result, neutral command, and detection no
  more than one planner-scale spatial/time increment beyond the threshold.
- The wide-panel case proves a legitimate bounded bypass remains supervised,
  explicitly reaches `route_rejoin`, and completes honestly without contact.
- The front-against-wall case remains a strict footprint-rejection safe stop:
  no reverse step and zero commanded forward/turn motion.
- Existing G0, G5, offset-wall/gap, hard-footprint, final-blocked, sign, and
  bypass-latch regressions remain passing.

Full software verification:

- Teensy 4.0 Arduino CLI compile: PASS (`FLASH code 124532`, RAM1 variables
  `116416`, RAM2 variables `12416`).
- Simulator: **30/30 PASS**.
- Firmware/source liveness and safety contracts: included in **92/92 Python
  PASS**. The first full run had the previously recorded local Tk resource
  load failure; an immediate unchanged rerun passed 92/92.
- Python `compileall`: PASS.
- `git diff --check`: PASS for the Task-03 files.

Task-03 files changed: `RobotCode/RobotTypes.h`, `RobotCode/Globals.cpp`,
`RobotCode/RobotConfig.h`, `RobotCode/LocalPlanner.cpp`,
`RobotCode/Bluetooth.cpp`, `RobocupSimulator/simulator-core.js`,
`RobocupSimulator/simulator-core.test.js`,
`SerialCommandUI/live_map_model.py`,
`SerialCommandUI/test_recovery_liveness_contract.py`, and this workstream.
No firmware upload, serial access, physical movement, arc-score tuning,
clearance weakening, timeout-as-success behavior, or Task-04 work was
performed. Residual risks remain unchanged: logging transport is still the
Task-04 synchronous path, and `RANGE_FAKE_REAR`/P0-05 coverage are unresolved.
Recommended next workstream task: **Task 04 â€” Make logging nonblocking and the
runner fail-fast.**

## Task 04 â€” Make logging nonblocking and the runner fail-fast

### Copy-ready task prompt

```text
Workstream Task 04: make regression logging nonblocking and fail-fast. Confirm
Task 01 is complete before relying on physical-sign telemetry.

The failed run used synchronous 115200-baud CSV snapshots at 100 ms cadence.
A typical telemetry row required about 33 ms of wire time, event bursts were
longer, the main-loop maximum gap reached 247 ms, there were 211 deadline
misses, and the 150 ms command lease expired three times.

Refactor telemetry so regression logging cannot block the control loop long
enough to threaten the command lease. Use a bounded queue/ring buffer or a
compact staged transmitter with an explicit per-loop byte/time budget. Do not
weaken or lengthen the watchdog merely to hide logging stalls. Rate-limit
duplicate safety events and avoid emitting a full telemetry snapshot for
every event when a compact event record is sufficient. Expose dropped/queued
telemetry counters so overload is visible.

Strengthen SerialCommandUI/run_navigation_regression.py preflight and live
oracles. Abort immediately on a new lease trip, loop-gap/deadline threshold,
unexpected authority, non-neutral output when stopped, invalid/stale critical
sensors, divergence reason, or unexpected terminal event. Put STOP, TEST
DISARM, MANUAL DISARM, CSV OFF, final STATUS, and log finalization in reliable
try/finally cleanup, including exceptions and Ctrl-C.

Add host tests for fragmented/queued rows, overload counters, each abort
oracle, timeout, serial exception, Ctrl-C-equivalent cleanup, and log/summary
preservation. Add firmware/source-contract tests for the transmit budget.
Run the complete Python suite, compileall, simulator tests, and firmware
compile. Do not upload or move the robot in this task.
```

### Task 04 evidence â€” 2026-07-14

Result: **Complete.** Task 01 was confirmed Complete before implementation.
Its dated evidence above retains the physically proven left/right channel
mapping, canonical positive-left/CCW contract, signed arc/encoder/raw-yaw/nav-
yaw agreement, tested build identity, and passing Task-01 verification.

The failed path synchronously issued hundreds of `Serial2.print` operations
for every 100 ms telemetry snapshot and repeated the full snapshot for every
event. The firmware now stages each complete row in a bounded 1,792-byte
buffer, atomically admits it to a 4,096-byte ring, and drains no more than 48
bytes per main-loop iteration. Drain writes are limited to
`Serial2.availableForWrite()`, so telemetry never waits for UART capacity.
Queue admission drops an entire row rather than emitting a corrupt partial
row. `CSV OFF` discards backlog before cleanup STATUS reporting.

The watchdog was not weakened or lengthened: `MAIN_LOOP_DEADLINE_MS` remains
`60 ms` and `MOTOR_COMMAND_LEASE_MS` remains `150 ms`. STATUS and telemetry
now expose queued rows, queued bytes, dropped rows, and rate-limited event
counts. Events are compact four-field records (`event,name,detail,ms`) instead
of full snapshots, and exact duplicate events are rate-limited over 250 ms.
The desktop live-map parser and regression parser accept both compact event
records and full telemetry rows, including fragmented serial reads.

`run_navigation_regression.py` now performs a stopped/disarmed/neutral STATUS
preflight with watchdog, lease, authority, fan-validity, and fan-age checks.
During a run it aborts immediately on a new lease trip, any deadline miss or
loop maximum above 60 ms, wrong authority, non-neutral stopped output,
invalid/stale fan sensing, safety/divergence events, typed recovery aborts,
wrong test terminals, and success events with invalid result details such as
`final_blocked_reached`. All test types, including signed arc and turn truth,
use the bounded CSV path so they receive the same live oracles.

The runner's unconditional cleanup attempts, in order, `STOP`, `TEST DISARM`,
`MANUAL DISARM`, `CSV OFF`, and final `STATUS`; optional speed/escape restore
follows. Each cleanup command is isolated so one serial error cannot skip the
others. Timeout, serial exception, and KeyboardInterrupt paths preserve raw,
CSV, and JSON summary artifacts through `try/finally` finalization.

Automated evidence includes fragmented/queued-row parsing, compact events,
queue/budget/drop source contracts, unchanged watchdog constants, every live
and event abort oracle, invalid terminal results, timeout, injected serial
write failure, interruption, exact cleanup order, and artifact preservation.

Full software verification:

- Teensy 4.0 Arduino CLI compile: PASS (`FLASH code 125880`, RAM1 variables
  `123456`, RAM2 variables `12416`).
- Simulator: **30/30 PASS**.
- Python: **115/115 PASS**.
- Python `compileall`: PASS.
- Focused Task-04/turn/live-map tests: **41/41 PASS**.
- `git diff --check`: PASS for the Task-04 source/test files (existing line-
  ending conversion warnings only).

Two post-change full Python reruns encountered the previously recorded local
Tk resource-load fault (`init.tcl`/`ttk/menubutton.tcl`) with 114/115 tests
passing. A direct unchanged Tk initialization then passed, and the immediate
unchanged mandated full-suite rerun passed 115/115.

Task-04 files changed: `RobotCode/Bluetooth.cpp`, `RobotCode/RobotConfig.h`,
`RobotCode/Robot.h`, `RobotCode/RobotCode.ino`,
`SerialCommandUI/run_navigation_regression.py`,
`SerialCommandUI/live_map_model.py`,
`SerialCommandUI/test_navigation_regression_runner.py`,
`SerialCommandUI/test_telemetry_transport_contract.py`,
`SerialCommandUI/test_live_map_model.py`,
`SerialCommandUI/test_turn_sign_contract.py`, and this workstream.

No firmware upload, serial access, physical movement, planner tuning, sensor-
safety weakening, collision-veto weakening, or Task-05 work was performed.
Residual risks remain: this is deterministic software/source-contract proof,
not physical UART/watchdog timing validation; `RANGE_FAKE_REAR` and P0-05
coverage are still unresolved. Recommended next workstream task: **Task 05 â€”
Run integrated software verification.**

## Task 05 â€” Run integrated software verification

### Copy-ready task prompt

```text
Workstream Task 05: integrate and verify Tasks 01-04. Do not add new planner
features or perform physical testing.

Read the dated evidence and diffs from Tasks 01-04. Review their interaction:
turn signs across all command modes, deterministic bypass transitions,
recovery abort ownership, motion authority, watchdog/lease renewal, safety
event emission, regression-runner parsing, and cleanup. Resolve only concrete
integration failures.

Run from the repository root:
  arduino-cli compile --fqbn teensy:avr:teensy40 RobotCode
  Push-Location RobocupSimulator; npm test; Pop-Location
  python -m pytest SerialCommandUI TOFReturnSignalExperiment
  python -m compileall SerialCommandUI TOFReturnSignalExperiment

Also run focused golden traces for both signed arcs, initial right and left
bypasses, changing side evidence, reverse/post-reverse transition, bounded
successful rejoin, deliberate divergence abort, telemetry overload, lease
trip abort, and cleanup after host failure.

Completion requires every command to pass, no test to weaken a collision or
unknown-sensor stop, and a concise physical-validation matrix specifying the
exact next commands, expected telemetry, abort criteria, and required setup.
Do not upload firmware or send serial commands.
```

### Task 05 evidence â€” 2026-07-14

Result: **Complete.** The dated evidence and current implementation for Tasks
01â€“04 were reviewed together before testing. Task 01's corrected physical
left/right channel mapping remains the lowest-layer fix, and every motor mode
retains positive turn as left/CCW: ordinary wheel-PID arcs, manual drive,
`TEST ARC`, calibrated navigation/test pivots, turn-pulse diagnostics, point
alignment, and weight-scan pivots. Non-neutral commands pass through
`setAuthorizedMotionCommand()`; `MotorControl.cpp` remains the sole periodic
motor writer and rechecks authority, lease, and live motion safety before
output. Only fresh non-neutral command publication renews the 150 ms lease;
the periodic writer cannot keep a stale command alive.

The bypass and recovery review confirmed that observed side evidence cannot
overwrite the active latch. The active side is preserved through
`SIDE_ESCAPE -> FRONT_WALL_REVERSE -> POST_REVERSE_ESCAPE -> ROUTE_REJOIN`
and changes only after a stopped explicit reset plus fresh stable evidence.
All six Task-03 liveness exits retain failure ownership, neutralize before the
abort event, and cannot become a partial success. Motion-authority transitions
revoke and cancel old owners before claiming a new one. Compact safety events,
fragmented queued telemetry, overload counters, live runner oracles, and
unconditional host cleanup were reviewed and exercised together.

Two demonstrated integration failures were resolved without changing planner
scores or any safety bound:

- The current source still advertised the Task-01 build label even though
  Tasks 02â€“04 changed planner and telemetry behavior. The build identity is
  now `V7-p0-obstacle-t05`, so Task 06 can distinguish this tested source from
  the earlier physical binary.
- Firmware emits `test_arc_end,duration_complete`, but the host runner accepted
  only `duration_elapsed`. The runner now accepts the firmware's exact terminal
  detail, with a regression proving that a successful signed-arc run is not
  falsely aborted. No abort oracle was removed or relaxed.

Final focused golden evidence:

- Signed ordinary arcs: `turn=+200` produced left/right targets
  `1500/1900` and positive navigation omega; `turn=-200` produced
  `1900/1500` and negative navigation omega.
- Simulator golden traces: **13/13 PASS**, covering hard forward/reverse
  footprint vetoes, unknown/invalid inner sensing, offset and wide-panel
  bypasses, initial right latch, stopped reset and fresh initial left latch,
  changing side evidence, reverse/post-reverse/rejoin, deliberate divergence,
  every typed liveness bound, and honest bounded rejoin.
- Focused host/source contracts: **44/44 PASS**, covering turn modes, bypass
  ownership, recovery aborts, compact/overload telemetry, lease-trip abort,
  successful arc parsing, serial failure/timeout/interrupt cleanup, and
  artifact preservation.

Final required verification from the repository root:

| Command | Result |
|---|---|
| `arduino-cli compile --fqbn teensy:avr:teensy40 RobotCode` | PASS; FLASH code `125880`, RAM1 variables `123456`, RAM2 variables `12416` |
| `Push-Location RobocupSimulator; npm test; Pop-Location` | **30/30 PASS** |
| `python -m pytest SerialCommandUI TOFReturnSignalExperiment` | **116/116 PASS** |
| `python -m compileall SerialCommandUI TOFReturnSignalExperiment` | PASS |

No collision, stale-sensor, unknown-sensor, authority, lease, or divergence
guard was weakened. No firmware upload, serial access, or physical movement
was performed.

### Exact Task 06 physical-validation matrix

The gates below are sequential. Before each motor-moving command or any setup
change, Task 06 must obtain fresh operator confirmation. Every runner command
is issued from the repository root and must save `.raw.txt`, `.csv`, and
`.summary.json` artifacts. Do not send `START` or any field-GOTO command.

| Gate | Required setup and exact commands | Expected evidence and pass criteria | Immediate abort conditions |
|---|---|---|---|
| 1 â€” stationary identity and safe idle | Motors powered but robot stationary in open space; at least 1.0 m clear front/rear and 0.75 m each side; all four fan sensors uncovered. Send, in order: `BUILD`, `STOP`, `TEST DISARM`, `MANUAL DISARM`, `CSV OFF`, `ZERO`, `LOOP RESET`, `STATUS`. | `BUILD V7-p0-obstacle-t05`; `STATUS state=END_MATCH`, `run=0`, `testArmed=0`, `manualArmed=0`, `manualActive=0`, `authority=NONE`, `goalAuthority=NONE`, `watchdogReady=1`, `commandLease=0`, motors `1500/1500`, `leaseTrips` recorded as the baseline, `loopMaxMs<=60`, `loopMisses=0`, `fanValid=1/1/1/1`, every fan age `<=750 ms`. Pass only if all fields are present and safe. If BUILD differs, do not upload: stop and ask for upload permission. | Any missing/wrong build or STATUS field; active authority/lease; non-neutral output; watchdog unavailable; invalid/stale fan; blocked front in the declared open setup; loop miss/gap; unexpected motion. |
| 2 â€” raised wheel/pivot and arc truth | Chassis securely raised/blocked, drive wheels free, motors unable to propel the robot, fan view still open. Run separately after confirmation for each: `python SerialCommandUI/run_navigation_regression.py --mark t06_up_turn_p15 --test turn --angle 15 --timeout 4 --layout "RAISED; wheels free; chassis cannot translate"`; `python SerialCommandUI/run_navigation_regression.py --mark t06_up_turn_n15 --test turn --angle -15 --timeout 4 --layout "RAISED; wheels free; chassis cannot translate"`; `python SerialCommandUI/run_navigation_regression.py --mark t06_up_arc_p200 --test arc --forward 1700 --turn 200 --duration 0.40 --timeout 8 --layout "RAISED; wheels free; chassis cannot translate"`; `python SerialCommandUI/run_navigation_regression.py --mark t06_up_arc_n200 --test arc --forward 1700 --turn -200 --duration 0.40 --timeout 8 --layout "RAISED; wheels free; chassis cannot translate"`. | `+15` calibrated pivot: left wheel reverse/right forward, approximately `1200/1800 us`; `-15`: left forward/right reverse, approximately `1800/1200 us`. A fixed raised chassis may intentionally end each pivot as `test_turn_abort,turn_progress_stalled`; that is acceptable for this isolation gate only if the physical wheel signs are correct and cleanup is safe. `+200` arc targets `1500/1900`, right wheel faster; `-200` targets `1900/1500`, left wheel faster; both end `test_arc_end,duration_complete`. `imu_raw_cw` and nav yaw must be opposite signs if any rotation is measurable; near-zero yaw is expected when the chassis is fixed. Every run must end `END_MATCH`, authority `NONE`, lease off, motors `1500/1500`, no new lease trip/deadline miss, and no cleanup error. | Wrong physical wheel, target, pulse, encoder, or yaw sign; any wheel reverses during an ordinary forward arc; non-TEST active authority; safety/lease/deadline/sensor oracle; non-neutral cleanup; unexpected chassis translation. |
| 3 â€” tiny open-floor motion | Remove blocks only after confirmation. Level open floor, marked start pose, at least 1.0 m clear front/rear and 0.75 m each side; observer beside STOP control. Run separately: `python SerialCommandUI/run_navigation_regression.py --mark t06_floor_arc_p200 --test arc --forward 1700 --turn 200 --duration 0.40 --timeout 8 --contact "live observer; result in Task06 report" --layout "open level floor"`; `python SerialCommandUI/run_navigation_regression.py --mark t06_floor_arc_n200 --test arc --forward 1700 --turn -200 --duration 0.40 --timeout 8 --contact "live observer; result in Task06 report" --layout "open level floor"`; `python SerialCommandUI/run_navigation_regression.py --mark t06_floor_drive020 --test drive --distance 0.20 --timeout 12 --contact "live observer; result in Task06 report" --layout "open level floor"`; `python SerialCommandUI/run_navigation_regression.py --mark t06_floor_goto030 --test goto --x 0.30 --y 0.00 --timeout 15 --contact "live observer; result in Task06 report" --layout "open level floor"`. Reset the physical start pose before each run and reconfirm. | Positive arc is visibly forward-left/CCW with positive nav-yaw delta and negative raw-yaw delta; negative arc is forward-right/CW with negative nav-yaw delta and positive raw-yaw delta. Encoder rates remain forward-positive and match the `1500/1900` then `1900/1500` targets. Arc terminals are `test_arc_end,duration_complete`; drive/GOTO terminal is `test_*_end` with `waypoint_reached`, `route_line_reached`, or `route_line_overshoot_reached`. No contact, scrape, reverse, recovery, planner safe-stop, new lease trip, loop miss/gap over 60 ms, sensor invalid/stale event, telemetry drop increase, or cleanup error. Final STATUS is safe idle and neutral. | Contact/scrape; sign disagreement; unexpected reverse/recovery; any runner oracle or wrong/timeout terminal; telemetry drop increase that makes the run incomplete; non-neutral/owned final STATUS. |
| 4 â€” one fake-rear obstacle revalidation | Only after Gates 1â€“3 pass. Level floor; robot at marked origin and heading; soft vertical panel perpendicular to route, nearest face `0.40 m` ahead, panel about `0.55 m` wide and offset to robot-left so left-inner/outer see it while the right bypass is open; at least `1.0 m` verified empty behind the complete expected reverse sweep and `0.75 m` free on both sides; observer has immediate STOP access. After confirmation run: `python SerialCommandUI/run_navigation_regression.py --mark t06_fake_rear_avoid12 --test avoid --distance 1.20 --escape on --timeout 30 --contact "observer: report contact/no-contact and reverse/no-reverse" --layout "FAKE-REAR ONLY; soft 0.55m panel 0.40m ahead offset left; right open; >=1.0m rear; >=0.75m sides"`. | Start event under TEST authority; initial stable right decision is `observed=-1;latched=-1`. Direct `side_escape -> route_rejoin` is acceptable; if reverse is selected, the only acceptable chain is `side_escape -> front_wall_reverse -> post_reverse_escape -> route_rejoin`, always `latched=-1`. No `obstacle_bypass_side_rejected` or implicit flip. Bounds remain: goal-distance growth `<=0.45 m` beyond best, `abs(lateral)<=0.75 m`, each phase `<12 s`, cumulative recovery distance `<=2.40 m`, recovery count `<=2`, and at least `0.05 m` goal/route improvement per `6 s`. Required terminal: `test_avoid_end` with `waypoint_reached`, `route_line_reached`, or `route_line_overshoot_reached`; no contact/scrape, zero new lease trips, no loop miss/gap over 60 ms, no telemetry drop increase, and final neutral/disarmed/NONE STATUS. Record reverse/no-reverse explicitly. This remains fake-rear scaffolding and does not close P0-04/P0-05. | Any contact/scrape, side flip/rejected write, phase skip, typed recovery abort, planner/motion safe-stop, host timeout, wrong terminal, failure to reach the goal, unexpected or unsafe reverse, lease/deadline/sensor/authority oracle, telemetry evidence loss, or unsafe cleanup. Send/retain STOP cleanup and end Task 06 immediately. |

Task-05 files changed: `RobotCode/RobotConfig.h`,
`SerialCommandUI/run_navigation_regression.py`,
`SerialCommandUI/test_navigation_regression_runner.py`, and this workstream.
Residual risks: all Task-05 evidence is software/source-contract evidence;
physical UART timing and motion remain Task 06. `RANGE_FAKE_REAR` and P0-05
coverage remain unresolved. Recommended next workstream task: **Task 06 â€” Run
staged physical revalidation.**

## Task 06 â€” Run staged physical revalidation

### Copy-ready task prompt

```text
Workstream Task 06: run staged physical revalidation after Tasks 01-05 pass.
This is the only workstream task authorized to propose physical movement, and
it must still ask for my confirmation before every motor-moving command or
setup change.

First inspect the Task 05 validation matrix and current code/build status.
Propose the exact command sequence and physical setup before sending serial
commands. Do not upload unless BUILD/STATUS shows that the robot is not
running the tested source; ask first if upload is needed.

Use these gates in order:
1. Stationary BUILD, ZERO, STATUS sanity: END_MATCH, authority NONE, motors
   1500/1500, watchdog ready, lease off, required sensors valid/current.
2. Wheels-up or blocks: small positive and negative pivots and ordinary
   forward-arc sign tests; verify physical direction, encoder rates, raw/nav
   yaw, authority cleanup, and zero lease trips.
3. Tiny open-floor signed arcs and short TEST DRIVE/TEST GOTO movement through
   the regression logger; zero lease trips/deadline violations and neutral end.
4. Only if all earlier gates pass, one generous TEST AVOID obstacle run with
   clear rear space. Mark it explicitly fake-rear testing. Record contact/no
   contact, reverse/no reverse, bypass phases and side, progress bounds, and
   terminal result.

No full START route and no field GOTO execution. Stop immediately on any abort
criterion. Save raw, CSV, and summary logs for every movement. Report exact
commands, log paths, pass/fail by gate, physical observations, final safe
STATUS, and whether the workstream can close. A contact-free timeout,
divergence, side flip, lease trip, or failure to reach the defined terminal
goal is a failure.
```

## Workstream closure criteria

Close this workstream only when all six tasks are marked `Complete` and the
final physical obstacle run:

- reaches its explicitly defined terminal goal;
- has no contact or scrape;
- has no unexpected reverse or fake-rear assumption beyond the logged test
  allowance;
- preserves its latched bypass side unless an explicit stopped reset occurs;
- stays inside the declared liveness/divergence envelope;
- records zero command-lease trips and no disallowed loop-deadline violation;
- ends at neutral `1500/1500`, authority `NONE`, and disarmed state.

Closure is still not evidence of real rear safety, competition readiness, or
P0-04/P0-05 completion.
