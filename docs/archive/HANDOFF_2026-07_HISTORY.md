# Handoff

Archived status, 2026-07-30: this is the long historical handoff preserved
after the documentation cleanup. It contains old physical-validation and
planner-history notes. It is not the current handoff and does not authorize
upload, serial access, arming, or movement. Use `../../../HANDOFF.md` and
`../CURRENT_STATE_AND_NEXT_STEPS.md` for current guidance.

## Current architecture: 2026-07-30

Navigation is now a nonblocking black box for onboard mission and future pickup
code. Normal callers include only `RobotCode/Navigation.h`, submit one goal,
poll `IDLE`, `RUNNING`, `REACHED`, or `FAILED`, and decide the next mission
action themselves.

The implementation is organized under `RobotCode/src/` by ownership:

- `mission/` owns the route, route index, pauses, actions, and weight search;
- `navigation/` owns goal storage, planning, obstacle handling, recovery, map
  state, and typed navigation results;
- `motion/` owns encoders, odometry, progress monitoring, and the sole periodic
  motor writer;
- `sensors/`, `operator/`, and `core/` own their corresponding services.

`NavigationAdmin.h` contains map reset and emergency-policy controls.
`NavigationTest.h` contains simulator/test adapters. Motor commands carry an
explicit standard/drive/turn/scan-turn mode, so `MotorControl.cpp` does not read
private navigation state. Navigation no longer advances the route or changes
mission/test state.

Latest software verification:

- clean warning-enabled Teensy 4.0 compile: PASS;
- firmware/WASM and simulator suite: 47/47 PASS;
- Python suite: 141 PASS, one intentional skip;
- exact custom and `heading-back` scenarios under clean and default sensing:
  PASS as part of the firmware/WASM suite;
- Visual Lab rebuilt-WASM clear GOTO smoke test: PASS, neutral
  `waypoint_reached`, no contact;
- Python `compileall`: PASS.

No upload, serial connection, or physical movement was performed. Simulator
results are deterministic software evidence only.

Current latest work as of 2026-07-20 is on the nested `RobotCode` feature
branch `PermanentObstacles`. The user saved the old baseline separately as
`RobotCode - Copy (3)` before the local-planner rebuild work.

## Latest simulator and planner work

The RoboCup browser simulator has been upgraded into a Visual Lab for manual
scenario testing. It keeps the existing canvas/run/step/reset controls and adds
a scenario library with built-ins, browser-saved custom scenarios, and the
current draft. Custom scenarios persist in `localStorage` and can be exported
and imported as JSON. The editor supports object selection on the canvas,
numeric property editing for field objects, duplicate/delete, goal/start-pose
editing, selected run command, view toggles, CSV export, firmware CSV
comparison, and a run summary with result, elapsed time, final pose, committed
wall side, wall phase path, stop reason, minimum clearance, and event count.

The simulator now has a WASM-backed firmware engine path so local planner
tuning in `RobotCode` is reflected in simulator behavior after rebuilding the
firmware core. Simulator telemetry exposes recovery state, endpoint clearance,
clearance gain, unexplored-heading score, plateau count, and map state. The
previous fabricated free-space painting was removed from the simulated recovery
path; simulated range channels are field raycasts with explicit blocked,
invalid, and moving-obstacle behavior.

The local planner was stripped back from the older bypass/rejoin-heavy behavior
and rebuilt around a cleaner obstacle context:

- Bypass distance is derived from the obstacle envelope, robot footprint, hard
  clearance, and map cell size instead of a fixed 500 mm distance.
- Obstacle-specific state and side commitment reset once the current obstacle
  is safely cleared, so a later obstacle within the same waypoint is scanned
  and handled as an independent obstacle.
- After an obstacle clears, the planner aims directly at the waypoint instead
  of forcing a separate route-rejoin target.
- Obstacles beyond a reachable waypoint no longer force avoidance or recovery;
  the waypoint can complete from a safe final pose.
- Long connected transverse edges hold gained lateral clearance until the
  direct waypoint corridor is clear, preventing repeated recoveries along the
  same edge.

Evidence-driven reverse repositioning was added for persistent geometric
no-path cases. It samples reverse arcs over cells previously marked clear,
keeps unknown or contradicted cells blocked, scores candidates first by
clearance band and then by forward-continuation quality, unexplored-heading
visibility, sweep clearance, efficiency, and command continuity. It hands back
to forward planning when clearance gain plateaus, or stops with typed
no-useful-recovery/liveness reasons. Traversed cells are marked strongly clear,
and later obstacle evidence revokes remembered clear cells immediately.

A compact persistent arena memory layer now augments the rolling local map in
`RobotCode/LocalPlanner.cpp` and `RobotCode/RobotConfig.h`. It uses the same
50 mm cell size, a 12 m square arena-memory grid centered at map reset, separate
known-clear and occupied bitsets, thresholded challenge evidence, endpoint
interpolation, free-ray interpolation, and traversal marking. Fresh local clear
evidence can override stale remembered occupancy; repeated contradictory
observations update the persistent memory.

## Latest measured outcome

The custom long-edge test case that previously caused repeated recovery and a
back-and-forth finish improved materially after the latest planner changes:

- Runtime improved from about `25.60 s` to `15.24 s`.
- Path length improved from about `4.20 m` to `2.34 m`.
- Recovery episodes dropped from `5` to `2`.
- Reverse-command ticks dropped from `282` to `12`.
- The final reverse oscillation was removed, with no simulator contact.

Latest verification recorded in this chat:

- `Push-Location RobocupSimulator; npm test; Pop-Location`: `32/32` pass.
- `arduino-cli compile --fqbn teensy:avr:teensy40 RobotCode`: pass.
- Targeted Python planner/contract checks: `36/36` pass.
- Full Python suite: `112 passed`, `1 skipped`, plus `26` known stale
  source-shape contract failures that predate the latest planner behavior.

No firmware upload or physical movement test was performed for the latest
Visual Lab, WASM simulator, persistent-memory, obstacle-context, or recovery
work. Treat the simulator evidence as deterministic software evidence only.

Current as of 2026-07-15. The governing workstream is
`RobotCode/docs/P0_NAVIGATION_REBUILD_WORKSTREAM.md`.

## Current outcome

Tasks 01-05 are complete and recorded in the workstream. Task 05 records GO
only to separately authorized staged physical validation. Task 06 Gate 0
attempt 1 stopped before connection because `COM13` did not exist. After the
user connected and reauthorized, attempt 2 completed the stationary sequence
and proved obsolete build `V7-p0-obstacle-t06e` is running. Neutral cleanup
passed, the port closed, and the user confirmed no physical motion and final
power-off; no upload or moving command occurred.

The implemented controller is:

```text
FOLLOW_ROUTE -> TURN_OUT -> FOLLOW_WALL -> CLEAR_WALL_END
             -> REJOIN_ROUTE -> FOLLOW_ROUTE
```

There is one `WallAvoidanceState`, one side commitment, and exactly one
deterministic forward arc proposal per point-control update. All wall commands
use the same observed-distance, stopping-cap, forward-wheel, corridor, and
swept-footprint validator.

## Safety invariants

- `MotorControl.cpp` is the only periodic servo writer.
- Authority and live safety are checked on command acceptance and every motor
  update; disarm, lease, watchdog, deadline, freshness, and neutral cleanup
  remain mandatory.
- Evidence-driven reverse repositioning is implemented for WASM simulation,
  but physical publication remains disabled by `hasTrustedRearCoverage()`.
  Fake rear remains diagnostic only and is not motion or map evidence.
- Direction-aware pending-close confirmation blocks both inner beams for
  forward motion and the turning-side pair for turns. An opposite outer return
  cannot globally freeze a turn away, but never counts as free.
- Wall end requires both wall-side beams clear for three fresh frames and at
  least 168 mm additional travel before route rejoin.
- Reverse uses one cooperative candidate epoch over persistent clear cells.
  No fixed retreat, survey, route rejoin, post-reverse escape, adaptive
  widening, collision bypass, or final-blocked success remains.

## Important integrated defect fixed during Task 05

The exact 1.20 m t06e fixture initially completed wall rejoin inside the frozen
bounded overshoot window, but the point controller evaluated behind-target
alignment before route terminals. It pivoted 180 degrees and started a second
obstacle attempt. Route-circle/plane/overshoot/missed terminals now run
immediately after a completed wall update and before point alignment. The exact
t06e and mirror simulator scenarios then complete once, monotonically, without
contact.

## Evidence and tools

- Simulator fixtures use 130 mm wall thickness. `t06e_right`/`t06e_left` use
  a 1.20 m target, wall near face 400 mm beyond the chassis front, and a 550 mm
  offset face.
- Current telemetry is schema v3, 61 fields, and uses a bounded nonblocking
  queue. The host fails on missing header/meta/telemetry, wrong width/version,
  sequence/drop evidence, or unsafe final status.
- Historical logs remain under `logs/navigation_regressions/`. The canonical
  failed baseline is
  `20260715_115546_t06_fake_rear_avoid12_t06e`; legacy parsing is isolated and
  cannot pass the current evidence oracle.
- The old sampled approach-envelope harness, reverse/escape regression path,
  candidate/recovery live-map fields, and stale operational documentation were
  removed.

## Task 05 closure evidence

- Scoped production count: `3,562` physical lines versus `5,408` at Task 01,
  a net `-1,846`; avoidance enum/state/functions: `437` lines.
- Exact t06e and mirror: `8.02 s`, `2158 ticks/s` median avoidance command,
  `0.0%` neutral, `86.3 mm` minimum reported clearance, no contact, one side,
  exact monotonic phases, and `route_line_overshoot_reached`.
- Full gate: Arduino Teensy 4.0 compile PASS; simulator `16/16`; Python
  `136 passed, 1 skipped`; compileall PASS.

## Residual risks and next action

Physical signed-arc, useful-speed wall following, clearance, ToF edge behavior,
and timing/lease performance remain unproven. Low central and complete
rear/side sensor coverage are still open P0 issues; reverse stays disabled.

Do not continue Task 06 until the user separately authorizes the frozen upload
setup and command. The build mismatch permits an upload to be considered, but
does not authorize one. Ask again before the Gate 0 retry, each moving command,
and every setup change. Never run full `START`, field GOTO, or reverse.

The user subsequently confirmed no motion and final power-off. The exact
main-power-off/USB-only upload gate is frozen in the workstream.

The user later supplied standing compile/upload authorization. Compile passed
and the upload command exited zero, but no post-upload `BUILD`/`STATUS` check
has yet confirmed the running identity. No retry, serial action,
hardware-button press, or setup change followed. Next run the frozen stationary
Gate 0 identity/safety sequence only after separate authorization; standing
authorization does not broaden any serial or motion permission.

The failed attempt and frozen Gate 0 setup, six-command sequence, expected replies, complete
`STATUS` oracle, abort rules, cleanup, and conditional upload boundary are
recorded under **Task 06 Gate 0 permission record — pending** in the governing
workstream. Do not improvise or combine it with a movement gate.

Gate 0 attempt 3 subsequently ran after the user confirmed the frozen blocked
setup ready. `BUILD` returned `V7-p0-nav-task05`; the complete host-side
`STATUS` oracle passed with `END_MATCH`, all modes disarmed, both authorities
`NONE`, `1500/1500`, zero command/targets/rates, watchdog ready, lease inactive
with zero trips, loop maximum `17 ms` and zero misses, four current valid fan
beams (maximum age `30 ms`), valid virtual front, clear/not stuck, and zero
telemetry drops. COM13 closed cleanly and no moving command was sent. Gate 0
awaits only the observer's no-motion/anomaly confirmation; no later setup or
movement gate is authorized.

The user then explicitly authorized one Gate 1 positive wheels-up arc:
`TEST ARC 1700 200 0.4`. It completed with declared targets `1500/1900`,
positive encoder totals `175/392`, raw/nav yaw `-0.06/+0.06 deg`, the expected
`duration_complete` terminal, 14 complete schema-v3 telemetry rows, zero
lease/deadline/drop/safety/cleanup fault, and final neutral/disarmed/NONE
status. Evidence prefix is
`logs/navigation_regressions/20260715_152835_t06_nav_up_p200`. Host oracle is
PASS, and the observer subsequently confirmed both wheels forward, the right
wheel faster than the left, secure chassis, and no physical anomaly. Gate 1A
passes. The mirrored arc is not authorized.

The user then authorized Gate 1B `TEST ARC 1700 -200 0.4`. It completed with
declared targets `1900/1500`, final in-motion rates `1150/450`, raw/nav yaw
`+0.06/-0.06 deg`, `duration_complete`, 13 complete rows, zero safety/timing/
lease/drop/cleanup fault, and final neutral/disarmed/NONE status. Evidence
prefix is `logs/navigation_regressions/20260715_153047_t06_nav_up_n200`.
Host oracle is PASS; observer confirmation of actual wheel directions,
left-faster/right-turn sign, secure chassis, and no anomaly was subsequently
received. Gate 1 passes. The user also confirmed the separately specified
level open-floor setup for the single `TEST DRIVE 0.200` Gate 2 command.

Gate 2 attempt 1 failed safely on the floor. Build `V7-p0-nav-task05` saw a
roughly 650 mm virtual-front return, unconditionally entered right-side wall
avoidance for the 0.20 m goal, passed the target, and stopped on
`wall_goal_divergence` at estimated pose `(0.483,-0.464) m`, heading
`-68.37 deg`. Cleanup was fully neutral/disarmed/NONE with zero lease, deadline,
sensor, drop, or artifact fault. Evidence prefix is
`logs/navigation_regressions/20260715_153446_t06_nav_floor_d020`. Operator
later confirmed no contact, scrape, abnormal sound, vibration, or damage; the
unintended turn was the only observed anomaly.

The demonstrated defect is fixed: wall acquisition now declines both normal
and rejected-arc entry when the point goal plus the existing 230 mm front-clear
reserve ends before the observed obstacle. Simulator parity and an exact
0.20 m/650 mm regression were added. Corrected identity is
`V7-p0-nav-task06a`; compile PASS, simulator 17/17, Python 136 passed/1 skipped,
compileall PASS. Current scoped size is 3,578 lines (`-1,830` from Task 01),
avoidance approximately 453 lines. No physical retry is authorized until the
new image is uploaded and confirmed by a fresh Gate 0.

The user authorized the corrected upload and conditional stationary identity
gate with the wheels clear. Compile and upload exited zero; `BUILD` then
returned `V7-p0-nav-task06a`. The complete stationary oracle passed:
END_MATCH, disarmed, authorities NONE, safety clear, watchdog ready, lease
inactive/zero trips, loop maximum 18 ms/zero misses, all fan evidence valid and
current (maximum age 51 ms), zero telemetry drops, neutral 1500/1500, and zero
command/targets/rates. COM13 closed cleanly and no moving command was sent.
Corrected identity is proven running; Gate 2 retry is not yet authorized.

Gate 2 attempt 2 on `V7-p0-nav-task06a` stayed in FOLLOW_ROUTE and moved only
about `(0.060,-0.001) m`, heading `-1.56 deg`, before a sudden right-inner
pending-close event safely neutralized motion. The point controller then
incorrectly terminal-aborted `arc_close_pending_relevant` instead of holding
for confirmation. Cleanup was neutral/disarmed/NONE with zero lease/deadline/
drop/artifact fault. Evidence prefix is
`logs/navigation_regressions/20260715_154557_t06a_floor_d020_r2`; operator
contact/path observation is pending.

This demonstrated defect is fixed with a distinct
`TRAJECTORY_PLAN_CONFIRMATION_PENDING` result handled neutrally by route, wall,
and wall-rejoin callers. The rejected-arc caller also now returns only when
wall acquisition actually accepts. Corrected identity is
`V7-p0-nav-task06b`; compile PASS, simulator 17/17, Python 136 passed/2 GUI
environment skips, compileall PASS. Scoped size is 3,591 (`-1,817`), avoidance
approximately 466 lines. No upload or physical retry is authorized yet.

The user then authorized a direct real-wall run after confirming the declared
130 mm soft-wall fixture (400 mm front gap, about 550 mm face length, right
bypass open with at least 750 mm side clearance). Build
`V7-p0-nav-task06b` passed compile/upload and the stationary identity/safety
oracle. `TEST AVOID 1.200` committed right and reached the exact monotonic
phases through REJOIN_ROUTE, confirmed the observed wall end, and traveled the
required 168 mm rear-clear distance. It nevertheless failed: TURN_OUT held its
fixed arc to about -126 degrees because its stability counter demanded a near
wall return even after the leading edge left the side rays. The resulting
rejoin arc was correctly rejected by the swept-footprint veto. Cleanup passed
fully neutral/disarmed/NONE with zero lease trips, loop misses, or telemetry
drops and no reverse. Evidence prefix is
`logs/navigation_regressions/20260715_155409_t06b_bypass120_r`; operator
contact/path/anomaly observation remains pending.

That defect is fixed in `V7-p0-nav-task06c`: after the bypass side's existing
two-frame near-wall commitment, TURN_OUT requires two fresh/current/non-pending
side-pair frames plus the fixed 42-degree heading, not continued near returns.
A fixed 65-degree maximum fails neutral if completion is unavailable. Mirrored
clear-edge and hard-limit regressions were added. Full gate passes: Teensy
compile (132,472 B flash code; RAM1 123,200 B; RAM2 12,416 B), simulator 20/20,
Python 138/138, compileall. Scoped production is 3,598 lines (`-1,810`), with
avoidance approximately 473 lines. Physical testing is stopped. Next boundary
is one combined task06c upload/stationary identity check followed conditionally
by the same real `TEST AVOID 1.200`; do not repeat blocks or tiny-floor gates.

The user confirmed no contact, reset the fixture, and authorized that boundary.
Upload and stationary identity passed on `V7-p0-nav-task06c`. The direct bypass
then stopped safely at about `(0.094,-0.017) m`, heading `-26.06 deg`: during
the right TURN_OUT, right-outer produced one invalid 0 mm sample. The final
motor owner immediately emitted `turn_side_invalid` and neutralized; wall
avoidance failed closed as `wall_required_sensor_stale`. Cleanup was
END_MATCH/disarmed/NONE/1500-1500 with zero lease trips, loop misses, or drops,
but final `safetyStop=1` retained the diagnostic and therefore does not pass
closure. Evidence prefix is
`logs/navigation_regressions/20260715_155945_t06c_bypass120_r`. This behavior
preserves the required unknown-is-blocked rule, so no sensor bypass or software
change was made. Operator observation and fixture reset are now required before
any separately authorized repeat.

The operator confirmed no contact/anomaly, reset the fixture, and authorized a
direct repeat. It had clean sensing and proved task06c TURN_OUT physically,
transitioning at about -44 degrees. It then failed on a new real defect: three
clear wall-side samples immediately after TURN_OUT were counted as a wall end
before FOLLOW_WALL had ever observed the committed wall. CLEAR_WALL_END began
near `(0.185,-0.091) m`; the wall then appeared at 483 mm and the monotonic
controller correctly aborted `wall_reacquired_after_end`. Cleanup was fully
neutral/disarmed/NONE/safety-clear with zero lease/deadline/drop fault. Evidence
prefix is `logs/navigation_regressions/20260715_161155_t06c_bypass120_r2`;
operator observation for this latest attempt remains pending.

Corrected `V7-p0-nav-task06d` adds only an observed-before-clear invariant:
FOLLOW_WALL must first see the committed wall below the existing 500 mm
threshold before three fresh clear samples can prove its end. A regression
freezes this near-to-clear sequence. The old asymmetric generic left fixture,
which could not supply wall-side evidence and had passed only through the bug,
was corrected to mirrored route-centred geometry. Full gate passes: firmware
compile (132,536 B flash; RAM1 123,200 B; RAM2 12,416 B), simulator 21/21,
Python 137 passed/1 GUI skip, compileall. Scoped production is 3,605 lines
(`-1,803`), avoidance approximately 480. Next boundary is task06d upload,
stationary identity, then conditionally the same direct 1.2 m bypass.

The operator confirmed no contact/anomaly, reset the fixture, and authorized
that boundary. Upload/stationary identity passed on task06d. The direct run
proved observed-before-end behavior but stopped safely during FOLLOW_WALL on
`arc_footprint_rejected` near `(0.394,-0.283) m`, heading `-42.13 deg`. The
committed left wall pair never acquired below 500 mm while right/front returns
closed to about 210/232 mm. Cleanup was fully safety-clear, neutral, disarmed,
NONE, with zero lease/deadline/drop fault. Evidence prefix is
`logs/navigation_regressions/20260715_161702_t06d_bypass120_r`. Do not tune
curvature or weaken the veto yet: the physical wall's lateral edge coordinate
was never recorded. The exact simulator has the right/bypass edge 100 mm to
robot-left of the starting centreline. Next obtain the latest no-contact report
and measure that physical edge offset; this distinguishes setup geometry from
a fan-frame/mapping defect before another movement.

The operator clarified that task06d first aimed around the obstacle and then
drove toward its side wall. That matches the immediate software defect:
FOLLOW_WALL began without acquiring the committed wall, and its `>500 mm`
distance-band branch steered inward until footprint rejection. This has been
simplified in `V7-p0-nav-task06e`. TURN_OUT now requires both at least 42
degrees outward heading and two fresh/non-pending committed-side readings below
the existing 500 mm threshold. Without acquisition by 65 degrees it fails
neutral; FOLLOW_WALL is never entered. The temporary observed-wall latch is
removed. The endpoint grid stays collision-veto evidence, not a steering target
or new wall map. Full gate passes: firmware compile (132,536 B flash; RAM1
123,200 B; RAM2 12,416 B), simulator 21/21, Python 137 passed/1 GUI skip,
compileall. Scoped production is 3,596 lines (`-1,812`), avoidance approximately
471. Latest task06d contact/anomaly confirmation remains pending; next boundary
is task06e upload/stationary identity plus the same direct bypass.

The operator confirmed no contact, reset the fixture, and authorized task06e.
Upload/stationary identity passed. The simplified gate stayed in TURN_OUT until
about -65.62 degrees, then acquired a non-parallel left pair around 428/954 mm.
FOLLOW_WALL commanded straight from the nearer-value band and the single arc
was footprint-rejected as all forward fan sectors closed; final pose was about
`(0.253,-0.227) m`, heading `-72.25 deg`. Cleanup was fully clear/neutral/
disarmed/NONE with zero lease/deadline/drop fault. Evidence prefix is
`logs/navigation_regressions/20260715_162410_t06e_bypass120_r`; latest operator
contact confirmation is pending. Offline fixed-heading sweeps from 75 through
90 degrees failed every authoritative/mirrored fixture because the forward fan
loses the wall near parallel. No production tuning followed. Task 06 is now at
an explicit observability/design blocker: either verified lateral sensing is
needed, or the workstream must be amended to authorize a deterministic wall
line/edge estimate from existing observations. A single saved endpoint is not
enough to prove wall orientation/end, and collision or wall-end safety must not
be weakened.

The operator then rejected further threshold/latch patching and requested a
small controller with direct tuning knobs after reporting that attempt 5 turned
hard right toward the side wall. Build `V7-p0-nav-task06f` replaces the coarse
near/far band with a projected two-ray proportional controller. Each observed
wall-side endpoint is projected onto the robot lateral axis. Two observed rays
provide mean standoff plus alignment error; one provides standoff only; no
observed rays produce zero correction while the unchanged paired wall-end proof
runs. This avoids averaging a close wall return with an open-boundary return.

The three wall-follow behavior knobs are target `350 mm`, alignment gain
`0.00060/mm`, and distance gain `0.00100/mm`; the existing maximum turn ratio
clamps the one proposed forward arc. The turn-out hard bound is now `75 deg`,
which contains the physically observed acquisition near `65.62 deg` without
adaptive widening. No map steering, fitted wall, new phase, recovery, reverse,
or safety bypass was added. Software gate passes: firmware compile
(`132,792 B` flash, RAM1 `123,200 B`, RAM2 `12,416 B`), simulator `23/23`,
Python `137 passed, 1 skipped`, and compileall. Scoped production is `3,612`
lines (`-1,796`); avoidance is approximately `488` lines. Task06f has not been
uploaded or physically tested, and attempt 5's contact/damage/anomaly result is
still pending.

The operator confirmed no damage from attempt 5, reset the fixture, and
authorized task06f upload plus one direct bypass. The first runner invocation
hit firmware sensor initialization and aborted before arm/motion. The repeat
passed stationary preflight, committed right, and entered FOLLOW_WALL near
`-66 deg`, but then continued right to about `-89 deg`. The unchanged footprint
veto stopped it around `(0.229,-0.240) m` with front-fan clearances near
`199..215 mm`. Cleanup was safe/neutral/disarmed/NONE with zero lease trips,
loop misses, or dropped rows. Evidence prefixes are
`logs/navigation_regressions/20260715_163723_t06f_bypass120_r` and
`logs/navigation_regressions/20260715_163736_t06f_bypass120_r2`; latest
operator contact/path/anomaly observation is pending.

The trace proves a sign defect rather than a tuning failure. For the left wall,
the inner and outer rays projected around `149/573 mm`; becoming parallel
requires a left correction, but `inner - outer` commanded right. Build
`V7-p0-nav-task06g` changes only that definition to `outer - inner` in firmware
and simulator parity. Mirrored physical-geometry regressions freeze both signs.
No gains, state, threshold, latch, recovery, map, safety, or motor path changed.
Full gate passes: firmware compile (`132,792 B` flash, RAM1 `123,200 B`, RAM2
`12,416 B`), simulator `25/25`, Python `137 passed, 1 skipped`, compileall.
Scoped production is `3,616` lines (`-1,792`); avoidance approximately `492`.
Task06g is not uploaded or physically tested.

The operator reported task06f safe, reset the fixture, and authorized task06g.
Upload and stationary preflight passed. The corrected sign did command left
(`plannerW` about `+588`) when both wall rays appeared, but only one frame
before footprint rejection at about `234 mm` clearance. TURN_OUT had continued
right while waiting for acquisition, and the forward-biased inner ray alone had
been treated as lateral standoff. Final pose was about `(0.268,-0.224) m`,
heading `-69.44 deg`; cleanup was fully safe/neutral/disarmed/NONE with zero
lease trips, loop misses, or telemetry drops. Evidence prefix is
`logs/navigation_regressions/20260715_164326_t06g_bypass120_r`; latest operator
contact/path/anomaly result is pending.

Build `V7-p0-nav-task06h` removes those assumptions without a new state or
constant. TURN_OUT arcs only to the fixed 42-degree target, then goes straight
while waiting; stable acquisition uses the existing 700 mm acquisition
distance. Inner 20-degree evidence alone produces zero correction, outer
60-degree evidence alone supplies standoff, and both rays supply standoff plus
alignment. Full gate passes: firmware compile (`132,856 B` flash, RAM1
`123,200 B`, RAM2 `12,416 B`), simulator `28/28`, Python `137 passed, 1
skipped`, compileall. Scoped production is `3,619` lines (`-1,789`), avoidance
approximately `495`. Task06h has not been uploaded or physically tested.

The operator confirmed task06g had no contact/anomaly, reset the fixture, and
authorized task06h upload plus one bypass. Upload and stationary preflight
passed, but the serial stream corrupted the MARK/TEST ARM portion. Firmware
remained disarmed and rejected TEST AVOID; encoder/pose/motor evidence stayed
exactly zero/neutral and no movement occurred. Cleanup was fully safe. Evidence
prefix is `logs/navigation_regressions/20260715_165122_t06h_bypass120_r`.

The runner now requires a second STATUS immediately before motion proving
END_MATCH, testArmed=1, TEST authority, manual modes off, neutral outputs,
safetyStop=0, watchdog ready, inactive lease, and fresh valid fan sensing. A
lost-arm regression proves the movement command is omitted. Full gate passes:
firmware compile, simulator `28/28`, Python `138 passed, 1 skipped`, compileall.
Task06h is uploaded but still has no physical movement evidence; serial/physical
testing is stopped after the transport anomaly.

The operator authorized one task06h retry. The new armed-status oracle passed,
then the robot committed right and held approximately `-44 deg` as intended.
The left inner ray acquired from about `1188` down to `667 mm`, but the left
outer ray remained open at `8190 mm`; FOLLOW_WALL therefore had no alignment
observation. At the same time the right fan collapsed to roughly `239/249 mm`.
The unchanged footprint veto stopped the only arc near `(0.368,-0.249) m`,
heading `-46.00 deg`, with about `192 mm` clearance. Cleanup was fully safe,
neutral, disarmed, NONE, with zero lease trips, loop misses, or telemetry
drops. Evidence prefix is
`logs/navigation_regressions/20260715_165936_t06h_bypass120_r2`; operator
contact/path/anomaly confirmation is pending.

This is now a concrete design blocker, not a gain-tuning problem. The physical
fan cannot provide the two-ray wall alignment used by every passing simulator
fixture, and the opposite side becomes the collision boundary first. Gains
cannot create missing evidence. Inner-only steering already produced the hard
right failure; weakening the footprint veto is forbidden. Task 06 and the
workstream remain NO-GO/incomplete pending an explicit scope decision for
verified lateral sensing or a different deterministic geometric controller.
No further physical retry or tuning patch is justified under the frozen KISS
contract.

The operator subsequently requested an explicit rollback to the Task 06E
decision boundary. Current firmware and simulator behavior are therefore
`V7-p0-nav-task06e`: fixed TURN_OUT, the coarse `260..500 mm` nearer-ray
distance band, and the original Task 06E thresholds. Task 06F-06H controller
changes and their simulator-only regressions are historical evidence only, not
the active architecture. Their saved physical logs remain part of the audit.
The host runner's mandatory armed `STATUS` oracle remains because it is an
independent fail-closed serial safety/evidence fix. No upload or physical test
was performed for this rollback.

The operator then approved the confirmed-edge/offset-waypoint amendment.
Current build `V7-p0-nav-task06i-edge` removes the distance-band steering:
TURN_OUT requires two newer observations of the same bypass-side occupied-grid
edge, freezes one route-forward/side-clear waypoint, and FOLLOW_WALL submits one
validated forward arc toward it. Live end confirmation and 168 mm rear clear
remain unchanged. The initial face-point draft failed simulator geometry and
was discarded before upload. The retained map-edge version passes simulator
21/21; full gate and physical evidence must still be recorded before closure.

The full task06i software gate subsequently passed: compile `133,304 B` flash,
RAM1 `123,232 B`, RAM2 `12,416 B`; simulator 21/21; Python 137 passed/2 skips;
compileall. Upload to COM13 exited zero. Two runner starts then failed before
port open with access denied and sent zero commands, so there was no arm or
motion. Evidence prefixes: `20260715_173524_t06i_edge_bypass120_r` and
`20260715_173601_t06i_edge_bypass120_r2`. Arduino IDE/background serial
processes remain active; obtain permission to release the serial monitor before
retrying the already planned direct bypass.

The operator requested another COM13 attempt after releasing the port. It
opened, but stationary STATUS proved obsolete `V7-local-planner` and the runner
aborted `preflight:watchdog_not_ready`; it sent no CSV arm or movement command
and cleaned up END_MATCH/disarmed/1500-1500. Evidence prefix:
`20260715_173903_t06i_edge_bypass120_r3`. Treat the previous Arduino CLI
exit-zero message as a failed upload: the task06i identity is not running.
Obtain explicit authorization for a corrected Teensy-port upload and repeat
the stationary identity oracle before any movement retry.
