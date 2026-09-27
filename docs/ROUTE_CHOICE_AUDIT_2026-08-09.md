# Route Choice Audit — 2026-08-09

> **Completed implementation/evidence record.** The baseline plan and residual
> opportunities below are retained for diagnosis, not as an active backlog.
> Current priorities are defined only by `../../HANDOFF.md`.

## Implemented outcome and change review

This section records the implementation performed after the baseline audit
below. The original findings are retained so the before/after reasoning remains
auditable.

The main path choices are now tighter and deterministic without reducing the
hard collision margin:

| Route | Baseline | Implemented result |
| --- | --- | --- |
| KISS wall - bypass left | 2.768 m, 288.6 deg rotation, 0.816 m lateral excursion | 2.331 m, 234.5 deg, 0.610 m; forced left end used |
| KISS wall - bypass right | Fixture did not actually force right; planner used left | Mirrored blocker now forces right; 2.342 m, 235.2 deg, 0.611 m; right end used |
| Diagonal clearance | 2.399-2.517 m, 309-320 deg, approximately 1.0 m lateral excursion | 1.199 m, 177.0 deg, approximately 0.27 m lateral excursion; shorter geometric end used |
| 2026 arena demonstration | Clean sensing stalled on leg 2 | Clean and website-default both complete in 18.24 s; 4.122 m travelled for 4.000 m of route |
| G1-G5 adverse seed 20260659 | G2-G5 failed or switched sides | All five complete and retain the selected open side |
| Both-sides-blocked wall | Contact under noisy seeds 20260621/20260622 | Both seeds fail neutral without contact |
| Persistent front invalidity / stuck wheels | Stayed active and neutral indefinitely | Typed neutral failure in 2.56 s / 0.04 s respectively |

### Review of each implemented change

1. **Geometric side choice before raw range preference.** Compact obstacles
   that are wholly offset from the route now use target/corridor feasibility and
   route cost before the outer-fan range comparison. A shallow face that
   straddles the route keeps the fan as a frame-aware tie-breaker. This removes
   the diagonal long-way choice while preserving the dropped-edge-cell behavior
   exercised by G1-G5. If neither side has a safe target or corridor, the choice
   remains unresolved rather than inventing a bypass.

2. **Stable initial obstacle face.** For an initially aligned, genuinely
   asymmetric observation, the first front-face position is latched for the
   outward approach stage. Later map growth can still enlarge the obstacle, but
   a boundary cell behind the robot can no longer drag the local target
   backwards and create the KISS-wall S curve. The latch is not used for
   anti-aligned starts, ambiguous fan evidence, or after a side reconsideration.

3. **Arrival-circle sampling allowance.** The waypoint tolerance increased from
   60 mm to 65 mm. The five-millimetre change covers the observed discrete
   minimum-speed sample miss that made the diagonal route pass the intended
   arrival circle, rotate approximately 140 degrees, and return. It is smaller
   than one 50 mm map cell and does not alter collision clearance.

4. **Narrow contradictory-fan safety rule.** Blanket map inflation and generic
   65-80 mm endpoint guards were tested and rejected because they broke valid
   narrow routes. The retained rule blocks only the measured blind-wedge
   signature: both outer projected clearances are close while both inner rays
   report a long open centre. The same pure policy is compiled into firmware and
   WASM, with compile-time positive and negative checks. This fixes the noisy
   both-sides-blocked contact without globally widening every obstacle.

5. **True mirrored wall fixtures.** The right-hand KISS scenario now contains a
   mirrored blocker that actually closes the left end. This makes its name and
   acceptance oracle meaningful; left and right routes now mirror one another
   within 11 mm of path length and 0.7 degrees of accumulated rotation.

6. **Bounded invalid-sensor terminal.** A point goal remains immediately
   fail-closed during an invalid front frame, but continuous invalidity now ends
   the goal neutral after 2.5 seconds. A fresh frame clears the timer. The timer
   is reset at goal start and terminal cleanup, preventing state leakage between
   goals.

7. **Simulator stuck-state parity.** The simulator now passes its actual wheel
   stuck state through the firmware bridge. This exposed the production progress
   watchdog to the UI scenario instead of leaving a permanently active neutral
   simulation.

8. **Permanent route and failure regressions.** Added exact forced-left,
   forced-right, diagonal-budget, G1-G5 adverse-seed, blocked-wall noisy-seed,
   persistent-front-invalid, and wheel-stuck assertions. These check side,
   contact, terminal state, path length, lateral excursion, and rotation where
   applicable; success alone is not sufficient.

No periodic motor output was added outside `MotorControl.cpp`. The final-writer
simulator bridge now evaluates the same diagonal warning as firmware, closing a
parity gap rather than adding another motion owner.

### Final scenario census

The regenerated audit covers 34 current cases and 230 clean,
website-default, and stress-seed executions. There are no collisions. All
built-in UI routes and all formal acceptance routes have the intended outcome.
Eight executions across four retained legacy/adversarial fixtures still fail:

- `custom-test-1` fails stress seeds `20260621` and `20260622`. It begins only
  200 mm from the arena edge and contains duplicated panels, leaving roughly
  31 mm of planner spare space. Relaxing the footprint proof for this malformed
  fixture would be unsafe.
- `heading-back-start-right` fails the website-default seed.
- `heading-back-start-left` fails stress seed `20260659`.
- `heading-back-goal-right` fails website-default and stress seeds `20260621`,
  `20260633`, and `20260659`.

Every residual run stops neutral without contact. These are the next legitimate
recovery/heading-back workstream; they were not hidden by adding collision
bypasses, unproven reverse, or broader clearance exceptions.

The G1-G5 fixture geometry still does not literally create five distinct named
right-hand gaps: all five valid paths use the same physically open upper
corridor. The planner now makes that choice consistently, including the adverse
seed, but the fixture itself should be redesigned before drawing conclusions
about minimum traversable gap width.

### Final evidence identity

- Firmware/WASM source hash:
  `4aeef91df0bdc2f2b80d8ba9c32c91d7d5f6a66e2a7a6dcec5bf6b78ab548e21`
- Teensy 4.0 compile: pass (`168376` code bytes).
- Firmware/WASM simulator: `87/87` pass.
- Python tests: `143` pass, `1` intentional skip.
- Python compile checks: pass.
- Full route atlas: `34` cases, `230` executions, `0` contacts,
  `8` unexpected safe failures in the four legacy fixture families listed
  above.
- The existing dirty worktree was preserved. No firmware upload, serial
  connection, arming, or physical movement was performed.
- Regenerated artifacts: `tmp/route-choice-audit/atlas.svg`,
  `tmp/route-choice-audit/summary.csv`, and
  `tmp/route-choice-audit/results.json`.

## Baseline outcome (before implementation)

The current formal suite passes, but several routes are not sensible enough to
accept as efficient navigation. The most important result is a safety defect:
the built-in `KISS wall - both sides blocked` scenario makes simulated physical
contact under noisy seeds `20260621` and `20260622`. Route tightening must wait
until that trace is protected by a permanent ground-truth-clearance regression.

The main efficiency defects are:

1. `KISS wall - bypass left` takes an unnecessary S-shaped approach.
2. `Diagonal clearance` chooses the long side of the obstacle and travels more
   than twice the length of a conservative grid reference route.
3. G1–G5 do not exercise their named right-side gaps. All five take essentially
   the same upper detour, and G2–G5 fail under seed `20260659`.
4. Several retained custom/heading-back neighbors still depend strongly on the
   sensor seed and may reverse repeatedly, violate the 20 mm simulated body
   margin, or fail despite a feasible route.
5. The competition demonstration completes with website-default sensing but
   stalls indefinitely on its second leg with clean sensing.

No navigation, safety, mission, or simulator production source was changed for
this audit.

## Baseline evidence identity and scope

- RobotCode/WASM source hash:
  `cb93815c63763e6991945b8808d4983210c222a815d437b056857250dfb64695`
- WASM protocol: `2`
- WASM generated UTC: `2026-08-09T02:12:35.1164154Z`
- The existing dirty worktree was preserved.
- No firmware upload, serial connection, robot arming, or physical movement was
  performed.

The audit covered:

- all 19 built-in UI scenario IDs, with the `g0_right`/`wall_right` duplicate
  counted once as geometry;
- all 9 saved JSON fixtures;
- all 12 distinct natural static/dynamic route geometries embedded only in the
  test code;
- the forced emergency, pickup, sensing, and low-level tests by their typed
  oracles rather than by route aesthetics.

This produced 39 distinct natural route/failure geometries. The main audit ran
230 clean/default/stress-seed point-route replays. A further 35 noisy expected-
failure replays and 10 clean/default runs of the remaining test-only geometries
brought the route audit to 275 executions.

Artifacts:

- `tmp/route-choice-audit/atlas.png` — clean and website-default path overlay.
- `tmp/route-choice-audit/summary.csv` — comparable route metrics.
- `tmp/route-choice-audit/results.json` — sampled paths and transition detail.
- `tmp/route-choice-audit.mjs` — audit runner.

The conservative reference path used below is an orientation-free 25 mm grid
search with 143 mm centre clearance. It is a comparison aid, not a replacement
for the firmware's complete oriented-footprint proof.

## Scenario review

### Built-in UI scenarios

| Scenario | Result | Route judgment | Evidence |
| --- | --- | --- | --- |
| Stage 1 — clear GOTO | Pass | Sensible | 0.450 m travelled for a 0.500 m goal; early stop is within the point tolerance. |
| Stage 1 — clear START | Pass | Sensible | 3.900 m for the 4.000 m rectangle, no avoidance or reverse. |
| KISS wall — bypass right (`g0_right`/`wall_right`) | Pass | Route acceptable, fixture misleading | 2.343 m versus 2.152 m reference, but the planner actually commits left (`+1`). Nothing in the fixture forces a right bypass. |
| KISS wall — bypass left | Pass | Inefficient | 2.768 m versus 2.152 m reference; 288.6° rotation; 0.816 m lateral excursion. |
| KISS long wall face | Pass | Sensible for geometry | 2.482 m versus 2.264 m reference; no reverse. |
| Task-01 t06e right | Pass | Acceptable | 1.592 m versus 1.407 m reference; 249° rotation is higher than desirable. |
| Task-01 t06e left | Pass | Sensible | 1.486 m versus 1.407 m reference. |
| Pressed | Expected failure | Safe but low-value recovery | Fails collision-free after 12.34 s, but moves only about 20 mm during one reverse episode. |
| KISS wall — both sides blocked | Expected failure | **Unsafe under noise** | Seeds `20260621` and `20260622` contact the wall after about 0.348 m. Planner clearance remains 29–63 mm while actual simulated body margin falls below 1 mm. |
| G1 — 700 mm | Pass in 8/8 | Fixture does not test its claim | Takes the same upper route as G2–G5 instead of the named right gap. Seed-dependent side selection appears. |
| G2 — 600 mm | Pass in 7/8 | Brittle and mis-specified | Seed `20260659` chooses the opposite side, enters nine reverse episodes, and fails. |
| G3 — 520 mm | Pass in 7/8 | Brittle and mis-specified | Seed `20260659` fails; actual margin reaches 14.3 mm. |
| G4 — 480 mm | Pass in 7/8 | Brittle and mis-specified | Seed `20260659` fails; actual margin reaches 17.4 mm. |
| G5 — 460 mm | Pass in 7/8 | Brittle and mis-specified | Seed `20260659` fails after four reverse episodes; actual margin reaches 6.1 mm. |
| Diagonal clearance | Pass | **Grossly inefficient** | 2.399–2.517 m versus 1.104 m reference, about 309–320° rotation, and about 0.98–1.00 m lateral excursion. The shorter lower/right side is feasible. |
| Degraded sensing | Neutral indefinitely | Safe output, incomplete terminal contract | Remains active and neutral with `front_invalid`; the scenario never produces a typed bounded failure. |
| Stuck safe stop | Neutral indefinitely | Incomplete liveness contract | Remains active and neutral for the full audit horizon instead of terminating `FAILED`. |
| 2026 arena demonstration | 1/2 | Not a reliable demonstration | Website-default completes in 22.0 s. Clean sensing stalls on leg 2 for at least 120 s with `no_safe_trajectory`/command-age guard. |

### Saved regression fixtures

| Scenario | Result | Route judgment | Evidence |
| --- | --- | --- | --- |
| `check-this` exact | Pass in 8/8 | Wide but largely geometry-required | 3.224–3.327 m; one open wall end; no reverse. About 14% above the grid reference. |
| `check-this` wall-end reduced | Pass in 8/8 | Acceptable | 3.070–3.296 m; no reverse. |
| `check-this` stage-transition reduced | Pass in 8/8 | Acceptable | 3.288–3.402 m; no reverse. |
| `custom-test-1` | Pass in 6/8 | **Still intermittent** | Seeds `20260621` and `20260622` fail after only 0.28 m. Successful paths vary from 2.676 to 3.673 m and can acquire two obstacle contexts. |
| rollout-progress reduced | Pass in 8/8 | Sensible | 1.531 m clean/default; 1.531–1.844 m stress. |
| heading-back boundary pocket reduced | Pass in 8/8 | Acceptable constrained route | 2.862–2.910 m; the upper side is the only feasible end. |
| heading-back | Pass in 8/8 | Long but justified | 3.864–3.937 m versus 3.585 m reference; no reverse. |
| heading-back heading-right | Pass in 8/8 | Long but justified | 3.853–3.928 m; no reverse. |
| heading-back start-right | Pass in 7/8 | **Default-sensor hole** | Clean and six neighboring stress seeds pass, but website seed `20260620` fails after three reverse episodes. |

### Test-only route geometries

| Scenario family | Result | Route judgment |
| --- | --- | --- |
| Short waypoint before obstacle | Pass | Sensible; stops at the waypoint without chasing the obstacle beyond it. |
| Forward-only panel bypass | Pass in 8/8 | Best obstacle route in the set: about 2.45 m for a 2.15 m direct goal, no reverse, about 148° rotation. |
| Big-wall short goal | Pass clean/default | Long route is required by the single open wall end. |
| Custom route-left / angled rear-edge geometry | Pass clean/default | Sensible for the cluttered boundary geometry; no reverse. |
| Heading-back start-left neighbor | Pass in 8/8 | One seed completes only after reverse and reaches 9.2 mm actual body margin; not acceptable as robust. |
| Heading-back goal-right neighbor | Pass in 4/8 | Worst feasible-route instability: up to 13 reverse episodes; one passing run takes 5.924 m, 815° rotation, and reaches 11.9 mm margin. |
| Impossible arena-spanning wall | Expected failure | Clean fails in 6.32 s, but noisy runs wander 2.75–3.71 m with 6–8 reverse episodes and take 17.48–22.58 s to fail. |
| Sequential opposite-side obstacles | Pass in 8/8 | Sensible; 3.31–3.54 m for a 3.10 m direct goal. |
| Close sequential opposite-side obstacles | Pass in 8/8 | Sensible overall; 3.02–3.18 m for a 2.60 m goal. Side-state telemetry differs by preset and needs an oracle that distinguishes a legitimate merged route from stale ownership. |
| Dynamically appearing forward blocker | Pass clean/default | One evidence-driven reverse episode, then completion in 3.226 m. Appropriate for the test contract. |
| Dynamically blocked rear | Expected failure clean/default | Fails neutral in 2.24 s without publishing reverse. Sensible safety outcome. |
| Large rear blind region | Pass clean/default | Completes forward-only around the obstacle; rear unknown correctly blocks reverse without blocking proven forward travel. |

The six matrix-pickup tests exercise tracking, handoff, feed, and typed loss
behavior rather than local route selection. The forced emergency tests inject
internal recovery exhaustion and are state-machine/safety tests, not natural
path-choice evidence. The remaining sensor, geometry, mapping, kinematics, CSV,
and adapter tests have no route to grade.

## Why KISS wall — bypass left veers so far

The added lower/right boundary is joined into the same retained obstacle
envelope as the front wall. At context start the front wall near distance is
about `0.591 m`, and the first local goal is `(1.327, 1.454)`. Once the
perpendicular boundary joins the component, `nearAlongM` moves backward to
about `0.141 m`. `buildObstacleLocalGoal()` uses this expanded near value, so
the local goal jumps backward to `(1.084, 1.454)`.

The robot is already at about `x=1.10` when that happens. It therefore curves
left and slightly backward in route coordinates, reaches `x=1.094`, climbs to
`y=1.816`, and only then travels along the wall. The boundary is correctly used
to reject the lower/right side, but it should not drag the front-wall approach
anchor behind the robot. `approachNearAlongM` already stores the original front
face, but the active local-goal calculation does not use it for this stage.

## Why the diagonal and gap routes choose the wrong side

For a sparse initial obstacle envelope, `chooseObstacleSide()` can return the
side indicated by the outer fan before comparing the two geometric target
costs. At the common start `(1.0, 1.0)` in a 2.4 m-high arena, the left outer
ray naturally sees farther to the arena boundary than the right outer ray.

In `Diagonal clearance`, the initial ranges are approximately:

- right outer: `1035 mm`
- right inner: `2839 mm`
- left inner: `349 mm`
- left outer: `1497 mm`

The inner readings clearly show the obstacle on the left/upper side, but the
outer-range shortcut selects left because the arena is more open in that
direction. The retained envelope then grows to the real upper edge and the
robot follows it all the way to about `y=1.98`. The lower/right path only needs
a small deviation from `y=1.0`.

The same arena-range preference makes the nominal `wall_right` fixture choose
left and makes all five gap-ladder scenarios avoid above the front panel. The
gap widths therefore have no effect on the clean/default path.

## Ordered implementation plan

### 1. Close the newly exposed safety regression

- Promote `wall_blocked` seeds `20260621` and `20260622` as exact WASM
  regressions with `collision == false`, minimum ground-truth body clearance
  at or above the configured hard margin, neutral terminal output, and bounded
  failure time.
- Record planner-estimated and ground-truth clearance on the same timeline.
  Trace why sparse/noisy map evidence reports 29–63 mm while the body is nearly
  touching.
- Add the same ground-truth clearance oracle to every feasible and expected-
  failure route. A route that eventually fails still may not consume the hard
  margin on its way to failure.
- Do not optimize local-goal distance or relax collision checks until these
  cases are safe.

### 2. Repair scenario semantics before tuning the planner

- Give every scenario explicit metadata: intended topology, feasible side or
  sides, expected terminal result, reverse policy, maximum path/time/rotation,
  and minimum actual clearance.
- Remove the duplicate `g0_right` alias or label it explicitly as an alias.
- Make `KISS wall — bypass right` the true mirror of the forced-left fixture,
  including an upper/left blocker that makes only the lower/right end feasible.
- Rebuild G1–G5 from edge-to-edge gap dimensions. Block the alternative end so
  the robot must traverse the named gap, then assert the intended side and
  passage. The five paths should not be identical.
- Rename `Diagonal clearance` if its intended oracle is merely safe stopping;
  otherwise assert the shorter lower/right bypass and add its Y-mirror.
- Give `sensor_fault`, `stuck`, and impossible-wall scenarios typed bounded
  terminal oracles instead of accepting indefinite neutral activity.

### 3. Make side choice geometric before using raw range preference

- For both sides, compute a configuration-space target, forward reachability,
  swept corridor feasibility, and estimated detour cost even while the
  envelope is sparse.
- Select the sole feasible side first, then the lower-cost side when the cost
  difference is meaningful.
- Use outer-fan evidence only as a final tie-breaker after transforming it into
  comparable route-frame clearance. Do not let unequal distance to arena
  boundaries override a clearly nearer inner-ray obstacle or a much shorter
  geometric route.
- Keep the selected side latched for one obstacle context. Reconsider only when
  new evidence proves that side infeasible.

### 4. Separate the front face from joined blocker topology

- Preserve the initial front-face/approach anchor independently from later
  component growth.
- Use that anchor for the outward approach stage. A perpendicular side blocker
  may constrain side feasibility and lateral extent, but must not pull the
  approach target backward along the route.
- Verify the narrow scope first by changing only which near value owns the
  approach stage; do not simultaneously retune join distance, clearance, or
  scoring.
- Protect exact/mirrored KISS-left and retained `check-this`/`heading-back`
  fixtures before considering a more complex orientation-aware component
  model.

### 5. Replace the rigid outward-then-along detour with an early safe tangent

- Once side choice and envelope ownership are correct, evaluate a small set of
  footprint-safe corner/tangent targets between the current pose and the wall
  end.
- Switch to the along-wall/end target as soon as a swept path is proven, rather
  than requiring the centre to reach the full nominal lateral target first.
- Score total predicted path-to-final-goal and heading change, not only progress
  to the current staged point. Keep clearance as a hard gate plus soft
  preference, never as an efficiency trade.
- Avoid adding another recovery state. This should simplify the existing
  two-stage target behavior.

Initial efficiency acceptance targets:

- forced KISS left/right: path no more than `2.45 m`, accumulated rotation no
  more than `250°`, lateral excursion no more than `0.65 m`, no reverse;
- diagonal and mirror: path no more than `1.45 m`, rotation no more than
  `180°`, lateral excursion no more than `0.35 m`, no reverse;
- simple panel/t06e routes: no more than 15% above the conservative reference;
- full-height single-open-end walls: no more than 20% above reference and no
  reverse under ordinary sensing;
- every route: no contact and at least the configured 20 mm actual body margin.

These are simulator regression budgets, not physical calibration claims.

### 6. Close seed and metamorphic coverage gaps

- Run clean, website seed `20260620`, and a declared noisy set including all
  newly failing seeds on every natural route scenario—not only selected wall
  fixtures.
- Add Y-mirrors for diagonal, forced KISS, gap, sequential, and wall-end cases.
- Add ±40 mm start/goal shifts and ±5° headings around each side-decision
  boundary.
- Require zero unexpected reverse on feasible static routes. If reverse is
  explicitly permitted, bound episodes, distance, and rotation.
- Reject successful runs that exceed the hard clearance, rotation, path, or
  neutral-duty budgets.

### 7. Simplify and then optimize speed

- Consolidate scenario execution and route metrics into one maintained audit
  helper so the UI, fixtures, and tests cannot drift into different meanings.
- Delete misleading aliases and redundant fixtures only after their unique
  contract is either preserved or proven redundant.
- Measure speed after route geometry is stable. Shorter paths and fewer heading
  reversals should reduce time without increasing the 2600 ticks/s ceiling.
- Only then tune reachable-wheel scoring or acceleration behavior, one
  mechanism at a time, against the complete safety/efficiency matrix.

## Validation

After the audit and a fresh WASM rebuild:

- Teensy 4.0 compile: PASS (`167864` code bytes reported)
- firmware/WASM plus simulator suite: PASS, `75/75`
- Python tests: PASS, `143 passed`, `1 skipped`
- Python compile checks: PASS

The fact that the formal suite passes while the expanded audit finds contact,
seed failures, and unused scenario topology is itself a test-coverage finding.
Simulator results remain deterministic software evidence only.
