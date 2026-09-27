# Path Efficiency Review

> **Completed implementation/evidence record.** “Remaining opportunities” and
> other suggestions below are context only, not retained future work. Consult
> them only for a related user request.

## Scope

This review covers deterministic firmware/WASM navigation behavior for the
downloaded `check-this.json` wall-end scenario, its mirror and reduced control,
and the built-in G1-G5 gap ladder. It does not establish physical readiness and
does not replace stationary or motion characterization.

## Why the downloaded route looked inefficient

The large visible detour was primarily a wrong-side lifecycle problem, not an
excess collision margin. The goal was west of the robot while the chassis faced
east. A sparse, sensor-built map was evaluated before the fan observations were
meaningful in the route frame. Body-left/body-right range differences were then
treated as route-left/route-right evidence. The first side was infeasible, so
the planner travelled away from the useful wall end, switched sides, and made a
large recovery turn.

The wall itself reaches approximately `y = 1.947 m`. With the declared body,
hard clearance and map discretisation, the robot centre genuinely needs to
reach roughly `y = 2.10 m` to pass its upper end. The remaining lateral detour
is therefore mostly geometric; the removed waste was the initial wrong-side
excursion and switch.

## Final changes and review

### Route-frame outer-fan side evidence

When an obstacle envelope is still sparse, the planner now projects the two
outer fan rays into the route frame. It uses them only when the chassis is
within the configured comparable-heading band or clearly anti-aligned and the
ranges differ by more than the existing side-score tie margin.

Review: this fixes the body-frame/world-frame mix-up without changing collision
acceptance. Perpendicular observations remain unresolved. Diagonal routes retain
the existing map/cost behavior because their two raw ranges are not directly
comparable.

### Delayed side reconsideration

A selected side is no longer reconsidered while the chassis still faces away
from the route half-plane. Sparse map growth at that stage is orientation-biased
and previously caused an immediate correct-side-to-wrong-side flip.

Review: the existing one-time reconsideration remains available after evidence
becomes route-relevant. No collision check, safety veto, or recovery gate was
removed.

### Aligned inner-range tie handling

If aligned inner ranges differ by less than the configured 80 mm tie margin,
the planner uses the wider-baseline outer pair after route-frame projection.
This prevents 2-3 mm sensor noise from choosing the blocked side of a gap.

Review: this is deterministic under clean and website-default sensing and uses
an existing noise/tie threshold rather than a fixture-specific distance.

### No-path wall-stage transition

A noisy endpoint can extend the retained wall envelope by one 50 mm map cell.
The normal outward stage remains unchanged. If that stage proves that no
geometric trajectory exists while the robot is within the final half-cell of
the normal transition band, the obstacle context now commits once to the
along-wall stage. A later side switch clears that commitment and starts the new
side normally.

Review: this is a fallback after a complete no-path result, not a global
clearance relaxation. The original alignment fallback retains priority, and
the along-wall command must still pass the unchanged sensor, turn-sweep and
complete swept-footprint checks.

### Regression fixtures and metrics

The exact downloaded scenario, a reduced continuous-wall control, a tight
single-wall stage-transition control, their mirrors, and G1-G5 under clean and
website-default sensing are now firmware/WASM tests. The tight wall-end cases
also run the deterministic sensor seeds that previously entered repeated
recovery.
The harness records completion, collision, reverse use, side transitions, body
clearance, path length, accumulated rotation, route divergence, lateral
excursion and elapsed time. Detailed transition diagnostics are opt-in through
`POINT_SCENARIO_DIAGNOSTICS=1`.

Review: these are behavior contracts rather than exact trajectory snapshots, so
safe future improvements remain possible while regressions to switching,
backtracking or unbounded detours fail clearly.

## Measured result

| Scenario | Before | After |
|---|---:|---:|
| Exact, clean: time | 25.80 s | 13.82 s |
| Exact, clean: path | 4.535 m | 3.277 m |
| Exact, clean: rotation | 577 deg | 302 deg |
| Exact, website: time | 21.24 s | 13.42 s |
| Exact, website: path | 4.293 m | 3.224 m |
| Exact, website: rotation | 542 deg | 308 deg |
| Exact side lifecycle | wrong side, switch, reverse | one correct side, no reverse |
| Exact noisy seeds | 19/40 complete | 40/40 complete, no reverse |
| Exact/reduced/mirror noisy stress | not covered | 120/120 complete, no reverse |
| G1-G5 website completion | 1/5 | 5/5 |
| G1-G5 after change | inconsistent | 7.38 s, 1.733 m, 268 deg each |

All final wall-end and gap runs are collision-free. Across the final 40-seed
exact wall-end sweep, minimum body clearance was approximately 54 mm;
gap-ladder minimum body clearance is approximately 56 mm.

## Rejected experiments

- Aligning every point goal before obstacle-context creation improved this one
  route but regressed ten retained heading and boundary scenarios.
- Delaying every side choice until route alignment caused stop/backtrack loops
  because an already-aligned robot could not gain different evidence.
- Aggressively choosing the innermost collision-free bypass band destabilized
  most retained wall scenarios.
- Shortening the obstacle stage-transition tolerance did not solve the tight
  continuous-wall failure and regressed retained cases.
- Truncating all intermediate-goal rollouts after arrival was too broad and
  regressed retained paths.

None of these rejected experiments remains in production code.

## Remaining opportunities

1. Replace range-based final tie-breaking with explicit route-frame candidate
   paths once the local map can prove both obstacle ends and boundary widths.
   This could let G1 choose its shorter feasible side without making G2-G5
   noise-sensitive.
2. Reduce the roughly 80-150 mm arc overshoot beyond nominal wall-end bypass
   lines using a verified stage-arrival controller. Preserve the current hard
   clearance and minimum-drivable-speed contracts.
3. Add randomized mirrored side-boundary fields beyond the current wall-end
   seed sweep to keep checking generalization outside the fixed gap ladder.

## Validation identity

Final deterministic validation on 2026-08-09:

- Teensy 4.0 firmware compile: passed.
- Fresh firmware WASM build: passed.
- Firmware/WASM tests: 62 passed.
- Legacy simulator tests: 13 passed.
- Python tests: 143 passed, 1 skipped.
- Python compile checks: passed.
- Periodic motor-output ownership: `MotorControl.cpp` only.
