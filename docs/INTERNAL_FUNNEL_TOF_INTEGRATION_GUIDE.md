# Internal-funnel bottom-ToF payload-confirmation implementation plan

Status: software implemented on 2026-09-20 at the operator's request; physical
characterization and acceptance remain pending. No physical action was run.

This was one of the three future initiatives retained by `HANDOFF.md`. The
implemented matrix pickup flow still uses an assumed funnel handoff and the
same bounded feed, while the bottom ToF can now add payload evidence during
that feed. It does not create an autonomous collection/scoring backlog.

## Scope and non-goals

The configured bottom ToF mount is `(-38.8, 0, 25)` mm in the canonical
wheel-midpoint frame (`+X` forward, `+Y` robot-left, `+Z` up). These are a
candidate mount position, not a calibrated optical pose. The implementation
uses a short-mode VL53L1X on the primary I2C bus, XSHUT4, address `0x37`, with
zero yaw/pitch/roll. Its usable minimum range and funnel visibility remain
pending hardware characterization.

The bottom ToF is an **internal-funnel payload sensor only**. It must not:

- veto forward, reverse, turn, manual, route, or pickup motion;
- change `MotorControl.cpp`, motion authority, the motor lease/watchdog, or
  the final motor safety gate;
- contribute collision evidence, map occupancy/free space, localization, or
  planner candidate selection;
- replace the forward fan, front matrix, or rear ToF safety contracts;
- identify material, payload count, or a successful collection from a single
  reading.

Invalid, stale, saturated, contradictory, or absent bottom-ToF evidence means
`PAYLOAD_UNKNOWN`; it does not mean clear space, empty funnel, unsafe motion,
or confirmed capture.

## Desired observation and outcome contract

During an active capture attempt, the sensor observes the funnel at a calibrated
near range. The initial target is approximately 10 mm, only if the selected
sensor produces repeatable measurements at that distance. Hardware traces,
not this target value, define the final confirmation band and required sample
count.

The feature publishes immutable, sequence-numbered observations with an
acquisition timestamp, range/status, signal/ambient information when supplied
by the sensor, and a capture-attempt identifier. A confirmation counter may
advance once only for each newer sequence.

The minimum state model is:

```text
PAYLOAD_UNKNOWN
  -> CAPTURE_ENTRY_SEEN              (fresh near-range evidence)
  -> PAYLOAD_PRESENT_UNCLASSIFIED    (calibrated multi-sample signature)

Any state -> PAYLOAD_UNKNOWN         (timeout, stale/invalid/saturated or
                                       contradictory evidence)
```

`PAYLOAD_PRESENT_UNCLASSIFIED` confirms a physical object in the funnel for
the active capture attempt. It does not confirm material, mass, count, or that
the object remains retained after later motion. Repeating an observation or
re-entering the same state must not increment a payload count. A future unload
feature requires an independently observed empty transition before a later
capture can be counted.

## Integration sequence

```text
matrix-guided approach
  -> assumed funnel handoff
  -> existing bounded unconfirmed feed
  -> bottom-ToF capture window
       -> present-unclassified evidence, or
       -> payload-unknown on timeout
  -> existing route-resume decision
```

The current `WEIGHT_FUNNEL_HANDOFF_ASSUMED` event remains an assumption. The
bottom sensor may strengthen the outcome to payload present, but it must not
change the motion command, prolong the feed, or retry movement. The existing
route/pickup policy decides how to use `PAYLOAD_UNKNOWN`; no confirmation result
may conceal a navigation failure.

## Implementation phases

### 1. Hardware and geometry characterization

1. Select the sensor model and verify its documented and measured minimum
   reliable range. Do not assume it can read 10 mm.
2. Measure the optical origin and orientation after mounting; replace the
   candidate position only with measured values.
3. Verify that the funnel, chassis, cabling, and collected weight neither
   permanently occlude the empty reading nor produce ambiguous reflections.
4. Allocate a unique bus/address/XSHUT identity after checking the current
   four forward VL53L0X channels, three rear VL53L1X channels, and front
   SEN0628 matrix allocation. The reserved channel/address is only a candidate,
   not a pre-approved assignment.
5. Record stationary traces for empty funnel, seated valid weight, partial
   entry, bounce, double object, wall/contact interference, ambient extremes,
   and cross-talk.

Exit criterion: saved traces define an empty band, a present band (if
separable), a timeout, freshness limit, valid-status rules, and a conservative
multi-sample signature. If empty and present cannot be separated reliably, do
not enable confirmation.

### 2. Sensor module and public observation API

1. Add one focused sensor module under `src/sensors/`, following the existing
   nonblocking readiness-polling pattern; do not add delays to the control loop.
2. Put physical identity, calibrated geometry, valid range, sample period,
   stale limit, and confirmation parameters in `RobotConfig.h`.
3. Add a small typed immutable observation and read-only accessor in
   `Robot.h`/shared types. It contains status, range, sequence, timestamp,
   optional signal/ambient values, and no motion command.
4. Publish diagnostic/CSV fields or explicit events sufficient to reconstruct
   each capture attempt: attempt id, sequence, age, range/status, payload
   state, and terminal reason. Keep the existing frozen telemetry-schema rules.

Exit criterion: the module compiles; an unavailable sensor produces fresh
`PAYLOAD_UNKNOWN` evidence without changing any planner or motor output.

### 3. Capture-attempt state ownership

1. Add payload-observation state to the pickup runtime owned by navigation's
   existing pickup-tracking lifecycle, not by `MotorControl.cpp` or mission
   route code.
2. Allocate a new capture-attempt id when pickup tracking starts; clear all
   candidate evidence on cancellation, route handoff, target loss, and new
   attempt.
3. Open the sensor's confirmation window only at the existing assumed-handoff
   / unconfirmed-feed phase. Consume only newer observations belonging to the
   same attempt.
4. Transition to present only after the characterized signature; transition to
   unknown on feed completion without confirmation, timeout, fault, or
   contradiction.
5. Keep the present/unknown result idempotent and separate from future payload
   counting or material classification.

Exit criterion: the current pickup command, speed, duration, safety checks,
and route-resume behavior are identical whether the bottom sensor is present,
absent, valid, or faulted; only the payload-evidence result differs.

### 4. Firmware-backed simulator coverage

1. Extend the firmware bridge and simulator with a dedicated internal-funnel
   range source at the measured pose. It must be independent of the navigation
   fan and collision rays.
2. Add deterministic fixtures for empty funnel, seated weight, partial/bounce,
   invalid/stale sensor, stuck feed, and two sequential capture attempts.
3. Assert that present requires fresh multi-sample evidence, faults resolve to
   unknown, sequences are not double-counted, and motor/planner commands are
   unchanged by this sensor.
4. Keep existing pickup, wall-route, reverse-safety, and fault tests. Repair
   the separately failing matrix-pickup geometry tests before treating a new
   funnel-confirmation result as acceptance evidence.

Exit criterion: every new case has an explicit payload-state oracle and the
simulator/source hash matches the rebuilt WASM artifact.

### 5. Validation and controlled physical evidence

1. Run Arduino compile, firmware/WASM simulator tests, Python tests, and
   compileall. A toolchain `SKIP` is a blocker, not a pass.
2. With separate explicit physical-test authorization, begin stationary and
   log every capture attempt. Only then run controlled pickup feeds from a safe
   `END_MATCH`, disarmed, neutral-output status.
3. Compare logged attempt ids, ranges, observation ages, state transitions,
   and final payload evidence against the saved characterization data.
4. Do not claim payload-confirmation reliability from simulator results alone.

## Acceptance criteria

- Bottom-ToF data never changes a motion command, motor output, map cell,
  collision result, or navigation terminal reason.
- A valid seated object produces `PAYLOAD_PRESENT_UNCLASSIFIED` only after the
  calibrated multi-sample signature for the active attempt.
- Empty, stale, invalid, saturated, partial, bouncing, contradictory, and
  timeout cases end in `PAYLOAD_UNKNOWN`, with no inferred empty/present state.
- Duplicate frames and a prior attempt's late frame cannot create confirmation.
- All current safety contracts remain intact, and tests prove the new sensor
  cannot bypass them.

## Software verification (2026-09-21)

- Teensy 4.0 compile passed at 169,784 bytes FLASH code.
- Python contracts passed: 151 passed, 2 skipped; `compileall` passed.
- All 8 focused firmware-backed pickup/payload simulator scenarios passed.
  This includes unavailable, stale/contradictory, safety-vetoed feed, route
  resume, and an exact seeded-trace check that payload faults do not perturb an
  ordinary navigation goal.
- WASM parity passed at source hash
  `47ae3c8ea9c1188dd5ad72debf0d3454219590250d198f8c15f74365c079ab26`.
- The full current-worktree simulator suite is not green: 64 passed and 13
  general-planner scenarios failed. None of those failures exercises the
  pickup/payload path. They remain separate navigation work and are not hidden
  by the focused acceptance result.
- No upload, serial connection, arming, or physical motion test was performed.
