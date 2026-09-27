# TOF-Dominant Wall-Anchored Localization

Status: retained future initiative; not implemented and not authorized without
an explicit user request.

This document preserves the final decision from the 2026-07-20 localization
planning session while updating its baseline to the current sensor
architecture. It supersedes the earlier encoder-primary SLAM-ish draft from
the same session. The IMU/encoder work in
`IMU_AIDED_POSITION_TRACKING_PLAN.md` is its prediction layer, not a competing
localization system.

## Retained objective

Use known arena boundary walls as the strongest source of observable absolute
X/Y correction:

1. Wheel encoders predict short-term translation.
2. BNO055 supplies or corrects heading through a health-checked estimator.
3. Geometrically consistent TOF-to-boundary observations correct the position
   components they can observe.
4. Internal obstacles contribute mapping and collision evidence but never move
   the estimated pose.
5. Optical flow, if revisited, remains an optional disagreement checker and
   never becomes the position authority.

This is known-wall localization plus obstacle mapping, not unrestricted SLAM
or loop closure.

## Current baseline

The active robot no longer has the side/rear layout assumed by the original
July plan:

- four forward VL53L0X fan rays at `-60`, `-20`, `+20`, and `+60` degrees;
- one front SEN0628 8x8 matrix with a nominal 60-degree field of view;
- three rear VL53L1X sensors at 180 degrees, separated laterally but pointing
  parallel to each other;
- wheel-encoder translation and BNO055 heading in `Odometry.cpp`;
- a rolling confidence map and persistent arena evidence;
- no TOF-to-wall pose correction, covariance estimator, wall association,
  localization mode, or absolute arena transform.

The current sensors provide rich forward and rear range evidence but no
dedicated ±90-degree side rays. Parallel rear channels add coverage and wall
orientation evidence, but they do not automatically provide the independent
wall-normal observations assumed by the July design.

## Observability boundary

- A measurement to one wall corrects only position perpendicular to that wall.
  Along-wall position must continue on encoder prediction until a nonparallel
  boundary is observed.
- The front fan and matrix can correct against walls visible ahead; the rear
  array can correct against walls visible behind.
- At headings where only one wall orientation is visible, the estimator must
  report partial observability rather than pretend both X and Y are locked.
- Internal panels, weights, ramps, robots, and moving obstacles must be rejected
  as localization anchors even when they are valid mapping/safety evidence.
- The implementation planning session must prove whether the existing fan,
  matrix columns, and parallel rear rays provide sufficient recurring
  nonparallel observations. If not, it must choose between constrained
  localization behavior and a deliberate hardware change. The obsolete July
  proposal for new ±90-degree sensors must not be copied silently.

## Retained estimator contract

- Keep separate predicted and corrected poses with timestamps and uncertainty
  by axis.
- Predict translation from separately calibrated wheel distances using the
  health-checked heading supplied by the IMU/encoder estimator layer.
- Associate a fresh TOF observation only with an immutable known arena boundary
  whose predicted range, incidence, residual, status, freshness, and repeated
  sequence evidence pass explicit gates.
- Apply accepted corrections along the associated wall normal. Do not change
  heading from a single TOF range in the initial milestone.
- Reject ambiguous or large residuals; never snap the pose to a convenient
  wall.
- Use explicit modes such as `LOCKED`, `PARTIAL`, `DEAD_RECKONING`, and `LOST`
  based on measured uncertainty and recent accepted corrections.
- Position-dependent autonomy must fail or degrade predictably when the
  estimator is `LOST`; manual and diagnostic motion remain governed by the
  existing direction-aware safety system.
- Map each sensor observation once using its sequence, timestamp, sensor mount,
  and pose at acquisition. Pose correction and obstacle mapping must not reuse
  stale frames as new evidence.

## Arena and initialization decisions retained from July

- The arena boundary dimensions are fixed and trusted when this mode is used.
- The robot starts in a declared left-side corner facing along the arena's long
  axis.
- Corner selection is explicit rather than inferred from one ambiguous range.
- Boot odometry may remain start-relative, but wall anchors require a clearly
  defined arena-to-odometry transform.
- Startup must not claim a wall lock until the required current observations
  are stable and geometrically consistent.

The exact corner command, dimensions, range gates, correction noise, residual
limits, uncertainty thresholds, and enable policy must be revalidated against
the current sensor mounts and current firmware before implementation. Values in
the July chat are design history, not current calibration constants.

## Required implementation-planning evidence

Before code changes, a new planning session must:

1. inspect current sensor geometry and field-of-view transforms from
   `RobotConfig.h`;
2. define which matrix cells or aggregated rays may participate in wall
   association without weakening their existing safety/perception consumers;
3. simulate arena-wall visibility over position and heading to quantify X/Y
   observability with the current mounts;
4. define the arena/odometry transform and startup contract;
5. reconcile estimator ownership with
   `IMU_AIDED_POSITION_TRACKING_PLAN.md` and current `Odometry.cpp`;
6. specify telemetry, replay, covariance, rejection, and typed degradation
   behavior;
7. begin in shadow mode and preserve current navigation authority until replay
   and deterministic simulation prove the estimator behavior.

No firmware upload, serial action, arming, or physical motion is implied by
retaining this plan.

