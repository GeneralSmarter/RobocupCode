# IMU-Aided Position Tracking Implementation Plan

Status: research and implementation plan only  
Date: 2026-07-29  
Scope: `RobotCode`, estimator-specific simulator support, telemetry, and tests  
Physical status: no upload, serial connection, arming, or movement was performed

## Decision

Do not implement position by double-integrating the BNO055 accelerometer.
Implement **planar IMU-aided wheel odometry**:

- wheel encoders remain the primary translation measurement;
- encoder difference predicts heading change;
- BNO055 yaw and gyro constrain heading and detect wheel slip or magnetic
  disturbances;
- BNO055 linear acceleration is used for stationary, bump, and consistency
  evidence, not as the primary source of X/Y displacement;
- an independent absolute observation is required to bound long-term X/Y drift.

This is already close to the firmware's present architecture. The current
odometry uses encoder centre distance for translation and replaces heading with
the latest BNO055 Euler yaw. The proposed estimator makes the sampling healthy,
uses both available heading sources, integrates a proper `SE(2)` motion
increment, reports uncertainty, and rejects bad IMU observations.

## Why pure IMU position is rejected

The BNO055 can publish gravity-separated linear acceleration, but Bosch states
that this signal normally cannot be integrated into useful velocity or
double-integrated into position: without another correcting sensor, integration
error can become larger than the signal in less than one second. The datasheet
also says the internal fusion was primarily designed for human motion and can
misinterpret sustained vehicle acceleration as gravity.

The practical consequence is:

1. a constant acceleration bias creates velocity error proportional to time;
2. the resulting position error grows approximately with time squared;
3. small roll/pitch or gravity-removal errors are much larger than the robot's
   useful horizontal accelerations;
4. zero-velocity updates help while stationary but do not make a single
   chassis-mounted consumer IMU an absolute position sensor.

The BNO055 is also marked "not recommended for new designs" by Bosch. It is
reasonable to keep using the installed part for this robot, but the estimator
API must not depend on BNO055-specific data structures so a later IMU can
replace it.

Primary references:

- [Bosch BNO055 datasheet](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bno055-ds000.pdf),
  especially fusion-mode limitations in sections 3.3.3 and 3.6.5.6.
- [Bosch BNO055 product status](https://www.bosch-sensortec.com/en/products/smart-sensor-systems/bno055).
- [Adafruit BNO055 vector outputs](https://learn.adafruit.com/adafruit-bno055-absolute-orientation-sensor?view=all),
  including gyro, linear acceleration, gravity, Euler, and quaternion outputs.
- [Adafruit BNO055 calibration guidance](https://learn.adafruit.com/adafruit-bno055-absolute-orientation-sensor/device-calibration).
- [WPILib differential-drive pose estimator](https://github.wpilib.org/allwpilib/docs/release/cpp/classfrc_1_1_differential_drive_pose_estimator.html),
  a reference architecture using encoders and gyro for odometry, with
  independent pose observations for drift correction.
- [Neto, Pires, and Moreira, 3-D position estimation from inertial sensing](https://arxiv.org/abs/1311.4572),
  experimental evidence that low-cost inertial position from double
  integration is useful only for short intervals without external constraints.

## Current repository findings

Research was performed against nested `RobotCode` commit
`de301b55c7a0661362401a68af56c10a7032d4aa`. The worktree already contains
uncommitted planner/config changes, which this plan does not modify. The current
WASM manifest reports source hash
`c732a13945b48a72b196e9bbd4b40228907cdf24697548f47ab6e768f08676c7`.

### Current behavior

- `Imu.cpp` calls `bno.begin()` in an unbounded boot loop, selects the library's
  default NDOF mode, enables the external crystal, and reads only Euler yaw.
- `zeroYaw()` and `readImuClockwiseYawDeg()` do not check read success,
  finiteness, calibration, fusion status, age, or discontinuity.
- `navigationHeadingDeg()` performs a fresh I2C read on every call.
- `Odometry.cpp` reads the IMU once per 20 ms odometry update, assigns that yaw
  directly to `robotTheta`, averages left/right encoder distance, and projects
  the distance at the final heading.
- Encoder-derived heading change is not used by odometry, despite
  `EFFECTIVE_TRACK_WIDTH_M` being available.
- The single distance scale, `TICKS_PER_METRE = 9125`, is still provisional.
  Existing documentation already calls for separate left/right calibration.
- The installed Adafruit BNO055 library is version `1.6.4`, but
  `platformio.ini` does not pin the dependency version.
- The frozen telemetry contains raw and navigation yaw, but not IMU
  calibration, status, sample age, gyro, acceleration, estimator innovation,
  rejected observations, uncertainty, or encoder totals.
- The WASM simulator gives the firmware planner an ideal pose through
  `firmware_sim_set_pose()`. It does not exercise `Imu.cpp`, `Odometry.cpp`,
  encoder error, IMU noise, IMU dropout, or estimator drift.

### Frame contract to preserve

The existing P0-01 contract remains authoritative:

- body and navigation `+X` is forward;
- `+Y` is robot-left;
- `+yaw`, `+heading`, and `+turn` are CCW/left;
- forward wheel distance is positive;
- `left = forward - turn`;
- `right = forward + turn`;
- encoder yaw is `(right - left) / trackWidth`;
- installed BNO055 relative yaw is currently CW/right-positive and is negated
  exactly once.

The implementation must add tests before changing any sensor axis remap,
mounting transform, yaw mode, or sign.

## Required coordinate frames

Use three explicit frames:

1. `body`: fixed to the wheel midpoint, `+X` forward, `+Y` left, `+Z` up.
2. `odom`: continuous local frame created by `ZERO` or boot-time initialization.
   Encoder/IMU dead reckoning lives here and may drift.
3. `arena`: optional field-fixed frame. It must not exist merely because the
   robot has an `odom` pose. Arena-to-odom needs a measured `SE(2)` transform
   and uncertainty.

`robotX`, `robotY`, and `robotTheta` currently mean an `odom` pose. Keep field
GOTO disabled until the separate arena transform, preview, bounds/status
checks, and explicit confirmation required by P0-08 exist.

## Target architecture

```text
Encoders ISR ──> signed cumulative wheel counts ──┐
                                                  │
BNO055 ──> one timestamped cached ImuSample ─────┼─> PoseEstimator2D
                                                  │      │
ZERO/base/landmark correction ───────────────────┘      ├─> odom pose
                                                         ├─> health/uncertainty
                                                         └─> pose history

odom pose + timestamped ToF frame ──> mapping/planning
```

Ownership:

- `Imu.cpp`: initialize, sample, validate, timestamp, and cache BNO055 data.
- `Odometry.cpp`: own `PoseEstimator2D` prediction/correction and pose history.
- `RobotConfig.h`: measured calibration values and explicit estimator limits.
- `MotorControl.cpp`: remain the only periodic motor-output owner.
- `Bluetooth.cpp`: diagnostics and telemetry only; no estimator side effects
  except an explicit stopped/disarmed `ZERO`.
- `LocalPlanner.cpp`: consume the published pose and health; do not contain
  estimator math.

## Estimator design

### 1. Timestamped IMU sample

Add a device-neutral cached record, conceptually:

```cpp
struct ImuSample {
  uint32_t sequence;
  uint32_t capturedUs;
  float relativeHeadingRad;
  float gyroZRadPerSec;
  float linearAccelXMps2;
  float linearAccelYMps2;
  float linearAccelZMps2;
  uint8_t systemCalibration;
  uint8_t gyroCalibration;
  uint8_t accelCalibration;
  uint8_t magCalibration;
  uint8_t systemStatus;
  uint8_t selfTest;
  uint8_t systemError;
  bool finite;
  bool valid;
};
```

`updateImu()` runs once per scheduled IMU period. All consumers read the cached
sample; they must not start another I2C transaction. Read calibration at a
slower diagnostic cadence and system status at initialization, periodically,
and after a fault.

Before implementation, physically document the BNO055 board orientation. Freeze
one explicit sensor-to-body axis permutation/sign transform and prove it with
stationary six-face observations and signed turns. Do not infer accelerometer
axes from the currently known yaw sign.

### 2. Health and initialization

Replace the unbounded startup retry with:

- a finite attempt/time budget;
- a safe IMU fault state that cannot arm navigation;
- recorded operation mode, system status, self-test, error code, calibration,
  sample age, and consecutive failure count;
- finite/range checks and a maximum plausible yaw step/rate;
- a freshness threshold less than `PLANNER_COMMAND_MAX_AGE_MS`, enforced before
  a stale pose can keep a prior motion command alive.

Do not require full magnetometer calibration merely to remain stationary.
Define separate health levels:

- `UNAVAILABLE`: no current finite sample; navigation is neutral.
- `RELATIVE_OK`: gyro and relative heading are usable; position tracking may
  continue with growing uncertainty.
- `ABSOLUTE_YAW_OK`: NDOF heading/calibration and innovation checks pass.

NDOF versus IMUPLUS is a measured decision. Start by logging NDOF behavior.
Switch to magnetometer-free IMUPLUS only if motor/arena magnetic disturbance is
shown to create heading jumps that innovation gating cannot safely use.

### 3. Wheel prediction

Calibrate separate left/right scales before tuning the filter:

```text
dl = deltaLeftTicks  / LEFT_TICKS_PER_METRE
dr = deltaRightTicks / RIGHT_TICKS_PER_METRE
ds = (dl + dr) / 2
dtheta_encoder = (dr - dl) / EFFECTIVE_TRACK_WIDTH_M
```

Track width must be fitted from repeated measured arcs after left/right distance
scales are fixed. Do not use IMU heading to fit both wheel scale and track width
from the same run without an independent distance/angle reference.

### 4. Heading correction

Start with a scalar heading filter, not a large general EKF:

```text
theta_prediction = theta_previous + dtheta_encoder
P_prediction     = P_previous + Q_encoder
innovation       = wrap(theta_imu - theta_prediction)
K                = P_prediction / (P_prediction + R_imu)
theta_corrected  = theta_prediction + K * innovation
P_corrected      = (1 - K) * P_prediction
```

Accept an IMU yaw correction only when the sample is current and the
innovation agrees with gyro and wheel-rate evidence. In particular:

- compare `gyroZ` with `dtheta_encoder / dt`;
- if encoder and gyro agree but NDOF yaw jumps, reject the yaw observation as a
  likely magnetic/fusion discontinuity;
- if gyro and encoder disagree during commanded motion, raise slip/kinematic
  uncertainty instead of silently choosing one;
- if IMU is temporarily rejected, continue encoder prediction for a short,
  bounded interval with increasing uncertainty, then stop navigation.

Derive `Q_encoder`, `R_imu`, and innovation thresholds from logs. Do not begin
with hand-tuned confidence constants hidden in `Odometry.cpp`.

### 5. `SE(2)` position integration

Integrate translation over the fused heading increment, rather than projecting
all distance at the final yaw:

```text
dtheta = wrap(theta_corrected - theta_previous)
travel = ds * sinc(dtheta / 2)
theta_mid = theta_previous + dtheta / 2
x += travel * cos(theta_mid)
y += travel * sin(theta_mid)
theta = theta_corrected
```

Use a stable small-angle `sinc` branch. This is exact for a constant-curvature
differential-drive increment and preserves the canonical left-positive frame.

### 6. Accelerometer role

Read and log BNO055 linear acceleration, but initially use it only for:

- stationary confidence together with neutral command, near-zero wheel rates,
  and low gyro rate;
- bump/ramp detection that increases estimator uncertainty;
- longitudinal acceleration consistency against change in encoder velocity;
- possible wheel-slip evidence;
- characterizing vibration and gravity-removal quality.

Do not update X/Y directly from integrated acceleration in the production
estimator. A short-lived acceleration-integrated diagnostic may be added to
telemetry for research, but it must never become planner or safety evidence.

### 7. Absolute correction interface

Encoder/IMU odometry will still drift. Define one gated API for timestamped
absolute or partial corrections:

```text
applyPoseObservation(source, timestamp, observed components, covariance)
```

Candidate observations, in priority order:

1. a known pose at `ZERO` or confirmed base/dock alignment;
2. a measured arena-wall constraint only after wall identity and the
   arena-to-odom transform are proved;
3. a camera/AprilTag or other external field-relative pose if hardware is
   added.

Do not correct pose from the planner's own occupancy map without an independent
scan-matching model and gating; that would feed odometry errors back into the
same evidence that was built from them.

## Implementation phases

### Phase 0 — requirements and stationary evidence

Deliverables:

- define the required frame (`odom` versus arena) and mission accuracy budget;
- pin the BNO055 dependency version used for validation;
- add a no-motion `TEST IMU`/status diagnostic path;
- record sample rate, latency, calibration, status, gyro, Euler/quaternion,
  linear acceleration, temperature if useful, and axis mapping;
- measure stationary drift cold and warm and magnetic effects with drive power
  enabled but motors neutral.

Exit gate:

- sensor-to-body axes and all signs are proven;
- initialization and runtime failure cases are known;
- no motion behavior has changed.

### Phase 1 — coherent IMU service and fail-closed health

Deliverables:

- cached `ImuSample`;
- bounded initialization and explicit IMU fault status;
- cached `navigationHeadingDeg()` with no consumer-side I2C reads;
- freshness/calibration/status telemetry;
- final-writer safety integration for an unavailable/stale estimator;
- reset semantics that align yaw, pose, wheel snapshots, filter covariance, and
  map reset atomically while already neutral/disarmed.

Exit gate:

- stale, non-finite, failed-self-test, and yaw-jump tests all stop or degrade
  exactly as specified;
- IMU sampling does not cause loop-deadline misses or threaten the 150 ms motor
  lease;
- existing P0 turn-sign tests still pass.

### Phase 2 — pure estimator in shadow mode

Deliverables:

- platform-neutral `PoseEstimator2D` core;
- separate left/right wheel scales;
- encoder heading prediction, scalar IMU correction, exact `SE(2)` integration;
- shadow pose, innovation, uncertainty, observation-rejection reason, and
  slip-consistency telemetry;
- CSV replay tool that runs old and new estimators on identical sensor records.

The existing pose remains authoritative during this phase. Shadow mode must not
affect planning, mapping, motor safety, or completion.

Exit gate:

- ideal traces are exact;
- synthetic bias, scale error, wrap crossing, dropout, and magnetic-jump cases
  remain bounded and typed;
- physical calibration logs show an improvement over the current estimator;
- CPU/RAM and telemetry queue budgets pass.

### Phase 3 — simulator estimator fidelity

Deliverables:

- keep `truePose` private to physics/collision/raycasting;
- model cumulative encoder readings separately from truth;
- inject IMU yaw/gyro/acceleration samples, timestamp, calibration, noise,
  bias, jump, and dropout into the estimator;
- stop giving the WASM planner ideal pose through `firmware_sim_set_pose()`;
- expose estimated pose and truth error separately;
- preserve the old deterministic planner scenarios as immutable regressions.

Add estimator scenario families:

- ideal straight, in-place turn, constant-radius arc, square, and out-and-back;
- left/right encoder scale mismatch;
- wrong track width;
- gyro bias;
- NDOF heading jump with consistent gyro/encoders;
- IMU dropout and stale sample;
- wheel slip;
- ramp/bump acceleration;
- irregular `dt` and loop-delay injection;
- mirror and heading-wrap metamorphic cases.

Exit gate:

- estimator faults never weaken collision checks, freshness gates, authority,
  watchdog, or motor lease;
- planner regressions remain behaviorally stable under ideal sensors;
- fault scenarios fail neutrally or continue only for their documented bounded
  interval.

### Phase 4 — controlled authority switch

Switch the estimator only at a `ZERO`/initialization boundary while neutral and
disarmed. Do not hot-swap pose sources during motion.

After acceptance:

- publish the new estimator through the existing `robotX/Y/Theta` interface;
- delete the old integration path rather than keeping two permanent policies;
- retain shadow/replay tooling outside the production motion path;
- stamp build identity, estimator config hash, and calibration identity into
  every regression log.

### Phase 5 — bounded absolute correction

Implement only after an actual correction source is selected and measured.
Start with confirmed base/dock pose reset if the mission provides a repeatable
mechanical alignment. Add delayed observation replay if observations arrive
with meaningful latency.

This phase is what converts locally continuous `odom` into bounded arena
position. IMU/encoder fusion alone does not.

## Telemetry additions

Version the schema rather than silently extending frozen schema v3. Include:

- IMU sequence and age;
- operation/health status and failure reason;
- system/gyro/accel/mag calibration;
- self-test and system error;
- relative yaw, unwrapped yaw, gyro Z, and body linear acceleration;
- cumulative left/right encoder counts;
- left/right calibrated distance increment;
- encoder yaw increment, IMU innovation, accepted/rejected flag and reason;
- pose covariance or compact standard deviations;
- slip/bump/stationary flags;
- estimator source/config version;
- estimated pose and, in simulation only, truth pose/error.

Keep rows bounded and nonblocking. Measure the added serialization and queue
cost before accepting the schema.

## Verification matrix

### Source and unit tests

- sensor-axis permutation/sign and P0-01 wheel/yaw invariants;
- degrees/radians and ticks/metres conversions;
- heading unwrap across `+180/-180`;
- exact straight, turn, and arc integration;
- zero distance with changing yaw and zero yaw with distance;
- asynchronous timestamps and repeated/out-of-order samples;
- `ZERO` atomic reset;
- stale/non-finite/calibration/status truth tables;
- yaw-jump rejection using gyro/encoder agreement;
- covariance remains finite, symmetric where applicable, and nonnegative;
- no acceleration-derived X/Y update in the production estimator.

### Replay and simulation

- replay every suitable physical straight/turn/avoidance log;
- compare current and shadow estimator using independent measured endpoint
  evidence, not one estimator as ground truth;
- run canonical clean and website-default simulator conditions;
- add exact estimator fault fixtures and mirrored/metamorphic neighbors;
- report firmware/WASM tests separately from legacy JavaScript planner tests.

### Proposed initial physical acceptance gates

These are starting gates to be confirmed against the mission accuracy budget,
not claims about present hardware:

- stationary, 120 s: X/Y unchanged; no invalid yaw step; healthy relative
  heading drift at most `1 deg`;
- repeated measured 1 m straight: median longitudinal error at most `2%`,
  lateral endpoint error at most `30 mm`, heading error at most `2 deg`;
- repeated measured `+90/-90 deg` turns: final heading error at most `2 deg`
  and translation at most `30 mm`;
- measured constant-radius left/right arcs: no sign asymmetry and pose endpoint
  error at most `50 mm`/`3 deg`;
- 1 m square: closure error at most `100 mm`, final heading error at most
  `3 deg`;
- two-minute representative route: new estimator must reduce independently
  measured endpoint error by at least `30%` relative to the current estimator,
  with no safety or planner regression;
- worst IMU/estimator phase remains below a measured budget that preserves the
  `60 ms` loop deadline, `120 ms` planner-command age guard, and `150 ms` motor
  lease with margin.

Every physical test requires a new explicit authorization and the repository's
safe `STATUS`/setup/cleanup gates. Simulator results are deterministic software
evidence only.

## File-level implementation map

Expected changes, one phase at a time:

- `RobotTypes.h`: device-neutral IMU, estimator health, pose, and observation
  records.
- `RobotConfig.h`: sample/freshness bounds, measured wheel scales, track width,
  noise values, and innovation gates.
- `Globals.cpp` / `Robot.h`: owned cached sample and estimator declarations.
- `Imu.cpp`: bounded initialization, cached sampling, axis transform, status,
  calibration, and health.
- `Odometry.cpp`: pure estimator update and pose publication.
- `Helpers.cpp`: cached navigation heading only; no direct I2C read.
- `MotorControl.cpp`: final fail-closed check for estimator health when motion
  depends on it.
- `Bluetooth.cpp`: versioned diagnostic/status/CSV output and no-motion IMU
  diagnostic.
- `RobotCode.ino` / controller schedule: explicit IMU sample before estimator
  update without adding another motor writer.
- `RobocupSimulator/firmware-wasm/firmware_bridge.cpp`: sensor/encoder injection,
  not ideal pose injection.
- `RobocupSimulator/simulator-core.js`: truth/sensor/estimate separation and
  deterministic estimator faults.
- `SerialCommandUI`: schema parsing, replay, plots, and contract tests.

## Risks and mitigations

| Risk | Required mitigation |
| --- | --- |
| NDOF magnetic heading changes near motors/metal | Log gyro, encoder yaw, calibration, and yaw innovation; reject inconsistent absolute yaw; evaluate IMUPLUS from evidence. |
| Wrong accelerometer axes | Freeze and test an explicit sensor-to-body transform before using acceleration for any decision. |
| Uncalibrated wheel distance/track width | Calibrate left/right scale first, track width second, estimator noise last. |
| More I2C traffic causes deadline/lease failures | Cache one scheduled sample, read slow diagnostics less often, instrument phase time, and reject the change if timing margin is lost. |
| Filter hides a hardware fault | Publish raw measurements, innovations, rejection reasons, and uncertainty; fail closed on stale/unavailable state. |
| Simulator gives false confidence | Separate truth from estimate and inject the same timestamped sensor interfaces used by firmware. |
| Permanent dual estimator complexity | Use shadow mode only for rollout, then remove the displaced production integration path. |
| Odometry mistaken for arena position | Keep `odom` and `arena` frames explicit; do not re-enable field GOTO as part of this work. |

## Recommended execution order

1. Phase 0 stationary diagnostics and encoder calibration.
2. Phase 1 cached/healthy IMU service.
3. Phase 2 shadow `PoseEstimator2D` and replay.
4. Phase 3 truth-separated simulator and fault matrix.
5. Phase 4 controlled authority switch.
6. Phase 5 base/landmark correction only if mission tests show unbounded drift
   is the next scoring constraint.

This order improves position truth without bypassing the repository's P0 safety
work or adding acceleration integration that the sensor manufacturer explicitly
warns against.
