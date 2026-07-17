#include "Robot.h"

// =====================================================
// Motor authority, live safety supervision, watchdog, and PID output
// =====================================================
// Responsibility:
//   Owns all periodic physical motor output. It enforces motion authority,
//   checks live direction-aware safety evidence, manages the command lease and
//   independent watchdog timer, mixes chassis velocity into wheel targets, and
//   converts wheel-speed errors into servo microsecond pulses.
// Interacts with:
//   StateMachine.cpp/LocalPlanner.cpp/Bluetooth.cpp request motion through
//   setMotionCommand() or setAuthorizedMotionCommand(). TofSensors.cpp and
//   LocalPlanner.cpp provide safety predicates. Encoders feed PID speed
//   feedback. RobotCode.ino calls serviceMotorSafetyWatchdog() and
//   updateMotorController() every loop.
// Control flow:
//   Commands are accepted only through setAuthorizedMotionCommand(). The final
//   servo write happens through writeMotorUS(), which re-checks the motor lease
//   before touching the Servo objects.
// Global state:
//   Owns desiredForwardSpeed/desiredTurnSpeed acceptance, motionAuthority,
//   motionCommandAuthority, safety-stop latches, motor lease diagnostics,
//   PID snapshot usage, last motor pulse diagnostics, and loop timing metrics.

static MotionSafetyReason lastSafetyReason = MOTION_SAFETY_CLEAR;
static bool safetyStopActive = false;
static IntervalTimer motorSafetyWatchdogTimer;
static volatile bool motorSafetyWatchdogReady = false;
static volatile bool motorCommandLeaseArmed = false;
static volatile unsigned int motorCommandLeaseTicksRemaining = 0;
static volatile bool motorCommandLeaseTripPending = false;
static volatile unsigned long motorCommandLeaseTrips = 0;
static unsigned long lastMainLoopHeartbeatMs = 0;
static unsigned long lastMainLoopGapMs = 0;
static unsigned long maxMainLoopGapMs = 0;
static unsigned long loopDeadlineMisses = 0;
static const char* maxMainLoopPhase = "none";
static unsigned long maxMainLoopPhaseDurationUs = 0;
static void writeMotorUS(int leftUs, int rightUs);

static void disarmMotorCommandLease() {
  // The lease is the "fresh command" proof. Disarming it means any non-neutral
  // writeMotorUS() call will be converted to STOP_US.
  noInterrupts();
  motorCommandLeaseArmed = false;
  motorCommandLeaseTicksRemaining = 0;
  interrupts();
}

static void renewMotorCommandLease() {
  // Called only after a command passes authority and safety checks. The
  // IntervalTimer ISR counts this down independently of the main loop.
  noInterrupts();
  motorCommandLeaseTicksRemaining =
    MOTOR_COMMAND_LEASE_MS / MOTOR_COMMAND_WATCHDOG_TICK_MS;
  motorCommandLeaseArmed = true;
  interrupts();
}

static void motorSafetyWatchdogISR() {
  // Runs in interrupt context every MOTOR_COMMAND_WATCHDOG_TICK_MS. Keep this
  // short and deterministic: no printing, no allocation, no sensor reads.
  if (!motorCommandLeaseArmed) {
    return;
  }
  if (motorCommandLeaseTicksRemaining > 0) {
    motorCommandLeaseTicksRemaining--;
  }
  if (motorCommandLeaseTicksRemaining != 0) {
    return;
  }

  // Servo::writeMicroseconds only replaces the timer-owned pulse-width word on
  // Teensy 4.0. Keep this ISR limited to neutralization and volatile latches.
  leftMotor.writeMicroseconds(STOP_US);
  rightMotor.writeMicroseconds(STOP_US);
  lastLeftMotorUs = STOP_US;
  lastRightMotorUs = STOP_US;
  motorCommandLeaseArmed = false;
  motorCommandLeaseTripPending = true;
  motorCommandLeaseTrips++;
}

void initializeMotorSafetyWatchdog() {
  // Starts the independent timer that can neutralize motors even if the main
  // loop stalls after publishing a command.
  disarmMotorCommandLease();
  motorSafetyWatchdogReady = motorSafetyWatchdogTimer.begin(
    motorSafetyWatchdogISR, MOTOR_COMMAND_WATCHDOG_TICK_MS * 1000UL);
  if (motorSafetyWatchdogReady) {
    motorSafetyWatchdogTimer.priority(32);
  }
}

void noteMainLoopHeartbeat() {
  // Records loop-to-loop timing so STATUS/CSV can show whether code paths are
  // exceeding MAIN_LOOP_DEADLINE_MS while motors are active.
  const unsigned long now = millis();
  if (lastMainLoopHeartbeatMs != 0) {
    lastMainLoopGapMs = now - lastMainLoopHeartbeatMs;
    if (lastMainLoopGapMs > maxMainLoopGapMs) {
      maxMainLoopGapMs = lastMainLoopGapMs;
    }
    if (lastMainLoopGapMs > MAIN_LOOP_DEADLINE_MS) {
      loopDeadlineMisses++;
    }
  }
  lastMainLoopHeartbeatMs = now;
}

bool isMotorSafetyWatchdogReady() {
  return motorSafetyWatchdogReady;
}

bool isMotorCommandLeaseArmed() {
  return motorCommandLeaseArmed;
}

unsigned long motorCommandLeaseTripCount() {
  return motorCommandLeaseTrips;
}

unsigned long currentMainLoopGapMs() {
  return lastMainLoopGapMs;
}

unsigned long maximumMainLoopGapMs() {
  return maxMainLoopGapMs;
}

unsigned long mainLoopDeadlineMissCount() {
  return loopDeadlineMisses;
}

void recordMainLoopPhaseDuration(const char* phase, unsigned long startedUs) {
  const unsigned long durationUs = micros() - startedUs;
  if (durationUs > maxMainLoopPhaseDurationUs) {
    maxMainLoopPhaseDurationUs = durationUs;
    maxMainLoopPhase = phase;
  }
}

const char* maximumMainLoopPhaseName() {
  return maxMainLoopPhase;
}

unsigned long maximumMainLoopPhaseUs() {
  return maxMainLoopPhaseDurationUs;
}

void resetMainLoopTimingDiagnostics() {
  const unsigned long now = millis();
  lastMainLoopHeartbeatMs = now;
  lastMainLoopGapMs = 0;
  maxMainLoopGapMs = 0;
  loopDeadlineMisses = 0;
  maxMainLoopPhase = "none";
  maxMainLoopPhaseDurationUs = 0;
}

static bool allFanSensorsCurrent() {
  for (int i = RANGE_RIGHT_OUTER; i <= RANGE_LEFT_OUTER; i++) {
    if (!isRangeSensorCurrent((RangeSensorId)i)) {
      return false;
    }
  }
  return true;
}

static bool turnSideCurrent(float turnSpeed) {
  if (turnSpeed > 0.0f) {
    return isRangeSensorCurrent(RANGE_LEFT_INNER) &&
           isRangeSensorCurrent(RANGE_LEFT_OUTER);
  }
  if (turnSpeed < 0.0f) {
    return isRangeSensorCurrent(RANGE_RIGHT_INNER) &&
           isRangeSensorCurrent(RANGE_RIGHT_OUTER);
  }
  return true;
}

static MotionSafetyReason evaluateMotionSafety(MotionAuthority claimant,
                                               float forwardSpeed,
                                               float turnSpeed) {
  // Builds the evidence object consumed by the pure policy in MotionSafety.h.
  // The requested command determines which directions need proof.
  // Match the motor controller's non-neutral boundary exactly. A command at
  // precisely 1 tick/s must not fall between the stop and safety thresholds.
  const bool needsForward = forwardSpeed >= 1.0f;
  const bool needsReverse = forwardSpeed <= -1.0f;
  const bool needsTurnSide = fabs(turnSpeed) >= 1.0f;
  const bool plannerGenerated =
    navigationGoal.active && navigationGoal.authority == claimant;
  const bool needsTurnSweep = needsTurnSide &&
    (fabs(forwardSpeed) <= 1.0f || !plannerGenerated);

  RangeSensorId diagonalSensor;
  float diagonalClearanceMm = 0.0f;
  const bool diagonalClear =
    !needsForward ||
    !getDiagonalClearanceWarning(diagonalSensor, diagonalClearanceMm);

  MotionSafetyEvidence evidence = {
    isRangeSensorCurrent(RANGE_RIGHT_INNER) &&
      isRangeSensorCurrent(RANGE_LEFT_INNER) &&
      isRangeSensorCurrent(RANGE_FRONT),
    !isRangeSensorBlocked(RANGE_FRONT),
    !isTofCloseReadingRevalidating(),
    diagonalClear,
    // P0-03 assumption: the legacy RANGE_FAKE_REAR channel represents a
    // working real rear ToF. P0-04 must replace the legacy name/scaffolding;
    // this supervisor deliberately preserves the current rear-safety API.
    hasTrustedRearCoverage() && isRangeSensorCurrent(RANGE_FAKE_REAR),
    hasTrustedRearCoverage() && !isRangeSensorBlocked(RANGE_FAKE_REAR),
    turnSideCurrent(turnSpeed) && isTurnDirectionObservable(turnSpeed),
    allFanSensorsCurrent() && isTurnSweepSafe()
  };

  return motionSafetyPolicy(needsForward, needsReverse, needsTurnSide,
                            needsTurnSweep, evidence);
}

const char* motionSafetyReasonName(MotionSafetyReason reason) {
  switch (reason) {
    case MOTION_SAFETY_CLEAR: return "clear";
    case MOTION_SAFETY_FRONT_INVALID: return "front_invalid";
    case MOTION_SAFETY_FRONT_BLOCKED: return "front_blocked";
    case MOTION_SAFETY_CLOSE_REVALIDATING: return "close_revalidating";
    case MOTION_SAFETY_DIAGONAL_CLEARANCE: return "diagonal_clearance";
    case MOTION_SAFETY_REAR_INVALID: return "rear_invalid";
    case MOTION_SAFETY_REAR_BLOCKED: return "rear_blocked";
    case MOTION_SAFETY_TURN_SIDE_INVALID: return "turn_side_invalid";
    case MOTION_SAFETY_TURN_SWEEP_BLOCKED: return "turn_sweep_blocked";
    case MOTION_SAFETY_WATCHDOG_UNAVAILABLE: return "watchdog_unavailable";
    case MOTION_SAFETY_COMMAND_LEASE_EXPIRED: return "command_lease_expired";
  }
  return "unknown";
}

MotionSafetyReason lastMotionSafetyReason() {
  return lastSafetyReason;
}

bool isMotionSafetyStopActive() {
  return safetyStopActive;
}

static void rejectUnsafeMotion(MotionSafetyReason reason) {
  // Central fail-closed path for unsafe accepted/pending commands. It clears
  // desired chassis speeds, disarms the lease, drops command authority, writes
  // neutral, and emits one event per changed reason.
  disarmMotorCommandLease();
  desiredForwardSpeed = 0.0f;
  desiredTurnSpeed = 0.0f;
  motionCommandAuthority = MOTION_AUTHORITY_NONE;
  motorStopRequested = true;
  if (!safetyStopActive || lastSafetyReason != reason) {
    sendBluetoothEvent("motion_safety_stop", motionSafetyReasonName(reason));
  }
  lastSafetyReason = reason;
  safetyStopActive = true;
  writeMotorUS(STOP_US, STOP_US);
}

static void noteSafeMotion() {
  safetyStopActive = false;
}

void serviceMotorSafetyWatchdog() {
  // Converts the ISR's volatile trip latch into normal-loop state and
  // telemetry. The ISR already wrote neutral; this records why.
  noInterrupts();
  const bool tripPending = motorCommandLeaseTripPending;
  motorCommandLeaseTripPending = false;
  interrupts();
  if (!tripPending) {
    return;
  }

  rejectUnsafeMotion(MOTION_SAFETY_COMMAND_LEASE_EXPIRED);
  lastLeftMotorUs = STOP_US;
  lastRightMotorUs = STOP_US;
  sendBluetoothEvent("motor_command_lease_expired", "watchdog_neutralized");
}

// =====================================================
// Motor and PID helpers
// =====================================================
void stopMotors() {
  // Public neutral command. It is intentionally available even without motion
  // authority so every cleanup path can force STOP_US.
  disarmMotorCommandLease();
  desiredForwardSpeed = 0.0f;
  desiredTurnSpeed = 0.0f;
  motionCommandAuthority = MOTION_AUTHORITY_NONE;
  motorStopRequested = true;
  writeMotorUS(STOP_US, STOP_US);
}

const char* motionAuthorityName(MotionAuthority authority) {
  switch (authority) {
    case MOTION_AUTHORITY_NONE: return "NONE";
    case MOTION_AUTHORITY_MISSION: return "MISSION";
    case MOTION_AUTHORITY_TEST: return "TEST";
    case MOTION_AUTHORITY_MANUAL: return "MANUAL";
  }
  return "UNKNOWN";
}

void revokeMotionAuthority() {
  // Removes the current owner first, then stops. After this, an old planner or
  // manual command cannot be accepted by setAuthorizedMotionCommand().
  motionAuthority = MOTION_AUTHORITY_NONE;
  stopMotors();
}

bool claimMotionAuthority(MotionAuthority authority) {
  // Grants exclusive ownership to mission, test, or manual mode. The stop call
  // resets any previous desired command before the new owner begins.
  if (authority == MOTION_AUTHORITY_NONE || motionAuthority != MOTION_AUTHORITY_NONE) {
    return false;
  }
  motionAuthority = authority;
  stopMotors();
  return true;
}

bool setAuthorizedMotionCommand(MotionAuthority authority, float forwardSpeed, float turnSpeed) {
  // Accepts a chassis command in encoder ticks/s only if the caller owns the
  // current motion authority and the latest safety evidence permits it.
  //
  // SAFETY: Passing this function does not directly write the motors. It only
  // stores desired speeds and renews the lease; updateMotorController() still
  // performs a fresh safety check before output.
  if (!motionAuthorityAllows(motionAuthority, authority)) {
    stopMotors();
    return false;
  }
  const bool nonNeutral = fabs(forwardSpeed) >= 1.0f || fabs(turnSpeed) >= 1.0f;
  if (nonNeutral && !motorSafetyWatchdogReady) {
    rejectUnsafeMotion(MOTION_SAFETY_WATCHDOG_UNAVAILABLE);
    return false;
  }
  if (nonNeutral && motorCommandLeaseTripPending) {
    rejectUnsafeMotion(MOTION_SAFETY_COMMAND_LEASE_EXPIRED);
    return false;
  }
  MotionSafetyReason safetyReason =
    evaluateMotionSafety(authority, forwardSpeed, turnSpeed);
  if (safetyReason != MOTION_SAFETY_CLEAR) {
    rejectUnsafeMotion(safetyReason);
    return false;
  }
  noteSafeMotion();
  desiredForwardSpeed = forwardSpeed;
  desiredTurnSpeed = turnSpeed;
  motionCommandAuthority = authority;
  motorStopRequested = !nonNeutral;
  if (nonNeutral) {
    renewMotorCommandLease();
  } else {
    disarmMotorCommandLease();
  }
  return true;
}

static void writeMotorUS(int leftUs, int rightUs) {
  // Final servo-output gate. Even if a bug computes a non-neutral pulse after
  // the lease expires, this function clamps it back to STOP_US before writing
  // the Servo objects.
  leftUs = constrain(leftUs, MIN_US, MAX_US);
  rightUs = constrain(rightUs, MIN_US, MAX_US);

  const bool nonNeutral = leftUs != STOP_US || rightUs != STOP_US;
  noInterrupts();
  const bool leaseAllowsOutput =
    motorLeasePolicy(nonNeutral, motorSafetyWatchdogReady,
                     motorCommandLeaseArmed && !motorCommandLeaseTripPending) ==
    MOTION_SAFETY_CLEAR;
  if (!leaseAllowsOutput) {
    leftUs = STOP_US;
    rightUs = STOP_US;
  }
  lastLeftMotorUs = leftUs;
  lastRightMotorUs = rightUs;

  leftMotor.writeMicroseconds(leftUs);
  rightMotor.writeMicroseconds(rightUs);
  interrupts();
}

int updatePID(float target, float actual, float &integral, float &lastError, float dt, int baseCommand) {
  // PID correction around a feed-forward servo pulse.
  // Inputs:
  //   target/actual are wheel speeds in encoder ticks/s, dt is seconds, and
  //   baseCommand is the feed-forward pulse in microseconds.
  // Output:
  //   Servo pulse in microseconds, clamped to a small band around baseCommand
  //   and to the global servo min/max.
  float error = target - actual;

  // Integral accumulates persistent error so a wheel that is consistently slow
  // receives more pulse width over time. The clamp prevents wind-up after a
  // stall or blocked wheel.
  integral += error * dt;
  integral = constrain(integral, -3000.0, 3000.0);

  // Derivative looks at how fast error is changing. Kd is currently zero, but
  // keeping the term visible makes future tuning easier.
  float derivative = (error - lastError) / dt;
  float correction = Kp * error + Ki * integral + Kd * derivative;

  lastError = error;

  // Add closed-loop correction to the feed-forward estimate. This code assumes
  // a larger pulse magnitude in the requested direction means more wheel speed.
  int command = baseCommand + correction;

  command = constrain(command, baseCommand - 150, baseCommand + 150);
  command = constrain(command, MIN_US, MAX_US);

  return command;
}

static int wheelFeedForwardBase(float targetTicksPerSec, int forwardBaseUs,
                                int reverseBaseUs, float fullScaleTicksPerSec) {
  // Converts the requested wheel speed into an approximate open-loop pulse.
  // PID then trims around this value using encoder feedback.
  float scale = constrain(fabs(targetTicksPerSec) / fullScaleTicksPerSec, 0.0, 1.0);
  if (targetTicksPerSec >= 0.0) {
    return STOP_US + (int)((forwardBaseUs - STOP_US) * scale);
  }
  return STOP_US - (int)((STOP_US - reverseBaseUs) * scale);
}

// The only periodic writer of motor pulse widths.  Behaviours request a
// chassis velocity through setMotionCommand(); they never own a servo output.
void updateMotorController() {
  // This function is intentionally the final authority on physical motor
  // pulses. Planner/state/safety code only changes desiredForwardSpeed and
  // desiredTurnSpeed, which makes output timing and debugging predictable.
  unsigned long now = millis();
  if (now - lastMotorControlUpdateMs < MOTOR_CONTROL_INTERVAL_MS) {
    return;
  }

  float dt = lastMotorControlUpdateMs == 0
               ? MOTOR_CONTROL_INTERVAL_MS / 1000.0
               : (now - lastMotorControlUpdateMs) / 1000.0;
  lastMotorControlUpdateMs = now;

  // Chassis command -> differential-drive wheel commands. Positive turn is
  // CCW/left, so it slows the left wheel and speeds the right wheel.
  const float leftTarget = constrain(
    leftWheelTargetFromChassis(desiredForwardSpeed, desiredTurnSpeed),
    -3000.0, 3000.0);
  const float rightTarget = constrain(
    rightWheelTargetFromChassis(desiredForwardSpeed, desiredTurnSpeed),
    -3000.0, 3000.0);
  lastRequestedLeftWheelSpeed = leftTarget;
  lastRequestedRightWheelSpeed = rightTarget;

  if (!motionAuthorityAllows(motionAuthority, motionCommandAuthority) ||
      motorStopRequested ||
      (fabs(leftTarget) < 1.0 && fabs(rightTarget) < 1.0)) {
    // Neutral is written every control period while stopped. This is safer than
    // relying on a previous servo value after a planner/sensor state change.
    lastMotorOutputMode = "neutral";
    writeMotorUS(STOP_US, STOP_US);
    return;
  }

  MotionSafetyReason leaseReason = motorLeasePolicy(
    true, motorSafetyWatchdogReady,
    motorCommandLeaseArmed && !motorCommandLeaseTripPending);
  if (leaseReason != MOTION_SAFETY_CLEAR) {
    rejectUnsafeMotion(leaseReason);
    writeMotorUS(STOP_US, STOP_US);
    return;
  }

  // Authority is necessary but not sufficient. Re-evaluate the latest sensor
  // snapshot on every motor-control opportunity so an old accepted command
  // cannot continue after direction-relevant evidence becomes unsafe.
  MotionSafetyReason safetyReason = evaluateMotionSafety(
    motionCommandAuthority, desiredForwardSpeed, desiredTurnSpeed);
  if (safetyReason != MOTION_SAFETY_CLEAR) {
    rejectUnsafeMotion(safetyReason);
    writeMotorUS(STOP_US, STOP_US);
    return;
  }
  noteSafeMotion();

  long leftCount;
  long rightCount;
  readEncoderCounts(leftCount, rightCount);

  float leftSpeed = (leftCount - lastLeftCount) / dt;
  float rightSpeed = (rightCount - lastRightCount) / dt;
  lastMeasuredLeftWheelSpeed = leftSpeed;
  lastMeasuredRightWheelSpeed = rightSpeed;
  lastLeftCount = leftCount;
  lastRightCount = rightCount;

  // Pivot turns use yaw-based progress monitoring in LocalPlanner.  Feeding
  // their opposing wheel targets into straight-drive monitoring can leave a
  // stale drive-stuck flag for the next point goal.
  bool navigationTurn = navigationGoal.active &&
                        navigationGoal.mode == NAV_GOAL_TURN;
  if (!navigationTurn) {
    updateStuckDriving(leftTarget, rightTarget, leftSpeed, rightSpeed);
  }

  // Restore the proven V4 in-place turn pulse pairs for navigation turns.
  // They remain owned by this single periodic writer, so planner and safety
  // scheduling stay nonblocking and no behaviour writes a servo directly.
  bool useCalibratedTurnPulses = navigationGoal.active &&
                                 navigationGoal.mode == NAV_GOAL_TURN &&
                                 fabs(desiredForwardSpeed) < 1.0 &&
                                 fabs(desiredTurnSpeed) >= 1.0;
  if (useCalibratedTurnPulses) {
    lastMotorOutputMode = "nav_calibrated_pivot";
    bool leftTurn = desiredTurnSpeed > 0.0;
    bool slowTurn = fabs(desiredTurnSpeed) < PLANNER_TURN_TARGET_SPEED;
    int leftPulse;
    int rightPulse;
    if (navigationGoal.owner == NAV_OWNER_WEIGHT_SCAN) {
      int offset = constrain(weightScanTurnOffsetUs,
                             WEIGHT_SCAN_TURN_OFFSET_MIN_US,
                             WEIGHT_SCAN_TURN_OFFSET_MAX_US);
      leftPulse = leftTurn ? STOP_US - offset : STOP_US + offset;
      rightPulse = leftTurn ? STOP_US + offset : STOP_US - offset;
    } else if (leftTurn) {
      leftPulse = slowTurn ? TURN_LEFT_LEFT_SLOW_US : TURN_LEFT_LEFT_FAST_US;
      rightPulse = slowTurn ? TURN_LEFT_RIGHT_SLOW_US : TURN_LEFT_RIGHT_FAST_US;
    } else {
      leftPulse = slowTurn ? TURN_RIGHT_LEFT_SLOW_US : TURN_RIGHT_LEFT_FAST_US;
      rightPulse = slowTurn ? TURN_RIGHT_RIGHT_SLOW_US : TURN_RIGHT_RIGHT_FAST_US;
    }
    writeMotorUS(leftPulse, rightPulse);
    return;
  }

  // A turn command is intentionally much lower than the nominal straight-line
  // wheel speed.  Scale an in-place turn against its own calibrated full-turn
  // target, otherwise a 1050 tick/s turn is incorrectly treated as a 46%
  // straight-drive request and cannot overcome drivetrain friction.
  bool turningInPlace = fabs(desiredForwardSpeed) < 1.0 && fabs(desiredTurnSpeed) >= 1.0;
  float feedForwardReference = turningInPlace ? PLANNER_TURN_TARGET_SPEED : baseTargetSpeed;

  // Feed-forward follows requested speed so the clearance cap and the turn
  // slow zone produce a real reduction in motor effort, while full-rate turns
  // retain their calibrated torque.
  int leftBase = wheelFeedForwardBase(leftTarget, leftForwardBaseUs, LEFT_REVERSE_US,
                                      feedForwardReference);
  int rightBase = wheelFeedForwardBase(rightTarget, rightForwardBaseUs, RIGHT_REVERSE_US,
                                       feedForwardReference);

  int leftCommand = updatePID(leftTarget, leftSpeed, leftIntegral, lastLeftError, dt, leftBase);
  int rightCommand = updatePID(rightTarget, rightSpeed, rightIntegral, lastRightError, dt, rightBase);
  lastMotorOutputMode = "wheel_pid";
  writeMotorUS(leftCommand, rightCommand);
}
