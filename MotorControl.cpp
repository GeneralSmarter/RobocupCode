#include "Robot.h"

// =====================================================
// Motor and PID helpers
// =====================================================
void stopMotors() {
  setMotionCommand(0.0, 0.0);
  motorStopRequested = true;
}

void writeMotorUS(int leftUs, int rightUs) {
  leftUs = constrain(leftUs, MIN_US, MAX_US);
  rightUs = constrain(rightUs, MIN_US, MAX_US);

  lastLeftMotorUs = leftUs;
  lastRightMotorUs = rightUs;

  leftMotor.writeMicroseconds(leftUs);
  rightMotor.writeMicroseconds(rightUs);
}

int updatePID(float target, float actual, float &integral, float &lastError, float dt, int baseCommand) {
  float error = target - actual;

  integral += error * dt;
  integral = constrain(integral, -3000.0, 3000.0);

  float derivative = (error - lastError) / dt;
  float correction = Kp * error + Ki * integral + Kd * derivative;

  lastError = error;

  int command = baseCommand + correction;

  command = constrain(command, baseCommand - 150, baseCommand + 150);
  command = constrain(command, MIN_US, MAX_US);

  return command;
}

static int wheelFeedForwardBase(float targetTicksPerSec, int forwardBaseUs,
                                int reverseBaseUs, float fullScaleTicksPerSec) {
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

  // Chassis command -> differential-drive wheel commands. Positive turn adds
  // speed to the left wheel and subtracts it from the right wheel.
  const float leftTarget = constrain(desiredForwardSpeed + desiredTurnSpeed, -3000.0, 3000.0);
  const float rightTarget = constrain(desiredForwardSpeed - desiredTurnSpeed, -3000.0, 3000.0);

  if (motorStopRequested || (fabs(leftTarget) < 1.0 && fabs(rightTarget) < 1.0)) {
    // Neutral is written every control period while stopped. This is safer than
    // relying on a previous servo value after a planner/sensor state change.
    writeMotorUS(STOP_US, STOP_US);
    return;
  }

  long leftCount;
  long rightCount;
  readEncoderCounts(leftCount, rightCount);

  float leftSpeed = (leftCount - lastLeftCount) / dt;
  float rightSpeed = (rightCount - lastRightCount) / dt;
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
    bool rightTurn = desiredTurnSpeed > 0.0;
    bool slowTurn = fabs(desiredTurnSpeed) < PLANNER_TURN_TARGET_SPEED;
    int leftPulse;
    int rightPulse;
    if (navigationGoal.owner == NAV_OWNER_WEIGHT_SCAN) {
      int offset = constrain(weightScanTurnOffsetUs,
                             WEIGHT_SCAN_TURN_OFFSET_MIN_US,
                             WEIGHT_SCAN_TURN_OFFSET_MAX_US);
      leftPulse = rightTurn ? STOP_US + offset : STOP_US - offset;
      rightPulse = rightTurn ? STOP_US - offset : STOP_US + offset;
    } else if (rightTurn) {
      leftPulse = slowTurn ? TURN_RIGHT_LEFT_SLOW_US : TURN_RIGHT_LEFT_FAST_US;
      rightPulse = slowTurn ? TURN_RIGHT_RIGHT_SLOW_US : TURN_RIGHT_RIGHT_FAST_US;
    } else {
      leftPulse = slowTurn ? TURN_LEFT_LEFT_SLOW_US : TURN_LEFT_LEFT_FAST_US;
      rightPulse = slowTurn ? TURN_LEFT_RIGHT_SLOW_US : TURN_LEFT_RIGHT_FAST_US;
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
  writeMotorUS(leftCommand, rightCommand);
}
