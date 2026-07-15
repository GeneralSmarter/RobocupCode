#include "Robot.h"

// =====================================================
// Progress monitoring
// =====================================================

void updateStuckDriving(float leftTargetSpeed, float rightTargetSpeed,
                        float leftSpeed, float rightSpeed) {
  float absLeftTarget = fabs(leftTargetSpeed);
  float absRightTarget = fabs(rightTargetSpeed);
  float avgTargetSpeed = (absLeftTarget + absRightTarget) * 0.5;
  float maxTargetSpeed = max(absLeftTarget, absRightTarget);
  float minTargetSpeed = min(absLeftTarget, absRightTarget);
  float absLeftSpeed = fabs(leftSpeed);
  float absRightSpeed = fabs(rightSpeed);
  float avgSpeed = (absLeftSpeed + absRightSpeed) * 0.5;
  float maxWheelSpeed = max(absLeftSpeed, absRightSpeed);
  float minWheelSpeed = min(absLeftSpeed, absRightSpeed);
  unsigned long now = millis();

  if (avgTargetSpeed > STUCK_COMMAND_SPEED_MIN && avgSpeed < STUCK_ENCODER_SPEED_MIN) {
    if (driveStuckStartMs == 0) {
      driveStuckStartMs = now;
    }
    if (now - driveStuckStartMs > DRIVE_STUCK_TIME_MS) {
      driveStuck = true;
    }
  } else {
    driveStuckStartMs = 0;
    driveStuck = false;
  }

  bool comparableWheelTargets =
    maxTargetSpeed > WHEEL_MISMATCH_SPEED_MIN &&
    minTargetSpeed >= maxTargetSpeed * WHEEL_MISMATCH_EXPECTED_RATIO;
  if (comparableWheelTargets && maxWheelSpeed > WHEEL_MISMATCH_SPEED_MIN &&
      minWheelSpeed < maxWheelSpeed * WHEEL_MISMATCH_RATIO) {
    if (wheelMismatchStartMs == 0) {
      wheelMismatchStartMs = now;
    }
    if (now - wheelMismatchStartMs > WHEEL_MISMATCH_TIME_MS) {
      wheelMismatchStuck = true;
    }
  } else {
    wheelMismatchStartMs = 0;
    wheelMismatchStuck = false;
  }
}

void resetTurnStuckCheck(float startYaw) {
  turnCheckStartMs = millis();
  turnCheckStartYaw = startYaw;
  turnStuck = false;
}

void updateStuckTurning(float currentYaw) {
  if (millis() - turnCheckStartMs < TURN_STUCK_TIME_MS) {
    return;
  }
  if (fabs(wrapAngle(currentYaw - turnCheckStartYaw)) < TURN_STUCK_YAW_MIN_DEG) {
    turnStuck = true;
  } else {
    turnStuck = false;
    turnCheckStartMs = millis();
    turnCheckStartYaw = currentYaw;
  }
}

void clearStuckFlags() {
  driveStuck = false;
  wheelMismatchStuck = false;
  turnStuck = false;
  driveStuckStartMs = 0;
  wheelMismatchStartMs = 0;
  turnCheckStartMs = millis();
  turnCheckStartYaw = navigationHeadingDeg();
}
