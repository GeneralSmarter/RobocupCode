#include "Robot.h"

// =====================================================
// Progress monitoring
// =====================================================
// Responsibility:
//   Detects likely drivetrain progress failures from commanded wheel speeds,
//   measured encoder speeds, and yaw change during turns.
// Interacts with:
//   MotorControl.cpp calls updateStuckDriving() for non-turn navigation, and
//   LocalPlanner.cpp calls resetTurnStuckCheck()/updateStuckTurning() for
//   direct yaw turns and point-alignment turns.
// Control flow:
//   This module only sets flags. LocalPlanner.cpp decides whether a stuck flag
//   fails the active navigation goal.
// Global state:
//   Modifies driveStuck, wheelMismatchStuck, turnStuck and their timers.

// Monitors forward/reverse/arc movement using wheel-speed targets and encoder
// rates, all in encoder ticks per second.
//
// What could go wrong:
//   Very low requested speeds are ignored to avoid false positives near stop.
//   A slipping wheel, lifted wheel, jammed wheel, or disconnected encoder can
//   all present as "target is large but measured speed is small/mismatched".
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

  // Wheel-mismatch detection only makes sense when both wheels were expected
  // to move at comparable speeds. Tight turns naturally command unequal wheel
  // speeds and should not trip this branch.
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

// Starts a yaw-progress timer for an in-place or alignment turn.
//
// Input:
//   startYaw is navigation-heading degrees, positive CCW/left.
void resetTurnStuckCheck(float startYaw) {
  turnCheckStartMs = millis();
  turnCheckStartYaw = startYaw;
  turnStuck = false;
}

// Flags turnStuck if the robot has not changed heading enough after the
// configured timeout.
//
// Assumption:
//   navigationHeadingDeg() is healthy enough to detect several degrees of
//   physical rotation. IMU failures can therefore look like a stuck turn.
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

// Clears all progress-failure latches, usually at the start of a new test or
// navigation goal.
void clearStuckFlags() {
  driveStuck = false;
  wheelMismatchStuck = false;
  turnStuck = false;
  driveStuckStartMs = 0;
  wheelMismatchStartMs = 0;
  turnCheckStartMs = millis();
  turnCheckStartYaw = navigationHeadingDeg();
}
