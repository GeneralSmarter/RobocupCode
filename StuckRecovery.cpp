#include "Robot.h"

// =====================================================
// Stuck recovery
// =====================================================
static bool shouldRecoveryTurnRight() {
  bool leftValid = isRangeSensorValid(RANGE_LEFT);
  bool rightValid = isRangeSensorValid(RANGE_RIGHT);
  uint16_t leftMm = getRangeSensorDistance(RANGE_LEFT);
  uint16_t rightMm = getRangeSensorDistance(RANGE_RIGHT);

  if (leftValid && !rightValid) {
    Serial.println("Recovery choice: only left side valid, turn right.");
    return true;
  }

  if (!leftValid && rightValid) {
    Serial.println("Recovery choice: only right side valid, turn left.");
    return false;
  }

  if (!leftValid && !rightValid) {
    Serial.println("Recovery choice: no valid side readings, default turn right.");
    return true;
  }

  if (leftMm < rightMm) {
    Serial.println("Recovery choice: obstacle nearer left, turn right.");
    return true;
  }

  Serial.println("Recovery choice: obstacle nearer right or equal, turn left.");
  return false;
}

void runStuckRecovery() {
  if (inRecovery) {
    Serial.println("Already in STUCK_RECOVERY. Ignoring nested trigger.");
    return;
  }

  RobotState stateBeforeRecovery = currentState;
  setRobotState(STUCK_RECOVERY);
  setMotionCommand(0.0, 0.0);

  inRecovery = true;

  stopMotors();
  updateTOFSensors();
  updateOdometry();

  stuckRecoveryCount++;

  Serial.println();
  Serial.println("STUCK_RECOVERY started.");
  Serial.print("Recovery count: ");
  Serial.print(stuckRecoveryCount);
  Serial.print(" / ");
  Serial.println(MAX_STUCK_RECOVERIES);
  sendBluetoothEvent("stuck_recovery_start", "triggered");

  if (stuckRecoveryCount > MAX_STUCK_RECOVERIES) {
    Serial.println("Too many stuck recoveries. Stopping safely.");
    stoppedSafely = true;
    stopMotors();
    setRobotState(END_MATCH);
    inRecovery = false;
    sendBluetoothEvent("stuck_recovery_end", "max_recoveries");
    return;
  }

  Serial.print("Front: ");
  Serial.print(getRangeSensorDistance(RANGE_FRONT));
  Serial.print(" mm  Left: ");
  Serial.print(getRangeSensorDistance(RANGE_LEFT));
  Serial.print(" mm  Right: ");
  Serial.print(getRangeSensorDistance(RANGE_RIGHT));
  Serial.println(" mm");

  clearStuckFlags();

  reverseDistanceOpenLoop(STUCK_REVERSE_DISTANCE_M);
  if (!robotRunEnabled || currentState == END_MATCH) {
    inRecovery = false;
    sendBluetoothEvent("stuck_recovery_end", "aborted_after_reverse");
    return;
  }

  updateTOFSensors();
  updateOdometry();

  if (shouldRecoveryTurnRight()) {
    turnAngleNoStuckCheck(STUCK_TURN_ANGLE_DEG);
  } else {
    turnAngleNoStuckCheck(-STUCK_TURN_ANGLE_DEG);
  }
  if (!robotRunEnabled || currentState == END_MATCH) {
    inRecovery = false;
    sendBluetoothEvent("stuck_recovery_end", "aborted_after_turn");
    return;
  }

  waitForFrontClear(FRONT_CLEAR_SETTLE_TIMEOUT_MS);
  if (!robotRunEnabled || currentState == END_MATCH) {
    inRecovery = false;
    sendBluetoothEvent("stuck_recovery_end", "aborted_after_clear_wait");
    return;
  }

  if (!isRangeSensorBlocked(RANGE_FRONT)) {
    float recoveryHeading = readYawDeg();
    driveDistanceWithHeadingNoAvoid(STUCK_FORWARD_DISTANCE_M, recoveryHeading);
    if (!robotRunEnabled || currentState == END_MATCH) {
      inRecovery = false;
      sendBluetoothEvent("stuck_recovery_end", "aborted_after_forward");
      return;
    }
  } else {
    Serial.println("Recovery forward skipped because front is still blocked.");
  }

  clearStuckFlags();
  updateTOFSensors();
  updateOdometry();

  Serial.println("STUCK_RECOVERY complete. Resuming waypoint.");
  printPose();
  sendBluetoothEvent("stuck_recovery_end", "complete");

  inRecovery = false;

  if (!stoppedSafely) {
    setRobotState(stateBeforeRecovery);
  }
}

void updateStuckDriving(float targetSpeed, float leftSpeed, float rightSpeed) {
  float absLeftSpeed = fabs(leftSpeed);
  float absRightSpeed = fabs(rightSpeed);
  float avgSpeed = (absLeftSpeed + absRightSpeed) / 2.0;
  float maxWheelSpeed = max(absLeftSpeed, absRightSpeed);
  float minWheelSpeed = min(absLeftSpeed, absRightSpeed);

  unsigned long now = millis();

  if (targetSpeed > STUCK_COMMAND_SPEED_MIN && avgSpeed < STUCK_ENCODER_SPEED_MIN) {
    if (driveStuckStartMs == 0) {
      driveStuckStartMs = now;
    }

    if (!driveStuck && now - driveStuckStartMs > DRIVE_STUCK_TIME_MS) {
      driveStuck = true;
      Serial.println("STUCK: drive command high but encoder speed low");
    }
  } else {
    driveStuckStartMs = 0;
    driveStuck = false;
  }

  if (maxWheelSpeed > WHEEL_MISMATCH_SPEED_MIN && minWheelSpeed < maxWheelSpeed * WHEEL_MISMATCH_RATIO) {
    if (wheelMismatchStartMs == 0) {
      wheelMismatchStartMs = now;
    }

    if (!wheelMismatchStuck && now - wheelMismatchStartMs > WHEEL_MISMATCH_TIME_MS) {
      wheelMismatchStuck = true;
      Serial.println("STUCK: left/right wheel speed mismatch");
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
  unsigned long now = millis();

  if (now - turnCheckStartMs < TURN_STUCK_TIME_MS) {
    return;
  }

  float yawChange = fabs(wrapAngle(currentYaw - turnCheckStartYaw));

  if (yawChange < TURN_STUCK_YAW_MIN_DEG) {
    if (!turnStuck) {
      turnStuck = true;
      Serial.println("STUCK: turn command active but yaw not changing");
    }
  } else {
    turnStuck = false;
    turnCheckStartMs = now;
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
  turnCheckStartYaw = readYawDeg();
}
