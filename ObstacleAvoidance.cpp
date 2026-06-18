#include "Robot.h"

// =====================================================
// Obstacle avoidance
// =====================================================
void runObstacleAvoidance(float originalPathHeadingDeg) {
  if (handleStuckPriority()) {
    return;
  }

  setRobotState(OBSTACLE_AVOID);
  setMotionCommand(0.0, 0.0);

  unsigned long startTime = millis();

  stopMotors();
  updateTOFSensors();
  updateOdometry();

  Serial.println();
  Serial.println("OBSTACLE_AVOID started.");
  Serial.print("Front: ");
  Serial.print(getRangeSensorDistance(RANGE_FRONT));
  Serial.print(" mm  Left: ");
  Serial.print(getRangeSensorDistance(RANGE_LEFT));
  Serial.print(" mm  Right: ");
  Serial.print(getRangeSensorDistance(RANGE_RIGHT));
  Serial.println(" mm");
  sendBluetoothEvent("obstacle_avoid_start", "front_blocked");

  reverseDistanceOpenLoop(AVOID_REVERSE_DISTANCE_M);
  if (!robotRunEnabled || currentState == END_MATCH) {
    sendBluetoothEvent("obstacle_avoid_end", "aborted");
    return;
  }

  if (millis() - startTime > AVOID_TIMEOUT_MS) {
    Serial.println("OBSTACLE_AVOID timeout after reverse.");
    stopMotors();
    sendBluetoothEvent("obstacle_avoid_end", "timeout_after_reverse");
    return;
  }

  updateTOFSensors();

  AvoidTurnChoice turnChoice = chooseAvoidTurnDirection();

  if (turnChoice == AVOID_TURN_LEFT) {
    Serial.println("Avoid choice: turn left.");
    turnAngle(-AVOID_TURN_ANGLE_DEG);
  } else {
    Serial.println("Avoid choice: turn right.");
    turnAngle(AVOID_TURN_ANGLE_DEG);
  }
  if (!robotRunEnabled || currentState == END_MATCH) {
    sendBluetoothEvent("obstacle_avoid_end", "aborted");
    return;
  }

  if (millis() - startTime > AVOID_TIMEOUT_MS) {
    Serial.println("OBSTACLE_AVOID timeout after turn.");
    stopMotors();
    sendBluetoothEvent("obstacle_avoid_end", "timeout_after_turn");
    return;
  }

  updateTOFSensors();

  if (!isRangeSensorBlocked(RANGE_FRONT)) {
    float bypassHeading = readYawDeg();

    if (ENABLE_SIDE_WALL_FOLLOW_FALLBACK) {
      driveDistanceWithHeadingWallFallback(AVOID_BYPASS_DISTANCE_M, bypassHeading);
    } else {
      driveDistanceWithHeadingNoAvoid(AVOID_BYPASS_DISTANCE_M, bypassHeading);
    }
    if (!robotRunEnabled || currentState == END_MATCH) {
      sendBluetoothEvent("obstacle_avoid_end", "aborted");
      return;
    }
  } else {
    Serial.println("Avoid bypass skipped because front is still blocked.");
  }

  updateTOFSensors();
  updateOdometry();

  if (isRangeSensorBlocked(RANGE_FRONT)) {
    Serial.println("OBSTACLE_AVOID failed to clear front. Starting stuck recovery.");
    sendBluetoothEvent("obstacle_avoid_end", "front_still_blocked");
    runStuckRecovery();
    return;
  }

  float rejoinTurn = wrapAngle(originalPathHeadingDeg - readYawDeg());

  Serial.print("Avoid rejoin turn: ");
  Serial.print(rejoinTurn, 2);
  Serial.print(" deg back toward path heading ");
  Serial.println(originalPathHeadingDeg, 2);

  if (fabs(rejoinTurn) > TURN_TOLERANCE_DEG) {
    turnAngle(rejoinTurn);
    if (!robotRunEnabled || currentState == END_MATCH) {
      sendBluetoothEvent("obstacle_avoid_end", "aborted");
      return;
    }
  }

  updateTOFSensors();

  if (!isRangeSensorBlocked(RANGE_FRONT)) {
    Serial.print("Avoid rejoin forward: ");
    Serial.print(AVOID_REJOIN_DISTANCE_M, 3);
    Serial.println(" m along original path heading.");
    driveDistanceWithHeadingNoAvoid(AVOID_REJOIN_DISTANCE_M, originalPathHeadingDeg);
    if (!robotRunEnabled || currentState == END_MATCH) {
      sendBluetoothEvent("obstacle_avoid_end", "aborted");
      return;
    }
  } else {
    Serial.println("Avoid rejoin skipped because front is blocked after turn.");
  }

  updateTOFSensors();
  updateOdometry();

  if (isRangeSensorBlocked(RANGE_FRONT)) {
    Serial.println("OBSTACLE_AVOID rejoin ended blocked. Starting stuck recovery.");
    sendBluetoothEvent("obstacle_avoid_end", "rejoin_blocked");
    runStuckRecovery();
    return;
  }

  Serial.println("OBSTACLE_AVOID complete. Resuming waypoint.");
  printPose();
  sendBluetoothEvent("obstacle_avoid_end", "complete");
}

AvoidTurnChoice chooseAvoidTurnDirection() {
  bool leftValid = isRangeSensorValid(RANGE_LEFT);
  bool rightValid = isRangeSensorValid(RANGE_RIGHT);
  uint16_t leftMm = getRangeSensorDistance(RANGE_LEFT);
  uint16_t rightMm = getRangeSensorDistance(RANGE_RIGHT);

  Serial.print("Avoid side check: left=");
  Serial.print(leftMm);
  Serial.print(leftValid ? " valid" : " invalid");
  Serial.print(" mm  right=");
  Serial.print(rightMm);
  Serial.print(rightValid ? " valid" : " invalid");
  Serial.println(" mm");

  if (leftValid && !rightValid) {
    Serial.println("Avoid reason: only left side has valid clearance.");
    return AVOID_TURN_LEFT;
  }

  if (!leftValid && rightValid) {
    Serial.println("Avoid reason: only right side has valid clearance.");
    return AVOID_TURN_RIGHT;
  }

  if (!leftValid && !rightValid) {
    Serial.println("Avoid reason: no valid side clearance, default right.");
    return AVOID_TURN_RIGHT;
  }

  if (leftMm > rightMm + AVOID_OPEN_MARGIN_MM) {
    Serial.println("Avoid reason: left side has more clearance.");
    return AVOID_TURN_LEFT;
  }

  if (rightMm > leftMm + AVOID_OPEN_MARGIN_MM) {
    Serial.println("Avoid reason: right side has more clearance.");
    return AVOID_TURN_RIGHT;
  }

  Serial.println("Avoid reason: similar clearance, default right.");
  return AVOID_TURN_RIGHT;
}

void reverseDistanceOpenLoop(float distanceMetres) {
  resetEncodersAndPID();

  long targetTicks = distanceMetres * TICKS_PER_METRE;
  unsigned long reverseStartMs = millis();
  const unsigned long REVERSE_TIMEOUT_MS = 1500;

  Serial.print("Reverse: ");
  Serial.print(distanceMetres, 3);
  Serial.println(" m");

  while (true) {
    if (handleBluetoothCommands()) {
      stopMotors();
      return;
    }

    long leftCount;
    long rightCount;
    readEncoderCounts(leftCount, rightCount);

    long avgTicks = (labs(leftCount) + labs(rightCount)) / 2;

    if (avgTicks >= targetTicks) {
      stopMotors();
      updateOdometry();
      delay(200);
      return;
    }

    if (millis() - reverseStartMs > REVERSE_TIMEOUT_MS) {
      stopMotors();
      updateOdometry();
      Serial.println("Reverse timeout. Continuing.");
      delay(200);
      return;
    }

    writeMotorUS(LEFT_REVERSE_US, RIGHT_REVERSE_US);

    delay(20);
  }
}
