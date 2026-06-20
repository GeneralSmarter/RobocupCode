#include "Robot.h"

// =====================================================
// Waypoint navigation
// =====================================================
static float waypointHeadingForwardScale(float absHeadingErrorDeg) {
  if (absHeadingErrorDeg <= WAYPOINT_STEER_SLOW_START_DEG) {
    return 1.0;
  }

  if (absHeadingErrorDeg >= WAYPOINT_PIVOT_TURN_DEG) {
    return WAYPOINT_MIN_FORWARD_SCALE;
  }

  float span = WAYPOINT_PIVOT_TURN_DEG - WAYPOINT_STEER_SLOW_START_DEG;
  float t = (absHeadingErrorDeg - WAYPOINT_STEER_SLOW_START_DEG) / span;
  return 1.0 - t * (1.0 - WAYPOINT_MIN_FORWARD_SCALE);
}

static float waypointDistanceForwardScale(float distanceMetres) {
  if (distanceMetres >= WAYPOINT_FINAL_APPROACH_M) {
    return 1.0;
  }

  float t = distanceMetres / WAYPOINT_FINAL_APPROACH_M;
  return WAYPOINT_FINAL_MIN_FORWARD_SCALE + t * (1.0 - WAYPOINT_FINAL_MIN_FORWARD_SCALE);
}

void goToPoint(float targetX, float targetY) {
  float dx = targetX - robotX;
  float dy = targetY - robotY;
  float distance = sqrt(dx * dx + dy * dy);
  float targetHeading = atan2(dy, dx) * 180.0 / PI;
  float currentYaw = readYawDeg();
  float turnNeeded = wrapAngle(targetHeading - currentYaw);

  Serial.println();
  Serial.print("Going to x=");
  Serial.print(targetX, 3);
  Serial.print(" y=");
  Serial.println(targetY, 3);

  Serial.print("Current pose: ");
  printPose();

  Serial.print("dx: ");
  Serial.print(dx, 3);
  Serial.print("  dy: ");
  Serial.print(dy, 3);
  Serial.print("  target heading: ");
  Serial.print(targetHeading, 2);
  Serial.print("  turn needed: ");
  Serial.print(turnNeeded, 2);
  Serial.print("  distance: ");
  Serial.println(distance, 3);

  if (distance <= WAYPOINT_TOLERANCE_M) {
    stuckRecoveryCount = 0;
    Serial.print("Pose after waypoint: ");
    printPose();
    return;
  }

  bool finalApproach = distance <= WAYPOINT_FINAL_APPROACH_M;
  bool skipSmallFinalTurn =
    finalApproach && fabs(turnNeeded) <= WAYPOINT_FINAL_SKIP_TURN_DEG;

  if (skipSmallFinalTurn && fabs(turnNeeded) > TURN_TOLERANCE_DEG) {
    Serial.print("Final approach: skipping ");
    Serial.print(turnNeeded, 2);
    Serial.println(" deg initial re-aim before lookahead steering.");
    sendBluetoothEvent("waypoint_final", "skip_turn");
  } else if (fabs(turnNeeded) > WAYPOINT_PIVOT_TURN_DEG) {
    turnAngle(turnNeeded);
    if (!robotRunEnabled || currentState == END_MATCH) {
      stopMotors();
      return;
    }
    delay(WAYPOINT_TURN_SETTLE_MS);
  }

  Serial.print("Waypoint lookahead steering: ");
  Serial.print(distance, 3);
  Serial.print(" m remaining, lookahead ");
  Serial.print(WAYPOINT_LOOKAHEAD_M, 3);
  Serial.println(" m.");
  sendBluetoothEvent("waypoint_drive", "lookahead");

  resetEncodersAndPID();
  lastTime = millis();

  while (true) {
    if (!robotRunEnabled || handleBluetoothCommands()) {
      stopMotors();
      return;
    }

    updateTOFSensors();

    dx = targetX - robotX;
    dy = targetY - robotY;
    distance = sqrt(dx * dx + dy * dy);
    targetHeading = atan2(dy, dx) * 180.0 / PI;

    if (distance <= WAYPOINT_TOLERANCE_M) {
      stopMotors();
      updateOdometry();
      stuckRecoveryCount = 0;
      Serial.print("Pose after waypoint: ");
      printPose();
      return;
    }

    if (handleDrivePriorities(targetX, targetY)) {
      if (!robotRunEnabled || currentState == END_MATCH) {
        stopMotors();
        return;
      }
      resetEncodersAndPID();
      lastTime = millis();
      continue;
    }

    unsigned long now = millis();

    if (now - lastTime >= 100) {
      float dt = (now - lastTime) / 1000.0;

      long leftCount;
      long rightCount;
      readEncoderCounts(leftCount, rightCount);

      long dLeft = leftCount - lastLeftCount;
      long dRight = rightCount - lastRightCount;

      float leftSpeed = dLeft / dt;
      float rightSpeed = dRight / dt;

      updateOdometry();

      dx = targetX - robotX;
      dy = targetY - robotY;
      distance = sqrt(dx * dx + dy * dy);

      float lookaheadDistance = distance;
      if (lookaheadDistance > WAYPOINT_LOOKAHEAD_M) {
        lookaheadDistance = WAYPOINT_LOOKAHEAD_M;
      }

      float lookaheadX = targetX;
      float lookaheadY = targetY;

      if (distance > 0.001) {
        lookaheadX = robotX + (dx / distance) * lookaheadDistance;
        lookaheadY = robotY + (dy / distance) * lookaheadDistance;
      }

      targetHeading = atan2(lookaheadY - robotY, lookaheadX - robotX) * 180.0 / PI;

      float yaw = readYawDeg();
      float headingError = wrapAngle(targetHeading - yaw);
      float absHeadingError = fabs(headingError);

      if (absHeadingError > WAYPOINT_PIVOT_TURN_DEG && distance > WAYPOINT_FINAL_APPROACH_M) {
        stopMotors();
        Serial.print("Waypoint pivot: heading error ");
        Serial.print(headingError, 2);
        Serial.println(" deg too large for steering arc.");
        sendBluetoothEvent("waypoint_pivot", "heading");
        turnAngle(headingError);
        if (!robotRunEnabled || currentState == END_MATCH) {
          stopMotors();
          return;
        }
        resetEncodersAndPID();
        lastTime = millis();
        continue;
      }

      float headingScale = waypointHeadingForwardScale(absHeadingError);
      float distanceScale = waypointDistanceForwardScale(distance);
      float forwardScale = min(headingScale, distanceScale);

      float forwardTargetSpeed = baseTargetSpeed * forwardScale;
      float headingCorrection = constrain(
        K_heading * WAYPOINT_STEER_GAIN_MULTIPLIER * headingError,
        -WAYPOINT_MAX_TURN_CORRECTION,
        WAYPOINT_MAX_TURN_CORRECTION
      );
      setMotionCommand(forwardTargetSpeed, headingCorrection);

      updateStuckDriving(forwardTargetSpeed + headingCorrection,
                          forwardTargetSpeed - headingCorrection,
                          leftSpeed, rightSpeed);

      if (handleStuckPriority()) {
        resetEncodersAndPID();
        lastTime = millis();
        continue;
      }

      int leftForwardUs = (int)((leftForwardBaseUs - STOP_US) * forwardScale);
      int rightForwardUs = (int)((rightForwardBaseUs - STOP_US) * forwardScale);
      int turnUs = (int)constrain(
        headingError * WAYPOINT_TURN_US_PER_DEG,
        -WAYPOINT_MAX_TURN_US,
        WAYPOINT_MAX_TURN_US
      );

      int leftCommand = STOP_US + leftForwardUs + turnUs;
      int rightCommand = STOP_US + rightForwardUs - turnUs;

      writeMotorUS(leftCommand, rightCommand);

      if (DEBUG_DRIVE) {
        Serial.print("Waypoint steer  Dist: ");
        Serial.print(distance, 3);
        Serial.print("  Lookahead: ");
        Serial.print(lookaheadDistance, 3);
        Serial.print("  Target heading: ");
        Serial.print(targetHeading, 2);
        Serial.print("  Yaw: ");
        Serial.print(yaw);
        Serial.print("  H err: ");
        Serial.print(headingError);
        Serial.print("  F scale: ");
        Serial.print(forwardScale, 2);
        Serial.print("  Turn us: ");
        Serial.print(turnUs);
        Serial.print("  X: ");
        Serial.print(robotX, 3);
        Serial.print("  Y: ");
        Serial.print(robotY, 3);
        Serial.print("  L speed: ");
        Serial.print(leftSpeed);
        Serial.print("  R speed: ");
        Serial.print(rightSpeed);
        Serial.print("  Drive stuck: ");
        Serial.print(driveStuck);
        Serial.print("  Mismatch: ");
        Serial.print(wheelMismatchStuck);
        Serial.print("  L cmd: ");
        Serial.print(leftCommand);
        Serial.print("  R cmd: ");
        Serial.println(rightCommand);
      }

      lastLeftCount = leftCount;
      lastRightCount = rightCount;
      lastTime = now;
    }
  }
}

void runWaypointAction(const char* action) {
  if (!robotRunEnabled || currentState == END_MATCH) {
    stopMotors();
    return;
  }

  Serial.print("Waypoint action: ");
  Serial.println(action);

  if (strcmp(action, "PAUSE") == 0) {
    stopMotors();
    delay(WAYPOINT_ACTION_PAUSE_MS);
  } else if (strcmp(action, "HOME") == 0) {
    stopMotors();
    Serial.println("Reached home waypoint.");
    delay(WAYPOINT_HOME_PAUSE_MS);
  }
}
