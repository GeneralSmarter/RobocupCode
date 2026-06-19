#include "Robot.h"

// =====================================================
// Side wall following fallback
// =====================================================
bool isValidWallFollowReading(uint16_t distanceMm) {
  return distanceMm >= WALL_FOLLOW_VALID_MIN_MM && distanceMm <= WALL_FOLLOW_VALID_MAX_MM;
}

WallFollowSide chooseWallFollowSide() {
  bool leftValid = isRangeSensorValid(RANGE_LEFT) && isValidWallFollowReading(getRangeSensorDistance(RANGE_LEFT));
  bool rightValid = isRangeSensorValid(RANGE_RIGHT) && isValidWallFollowReading(getRangeSensorDistance(RANGE_RIGHT));

  if (!leftValid && !rightValid) {
    return FOLLOW_NO_WALL;
  }

  if (leftValid && !rightValid) {
    return FOLLOW_LEFT_WALL;
  }

  if (!leftValid && rightValid) {
    return FOLLOW_RIGHT_WALL;
  }

  if (getRangeSensorDistance(RANGE_LEFT) <= getRangeSensorDistance(RANGE_RIGHT)) {
    return FOLLOW_LEFT_WALL;
  }

  return FOLLOW_RIGHT_WALL;
}

float getWallFollowCorrection(WallFollowSide side) {
  if (side == FOLLOW_LEFT_WALL && isRangeSensorValid(RANGE_LEFT) && isValidWallFollowReading(getRangeSensorDistance(RANGE_LEFT))) {
    float error = WALL_FOLLOW_TARGET_MM - getRangeSensorDistance(RANGE_LEFT);
    return constrain(WALL_FOLLOW_K * error, -WALL_FOLLOW_MAX_CORRECTION, WALL_FOLLOW_MAX_CORRECTION);
  }

  if (side == FOLLOW_RIGHT_WALL && isRangeSensorValid(RANGE_RIGHT) && isValidWallFollowReading(getRangeSensorDistance(RANGE_RIGHT))) {
    float error = getRangeSensorDistance(RANGE_RIGHT) - WALL_FOLLOW_TARGET_MM;
    return constrain(WALL_FOLLOW_K * error, -WALL_FOLLOW_MAX_CORRECTION, WALL_FOLLOW_MAX_CORRECTION);
  }

  return 0.0;
}

void driveDistanceWithHeadingWallFallback(float distanceMetres, float targetHeadingDeg) {
  resetEncodersAndPID();
  updateTOFSensors();

  WallFollowSide followSide = chooseWallFollowSide();

  if (followSide == FOLLOW_NO_WALL) {
    Serial.println("SIDE_WALL_FOLLOW: no valid side wall. Using heading-only bypass.");
    driveDistanceWithHeadingNoAvoid(distanceMetres, targetHeadingDeg);
    return;
  }

  long targetTicks = distanceMetres * TICKS_PER_METRE;
  unsigned long wallFollowStartMs = millis();

  Serial.print("SIDE_WALL_FOLLOW: bypass using ");
  if (followSide == FOLLOW_LEFT_WALL) {
    Serial.println("left wall.");
  } else {
    Serial.println("right wall.");
  }

  lastTime = millis();

  while (true) {
    if (handleBluetoothCommands()) {
      return;
    }

    updateTOFSensors();

    if (handleEmergencyStopPriority("side wall follow")) {
      Serial.println("SIDE_WALL_FOLLOW stopped because front became blocked again.");
      delay(200);
      return;
    }

    if (millis() - wallFollowStartMs > WALL_FOLLOW_TIMEOUT_MS) {
      stopMotors();
      updateOdometry();
      Serial.println("SIDE_WALL_FOLLOW timeout. Resuming waypoint.");
      delay(200);
      return;
    }

    long leftCount;
    long rightCount;
    readEncoderCounts(leftCount, rightCount);

    long avgTicks = (leftCount + rightCount) / 2;

    if (avgTicks >= targetTicks) {
      stopMotors();
      updateOdometry();
      delay(200);
      return;
    }

    unsigned long now = millis();

    if (now - lastTime >= 100) {
      float dt = (now - lastTime) / 1000.0;

      readEncoderCounts(leftCount, rightCount);

      long dLeft = leftCount - lastLeftCount;
      long dRight = rightCount - lastRightCount;

      float leftSpeed = dLeft / dt;
      float rightSpeed = dRight / dt;

      updateStuckDriving(baseTargetSpeed, leftSpeed, rightSpeed);

      if (handleStuckPriority()) {
        return;
      }

      updateOdometry();

      float yaw = readYawDeg();
      float headingError = wrapAngle(targetHeadingDeg - yaw);
      float headingCorrection = K_heading * headingError;
      setMotionCommand(baseTargetSpeed, headingCorrection);
      float wallCorrection = getWallFollowCorrection(followSide);
      float totalCorrection = constrain(headingCorrection + wallCorrection, -900.0, 900.0);

      float leftTargetSpeed  = baseTargetSpeed + totalCorrection;
      float rightTargetSpeed = baseTargetSpeed - totalCorrection;

      leftTargetSpeed  = constrain(leftTargetSpeed, 0.0, 3000.0);
      rightTargetSpeed = constrain(rightTargetSpeed, 0.0, 3000.0);

      int leftCommand = updatePID(leftTargetSpeed, leftSpeed, leftIntegral, lastLeftError, dt, leftForwardBaseUs);
      int rightCommand = updatePID(rightTargetSpeed, rightSpeed, rightIntegral, lastRightError, dt, rightForwardBaseUs);

      writeMotorUS(leftCommand, rightCommand);

      if (DEBUG_DRIVE) {
        Serial.print("SIDE_WALL_FOLLOW  Avg ticks: ");
        Serial.print(avgTicks);
        Serial.print("  Front: ");
        Serial.print(frontDistance);
        Serial.print(" mm  Left: ");
        Serial.print(leftDistance);
        Serial.print(" mm  Right: ");
        Serial.print(rightDistance);
        Serial.print(" mm  H corr: ");
        Serial.print(headingCorrection);
        Serial.print("  W corr: ");
        Serial.print(wallCorrection);
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

// =====================================================
// Forward driving
// =====================================================
void driveDistanceWithHeading(float distanceMetres, float targetHeadingDeg) {
  resetEncodersAndPID();

  long targetTicks = distanceMetres * TICKS_PER_METRE;

  Serial.println();
  Serial.print("Driving ");
  Serial.print(distanceMetres, 3);
  Serial.print(" m at heading ");
  Serial.println(targetHeadingDeg, 2);

  lastTime = millis();

  while (true) {
    if (handleBluetoothCommands()) {
      return;
    }

    updateTOFSensors();

    if (handleDrivePriorities(targetHeadingDeg)) {
      return;
    }

    long leftCount;
    long rightCount;
    readEncoderCounts(leftCount, rightCount);

    long avgTicks = (leftCount + rightCount) / 2;

    if (avgTicks >= targetTicks) {
      stopMotors();
      updateOdometry();

      Serial.println("Drive complete.");
      Serial.print("Final avg ticks: ");
      Serial.println(avgTicks);
      Serial.print("Final yaw: ");
      Serial.println(readYawDeg());
      printPose();

      delay(200);
      return;
    }

    unsigned long now = millis();

    if (now - lastTime >= 100) {
      float dt = (now - lastTime) / 1000.0;

      readEncoderCounts(leftCount, rightCount);
      avgTicks = (leftCount + rightCount) / 2;

      long dLeft = leftCount - lastLeftCount;
      long dRight = rightCount - lastRightCount;

      float leftSpeed = dLeft / dt;
      float rightSpeed = dRight / dt;

      updateStuckDriving(baseTargetSpeed, leftSpeed, rightSpeed);

      if (handleStuckPriority()) {
        return;
      }

      updateOdometry();

      float yaw = readYawDeg();
      float headingError = wrapAngle(targetHeadingDeg - yaw);
      float headingCorrection = K_heading * headingError;
      setMotionCommand(baseTargetSpeed, headingCorrection);

      float leftTargetSpeed  = baseTargetSpeed + headingCorrection;
      float rightTargetSpeed = baseTargetSpeed - headingCorrection;

      leftTargetSpeed  = constrain(leftTargetSpeed, 0.0, 3000.0);
      rightTargetSpeed = constrain(rightTargetSpeed, 0.0, 3000.0);

      int leftCommand = updatePID(leftTargetSpeed, leftSpeed, leftIntegral, lastLeftError, dt, leftForwardBaseUs);
      int rightCommand = updatePID(rightTargetSpeed, rightSpeed, rightIntegral, lastRightError, dt, rightForwardBaseUs);

      writeMotorUS(leftCommand, rightCommand);

      if (DEBUG_DRIVE) {
        Serial.print("Avg ticks: ");
        Serial.print(avgTicks);
        Serial.print("  Front: ");
        Serial.print(frontDistance);
        Serial.print(" mm  Left: ");
        Serial.print(leftDistance);
        Serial.print(" mm  Right: ");
        Serial.print(rightDistance);
        Serial.print(" mm  Yaw: ");
        Serial.print(yaw);
        Serial.print("  H err: ");
        Serial.print(headingError);
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

void driveDistanceWithHeadingNoAvoid(float distanceMetres, float targetHeadingDeg) {
  resetEncodersAndPID();

  long targetTicks = distanceMetres * TICKS_PER_METRE;

  Serial.print("Bypass forward: ");
  Serial.print(distanceMetres, 3);
  Serial.print(" m at heading ");
  Serial.println(targetHeadingDeg, 2);

  lastTime = millis();

  while (true) {
    if (handleBluetoothCommands()) {
      return;
    }

    updateTOFSensors();

    if (handleEmergencyStopPriority("bypass drive")) {
      Serial.println("Bypass stopped because front became blocked again.");
      delay(200);
      return;
    }

    long leftCount;
    long rightCount;
    readEncoderCounts(leftCount, rightCount);

    long avgTicks = (leftCount + rightCount) / 2;

    if (avgTicks >= targetTicks) {
      stopMotors();
      updateOdometry();
      delay(200);
      return;
    }

    unsigned long now = millis();

    if (now - lastTime >= 100) {
      float dt = (now - lastTime) / 1000.0;

      readEncoderCounts(leftCount, rightCount);

      long dLeft = leftCount - lastLeftCount;
      long dRight = rightCount - lastRightCount;

      float leftSpeed = dLeft / dt;
      float rightSpeed = dRight / dt;

      updateStuckDriving(baseTargetSpeed, leftSpeed, rightSpeed);

      if (handleStuckPriority()) {
        return;
      }

      updateOdometry();

      float yaw = readYawDeg();
      float headingError = wrapAngle(targetHeadingDeg - yaw);
      float headingCorrection = K_heading * headingError;
      setMotionCommand(baseTargetSpeed, headingCorrection);

      float leftTargetSpeed  = baseTargetSpeed + headingCorrection;
      float rightTargetSpeed = baseTargetSpeed - headingCorrection;

      leftTargetSpeed  = constrain(leftTargetSpeed, 0.0, 3000.0);
      rightTargetSpeed = constrain(rightTargetSpeed, 0.0, 3000.0);

      int leftCommand = updatePID(leftTargetSpeed, leftSpeed, leftIntegral, lastLeftError, dt, leftForwardBaseUs);
      int rightCommand = updatePID(rightTargetSpeed, rightSpeed, rightIntegral, lastRightError, dt, rightForwardBaseUs);

      writeMotorUS(leftCommand, rightCommand);

      lastLeftCount = leftCount;
      lastRightCount = rightCount;
      lastTime = now;
    }
  }
}
