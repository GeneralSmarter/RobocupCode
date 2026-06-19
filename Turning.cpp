#include "Robot.h"

// =====================================================
// Turning
// =====================================================
void turnAngle(float relativeTurnDeg) {
  float startYaw = readYawDeg();
  float targetYaw = wrapAngle(startYaw + relativeTurnDeg);

  resetTurnStuckCheck(startYaw);

  Serial.println();
  Serial.print("Turning ");
  Serial.print(relativeTurnDeg, 2);
  Serial.print(" deg. Target yaw: ");
  Serial.println(targetYaw, 2);

  while (true) {
    if (handleBluetoothCommands()) {
      return;
    }

    float yaw = readYawDeg();
    float error = wrapAngle(targetYaw - yaw);
    float absError = fabs(error);
    setMotionCommand(0.0, error);

    updateStuckTurning(yaw);

    if (handleStuckPriority()) {
      return;
    }

    if (DEBUG_TURN) {
      Serial.print("Yaw: ");
      Serial.print(yaw);
      Serial.print("  Turn error: ");
      Serial.println(error);
    }

    if (absError <= TURN_TOLERANCE_DEG) {
      stopMotors();
      delay(TURN_SETTLE_MS);

      robotTheta = readYawDeg();

      Serial.println("Turn complete.");
      Serial.print("Final yaw: ");
      Serial.println(robotTheta);
      printPose();

      return;
    }

    if (error > 0) {
      if (absError < SLOW_ZONE_DEG) {
        turnRightSlow();
      } else {
        turnRightFast();
      }
    } else {
      if (absError < SLOW_ZONE_DEG) {
        turnLeftSlow();
      } else {
        turnLeftFast();
      }
    }

    delay(50);
  }
}

void turnAngleNoStuckCheck(float relativeTurnDeg) {
  float startYaw = readYawDeg();
  float targetYaw = wrapAngle(startYaw + relativeTurnDeg);

  Serial.println();
  Serial.print("Recovery turning ");
  Serial.print(relativeTurnDeg, 2);
  Serial.print(" deg. Target yaw: ");
  Serial.println(targetYaw, 2);

  unsigned long turnStartMs = millis();
  const unsigned long RECOVERY_TURN_TIMEOUT_MS = 2500;

  while (true) {
    if (handleBluetoothCommands()) {
      return;
    }

    float yaw = readYawDeg();
    float error = wrapAngle(targetYaw - yaw);
    float absError = fabs(error);
    setMotionCommand(0.0, error);

    if (absError <= TURN_TOLERANCE_DEG) {
      stopMotors();
      delay(300);

      robotTheta = readYawDeg();

      Serial.println("Recovery turn complete.");
      printPose();
      return;
    }

    if (millis() - turnStartMs > RECOVERY_TURN_TIMEOUT_MS) {
      stopMotors();
      robotTheta = readYawDeg();
      Serial.println("Recovery turn timeout. Continuing recovery.");
      printPose();
      return;
    }

    if (error > 0) {
      turnRightSlow();
    } else {
      turnLeftSlow();
    }

    delay(50);
  }
}

void turnRightFast() {
  writeMotorUS(TURN_RIGHT_LEFT_FAST_US, TURN_RIGHT_RIGHT_FAST_US);
}

void turnRightSlow() {
  writeMotorUS(TURN_RIGHT_LEFT_SLOW_US, TURN_RIGHT_RIGHT_SLOW_US);
}

void turnLeftFast() {
  writeMotorUS(TURN_LEFT_LEFT_FAST_US, TURN_LEFT_RIGHT_FAST_US);
}

void turnLeftSlow() {
  writeMotorUS(TURN_LEFT_LEFT_SLOW_US, TURN_LEFT_RIGHT_SLOW_US);
}
