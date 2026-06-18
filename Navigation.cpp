#include "Robot.h"

// =====================================================
// Waypoint navigation
// =====================================================
void goToPoint(float targetX, float targetY) {
  while (true) {
    if (!robotRunEnabled || handleBluetoothCommands()) {
      stopMotors();
      return;
    }

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

    if (fabs(turnNeeded) > TURN_TOLERANCE_DEG) {
      turnAngle(turnNeeded);
      if (!robotRunEnabled || currentState == END_MATCH) {
        stopMotors();
        return;
      }
      delay(300);
    }

    driveDistanceWithHeading(distance, targetHeading);
    if (!robotRunEnabled || currentState == END_MATCH) {
      stopMotors();
      return;
    }
    delay(300);
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
    delay(1000);
  } else if (strcmp(action, "HOME") == 0) {
    stopMotors();
    Serial.println("Reached home waypoint.");
    delay(1000);
  }
}
