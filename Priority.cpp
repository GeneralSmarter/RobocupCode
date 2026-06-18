#include "Robot.h"

// =====================================================
// Priority checks
// =====================================================
bool handleEmergencyStopPriority(const char* contextLabel) {
  if (!isRangeSensorBlocked(RANGE_FRONT)) {
    return false;
  }

  stopMotors();
  updateOdometry();

  Serial.print("PRIORITY 1 EMERGENCY_STOP active during ");
  Serial.print(contextLabel);
  Serial.print(". Front distance: ");
  Serial.print(getRangeSensorDistance(RANGE_FRONT));
  Serial.println(" mm");

  return true;
}

bool handleStuckPriority() {
  if (inRecovery) {
    return false;
  }

  if (driveStuck || wheelMismatchStuck || turnStuck) {
    Serial.println("PRIORITY 2 STUCK_RECOVERY active.");
    runStuckRecovery();
    return true;
  }

  return false;
}

bool handleObstacleAvoidPriority(float originalPathHeadingDeg) {
  if (!isRangeSensorBlocked(RANGE_FRONT)) {
    return false;
  }

  Serial.println("PRIORITY 3 OBSTACLE_AVOID active.");
  runObstacleAvoidance(originalPathHeadingDeg);

  if (!stoppedSafely) {
    setRobotState(FOLLOW_PATH);
  }

  return true;
}

bool handleReturnHomePriority() {
  if (!returnHomeRequested) {
    return false;
  }

  if (currentState == RETURN_HOME || currentState == END_MATCH) {
    return false;
  }

  Serial.println("PRIORITY 4 RETURN_HOME active.");
  setRobotState(RETURN_HOME);
  return true;
}

bool handleDrivePriorities(float originalPathHeadingDeg) {
  if (handleEmergencyStopPriority("normal drive")) {
    handleObstacleAvoidPriority(originalPathHeadingDeg);
    return true;
  }

  if (handleStuckPriority()) {
    return true;
  }

  if (handleReturnHomePriority()) {
    return true;
  }

  return false;
}
