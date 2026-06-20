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

static bool handleDiagonalClearancePriority(float targetX, float targetY) {
  RangeSensorId sensorId;
  float clearanceMm = 0.0;
  if (!getDiagonalClearanceWarning(sensorId, clearanceMm)) {
    return false;
  }

  stopMotors();
  updateOdometry();

  Serial.print("PRIORITY 1 DIAGONAL_CLEARANCE warning from ");
  Serial.print(rangeSensors[sensorId].name);
  Serial.print(": ");
  Serial.print(clearanceMm, 1);
  Serial.println(" mm. Starting clearance escape.");

  char detail[64];
  snprintf(detail, sizeof(detail), "%s;clearance=%.0f", rangeSensors[sensorId].name, clearanceMm);
  sendBluetoothEvent("diagonal_clearance_warning", detail);
  runObstacleAvoidance(targetX, targetY, "diagonal_clearance");

  if (!stoppedSafely) {
    setRobotState(FOLLOW_PATH);
  }

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

bool handleObstacleAvoidPriority(float targetX, float targetY) {
  if (!isRangeSensorBlocked(RANGE_FRONT)) {
    return false;
  }

  Serial.println("PRIORITY 3 OBSTACLE_AVOID active.");
  runObstacleAvoidance(targetX, targetY);

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

bool handleDrivePriorities(float targetX, float targetY) {
  if (handleEmergencyStopPriority("normal drive")) {
    handleObstacleAvoidPriority(targetX, targetY);
    return true;
  }

  if (handleDiagonalClearancePriority(targetX, targetY)) {
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
