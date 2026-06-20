#include "Robot.h"

// =====================================================
// Obstacle avoidance
// =====================================================
static float getAvoidTurnSweepRadiusMm();
static bool restoreDiagonalClearance(AvoidTurnChoice turnChoice,
                                    unsigned long startTime);

static bool planAdaptiveBypass(const AvoidSideClearance &sideClearance,
                                float targetX, float targetY,
                                float &bypassDistanceM,
                                float &targetHeadingDeg) {
  float dx = targetX - robotX;
  float dy = targetY - robotY;
  float remainingDistanceM = sqrtf(dx * dx + dy * dy);
  targetHeadingDeg = atan2(dy, dx) * 180.0 / PI;

  const float turnSin = fabs(sinf(AVOID_TURN_ANGLE_DEG * DEG_TO_RAD));
  if (!sideClearance.passable || turnSin < 0.01 || remainingDistanceM <= WAYPOINT_TOLERANCE_M) {
    return false;
  }

  const float requiredLateralOffsetM =
    (getAvoidTurnSweepRadiusMm() + AVOID_CLEARANCE_MARGIN_MM) / 1000.0;
  const float footprintBypassM = requiredLateralOffsetM / turnSin;
  const float targetLimitedBypassM = remainingDistanceM * AVOID_TARGET_BYPASS_FRACTION;
  // The inner ray can continue to see the obstacle edge while the robot turns
  // into a usable side gap. Keep it in the initial swept-turn check above, but
  // use the outward ray to decide whether there is room to travel the bypass.
  const float outerTravelClearanceM =
    (sideClearance.outerSweepClearanceMm - AVOID_CLEARANCE_MARGIN_MM) / 1000.0;

  bypassDistanceM = min(footprintBypassM, targetLimitedBypassM);
  bypassDistanceM = constrain(bypassDistanceM,
                              AVOID_MIN_BYPASS_DISTANCE_M,
                              AVOID_MAX_BYPASS_DISTANCE_M);

  return outerTravelClearanceM >= bypassDistanceM;
}

void runObstacleAvoidance(float targetX, float targetY, const char* trigger) {
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
  sendBluetoothEvent("obstacle_avoid_start", trigger);

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

  if (turnChoice == AVOID_TURN_NONE) {
    Serial.println("Avoidance has no footprint-safe turn corridor. Starting stuck recovery.");
    stopMotors();
    sendBluetoothEvent("obstacle_avoid_end", "no_safe_corridor");
    runStuckRecovery();
    return;
  }

  AvoidSideClearance leftClearance;
  AvoidSideClearance rightClearance;
  const char* clearanceReason = "";
  evaluateAvoidTurnDirection(leftClearance, rightClearance, clearanceReason);
  const AvoidSideClearance &selectedClearance =
    turnChoice == AVOID_TURN_LEFT ? leftClearance : rightClearance;

  float bypassDistanceM = 0.0;
  float targetHeadingDeg = 0.0;
  if (!planAdaptiveBypass(selectedClearance, targetX, targetY,
                           bypassDistanceM, targetHeadingDeg)) {
    Serial.println("Avoidance has no outward clearance for the required adaptive bypass. Starting stuck recovery.");
    stopMotors();
    sendBluetoothEvent("obstacle_avoid_end", "insufficient_bypass_clearance");
    runStuckRecovery();
    return;
  }

  char planDetail[96];
  snprintf(planDetail, sizeof(planDetail), "%s;%s;bypass=%.3f;score=%.0f;outer=%.0f;target=%.1f",
           turnChoice == AVOID_TURN_LEFT ? "left" : "right",
           clearanceReason, bypassDistanceM, selectedClearance.scoreMm,
           selectedClearance.outerSweepClearanceMm, targetHeadingDeg);
  sendBluetoothEvent("adaptive_bypass_plan", planDetail);
  Serial.print("Adaptive bypass plan: ");
  Serial.println(planDetail);

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

  if (!restoreDiagonalClearance(turnChoice, startTime)) {
    Serial.println("Avoidance could not establish diagonal footprint clearance. Starting stuck recovery.");
    stopMotors();
    sendBluetoothEvent("obstacle_avoid_end", "diagonal_clearance_unresolved");
    runStuckRecovery();
    return;
  }

  waitForFrontClear(FRONT_CLEAR_SETTLE_TIMEOUT_MS);
  if (!robotRunEnabled || currentState == END_MATCH) {
    sendBluetoothEvent("obstacle_avoid_end", "aborted");
    return;
  }

  if (!isRangeSensorBlocked(RANGE_FRONT)) {
    float bypassHeading = readYawDeg();

    if (ENABLE_SIDE_WALL_FOLLOW_FALLBACK) {
      driveDistanceWithHeadingWallFallback(bypassDistanceM, bypassHeading);
    } else {
      driveDistanceWithHeadingNoAvoid(bypassDistanceM, bypassHeading);
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

  float targetRejoinHeadingDeg = atan2(targetY - robotY, targetX - robotX) * 180.0 / PI;
  float currentHeadingDeg = readYawDeg();
  float targetHeadingErrorDeg = wrapAngle(targetRejoinHeadingDeg - currentHeadingDeg);
  float handoffHeadingDeg = targetRejoinHeadingDeg;

  if (fabs(targetHeadingErrorDeg) > AVOID_REJOIN_HANDOFF_MAX_ERROR_DEG) {
    handoffHeadingDeg = targetRejoinHeadingDeg -
      (targetHeadingErrorDeg > 0.0 ? AVOID_REJOIN_HANDOFF_MAX_ERROR_DEG
                                    : -AVOID_REJOIN_HANDOFF_MAX_ERROR_DEG);
  }

  float handoffTurnDeg = wrapAngle(handoffHeadingDeg - currentHeadingDeg);
  Serial.print("Adaptive rejoin: target/handoff/turn=");
  Serial.print(targetRejoinHeadingDeg, 2);
  Serial.print("/");
  Serial.print(handoffHeadingDeg, 2);
  Serial.print("/");
  Serial.print(handoffTurnDeg, 2);
  Serial.println(" deg.");

  char rejoinDetail[80];
  snprintf(rejoinDetail, sizeof(rejoinDetail), "target=%.1f;handoff=%.1f;turn=%.1f",
           targetRejoinHeadingDeg, handoffHeadingDeg, handoffTurnDeg);
  sendBluetoothEvent("adaptive_rejoin", rejoinDetail);

  if (fabs(handoffTurnDeg) > TURN_TOLERANCE_DEG) {
    turnAngle(handoffTurnDeg);
    if (!robotRunEnabled || currentState == END_MATCH) {
      sendBluetoothEvent("obstacle_avoid_end", "aborted");
      return;
    }
  }

  Serial.println("OBSTACLE_AVOID complete. Resuming waypoint.");
  printPose();
  sendBluetoothEvent("obstacle_avoid_end", "complete");
}

static bool restoreDiagonalClearance(AvoidTurnChoice turnChoice,
                                    unsigned long startTime) {
  for (int attempt = 0; attempt <= AVOID_ESCAPE_MAX_CLEARANCE_ATTEMPTS; attempt++) {
    updateTOFSensors();

    RangeSensorId sensorId;
    float clearanceMm = 0.0;
    if (!getDiagonalClearanceWarning(sensorId, clearanceMm)) {
      if (attempt > 0) {
        sendBluetoothEvent("diagonal_clearance_restored", "escape_complete");
      }
      return true;
    }

    Serial.print("DIAGONAL CLEARANCE warning: ");
    Serial.print(rangeSensors[sensorId].name);
    Serial.print(" sweep clearance ");
    Serial.print(clearanceMm, 1);
    Serial.println(" mm.");

    char detail[64];
    snprintf(detail, sizeof(detail), "%s;clearance=%.0f;attempt=%d",
             rangeSensors[sensorId].name, clearanceMm, attempt + 1);
    sendBluetoothEvent("diagonal_clearance_warning", detail);

    if (attempt == AVOID_ESCAPE_MAX_CLEARANCE_ATTEMPTS ||
        millis() - startTime > AVOID_TIMEOUT_MS) {
      break;
    }

    Serial.print("Clearance escape: reverse ");
    Serial.print(AVOID_ESCAPE_REVERSE_STEP_M, 3);
    Serial.print(" m, then turn ");
    Serial.print(AVOID_ESCAPE_TURN_STEP_DEG, 1);
    Serial.println(" deg farther toward the selected side.");

    reverseDistanceOpenLoop(AVOID_ESCAPE_REVERSE_STEP_M);
    if (!robotRunEnabled || currentState == END_MATCH) {
      return false;
    }

    const float signedTurnDeg = turnChoice == AVOID_TURN_LEFT
                                  ? -AVOID_ESCAPE_TURN_STEP_DEG
                                  : AVOID_ESCAPE_TURN_STEP_DEG;
    turnAngle(signedTurnDeg);
    if (!robotRunEnabled || currentState == END_MATCH) {
      return false;
    }
  }

  return false;
}

static float getAvoidTurnSweepRadiusMm() {
  const float frontLeft = sqrtf(ROBOT_FOOTPRINT_GEOMETRY.frontExtentMm * ROBOT_FOOTPRINT_GEOMETRY.frontExtentMm +
                                ROBOT_FOOTPRINT_GEOMETRY.leftExtentMm * ROBOT_FOOTPRINT_GEOMETRY.leftExtentMm);
  const float frontRight = sqrtf(ROBOT_FOOTPRINT_GEOMETRY.frontExtentMm * ROBOT_FOOTPRINT_GEOMETRY.frontExtentMm +
                                 ROBOT_FOOTPRINT_GEOMETRY.rightExtentMm * ROBOT_FOOTPRINT_GEOMETRY.rightExtentMm);
  const float rearLeft = sqrtf(ROBOT_FOOTPRINT_GEOMETRY.rearExtentMm * ROBOT_FOOTPRINT_GEOMETRY.rearExtentMm +
                               ROBOT_FOOTPRINT_GEOMETRY.leftExtentMm * ROBOT_FOOTPRINT_GEOMETRY.leftExtentMm);
  const float rearRight = sqrtf(ROBOT_FOOTPRINT_GEOMETRY.rearExtentMm * ROBOT_FOOTPRINT_GEOMETRY.rearExtentMm +
                                ROBOT_FOOTPRINT_GEOMETRY.rightExtentMm * ROBOT_FOOTPRINT_GEOMETRY.rightExtentMm);
  return max(max(frontLeft, frontRight), max(rearLeft, rearRight));
}

static float getSensorSweepClearanceMm(RangeSensorId id, float sweepRadiusMm) {
  const FanSensorGeometry &sensor = FAN_SENSOR_GEOMETRY[(int)id];
  const float rangeMm = (float)getRangeSensorDistance(id);
  const float angleRad = sensor.angleDeg * DEG_TO_RAD;
  const float endpointX = sensor.xMm + rangeMm * cosf(angleRad);
  const float endpointY = sensor.yMm + rangeMm * sinf(angleRad);
  return sqrtf(endpointX * endpointX + endpointY * endpointY) - sweepRadiusMm;
}

static void assessAvoidSideClearance(AvoidTurnChoice choice,
                                     AvoidSideClearance &assessment) {
  const RangeSensorId innerId = choice == AVOID_TURN_LEFT ? RANGE_LEFT_INNER : RANGE_RIGHT_INNER;
  const RangeSensorId outerId = choice == AVOID_TURN_LEFT ? RANGE_LEFT_OUTER : RANGE_RIGHT_OUTER;

  assessment.valid = isRangeSensorValid(innerId) && isRangeSensorValid(outerId);
  assessment.passable = false;
  assessment.innerSweepClearanceMm = -1.0;
  assessment.outerSweepClearanceMm = -1.0;
  assessment.scoreMm = -1.0;

  if (!assessment.valid) {
    return;
  }

  const float sweepRadiusMm = getAvoidTurnSweepRadiusMm();
  assessment.innerSweepClearanceMm = getSensorSweepClearanceMm(innerId, sweepRadiusMm);
  assessment.outerSweepClearanceMm = getSensorSweepClearanceMm(outerId, sweepRadiusMm);
  assessment.scoreMm = min(assessment.innerSweepClearanceMm, assessment.outerSweepClearanceMm);
  assessment.passable = assessment.scoreMm >= AVOID_CLEARANCE_MARGIN_MM;
}

AvoidTurnChoice evaluateAvoidTurnDirection(AvoidSideClearance &left,
                                           AvoidSideClearance &right,
                                           const char* &reason) {
  assessAvoidSideClearance(AVOID_TURN_LEFT, left);
  assessAvoidSideClearance(AVOID_TURN_RIGHT, right);

  if (left.passable && !right.passable) {
    reason = "only_left_footprint_safe";
    return AVOID_TURN_LEFT;
  }

  if (!left.passable && right.passable) {
    reason = "only_right_footprint_safe";
    return AVOID_TURN_RIGHT;
  }

  if (!left.passable && !right.passable) {
    reason = (!left.valid && !right.valid) ? "no_valid_side_corridor" : "no_footprint_safe_corridor";
    return AVOID_TURN_NONE;
  }

  if (left.scoreMm > right.scoreMm + AVOID_SCORE_TIE_MARGIN_MM) {
    reason = "left_more_footprint_clearance";
    return AVOID_TURN_LEFT;
  }

  if (right.scoreMm > left.scoreMm + AVOID_SCORE_TIE_MARGIN_MM) {
    reason = "right_more_footprint_clearance";
    return AVOID_TURN_RIGHT;
  }

  reason = "similar_footprint_clearance_default_right";
  return AVOID_TURN_RIGHT;
}

AvoidTurnChoice chooseAvoidTurnDirection() {
  AvoidSideClearance left;
  AvoidSideClearance right;
  const char* reason = "";
  AvoidTurnChoice choice = evaluateAvoidTurnDirection(left, right, reason);

  Serial.print("Avoid side check: left=");
  Serial.print(getRangeSensorDistance(RANGE_LEFT));
  Serial.print(left.valid ? " valid" : " invalid");
  Serial.print(" mm  right=");
  Serial.print(getRangeSensorDistance(RANGE_RIGHT));
  Serial.print(right.valid ? " valid" : " invalid");
  Serial.println(" mm");

  Serial.print("Avoid footprint clearance: left inner/outer/score=");
  Serial.print(left.innerSweepClearanceMm, 1);
  Serial.print("/");
  Serial.print(left.outerSweepClearanceMm, 1);
  Serial.print("/");
  Serial.print(left.scoreMm, 1);
  Serial.print(" mm  right inner/outer/score=");
  Serial.print(right.innerSweepClearanceMm, 1);
  Serial.print("/");
  Serial.print(right.outerSweepClearanceMm, 1);
  Serial.print("/");
  Serial.print(right.scoreMm, 1);
  Serial.println(" mm");

  Serial.print("Avoid reason: ");
  Serial.println(reason);
  return choice;
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
