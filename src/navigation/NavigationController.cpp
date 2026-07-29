#include "../../Robot.h"
#include "NavigationInternal.h"
#include "NavigationRuntime.h"

// =====================================================
// Navigation goal lifecycle and local-planner orchestration
// =====================================================
// Responsibility:
//   Owns local perception-to-motion planning: rolling occupancy evidence,
//   persistent arena memory,
//   footprint collision checks, differential-drive arc rollout/scoring, point
//   and turn navigation goals, obstacle-local forward bypass, planner telemetry,
//   and the scheduled controller pipeline.
// Interacts with:
//   TofSensors.cpp supplies fan and rear range evidence. Odometry.cpp updates
//   robot pose. Mission code and Bluetooth create/cancel goals through the
//   public navigation interfaces.
//   MotorControl.cpp is the only physical motor writer and accepts commands
//   through the planner publish wrapper and the authorized motor-command API.
// Control flow:
//   updateNavigationRuntime() is called by RobotController.cpp. It schedules sensors/map,
//   odometry, updateNavigationController(), updateMotorController(), and
//   telemetry construction. Point goals use cooperative planner epochs; turn
//   goals use direct yaw feedback.
// Global state:
//   Modifies localMap, navigationGoal, plannerTelemetry, one obstacle-scoped
//   context, planner epochs, motorStopRequested/desired motion through safe
//   command APIs, and controller timing stamps.
// The rolling map preserves fresh dynamic evidence. A compact fixed-world
// layer remembers thresholded occupied and clear cells for the match while
// still allowing repeated contradictory observations to correct them.

#include "PlannerContext.h"
#include "PlannerCollision.h"
#include "ForwardTrajectoryPlanner.h"
#include "NavigationControllerInternal.h"
#include "PlannerMap.h"
#include "PlannerProgress.h"
#include "PlannerDebug.h"
#include "ObstacleContext.h"
#include "RecoveryPlanner.h"


bool setEmergencyScanPolicyEnabled(bool enabled) {
  if (navigationGoal.active) {
    return false;
  }
  plannerContext.emergencyScanPolicyEnabled = enabled;
  return true;
}

bool isEmergencyScanPolicyEnabled() {
  return plannerContext.emergencyScanPolicyEnabled;
}

void resetPlannerEpoch() {
  // Clears any in-progress forward planning epoch and resets planner timing
  // telemetry. It does not cancel the navigation goal itself.
  plannerContext.plannerEpoch.active = false;
  plannerContext.plannerEpoch.awaitingRevalidation = false;
  plannerContext.plannerEpoch.commandStoppedForAge = false;
  plannerTelemetry.plannerEpochActive = false;
  plannerTelemetry.plannerEpochAgeMs = 0;
  plannerTelemetry.plannerCandidatesProcessed = 0;
  plannerTelemetry.plannerYieldCount = 0;
  plannerTelemetry.plannerSliceUs = 0;
  plannerTelemetry.plannerSliceMaxUs = 0;
  plannerTelemetry.plannerEpochWorkUs = 0;
  plannerTelemetry.plannerEpochMaxWorkUs = 0;
  plannerTelemetry.plannerCommandAgeMs = 0;
}

void resetReversePlannerEpoch() {
  plannerContext.reversePlannerEpoch.active = false;
  plannerContext.reversePlannerEpoch.awaitingRevalidation = false;
  plannerContext.reversePlannerEpoch.commandStoppedForAge = false;
  if (!plannerContext.plannerEpoch.active) {
    plannerTelemetry.plannerEpochActive = false;
  }
}

void closePlannerEpoch() {
  // Marks the current epoch complete and records its age for telemetry. The
  // chosen command, if any, has already been published before this is called.
  plannerContext.plannerEpoch.active = false;
  plannerContext.plannerEpoch.awaitingRevalidation = false;
  plannerTelemetry.plannerEpochActive = false;
  plannerTelemetry.plannerEpochAgeMs = millis() - plannerContext.plannerEpoch.startedMs;
}

void resetGeometricNoPathEvidence() {
  plannerContext.noSafeTrajectorySinceMs = 0;
  plannerContext.geometricNoPathEpochCount = 0;
  plannerContext.lastForwardNoPathWasGeometric = false;
}

void noteGeometricNoPathEpoch() {
  // Counts consecutive no-path epochs that were caused by geometry/footprint
  // evidence rather than stale sensors, authority loss, or policy retries.
  unsigned long now = millis();
  if (plannerContext.geometricNoPathEpochCount == 0) {
    plannerContext.noSafeTrajectorySinceMs = now;
  }
  if (plannerContext.geometricNoPathEpochCount < 255) {
    plannerContext.geometricNoPathEpochCount++;
  }
}

bool currentPlannerFailureIsGeometricNoPath() {
  return plannerContext.lastForwardNoPathWasGeometric;
}

static bool areTurnSweepSensorsValid() {
  // A pivot sweeps every corner of the chassis through a circle, so it needs
  // all four fan sectors. Straight driving does not make this assumption.
  for (int i = RANGE_RIGHT_OUTER; i <= RANGE_LEFT_OUTER; i++) {
    if (!isRangeSensorValid((RangeSensorId)i)) {
      return false;
    }
  }
  return true;
}


bool publishNavigationMotion(float forwardSpeed, float turnSpeed) {
  // Final planner-to-motor handoff. Inputs are chassis speeds in ticks/s:
  // positive forward drives +X, positive turn is CCW/left. MotorControl.cpp
  // may still veto the command if live safety evidence changed.
  MotionCommandMode mode = MOTION_COMMAND_NAV_DRIVE;
  if (navigationGoal.mode == NAV_GOAL_TURN) {
    mode = navigationGoal.owner == NAV_OWNER_WEIGHT_SCAN
      ? MOTION_COMMAND_NAV_SCAN_TURN
      : MOTION_COMMAND_NAV_TURN;
  }
  if (setAuthorizedMotionCommand(navigationGoal.authority,
                                 forwardSpeed, turnSpeed, mode)) {
    return true;
  }

  // Keep the goal and its P0-02 authority intact so the planner can replan on
  // the next fresh snapshot. The final motor owner has already forced neutral.
  plannerTelemetry.safeStopReason =
    motionSafetyReasonName(lastMotionSafetyReason());
  plannerTelemetry.replanReason = "continuous_safety_veto";
  return false;
}





static bool obstacleProgressStalled(float localGoalX, float localGoalY,
                                    float localGoalDistanceM) {
  if (!plannerContext.obstacleContext.active) {
    return false;
  }

  unsigned long now = millis();
  float goalShiftM = plannerContext.obstacleContext.progressGoalValid
    ? hypotf(localGoalX - plannerContext.obstacleContext.progressGoalX,
             localGoalY - plannerContext.obstacleContext.progressGoalY)
    : 0.0f;
  if (!plannerContext.obstacleContext.progressGoalValid ||
      goalShiftM >= LOCAL_MAP_CELL_M) {
    plannerContext.obstacleContext.progressGoalValid = true;
    plannerContext.obstacleContext.progressGoalX = localGoalX;
    plannerContext.obstacleContext.progressGoalY = localGoalY;
    plannerContext.obstacleContext.progressStartDistanceM = localGoalDistanceM;
    plannerContext.obstacleContext.progressBestDistanceM = localGoalDistanceM;
    plannerContext.obstacleContext.progressLastMs = now;
  } else if (localGoalDistanceM <=
             plannerContext.obstacleContext.progressBestDistanceM -
               PLANNER_OBSTACLE_PROGRESS_EPSILON_M) {
    plannerContext.obstacleContext.progressBestDistanceM = localGoalDistanceM;
    plannerContext.obstacleContext.progressLastMs = now;
  }

  plannerTelemetry.obstacleProgressAgeS =
    (now - plannerContext.obstacleContext.progressLastMs) / 1000.0f;
  plannerTelemetry.obstacleBestProgressM = max(
    0.0f,
    plannerContext.obstacleContext.progressStartDistanceM -
      plannerContext.obstacleContext.progressBestDistanceM);
  return now - plannerContext.obstacleContext.progressLastMs >
    PLANNER_OBSTACLE_PROGRESS_TIMEOUT_MS;
}

static bool routeLineGoalReached(float routeLengthM, float routeUx, float routeUy,
                                 float routeHeadingRad) {
  float alongM = routeLineAlongM(robotX, robotY, routeUx, routeUy);
  float lateralErrorM = routeLineLateralErrorM(robotX, robotY, routeUx, routeUy);
  float headingErrorDeg = fabs(wrapAngle(routeHeadingRad * RAD_TO_DEG -
                                         navigationHeadingDeg()));
  return alongM >= routeLengthM - WAYPOINT_TOLERANCE_M &&
         alongM <= routeLengthM + PLANNER_LINE_FOLLOW_FINISH_OVERSHOOT_M &&
         lateralErrorM <= PLANNER_LINE_FOLLOW_FINISH_LATERAL_M &&
         headingErrorDeg <= PLANNER_LINE_FOLLOW_FINISH_HEADING_DEG;
}

SafePivotStepResult commandSafePivotStep(
    float turnTarget, const char* motionPlanReason,
    const char* sideRevalidatePlanReason,
    const char* sweepRevalidatePlanReason) {
  if (!isTurnDirectionObservable(turnTarget)) {
    unsigned long now = millis();
    if (plannerContext.turnSideInvalidSinceMs == 0) {
      plannerContext.turnSideInvalidSinceMs = now;
      sendBluetoothEvent("turn_side_revalidate",
                         "motors_stopped_for_fresh_sample");
    }
    motorStopRequested = true;
    requestMotionStop();
    if (now - plannerContext.turnSideInvalidSinceMs < PLANNER_TURN_SENSOR_REVALIDATE_MS) {
      plannerTelemetry.planReason = sideRevalidatePlanReason;
      return SAFE_PIVOT_STEP_REVALIDATING;
    }
    return SAFE_PIVOT_STEP_SIDE_INVALID;
  }
  plannerContext.turnSideInvalidSinceMs = 0;

  if (!areTurnSweepSensorsValid()) {
    unsigned long now = millis();
    if (plannerContext.turnSweepInvalidSinceMs == 0) {
      plannerContext.turnSweepInvalidSinceMs = now;
      sendBluetoothEvent("turn_sweep_revalidate",
                         "motors_stopped_for_fresh_sample");
    }
    motorStopRequested = true;
    requestMotionStop();
    if (now - plannerContext.turnSweepInvalidSinceMs < PLANNER_TURN_SENSOR_REVALIDATE_MS) {
      plannerTelemetry.planReason = sweepRevalidatePlanReason;
      return SAFE_PIVOT_STEP_REVALIDATING;
    }
    return SAFE_PIVOT_STEP_SWEEP_INVALID;
  }
  plannerContext.turnSweepInvalidSinceMs = 0;

  if (!isTurnSweepSafe()) {
    return SAFE_PIVOT_STEP_SWEEP_BLOCKED;
  }

  updateStuckTurning(navigationHeadingDeg());
  if (turnStuck) {
    return SAFE_PIVOT_STEP_STUCK;
  }

  plannerTelemetry.selectedForwardTicksPerSec = 0.0f;
  plannerTelemetry.selectedTurnTicksPerSec = turnTarget;
  plannerTelemetry.selectedCurvature = 0.0f;
  plannerTelemetry.minimumSweptClearanceMm = minimumFanSweepClearanceMm();
  plannerTelemetry.speedCapTicksPerSec = 0.0f;
  plannerTelemetry.candidateCount = 1;
  plannerTelemetry.stopReason = PLANNER_STOP_NONE;
  plannerTelemetry.planReason = motionPlanReason;
  motorStopRequested = false;
  return publishNavigationMotion(0.0f, turnTarget)
    ? SAFE_PIVOT_STEP_PUBLISHED
    : SAFE_PIVOT_STEP_PUBLICATION_VETOED;
}

static bool routeLineOvershootStopReached(float routeLengthM, float routeUx,
                                          float routeUy, float routeHeadingRad) {
  float alongM = routeLineAlongM(robotX, robotY, routeUx, routeUy);
  float lateralErrorM = routeLineLateralErrorM(robotX, robotY, routeUx, routeUy);
  float headingErrorDeg = fabs(wrapAngle(routeHeadingRad * RAD_TO_DEG -
                                         navigationHeadingDeg()));
  return alongM > routeLengthM + PLANNER_LINE_FOLLOW_FINISH_OVERSHOOT_M &&
         alongM <= routeLengthM + PLANNER_LINE_FOLLOW_MISSED_STOP_OVERSHOOT_M &&
         lateralErrorM <= PLANNER_LINE_FOLLOW_FINISH_LATERAL_M &&
         headingErrorDeg <= PLANNER_LINE_FOLLOW_ENABLE_HEADING_DEG;
}

static bool routeLineClearlyMissed(float routeLengthM, float routeUx, float routeUy) {
  float alongM = routeLineAlongM(robotX, robotY, routeUx, routeUy);
  float lateralErrorM = routeLineLateralErrorM(robotX, robotY, routeUx, routeUy);
  return alongM > routeLengthM + PLANNER_LINE_FOLLOW_MISSED_STOP_OVERSHOOT_M &&
         lateralErrorM <= PLANNER_LINE_FOLLOW_LATERAL_TOLERANCE_M;
}


static bool huntPickupCarryThroughActive(float routeLengthM, float routeUx,
                                         float routeUy) {
  if (!ownerIsObjectHunt(navigationGoal.owner)) {
    return false;
  }

  float alongM = routeLineAlongM(robotX, robotY, routeUx, routeUy);
  float lateralErrorM = routeLineLateralErrorM(robotX, robotY, routeUx, routeUy);
  return alongM >= routeLengthM - PLANNER_HUNT_PICKUP_CARRY_ZONE_M &&
         alongM <= routeLengthM + PLANNER_HUNT_FINISH_OVERSHOOT_M &&
         lateralErrorM <= PLANNER_HUNT_FINISH_LATERAL_M;
}



static bool huntPickupZoneReached(float routeLengthM, float routeUx, float routeUy) {
  if (!ownerIsObjectHunt(navigationGoal.owner)) {
    return false;
  }

  float alongM = routeLineAlongM(robotX, robotY, routeUx, routeUy);
  float lateralErrorM = routeLineLateralErrorM(robotX, robotY, routeUx, routeUy);
  return alongM >= routeLengthM - PLANNER_HUNT_FINISH_TARGET_TOLERANCE_M &&
         alongM <= routeLengthM + PLANNER_HUNT_FINISH_OVERSHOOT_M &&
         lateralErrorM <= PLANNER_HUNT_FINISH_LATERAL_M;
}

static float pointAlignmentTurnErrorDeg(float headingErrorDeg) {
  return headingErrorDeg;
}

static bool commandPointAlignmentTurn(float headingErrorDeg) {
  float selectedHeadingErrorDeg = pointAlignmentTurnErrorDeg(headingErrorDeg);
  if (!plannerContext.pointAlignTurnActive || plannerContext.pointAlignTurnDirection == 0.0f) {
    plannerContext.pointAlignTurnActive = true;
    plannerContext.pointAlignTurnDirection = selectedHeadingErrorDeg >= 0.0f ? 1.0f : -1.0f;
    resetTurnStuckCheck(navigationHeadingDeg());
  }
  bool slowTurn = fabs(headingErrorDeg) < SLOW_ZONE_DEG;
  float turnTarget = plannerContext.pointAlignTurnDirection *
                     (slowTurn ? PLANNER_TURN_SLOW_TARGET_SPEED : PLANNER_TURN_TARGET_SPEED);

  if (!isTurnDirectionObservable(turnTarget)) {
    unsigned long now = millis();
    if (plannerContext.turnSideInvalidSinceMs == 0) {
      plannerContext.turnSideInvalidSinceMs = now;
      sendBluetoothEvent("turn_side_revalidate", "point_align_sensor_recheck");
    }
    motorStopRequested = true;
    requestMotionStop();
    if (now - plannerContext.turnSideInvalidSinceMs < PLANNER_TURN_SENSOR_REVALIDATE_MS) {
      plannerTelemetry.planReason = "point_align_side_revalidating";
      return true;
    }
    finishNavigationGoal(false, PLANNER_STOP_TURN_SIDE_INVALID, "point_align_side_sensor_invalid");
    return true;
  }
  plannerContext.turnSideInvalidSinceMs = 0;

  if (!areTurnSweepSensorsValid()) {
    unsigned long now = millis();
    if (plannerContext.turnSweepInvalidSinceMs == 0) {
      plannerContext.turnSweepInvalidSinceMs = now;
      sendBluetoothEvent("turn_sweep_revalidate", "point_align_sensor_recheck");
    }
    motorStopRequested = true;
    requestMotionStop();
    if (now - plannerContext.turnSweepInvalidSinceMs < PLANNER_TURN_SENSOR_REVALIDATE_MS) {
      plannerTelemetry.planReason = "point_align_sweep_revalidating";
      return true;
    }
    finishNavigationGoal(false, PLANNER_STOP_TURN_CLEARANCE, "point_align_sweep_sensor_invalid");
    return true;
  }
  plannerContext.turnSweepInvalidSinceMs = 0;

  if (!isTurnSweepSafe()) {
    finishNavigationGoal(false, PLANNER_STOP_TURN_CLEARANCE, "point_align_sweep_not_clear");
    return true;
  }

  updateStuckTurning(navigationHeadingDeg());
  if (turnStuck) {
    finishNavigationGoal(false, PLANNER_STOP_STUCK, "point_align_progress_stalled");
    return true;
  }

  plannerTelemetry.selectedForwardTicksPerSec = 0.0;
  plannerTelemetry.selectedTurnTicksPerSec = turnTarget;
  plannerTelemetry.selectedCurvature = 0.0;
  plannerTelemetry.minimumSweptClearanceMm = minimumFanSweepClearanceMm();
  plannerTelemetry.speedCapTicksPerSec = 0.0;
  plannerTelemetry.localGoalDistanceM = 0.0;
  plannerTelemetry.candidateCount = 1;
  plannerTelemetry.stopReason = PLANNER_STOP_NONE;
  plannerTelemetry.planReason = slowTurn ? "point_align_turn_slow" : "point_align_turn_fast";
  plannerTelemetry.replanReason = "point_target_behind";
  plannerTelemetry.safeStopReason = "";
  plannerContext.lastReportedStopReason = PLANNER_STOP_NONE;
  motorStopRequested = false;
  publishNavigationMotion(0.0, turnTarget);
  return true;
}


static void reportPlannerStopIfChanged() {
  // Avoid spamming the Bluetooth stream every 40 ms while the same safe-stop
  // condition persists. A changed reason is an event worth investigating.
  if (plannerTelemetry.stopReason == plannerContext.lastReportedStopReason) {
    return;
  }
  plannerContext.lastReportedStopReason = plannerTelemetry.stopReason;
  sendBluetoothEvent("planner_safe_stop", plannerStopReasonName(plannerTelemetry.stopReason));
}




static void updatePointGoal() {
  float dx = navigationGoal.targetX - robotX;
  float dy = navigationGoal.targetY - robotY;
  float distanceM = sqrtf(dx * dx + dy * dy);
  float arrivalToleranceM = ownerIsObjectHunt(navigationGoal.owner)
                              ? PLANNER_HUNT_FINISH_TARGET_TOLERANCE_M
                              : WAYPOINT_TOLERANCE_M;
  if (distanceM <= arrivalToleranceM) {
    finishNavigationGoal(true, PLANNER_STOP_NONE, "waypoint_reached");
    return;
  }

  float routeLengthM = 0.0f;
  float routeUx = 1.0f;
  float routeUy = 0.0f;
  float routeHeadingRad = 0.0f;
  bool routeFrameValid =
    routeLineFrame(routeLengthM, routeUx, routeUy, routeHeadingRad);
  bool routeLineEligible =
    routeFrameValid &&
    fabs(wrapAngle(routeHeadingRad * RAD_TO_DEG - navigationHeadingDeg())) <=
      PLANNER_LINE_FOLLOW_ENABLE_HEADING_DEG;

  if (routeFrameValid && huntPickupZoneReached(routeLengthM, routeUx, routeUy)) {
    finishNavigationGoal(true, PLANNER_STOP_NONE, "hunt_pickup_zone_reached");
    return;
  }

  if (plannerContext.emergencyRecoveryState.phase != EMERGENCY_RECOVERY_IDLE &&
      plannerContext.emergencyRecoveryState.phase !=
        EMERGENCY_RECOVERY_RETRY_ACTIVE) {
    updateEmergencyRecovery();
    return;
  }

  if (plannerContext.reverseRecoveryState.active) {
    float lookaheadM = min(distanceM, WAYPOINT_LOOKAHEAD_M);
    float recoveryGoalX = robotX + (dx / distanceM) * lookaheadM;
    float recoveryGoalY = robotY + (dy / distanceM) * lookaheadM;
    if (updateReverseRecoveryBudget()) {
      return;
    }

    float reverseSegmentDistanceM = sqrtf(
      (robotX - plannerContext.reverseRecoveryState.startX) *
        (robotX - plannerContext.reverseRecoveryState.startX) +
      (robotY - plannerContext.reverseRecoveryState.startY) *
        (robotY - plannerContext.reverseRecoveryState.startY));
    if (!plannerContext.reverseRecoveryState.checkingForward &&
        reverseSegmentDistanceM >=
          PLANNER_REVERSE_FORWARD_RECHECK_DISTANCE_M) {
      stopMotors();
      plannerContext.reverseRecoveryState.checkingForward = true;
      plannerContext.reverseRecoveryState.forwardCheckAfterMovement = true;
      resetReversePlannerEpoch();
      plannerTelemetry.planReason = "reverse_distance_checkpoint";
      plannerTelemetry.replanReason = "forward_takeover_check";
      return;
    }

    if (plannerContext.reverseRecoveryState.checkingForward) {
      bool takeoverAvoidance = updateObstacleContext(
        navigationGoal.targetX, navigationGoal.targetY);
      if (takeoverAvoidance) {
        buildObstacleLocalGoal(recoveryGoalX, recoveryGoalY);
      }
      TrajectoryPlanResult forwardResult =
        selectTrajectory(recoveryGoalX, recoveryGoalY);
      if (forwardResult == TRAJECTORY_PLAN_PENDING ||
          forwardResult == TRAJECTORY_PLAN_ABORTED ||
          forwardResult == TRAJECTORY_PLAN_RETRY) {
        return;
      }
      if (forwardResult == TRAJECTORY_PLAN_SUCCESS) {
        completeEvidenceDrivenReverse();
        return;
      }
      if (!plannerContext.reverseRecoveryState.forwardCheckAfterMovement) {
        handleRecoveryExhaustion(
          PLANNER_STOP_RECOVERY_NO_USEFUL_OUTCOME,
          "reverse_corridor_without_forward_path");
        return;
      }
      stopMotors();
      plannerContext.reverseRecoveryState.checkingForward = false;
      plannerContext.reverseRecoveryState.forwardCheckAfterMovement = false;
      plannerContext.reverseRecoveryState.startX = robotX;
      plannerContext.reverseRecoveryState.startY = robotY;
      plannerContext.reverseRecoveryState.plateauCount = 0;
      resetPlannerEpoch();
      resetReversePlannerEpoch();
      plannerTelemetry.planReason = "reverse_checkpoint_continuing";
      plannerTelemetry.replanReason = "forward_takeover_not_ready";
      return;
    }

    TrajectoryPlanResult reverseResult =
      selectReverseRecoveryTrajectory(recoveryGoalX, recoveryGoalY);
    if (reverseResult == TRAJECTORY_PLAN_PENDING ||
        reverseResult == TRAJECTORY_PLAN_SUCCESS ||
        reverseResult == TRAJECTORY_PLAN_RETRY ||
        reverseResult == TRAJECTORY_PLAN_ABORTED) {
      return;
    }
    // Exhausting the remembered reverse corridor may mean the repositioning
    // succeeded. Stop and give the normal forward planner one fresh,
    // revalidated takeover attempt before declaring recovery impossible.
    stopMotors();
    plannerContext.reverseRecoveryState.checkingForward = true;
    plannerContext.reverseRecoveryState.forwardCheckAfterMovement = false;
    resetReversePlannerEpoch();
    plannerTelemetry.planReason = "reverse_corridor_exhausted";
    plannerTelemetry.replanReason = "forward_takeover_check";
    return;
  }

  bool huntCarryThroughActive =
    routeLineEligible && huntPickupCarryThroughActive(routeLengthM, routeUx, routeUy);
  bool avoidanceActive = updateObstacleContext(navigationGoal.targetX,
                                               navigationGoal.targetY);
  float targetHeadingDeg = atan2f(dy, dx) * RAD_TO_DEG;
  float targetHeadingErrorDeg = wrapAngle(targetHeadingDeg - navigationHeadingDeg());
  if (!plannerContext.plannerEpoch.active && !avoidanceActive &&
      !huntCarryThroughActive &&
      distanceM > PLANNER_FINAL_BLOCKED_ACCEPTANCE_M &&
      fabs(targetHeadingErrorDeg) > PLANNER_POINT_ALIGN_START_DEG) {
    commandPointAlignmentTurn(targetHeadingErrorDeg);
    return;
  }
  plannerContext.pointAlignTurnActive = false;
  plannerContext.pointAlignTurnDirection = 0.0;

  float localGoalX = 0.0f;
  float localGoalY = 0.0f;
  if (avoidanceActive) {
    buildObstacleLocalGoal(localGoalX, localGoalY);
    plannerTelemetry.planReason = "obstacle_local_bypass";
  } else {
    float lookaheadM = min(distanceM, WAYPOINT_LOOKAHEAD_M);
    localGoalX = robotX + (dx / distanceM) * lookaheadM;
    localGoalY = robotY + (dy / distanceM) * lookaheadM;
    plannerTelemetry.planReason = "direct_waypoint";
  }

  float localGoalDistanceM = sqrtf(
    (localGoalX - robotX) * (localGoalX - robotX) +
    (localGoalY - robotY) * (localGoalY - robotY));
  float routeAlongM = routeFrameValid
    ? routeLineAlongM(robotX, robotY, routeUx, routeUy) : 0.0f;
  plannerTelemetry.globalGoalDistanceM = distanceM;
  plannerTelemetry.localGoalDistanceM = localGoalDistanceM;
  plannerTelemetry.routeAlongProgressM = routeAlongM;
  plannerTelemetry.routeSignedLateralErrorM = routeFrameValid
    ? routeLineSignedLateralErrorM(robotX, robotY, routeUx, routeUy) : 0.0f;
  plannerTelemetry.cumulativeReverseDistanceM =
    plannerContext.recoveryBudget.cumulativeReverseDistanceM;
  plannerTelemetry.recoveryCount = plannerContext.recoveryBudget.attemptCount;

  bool madeNetTakeoverProgress =
    distanceM <= plannerContext.recoveryBudget.takeoverStartGoalDistanceM -
                   PLANNER_RECOVERY_TAKEOVER_PROGRESS_M ||
    (routeFrameValid &&
     routeAlongM >= plannerContext.recoveryBudget.takeoverStartRouteAlongM +
                      PLANNER_RECOVERY_TAKEOVER_PROGRESS_M);
  if (plannerContext.recoveryBudget.forwardTakeoverPending && !avoidanceActive &&
      madeNetTakeoverProgress) {
    resetRecoveryBudget();
  }
  if (avoidanceActive &&
      obstacleProgressStalled(localGoalX, localGoalY, localGoalDistanceM)) {
    motorStopRequested = true;
    requestMotionStop();
    if (canStartSafeReverse()) {
      startEvidenceDrivenReverse("obstacle_local_goal_stalled");
    } else {
      handleRecoveryExhaustion(
        PLANNER_STOP_RECOVERY_NO_PROGRESS,
        "obstacle_local_goal_progress_stalled");
    }
    return;
  }

  if (!avoidanceActive && routeLineEligible &&
      routeLineGoalReached(routeLengthM, routeUx, routeUy, routeHeadingRad)) {
    finishNavigationGoal(true, PLANNER_STOP_NONE, "route_line_reached");
    return;
  }
  if (!avoidanceActive && routeLineEligible &&
      routeLineOvershootStopReached(routeLengthM, routeUx, routeUy, routeHeadingRad)) {
    finishNavigationGoal(true, PLANNER_STOP_NONE, "route_line_overshoot_reached");
    return;
  }
  if (!avoidanceActive && routeLineEligible &&
      routeLineClearlyMissed(routeLengthM, routeUx, routeUy)) {
    finishNavigationGoal(false, PLANNER_STOP_ABORTED, "route_line_missed");
    return;
  }

  if (driveStuck || wheelMismatchStuck) {
    plannerTelemetry.replanReason = "wheel_progress_stalled";
    finishNavigationGoal(false, PLANNER_STOP_STUCK, "drive_progress_stalled");
    return;
  }

  TrajectoryPlanResult trajectoryResult =
    selectTrajectory(localGoalX, localGoalY);
  if (trajectoryResult == TRAJECTORY_PLAN_PENDING ||
      trajectoryResult == TRAJECTORY_PLAN_ABORTED) {
    return;
  }
  if (trajectoryResult == TRAJECTORY_PLAN_RETRY) {
    resetGeometricNoPathEvidence();
    return;
  }
  if (trajectoryResult == TRAJECTORY_PLAN_SUCCESS) {
    resetGeometricNoPathEvidence();
    return;
  }

  motorStopRequested = true;
  requestMotionStop();
  reportPlannerStopIfChanged();
  if (!currentPlannerFailureIsGeometricNoPath()) {
    resetGeometricNoPathEvidence();
    return;
  }
  if (!avoidanceActive &&
      distanceM <= PLANNER_FINAL_BLOCKED_ACCEPTANCE_M) {
    PlannerCollisionSnapshot currentCollision;
    plannerMapCaptureCollisionSnapshot(currentCollision);
    if (footprintClearOnSnapshot(currentCollision, robotX, robotY,
                                 plannerNavigationHeadingRad())) {
      // The rejected rollout may extend toward an obstacle beyond the point,
      // but no motion is needed from this already-safe bounded final pose.
      finishNavigationGoal(true, PLANNER_STOP_NONE, "final_blocked_reached");
      return;
    }
  }
  noteGeometricNoPathEpoch();
  if (canStartEvidenceDrivenReverse()) {
    startEvidenceDrivenReverse("persistent_geometric_no_path");
    return;
  }
  if (millis() - plannerContext.noSafeTrajectorySinceMs >= PLANNER_NO_PATH_ABORT_MS) {
    handleRecoveryExhaustion(
      PLANNER_STOP_NO_SAFE_TRAJECTORY,
      avoidanceActive
        ? "obstacle_has_no_safe_forward_bypass"
        : "no_safe_forward_trajectory");
  }
}

static void updateTurnGoal() {
  // Turn control is intentionally simpler than point planning: command a
  // calibrated direction, observe yaw, slow inside 30 degrees, then brake.
  if (plannerContext.turnBrakeActive) {
    // Motors are briefly commanded opposite the last turn direction to cancel
    // measured coast. The pulse is safety-gated by the same sweep checks used
    // for the normal pivot.
    if (millis() < plannerContext.turnBrakeUntilMs) {
      float brakeTarget = -plannerContext.turnLastCommandDirection * PLANNER_TURN_SLOW_TARGET_SPEED;
      plannerTelemetry.selectedForwardTicksPerSec = 0.0;
      plannerTelemetry.selectedTurnTicksPerSec = brakeTarget;
      plannerTelemetry.selectedCurvature = 0.0;
      plannerTelemetry.minimumSweptClearanceMm = minimumFanSweepClearanceMm();
      plannerTelemetry.speedCapTicksPerSec = 0.0;
      plannerTelemetry.candidateCount = 1;
      plannerTelemetry.stopReason = PLANNER_STOP_NONE;
      plannerTelemetry.planReason = "calibrated_turn_brake";
      motorStopRequested = false;
      publishNavigationMotion(0.0, brakeTarget);
      return;
    }

    plannerContext.turnBrakeActive = false;
    sendBluetoothEvent("turn_brake_end", "pulse_complete");
    finishNavigationGoal(true, PLANNER_STOP_NONE, "turn_reached");
    return;
  }

  // wrapAngle gives the shortest signed route to target heading in [-180, +180].
  float error = wrapAngle(navigationGoal.targetYawDeg - navigationHeadingDeg());
  if (fabs(error) <= TURN_TOLERANCE_DEG) {
    float brakeTarget = -plannerContext.turnLastCommandDirection * PLANNER_TURN_SLOW_TARGET_SPEED;
    if (isTurnDirectionObservable(brakeTarget) && isTurnSweepSafe()) {
      unsigned long brakePulseMs = plannerContext.turnLastCommandDirection > 0.0
        ? PLANNER_TURN_LEFT_BRAKE_PULSE_MS
        : PLANNER_TURN_RIGHT_BRAKE_PULSE_MS;
      plannerContext.turnBrakeActive = true;
      plannerContext.turnBrakeUntilMs = millis() + brakePulseMs;
      motorStopRequested = false;
      publishNavigationMotion(0.0, brakeTarget);
      sendBluetoothEvent("turn_brake_start", "calibrated_counterturn");
      return;
    }
    finishNavigationGoal(true, PLANNER_STOP_NONE, "turn_reached_brake_unavailable");
    return;
  }

  bool slowTurn = fabs(error) < SLOW_ZONE_DEG;
  plannerContext.turnLastCommandDirection = error > 0.0 ? 1.0 : -1.0;
  float turnTarget = (error > 0.0 ? 1.0 : -1.0) *
                     (slowTurn ? PLANNER_TURN_SLOW_TARGET_SPEED : PLANNER_TURN_TARGET_SPEED);
  SafePivotStepResult pivotResult = commandSafePivotStep(
    turnTarget,
    slowTurn ? "calibrated_turn_slow" : "calibrated_turn_fast",
    "turn_side_revalidating",
    "turn_sweep_revalidating");
  if (pivotResult == SAFE_PIVOT_STEP_SIDE_INVALID) {
    finishNavigationGoal(false, PLANNER_STOP_TURN_SIDE_INVALID,
                         "turn_side_sensor_invalid");
  } else if (pivotResult == SAFE_PIVOT_STEP_SWEEP_INVALID) {
    finishNavigationGoal(false, PLANNER_STOP_TURN_CLEARANCE,
                         "turn_sweep_sensor_invalid");
  } else if (pivotResult == SAFE_PIVOT_STEP_SWEEP_BLOCKED) {
    finishNavigationGoal(false, PLANNER_STOP_TURN_CLEARANCE,
                         "turn_sweep_not_clear");
  } else if (pivotResult == SAFE_PIVOT_STEP_STUCK) {
    finishNavigationGoal(false, PLANNER_STOP_STUCK,
                         "turn_progress_stalled");
  }
}

void updateNavigationController() {
  if (!navigationGoal.active) {
    return;
  }
  if (navigationGoal.authority == MOTION_AUTHORITY_NONE ||
      navigationGoal.authority != motionAuthority) {
    cancelNavigationGoal(PLANNER_STOP_ABORTED, "motion_authority_revoked");
    return;
  }

  unsigned long now = millis();
  // Map-based arc selection runs at 40 ms, but a pivot turn is a direct
  // yaw feedback task.  Run it alongside the 20 ms motor/sensor schedule so
  // it does not spend another 80 ms driving after crossing its tolerance.
  // Arc rollout is comparatively expensive and can run at 40 ms. A turn is a
  // simple yaw threshold problem, so it is sampled at the 20 ms motor cadence
  // to reduce overshoot at the tolerance boundary.
  unsigned long updateIntervalMs = navigationGoal.mode == NAV_GOAL_TURN
    ? MOTOR_CONTROL_INTERVAL_MS
    : PLANNER_UPDATE_INTERVAL_MS;
  bool plannerEpochPending =
    navigationGoal.mode == NAV_GOAL_POINT &&
    (plannerContext.plannerEpoch.active || plannerContext.reversePlannerEpoch.active);
  if (!plannerEpochPending && now - lastPlannerUpdateMs < updateIntervalMs) {
    return;
  }
  if (!plannerEpochPending) {
    lastPlannerUpdateMs = now;
    if (navigationGoal.mode == NAV_GOAL_TURN) {
      plannerTelemetry.lastPlanMs = now;
    }
  }
  if (navigationGoal.mode == NAV_GOAL_POINT) {
    updatePointGoal();
  } else if (navigationGoal.mode == NAV_GOAL_TURN) {
    updateTurnGoal();
  }
}

void initializeNavigationController() {
  // Called once after yaw/pose initialisation. From this point forward the map
  // moves with the robot rather than being re-created every route waypoint.
  initializePlannerContextDefaults();
  clearLocalMap();
  resetPlannerEpoch();
  resetReversePlannerEpoch();
  plannerContext.lastPlannerCommandPublishedMs = 0;
  resetEncodersAndPID();
  resetObstacleContext("controller_initialised");
  plannerTelemetry.planReason = "initialised";
}

void initializeNavigationRuntime() {
  initializeNavigationController();
}

void updateNavigationRuntime() {
  // This is the timing backbone of autonomous motion. Its order is deliberate:
  // sense/map first, take immediate safety action second, update odometry,
  // select a new command if safe, then let the sole motor writer apply it.
  unsigned long now = millis();
  bool immediateSafetyStop = false;
  unsigned long phaseStartedUs = 0;
  if (now - lastSensorUpdateMs >= SENSOR_UPDATE_INTERVAL_MS) {
    lastSensorUpdateMs = now;
    phaseStartedUs = micros();
    updateTOFSensors();
    recordMainLoopPhaseDuration("fan_tof", phaseStartedUs);
    phaseStartedUs = micros();
    updateLocalMapFromSensors();
    recordMainLoopPhaseDuration("local_map", phaseStartedUs);

    if (navigationGoal.active && isTofCloseReadingRevalidating()) {
      // A sudden close return is plausible collision evidence but also a known
      // ToF/I2C failure mode. Pause for one confirmation sample; do not map it
      // as a wall unless the sensor repeats it.
      immediateSafetyStop = true;
      motorStopRequested = true;
      requestMotionStop();
      plannerTelemetry.stopReason = PLANNER_STOP_NONE;
      plannerTelemetry.safeStopReason = "tof_close_revalidating";
      plannerTelemetry.replanReason = "tof_close_revalidating";
    }

    if (!immediateSafetyStop && navigationGoal.active &&
        isRangeSensorBlocked(RANGE_FRONT)) {
      immediateSafetyStop = true;
      motorStopRequested = true;
      requestMotionStop();
      plannerTelemetry.stopReason = PLANNER_STOP_FRONT_BLOCKED;
      plannerTelemetry.safeStopReason = "front_blocked";
      plannerTelemetry.replanReason = "front_blocked";
      reportPlannerStopIfChanged();
      if (navigationGoal.mode == NAV_GOAL_POINT) {
        // Keep the immediate neutral command, then allow the point planner to
        // publish only if a fresh forward arc passes the same sensor/map veto.
        immediateSafetyStop = false;
      } else if (navigationGoal.mode == NAV_GOAL_TURN) {
        cancelNavigationGoal(PLANNER_STOP_FRONT_BLOCKED, "front_blocked_during_turn");
      }
    }

    RangeSensorId diagonalSensor;
    float diagonalClearanceMm = 0.0;
    if (!immediateSafetyStop && navigationGoal.active &&
        getDiagonalClearanceWarning(diagonalSensor, diagonalClearanceMm)) {
      // This is the fast, current-pose guard. It uses chassis-footprint
      // clearance rather than turn radius, so it can stop an imminent clip
      // without wrongly forbidding a pre-aligned narrow straight passage.
      immediateSafetyStop = true;
      motorStopRequested = true;
      requestMotionStop();
      plannerTelemetry.stopReason = PLANNER_STOP_NO_SAFE_TRAJECTORY;
      plannerTelemetry.safeStopReason = "diagonal_clearance";
      plannerTelemetry.replanReason = "diagonal_clearance";
      reportPlannerStopIfChanged();
      if (navigationGoal.mode == NAV_GOAL_TURN) {
        cancelNavigationGoal(PLANNER_STOP_TURN_CLEARANCE, "diagonal_clearance_during_turn");
      }
    }
  }

  if (now - lastOdometryUpdateMs >= ODOMETRY_UPDATE_INTERVAL_MS) {
    lastOdometryUpdateMs = now;
    phaseStartedUs = micros();
    updateOdometry();
    recordMainLoopPhaseDuration("odometry_imu", phaseStartedUs);
    // Update pose before marking the footprint; otherwise the free patch would
    // lag behind the physical chassis by one odometry period.
    markTraversedFreeSpace();
  }

  if (!immediateSafetyStop) {
    // An immediate safety stop owns this cycle. The planner must not overwrite
    // it with a fresh arc until another controller pass has seen new evidence.
    phaseStartedUs = micros();
    updateNavigationController();
    recordMainLoopPhaseDuration("planner", phaseStartedUs);
  }
  // No other file writes servo pulses during normal navigation.
  phaseStartedUs = micros();
  updateMotorController();
  recordMainLoopPhaseDuration("motor_writer", phaseStartedUs);
  phaseStartedUs = micros();
  sendBluetoothTelemetry();
  recordMainLoopPhaseDuration("telemetry_build", phaseStartedUs);
}
