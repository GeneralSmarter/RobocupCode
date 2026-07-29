#include "../../Robot.h"
#include "NavigationInternal.h"
#include "ForwardTrajectoryPlanner.h"
#include "NavigationControllerInternal.h"
#include "ObstacleContext.h"
#include "PlannerCollision.h"
#include "PlannerContext.h"
#include "PlannerMap.h"
#include "PlannerProgress.h"
#include "RecoveryPlanner.h"


void resetRecoveryBudget() {
  plannerContext.recoveryBudget = {};
  plannerContext.recoveryBudget.lastReverseX = robotX;
  plannerContext.recoveryBudget.lastReverseY = robotY;
  plannerTelemetry.obstacleProgressAgeS = 0.0f;
  plannerTelemetry.cumulativeReverseDistanceM = 0.0f;
  plannerTelemetry.obstacleBestProgressM = 0.0f;
  plannerTelemetry.recoveryCount = 0;
}

static bool emergencyRecoveryOwnerEligible(NavigationGoalOwner owner) {
  return owner == NAV_OWNER_ROUTE ||
         owner == NAV_OWNER_RETURN_HOME ||
         owner == NAV_OWNER_TEST_GOTO ||
         owner == NAV_OWNER_TEST_AVOID ||
         owner == NAV_OWNER_TEST_ESCAPE;
}

static bool emergencyRecoveryReasonEligible(PlannerStopReason reason) {
  return reason == PLANNER_STOP_RECOVERY_TIME ||
         reason == PLANNER_STOP_RECOVERY_DISTANCE ||
         reason == PLANNER_STOP_RECOVERY_REPEATED ||
         reason == PLANNER_STOP_RECOVERY_NO_PROGRESS ||
         reason == PLANNER_STOP_RECOVERY_NO_USEFUL_OUTCOME ||
         reason == PLANNER_STOP_NO_SAFE_TRAJECTORY;
}

static bool emergencySensorFramesCurrent() {
  for (int i = RANGE_RIGHT_OUTER; i <= RANGE_LEFT_OUTER; ++i) {
    if (!isRangeSensorCurrent((RangeSensorId)i)) {
      return false;
    }
  }
  return hasTrustedRearCoverage() &&
         isRangeSensorCurrent(RANGE_FAKE_REAR);
}

static bool emergencyRecoverySensorsHealthy() {
  return emergencySensorFramesCurrent() &&
         !isRangeSensorBlocked(RANGE_FAKE_REAR);
}

static bool tryBeginEmergencyRecovery(PlannerStopReason reason,
                                      const char* detail) {
  if (!plannerContext.emergencyScanPolicyEnabled ||
      plannerContext.emergencyRecoveryState.consumed ||
      navigationGoal.mode != NAV_GOAL_POINT ||
      !emergencyRecoveryOwnerEligible(navigationGoal.owner) ||
      !emergencyRecoveryReasonEligible(reason) ||
      navigationGoal.authority == MOTION_AUTHORITY_NONE ||
      navigationGoal.authority != motionAuthority ||
      driveStuck || wheelMismatchStuck || turnStuck ||
      !emergencyRecoverySensorsHealthy()) {
    return false;
  }
  if (reason == PLANNER_STOP_NO_SAFE_TRAJECTORY &&
      plannerContext.recoveryBudget.attemptCount == 0) {
    return false;
  }

  const unsigned long now = millis();
  resetPlannerEpoch();
  resetReversePlannerEpoch();
  plannerContext.reverseRecoveryActive = false;
  plannerContext.reverseRecoveryState = {};
  resetRecoveryBudget();
  resetGeometricNoPathEvidence();
  plannerContext.emergencyRecoveryState = {};
  plannerContext.emergencyRecoveryState.phase = EMERGENCY_RECOVERY_SETTLE_CURRENT;
  plannerContext.emergencyRecoveryState.consumed = true;
  plannerContext.emergencyRecoveryState.startedMs = now;
  plannerContext.emergencyRecoveryState.phaseStartedMs = now;
  plannerContext.emergencyRecoveryState.startX = robotX;
  plannerContext.emergencyRecoveryState.startY = robotY;
  plannerContext.emergencyRecoveryState.lastX = robotX;
  plannerContext.emergencyRecoveryState.lastY = robotY;
  plannerContext.emergencyRecoveryState.relocationCheckpointX = robotX;
  plannerContext.emergencyRecoveryState.relocationCheckpointY = robotY;
  plannerContext.emergencyRecoveryState.scanLastYawDeg = navigationHeadingDeg();
  for (int i = RANGE_RIGHT_OUTER; i <= RANGE_LEFT_OUTER; ++i) {
    plannerContext.emergencyRecoveryState.fanReadBaselineMs[i - RANGE_RIGHT_OUTER] =
      rangeSensors[i].lastReadMs;
  }
  plannerContext.emergencyRecoveryState.rearFrameBaseline =
    getRearObstacleFrameSequence();
  motorStopRequested = true;
  requestMotionStop();
  plannerTelemetry.reverseRecoveryActive = false;
  plannerTelemetry.stopReason = PLANNER_STOP_NONE;
  plannerTelemetry.planReason = "emergency_scan_settle";
  plannerTelemetry.replanReason = detail;
  plannerTelemetry.safeStopReason = "";
  sendBluetoothEvent("emergency_scan_start", detail);
  return true;
}

bool handleRecoveryExhaustion(PlannerStopReason reason,
                                     const char* detail) {
  plannerTelemetry.replanReason = detail;
  if (emergencyRecoveryReasonEligible(reason) &&
      plannerContext.emergencyRecoveryState.consumed &&
      plannerContext.emergencyRecoveryState.retryActive) {
    finishNavigationGoal(false,
      PLANNER_STOP_EMERGENCY_RETRY_EXHAUSTED,
      "emergency_scan_retry_exhausted");
    return true;
  }
  if (tryBeginEmergencyRecovery(reason, detail)) {
    return true;
  }
  finishNavigationGoal(false, reason, detail);
  return true;
}

bool updateReverseRecoveryBudget() {
  unsigned long now = millis();
  plannerContext.recoveryBudget.cumulativeReverseDistanceM += hypotf(
    robotX - plannerContext.recoveryBudget.lastReverseX,
    robotY - plannerContext.recoveryBudget.lastReverseY);
  plannerContext.recoveryBudget.lastReverseX = robotX;
  plannerContext.recoveryBudget.lastReverseY = robotY;
  plannerTelemetry.obstacleProgressAgeS = 0.0f;
  plannerTelemetry.cumulativeReverseDistanceM =
    plannerContext.recoveryBudget.cumulativeReverseDistanceM;
  plannerTelemetry.recoveryCount = plannerContext.recoveryBudget.attemptCount;

  if (plannerContext.reverseRecoveryStartedMs != 0 &&
      now - plannerContext.reverseRecoveryStartedMs >
        PLANNER_REVERSE_RECOVERY_MAX_TIME_MS) {
    return handleRecoveryExhaustion(
      PLANNER_STOP_RECOVERY_TIME,
      "reverse_recovery_time_exhausted");
  }
  if (plannerContext.recoveryBudget.cumulativeReverseDistanceM >
      PLANNER_RECOVERY_MAX_CUMULATIVE_REVERSE_DISTANCE_M) {
    return handleRecoveryExhaustion(
      PLANNER_STOP_RECOVERY_DISTANCE,
      "reverse_recovery_distance_exhausted");
  }
  uint8_t maximumAttempts =
    plannerContext.emergencyRecoveryState.retryActive ? 1 : PLANNER_RECOVERY_MAX_COUNT;
  if (plannerContext.recoveryBudget.attemptCount > maximumAttempts) {
    return handleRecoveryExhaustion(
      PLANNER_STOP_RECOVERY_REPEATED,
      "recovery_count_exhausted");
  }
  return false;
}

static float forwardContinuationQuality(
    const PlannerCollisionSnapshot &snapshot,
    float startX, float startY, float startHeadingRad,
    float goalX, float goalY) {
  const float startDistanceM = sqrtf((goalX - startX) * (goalX - startX) +
                                     (goalY - startY) * (goalY - startY));
  float bestQuality = 0.0f;
  const float normalizedTurns[] = {-1.0f, -0.5f, 0.0f, 0.5f, 1.0f};
  for (unsigned int index = 0;
       index < sizeof(normalizedTurns) / sizeof(normalizedTurns[0]); ++index) {
    float forwardTicks = PLANNER_MIN_DRIVABLE_SPEED_TPS;
    float turnTicks = forwardTicks * normalizedTurns[index] *
                      PLANNER_MAX_TURN_RATIO;
    float leftMps = leftWheelTargetFromChassis(forwardTicks, turnTicks) /
                    TICKS_PER_METRE;
    float rightMps = rightWheelTargetFromChassis(forwardTicks, turnTicks) /
                     TICKS_PER_METRE;
    float linearMps = (leftMps + rightMps) * 0.5f;
    float angularRadPerSec = navigationOmegaFromWheelSpeeds(
      leftMps, rightMps, EFFECTIVE_TRACK_WIDTH_M);
    float x = startX;
    float y = startY;
    float heading = startHeadingRad;
    bool safe = true;
    for (float elapsed = 0.0f; elapsed < PLANNER_HORIZON_S;
         elapsed += PLANNER_ROLLOUT_STEP_S) {
      float midpointHeading = heading +
        angularRadPerSec * PLANNER_ROLLOUT_STEP_S * 0.5f;
      x += linearMps * cosf(midpointHeading) * PLANNER_ROLLOUT_STEP_S;
      y += linearMps * sinf(midpointHeading) * PLANNER_ROLLOUT_STEP_S;
      heading += angularRadPerSec * PLANNER_ROLLOUT_STEP_S;
      if (!footprintKnownClearOnSnapshot(snapshot, x, y, heading)) {
        safe = false;
        break;
      }
    }
    if (!safe) {
      continue;
    }
    float finalDistanceM = sqrtf((goalX - x) * (goalX - x) +
                                 (goalY - y) * (goalY - y));
    float progress = constrain((startDistanceM - finalDistanceM) /
                               max(PLANNER_MIN_PROGRESS_M, startDistanceM),
                               0.0f, 1.0f);
    float desiredHeadingDeg = atan2f(goalY - y, goalX - x) * RAD_TO_DEG;
    float headingErrorDeg = fabs(wrapAngle(
      desiredHeadingDeg - heading * RAD_TO_DEG));
    float headingQuality = 1.0f - min(1.0f, headingErrorDeg / 90.0f);
    bestQuality = max(bestQuality, 0.7f * progress + 0.3f * headingQuality);
  }
  return bestQuality;
}

static float calculateReverseRecoverySpeedCapTicksPerSec() {
  if (!hasTrustedRearCoverage() ||
      !isRangeSensorCurrent(RANGE_FAKE_REAR) ||
      isRangeSensorBlocked(RANGE_FAKE_REAR)) {
    return 0.0f;
  }

  float availableM = getRangeSensorDistance(RANGE_FAKE_REAR) / 1000.0f -
                     PLANNER_TOTAL_HARD_CLEARANCE_M -
                     PLANNER_REVERSE_RECOVERY_REAR_BUFFER_M;
  availableM = max(0.0f, availableM);
  float a = PLANNER_MAX_DECELERATION_MPS2;
  float latency = PLANNER_SENSING_LATENCY_S;
  float speedMps = -a * latency + sqrtf(a * a * latency * latency + 2.0f * a * availableM);
  speedMps = max(0.0f, speedMps);
  return min(PLANNER_REVERSE_RECOVERY_MAX_SPEED_TPS,
             min(baseTargetSpeed, speedMps * TICKS_PER_METRE));
}

static bool rolloutReverseRecoveryCandidate(const ReversePlannerEpoch &epoch,
                                            float reverseTicks, float turnTicks,
                                            float goalX, float goalY,
                                            float &rearClearanceMm,
                                            float &endpointClearanceM,
                                            float &sweepClearanceM,
                                            float &forwardQuality,
                                            float &unexploredScore,
                                            float &finalX,
                                            float &finalY,
                                            float &finalHeadingRad,
                                            CandidateRejectReason &rejectReason) {
  if (reverseTicks >= 0.0f) {
    rejectReason = CANDIDATE_REJECT_REAR_OBSERVATION;
    return false;
  }
  const float leftTicks = leftWheelTargetFromChassis(reverseTicks, turnTicks);
  const float rightTicks = rightWheelTargetFromChassis(reverseTicks, turnTicks);
  if (leftTicks >= 0.0f || rightTicks >= 0.0f) {
    rejectReason = CANDIDATE_REJECT_REAR_OBSERVATION;
    return false;
  }
  if (!epoch.rearValid || epoch.rearBlocked) {
    rejectReason = CANDIDATE_REJECT_REAR_OBSERVATION;
    return false;
  }

  float observedRearM = epoch.observedRearM;
  float trailingEnvelopeM = PLANNER_TOTAL_HARD_CLEARANCE_M +
                            PLANNER_REVERSE_RECOVERY_REAR_BUFFER_M;
  float heading = epoch.startHeadingRad;
  float x = epoch.startX;
  float y = epoch.startY;
  float leftMps = leftWheelTargetFromChassis(reverseTicks, turnTicks) /
                  TICKS_PER_METRE;
  float rightMps = rightWheelTargetFromChassis(reverseTicks, turnTicks) /
                   TICKS_PER_METRE;
  float linearMps = (leftMps + rightMps) * 0.5f;
  float angularRadPerSec = navigationOmegaFromWheelSpeeds(
    leftMps, rightMps, EFFECTIVE_TRACK_WIDTH_M);
  if (leftMps >= 0.0f || rightMps >= 0.0f) {
    rejectReason = CANDIDATE_REJECT_REAR_OBSERVATION;
    return false;
  }
  float travelledRearM = 0.0f;
  sweepClearanceM = PLANNER_REVERSE_CLEARANCE_CAP_M;

  for (float elapsed = 0.0f; elapsed < PLANNER_HORIZON_S; elapsed += PLANNER_ROLLOUT_STEP_S) {
    float midpointHeading = heading + angularRadPerSec * PLANNER_ROLLOUT_STEP_S * 0.5f;
    x += linearMps * cosf(midpointHeading) * PLANNER_ROLLOUT_STEP_S;
    y += linearMps * sinf(midpointHeading) * PLANNER_ROLLOUT_STEP_S;
    heading += angularRadPerSec * PLANNER_ROLLOUT_STEP_S;
    travelledRearM = -linearMps * (elapsed + PLANNER_ROLLOUT_STEP_S);

    if (travelledRearM + trailingEnvelopeM > observedRearM) {
      rejectReason = CANDIDATE_REJECT_REAR_OBSERVATION;
      return false;
    }
    // Reverse recovery uses the exact same inflated hard-footprint predicate
    // as forward planning. Direction-specific recovery logic may constrain a
    // candidate further, but it may never weaken collision rejection.
    if (!footprintClearOnSnapshot(epoch.collision, x, y, heading)) {
      rejectReason = CANDIDATE_REJECT_FOOTPRINT;
      return false;
    }
    if (footprintUnknownFractionOnSnapshot(epoch.collision, x, y, heading) >
        epoch.allowedUnknownFraction) {
      rejectReason = CANDIDATE_REJECT_CLEAR_EVIDENCE;
      return false;
    }
    sweepClearanceM = min(sweepClearanceM,
      poseKnownClearanceM(epoch.collision, x, y, heading));
  }

  rearClearanceMm = max(0.0f, (observedRearM - travelledRearM - trailingEnvelopeM) * 1000.0f);
  endpointClearanceM = poseKnownClearanceM(epoch.collision, x, y, heading);
  forwardQuality = forwardContinuationQuality(epoch.collision,
    x, y, heading, goalX, goalY);
  unexploredScore = projectedUnexploredScore(epoch.collision, x, y, heading);
  finalX = x;
  finalY = y;
  finalHeadingRad = heading;
  rejectReason = CANDIDATE_REJECT_NONE;
  return true;
}

static float reverseRecoveryScore(float reverseTicks, float turnTicks,
                                  float previousSelectedTurn,
                                  float sweepClearanceM,
                                  float forwardQuality,
                                  float unexploredScore) {
  float curvature = fabs(turnTicks) / max(1.0f, fabs(reverseTicks));
  float smoothnessScore = 1.0f - min(1.0f, fabs(turnTicks - previousSelectedTurn) /
                                          max(1.0f, baseTargetSpeed));
  float sweepScore = constrain(sweepClearanceM /
    PLANNER_REVERSE_CLEARANCE_CAP_M, 0.0f, 1.0f);
  float efficiencyScore = 0.5f * smoothnessScore +
    0.5f * (1.0f - min(1.0f, curvature /
      PLANNER_REVERSE_RECOVERY_MAX_TURN_RATIO));
  return PLANNER_REVERSE_FORWARD_QUALITY_WEIGHT * forwardQuality +
         PLANNER_REVERSE_UNEXPLORED_WEIGHT * unexploredScore +
         PLANNER_REVERSE_SWEEP_CLEARANCE_WEIGHT * sweepScore +
         PLANNER_REVERSE_EFFICIENCY_WEIGHT * efficiencyScore;
}

static void captureReversePlannerEpochView(ReversePlannerEpoch &epoch) {
  epoch.startX = robotX;
  epoch.startY = robotY;
  epoch.startHeadingRad = plannerNavigationHeadingRad();
  epoch.rearValid = hasTrustedRearCoverage() &&
                    isRangeSensorCurrent(RANGE_FAKE_REAR);
  epoch.rearBlocked = isRangeSensorBlocked(RANGE_FAKE_REAR);
  epoch.observedRearM = epoch.rearValid
    ? getRangeSensorDistance(RANGE_FAKE_REAR) / 1000.0f : 0.0f;
  epoch.allowedUnknownFraction = currentReverseUnknownAllowance();
  plannerMapCaptureCollisionSnapshot(epoch.collision);
}

static void recordReversePlannerSlice(unsigned long sliceStartedUs) {
  unsigned long sliceUs = micros() - sliceStartedUs;
  plannerContext.reversePlannerEpoch.accumulatedWorkUs += sliceUs;
  plannerTelemetry.plannerSliceUs = sliceUs;
  plannerTelemetry.plannerSliceMaxUs =
    max(plannerTelemetry.plannerSliceMaxUs, sliceUs);
  plannerTelemetry.plannerEpochWorkUs =
    plannerContext.reversePlannerEpoch.accumulatedWorkUs;
  plannerTelemetry.plannerEpochMaxWorkUs =
    max(plannerTelemetry.plannerEpochMaxWorkUs,
        plannerContext.reversePlannerEpoch.accumulatedWorkUs);
  recordMainLoopPhaseDuration("reverse_planner_slice", sliceStartedUs);
}

static void closeReversePlannerEpoch() {
  plannerContext.reversePlannerEpoch.active = false;
  plannerContext.reversePlannerEpoch.awaitingRevalidation = false;
  plannerTelemetry.plannerEpochActive = false;
  plannerTelemetry.plannerEpochAgeMs =
    millis() - plannerContext.reversePlannerEpoch.startedMs;
}

static TrajectoryPlanResult retryReversePlannerEpoch(const char* safeReason,
                                                     const char* replanReason) {
  stopMotors();
  plannerTelemetry.stopReason = PLANNER_STOP_NO_SAFE_TRAJECTORY;
  plannerTelemetry.safeStopReason = safeReason;
  plannerTelemetry.replanReason = replanReason;
  closeReversePlannerEpoch();
  return TRAJECTORY_PLAN_RETRY;
}

static TrajectoryPlanResult beginReversePlannerEpoch(float goalX, float goalY) {
  // Captures the snapshot used to evaluate reverse recovery arcs. In the
  // simulator, rear evidence comes from the field-raycast rear channel.
  memset(&plannerContext.reversePlannerEpoch, 0, sizeof(plannerContext.reversePlannerEpoch));
  plannerContext.reversePlannerEpoch.active = true;
  plannerContext.reversePlannerEpoch.startedMs = millis();
  plannerContext.reversePlannerEpoch.goalStartedMs = navigationGoal.startedMs;
  plannerContext.reversePlannerEpoch.authority = navigationGoal.authority;
  plannerContext.reversePlannerEpoch.emergencyScanObjective =
    plannerContext.emergencyRecoveryState.phase == EMERGENCY_RECOVERY_RELOCATE;
  plannerContext.reversePlannerEpoch.goalX = goalX;
  plannerContext.reversePlannerEpoch.goalY = goalY;
  plannerContext.reversePlannerEpoch.bestScore = -1000000.0f;
  plannerContext.reversePlannerEpoch.bestClearanceBand = -1;
  plannerContext.reversePlannerEpoch.previousSelectedTurn =
    plannerTelemetry.selectedTurnTicksPerSec;
  captureReversePlannerEpochView(plannerContext.reversePlannerEpoch);
  plannerContext.reversePlannerEpoch.speedCap =
    calculateReverseRecoverySpeedCapTicksPerSec();
  plannerTelemetry.speedCapTicksPerSec = plannerContext.reversePlannerEpoch.speedCap;
  plannerTelemetry.localGoalDistanceM = sqrtf(
    (goalX - plannerContext.reversePlannerEpoch.startX) *
      (goalX - plannerContext.reversePlannerEpoch.startX) +
    (goalY - plannerContext.reversePlannerEpoch.startY) *
      (goalY - plannerContext.reversePlannerEpoch.startY));
  plannerTelemetry.candidateCount = 0;
  plannerTelemetry.plannerCandidatesProcessed = 0;
  plannerTelemetry.plannerYieldCount = 0;
  plannerTelemetry.plannerEpochWorkUs = 0;
  plannerTelemetry.plannerEpochAgeMs = 0;
  plannerTelemetry.plannerEpochActive = true;
  if (plannerContext.reversePlannerEpoch.speedCap <
      PLANNER_REVERSE_RECOVERY_MIN_SPEED_TPS) {
    plannerTelemetry.stopReason = PLANNER_STOP_NO_SAFE_TRAJECTORY;
    plannerTelemetry.safeStopReason = "rear_path_unavailable";
    plannerTelemetry.replanReason = "trusted_rear_unavailable";
    closeReversePlannerEpoch();
    return TRAJECTORY_PLAN_NO_PATH;
  }
  return TRAJECTORY_PLAN_PENDING;
}

TrajectoryPlanResult selectReverseRecoveryTrajectory(float goalX,
                                                             float goalY) {
  // Cooperative reverse-arc sampler. It mirrors selectTrajectory(): evaluate
  // a bounded number of candidates per loop, then revalidate the winner before
  // publication.
  if (!plannerContext.reversePlannerEpoch.active) {
    unsigned long sliceStartedUs = micros();
    TrajectoryPlanResult beginResult =
      beginReversePlannerEpoch(goalX, goalY);
    recordReversePlannerSlice(sliceStartedUs);
    if (beginResult == TRAJECTORY_PLAN_PENDING) {
      plannerContext.reversePlannerEpoch.yieldCount++;
      plannerTelemetry.plannerYieldCount = plannerContext.reversePlannerEpoch.yieldCount;
      plannerTelemetry.planReason = "reverse_planner_epoch_pending";
    }
    return beginResult;
  }
  if (plannerContext.reversePlannerEpoch.goalStartedMs != navigationGoal.startedMs ||
      plannerContext.reversePlannerEpoch.authority != navigationGoal.authority) {
    resetReversePlannerEpoch();
    return TRAJECTORY_PLAN_ABORTED;
  }

  unsigned long now = millis();
  plannerTelemetry.plannerEpochAgeMs =
    now - plannerContext.reversePlannerEpoch.startedMs;
  plannerTelemetry.plannerCommandAgeMs = plannerContext.lastPlannerCommandPublishedMs == 0
    ? 0 : now - plannerContext.lastPlannerCommandPublishedMs;
  if (!plannerContext.reversePlannerEpoch.commandStoppedForAge &&
      isMotorCommandLeaseArmed() &&
      plannerTelemetry.plannerCommandAgeMs >= PLANNER_COMMAND_MAX_AGE_MS) {
    stopMotors();
    plannerContext.reversePlannerEpoch.commandStoppedForAge = true;
    plannerTelemetry.safeStopReason = "planner_command_age_guard";
  }
  if (now - plannerContext.reversePlannerEpoch.startedMs > PLANNER_EPOCH_MAX_AGE_MS) {
    stopMotors();
    closeReversePlannerEpoch();
    finishNavigationGoal(false, PLANNER_STOP_ABORTED,
                         "reverse_planner_epoch_timeout");
    return TRAJECTORY_PLAN_ABORTED;
  }

  if (!plannerContext.reversePlannerEpoch.awaitingRevalidation) {
    const int totalCandidates =
      2 * PLANNER_REVERSE_RECOVERY_CURVATURE_SAMPLES;
    unsigned long sliceStartedUs = micros();
    uint8_t processedThisSlice = 0;
    while (plannerContext.reversePlannerEpoch.candidateIndex < totalCandidates) {
      if (processedThisSlice > 0 &&
          (processedThisSlice >= PLANNER_MAX_CANDIDATES_PER_SLICE ||
           micros() - sliceStartedUs >= PLANNER_SLICE_BUDGET_US)) {
        break;
      }
      int candidateIndex = plannerContext.reversePlannerEpoch.candidateIndex++;
      processedThisSlice++;
      plannerTelemetry.plannerCandidatesProcessed =
        plannerContext.reversePlannerEpoch.candidateIndex;
      int speedIndex = candidateIndex /
        PLANNER_REVERSE_RECOVERY_CURVATURE_SAMPLES;
      int curvatureIndex = candidateIndex %
        PLANNER_REVERSE_RECOVERY_CURVATURE_SAMPLES;
      float speedScale = speedIndex == 0
        ? 1.0f : PLANNER_REVERSE_RECOVERY_MIN_SPEED_SCALE;
      float reverseMagnitude = max(
        PLANNER_REVERSE_RECOVERY_MIN_SPEED_TPS,
        plannerContext.reversePlannerEpoch.speedCap * speedScale);
      reverseMagnitude = min(reverseMagnitude,
                             PLANNER_REVERSE_RECOVERY_MAX_SPEED_TPS);
      float normalized = -1.0f + (2.0f * curvatureIndex) /
        (PLANNER_REVERSE_RECOVERY_CURVATURE_SAMPLES - 1);
      float reverseTicks = -reverseMagnitude;
      float turnTicks = reverseMagnitude * normalized *
                        PLANNER_REVERSE_RECOVERY_MAX_TURN_RATIO;
      float rearClearanceMm = -1.0f;
      float endpointClearanceM = 0.0f;
      float sweepClearanceM = 0.0f;
      float forwardQuality = 0.0f;
      float unexploredScore = 0.0f;
      float finalX = plannerContext.reversePlannerEpoch.startX;
      float finalY = plannerContext.reversePlannerEpoch.startY;
      float finalHeadingRad = plannerContext.reversePlannerEpoch.startHeadingRad;
      CandidateRejectReason rejectReason = CANDIDATE_REJECT_NONE;
      if (!rolloutReverseRecoveryCandidate(
            plannerContext.reversePlannerEpoch, reverseTicks, turnTicks,
            plannerContext.reversePlannerEpoch.goalX, plannerContext.reversePlannerEpoch.goalY,
            rearClearanceMm, endpointClearanceM, sweepClearanceM,
            forwardQuality, unexploredScore,
            finalX, finalY, finalHeadingRad, rejectReason)) {
        if (rejectReason == CANDIDATE_REJECT_REAR_OBSERVATION) {
          plannerContext.reversePlannerEpoch.rejectedRear++;
        } else if (rejectReason == CANDIDATE_REJECT_FOOTPRINT) {
          plannerContext.reversePlannerEpoch.rejectedFootprint++;
        } else if (rejectReason == CANDIDATE_REJECT_CLEAR_EVIDENCE) {
          plannerContext.reversePlannerEpoch.rejectedEvidence++;
        }
        continue;
      }
      plannerContext.reversePlannerEpoch.acceptedCount++;
      plannerTelemetry.candidateCount =
        plannerContext.reversePlannerEpoch.acceptedCount;
      float rotationalClearanceM = endpointClearanceM;
      if (plannerContext.reversePlannerEpoch.emergencyScanObjective) {
        rotationalClearanceM = rotationalEnvelopeClearanceM(
          plannerContext.reversePlannerEpoch.collision, finalX, finalY);
      }
      float score = reverseRecoveryScore(
        reverseTicks, turnTicks,
        plannerContext.reversePlannerEpoch.previousSelectedTurn,
        sweepClearanceM, forwardQuality, unexploredScore);
      if (plannerContext.reversePlannerEpoch.emergencyScanObjective) {
        score += constrain(
          rotationalClearanceM / PLANNER_REVERSE_CLEARANCE_CAP_M,
          0.0f, 1.0f);
      }
      float selectionClearanceM =
        plannerContext.reversePlannerEpoch.emergencyScanObjective
          ? rotationalClearanceM : endpointClearanceM;
      int clearanceBand = (int)floorf(
        min(selectionClearanceM, PLANNER_REVERSE_CLEARANCE_CAP_M) /
        PLANNER_REVERSE_CLEARANCE_BAND_M + 0.0001f);
      if (clearanceBand > plannerContext.reversePlannerEpoch.bestClearanceBand ||
          (clearanceBand == plannerContext.reversePlannerEpoch.bestClearanceBand &&
           score > plannerContext.reversePlannerEpoch.bestScore)) {
        plannerContext.reversePlannerEpoch.bestClearanceBand = clearanceBand;
        plannerContext.reversePlannerEpoch.bestScore = score;
        plannerContext.reversePlannerEpoch.bestReverse = reverseTicks;
        plannerContext.reversePlannerEpoch.bestTurn = turnTicks;
      }
    }
    recordReversePlannerSlice(sliceStartedUs);
    if (plannerContext.reversePlannerEpoch.candidateIndex < totalCandidates) {
      plannerContext.reversePlannerEpoch.yieldCount++;
      plannerTelemetry.plannerYieldCount = plannerContext.reversePlannerEpoch.yieldCount;
      plannerTelemetry.planReason = "reverse_planner_epoch_pending";
      return TRAJECTORY_PLAN_PENDING;
    }
    plannerContext.reversePlannerEpoch.awaitingRevalidation = true;
    plannerContext.reversePlannerEpoch.yieldCount++;
    plannerTelemetry.plannerYieldCount = plannerContext.reversePlannerEpoch.yieldCount;
    plannerTelemetry.planReason = "reverse_planner_revalidating";
    return TRAJECTORY_PLAN_PENDING;
  }

  unsigned long sliceStartedUs = micros();
  if (plannerContext.reversePlannerEpoch.acceptedCount == 0) {
    recordReversePlannerSlice(sliceStartedUs);
    const bool waitingForEvidence =
      plannerContext.reversePlannerEpoch.rejectedEvidence > 0 &&
      plannerContext.reversePlannerEpoch.rearValid &&
      !plannerContext.reversePlannerEpoch.rearBlocked;
    plannerTelemetry.stopReason = PLANNER_STOP_NO_SAFE_TRAJECTORY;
    plannerTelemetry.safeStopReason = waitingForEvidence
      ? "reverse_waiting_for_evidence" : "no_reverse_recovery_arc";
    plannerTelemetry.replanReason = waitingForEvidence
      ? "reverse_unknown_allowance_ramp" : "no_reverse_arc";
    if (!plannerContext.reverseRecoveryRejectsReported) {
      char detail[64];
      snprintf(detail, sizeof(detail), "rear=%d;footprint=%d;evidence=%d",
               plannerContext.reversePlannerEpoch.rejectedRear,
               plannerContext.reversePlannerEpoch.rejectedFootprint,
               plannerContext.reversePlannerEpoch.rejectedEvidence);
      sendBluetoothEvent("reverse_recovery_rejects", detail);
      plannerContext.reverseRecoveryRejectsReported = true;
    }
    closeReversePlannerEpoch();
    return waitingForEvidence
      ? TRAJECTORY_PLAN_RETRY : TRAJECTORY_PLAN_NO_PATH;
  }

  captureReversePlannerEpochView(plannerContext.reversePlannerEpoch);
  float freshSpeedCap = calculateReverseRecoverySpeedCapTicksPerSec();
  if (freshSpeedCap < PLANNER_REVERSE_RECOVERY_MIN_SPEED_TPS) {
    recordReversePlannerSlice(sliceStartedUs);
    return retryReversePlannerEpoch("reverse_speed_cap_below_drivable_min",
                                    "reverse_speed_cap_retry");
  }
  float publishReverse = plannerContext.reversePlannerEpoch.bestReverse;
  float publishTurn = plannerContext.reversePlannerEpoch.bestTurn;
  float selectedMagnitude = fabs(publishReverse);
  if (selectedMagnitude > freshSpeedCap + 0.5f) {
    float speedScale = freshSpeedCap / selectedMagnitude;
    publishReverse = -freshSpeedCap;
    publishTurn *= speedScale;
  }
  float rearClearanceMm = -1.0f;
  float endpointClearanceM = 0.0f;
  float sweepClearanceM = 0.0f;
  float forwardQuality = 0.0f;
  float unexploredScore = 0.0f;
  float finalX = plannerContext.reversePlannerEpoch.startX;
  float finalY = plannerContext.reversePlannerEpoch.startY;
  float finalHeadingRad = plannerContext.reversePlannerEpoch.startHeadingRad;
  CandidateRejectReason rejectReason = CANDIDATE_REJECT_NONE;
  bool winnerStillSafe = rolloutReverseRecoveryCandidate(
      plannerContext.reversePlannerEpoch,
      publishReverse, publishTurn,
      plannerContext.reversePlannerEpoch.goalX, plannerContext.reversePlannerEpoch.goalY,
      rearClearanceMm, endpointClearanceM, sweepClearanceM,
      forwardQuality, unexploredScore,
      finalX, finalY, finalHeadingRad, rejectReason);
  recordReversePlannerSlice(sliceStartedUs);
  if (!winnerStillSafe) {
    return retryReversePlannerEpoch("reverse_winner_revalidation_rejected",
                                    "reverse_winner_revalidation_retry");
  }

  plannerContext.reverseRecoveryState.currentClearanceM = poseKnownClearanceM(
    plannerContext.reversePlannerEpoch.collision,
    plannerContext.reversePlannerEpoch.startX, plannerContext.reversePlannerEpoch.startY,
    plannerContext.reversePlannerEpoch.startHeadingRad);
  plannerContext.reverseRecoveryState.clearanceGainM = endpointClearanceM -
    plannerContext.reverseRecoveryState.currentClearanceM;
  if (plannerContext.reversePlannerEpoch.emergencyScanObjective) {
    float rotationalClearanceM = rotationalEnvelopeClearanceM(
      plannerContext.reversePlannerEpoch.collision, finalX, finalY);
    plannerContext.emergencyRecoveryState.bestRotationalClearanceM = max(
      plannerContext.emergencyRecoveryState.bestRotationalClearanceM,
      rotationalClearanceM);
  }
  if (plannerContext.reverseRecoveryState.clearanceGainM <=
      PLANNER_REVERSE_CLEARANCE_GAIN_M) {
    if (plannerContext.reverseRecoveryState.plateauCount <
        PLANNER_REVERSE_PLATEAU_EPOCHS) {
      plannerContext.reverseRecoveryState.plateauCount++;
    }
  } else {
    plannerContext.reverseRecoveryState.plateauCount = 0;
  }
  plannerTelemetry.recoveryCurrentClearanceM =
    plannerContext.reverseRecoveryState.currentClearanceM;
  plannerTelemetry.recoveryEndpointClearanceM = endpointClearanceM;
  plannerTelemetry.recoveryClearanceGainM =
    plannerContext.reverseRecoveryState.clearanceGainM;
  plannerTelemetry.recoveryUnexploredScore = unexploredScore;
  plannerTelemetry.recoveryPlateauCount =
    plannerContext.reverseRecoveryState.plateauCount;

  plannerContext.reverseRecoveryStepCount++;
  plannerTelemetry.selectedForwardTicksPerSec =
    publishReverse;
  plannerTelemetry.selectedTurnTicksPerSec = publishTurn;
  plannerTelemetry.selectedCurvature = publishTurn /
    max(1.0f, fabs(publishReverse));
  plannerTelemetry.minimumSweptClearanceMm =
    min(rearClearanceMm, sweepClearanceM * 1000.0f);
  plannerTelemetry.stopReason = PLANNER_STOP_NONE;
  plannerTelemetry.planReason = "reverse_recovery_arc_revalidated";
  plannerTelemetry.replanReason = "no_forward_path";
  plannerTelemetry.safeStopReason = "";
  plannerContext.lastReportedStopReason = PLANNER_STOP_NONE;
  plannerContext.reverseRecoveryRejectsReported = false;
  motorStopRequested = false;
  if (!publishNavigationMotion(publishReverse, publishTurn)) {
    return retryReversePlannerEpoch("reverse_winner_publication_vetoed",
                                    "reverse_winner_publication_retry");
  }
  plannerContext.lastPlannerCommandPublishedMs = millis();
  plannerTelemetry.plannerCommandAgeMs = 0;
  plannerTelemetry.lastPlanMs = plannerContext.lastPlannerCommandPublishedMs;
  lastPlannerUpdateMs = plannerContext.lastPlannerCommandPublishedMs;
  closeReversePlannerEpoch();
  if (plannerContext.reverseRecoveryStepCount == 1 ||
      (plannerContext.reverseRecoveryStepCount % 20) == 0) {
    sendBluetoothEvent("reverse_recovery_step", "arc_selected");
  }
  return TRAJECTORY_PLAN_SUCCESS;
}

static void recordEmergencySensorBaselines() {
  for (int i = RANGE_RIGHT_OUTER; i <= RANGE_LEFT_OUTER; ++i) {
    plannerContext.emergencyRecoveryState.fanReadBaselineMs[i - RANGE_RIGHT_OUTER] =
      rangeSensors[i].lastReadMs;
  }
  plannerContext.emergencyRecoveryState.rearFrameBaseline =
    getRearObstacleFrameSequence();
}

static bool emergencySensorsAdvancedSinceBaseline() {
  if (!emergencySensorFramesCurrent()) {
    return false;
  }
  for (int i = RANGE_RIGHT_OUTER; i <= RANGE_LEFT_OUTER; ++i) {
    if (rangeSensors[i].lastReadMs <=
        plannerContext.emergencyRecoveryState
          .fanReadBaselineMs[i - RANGE_RIGHT_OUTER]) {
      return false;
    }
  }
  uint32_t rearSequence = getRearObstacleFrameSequence();
  return rearSequence != 0 &&
         rearSequence != plannerContext.emergencyRecoveryState.rearFrameBaseline;
}

static void abortEmergencyRecovery(const char* detail) {
  motorStopRequested = true;
  requestMotionStop();
  sendBluetoothEvent("emergency_scan_abort", detail);
  finishNavigationGoal(false, PLANNER_STOP_EMERGENCY_SCAN_ABORTED, detail);
}

static int8_t chooseEmergencyScanDirection() {
  float leftClearanceMm = min(
    getFanSweepClearanceMm(RANGE_LEFT_INNER),
    getFanSweepClearanceMm(RANGE_LEFT_OUTER));
  float rightClearanceMm = min(
    getFanSweepClearanceMm(RANGE_RIGHT_INNER),
    getFanSweepClearanceMm(RANGE_RIGHT_OUTER));
  return leftClearanceMm >= rightClearanceMm ? 1 : -1;
}

static void startEmergencyScanAtCurrentPose() {
  resetReversePlannerEpoch();
  plannerContext.reverseRecoveryActive = false;
  plannerContext.reverseRecoveryState = {};
  plannerTelemetry.reverseRecoveryActive = false;
  plannerContext.emergencyRecoveryState.phase = EMERGENCY_RECOVERY_SCAN_TURN;
  plannerContext.emergencyRecoveryState.phaseStartedMs = millis();
  plannerContext.emergencyRecoveryState.scanDirection = chooseEmergencyScanDirection();
  plannerContext.emergencyRecoveryState.scanSector = 0;
  plannerContext.emergencyRecoveryState.scanLastYawDeg = navigationHeadingDeg();
  plannerContext.emergencyRecoveryState.scanAccumulatedDeg = 0.0f;
  plannerContext.emergencyRecoveryState.scanTargetAccumulatedDeg =
    PLANNER_EMERGENCY_SCAN_STEP_DEG;
  plannerContext.turnSideInvalidSinceMs = 0;
  plannerContext.turnSweepInvalidSinceMs = 0;
  resetTurnStuckCheck(navigationHeadingDeg());
  plannerTelemetry.planReason = "emergency_scan_turn";
  plannerTelemetry.replanReason = "stationary_scan";
}

static void enterEmergencyRelocation(const char* detail) {
  motorStopRequested = true;
  requestMotionStop();
  resetReversePlannerEpoch();
  if (plannerContext.emergencyRecoveryState.relocationStartedMs == 0) {
    plannerContext.emergencyRecoveryState.relocationStartedMs = millis();
    plannerContext.emergencyRecoveryState.relocationCheckpointX = robotX;
    plannerContext.emergencyRecoveryState.relocationCheckpointY = robotY;
    sendBluetoothEvent("emergency_relocation_start", detail);
  }
  plannerContext.emergencyRecoveryState.phase = EMERGENCY_RECOVERY_RELOCATE;
  plannerContext.emergencyRecoveryState.phaseStartedMs = millis();
  plannerContext.reverseRecoveryActive = true;
  plannerContext.reverseRecoveryStartedMs = millis();
  plannerContext.reverseRecoveryStepCount = 0;
  plannerContext.reverseRecoveryState = {};
  plannerTelemetry.reverseRecoveryActive = true;
  plannerTelemetry.planReason = "emergency_relocation";
  plannerTelemetry.replanReason = detail;
}

static void enterEmergencyScanUnwind(const char* detail) {
  motorStopRequested = true;
  requestMotionStop();
  resetReversePlannerEpoch();
  plannerContext.emergencyRecoveryState.phase =
    EMERGENCY_RECOVERY_SCAN_UNWIND;
  plannerContext.emergencyRecoveryState.phaseStartedMs = millis();
  plannerContext.emergencyRecoveryState.scanLastYawDeg =
    navigationHeadingDeg();
  plannerContext.turnSideInvalidSinceMs = 0;
  plannerContext.turnSweepInvalidSinceMs = 0;
  resetTurnStuckCheck(navigationHeadingDeg());
  plannerTelemetry.planReason = "emergency_scan_unwind";
  plannerTelemetry.replanReason = detail;
  sendBluetoothEvent("emergency_scan_blocked", detail);
}

static void enterEmergencySensorSettle(EmergencyRecoveryPhase phase,
                                       const char* planReason) {
  motorStopRequested = true;
  requestMotionStop();
  resetReversePlannerEpoch();
  plannerContext.reverseRecoveryActive = false;
  plannerTelemetry.reverseRecoveryActive = false;
  plannerContext.emergencyRecoveryState.phase = phase;
  plannerContext.emergencyRecoveryState.phaseStartedMs = millis();
  recordEmergencySensorBaselines();
  plannerTelemetry.planReason = planReason;
}

static void completeEmergencyScanAndPrepareRetry() {
  motorStopRequested = true;
  requestMotionStop();
  plannerContext.reverseRecoveryActive = false;
  plannerContext.reverseRecoveryState = {};
  resetPlannerEpoch();
  resetReversePlannerEpoch();
  resetGeometricNoPathEvidence();
  resetRecoveryBudget();
  resetObstacleContext("emergency_scan_retry");
  plannerContext.pointAlignTurnActive = false;
  plannerContext.pointAlignTurnDirection = 0.0f;
  plannerContext.turnBrakeActive = false;
  plannerContext.turnSideInvalidSinceMs = 0;
  plannerContext.turnSweepInvalidSinceMs = 0;
  resetTurnStuckCheck(navigationHeadingDeg());
  plannerContext.emergencyRecoveryState.phase = EMERGENCY_RECOVERY_RETRY_ACTIVE;
  plannerContext.emergencyRecoveryState.retryActive = true;
  plannerContext.emergencyRecoveryState.phaseStartedMs = millis();
  lastPlannerUpdateMs = 0;
  plannerTelemetry.reverseRecoveryActive = false;
  plannerTelemetry.stopReason = PLANNER_STOP_NONE;
  plannerTelemetry.planReason = "emergency_retry_ready";
  plannerTelemetry.replanReason = "scan_complete";
  plannerTelemetry.safeStopReason = "";
  sendBluetoothEvent("emergency_scan_complete", "full_revolution");
  sendBluetoothEvent("emergency_retry_start", "original_goal");
}

void updateEmergencyRecovery() {
  EmergencyRecoveryState &state = plannerContext.emergencyRecoveryState;
  const unsigned long now = millis();
  float translationM = hypotf(robotX - state.lastX, robotY - state.lastY);
  state.lastX = robotX;
  state.lastY = robotY;
  state.relocationDistanceM += translationM;
  plannerTelemetry.cumulativeReverseDistanceM =
    state.relocationDistanceM;

  if (now - state.startedMs > PLANNER_EMERGENCY_TOTAL_TIMEOUT_MS) {
    abortEmergencyRecovery("emergency_total_timeout");
    return;
  }

  if (state.phase == EMERGENCY_RECOVERY_SETTLE_CURRENT) {
    motorStopRequested = true;
    requestMotionStop();
    if (emergencySensorsAdvancedSinceBaseline()) {
      startEmergencyScanAtCurrentPose();
      return;
    }
    if (now - state.phaseStartedMs >
        PLANNER_EMERGENCY_SENSOR_WAIT_MS) {
      abortEmergencyRecovery("emergency_initial_sensor_timeout");
    }
    return;
  }

  if (state.phase == EMERGENCY_RECOVERY_SCAN_TURN) {
    if (!emergencySensorFramesCurrent()) {
      abortEmergencyRecovery("emergency_scan_sensor_unhealthy");
      return;
    }
    if (isRangeSensorBlocked(RANGE_FAKE_REAR)) {
      enterEmergencyScanUnwind("scan_rear_became_blocked");
      return;
    }
    float headingDeg = navigationHeadingDeg();
    float yawDeltaDeg = wrapAngle(headingDeg - state.scanLastYawDeg);
    state.scanLastYawDeg = headingDeg;
    float directedProgressDeg = yawDeltaDeg * state.scanDirection;
    if (fabs(yawDeltaDeg) <= PLANNER_EMERGENCY_SCAN_STEP_DEG) {
      state.scanAccumulatedDeg = max(
        0.0f, state.scanAccumulatedDeg + directedProgressDeg);
    }
    float remainingDeg = state.scanTargetAccumulatedDeg -
                         state.scanAccumulatedDeg;
    if (remainingDeg <= TURN_TOLERANCE_DEG) {
      motorStopRequested = true;
      requestMotionStop();
      state.phase = EMERGENCY_RECOVERY_SCAN_DWELL;
      state.phaseStartedMs = now;
      recordEmergencySensorBaselines();
      plannerTelemetry.planReason = "emergency_scan_dwell";
      return;
    }
    if (now - state.phaseStartedMs >
        PLANNER_EMERGENCY_SECTOR_TIMEOUT_MS) {
      abortEmergencyRecovery("emergency_scan_sector_timeout");
      return;
    }

    PlannerCollisionSnapshot snapshot;
    plannerMapCaptureCollisionSnapshot(snapshot);
    if (!turnSegmentFootprintClear(
          snapshot, robotX, robotY, headingDeg,
          state.scanDirection, remainingDeg)) {
      enterEmergencyScanUnwind("scan_segment_map_blocked");
      return;
    }

    float turnTarget = state.scanDirection *
      PLANNER_TURN_SLOW_TARGET_SPEED;
    SafePivotStepResult pivotResult = commandSafePivotStep(
      turnTarget,
      "emergency_scan_turn",
      "emergency_scan_side_revalidating",
      "emergency_scan_sweep_revalidating");
    if (pivotResult == SAFE_PIVOT_STEP_SWEEP_BLOCKED) {
      enterEmergencyScanUnwind("scan_live_sweep_blocked");
    } else if (pivotResult == SAFE_PIVOT_STEP_SIDE_INVALID ||
               pivotResult == SAFE_PIVOT_STEP_SWEEP_INVALID) {
      abortEmergencyRecovery("emergency_scan_sensor_invalid");
    } else if (pivotResult == SAFE_PIVOT_STEP_STUCK) {
      abortEmergencyRecovery("emergency_scan_turn_stalled");
    } else if (pivotResult ==
               SAFE_PIVOT_STEP_PUBLICATION_VETOED) {
      abortEmergencyRecovery("emergency_scan_motion_vetoed");
    }
    return;
  }

  if (state.phase == EMERGENCY_RECOVERY_SCAN_UNWIND) {
    if (!emergencySensorFramesCurrent()) {
      abortEmergencyRecovery("emergency_unwind_sensor_unhealthy");
      return;
    }
    float headingDeg = navigationHeadingDeg();
    float yawDeltaDeg = wrapAngle(headingDeg - state.scanLastYawDeg);
    state.scanLastYawDeg = headingDeg;
    float directedProgressDeg = yawDeltaDeg * state.scanDirection;
    if (fabs(yawDeltaDeg) <= PLANNER_EMERGENCY_SCAN_STEP_DEG) {
      state.scanAccumulatedDeg = max(
        0.0f, state.scanAccumulatedDeg + directedProgressDeg);
    }
    if (state.scanAccumulatedDeg <= TURN_TOLERANCE_DEG) {
      if (!emergencyRecoverySensorsHealthy()) {
        abortEmergencyRecovery("emergency_unwind_rear_blocked");
        return;
      }
      enterEmergencyRelocation("partial_scan_unwound");
      return;
    }
    if (now - state.phaseStartedMs >
        PLANNER_EMERGENCY_SECTOR_TIMEOUT_MS *
          PLANNER_EMERGENCY_SCAN_SECTORS) {
      abortEmergencyRecovery("emergency_scan_unwind_timeout");
      return;
    }

    PlannerCollisionSnapshot snapshot;
    plannerMapCaptureCollisionSnapshot(snapshot);
    if (!turnSegmentFootprintClear(
          snapshot, robotX, robotY, headingDeg,
          -state.scanDirection, state.scanAccumulatedDeg)) {
      abortEmergencyRecovery("emergency_scan_unwind_map_blocked");
      return;
    }
    float turnTarget = -state.scanDirection *
      PLANNER_TURN_SLOW_TARGET_SPEED;
    SafePivotStepResult pivotResult = commandSafePivotStep(
      turnTarget,
      "emergency_scan_unwind",
      "emergency_unwind_side_revalidating",
      "emergency_unwind_sweep_revalidating");
    if (pivotResult == SAFE_PIVOT_STEP_SIDE_INVALID ||
        pivotResult == SAFE_PIVOT_STEP_SWEEP_INVALID) {
      abortEmergencyRecovery("emergency_scan_unwind_sensor_invalid");
    } else if (pivotResult == SAFE_PIVOT_STEP_SWEEP_BLOCKED) {
      abortEmergencyRecovery("emergency_scan_unwind_sweep_blocked");
    } else if (pivotResult == SAFE_PIVOT_STEP_STUCK) {
      abortEmergencyRecovery("emergency_scan_unwind_stalled");
    } else if (pivotResult ==
               SAFE_PIVOT_STEP_PUBLICATION_VETOED) {
      abortEmergencyRecovery("emergency_scan_unwind_motion_vetoed");
    }
    return;
  }

  if (state.phase == EMERGENCY_RECOVERY_SCAN_DWELL) {
    motorStopRequested = true;
    requestMotionStop();
    if (emergencySensorsAdvancedSinceBaseline()) {
      state.scanSector++;
      char detail[48];
      snprintf(detail, sizeof(detail), "sector=%u;yaw=%.1f",
               state.scanSector, state.scanAccumulatedDeg);
      sendBluetoothEvent("emergency_scan_sector", detail);
      if (state.scanSector >= PLANNER_EMERGENCY_SCAN_SECTORS) {
        state.phase = EMERGENCY_RECOVERY_RESET_RETRY;
        state.phaseStartedMs = now;
        completeEmergencyScanAndPrepareRetry();
        return;
      }
      state.scanTargetAccumulatedDeg =
        (state.scanSector + 1) * PLANNER_EMERGENCY_SCAN_STEP_DEG;
      state.phase = EMERGENCY_RECOVERY_SCAN_TURN;
      state.phaseStartedMs = now;
      plannerContext.turnSideInvalidSinceMs = 0;
      plannerContext.turnSweepInvalidSinceMs = 0;
      resetTurnStuckCheck(navigationHeadingDeg());
      plannerTelemetry.planReason = "emergency_scan_turn";
      return;
    }
    if (now - state.phaseStartedMs >
        PLANNER_EMERGENCY_SENSOR_WAIT_MS) {
      abortEmergencyRecovery("emergency_scan_dwell_sensor_timeout");
    }
    return;
  }

  if (state.phase == EMERGENCY_RECOVERY_RELOCATE) {
    if (!emergencyRecoverySensorsHealthy()) {
      abortEmergencyRecovery("emergency_relocation_sensor_unhealthy");
      return;
    }
    if (state.relocationDistanceM >
        PLANNER_EMERGENCY_MAX_RELOCATION_M) {
      abortEmergencyRecovery("emergency_relocation_distance_exhausted");
      return;
    }
    if (state.relocationStartedMs != 0 &&
        now - state.relocationStartedMs >
          PLANNER_EMERGENCY_RELOCATE_TIMEOUT_MS) {
      abortEmergencyRecovery("emergency_relocation_timeout");
      return;
    }
    float checkpointDistanceM = hypotf(
      robotX - state.relocationCheckpointX,
      robotY - state.relocationCheckpointY);
    if (checkpointDistanceM >=
        PLANNER_EMERGENCY_RECHECK_DISTANCE_M) {
      state.relocationCheckpointX = robotX;
      state.relocationCheckpointY = robotY;
      sendBluetoothEvent("emergency_relocation_checkpoint",
                         "stationary_reobservation");
      enterEmergencySensorSettle(
        EMERGENCY_RECOVERY_SETTLE_RELOCATED,
        "emergency_relocation_settle");
      return;
    }

    float goalDx = navigationGoal.targetX - robotX;
    float goalDy = navigationGoal.targetY - robotY;
    float goalDistanceM = hypotf(goalDx, goalDy);
    float lookaheadM = min(goalDistanceM, WAYPOINT_LOOKAHEAD_M);
    float recoveryGoalX = goalDistanceM > 0.001f
      ? robotX + goalDx / goalDistanceM * lookaheadM : robotX;
    float recoveryGoalY = goalDistanceM > 0.001f
      ? robotY + goalDy / goalDistanceM * lookaheadM : robotY;
    TrajectoryPlanResult reverseResult =
      selectReverseRecoveryTrajectory(recoveryGoalX, recoveryGoalY);
    if (reverseResult == TRAJECTORY_PLAN_NO_PATH) {
      abortEmergencyRecovery("emergency_relocation_no_safe_arc");
    } else if (reverseResult == TRAJECTORY_PLAN_SUCCESS) {
      plannerTelemetry.planReason = "emergency_relocation_arc";
      plannerTelemetry.replanReason = "seek_scan_clearance";
    }
    return;
  }

  if (state.phase == EMERGENCY_RECOVERY_SETTLE_RELOCATED) {
    motorStopRequested = true;
    requestMotionStop();
    if (emergencySensorsAdvancedSinceBaseline()) {
      PlannerCollisionSnapshot snapshot;
      plannerMapCaptureCollisionSnapshot(snapshot);
      float rotationalClearanceM = rotationalEnvelopeClearanceM(
        snapshot, robotX, robotY);
      state.bestRotationalClearanceM = max(
        state.bestRotationalClearanceM, rotationalClearanceM);
      if (rotationalClearanceM >=
          PLANNER_EMERGENCY_TARGET_CLEARANCE_M) {
        startEmergencyScanAtCurrentPose();
      } else {
        enterEmergencyRelocation("scan_pose_clearance_insufficient");
      }
      return;
    }
    if (now - state.phaseStartedMs >
        PLANNER_EMERGENCY_SENSOR_WAIT_MS) {
      abortEmergencyRecovery("emergency_relocation_sensor_timeout");
    }
  }
}


bool canStartSafeReverse() {
  return navigationGoal.mode == NAV_GOAL_POINT &&
         escapeBacktrackEnabled &&
         hasTrustedRearCoverage() &&
         isRangeSensorCurrent(RANGE_FAKE_REAR) &&
         !isRangeSensorBlocked(RANGE_FAKE_REAR);
}

bool canStartEvidenceDrivenReverse() {
  return canStartSafeReverse() &&
         plannerContext.geometricNoPathEpochCount >=
           PLANNER_REVERSE_MIN_GEOMETRIC_NO_PATH_EPOCHS &&
         plannerContext.noSafeTrajectorySinceMs != 0 &&
         millis() - plannerContext.noSafeTrajectorySinceMs >=
           PLANNER_NO_PATH_BACKTRACK_DELAY_MS;
}

void startEvidenceDrivenReverse(const char* trigger) {
  resetPlannerEpoch();
  resetReversePlannerEpoch();
  plannerContext.reverseRecoveryState = {};
  plannerContext.reverseRecoveryState.active = true;
  plannerContext.reverseRecoveryState.startX = robotX;
  plannerContext.reverseRecoveryState.startY = robotY;
  plannerContext.reverseRecoveryActive = true;
  plannerContext.reverseRecoveryStartedMs = millis();
  plannerContext.reverseRecoveryStepCount = 0;
  plannerContext.recoveryBudget.forwardTakeoverPending = false;
  plannerContext.recoveryBudget.lastReverseX = robotX;
  plannerContext.recoveryBudget.lastReverseY = robotY;
  if (plannerContext.recoveryBudget.attemptCount < 255) {
    plannerContext.recoveryBudget.attemptCount++;
  }
  plannerContext.obstacleContext.progressGoalValid = false;
  plannerTelemetry.recoveryCount = plannerContext.recoveryBudget.attemptCount;
  plannerTelemetry.reverseRecoveryActive = true;
  plannerTelemetry.recoveryPlateauCount = 0;
  plannerTelemetry.planReason = "reverse_reposition_start";
  plannerTelemetry.replanReason = trigger;
  plannerTelemetry.safeStopReason = "";
  sendBluetoothEvent("reverse_recovery_start", trigger);
}

void completeEvidenceDrivenReverse() {
  resetReversePlannerEpoch();
  plannerContext.reverseRecoveryState = {};
  plannerContext.reverseRecoveryActive = false;
  plannerContext.recoveryBudget.forwardTakeoverPending = true;
  plannerContext.recoveryBudget.takeoverStartGoalDistanceM = hypotf(
    navigationGoal.targetX - robotX,
    navigationGoal.targetY - robotY);
  float routeLengthM;
  float routeUx;
  float routeUy;
  float routeHeadingRad;
  plannerContext.recoveryBudget.takeoverStartRouteAlongM =
    routeLineFrame(routeLengthM, routeUx, routeUy, routeHeadingRad)
      ? routeLineAlongM(robotX, robotY, routeUx, routeUy)
      : 0.0f;
  plannerContext.obstacleContext.progressGoalValid = false;
  plannerTelemetry.reverseRecoveryActive = false;
  plannerTelemetry.obstacleProgressAgeS = 0.0f;
  plannerTelemetry.recoveryPlateauCount = 0;
  resetGeometricNoPathEvidence();
  sendBluetoothEvent("reverse_recovery_end", "forward_takeover");
}
