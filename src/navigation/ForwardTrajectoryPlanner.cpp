#include "../../Robot.h"
#include "NavigationInternal.h"
#include "ForwardTrajectoryPlanner.h"
#include "NavigationControllerInternal.h"
#include "ObstacleContext.h"
#include "PlannerCollision.h"
#include "PlannerContext.h"
#include "PlannerMap.h"
#include "PlannerProgress.h"

static float calculateSpeedCapTicksPerSec(float requestedCapTicksPerSec) {
  if (!isRangeSensorValid(RANGE_FRONT)) {
    return 0.0;
  }

  float availableM = getRangeSensorDistance(RANGE_FRONT) / 1000.0 -
                     PLANNER_FRONT_SPEED_BUFFER_M;
  availableM = max(0.0f, availableM);
  // d = v * latency + v^2 / (2a). Solve this braking-distance equation for
  // v, then convert metres/s to encoder ticks/s. This cap is independent of
  // goal distance: it answers only "can the robot stop before the front
  // observation runs out?".
  float a = PLANNER_MAX_DECELERATION_MPS2;
  float latency = PLANNER_SENSING_LATENCY_S;
  float speedMps = -a * latency + sqrtf(a * a * latency * latency + 2.0 * a * availableM);
  speedMps = max(0.0f, speedMps);
  return min(requestedCapTicksPerSec, speedMps * TICKS_PER_METRE);
}


static bool epochTurnDirectionObservable(const PlannerEpoch &epoch,
                                         float turnTicks) {
  if (fabs(turnTicks) < 1.0f) {
    return epoch.rightInnerValid && epoch.leftInnerValid;
  }
  if (turnTicks > 0.0f) {
    return epoch.leftInnerValid && epoch.leftOuterValid;
  }
  return epoch.rightInnerValid && epoch.rightOuterValid;
}

static float epochObservedForwardM(const PlannerEpoch &epoch,
                                   float turnTicks) {
  if (turnTicks > 1.0f) {
    return epoch.observedLeftInnerM;
  }
  if (turnTicks < -1.0f) {
    return epoch.observedRightInnerM;
  }
  if (epoch.rightInnerValid && epoch.leftInnerValid) {
    return min(epoch.observedRightInnerM, epoch.observedLeftInnerM);
  }
  if (epoch.rightInnerValid) {
    return epoch.observedRightInnerM;
  }
  if (epoch.leftInnerValid) {
    return epoch.observedLeftInnerM;
  }
  return 0.0f;
}

static bool rolloutCandidate(const PlannerEpoch &epoch,
                             float forwardTicks, float turnTicks,
                             float goalX, float goalY,
                             float &minimumClearanceMm,
                             float &closestGoalDistanceM,
                             float &headingAtClosestGoalRad,
                             float &finalX,
                             float &finalY,
                             float &finalHeadingRad,
                             float &arrivalTimeS,
                             float &safeTravelM,
                             float &maximumUnknownFraction,
                             CandidateRejectReason &rejectReason) {
  // A candidate is a constant chassis command (forward, turn) simulated for
  // PLANNER_HORIZON_S. Positive turn is CCW/left; MotorControl later converts
  // this into a slower left wheel and faster right wheel.
  if (forwardTicks <= 0.0 ||
      !epochTurnDirectionObservable(epoch, turnTicks)) {
    rejectReason = CANDIDATE_REJECT_TURN_OBSERVABILITY;
    return false;
  }
  rejectReason = CANDIDATE_REJECT_NONE;

  // A candidate may not run its leading footprint beyond the distance that the
  // valid inner fan has actually observed.  Unknown space is not converted to
  // free space merely because it is absent from the local map.
  float observedForwardM = epochObservedForwardM(epoch, turnTicks);
  float leadingEnvelopeM = ROBOT_FOOTPRINT_GEOMETRY.frontExtentMm / 1000.0 +
                           PLANNER_TOTAL_HARD_CLEARANCE_M;

  float heading = epoch.startHeadingRad;
  float x = epoch.startX;
  float y = epoch.startY;
  // Differential-drive kinematics: command is expressed as average wheel
  // speed plus/minus a turn component, then integrated at the midpoint
  // heading to avoid the bias of a simple Euler step.
  float leftMps = leftWheelTargetFromChassis(forwardTicks, turnTicks) /
                  TICKS_PER_METRE;
  float rightMps = rightWheelTargetFromChassis(forwardTicks, turnTicks) /
                   TICKS_PER_METRE;
  float linearMps = (leftMps + rightMps) * 0.5;
  float angularRadPerSec = navigationOmegaFromWheelSpeeds(
    leftMps, rightMps, EFFECTIVE_TRACK_WIDTH_M);
  minimumClearanceMm = epoch.minimumFanClearanceMm;
  closestGoalDistanceM = sqrtf((goalX - x) * (goalX - x) +
                               (goalY - y) * (goalY - y));
  headingAtClosestGoalRad = heading;
  finalX = x;
  finalY = y;
  finalHeadingRad = heading;
  arrivalTimeS = -1.0;
  safeTravelM = 0.0f;
  maximumUnknownFraction = 0.0f;

  float elapsed = 0.0f;
  while (elapsed < PLANNER_HORIZON_S - 0.000001f) {
    float integrationStepS =
      min(PLANNER_ROLLOUT_STEP_S, PLANNER_HORIZON_S - elapsed);
    if (fabs(linearMps) > 0.000001f) {
      integrationStepS = min(
        integrationStepS,
        PLANNER_ROLLOUT_MAX_SPATIAL_STEP_M / fabs(linearMps));
    }
    if (fabs(angularRadPerSec) > 0.000001f) {
      integrationStepS = min(
        integrationStepS,
        PLANNER_ROLLOUT_MAX_HEADING_STEP_RAD / fabs(angularRadPerSec));
    }
    // Keep the start of this integration segment so terminal scoring can
    // measure the closest *continuous* approach to a point goal. Comparing
    // only discrete sample endpoints can favour a slower command simply
    // because one of its samples happens to land nearer the goal.
    float segmentStartX = x;
    float segmentStartY = y;
    float segmentStartHeading = heading;
    float midpointHeading =
      heading + angularRadPerSec * integrationStepS * 0.5f;
    x += linearMps * cosf(midpointHeading) * integrationStepS;
    y += linearMps * sinf(midpointHeading) * integrationStepS;
    heading += angularRadPerSec * integrationStepS;
    // Project the point goal onto this segment. The robot stops as soon as it
    // enters the arrival circle, so a fast candidate must not lose merely
    // because its next discrete rollout sample has passed that point.
    float segmentDx = x - segmentStartX;
    float segmentDy = y - segmentStartY;
    float segmentLengthSquared = segmentDx * segmentDx + segmentDy * segmentDy;
    float segmentProgress = 0.0;
    if (segmentLengthSquared > 0.0000001f) {
      segmentProgress = ((goalX - segmentStartX) * segmentDx +
                         (goalY - segmentStartY) * segmentDy) /
                        segmentLengthSquared;
      segmentProgress = constrain(segmentProgress, 0.0f, 1.0f);
    }
    float closestX = segmentStartX + segmentDx * segmentProgress;
    float closestY = segmentStartY + segmentDy * segmentProgress;
    float goalDistanceM = sqrtf((goalX - closestX) * (goalX - closestX) +
                                (goalY - closestY) * (goalY - closestY));
    if (goalDistanceM < closestGoalDistanceM) {
      closestGoalDistanceM = goalDistanceM;
      headingAtClosestGoalRad = segmentStartHeading +
                                (heading - segmentStartHeading) * segmentProgress;
    }
    // Arrival time is a primary point-goal objective, not an arbitrary score
    // weight. Once a safe arc can enter the arrival circle, prefer the one
    // that gets there sooner rather than a slower arc that happens to improve
    // a secondary heading score.
    if (arrivalTimeS < 0.0 && segmentLengthSquared > 0.0000001f) {
      float fromGoalX = segmentStartX - goalX;
      float fromGoalY = segmentStartY - goalY;
      float radiusSquared = WAYPOINT_TOLERANCE_M * WAYPOINT_TOLERANCE_M;
      float b = 2.0f * (fromGoalX * segmentDx + fromGoalY * segmentDy);
      float c = fromGoalX * fromGoalX + fromGoalY * fromGoalY - radiusSquared;
      float discriminant = b * b - 4.0f * segmentLengthSquared * c;
      if (discriminant >= 0.0f) {
        float entryProgress = (-b - sqrtf(discriminant)) /
                              (2.0f * segmentLengthSquared);
        if (entryProgress >= 0.0f && entryProgress <= 1.0f) {
          arrivalTimeS = elapsed + entryProgress * integrationStepS;
          if (epoch.finalWaypointIsLocalGoal) {
            float arrivalX = segmentStartX + segmentDx * entryProgress;
            float arrivalY = segmentStartY + segmentDy * entryProgress;
            float arrivalHeading = segmentStartHeading +
              (heading - segmentStartHeading) * entryProgress;
            float arrivalTravelM = linearMps * arrivalTimeS;
            if (arrivalTravelM + leadingEnvelopeM > observedForwardM) {
              rejectReason = CANDIDATE_REJECT_FORWARD_OBSERVATION;
              return false;
            }
            if (!footprintClearOnSnapshot(epoch.collision,
                                          arrivalX, arrivalY,
                                          arrivalHeading)) {
              rejectReason = CANDIDATE_REJECT_FOOTPRINT;
              return false;
            }
            finalX = arrivalX;
            finalY = arrivalY;
            finalHeadingRad = arrivalHeading;
            closestGoalDistanceM = 0.0f;
            headingAtClosestGoalRad = arrivalHeading;
            safeTravelM = arrivalTravelM;
            return true;
          }
        }
      }
    }
    float travelledM = linearMps * (elapsed + integrationStepS);
    // Unknown front space is unsafe. Even an empty local-map cell cannot make
    // this candidate legal if the inner fan has not actually observed far
    // enough ahead of the leading edge of the chassis.
    if (travelledM + leadingEnvelopeM > observedForwardM) {
      rejectReason = CANDIDATE_REJECT_FORWARD_OBSERVATION;
      return false;
    }
    if (!footprintClearOnSnapshot(epoch.collision, x, y, heading)) {
      rejectReason = CANDIDATE_REJECT_FOOTPRINT;
      return false;
    }
    maximumUnknownFraction = max(
      maximumUnknownFraction,
      footprintUnknownFractionOnSnapshot(
        epoch.collision, x, y, heading));
    // A 400 mm passage is a straight-traverse problem, not a turning-space
    // problem. Once both nearby walls are evidenced, reject arcs that would
    // keep steering inside it. The planner therefore aligns before entry or
    // stops outside, where a safe turn/backtrack remains possible.
    if (fabs(turnTicks / max(1.0f, forwardTicks)) >
          PLANNER_CORRIDOR_MAX_TURN_RATIO &&
        isNarrowObservedCorridorOnSnapshot(epoch.collision, x, y, heading)) {
      rejectReason = CANDIDATE_REJECT_CORRIDOR;
      return false;
    }
    safeTravelM = travelledM;
    elapsed += integrationStepS;
  }
  finalX = x;
  finalY = y;
  finalHeadingRad = heading;
  return true;
}

struct ForwardCandidateEvaluation {
  float clearanceMm;
  float closestGoalDistanceM;
  float headingAtClosestGoalRad;
  float finalX;
  float finalY;
  float finalHeadingRad;
  float arrivalTimeS;
  float safeTravelM;
  float maximumUnknownFraction;
  CandidateRejectReason rejectReason;
};

static bool evaluateForwardCandidate(const PlannerEpoch &epoch,
                                     float forwardTicks, float turnTicks,
                                     ForwardCandidateEvaluation &evaluation) {
  evaluation.clearanceMm = -1.0f;
  evaluation.closestGoalDistanceM = epoch.localGoalDistanceM;
  evaluation.headingAtClosestGoalRad = epoch.startHeadingRad;
  evaluation.finalX = epoch.startX;
  evaluation.finalY = epoch.startY;
  evaluation.finalHeadingRad = epoch.startHeadingRad;
  evaluation.arrivalTimeS = -1.0f;
  evaluation.safeTravelM = 0.0f;
  evaluation.maximumUnknownFraction = 0.0f;
  evaluation.rejectReason = CANDIDATE_REJECT_NONE;
  return rolloutCandidate(
    epoch, forwardTicks, turnTicks, epoch.goalX, epoch.goalY,
    evaluation.clearanceMm, evaluation.closestGoalDistanceM,
    evaluation.headingAtClosestGoalRad,
    evaluation.finalX, evaluation.finalY, evaluation.finalHeadingRad,
    evaluation.arrivalTimeS, evaluation.safeTravelM,
    evaluation.maximumUnknownFraction, evaluation.rejectReason);
}

static bool obstacleRolloutMakesRequiredLateralProgress(
    float startX, float startY, float finalX, float finalY,
    float localGoalX, float localGoalY,
    float &lateralProgressScore) {
  lateralProgressScore = 0.0f;
  if (!plannerContext.obstacleContext.active) {
    return true;
  }
  float startDx = startX - plannerContext.obstacleContext.originX;
  float startDy = startY - plannerContext.obstacleContext.originY;
  float currentAlongM = startDx * plannerContext.obstacleContext.routeUx +
                        startDy * plannerContext.obstacleContext.routeUy;
  float currentLateralM = -startDx * plannerContext.obstacleContext.routeUy +
                           startDy * plannerContext.obstacleContext.routeUx;
  float targetDx = localGoalX - plannerContext.obstacleContext.originX;
  float targetDy = localGoalY - plannerContext.obstacleContext.originY;
  float targetLateralM =
    -targetDx * plannerContext.obstacleContext.routeUy +
     targetDy * plannerContext.obstacleContext.routeUx;
  float countersteerLeadM = plannerContext.obstacleContext.sideReconsidered
    ? PLANNER_OBSTACLE_RECONSIDERED_COUNTERSTEER_LEAD_M
    : PLANNER_OBSTACLE_COUNTERSTEER_LEAD_M;
  bool needsLateralClearance =
    currentAlongM < plannerContext.obstacleContext.nearAlongM &&
    plannerContext.obstacleContext.sideSign * currentLateralM <
      plannerContext.obstacleContext.sideSign * targetLateralM -
        countersteerLeadM;
  if (!needsLateralClearance) {
    return true;
  }
  float finalDx = finalX - plannerContext.obstacleContext.originX;
  float finalDy = finalY - plannerContext.obstacleContext.originY;
  float finalLateralM = -finalDx * plannerContext.obstacleContext.routeUy +
                         finalDy * plannerContext.obstacleContext.routeUx;
  float signedProgressM =
    plannerContext.obstacleContext.sideSign * (finalLateralM - currentLateralM);
  float remainingClearanceM =
    plannerContext.obstacleContext.sideSign * (targetLateralM - currentLateralM);
  lateralProgressScore = constrain(
    signedProgressM / max(LOCAL_MAP_CELL_M, remainingClearanceM),
    0.0f, 1.0f);
  return signedProgressM > 0.0f;
}

static float candidateScore(float forwardTicks, float turnTicks,
                            float localGoalX, float localGoalY,
                            float plannerStartX, float plannerStartY,
                            float previousSelectedTurn,
                            float closestGoalDistanceM,
                            float headingAtClosestGoalRad,
                            float finalX,
                            float finalY,
                            float finalHeadingRad,
                            float clearanceMm,
                            bool lineFollowActive,
                            float routeStartX,
                            float routeStartY,
                            float routeHeadingRad,
                            float finalGoalDistanceM) {
  float startGoalDistanceM = sqrtf(
    (localGoalX - plannerStartX) * (localGoalX - plannerStartX) +
    (localGoalY - plannerStartY) * (localGoalY - plannerStartY));
  // A point goal is complete as soon as the chassis enters its arrival
  // circle.  Score the closest point on the rollout, rather than its final
  // point after a hypothetical 0.8 s of continued driving.  Otherwise a
  // sound fast candidate is incorrectly penalised merely for passing the
  // target after the real controller would already have stopped.
  // The score deliberately mixes several modest preferences. Hard safety was
  // already checked in rolloutCandidate(); scoring is only how we choose among
  // commands that are all legal.
  float progressScore = constrain((startGoalDistanceM - closestGoalDistanceM) /
                                  max(PLANNER_MIN_PROGRESS_M, startGoalDistanceM),
                                  -1.0, 1.0);
  float desiredHeadingDeg = atan2f(localGoalY - plannerStartY,
                                   localGoalX - plannerStartX) * 180.0 / PI;
  float headingError = fabs(wrapAngle(desiredHeadingDeg -
                                      headingAtClosestGoalRad * RAD_TO_DEG));
  float curvature = fabs(turnTicks) / max(1.0f, forwardTicks);
  // 50 mm is a *preference*, not a binary wall. A tight but hard-safe arc can
  // still win when it is the only way through a valid narrow passage.
  float clearanceScore = constrain(max(0.0f, clearanceMm) /
                                   (PLANNER_PREFERRED_CLEARANCE_M * 1000.0f),
                                   0.0f, 1.0f);
  float headingScore = 1.0 - min(1.0f, headingError / 90.0f);
  // Discourage left-right twitching between successive 40 ms replans. This is
  // intentionally a soft bias so safety/progress can always override it.
  float smoothnessScore = 1.0 - min(1.0f, fabs(turnTicks - previousSelectedTurn) /
                                          max(1.0f, baseTargetSpeed));
  const float curvaturePenaltyWeight = 0.8f;
  float lineScore = 0.0f;
  float routeHeadingScore = 0.0f;
  float nearGoalStraightPenalty = 0.0f;
  if (lineFollowActive) {
    float routeDx = cosf(routeHeadingRad);
    float routeDy = sinf(routeHeadingRad);
    float lateralErrorM = fabs(-(finalX - routeStartX) * routeDy +
                               (finalY - routeStartY) * routeDx);
    lineScore = 1.0f - min(1.0f, lateralErrorM /
                                  PLANNER_LINE_FOLLOW_LATERAL_TOLERANCE_M);
    float routeHeadingErrorDeg = fabs(wrapAngle(routeHeadingRad * RAD_TO_DEG -
                                                finalHeadingRad * RAD_TO_DEG));
    routeHeadingScore = 1.0f - min(1.0f, routeHeadingErrorDeg /
                                          PLANNER_LINE_FOLLOW_HEADING_TOLERANCE_DEG);
    if (finalGoalDistanceM <= PLANNER_NEAR_GOAL_STRAIGHTEN_DISTANCE_M) {
      nearGoalStraightPenalty = 1.4f * curvature;
    }
  }
  return 3.0 * progressScore + 2.2 * headingScore + 1.4 * clearanceScore +
         0.8 * smoothnessScore + 1.6 * lineScore + 1.0 * routeHeadingScore -
         curvaturePenaltyWeight * curvature - nearGoalStraightPenalty;
}

static float requestedPointGoalSpeedCap() {
  return min(baseTargetSpeed, PLANNER_FORWARD_MAX_SPEED_TPS);
}

static void capturePlannerEpochView(PlannerEpoch &epoch) {
  epoch.startX = robotX;
  epoch.startY = robotY;
  epoch.startHeadingRad = plannerNavigationHeadingRad();
  epoch.rightInnerValid = isRangeSensorValid(RANGE_RIGHT_INNER);
  epoch.leftInnerValid = isRangeSensorValid(RANGE_LEFT_INNER);
  epoch.rightOuterValid = isRangeSensorValid(RANGE_RIGHT_OUTER);
  epoch.leftOuterValid = isRangeSensorValid(RANGE_LEFT_OUTER);
  epoch.observedRightInnerM = epoch.rightInnerValid
    ? plannerMapFanForwardObservationDistanceM(RANGE_RIGHT_INNER) : 0.0f;
  epoch.observedLeftInnerM = epoch.leftInnerValid
    ? plannerMapFanForwardObservationDistanceM(RANGE_LEFT_INNER) : 0.0f;
  epoch.minimumFanClearanceMm = minimumFanSweepClearanceMm();
  plannerMapCaptureCollisionSnapshot(epoch.collision);
}

static void recordPlannerSlice(unsigned long sliceStartedUs) {
  unsigned long sliceUs = micros() - sliceStartedUs;
  plannerContext.plannerEpoch.accumulatedWorkUs += sliceUs;
  plannerTelemetry.plannerSliceUs = sliceUs;
  plannerTelemetry.plannerSliceMaxUs =
    max(plannerTelemetry.plannerSliceMaxUs, sliceUs);
  plannerTelemetry.plannerEpochWorkUs = plannerContext.plannerEpoch.accumulatedWorkUs;
  plannerTelemetry.plannerEpochMaxWorkUs =
    max(plannerTelemetry.plannerEpochMaxWorkUs,
        plannerContext.plannerEpoch.accumulatedWorkUs);
  recordMainLoopPhaseDuration("planner_slice", sliceStartedUs);
}

static void notePlannerPending() {
  plannerContext.plannerEpoch.yieldCount++;
  plannerTelemetry.plannerYieldCount = plannerContext.plannerEpoch.yieldCount;
  plannerTelemetry.plannerEpochActive = true;
  plannerTelemetry.plannerEpochAgeMs = millis() - plannerContext.plannerEpoch.startedMs;
  plannerTelemetry.planReason = "planner_epoch_pending";
}

static TrajectoryPlanResult failPlannerEpochNoPath(const char* safeReason,
                                                   const char* replanReason) {
  stopMotors();
  plannerContext.lastForwardNoPathWasGeometric =
    plannerContext.plannerEpoch.acceptedCount == 0 &&
    plannerContext.plannerEpoch.rejectedTurnObservability == 0 &&
    plannerContext.plannerEpoch.skippedLinePolicy == 0;
  if (plannerContext.plannerEpoch.acceptedCount == 0 && !plannerContext.candidateRejectsReported) {
    char detail[224];
    snprintf(detail, sizeof(detail),
             "turn=%d;observed=%d;footprint=%d;corridor=%d;policy=%d;fp=%.3f/%.3f@%d/%d;corr=%.2f/%.2f",
             plannerContext.plannerEpoch.rejectedTurnObservability,
             plannerContext.plannerEpoch.rejectedForwardObservation,
             plannerContext.plannerEpoch.rejectedFootprint,
             plannerContext.plannerEpoch.rejectedCorridor,
             plannerContext.plannerEpoch.skippedLinePolicy,
             plannerContext.lastFootprintRejectWorldX, plannerContext.lastFootprintRejectWorldY,
             plannerContext.lastFootprintRejectCellX, plannerContext.lastFootprintRejectCellY,
             plannerContext.lastCorridorRejectLeftM, plannerContext.lastCorridorRejectRightM);
    sendBluetoothEvent("planner_candidate_rejects", detail);
    plannerContext.candidateRejectsReported = true;
  }
  plannerTelemetry.candidateCount = plannerContext.plannerEpoch.acceptedCount;
  plannerTelemetry.stopReason = PLANNER_STOP_NO_SAFE_TRAJECTORY;
  plannerTelemetry.safeStopReason = safeReason;
  plannerTelemetry.replanReason = replanReason;
  closePlannerEpoch();
  return TRAJECTORY_PLAN_NO_PATH;
}

static TrajectoryPlanResult retryPlannerEpoch(PlannerStopReason stopReason,
                                              const char* safeReason,
                                              const char* replanReason) {
  // A stale snapshot or changed velocity envelope is not evidence that every
  // forward path is blocked. Stop, discard the epoch, and build a fresh one
  // without advancing the reverse-recovery debounce.
  stopMotors();
  plannerContext.lastForwardNoPathWasGeometric = false;
  plannerTelemetry.stopReason = stopReason;
  plannerTelemetry.safeStopReason = safeReason;
  plannerTelemetry.replanReason = replanReason;
  closePlannerEpoch();
  return TRAJECTORY_PLAN_RETRY;
}

static TrajectoryPlanResult beginPlannerEpoch(float goalX, float goalY) {
  // Captures a coherent planning snapshot for one local point goal.
  // goalX/goalY are world metres for the current local target, which may be a
  // lookahead point, side-escape waypoint, or final waypoint.
  memset(&plannerContext.plannerEpoch, 0, sizeof(plannerContext.plannerEpoch));
  plannerContext.lastForwardNoPathWasGeometric = false;
  plannerContext.plannerEpoch.active = true;
  plannerContext.plannerEpoch.startedMs = millis();
  plannerContext.plannerEpoch.goalStartedMs = navigationGoal.startedMs;
  plannerContext.plannerEpoch.authority = navigationGoal.authority;
  plannerContext.plannerEpoch.goalX = goalX;
  plannerContext.plannerEpoch.goalY = goalY;
  plannerContext.plannerEpoch.bestScore = -1000000.0f;
  capturePlannerEpochView(plannerContext.plannerEpoch);

  float dx = goalX - plannerContext.plannerEpoch.startX;
  float dy = goalY - plannerContext.plannerEpoch.startY;
  plannerContext.plannerEpoch.localGoalDistanceM = sqrtf(dx * dx + dy * dy);
  float finalDx = navigationGoal.targetX - plannerContext.plannerEpoch.startX;
  float finalDy = navigationGoal.targetY - plannerContext.plannerEpoch.startY;
  plannerContext.plannerEpoch.finalGoalDistanceM = sqrtf(finalDx * finalDx + finalDy * finalDy);
  plannerContext.plannerEpoch.finalWaypointIsLocalGoal =
    navigationGoal.mode == NAV_GOAL_POINT &&
    fabs(goalX - navigationGoal.targetX) < 0.001f &&
    fabs(goalY - navigationGoal.targetY) < 0.001f;
  float routeLengthM = 0.0f;
  float routeUx = 1.0f;
  float routeUy = 0.0f;
  plannerContext.plannerEpoch.lineFollowActive =
    !plannerContext.obstacleContext.active &&
    routeLineTrackingEligible(routeLengthM, routeUx, routeUy,
                              plannerContext.plannerEpoch.routeHeadingRad);
  plannerContext.plannerEpoch.requestedSpeedCap = requestedPointGoalSpeedCap();
  plannerContext.plannerEpoch.speedCap =
    calculateSpeedCapTicksPerSec(plannerContext.plannerEpoch.requestedSpeedCap);
  plannerContext.plannerEpoch.previousSelectedTurn =
    plannerTelemetry.selectedTurnTicksPerSec;

  plannerTelemetry.speedCapTicksPerSec = plannerContext.plannerEpoch.speedCap;
  plannerTelemetry.localGoalDistanceM = plannerContext.plannerEpoch.localGoalDistanceM;
  plannerTelemetry.candidateCount = 0;
  plannerTelemetry.plannerCandidatesProcessed = 0;
  plannerTelemetry.plannerYieldCount = 0;
  plannerTelemetry.plannerEpochWorkUs = 0;
  plannerTelemetry.plannerEpochAgeMs = 0;
  plannerTelemetry.plannerEpochActive = true;
  plannerContext.lastFootprintRejectWorldX = 0.0f;
  plannerContext.lastFootprintRejectWorldY = 0.0f;
  plannerContext.lastFootprintRejectCellX = -1;
  plannerContext.lastFootprintRejectCellY = -1;
  plannerContext.lastCorridorRejectLeftM = -1.0f;
  plannerContext.lastCorridorRejectRightM = -1.0f;

  if (!isRangeSensorValid(RANGE_FRONT)) {
    return retryPlannerEpoch(PLANNER_STOP_FRONT_INVALID,
                             "front_sensor_invalid",
                             "front_sensor_invalid_retry");
  }
  if (isRangeSensorBlocked(RANGE_FRONT)) {
    plannerContext.lastForwardNoPathWasGeometric = true;
    plannerTelemetry.stopReason = PLANNER_STOP_FRONT_BLOCKED;
    plannerTelemetry.safeStopReason = "front_blocked";
    plannerTelemetry.replanReason = "front_blocked";
    closePlannerEpoch();
    return TRAJECTORY_PLAN_NO_PATH;
  }
  if (plannerContext.plannerEpoch.speedCap < PLANNER_MIN_DRIVABLE_SPEED_TPS) {
    return retryPlannerEpoch(PLANNER_STOP_NO_SAFE_TRAJECTORY,
                             "speed_cap_below_drivable_min",
                             "speed_cap_retry");
  }
  return TRAJECTORY_PLAN_PENDING;
}

TrajectoryPlanResult selectTrajectory(float goalX, float goalY) {
  // Candidate selection is a cooperative epoch. Every candidate sees one
  // immutable pose/map/sensor snapshot, but only a bounded pair is evaluated
  // per main-loop pass. An incomplete epoch can never publish or renew motion.
  if (!plannerContext.plannerEpoch.active) {
    unsigned long sliceStartedUs = micros();
    TrajectoryPlanResult beginResult = beginPlannerEpoch(goalX, goalY);
    recordPlannerSlice(sliceStartedUs);
    if (beginResult == TRAJECTORY_PLAN_PENDING) {
      notePlannerPending();
    }
    return beginResult;
  }

  if (plannerContext.plannerEpoch.goalStartedMs != navigationGoal.startedMs ||
      plannerContext.plannerEpoch.authority != navigationGoal.authority) {
    resetPlannerEpoch();
    return TRAJECTORY_PLAN_ABORTED;
  }

  unsigned long now = millis();
  plannerTelemetry.plannerEpochAgeMs = now - plannerContext.plannerEpoch.startedMs;
  plannerTelemetry.plannerCommandAgeMs = plannerContext.lastPlannerCommandPublishedMs == 0
    ? 0 : now - plannerContext.lastPlannerCommandPublishedMs;
  if (!plannerContext.plannerEpoch.commandStoppedForAge && isMotorCommandLeaseArmed() &&
      plannerTelemetry.plannerCommandAgeMs >= PLANNER_COMMAND_MAX_AGE_MS) {
    stopMotors();
    plannerContext.plannerEpoch.commandStoppedForAge = true;
    plannerTelemetry.safeStopReason = "planner_command_age_guard";
  }
  if (now - plannerContext.plannerEpoch.startedMs > PLANNER_EPOCH_MAX_AGE_MS) {
    stopMotors();
    closePlannerEpoch();
    finishNavigationGoal(false, PLANNER_STOP_ABORTED, "planner_epoch_timeout");
    return TRAJECTORY_PLAN_ABORTED;
  }

  if (!plannerContext.plannerEpoch.awaitingRevalidation) {
    const int totalCandidates = 2 * PLANNER_CURVATURE_SAMPLES;
    unsigned long sliceStartedUs = micros();
    uint8_t processedThisSlice = 0;
    while (plannerContext.plannerEpoch.candidateIndex < totalCandidates) {
      if (processedThisSlice > 0 &&
          (processedThisSlice >= PLANNER_MAX_CANDIDATES_PER_SLICE ||
           micros() - sliceStartedUs >= PLANNER_SLICE_BUDGET_US)) {
        break;
      }
      int candidateIndex = plannerContext.plannerEpoch.candidateIndex++;
      plannerTelemetry.plannerCandidatesProcessed =
        plannerContext.plannerEpoch.candidateIndex;
      int speedIndex = candidateIndex / PLANNER_CURVATURE_SAMPLES;
      int curvatureIndex = candidateIndex % PLANNER_CURVATURE_SAMPLES;
      float normalized = -1.0f +
        (2.0f * curvatureIndex) / (PLANNER_CURVATURE_SAMPLES - 1);
      float requestedForward = speedIndex == 0
        ? plannerContext.plannerEpoch.speedCap
        : PLANNER_MIN_DRIVABLE_SPEED_TPS;
      float forward = min(requestedForward, 3000.0f /
        (1.0f + fabs(normalized * PLANNER_MAX_TURN_RATIO)));
      float turn = forward * normalized * PLANNER_MAX_TURN_RATIO;

      bool turnPointsToObstacleGoal = true;
      bool turnIsGentleObstacleCountersteer = true;
      if (plannerContext.obstacleContext.active) {
        float signedTurnRatio = turn / max(1.0f, forward);
        float desiredHeadingDeg = atan2f(
          plannerContext.plannerEpoch.goalY - plannerContext.plannerEpoch.startY,
          plannerContext.plannerEpoch.goalX - plannerContext.plannerEpoch.startX) * RAD_TO_DEG;
        float localHeadingErrorDeg = wrapAngle(
          desiredHeadingDeg -
          plannerContext.plannerEpoch.startHeadingRad * RAD_TO_DEG);
        float turnTowardGoalRatio =
          (localHeadingErrorDeg > 0.0f ? 1.0f : -1.0f) *
          signedTurnRatio;
        turnPointsToObstacleGoal =
          fabs(localHeadingErrorDeg) <= 10.0f ||
          turnTowardGoalRatio >= 0.15f;
        turnIsGentleObstacleCountersteer =
          turnTowardGoalRatio >=
            -PLANNER_OBSTACLE_COUNTERSTEER_MAX_RATIO;
      }
      if ((!plannerContext.plannerEpoch.countersteerFallbackPass &&
           !turnPointsToObstacleGoal) ||
          (plannerContext.plannerEpoch.countersteerFallbackPass &&
           (turnPointsToObstacleGoal ||
            !turnIsGentleObstacleCountersteer))) {
        continue;
      }

      if (plannerContext.plannerEpoch.lineFollowActive &&
          plannerContext.plannerEpoch.finalGoalDistanceM <=
            PLANNER_NEAR_GOAL_STRAIGHTEN_DISTANCE_M &&
          fabs(turn / max(1.0f, forward)) >
            PLANNER_LINE_FOLLOW_NEAR_GOAL_MAX_TURN_RATIO) {
        plannerContext.plannerEpoch.skippedLinePolicy++;
        continue;
      }
      
      if (forward < PLANNER_MIN_DRIVABLE_SPEED_TPS) {
        continue;
      }
      processedThisSlice++;

      ForwardCandidateEvaluation evaluation;
      bool rolloutAccepted = evaluateForwardCandidate(
        plannerContext.plannerEpoch, forward, turn, evaluation);
      if (!rolloutAccepted && speedIndex == 0 &&
          evaluation.rejectReason != CANDIDATE_REJECT_TURN_OBSERVABILITY) {
        // For a fixed turn ratio, speed changes how far the same geometric arc
        // is traversed during the horizon. Reuse the last fully checked prefix
        // as this curve's computed safe speed instead of selecting a separate
        // context-specific speed tier.
        float safePrefixSpeed = evaluation.safeTravelM * TICKS_PER_METRE /
                                PLANNER_HORIZON_S - 1.0f;
        if (safePrefixSpeed >= PLANNER_MIN_DRIVABLE_SPEED_TPS &&
            safePrefixSpeed < forward - 0.5f) {
          forward = safePrefixSpeed;
          turn = forward * normalized * PLANNER_MAX_TURN_RATIO;
          rolloutAccepted = evaluateForwardCandidate(
            plannerContext.plannerEpoch, forward, turn, evaluation);
        }
      }
      if (rolloutAccepted && speedIndex == 0 &&
          plannerContext.obstacleContext.active) {
        float clearanceConfidence = constrain(
          evaluation.clearanceMm /
            (PLANNER_PREFERRED_CLEARANCE_M * 1000.0f),
          0.0f, 1.0f);
        float evidenceConfidence = constrain(
          1.0f - evaluation.maximumUnknownFraction, 0.0f, 1.0f);
        float speedConfidence = plannerContext.recoveryBudget.forwardTakeoverPending
          ? 0.0f
          : min(clearanceConfidence, evidenceConfidence);
        float confidenceSpeed = PLANNER_MIN_DRIVABLE_SPEED_TPS +
          (forward - PLANNER_MIN_DRIVABLE_SPEED_TPS) * speedConfidence;
        if (confidenceSpeed < forward - 0.5f) {
          forward = confidenceSpeed;
          turn = forward * normalized * PLANNER_MAX_TURN_RATIO;
          rolloutAccepted = evaluateForwardCandidate(
            plannerContext.plannerEpoch, forward, turn, evaluation);
        }
      }
      if (!rolloutAccepted) {
        if (evaluation.rejectReason == CANDIDATE_REJECT_TURN_OBSERVABILITY) {
          plannerContext.plannerEpoch.rejectedTurnObservability++;
        } else if (evaluation.rejectReason ==
                   CANDIDATE_REJECT_FORWARD_OBSERVATION) {
          plannerContext.plannerEpoch.rejectedForwardObservation++;
        } else if (evaluation.rejectReason == CANDIDATE_REJECT_FOOTPRINT) {
          plannerContext.plannerEpoch.rejectedFootprint++;
        } else if (evaluation.rejectReason == CANDIDATE_REJECT_CORRIDOR) {
          plannerContext.plannerEpoch.rejectedCorridor++;
        }
        continue;
      }
      float obstacleLateralProgressScore = 0.0f;
      if (!obstacleRolloutMakesRequiredLateralProgress(
            plannerContext.plannerEpoch.startX, plannerContext.plannerEpoch.startY,
            evaluation.finalX, evaluation.finalY,
            plannerContext.plannerEpoch.goalX, plannerContext.plannerEpoch.goalY,
            obstacleLateralProgressScore)) {
        continue;
      }
      if (plannerContext.plannerEpoch.countersteerFallbackPass &&
          obstacleLateralProgressScore <
            PLANNER_OBSTACLE_COUNTERSTEER_PROGRESS_FRACTION) {
        continue;
      }

      plannerContext.plannerEpoch.acceptedCount++;
      plannerTelemetry.candidateCount = plannerContext.plannerEpoch.acceptedCount;
      float score = candidateScore(
        forward, turn, plannerContext.plannerEpoch.goalX, plannerContext.plannerEpoch.goalY,
        plannerContext.plannerEpoch.startX, plannerContext.plannerEpoch.startY,
        plannerContext.plannerEpoch.previousSelectedTurn,
        evaluation.closestGoalDistanceM,
        evaluation.headingAtClosestGoalRad,
        evaluation.finalX, evaluation.finalY,
        evaluation.finalHeadingRad, evaluation.clearanceMm,
        plannerContext.plannerEpoch.lineFollowActive,
        navigationGoal.startX, navigationGoal.startY,
        plannerContext.plannerEpoch.routeHeadingRad, plannerContext.plannerEpoch.finalGoalDistanceM);
      if (plannerContext.recoveryBudget.forwardTakeoverPending) {
        // After reverse has deliberately spent route progress, prefer a
        // decisive clearance gain before resuming ordinary goal scoring.
        score += 3.0f * obstacleLateralProgressScore;
      }
      bool reachesGoal = evaluation.arrivalTimeS >= 0.0f;
      bool betterArrival =
        plannerContext.plannerEpoch.finalWaypointIsLocalGoal &&
        !plannerContext.plannerEpoch.lineFollowActive && reachesGoal &&
        (!plannerContext.plannerEpoch.bestReachesGoal ||
         evaluation.arrivalTimeS <
           plannerContext.plannerEpoch.bestArrivalTimeS - 0.0001f);
      bool equalArrivalClass =
        !plannerContext.plannerEpoch.finalWaypointIsLocalGoal ||
        plannerContext.plannerEpoch.lineFollowActive ||
        (reachesGoal == plannerContext.plannerEpoch.bestReachesGoal &&
         (!reachesGoal ||
          fabs(evaluation.arrivalTimeS -
               plannerContext.plannerEpoch.bestArrivalTimeS) <= 0.0001f));
      if (betterArrival ||
          (equalArrivalClass && score > plannerContext.plannerEpoch.bestScore)) {
        plannerContext.plannerEpoch.bestScore = score;
        plannerContext.plannerEpoch.bestForward = forward;
        plannerContext.plannerEpoch.bestTurn = turn;
        plannerContext.plannerEpoch.bestReachesGoal = reachesGoal;
        plannerContext.plannerEpoch.bestArrivalTimeS = evaluation.arrivalTimeS;
      }
    }
    recordPlannerSlice(sliceStartedUs);
    if (plannerContext.plannerEpoch.candidateIndex < totalCandidates) {
      notePlannerPending();
      return TRAJECTORY_PLAN_PENDING;
    }
    if (plannerContext.plannerEpoch.acceptedCount == 0 &&
        plannerContext.obstacleContext.active &&
        !plannerContext.plannerEpoch.countersteerFallbackPass) {
      plannerContext.plannerEpoch.countersteerFallbackPass = true;
      plannerContext.plannerEpoch.candidateIndex = 0;
      plannerTelemetry.replanReason =
        "obstacle_countersteer_fallback";
      notePlannerPending();
      return TRAJECTORY_PLAN_PENDING;
    }
    plannerContext.plannerEpoch.awaitingRevalidation = true;
    notePlannerPending();
    return TRAJECTORY_PLAN_PENDING;
  }

  // The winner was selected from a coherent older snapshot. Re-run that one
  // command against the newest pose/map/sensors before it may reach the motor
  // owner. A changed hazard therefore invalidates publication, never safety.
  unsigned long sliceStartedUs = micros();
  if (plannerContext.plannerEpoch.acceptedCount == 0) {
    recordPlannerSlice(sliceStartedUs);
    return failPlannerEpochNoPath("no_footprint_safe_arc",
                                  "all_arc_candidates_rejected");
  }
  capturePlannerEpochView(plannerContext.plannerEpoch);
  if (!isRangeSensorValid(RANGE_FRONT)) {
    recordPlannerSlice(sliceStartedUs);
    return retryPlannerEpoch(PLANNER_STOP_FRONT_INVALID,
                             "front_sensor_invalid_revalidate",
                             "winner_revalidation_retry");
  }
  if (isRangeSensorBlocked(RANGE_FRONT)) {
    recordPlannerSlice(sliceStartedUs);
    plannerContext.lastForwardNoPathWasGeometric = true;
    plannerTelemetry.stopReason = PLANNER_STOP_FRONT_BLOCKED;
    plannerTelemetry.safeStopReason = "front_blocked_revalidate";
    plannerTelemetry.replanReason = "winner_revalidation_failed";
    closePlannerEpoch();
    return TRAJECTORY_PLAN_NO_PATH;
  }
  float freshSpeedCap =
    calculateSpeedCapTicksPerSec(plannerContext.plannerEpoch.requestedSpeedCap);
  if (freshSpeedCap < PLANNER_MIN_DRIVABLE_SPEED_TPS) {
    recordPlannerSlice(sliceStartedUs);
    return retryPlannerEpoch(PLANNER_STOP_NO_SAFE_TRAJECTORY,
                             "winner_speed_cap_below_drivable_min",
                             "winner_speed_cap_retry");
  }
  float publishForward = plannerContext.plannerEpoch.bestForward;
  float publishTurn = plannerContext.plannerEpoch.bestTurn;
  if (publishForward > freshSpeedCap + 0.5f) {
    float speedScale = freshSpeedCap / publishForward;
    publishForward = freshSpeedCap;
    publishTurn *= speedScale;
  }
  ForwardCandidateEvaluation evaluation;
  bool winnerStillSafe = evaluateForwardCandidate(
    plannerContext.plannerEpoch, publishForward, publishTurn, evaluation);
  recordPlannerSlice(sliceStartedUs);
  if (!winnerStillSafe) {
    return retryPlannerEpoch(PLANNER_STOP_NO_SAFE_TRAJECTORY,
                             "winner_revalidation_rejected",
                             "winner_revalidation_retry");
  }
  float obstacleLateralProgressScore = 0.0f;
  if (!obstacleRolloutMakesRequiredLateralProgress(
        plannerContext.plannerEpoch.startX, plannerContext.plannerEpoch.startY,
        evaluation.finalX, evaluation.finalY,
        plannerContext.plannerEpoch.goalX, plannerContext.plannerEpoch.goalY,
        obstacleLateralProgressScore)) {
    return retryPlannerEpoch(PLANNER_STOP_NO_SAFE_TRAJECTORY,
                             "winner_obstacle_progress_rejected",
                             "winner_revalidation_retry");
  }

  plannerTelemetry.selectedForwardTicksPerSec = publishForward;
  plannerTelemetry.selectedTurnTicksPerSec = publishTurn;
  plannerTelemetry.selectedCurvature = publishTurn /
    max(1.0f, publishForward);
  plannerTelemetry.minimumSweptClearanceMm = evaluation.clearanceMm;
  plannerTelemetry.candidateCount = plannerContext.plannerEpoch.acceptedCount;
  plannerTelemetry.stopReason = PLANNER_STOP_NONE;
  plannerTelemetry.planReason = "best_safe_arc_revalidated";
  plannerTelemetry.replanReason = "local_goal_visible";
  plannerTelemetry.safeStopReason = "";
  plannerContext.lastReportedStopReason = PLANNER_STOP_NONE;
  plannerContext.candidateRejectsReported = false;
  motorStopRequested = false;
  bool published = publishNavigationMotion(publishForward, publishTurn);
  if (!published) {
    return retryPlannerEpoch(PLANNER_STOP_NO_SAFE_TRAJECTORY,
                             "winner_publication_vetoed",
                             "winner_publication_retry");
  }
  plannerContext.lastPlannerCommandPublishedMs = millis();
  plannerTelemetry.plannerCommandAgeMs = 0;
  plannerTelemetry.lastPlanMs = plannerContext.lastPlannerCommandPublishedMs;
  lastPlannerUpdateMs = plannerContext.lastPlannerCommandPublishedMs;
  closePlannerEpoch();
  return TRAJECTORY_PLAN_SUCCESS;
}
