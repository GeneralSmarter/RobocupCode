#include "../../Robot.h"
#include "ForwardTrajectoryPlanner.h"
#include "ObstacleContext.h"
#include "PlannerCollision.h"
#include "PlannerContext.h"
#include "PlannerMap.h"
#include "PlannerProgress.h"




void resetObstacleContext(const char* reason) {
  bool wasActive = plannerContext.obstacleContext.active;
  plannerContext.obstacleContext = {};
  if (wasActive) {
    plannerTelemetry.replanReason = reason;
    sendBluetoothEvent("obstacle_context_clear", reason);
  }
}

static void obstacleCellRoutePosition(int cellX, int cellY,
                                      float originX, float originY,
                                      float routeUx, float routeUy,
                                      float &alongM, float &lateralM) {
  float worldX;
  float worldY;
  plannerMapCellCenter(cellX, cellY, worldX, worldY);
  float dx = worldX - originX;
  float dy = worldY - originY;
  alongM = dx * routeUx + dy * routeUy;
  lateralM = -dx * routeUy + dy * routeUx;
}

static void growObstacleEnvelopeFromNearbyEvidence(
    float originX, float originY,
    float routeUx, float routeUy,
    float routeLengthM,
    ObstacleEnvelope &envelope) {
  if (!envelope.found) {
    return;
  }

  // Side-facing rays observe a long wall in separate endpoint patches. Grow
  // from the retained envelope so a newly observed patch can extend the same
  // obstacle even when one 50 mm map row between them has no endpoint return.
  const float joinM = LOCAL_MAP_CELL_M * 2.75f;
  const float endpointReachM = routeLengthM +
                               ROBOT_FOOTPRINT_GEOMETRY.frontExtentMm / 1000.0f +
                               PLANNER_TOTAL_HARD_CLEARANCE_M;
  for (int pass = 0; pass < 12; pass++) {
    bool expanded = false;
    for (int y = 0; y < LOCAL_MAP_CELLS; y++) {
      for (int x = 0; x < LOCAL_MAP_CELLS; x++) {
        if (!plannerMapCellOccupied(x, y)) {
          continue;
        }
        float alongM;
        float lateralM;
        obstacleCellRoutePosition(x, y, originX, originY,
                                  routeUx, routeUy, alongM, lateralM);
        if (alongM < -joinM || alongM > endpointReachM + joinM ||
            alongM < envelope.nearAlongM - joinM ||
            alongM > envelope.farAlongM + joinM ||
            lateralM < envelope.minLateralM - joinM ||
            lateralM > envelope.maxLateralM + joinM) {
          continue;
        }
        float oldNear = envelope.nearAlongM;
        float oldFar = envelope.farAlongM;
        float oldMin = envelope.minLateralM;
        float oldMax = envelope.maxLateralM;
        envelope.nearAlongM = min(envelope.nearAlongM, alongM);
        envelope.farAlongM = max(envelope.farAlongM, alongM);
        envelope.minLateralM = min(envelope.minLateralM, lateralM);
        envelope.maxLateralM = max(envelope.maxLateralM, lateralM);
        expanded = expanded || oldNear != envelope.nearAlongM ||
                   oldFar != envelope.farAlongM ||
                   oldMin != envelope.minLateralM ||
                   oldMax != envelope.maxLateralM;
      }
    }
    if (!expanded) {
      break;
    }
  }
}

static bool scanRouteObstacle(float originX, float originY,
                              float routeUx, float routeUy,
                              float routeLengthM,
                              ObstacleEnvelope &envelope) {
  envelope = {};
  float halfWidthM = max(ROBOT_FOOTPRINT_GEOMETRY.leftExtentMm,
                         ROBOT_FOOTPRINT_GEOMETRY.rightExtentMm) / 1000.0f +
                     PLANNER_TOTAL_HARD_CLEARANCE_M +
                     LOCAL_MAP_CELL_M * 0.5f;
  float endpointReachM = routeLengthM +
                         ROBOT_FOOTPRINT_GEOMETRY.frontExtentMm / 1000.0f +
                         PLANNER_TOTAL_HARD_CLEARANCE_M;
  int seedX = -1;
  int seedY = -1;
  float nearestAlongM = 1000000.0f;
  float seedLateralM = 0.0f;

  for (int y = 0; y < LOCAL_MAP_CELLS; y++) {
    for (int x = 0; x < LOCAL_MAP_CELLS; x++) {
      if (!plannerMapCellOccupied(x, y)) {
        continue;
      }
      float alongM;
      float lateralM;
      obstacleCellRoutePosition(x, y, originX, originY,
                                routeUx, routeUy, alongM, lateralM);
      if (alongM >= 0.0f && alongM <= endpointReachM &&
          fabs(lateralM) <= halfWidthM && alongM < nearestAlongM) {
        nearestAlongM = alongM;
        seedLateralM = lateralM;
        seedX = x;
        seedY = y;
      }
    }
  }
  if (seedX < 0 || seedY < 0) {
    return false;
  }

  envelope.found = true;
  envelope.nearAlongM = nearestAlongM;
  envelope.farAlongM = nearestAlongM;
  envelope.minLateralM = seedLateralM;
  envelope.maxLateralM = seedLateralM;
  growObstacleEnvelopeFromNearbyEvidence(
    originX, originY, routeUx, routeUy, routeLengthM, envelope);
  return true;
}

static float obstacleTargetMinimumLateralM() {
  return
    max(ROBOT_FOOTPRINT_GEOMETRY.leftExtentMm,
        ROBOT_FOOTPRINT_GEOMETRY.rightExtentMm) / 1000.0f +
    PLANNER_TOTAL_HARD_CLEARANCE_M + LOCAL_MAP_CELL_M * 0.5f;
}

static float obstacleTargetLateralM(float sideSign,
                                    float minimumLateralM,
                                    float maximumLateralM) {
  float lateralClearanceM = obstacleTargetMinimumLateralM();
  float targetLateralM = sideSign > 0.0f
    ? maximumLateralM + lateralClearanceM
    : minimumLateralM - lateralClearanceM;
  return sideSign > 0.0f
    ? max(targetLateralM, lateralClearanceM)
    : min(targetLateralM, -lateralClearanceM);
}

static void obstacleTargetWorldPosition(
    float routeUx, float routeUy,
    float targetAlongM, float targetLateralM,
    float &targetX, float &targetY) {
  targetX = plannerContext.obstacleContext.originX +
            routeUx * targetAlongM - routeUy * targetLateralM;
  targetY = plannerContext.obstacleContext.originY +
            routeUy * targetAlongM + routeUx * targetLateralM;
}

static bool obstacleTargetPoseClear(
    const PlannerCollisionSnapshot &snapshot,
    float routeUx, float routeUy,
    float targetAlongM, float targetLateralM) {
  float targetX;
  float targetY;
  obstacleTargetWorldPosition(
    routeUx, routeUy, targetAlongM, targetLateralM,
    targetX, targetY);
  float approachHeadingRad = atan2f(targetY - robotY,
                                    targetX - robotX);
  return footprintClearOnSnapshot(
    snapshot, targetX, targetY, approachHeadingRad);
}

static bool chooseInsideBandTargetLateral(
    const PlannerCollisionSnapshot &snapshot,
    float routeUx, float routeUy, float sideSign,
    float targetAlongM, float nominalTargetLateralM,
  float &selectedTargetLateralM) {
  selectedTargetLateralM = nominalTargetLateralM;
  if (obstacleTargetPoseClear(
        snapshot, routeUx, routeUy,
        targetAlongM, nominalTargetLateralM)) {
    return true;
  }

  const float minimumSignedLateralM =
    obstacleTargetMinimumLateralM();
  const float nominalSignedLateralM =
    sideSign * nominalTargetLateralM;
  const float maximumInwardShiftM =
    nominalSignedLateralM - minimumSignedLateralM;
  if (maximumInwardShiftM <= 0.0f) {
    return false;
  }

  const float sampleStepM = LOCAL_MAP_CELL_M * 0.5f;
  const int sampleCount = max(
    1, (int)ceilf(maximumInwardShiftM / sampleStepM));
  bool foundSafeBand = false;
  float firstSafeInwardShiftM = 0.0f;
  float outerSafeLateralM = nominalTargetLateralM;
  float innerSafeLateralM = nominalTargetLateralM;
  for (int sample = 1; sample <= sampleCount; ++sample) {
    float inwardShiftM = min(
      maximumInwardShiftM, sample * sampleStepM);
    float candidateLateralM =
      nominalTargetLateralM - sideSign * inwardShiftM;
    bool clear = obstacleTargetPoseClear(
      snapshot, routeUx, routeUy,
      targetAlongM, candidateLateralM);
    if (!clear) {
      if (foundSafeBand) {
        break;
      }
      continue;
    }
    if (!foundSafeBand) {
      firstSafeInwardShiftM = inwardShiftM;
      outerSafeLateralM = candidateLateralM;
      foundSafeBand = true;
    }
    innerSafeLateralM = candidateLateralM;
  }
  if (!foundSafeBand) {
    return false;
  }
  if (firstSafeInwardShiftM <= sampleStepM + 0.001f) {
    return true;
  }

  float signedOuterM = sideSign * outerSafeLateralM;
  float signedInnerM = sideSign * innerSafeLateralM;
  float safeBandWidthM = max(0.0f, signedOuterM - signedInnerM);
  float insideBufferM = min(
    safeBandWidthM,
    PLANNER_PREFERRED_CLEARANCE_M * 0.5f);
  selectedTargetLateralM =
    innerSafeLateralM + sideSign * insideBufferM;
  return true;
}

static void obstacleSideEscapeTarget(
    const ObstacleEnvelope &envelope,
    float routeUx, float routeUy,
    float sideSign,
    float &targetX, float &targetY) {
  float frontClearM = ROBOT_FOOTPRINT_GEOMETRY.frontExtentMm / 1000.0f +
                      PLANNER_TOTAL_HARD_CLEARANCE_M +
                      LOCAL_MAP_CELL_M * 0.5f;
  float targetAlongM = max(0.05f, envelope.nearAlongM - frontClearM -
                                  PLANNER_OBSTACLE_TURN_ROOM_M);
  float targetLateralM = obstacleTargetLateralM(
    sideSign, envelope.minLateralM, envelope.maxLateralM);
  obstacleTargetWorldPosition(
    routeUx, routeUy, targetAlongM, targetLateralM,
    targetX, targetY);
}

static bool obstacleSideEscapeCorridorBlocked(
    const PlannerCollisionSnapshot &snapshot,
    const ObstacleEnvelope &envelope,
    float routeUx, float routeUy, float sideSign) {
  float targetX;
  float targetY;
  obstacleSideEscapeTarget(
    envelope, routeUx, routeUy,
    sideSign, targetX, targetY);
  float dx = targetX - robotX;
  float dy = targetY - robotY;
  float distanceM = sqrtf(dx * dx + dy * dy);
  if (distanceM <= LOCAL_MAP_CELL_M) {
    return false;
  }
  float stepM = LOCAL_MAP_CELL_M * 0.5f;
  for (float travelledM = stepM;
       travelledM <= distanceM + 0.001f;
       travelledM += stepM) {
    float progress = min(1.0f, travelledM / distanceM);
    if (snapshotWorldOccupied(
          snapshot,
          robotX + dx * progress,
          robotY + dy * progress)) {
      return true;
    }
  }
  return false;
}

static float chooseObstacleSide(const ObstacleEnvelope &envelope,
                                float routeUx, float routeUy) {
  float leftTargetM = obstacleTargetLateralM(
    1.0f, envelope.minLateralM, envelope.maxLateralM);
  float rightTargetM = obstacleTargetLateralM(
    -1.0f, envelope.minLateralM, envelope.maxLateralM);
  PlannerCollisionSnapshot collision;
  plannerMapCaptureCollisionSnapshot(collision);
  bool leftCorridorBlocked = obstacleSideEscapeCorridorBlocked(
    collision, envelope, routeUx, routeUy, 1.0f);
  bool rightCorridorBlocked = obstacleSideEscapeCorridorBlocked(
    collision, envelope, routeUx, routeUy, -1.0f);
  if (leftCorridorBlocked != rightCorridorBlocked) {
    return leftCorridorBlocked ? -1.0f : 1.0f;
  }
  float dx = robotX - plannerContext.obstacleContext.originX;
  float dy = robotY - plannerContext.obstacleContext.originY;
  float currentLateralM = -dx * routeUy + dy * routeUx;
  float frontClearM = ROBOT_FOOTPRINT_GEOMETRY.frontExtentMm / 1000.0f +
                      PLANNER_TOTAL_HARD_CLEARANCE_M +
                      LOCAL_MAP_CELL_M * 0.5f;
  float targetAlongM = max(0.05f, envelope.nearAlongM - frontClearM -
                                  PLANNER_OBSTACLE_TURN_ROOM_M);
  float robotHeadingRad = robotTheta * PI / 180.0f;
  float headingX = cosf(robotHeadingRad);
  float headingY = sinf(robotHeadingRad);
  float leftForwardM =
    (routeUx * targetAlongM - routeUy * leftTargetM) * headingX +
    (routeUy * targetAlongM + routeUx * leftTargetM) * headingY;
  float rightForwardM =
    (routeUx * targetAlongM - routeUy * rightTargetM) * headingX +
    (routeUy * targetAlongM + routeUx * rightTargetM) * headingY;
  if (leftForwardM > 0.0f && rightForwardM <= 0.0f) {
    return 1.0f;
  }
  if (rightForwardM > 0.0f && leftForwardM <= 0.0f) {
    return -1.0f;
  }
  float leftCostM = fabs(leftTargetM - currentLateralM);
  float rightCostM = fabs(rightTargetM - currentLateralM);
  if (fabs(leftCostM - rightCostM) > LOCAL_MAP_CELL_M) {
    return leftCostM < rightCostM ? 1.0f : -1.0f;
  }
  float leftRangeM = isRangeSensorValid(RANGE_LEFT_INNER)
    ? getRangeSensorDistance(RANGE_LEFT_INNER) / 1000.0f : 0.0f;
  float rightRangeM = isRangeSensorValid(RANGE_RIGHT_INNER)
    ? getRangeSensorDistance(RANGE_RIGHT_INNER) / 1000.0f : 0.0f;
  return leftRangeM >= rightRangeM ? 1.0f : -1.0f;
}

static bool directWaypointCorridorClear(float targetX, float targetY) {
  float dx = targetX - robotX;
  float dy = targetY - robotY;
  float distanceM = sqrtf(dx * dx + dy * dy);
  if (distanceM <= WAYPOINT_TOLERANCE_M) {
    return true;
  }
  ObstacleEnvelope directObstacle;
  return !scanRouteObstacle(robotX, robotY, dx / distanceM, dy / distanceM,
                            distanceM, directObstacle);
}

static bool clipDirectSegmentToEnvelopeAxis(float start, float delta,
                                            float minimum, float maximum,
                                            float &entry, float &exit) {
  if (fabs(delta) < 0.000001f) {
    return start >= minimum && start <= maximum;
  }
  float first = (minimum - start) / delta;
  float second = (maximum - start) / delta;
  if (first > second) {
    float swap = first;
    first = second;
    second = swap;
  }
  entry = max(entry, first);
  exit = min(exit, second);
  return entry <= exit;
}

static bool directSegmentClearsRetainedObstacle(float targetX, float targetY) {
  float currentDx = robotX - plannerContext.obstacleContext.originX;
  float currentDy = robotY - plannerContext.obstacleContext.originY;
  float currentAlongM = currentDx * plannerContext.obstacleContext.routeUx +
                        currentDy * plannerContext.obstacleContext.routeUy;
  float currentLateralM = -currentDx * plannerContext.obstacleContext.routeUy +
                          currentDy * plannerContext.obstacleContext.routeUx;
  float targetDx = targetX - plannerContext.obstacleContext.originX;
  float targetDy = targetY - plannerContext.obstacleContext.originY;
  float targetAlongM = targetDx * plannerContext.obstacleContext.routeUx +
                       targetDy * plannerContext.obstacleContext.routeUy;
  float targetLateralM = -targetDx * plannerContext.obstacleContext.routeUy +
                         targetDy * plannerContext.obstacleContext.routeUx;
  float alongClearanceM =
    max(ROBOT_FOOTPRINT_GEOMETRY.frontExtentMm,
        ROBOT_FOOTPRINT_GEOMETRY.rearExtentMm) / 1000.0f +
    PLANNER_TOTAL_HARD_CLEARANCE_M + LOCAL_MAP_CELL_M * 0.5f;
  float lateralClearanceM =
    max(ROBOT_FOOTPRINT_GEOMETRY.leftExtentMm,
        ROBOT_FOOTPRINT_GEOMETRY.rightExtentMm) / 1000.0f +
    PLANNER_TOTAL_HARD_CLEARANCE_M + LOCAL_MAP_CELL_M * 0.5f;
  float entry = 0.0f;
  float exit = 1.0f;
  bool crossesAlong = clipDirectSegmentToEnvelopeAxis(
    currentAlongM, targetAlongM - currentAlongM,
    plannerContext.obstacleContext.nearAlongM - alongClearanceM,
    plannerContext.obstacleContext.farAlongM + alongClearanceM,
    entry, exit);
  bool crossesLateral = crossesAlong && clipDirectSegmentToEnvelopeAxis(
    currentLateralM, targetLateralM - currentLateralM,
    plannerContext.obstacleContext.minLateralM - lateralClearanceM,
    plannerContext.obstacleContext.maxLateralM + lateralClearanceM,
    entry, exit);
  return !crossesLateral;
}

bool updateObstacleContext(float targetX, float targetY) {
  if (!plannerContext.obstacleContext.active) {
    float dx = targetX - robotX;
    float dy = targetY - robotY;
    float distanceM = sqrtf(dx * dx + dy * dy);
    if (distanceM <= WAYPOINT_TOLERANCE_M) {
      return false;
    }
    ObstacleEnvelope envelope;
    float routeUx = dx / distanceM;
    float routeUy = dy / distanceM;
    if (!scanRouteObstacle(robotX, robotY, routeUx, routeUy,
                           distanceM, envelope)) {
      return false;
    }
    plannerContext.obstacleContext.active = true;
    plannerContext.obstacleContext.originX = robotX;
    plannerContext.obstacleContext.originY = robotY;
    plannerContext.obstacleContext.routeUx = routeUx;
    plannerContext.obstacleContext.routeUy = routeUy;
    plannerContext.obstacleContext.routeLengthM = distanceM;
    // Envelope growth may join a perpendicular arena boundary behind us.
    // Keep the originally observed front face as the forward-phase gate.
    plannerContext.obstacleContext.approachNearAlongM = envelope.nearAlongM;
    plannerContext.obstacleContext.nearAlongM = envelope.nearAlongM;
    plannerContext.obstacleContext.farAlongM = envelope.farAlongM;
    plannerContext.obstacleContext.minLateralM = envelope.minLateralM;
    plannerContext.obstacleContext.maxLateralM = envelope.maxLateralM;
    plannerContext.obstacleContext.sideSign = chooseObstacleSide(envelope, routeUx, routeUy);
    plannerContext.obstacleContext.sideEscapeAlongM = 0.0f;
    plannerContext.obstacleContext.sideReconsidered = false;
    plannerContext.obstacleContext.clearSinceMs = 0;
    plannerTelemetry.replanReason = "obstacle_context_started";
    sendBluetoothEvent("obstacle_context_start",
                       plannerContext.obstacleContext.sideSign > 0.0f ? "left" : "right");
    resetPlannerEpoch();
    return true;
  }

  ObstacleEnvelope observed = {
    true,
    plannerContext.obstacleContext.nearAlongM,
    plannerContext.obstacleContext.farAlongM,
    plannerContext.obstacleContext.minLateralM,
    plannerContext.obstacleContext.maxLateralM
  };
  float previousNearAlongM = observed.nearAlongM;
  float previousFarAlongM = observed.farAlongM;
  float previousMinLateralM = observed.minLateralM;
  float previousMaxLateralM = observed.maxLateralM;
  growObstacleEnvelopeFromNearbyEvidence(
    plannerContext.obstacleContext.originX,
    plannerContext.obstacleContext.originY,
    plannerContext.obstacleContext.routeUx,
    plannerContext.obstacleContext.routeUy,
    plannerContext.obstacleContext.routeLengthM,
    observed);
  plannerContext.obstacleContext.nearAlongM = observed.nearAlongM;
  plannerContext.obstacleContext.farAlongM = observed.farAlongM;
  plannerContext.obstacleContext.minLateralM = observed.minLateralM;
  plannerContext.obstacleContext.maxLateralM = observed.maxLateralM;
  bool envelopeExpanded =
    observed.nearAlongM != previousNearAlongM ||
    observed.farAlongM != previousFarAlongM ||
    observed.minLateralM != previousMinLateralM ||
    observed.maxLateralM != previousMaxLateralM;
  if (envelopeExpanded && !plannerContext.obstacleContext.sideReconsidered) {
    PlannerCollisionSnapshot collision;
    plannerMapCaptureCollisionSnapshot(collision);
    bool currentSideBlocked = obstacleSideEscapeCorridorBlocked(
      collision, observed,
      plannerContext.obstacleContext.routeUx, plannerContext.obstacleContext.routeUy,
      plannerContext.obstacleContext.sideSign);
    bool oppositeSideBlocked = obstacleSideEscapeCorridorBlocked(
      collision, observed,
      plannerContext.obstacleContext.routeUx, plannerContext.obstacleContext.routeUy,
      -plannerContext.obstacleContext.sideSign);
    if (currentSideBlocked && !oppositeSideBlocked) {
      plannerContext.obstacleContext.sideSign = -plannerContext.obstacleContext.sideSign;
      float switchDx = robotX - plannerContext.obstacleContext.originX;
      float switchDy = robotY - plannerContext.obstacleContext.originY;
      plannerContext.obstacleContext.sideEscapeAlongM =
        switchDx * plannerContext.obstacleContext.routeUx +
        switchDy * plannerContext.obstacleContext.routeUy;
      plannerContext.obstacleContext.sideReconsidered = true;
      plannerContext.obstacleContext.clearSinceMs = 0;
      plannerTelemetry.replanReason = "obstacle_side_infeasible";
      sendBluetoothEvent(
        "obstacle_side_switch",
        plannerContext.obstacleContext.sideSign > 0.0f ? "left" : "right");
      resetPlannerEpoch();
    }
  }

  float poseDx = robotX - plannerContext.obstacleContext.originX;
  float poseDy = robotY - plannerContext.obstacleContext.originY;
  float alongM = poseDx * plannerContext.obstacleContext.routeUx +
                 poseDy * plannerContext.obstacleContext.routeUy;
  float currentLateralM = -poseDx * plannerContext.obstacleContext.routeUy +
                          poseDy * plannerContext.obstacleContext.routeUx;
  float lateralClearanceM =
    max(ROBOT_FOOTPRINT_GEOMETRY.leftExtentMm,
        ROBOT_FOOTPRINT_GEOMETRY.rightExtentMm) / 1000.0f +
    PLANNER_TOTAL_HARD_CLEARANCE_M + LOCAL_MAP_CELL_M * 0.5f;
  float releaseLateralM = plannerContext.obstacleContext.sideSign > 0.0f
    ? plannerContext.obstacleContext.maxLateralM + lateralClearanceM
    : plannerContext.obstacleContext.minLateralM - lateralClearanceM;
  bool outsideObstacleSide =
    plannerContext.obstacleContext.sideSign * currentLateralM >=
    plannerContext.obstacleContext.sideSign * releaseLateralM;
  bool reachedObstacleApproach =
    alongM >= plannerContext.obstacleContext.approachNearAlongM -
                LOCAL_MAP_CELL_M * 0.5f;
  bool directRelease = reachedObstacleApproach &&
                       outsideObstacleSide &&
                       directSegmentClearsRetainedObstacle(targetX, targetY) &&
                       directWaypointCorridorClear(targetX, targetY);
  float rearClearM = ROBOT_FOOTPRINT_GEOMETRY.rearExtentMm / 1000.0f +
                     PLANNER_TOTAL_HARD_CLEARANCE_M +
                     LOCAL_MAP_CELL_M * 0.5f;
  bool rearPassedObstacle =
    alongM >= plannerContext.obstacleContext.farAlongM + rearClearM;
  if (directRelease || rearPassedObstacle) {
    if (plannerContext.obstacleContext.clearSinceMs == 0) {
      plannerContext.obstacleContext.clearSinceMs = millis();
    }
    if (millis() - plannerContext.obstacleContext.clearSinceMs >= 80) {
      resetObstacleContext(directRelease
        ? "direct_waypoint_corridor_clear" : "obstacle_cleared");
      resetPlannerEpoch();
      // The completed envelope owns only this obstacle. Reacquire immediately
      // so a later blocker gets a fresh envelope and independent side choice.
      return updateObstacleContext(targetX, targetY);
    }
  } else {
    plannerContext.obstacleContext.clearSinceMs = 0;
  }
  return true;
}

void buildObstacleLocalGoal(float &localGoalX, float &localGoalY) {
  float rearClearM = ROBOT_FOOTPRINT_GEOMETRY.rearExtentMm / 1000.0f +
                     PLANNER_TOTAL_HARD_CLEARANCE_M +
                     LOCAL_MAP_CELL_M * 0.5f;
  float nominalTargetLateralM = obstacleTargetLateralM(
    plannerContext.obstacleContext.sideSign,
    plannerContext.obstacleContext.minLateralM,
    plannerContext.obstacleContext.maxLateralM);
  float poseDx = robotX - plannerContext.obstacleContext.originX;
  float poseDy = robotY - plannerContext.obstacleContext.originY;
  float currentAlongM = poseDx * plannerContext.obstacleContext.routeUx +
                        poseDy * plannerContext.obstacleContext.routeUy;
  float currentLateralM = -poseDx * plannerContext.obstacleContext.routeUy +
                          poseDy * plannerContext.obstacleContext.routeUx;
  float countersteerLeadM = plannerContext.obstacleContext.sideReconsidered
    ? PLANNER_OBSTACLE_RECONSIDERED_COUNTERSTEER_LEAD_M
    : PLANNER_OBSTACLE_COUNTERSTEER_LEAD_M;
  bool laterallyClear = plannerContext.obstacleContext.sideSign * currentLateralM >=
    plannerContext.obstacleContext.sideSign * nominalTargetLateralM -
      countersteerLeadM;
  float targetAlongM;
  if (laterallyClear ||
      currentAlongM >= plannerContext.obstacleContext.approachNearAlongM) {
    targetAlongM = plannerContext.obstacleContext.farAlongM + rearClearM;
  } else {
    float frontClearM = ROBOT_FOOTPRINT_GEOMETRY.frontExtentMm / 1000.0f +
                        PLANNER_TOTAL_HARD_CLEARANCE_M +
                        LOCAL_MAP_CELL_M * 0.5f;
    float approachAlongM =
      max(0.05f, plannerContext.obstacleContext.nearAlongM - frontClearM -
                 PLANNER_OBSTACLE_TURN_ROOM_M);
    if (plannerContext.obstacleContext.sideReconsidered) {
      float sideEscapeTargetAlongM = plannerContext.obstacleContext.sideEscapeAlongM;
      if (sideEscapeTargetAlongM >
          plannerContext.obstacleContext.nearAlongM + PLANNER_PREFERRED_CLEARANCE_M) {
        sideEscapeTargetAlongM =
          plannerContext.obstacleContext.nearAlongM - PLANNER_PREFERRED_CLEARANCE_M;
      }
      targetAlongM = min(sideEscapeTargetAlongM, approachAlongM);
    } else {
      targetAlongM = approachAlongM;
    }
  }
  PlannerCollisionSnapshot collision;
  plannerMapCaptureCollisionSnapshot(collision);
  float targetLateralM = nominalTargetLateralM;
  chooseInsideBandTargetLateral(
    collision,
    plannerContext.obstacleContext.routeUx,
    plannerContext.obstacleContext.routeUy,
    plannerContext.obstacleContext.sideSign,
    targetAlongM,
    targetLateralM,
    targetLateralM);
  obstacleTargetWorldPosition(
    plannerContext.obstacleContext.routeUx,
    plannerContext.obstacleContext.routeUy,
    targetAlongM,
    targetLateralM,
    localGoalX,
    localGoalY);
}
