#include "../../Robot.h"
#include "../../Navigation.h"
#include "NavigationInternal.h"
#include "ObstacleContext.h"
#include "PlannerContext.h"
#include "PlannerDebug.h"
#include "RecoveryPlanner.h"

PlannerDebugSnapshot getPlannerDebugSnapshot() {
  float obstacleGoalX = navigationGoal.targetX;
  float obstacleGoalY = navigationGoal.targetY;
  if (plannerContext.obstacleContext.active) {
    buildObstacleLocalGoal(obstacleGoalX, obstacleGoalY);
  }
  PlannerDebugSnapshot snapshot = {
    static_cast<int>(plannerContext.emergencyRecoveryState.phase),
    plannerContext.emergencyRecoveryState.consumed,
    plannerContext.emergencyRecoveryState.scanSector,
    plannerContext.emergencyRecoveryState.scanAccumulatedDeg,
    plannerContext.emergencyRecoveryState.relocationDistanceM,
    plannerContext.emergencyRecoveryState.bestRotationalClearanceM,
    plannerContext.obstacleContext.active ? 1 : 0,
    plannerContext.obstacleContext.active ? plannerContext.obstacleContext.sideSign : 0.0f,
    plannerContext.obstacleContext.nearAlongM,
    plannerContext.obstacleContext.farAlongM,
    plannerContext.obstacleContext.minLateralM,
    plannerContext.obstacleContext.maxLateralM,
    obstacleGoalX,
    obstacleGoalY
  };
  return snapshot;
}

const PlannerTelemetry& getPlannerTelemetry() {
  return plannerTelemetry;
}

const char* plannerStopReasonName(PlannerStopReason reason) {
  switch (reason) {
    case PLANNER_STOP_NONE: return "none";
    case PLANNER_STOP_FRONT_BLOCKED: return "front_blocked";
    case PLANNER_STOP_FRONT_INVALID: return "front_invalid";
    case PLANNER_STOP_NO_SAFE_TRAJECTORY: return "no_safe_trajectory";
    case PLANNER_STOP_TURN_SIDE_INVALID: return "turn_side_invalid";
    case PLANNER_STOP_TURN_CLEARANCE: return "turn_clearance";
    case PLANNER_STOP_STUCK: return "stuck";
    case PLANNER_STOP_RECOVERY_TIME: return "recovery_time";
    case PLANNER_STOP_RECOVERY_DISTANCE: return "recovery_distance";
    case PLANNER_STOP_RECOVERY_REPEATED: return "recovery_repeated";
    case PLANNER_STOP_RECOVERY_NO_PROGRESS: return "recovery_no_progress";
    case PLANNER_STOP_RECOVERY_NO_USEFUL_OUTCOME: return "recovery_no_useful_outcome";
    case PLANNER_STOP_EMERGENCY_SCAN_ABORTED: return "emergency_scan_aborted";
    case PLANNER_STOP_EMERGENCY_RETRY_EXHAUSTED: return "emergency_retry_exhausted";
    case PLANNER_STOP_ABORTED: return "aborted";
  }
  return "unknown";
}

bool plannerDebugForceRecoveryExhaustion() {
  NavigationInternalStatus status = getNavigationInternalStatus();
  if (!status.active || status.mode != NAV_GOAL_POINT) {
    return false;
  }
  handleRecoveryExhaustion(
    PLANNER_STOP_RECOVERY_NO_PROGRESS,
    "host_forced_recovery_exhaustion");
  return getPlannerDebugSnapshot().emergencyConsumed;
}

NavigationDebugStatus getNavigationDebugStatus() {
  NavigationInternalStatus status = getNavigationInternalStatus();
  NavigationDebugStatus debugStatus = {
    status.active,
    status.completed,
    status.failed,
    status.active && status.mode == NAV_GOAL_TURN &&
      status.owner == NAV_OWNER_TEST_TURN,
    status.authority
  };
  return debugStatus;
}
