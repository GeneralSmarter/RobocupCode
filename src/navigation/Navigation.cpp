#include "../../Robot.h"

#include "../../Navigation.h"
#include "../../NavigationAdmin.h"
#include "../../NavigationTest.h"
#include "ForwardTrajectoryPlanner.h"
#include "NavigationControllerInternal.h"
#include "NavigationInternal.h"
#include "ObstacleContext.h"
#include "PlannerContext.h"
#include "PlannerDebug.h"
#include "RecoveryPlanner.h"

NavigationGoal navigationGoal = {
  NAV_GOAL_NONE,
  NAV_OWNER_ROUTE,
  MOTION_AUTHORITY_NONE,
  false,
  false,
  false,
  0.0f,
  0.0f,
  0.0f,
  0.0f,
  0.0f,
  0.0f,
  0
};

PickupTrackingRuntime pickupTrackingRuntime = {
  {PICKUP_TRACKING_IDLE, PICKUP_OUTCOME_NONE, 0, 0.0f, 0.0f, 0.0f,
   false, "idle"},
  {}, {}, false, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
  0.0f, 0.0f, 0.0f, 0
};

static const char* ownerEventName(NavigationGoalOwner owner, bool success) {
  switch (owner) {
    case NAV_OWNER_TEST_DRIVE: return success ? "test_drive_end" : "test_drive_abort";
    case NAV_OWNER_TEST_GOTO: return success ? "test_goto_end" : "test_goto_abort";
    case NAV_OWNER_TEST_AVOID: return success ? "test_avoid_end" : "test_avoid_abort";
    case NAV_OWNER_TEST_ESCAPE: return success ? "test_escape_end" : "test_escape_abort";
    case NAV_OWNER_TEST_TURN: return success ? "test_turn_end" : "test_turn_abort";
    case NAV_OWNER_TEST_HUNT: return success ? "test_hunt_end" : "test_hunt_abort";
    case NAV_OWNER_WEIGHT_SCAN: return success ? "weight_scan_end" : "weight_scan_abort";
    case NAV_OWNER_PICKUP_TRACK: return success ? "pickup_track_end" : "pickup_track_abort";
    default: return success ? "navigation_goal_complete" : "navigation_goal_stop";
  }
}

void finishNavigationGoal(bool success, PlannerStopReason reason,
                          const char* detail) {
  // This is the one exit path for both route goals and test goals. It removes
  // any pending motion command before publishing the completion/abort event.
  NavigationGoalOwner owner = navigationGoal.owner;
  const bool pickupWasActive = navigationGoal.mode == NAV_GOAL_PICKUP_TRACK;
  if (plannerContext.plannerEpoch.active) {
    closePlannerEpoch();
  }
  resetReversePlannerEpoch();
  plannerContext.reverseRecoveryActive = false;
  plannerContext.reverseRecoveryState = {};
  resetRecoveryBudget();
  plannerContext.emergencyRecoveryState = {};
  plannerTelemetry.reverseRecoveryActive = false;
  resetGeometricNoPathEvidence();
  plannerContext.frontInvalidSinceMs = 0;
  plannerContext.reverseRecoveryRejectsReported = false;
  resetObstacleContext(success ? "goal_complete" : "goal_abort");
  motorStopRequested = true;
  requestMotionStop();
  navigationGoal.active = false;
  navigationGoal.authority = MOTION_AUTHORITY_NONE;
  navigationGoal.completed = success;
  navigationGoal.failed = !success;
  plannerTelemetry.stopReason = reason;
  plannerTelemetry.safeStopReason = detail;
  plannerTelemetry.selectedForwardTicksPerSec = 0.0f;
  plannerTelemetry.selectedTurnTicksPerSec = 0.0f;
  plannerTelemetry.selectedCurvature = 0.0f;
  plannerTelemetry.plannerEpochActive = false;
  plannerTelemetry.plannerCommandAgeMs = 0;
  sendBluetoothEvent(ownerEventName(owner, success), detail);
  setMatrixPickupTrackId(0);
  if (pickupWasActive && !success &&
      pickupTrackingRuntime.status.outcome == PICKUP_OUTCOME_RUNNING) {
    pickupTrackingRuntime.status.phase = PICKUP_TRACKING_FAILED;
    pickupTrackingRuntime.status.outcome = PICKUP_OUTCOME_NAVIGATION_FAILED;
    pickupTrackingRuntime.status.detail = detail;
  }

}

bool startNavigationPoint(float targetX, float targetY,
                          NavigationGoalOwner owner) {
  // Creates a world-frame point goal in metres.
  //
  // Called by:
  //   Navigation.h mission wrappers, NavigationTest.h adapters, and the
  //   weight-search state machine.
  //
  // Global effects:
  //   Resets planner epochs, recovery state, PID/encoder snapshots, goal
  //   telemetry, and schedules an immediate planner update.
  if (motionAuthority != MOTION_AUTHORITY_MISSION &&
      motionAuthority != MOTION_AUTHORITY_TEST) {
    stopMotors();
    plannerTelemetry.stopReason = PLANNER_STOP_ABORTED;
    plannerTelemetry.safeStopReason = "no_motion_authority";
    return false;
  }
  if (navigationGoal.active) {
    return false;
  }
  // A point goal does not mean "drive this exact line". It means repeatedly
  // choose a short safe arc that makes progress toward this world coordinate.
  navigationGoal.mode = NAV_GOAL_POINT;
  resetPlannerEpoch();
  resetReversePlannerEpoch();
  plannerContext.lastPlannerCommandPublishedMs = 0;
  navigationGoal.owner = owner;
  navigationGoal.authority = motionAuthority;
  navigationGoal.active = true;
  navigationGoal.completed = false;
  navigationGoal.failed = false;
  navigationGoal.targetX = targetX;
  navigationGoal.targetY = targetY;
  navigationGoal.targetYawDeg = 0.0;
  navigationGoal.startX = robotX;
  navigationGoal.startY = robotY;
  navigationGoal.startYawDeg = navigationHeadingDeg();
  navigationGoal.startedMs = millis();
  // Keep cumulative encoder totals intact, but reset the control snapshots so
  // a previous motion segment cannot create a derivative/PID kick here.
  resetEncodersAndPID();
  // Force a plan on the next controller pass rather than waiting for the old
  // goal's 40 ms schedule phase.
  lastPlannerUpdateMs = 0;
  motorStopRequested = true;
  requestMotionStop();
  // Telemetry is reset with the goal. Without this, a previous turn command
  // can make the first point-goal CSV row look like it is steering.
  plannerTelemetry.selectedForwardTicksPerSec = 0.0;
  plannerTelemetry.selectedTurnTicksPerSec = 0.0;
  plannerTelemetry.selectedCurvature = 0.0;
  plannerTelemetry.minimumSweptClearanceMm = -1.0;
  plannerTelemetry.speedCapTicksPerSec = 0.0;
  plannerTelemetry.globalGoalDistanceM =
    sqrtf((targetX - robotX) * (targetX - robotX) +
          (targetY - robotY) * (targetY - robotY));
  plannerTelemetry.localGoalDistanceM = 0.0;
  plannerTelemetry.routeAlongProgressM = 0.0;
  plannerTelemetry.routeSignedLateralErrorM = 0.0;
  plannerTelemetry.obstacleProgressAgeS = 0.0;
  plannerTelemetry.cumulativeReverseDistanceM = 0.0;
  plannerTelemetry.obstacleBestProgressM = 0.0;
  plannerTelemetry.recoveryCurrentClearanceM = 0.0f;
  plannerTelemetry.recoveryEndpointClearanceM = 0.0f;
  plannerTelemetry.recoveryClearanceGainM = 0.0f;
  plannerTelemetry.recoveryUnexploredScore = 0.0f;
  plannerTelemetry.recoveryCount = 0;
  plannerTelemetry.recoveryPlateauCount = 0;
  plannerTelemetry.reverseRecoveryActive = false;
  plannerTelemetry.candidateCount = 0;
  plannerTelemetry.stopReason = PLANNER_STOP_NONE;
  plannerTelemetry.planReason = "goal_started";
  plannerContext.lastReportedStopReason = PLANNER_STOP_NONE;
  plannerContext.reverseRecoveryActive = false;
  plannerContext.reverseRecoveryState = {};
  resetRecoveryBudget();
  plannerContext.emergencyRecoveryState = {};
  plannerContext.reverseRecoveryStepCount = 0;
  resetGeometricNoPathEvidence();
  plannerContext.candidateRejectsReported = false;
  plannerContext.reverseRecoveryRejectsReported = false;
  resetObstacleContext("point_goal_start");
  motorStopRequested = false;
  plannerTelemetry.replanReason = "goal_started";
  plannerTelemetry.safeStopReason = "";
  resetTurnStuckCheck(navigationHeadingDeg());
  plannerContext.pointAlignTurnActive = false;
  plannerContext.pointAlignTurnDirection = 0.0;
  plannerContext.frontInvalidSinceMs = 0;
  plannerContext.turnSideInvalidSinceMs = 0;
  plannerContext.turnSweepInvalidSinceMs = 0;
  sendBluetoothEvent("navigation_goal_start", "point");
  return true;
}

bool startNavigationTurn(float relativeTurnDeg, NavigationGoalOwner owner) {
  // Creates a relative in-place yaw goal in degrees.
  //
  // Inputs/outputs:
  //   relativeTurnDeg is positive CCW/left. The stored targetYawDeg is an
  //   absolute navigation heading wrapped to [-180, +180].
  //
  // Safety:
  //   updateTurnGoal() validates turn-side and full sweep sensing before each
  //   command and MotorControl.cpp checks again at output time.
  if (motionAuthority != MOTION_AUTHORITY_MISSION &&
      motionAuthority != MOTION_AUTHORITY_TEST) {
    stopMotors();
    plannerTelemetry.stopReason = PLANNER_STOP_ABORTED;
    plannerTelemetry.safeStopReason = "no_motion_authority";
    return false;
  }
  if (navigationGoal.active) {
    return false;
  }
  // Turns are their own direct yaw-feedback task. They do not use the map arc
  // sampler because they are intentionally in-place and run at 20 ms.
  float startYawDeg = navigationHeadingDeg();
  resetPlannerEpoch();
  resetReversePlannerEpoch();
  plannerContext.lastPlannerCommandPublishedMs = 0;
  navigationGoal.mode = NAV_GOAL_TURN;
  navigationGoal.owner = owner;
  navigationGoal.authority = motionAuthority;
  navigationGoal.active = true;
  navigationGoal.completed = false;
  navigationGoal.failed = false;
  navigationGoal.targetX = robotX;
  navigationGoal.targetY = robotY;
  navigationGoal.targetYawDeg = wrapAngle(startYawDeg + relativeTurnDeg);
  navigationGoal.startX = robotX;
  navigationGoal.startY = robotY;
  navigationGoal.startYawDeg = startYawDeg;
  navigationGoal.startedMs = millis();
  resetEncodersAndPID();
  lastPlannerUpdateMs = 0;
  resetTurnStuckCheck(startYawDeg);
  plannerContext.turnBrakeActive = false;
  plannerContext.turnBrakeUntilMs = 0;
  plannerContext.turnLastCommandDirection = relativeTurnDeg >= 0.0 ? 1.0 : -1.0;
  plannerContext.turnSideInvalidSinceMs = 0;
  plannerContext.turnSweepInvalidSinceMs = 0;
  plannerContext.reverseRecoveryActive = false;
  plannerContext.reverseRecoveryState = {};
  resetGeometricNoPathEvidence();
  plannerContext.reverseRecoveryRejectsReported = false;
  motorStopRequested = true;
  requestMotionStop();
  resetObstacleContext("turn_goal_start");
  motorStopRequested = false;
  plannerTelemetry.replanReason = "turn_started";
  plannerTelemetry.safeStopReason = "";
  sendBluetoothEvent("navigation_goal_start", "turn");
  return true;
}

void cancelNavigationGoal(PlannerStopReason reason, const char* detail) {
  // Public cancellation path. If a goal is active, finishNavigationGoal()
  // performs the full neutral/telemetry cleanup. If no goal is active, keep
  // telemetry honest and force neutral anyway.
  if (!navigationGoal.active) {
    navigationGoal.authority = MOTION_AUTHORITY_NONE;
    plannerTelemetry.stopReason = reason;
    plannerTelemetry.safeStopReason = detail;
    motorStopRequested = true;
    requestMotionStop();
    return;
  }
  finishNavigationGoal(false, reason, detail);
}

NavigationInternalStatus getNavigationInternalStatus() {
  NavigationInternalStatus status = {
    navigationGoal.active,
    navigationGoal.completed,
    navigationGoal.failed,
    navigationGoal.mode,
    navigationGoal.owner,
    navigationGoal.authority,
    plannerTelemetry.stopReason,
    plannerTelemetry.safeStopReason
  };
  return status;
}


void clearNavigationGoalResult() {
  navigationGoal.completed = false;
  navigationGoal.failed = false;
}

bool navigationGoTo(float worldX, float worldY) {
  return startNavigationPoint(worldX, worldY, NAV_OWNER_ROUTE);
}

bool navigationStartPickupTracking(const MatrixTargetObservation &target,
                                   const RouteResumeContext &resume) {
  if (!target.valid || !target.confirmedStatic || target.trackId == 0 ||
      navigationGoal.active ||
      (motionAuthority != MOTION_AUTHORITY_MISSION &&
       motionAuthority != MOTION_AUTHORITY_TEST)) {
    return false;
  }
  resetPlannerEpoch();
  resetReversePlannerEpoch();
  resetObstacleContext("pickup_tracking_start");
  resetRecoveryBudget();
  resetGeometricNoPathEvidence();
  navigationGoal = {
    NAV_GOAL_PICKUP_TRACK,
    motionAuthority == MOTION_AUTHORITY_TEST
      ? NAV_OWNER_TEST_HUNT : NAV_OWNER_PICKUP_TRACK,
    motionAuthority,
    true, false, false,
    resume.valid ? resume.segmentEndX : target.worldX,
    resume.valid ? resume.segmentEndY : target.worldY,
    0.0f,
    robotX, robotY, navigationHeadingDeg(), millis()
  };
  pickupTrackingRuntime = {
    {PICKUP_TRACKING_FULL_SPEED, PICKUP_OUTCOME_RUNNING, target.trackId,
     target.directGapMm, target.columnError,
     PICKUP_MIN_FORWARD_FEED_DISTANCE_MM, false, "tracking"},
    target, resume, false, target.directGapMm,
    target.worldX, target.worldY,
    0.0f, 0.0f, 0.0f, robotX, robotY,
    navigationHeadingDeg(), millis()
  };
  setMatrixPickupTrackId(target.trackId);
  resetEncodersAndPID();
  lastPlannerUpdateMs = 0;
  plannerTelemetry.stopReason = PLANNER_STOP_NONE;
  plannerTelemetry.planReason = "pickup_tracking_start";
  plannerTelemetry.replanReason = "pickup_tracking_start";
  plannerTelemetry.safeStopReason = "";
  motorStopRequested = true;
  requestMotionStop();
  motorStopRequested = false;
  sendBluetoothEvent("pickup_tracking_start", "latched_static_track");
  return true;
}

bool navigationUpdatePickupTracking(const MatrixTargetObservation &target) {
  if (!navigationGoal.active ||
      navigationGoal.mode != NAV_GOAL_PICKUP_TRACK ||
      target.trackId != pickupTrackingRuntime.status.trackId ||
      target.frameSequence <=
        pickupTrackingRuntime.observation.frameSequence) {
    return false;
  }
  pickupTrackingRuntime.observation = target;
  pickupTrackingRuntime.status.columnError = target.columnError;
  pickupTrackingRuntime.status.bestGapMm = target.directGapMm;
  return true;
}

PickupTrackingStatus navigationGetPickupTrackingStatus() {
  return pickupTrackingRuntime.status;
}

bool navigationRetargetDiagnosticPoint(float worldX, float worldY) {
  if (!navigationGoal.active || navigationGoal.mode != NAV_GOAL_POINT ||
      navigationGoal.authority != MOTION_AUTHORITY_TEST ||
      navigationGoal.owner != NAV_OWNER_TEST_GOTO) return false;
  navigationGoal.targetX = worldX;
  navigationGoal.targetY = worldY;
  resetPlannerEpoch();
  lastPlannerUpdateMs = 0;
  return true;
}

bool navigationTurnBy(float relativeDegrees) {
  return startNavigationTurn(relativeDegrees, NAV_OWNER_ROUTE);
}

bool navigationScanTurnBy(float relativeDegrees) {
  return startNavigationTurn(relativeDegrees, NAV_OWNER_WEIGHT_SCAN);
}

void navigationCancel() {
  cancelNavigationGoal(PLANNER_STOP_ABORTED, "navigation_cancelled");
}

NavigationStatus navigationGetStatus() {
  NavigationRunState state = NAVIGATION_IDLE;
  if (navigationGoal.active) {
    state = NAVIGATION_RUNNING;
  } else if (navigationGoal.completed) {
    state = NAVIGATION_REACHED;
  } else if (navigationGoal.failed) {
    state = NAVIGATION_FAILED;
  }
  NavigationStatus status = {
    state,
    plannerTelemetry.stopReason,
    plannerTelemetry.safeStopReason
  };
  return status;
}

void navigationClearResult() {
  clearNavigationGoalResult();
}

void navigationCancelWithReason(PlannerStopReason reason, const char* detail) {
  cancelNavigationGoal(reason, detail);
}

bool navigationSetEmergencyScanEnabled(bool enabled) {
  return setEmergencyScanPolicyEnabled(enabled);
}

bool navigationIsEmergencyScanEnabled() {
  return isEmergencyScanPolicyEnabled();
}

void navigationResetMap() {
  clearLocalMap();
}

bool navigationStartTestPoint(float worldX, float worldY,
                              NavigationTestPointKind kind) {
  NavigationGoalOwner owner = NAV_OWNER_TEST_GOTO;
  switch (kind) {
    case NAVIGATION_TEST_DRIVE: owner = NAV_OWNER_TEST_DRIVE; break;
    case NAVIGATION_TEST_GOTO: owner = NAV_OWNER_TEST_GOTO; break;
    case NAVIGATION_TEST_AVOID: owner = NAV_OWNER_TEST_AVOID; break;
    case NAVIGATION_TEST_ESCAPE: owner = NAV_OWNER_TEST_ESCAPE; break;
    case NAVIGATION_TEST_PICKUP: owner = NAV_OWNER_TEST_HUNT; break;
  }
  return startNavigationPoint(worldX, worldY, owner);
}

bool navigationStartTestTurn(float relativeDegrees) {
  return startNavigationTurn(relativeDegrees, NAV_OWNER_TEST_TURN);
}

bool navigationStartPointForSimulator(float worldX, float worldY,
                                      int ownerCode) {
  NavigationGoalOwner owner =
    ownerCode >= static_cast<int>(NAV_OWNER_ROUTE) &&
    ownerCode <= static_cast<int>(NAV_OWNER_PICKUP_TRACK)
      ? static_cast<NavigationGoalOwner>(ownerCode)
      : NAV_OWNER_TEST_AVOID;
  return startNavigationPoint(worldX, worldY, owner);
}

bool navigationSimulatorOwnerUsesMissionAuthority(int ownerCode) {
  return ownerCode == static_cast<int>(NAV_OWNER_ROUTE) ||
         ownerCode == static_cast<int>(NAV_OWNER_RETURN_HOME);
}

void navigationResetForSimulator() {
  navigationGoal = {};
  pickupTrackingRuntime = {};
  initializeNavigationController();
}

void navigationUpdateForSimulator() {
  updateNavigationController();
}
