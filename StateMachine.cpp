#include "Robot.h"

// =====================================================
// Mission state machine
// =====================================================
// Motion is never executed from this state machine.  It only assigns goals to
// the incremental navigation controller and observes their results.

static unsigned long waypointActionUntilMs = 0;

enum WeightSearchPhase {
  WEIGHT_SEARCH_IDLE,
  WEIGHT_SEARCH_ALIGN_CENTER,
  WEIGHT_SEARCH_SETTLE_CENTER,
  WEIGHT_SEARCH_CHECK_CENTER,
  WEIGHT_SEARCH_TURN_LEFT,
  WEIGHT_SEARCH_SETTLE_LEFT,
  WEIGHT_SEARCH_CHECK_LEFT,
  WEIGHT_SEARCH_TURN_RIGHT,
  WEIGHT_SEARCH_SETTLE_RIGHT,
  WEIGHT_SEARCH_CHECK_RIGHT,
  WEIGHT_SEARCH_RETURN_CENTER,
  WEIGHT_SEARCH_CONFIRM_TURN,
  WEIGHT_SEARCH_SETTLE_CONFIRM,
  WEIGHT_SEARCH_CHECK_CONFIRM,
  WEIGHT_SEARCH_HUNTING
};

enum WeightSearchMode {
  WEIGHT_SEARCH_MODE_NONE,
  WEIGHT_SEARCH_MODE_TEST,
  WEIGHT_SEARCH_MODE_SEARCH_WAYPOINT,
  WEIGHT_SEARCH_MODE_ROUTE_INTERRUPT_RESUME
};

static WeightSearchPhase weightSearchPhase = WEIGHT_SEARCH_IDLE;
static WeightSearchMode weightSearchMode = WEIGHT_SEARCH_MODE_NONE;
static unsigned long weightSearchPhaseStartedMs = 0;
static unsigned long weightSearchHuntStartedMs = 0;
static unsigned long weightInterruptLastMs = 0;
static bool weightSearchTurnStarted = false;
static float weightSearchAnchorX = 0.0;
static float weightSearchAnchorY = 0.0;
static WeightSearchPhase weightSearchConfirmResumePhase = WEIGHT_SEARCH_IDLE;
static const char* weightSearchConfirmDetail = "confirm";

static bool waypointActionIs(const char* action, const char* expected) {
  return action != NULL && strcmp(action, expected) == 0;
}

bool isWeightSearchActive() {
  return weightSearchPhase != WEIGHT_SEARCH_IDLE;
}

static void setWeightSearchPhase(WeightSearchPhase phase) {
  weightSearchPhase = phase;
  weightSearchPhaseStartedMs = millis();
  weightSearchTurnStarted = false;
}

static void clearWeightSearchState() {
  weightSearchPhase = WEIGHT_SEARCH_IDLE;
  weightSearchMode = WEIGHT_SEARCH_MODE_NONE;
}

static void resumeInterruptedRoute(const char* eventName, const char* detail) {
  sendBluetoothEvent(eventName, detail);
  clearWeightSearchState();
  weightInterruptLastMs = millis();
  waypointActionUntilMs = millis() + WAYPOINT_ACTION_PAUSE_MS;
  motorStopRequested = true;
  setMotionCommand(0.0, 0.0);
  clearNavigationGoalResult();
  sendBluetoothEvent("weight_interrupt_resume_route", detail);
}

static void completeWeightSearch(const char* eventName, const char* detail) {
  sendBluetoothEvent(eventName, detail);
  WeightSearchMode completedMode = weightSearchMode;
  clearWeightSearchState();

  if (completedMode == WEIGHT_SEARCH_MODE_TEST) {
    robotRunEnabled = false;
    motorStopRequested = true;
    setMotionCommand(0.0, 0.0);
    clearNavigationGoalResult();
    setRobotState(END_MATCH);
    return;
  }

  if (completedMode == WEIGHT_SEARCH_MODE_ROUTE_INTERRUPT_RESUME) {
    weightInterruptLastMs = millis();
    waypointActionUntilMs = millis() + WAYPOINT_ACTION_PAUSE_MS;
    motorStopRequested = true;
    setMotionCommand(0.0, 0.0);
    clearNavigationGoalResult();
    sendBluetoothEvent("weight_interrupt_resume_route", detail);
    return;
  }

  currentWaypointIndex++;
  waypointActionUntilMs = millis() + WAYPOINT_ACTION_PAUSE_MS;
  clearNavigationGoalResult();
}

static void failWeightSearch(const char* detail) {
  sendBluetoothEvent("weight_search_hunt_failed", detail);
  clearWeightSearchState();
  motorStopRequested = true;
  setMotionCommand(0.0, 0.0);
  clearNavigationGoalResult();
  setRobotState(END_MATCH);
}

static bool searchTargetVisible() {
  refreshObjectTargetEstimate();
  return objectCandidate.kind == OBJECT_CANDIDATE_WEIGHT_SIZED &&
         objectCandidate.confirmed &&
         isObjectTargetFresh();
}

static bool weightInterruptCooldownActive() {
  return weightInterruptLastMs != 0 &&
         millis() - weightInterruptLastMs < WEIGHT_INTERRUPT_COOLDOWN_MS;
}

static void beginSearchTargetConfirm(const char* detail,
                                     WeightSearchPhase resumePhase);

static void beginWeightSearch(WeightSearchMode mode, bool alignToWaypoint,
                              const char* detail) {
  sendBluetoothEvent("weight_search_start", detail);
  weightSearchMode = mode;
  if (mode == WEIGHT_SEARCH_MODE_SEARCH_WAYPOINT &&
      currentWaypointIndex < NUM_POINTS) {
    weightSearchAnchorX = path[currentWaypointIndex].x;
    weightSearchAnchorY = path[currentWaypointIndex].y;
  } else {
    weightSearchAnchorX = robotX;
    weightSearchAnchorY = robotY;
  }
  motorStopRequested = true;
  setMotionCommand(0.0, 0.0);
  setWeightSearchPhase(alignToWaypoint ? WEIGHT_SEARCH_ALIGN_CENTER
                                        : WEIGHT_SEARCH_SETTLE_CENTER);
}

static void beginRouteWeightInterrupt(WeightSearchMode mode, const char* detail) {
  if (isNavigationGoalActive()) {
    cancelNavigationGoal(PLANNER_STOP_ABORTED, detail);
  }
  clearNavigationGoalResult();
  weightSearchMode = mode;
  if (mode == WEIGHT_SEARCH_MODE_SEARCH_WAYPOINT &&
      currentWaypointIndex < NUM_POINTS) {
    weightSearchAnchorX = path[currentWaypointIndex].x;
    weightSearchAnchorY = path[currentWaypointIndex].y;
  } else {
    weightSearchAnchorX = robotX;
    weightSearchAnchorY = robotY;
  }
  sendBluetoothEvent("weight_interrupt_start", detail);
  beginSearchTargetConfirm(detail, WEIGHT_SEARCH_IDLE);
}

static void lockSearchTarget(const char* detail) {
  sendBluetoothEvent("weight_search_target_locked", detail);
  startNavigationPoint(objectTargetEstimate.worldX, objectTargetEstimate.worldY,
                       NAV_OWNER_OBJECT_HUNT);
  weightSearchHuntStartedMs = millis();
  setWeightSearchPhase(WEIGHT_SEARCH_HUNTING);
}

static void beginSearchTargetConfirm(const char* detail,
                                     WeightSearchPhase resumePhase) {
  weightSearchConfirmResumePhase = resumePhase;
  weightSearchConfirmDetail = detail;

  float rawTargetXmm = objectTargetEstimate.robotXmm - OBJECT_PICKUP_OVERSHOOT_MM;
  if (rawTargetXmm < 50.0f) {
    rawTargetXmm = objectTargetEstimate.robotXmm;
  }
  float targetBearingDeg = atan2f(objectTargetEstimate.robotYmm, rawTargetXmm) *
                           RAD_TO_DEG;

  if (fabs(targetBearingDeg) <= WEIGHT_SEARCH_CONFIRM_TURN_MIN_DEG) {
    lockSearchTarget(detail);
    return;
  }

  float confirmTurnDeg = constrain(targetBearingDeg,
                                   -WEIGHT_SEARCH_CONFIRM_TURN_MAX_DEG,
                                   WEIGHT_SEARCH_CONFIRM_TURN_MAX_DEG);
  sendBluetoothEvent("weight_search_confirm_turn", detail);
  startNavigationTurn(confirmTurnDeg, NAV_OWNER_WEIGHT_SCAN);
  weightSearchTurnStarted = true;
  setWeightSearchPhase(WEIGHT_SEARCH_CONFIRM_TURN);
}

static bool checkSearchTargetWindow(const char* detail,
                                    WeightSearchPhase resumePhase) {
  sendBluetoothEvent("weight_search_check", detail);
  if (searchTargetVisible()) {
    beginSearchTargetConfirm(detail, resumePhase);
    return false;
  }
  return millis() - weightSearchPhaseStartedMs >= WEIGHT_SEARCH_CONFIRM_MS;
}

void startWeightSearchTest() {
  clearNavigationGoalResult();
  beginWeightSearch(WEIGHT_SEARCH_MODE_TEST, false, "test_search");
}

void cancelWeightSearch(const char* detail) {
  if (isNavigationGoalActive() &&
      (navigationGoal.owner == NAV_OWNER_WEIGHT_SCAN ||
       navigationGoal.owner == NAV_OWNER_OBJECT_HUNT)) {
    cancelNavigationGoal(PLANNER_STOP_ABORTED, detail);
  }
  clearWeightSearchState();
  motorStopRequested = true;
  setMotionCommand(0.0, 0.0);
  clearNavigationGoalResult();
  sendBluetoothEvent("weight_search_hunt_failed", detail);
}

static bool updateWeightSearchTurn(float relativeTurnDeg, WeightSearchPhase nextPhase) {
  if (!weightSearchTurnStarted) {
    startNavigationTurn(relativeTurnDeg, NAV_OWNER_WEIGHT_SCAN);
    weightSearchTurnStarted = true;
    return false;
  }

  if (isNavigationGoalActive()) {
    return false;
  }

  if (didNavigationGoalComplete()) {
    clearNavigationGoalResult();
    setWeightSearchPhase(nextPhase);
    return true;
  }

  if (didNavigationGoalFail()) {
    completeWeightSearch("weight_search_scan_skipped", "scan_turn_failed");
  }
  return false;
}

static void updateWeightSearchAlignment() {
  if (currentWaypointIndex >= NUM_POINTS) {
    completeWeightSearch("weight_search_align_failed", "no_search_waypoint");
    return;
  }

  float dx = path[currentWaypointIndex].x - robotX;
  float dy = path[currentWaypointIndex].y - robotY;
  if (sqrtf(dx * dx + dy * dy) <= 0.001f) {
    setWeightSearchPhase(WEIGHT_SEARCH_SETTLE_CENTER);
    return;
  }

  float targetHeadingDeg = atan2f(dy, dx) * RAD_TO_DEG;
  float relativeTurnDeg = wrapAngle(targetHeadingDeg - navigationHeadingDeg());
  if (fabs(relativeTurnDeg) <= TURN_TOLERANCE_DEG) {
    setWeightSearchPhase(WEIGHT_SEARCH_SETTLE_CENTER);
    return;
  }

  if (!weightSearchTurnStarted) {
    sendBluetoothEvent("weight_search_align_start", "standoff");
    startNavigationTurn(relativeTurnDeg, NAV_OWNER_WEIGHT_SCAN);
    weightSearchTurnStarted = true;
    return;
  }

  if (isNavigationGoalActive()) {
    return;
  }

  if (didNavigationGoalComplete()) {
    clearNavigationGoalResult();
    setWeightSearchPhase(WEIGHT_SEARCH_SETTLE_CENTER);
    return;
  }

  if (didNavigationGoalFail()) {
    clearNavigationGoalResult();
    completeWeightSearch("weight_search_align_failed", "align_turn_failed");
  }
}

static void updateWeightSearch() {
  if (currentWaypointIndex >= NUM_POINTS) {
    clearWeightSearchState();
    return;
  }

  switch (weightSearchPhase) {
    case WEIGHT_SEARCH_ALIGN_CENTER:
      updateWeightSearchAlignment();
      break;

    case WEIGHT_SEARCH_SETTLE_CENTER:
      motorStopRequested = true;
      setMotionCommand(0.0, 0.0);
      if (millis() - weightSearchPhaseStartedMs >= WEIGHT_SEARCH_SETTLE_MS) {
        setWeightSearchPhase(WEIGHT_SEARCH_CHECK_CENTER);
      }
      break;

    case WEIGHT_SEARCH_CHECK_CENTER:
      if (checkSearchTargetWindow("center", WEIGHT_SEARCH_TURN_LEFT)) {
        if (weightSearchPhase != WEIGHT_SEARCH_HUNTING) {
          setWeightSearchPhase(WEIGHT_SEARCH_TURN_LEFT);
        }
      }
      break;

    case WEIGHT_SEARCH_TURN_LEFT:
      updateWeightSearchTurn(WEIGHT_SEARCH_SWEEP_DEG, WEIGHT_SEARCH_SETTLE_LEFT);
      break;

    case WEIGHT_SEARCH_SETTLE_LEFT:
      motorStopRequested = true;
      setMotionCommand(0.0, 0.0);
      if (millis() - weightSearchPhaseStartedMs >= WEIGHT_SEARCH_SETTLE_MS) {
        setWeightSearchPhase(WEIGHT_SEARCH_CHECK_LEFT);
      }
      break;

    case WEIGHT_SEARCH_CHECK_LEFT:
      if (checkSearchTargetWindow("left", WEIGHT_SEARCH_TURN_RIGHT)) {
        if (weightSearchPhase != WEIGHT_SEARCH_HUNTING) {
          setWeightSearchPhase(WEIGHT_SEARCH_TURN_RIGHT);
        }
      }
      break;

    case WEIGHT_SEARCH_TURN_RIGHT:
      updateWeightSearchTurn(-2.0 * WEIGHT_SEARCH_SWEEP_DEG, WEIGHT_SEARCH_SETTLE_RIGHT);
      break;

    case WEIGHT_SEARCH_SETTLE_RIGHT:
      motorStopRequested = true;
      setMotionCommand(0.0, 0.0);
      if (millis() - weightSearchPhaseStartedMs >= WEIGHT_SEARCH_SETTLE_MS) {
        setWeightSearchPhase(WEIGHT_SEARCH_CHECK_RIGHT);
      }
      break;

    case WEIGHT_SEARCH_CHECK_RIGHT:
      if (checkSearchTargetWindow("right", WEIGHT_SEARCH_RETURN_CENTER)) {
        if (weightSearchPhase != WEIGHT_SEARCH_HUNTING) {
          setWeightSearchPhase(WEIGHT_SEARCH_RETURN_CENTER);
        }
      }
      break;

    case WEIGHT_SEARCH_RETURN_CENTER:
      if (updateWeightSearchTurn(WEIGHT_SEARCH_SWEEP_DEG, WEIGHT_SEARCH_IDLE) &&
          weightSearchPhase == WEIGHT_SEARCH_IDLE) {
        completeWeightSearch("weight_search_no_target", "sweep_complete");
      }
      break;

    case WEIGHT_SEARCH_CONFIRM_TURN:
      if (isNavigationGoalActive()) {
        break;
      }
      if (didNavigationGoalComplete()) {
        clearNavigationGoalResult();
        setWeightSearchPhase(WEIGHT_SEARCH_SETTLE_CONFIRM);
        break;
      }
      if (didNavigationGoalFail()) {
        clearNavigationGoalResult();
        if (weightSearchConfirmResumePhase == WEIGHT_SEARCH_IDLE) {
          if (weightSearchMode == WEIGHT_SEARCH_MODE_ROUTE_INTERRUPT_RESUME) {
            resumeInterruptedRoute("weight_interrupt_confirm_lost", "confirm_turn_failed");
          } else {
            resumeInterruptedRoute("weight_search_no_target", "confirm_turn_failed");
          }
          break;
        }
        sendBluetoothEvent("weight_search_scan_skipped", "confirm_turn_failed");
        setWeightSearchPhase(weightSearchConfirmResumePhase);
      }
      break;

    case WEIGHT_SEARCH_SETTLE_CONFIRM:
      motorStopRequested = true;
      setMotionCommand(0.0, 0.0);
      if (millis() - weightSearchPhaseStartedMs >= WEIGHT_SEARCH_SETTLE_MS) {
        setWeightSearchPhase(WEIGHT_SEARCH_CHECK_CONFIRM);
      }
      break;

    case WEIGHT_SEARCH_CHECK_CONFIRM:
      sendBluetoothEvent("weight_search_check", "confirm");
      if (searchTargetVisible()) {
        lockSearchTarget(weightSearchConfirmDetail);
        break;
      }
      if (millis() - weightSearchPhaseStartedMs >= WEIGHT_SEARCH_CONFIRM_MS) {
        if (weightSearchConfirmResumePhase == WEIGHT_SEARCH_IDLE) {
          if (weightSearchMode == WEIGHT_SEARCH_MODE_ROUTE_INTERRUPT_RESUME) {
            resumeInterruptedRoute("weight_interrupt_confirm_lost", "confirm_lost");
          } else {
            resumeInterruptedRoute("weight_search_no_target", "confirm_lost");
          }
          break;
        }
        sendBluetoothEvent("weight_search_no_target", "confirm_lost");
        setWeightSearchPhase(weightSearchConfirmResumePhase);
      }
      break;

    case WEIGHT_SEARCH_HUNTING: {
      float dx = robotX - weightSearchAnchorX;
      float dy = robotY - weightSearchAnchorY;
      float deviationM = sqrtf(dx * dx + dy * dy);
      if (deviationM > WEIGHT_SEARCH_MAX_ROUTE_DEVIATION_M) {
        cancelNavigationGoal(PLANNER_STOP_ABORTED, "weight_search_max_deviation");
        failWeightSearch("max_route_deviation");
        break;
      }
      if (millis() - weightSearchHuntStartedMs > WEIGHT_SEARCH_HUNT_TIMEOUT_MS) {
        cancelNavigationGoal(PLANNER_STOP_ABORTED, "weight_search_hunt_timeout");
        failWeightSearch("hunt_timeout");
        break;
      }
      if (isNavigationGoalActive()) {
        break;
      }
      if (didNavigationGoalComplete()) {
        completeWeightSearch(
          weightSearchMode == WEIGHT_SEARCH_MODE_ROUTE_INTERRUPT_RESUME
            ? "weight_interrupt_hunt_success"
            : "weight_search_hunt_success",
          "object_hunt_complete");
        break;
      }
      if (didNavigationGoalFail()) {
        failWeightSearch("object_hunt_failed");
      }
      break;
    }

    case WEIGHT_SEARCH_IDLE:
    default:
      break;
  }
}

static bool assignSearchWaypointStandoffOrStartSearch() {
  if (currentWaypointIndex >= NUM_POINTS) {
    return false;
  }

  float searchX = path[currentWaypointIndex].x;
  float searchY = path[currentWaypointIndex].y;
  float robotToSearchX = searchX - robotX;
  float robotToSearchY = searchY - robotY;
  float robotToSearchM = sqrtf(robotToSearchX * robotToSearchX +
                               robotToSearchY * robotToSearchY);
  if (robotToSearchM <= WEIGHT_SEARCH_STANDOFF_M) {
    sendBluetoothEvent("weight_search_standoff_start", "inside_standoff");
    beginWeightSearch(WEIGHT_SEARCH_MODE_SEARCH_WAYPOINT, true,
                      path[currentWaypointIndex].action);
    return true;
  }

  float originX = currentWaypointIndex > 0 ? path[currentWaypointIndex - 1].x : robotX;
  float originY = currentWaypointIndex > 0 ? path[currentWaypointIndex - 1].y : robotY;
  float approachX = searchX - originX;
  float approachY = searchY - originY;
  float approachM = sqrtf(approachX * approachX + approachY * approachY);
  if (approachM <= WEIGHT_SEARCH_STANDOFF_M) {
    sendBluetoothEvent("weight_search_standoff_start", "short_segment");
    beginWeightSearch(WEIGHT_SEARCH_MODE_SEARCH_WAYPOINT, true,
                      path[currentWaypointIndex].action);
    return true;
  }

  float unitX = approachX / approachM;
  float unitY = approachY / approachM;
  float standoffX = searchX - unitX * WEIGHT_SEARCH_STANDOFF_M;
  float standoffY = searchY - unitY * WEIGHT_SEARCH_STANDOFF_M;
  sendBluetoothEvent("weight_search_standoff_start", "route_standoff");
  startNavigationPoint(standoffX, standoffY, NAV_OWNER_ROUTE);
  return true;
}

static bool tryStartRouteWeightInterrupt() {
  if (weightInterruptCooldownActive() ||
      currentWaypointIndex >= NUM_POINTS ||
      !isNavigationGoalActive() ||
      navigationGoal.owner != NAV_OWNER_ROUTE ||
      !searchTargetVisible()) {
    return false;
  }

  WeightSearchMode mode = waypointActionIs(path[currentWaypointIndex].action, "SEARCH")
    ? WEIGHT_SEARCH_MODE_SEARCH_WAYPOINT
    : WEIGHT_SEARCH_MODE_ROUTE_INTERRUPT_RESUME;
  beginRouteWeightInterrupt(mode,
                            mode == WEIGHT_SEARCH_MODE_SEARCH_WAYPOINT
                              ? "search_waypoint_interrupt"
                              : "route_interrupt");
  return true;
}

void runStateMachine() {
  switch (currentState) {
    case INIT:
      runInitState();
      break;
    case FOLLOW_PATH:
      runFollowPathState();
      break;
    case RETURN_HOME:
      runReturnHomeState();
      break;
    case END_MATCH:
      runEndMatchState();
      break;
    case OBSTACLE_AVOID:
    case STUCK_RECOVERY:
      // These labels are retained for telemetry compatibility.  Local
      // planning now absorbs avoidance and recovery without nested routines.
      setRobotState(FOLLOW_PATH);
      break;
    default:
      runUnusedState(robotStateName(currentState));
      break;
  }
}

void runInitState() {
  setMotionCommand(0.0, 0.0);
  motorStopRequested = true;
  currentWaypointIndex = 0;
  waypointActionUntilMs = 0;
  clearWeightSearchState();
  weightInterruptLastMs = 0;
  endMatchPrinted = false;
  clearNavigationGoalResult();
  clearLocalMap();

  Serial.println();
  Serial.println("INIT complete. Starting FOLLOW_PATH.");
  setRobotState(FOLLOW_PATH);
}

void runFollowPathState() {
  // The mission layer advances waypoint indices only. It never makes a motor
  // decision and therefore cannot fight the local planner underneath it.
  if (returnHomeRequested) {
    setRobotState(RETURN_HOME);
    return;
  }

  if (isWeightSearchActive()) {
    updateWeightSearch();
    return;
  }

  // Test goals own the same controller without advancing the route.
  if (isNavigationGoalActive() && navigationGoal.owner != NAV_OWNER_ROUTE) {
    return;
  }

  if (tryStartRouteWeightInterrupt()) {
    return;
  }

  if (didNavigationGoalFail()) {
    // Keep the robot in a visible, re-plannable safe stop.  Do not quietly
    // advance the route or replay a fixed recovery routine.
    return;
  }

  if (didNavigationGoalComplete()) {
    if (navigationGoal.owner == NAV_OWNER_ROUTE) {
      const char* action = path[currentWaypointIndex].action;
      if (waypointActionIs(action, "SEARCH")) {
        clearNavigationGoalResult();
        beginWeightSearch(WEIGHT_SEARCH_MODE_SEARCH_WAYPOINT, true, action);
        return;
      }
      runWaypointAction(action);
      currentWaypointIndex++;
      waypointActionUntilMs = millis() + WAYPOINT_ACTION_PAUSE_MS;
    }
    clearNavigationGoalResult();
  }

  if (currentWaypointIndex >= NUM_POINTS) {
    setRobotState(END_MATCH);
    return;
  }

  if (waypointActionUntilMs != 0) {
    if (millis() < waypointActionUntilMs) {
      motorStopRequested = true;
      setMotionCommand(0.0, 0.0);
      return;
    }
    waypointActionUntilMs = 0;
  }

  if (!isNavigationGoalActive()) {
    Serial.print("Waypoint ");
    Serial.print(currentWaypointIndex + 1);
    Serial.print(" of ");
    Serial.println(NUM_POINTS);
    if (waypointActionIs(path[currentWaypointIndex].action, "SEARCH")) {
      assignSearchWaypointStandoffOrStartSearch();
    } else {
      goToPoint(path[currentWaypointIndex].x, path[currentWaypointIndex].y);
    }
  }
}

void runReturnHomeState() {
  if (!isNavigationGoalActive() && !didNavigationGoalComplete() && !didNavigationGoalFail()) {
    Serial.println("RETURN_HOME: assigning local navigation goal x=0.000 y=0.000");
    startNavigationPoint(0.0, 0.0, NAV_OWNER_RETURN_HOME);
    return;
  }

  if (didNavigationGoalComplete()) {
    Serial.println("RETURN_HOME complete.");
    clearNavigationGoalResult();
    setRobotState(END_MATCH);
  }
}

void runUnusedState(const char* stateName) {
  motorStopRequested = true;
  setMotionCommand(0.0, 0.0);
  Serial.print(stateName);
  Serial.println(" is not implemented. Entering safe stop.");
  setRobotState(END_MATCH);
}

void runEndMatchState() {
  motorStopRequested = true;
  setMotionCommand(0.0, 0.0);
  robotRunEnabled = false;

  if (!endMatchPrinted) {
    Serial.println();
    Serial.println("END_MATCH. Robot stopped.");
    printPose();
    printWaitingForStart();
    endMatchPrinted = true;
  }
}

void setRobotState(RobotState newState) {
  if (currentState != newState) {
    previousState = currentState;
    Serial.print("STATE: ");
    Serial.print(robotStateName(currentState));
    Serial.print(" -> ");
    Serial.println(robotStateName(newState));
  }
  currentState = newState;
}

const char* robotStateName(RobotState state) {
  switch (state) {
    case INIT: return "INIT";
    case FOLLOW_PATH: return "FOLLOW_PATH";
    case APPROACH_OBJECT: return "APPROACH_OBJECT";
    case COLLECT_SORT: return "COLLECT_SORT";
    case RETURN_HOME: return "RETURN_HOME";
    case UNLOAD: return "UNLOAD";
    case OBSTACLE_AVOID: return "OBSTACLE_AVOID";
    case STUCK_RECOVERY: return "STUCK_RECOVERY";
    case END_MATCH: return "END_MATCH";
  }
  return "UNKNOWN";
}

void setMotionCommand(float forwardSpeed, float turnSpeed) {
  desiredForwardSpeed = forwardSpeed;
  desiredTurnSpeed = turnSpeed;
}
