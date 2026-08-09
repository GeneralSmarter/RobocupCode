#include "../../Robot.h"
#include "../../Navigation.h"
#include "WeightSearch.h"

// =====================================================
// Weight-search mission component
// =====================================================
// Responsibility:
//   Owns the matrix-weight search mini-state-machine and its route interrupt.
// Interacts with:
//   Navigation.h receives point, turn, and pickup-tracking goals. The front matrix supplies
//   candidate/target state. RouteMission.cpp owns route dispatch and consumes
//   typed search results. Bluetooth.cpp starts tests and prints telemetry.
// Control flow:
//   RouteMission.cpp calls updateWeightSearch() while a search is active. This
//   component assigns goals and watches their results; it never writes servo
//   pulses directly.
// Global state:
//   Modifies only weight-search latches and navigation results through the
//   public API. RouteMission.cpp alone decides whether to advance its index.

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
static MatrixTargetObservation weightSearchTarget = {};
static RouteResumeContext weightRouteResume = {};
static WeightSearchStatus weightSearchStatus = {
  WEIGHT_SEARCH_RESULT_IDLE,
  WEIGHT_SEARCH_ORIGIN_NONE,
  "idle"
};

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

void initializeWeightSearch() {
  clearWeightSearchState();
  weightSearchPhaseStartedMs = 0;
  weightSearchHuntStartedMs = 0;
  weightInterruptLastMs = 0;
  weightSearchTurnStarted = false;
  weightSearchTarget = {};
  weightRouteResume = {};
  weightSearchStatus = {
    WEIGHT_SEARCH_RESULT_IDLE,
    WEIGHT_SEARCH_ORIGIN_NONE,
    "idle"
  };
}

WeightSearchStatus getWeightSearchStatus() {
  if (isWeightSearchActive()) {
    WeightSearchStatus running = weightSearchStatus;
    running.result = WEIGHT_SEARCH_RESULT_RUNNING;
    return running;
  }
  return weightSearchStatus;
}

void clearWeightSearchResult() {
  if (!isWeightSearchActive()) {
    weightSearchStatus = {
      WEIGHT_SEARCH_RESULT_IDLE,
      WEIGHT_SEARCH_ORIGIN_NONE,
      "idle"
    };
  }
}

static void resumeInterruptedRoute(const char* eventName, const char* detail) {
  // Ends a route interrupt without advancing the waypoint. A short pause gives
  // the route controller a clean stopped handoff before it resumes.
  sendBluetoothEvent(eventName, detail);
  weightSearchStatus = {
    WEIGHT_SEARCH_RESULT_COMPLETED,
    WEIGHT_SEARCH_ORIGIN_ROUTE_INTERRUPT,
    detail
  };
  clearWeightSearchState();
  weightInterruptLastMs = millis();
  motorStopRequested = true;
  requestMotionStop();
  navigationClearResult();
  sendBluetoothEvent("weight_interrupt_resume_route", detail);
}

static void completeWeightSearch(const char* eventName, const char* detail) {
  // Common success/no-target exit. TEST searches stop the robot; route
  // searches either resume the interrupted route or advance past the SEARCH
  // waypoint.
  sendBluetoothEvent(eventName, detail);
  WeightSearchMode completedMode = weightSearchMode;
  WeightSearchOrigin completedOrigin = completedMode == WEIGHT_SEARCH_MODE_TEST
    ? WEIGHT_SEARCH_ORIGIN_TEST
    : completedMode == WEIGHT_SEARCH_MODE_SEARCH_WAYPOINT
      ? WEIGHT_SEARCH_ORIGIN_WAYPOINT
      : WEIGHT_SEARCH_ORIGIN_ROUTE_INTERRUPT;
  weightSearchStatus = {
    WEIGHT_SEARCH_RESULT_COMPLETED,
    completedOrigin,
    detail
  };
  clearWeightSearchState();
  if (completedMode == WEIGHT_SEARCH_MODE_ROUTE_INTERRUPT_RESUME) {
    weightInterruptLastMs = millis();
    sendBluetoothEvent("weight_interrupt_resume_route", detail);
  }
  motorStopRequested = true;
  requestMotionStop();
  navigationClearResult();
}

static void completeWeightSearchRouteBlended(const char* detail) {
  sendBluetoothEvent("weight_search_route_blended", detail);
  const WeightSearchOrigin completedOrigin =
    weightSearchMode == WEIGHT_SEARCH_MODE_SEARCH_WAYPOINT
      ? WEIGHT_SEARCH_ORIGIN_WAYPOINT
      : WEIGHT_SEARCH_ORIGIN_ROUTE_INTERRUPT;
  weightSearchStatus = {
    WEIGHT_SEARCH_RESULT_COMPLETED,
    completedOrigin,
    detail
  };
  clearWeightSearchState();
  weightInterruptLastMs = millis();
}

static void failWeightSearch(const char* detail) {
  // Fail closed on object-hunt/search faults. The current implementation does
  // not try to continue the mission after a hunt failure.
  sendBluetoothEvent("weight_search_hunt_failed", detail);
  WeightSearchOrigin failedOrigin = weightSearchMode == WEIGHT_SEARCH_MODE_TEST
    ? WEIGHT_SEARCH_ORIGIN_TEST
    : weightSearchMode == WEIGHT_SEARCH_MODE_SEARCH_WAYPOINT
      ? WEIGHT_SEARCH_ORIGIN_WAYPOINT
      : WEIGHT_SEARCH_ORIGIN_ROUTE_INTERRUPT;
  weightSearchStatus = {
    WEIGHT_SEARCH_RESULT_FAILED,
    failedOrigin,
    detail
  };
  clearWeightSearchState();
  motorStopRequested = true;
  requestMotionStop();
  navigationClearResult();
}

static bool searchTargetVisible() {
  // Consume the timestamped snapshot maintained by updateRobotController().
  // A route-interrupt check must never pause active motion to force new reads.
  MatrixTargetObservation observation;
  if (!getMatrixTargetObservation(observation)) {
    return false;
  }
  weightSearchTarget = observation;
  return true;
}

static bool weightInterruptCooldownActive() {
  return weightInterruptLastMs != 0 &&
         millis() - weightInterruptLastMs < WEIGHT_INTERRUPT_COOLDOWN_MS;
}

static void beginSearchTargetConfirm(const char* detail,
                                     WeightSearchPhase resumePhase);

static void beginWeightSearch(WeightSearchMode mode, bool alignToWaypoint,
                              float anchorX, float anchorY,
                              const char* detail) {
  // Starts the search scan from either the current pose or a route waypoint
  // anchor. The scan itself is built from ordinary navigation turn goals, so
  // it remains safety-supervised.
  sendBluetoothEvent("weight_search_start", detail);
  weightSearchMode = mode;
  weightSearchStatus.origin = mode == WEIGHT_SEARCH_MODE_TEST
    ? WEIGHT_SEARCH_ORIGIN_TEST
    : mode == WEIGHT_SEARCH_MODE_SEARCH_WAYPOINT
      ? WEIGHT_SEARCH_ORIGIN_WAYPOINT
      : WEIGHT_SEARCH_ORIGIN_ROUTE_INTERRUPT;
  weightSearchStatus.result = WEIGHT_SEARCH_RESULT_RUNNING;
  weightSearchStatus.detail = detail;
  weightSearchAnchorX = anchorX;
  weightSearchAnchorY = anchorY;
  if (mode == WEIGHT_SEARCH_MODE_TEST) {
    weightRouteResume = {};
  }
  motorStopRequested = true;
  requestMotionStop();
  setWeightSearchPhase(alignToWaypoint ? WEIGHT_SEARCH_ALIGN_CENTER
                                        : WEIGHT_SEARCH_SETTLE_CENTER);
}

static void beginRouteWeightInterrupt(WeightSearchMode mode,
                                      int routeIndex,
                                      float segmentStartX,
                                      float segmentStartY,
                                      float anchorX, float anchorY,
                                      const char* detail) {
  // Cancels the route-owned goal before beginning an opportunistic object
  // confirmation. This prevents route and hunt goals from owning motion at
  // the same time.
  if (navigationGetStatus().state == NAVIGATION_RUNNING) {
    navigationCancel();
  }
  navigationClearResult();
  weightSearchMode = mode;
  weightSearchStatus.origin = mode == WEIGHT_SEARCH_MODE_SEARCH_WAYPOINT
    ? WEIGHT_SEARCH_ORIGIN_WAYPOINT
    : WEIGHT_SEARCH_ORIGIN_ROUTE_INTERRUPT;
  weightSearchStatus.result = WEIGHT_SEARCH_RESULT_RUNNING;
  weightSearchStatus.detail = detail;
  weightSearchAnchorX = anchorX;
  weightSearchAnchorY = anchorY;
  const float segmentDx = anchorX - segmentStartX;
  const float segmentDy = anchorY - segmentStartY;
  const float segmentLength = hypotf(segmentDx, segmentDy);
  const float along = segmentLength > 0.001f
    ? ((robotX - segmentStartX) * segmentDx +
       (robotY - segmentStartY) * segmentDy) / segmentLength
    : 0.0f;
  weightRouteResume = {
    true, routeIndex, segmentStartX, segmentStartY,
    anchorX, anchorY, constrain(along, 0.0f, segmentLength),
    robotX, robotY, weightSearchTarget.trackId
  };
  sendBluetoothEvent("weight_interrupt_start", detail);
  beginSearchTargetConfirm(detail, WEIGHT_SEARCH_IDLE);
}

static void lockSearchTarget(const char* detail) {
  // Starts the nonblocking matrix tracking goal with a latched track and the
  // exact route context that must resume after the assumed funnel handoff.
  sendBluetoothEvent("weight_search_target_locked", detail);
  if (!searchTargetVisible()) {
    failWeightSearch("locked_target_not_fresh");
    return;
  }
  weightRouteResume.trackId = weightSearchTarget.trackId;
  if (!navigationStartPickupTracking(weightSearchTarget,
                                     weightRouteResume)) {
    failWeightSearch("hunt_goal_rejected");
    return;
  }
  weightSearchHuntStartedMs = millis();
  setWeightSearchPhase(WEIGHT_SEARCH_HUNTING);
}

static void beginSearchTargetConfirm(const char* detail,
                                     WeightSearchPhase resumePhase) {
  // If the detected object is not centered, turn toward its estimated bearing
  // before locking the hunt target. The turn angle is clamped so a noisy target
  // estimate cannot demand a large spin.
  weightSearchConfirmResumePhase = resumePhase;
  weightSearchConfirmDetail = detail;

  float targetBearingDeg = atan2f(weightSearchTarget.robotYmm,
                                  weightSearchTarget.robotXmm) *
                           RAD_TO_DEG;

  if (fabs(targetBearingDeg) <= WEIGHT_SEARCH_CONFIRM_TURN_MIN_DEG) {
    lockSearchTarget(detail);
    return;
  }

  float confirmTurnDeg = constrain(targetBearingDeg,
                                   -WEIGHT_SEARCH_CONFIRM_TURN_MAX_DEG,
                                   WEIGHT_SEARCH_CONFIRM_TURN_MAX_DEG);
  sendBluetoothEvent("weight_search_confirm_turn", detail);
  if (!navigationScanTurnBy(confirmTurnDeg)) {
    completeWeightSearch("weight_search_scan_skipped",
                         "confirm_turn_rejected");
    return;
  }
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
  // Bluetooth TEST SEARCH entry point. It uses the same phase machine as route
  // searches but finishes by stopping in END_MATCH.
  navigationClearResult();
  beginWeightSearch(WEIGHT_SEARCH_MODE_TEST, false, robotX, robotY,
                    "test_search");
}

void cancelWeightSearch(const char* detail) {
  // Cancels any scan/hunt-owned navigation goal and returns the mission layer
  // to a stopped, non-searching state.
  if (navigationGetStatus().state == NAVIGATION_RUNNING) {
    navigationCancel();
  }
  weightSearchStatus.result = WEIGHT_SEARCH_RESULT_FAILED;
  weightSearchStatus.detail = detail;
  clearWeightSearchState();
  motorStopRequested = true;
  requestMotionStop();
  navigationClearResult();
  sendBluetoothEvent("weight_search_hunt_failed", detail);
}

static bool updateWeightSearchTurn(float relativeTurnDeg, WeightSearchPhase nextPhase) {
  // Starts a turn goal once, then waits across loop iterations for the planner
  // to report completion/failure. This is the nonblocking replacement for a
  // delay-based scan.
  if (!weightSearchTurnStarted) {
    if (!navigationScanTurnBy(relativeTurnDeg)) {
      completeWeightSearch("weight_search_scan_skipped",
                           "scan_turn_rejected");
      return false;
    }
    weightSearchTurnStarted = true;
    return false;
  }

  NavigationStatus navigation = navigationGetStatus();
  if (navigation.state == NAVIGATION_RUNNING) {
    return false;
  }

  if (navigation.state == NAVIGATION_REACHED) {
    navigationClearResult();
    setWeightSearchPhase(nextPhase);
    return true;
  }

  if (navigation.state == NAVIGATION_FAILED) {
    completeWeightSearch("weight_search_scan_skipped", "scan_turn_failed");
  }
  return false;
}

static void updateWeightSearchAlignment() {
  // Aligns the robot to face the SEARCH waypoint before the centre/left/right
  // scan windows. All angles are navigation degrees, positive CCW/left.
  float dx = weightSearchAnchorX - robotX;
  float dy = weightSearchAnchorY - robotY;
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
    if (!navigationScanTurnBy(relativeTurnDeg)) {
      completeWeightSearch("weight_search_align_failed",
                           "align_turn_rejected");
      return;
    }
    weightSearchTurnStarted = true;
    return;
  }

  NavigationStatus navigation = navigationGetStatus();
  if (navigation.state == NAVIGATION_RUNNING) {
    return;
  }

  if (navigation.state == NAVIGATION_REACHED) {
    navigationClearResult();
    setWeightSearchPhase(WEIGHT_SEARCH_SETTLE_CENTER);
    return;
  }

  if (navigation.state == NAVIGATION_FAILED) {
    navigationClearResult();
    completeWeightSearch("weight_search_align_failed", "align_turn_failed");
  }
}

void updateWeightSearch() {
  // Runs one tick of the weight-search phase machine. Settle phases command
  // neutral and wait for fresh object readings; turn phases delegate to
  // the navigation API; hunt phase delegates to a point goal.
  switch (weightSearchPhase) {
    case WEIGHT_SEARCH_ALIGN_CENTER:
      updateWeightSearchAlignment();
      break;

    case WEIGHT_SEARCH_SETTLE_CENTER:
      motorStopRequested = true;
      requestMotionStop();
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
      requestMotionStop();
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
      requestMotionStop();
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
      if (navigationGetStatus().state == NAVIGATION_RUNNING) {
        break;
      }
      if (navigationGetStatus().state == NAVIGATION_REACHED) {
        navigationClearResult();
        setWeightSearchPhase(WEIGHT_SEARCH_SETTLE_CONFIRM);
        break;
      }
      if (navigationGetStatus().state == NAVIGATION_FAILED) {
        navigationClearResult();
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
      requestMotionStop();
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
      MatrixTargetObservation latestTarget;
      if (getMatrixTargetObservation(latestTarget) &&
          latestTarget.trackId == weightSearchTarget.trackId) {
        weightSearchTarget = latestTarget;
        navigationUpdatePickupTracking(latestTarget);
      }
      PickupTrackingStatus pickup = navigationGetPickupTrackingStatus();
      const bool handoffOrFeed =
        pickup.phase == PICKUP_TRACKING_HANDOFF_ASSUMED ||
        pickup.phase == PICKUP_TRACKING_FEEDING_UNCONFIRMED ||
        pickup.phase == PICKUP_TRACKING_FEED_COMPLETE_UNCONFIRMED;
      if (!handoffOrFeed &&
          millis() - weightSearchHuntStartedMs > WEIGHT_SEARCH_HUNT_TIMEOUT_MS) {
        navigationCancel();
        failWeightSearch("hunt_timeout");
        break;
      }
      NavigationStatus navigation = navigationGetStatus();
      if (pickup.phase == PICKUP_TRACKING_FEED_COMPLETE_UNCONFIRMED &&
          navigation.state == NAVIGATION_RUNNING) {
        completeWeightSearchRouteBlended(pickup.detail);
        break;
      }
      if (navigation.state == NAVIGATION_RUNNING) {
        break;
      }
      if (navigation.state == NAVIGATION_REACHED) {
        completeWeightSearch(
          weightSearchMode == WEIGHT_SEARCH_MODE_ROUTE_INTERRUPT_RESUME
            ? "weight_interrupt_hunt_success"
            : "weight_search_hunt_success",
          pickup.detail);
        break;
      }
      if (navigation.state == NAVIGATION_FAILED) {
        if (weightSearchMode == WEIGHT_SEARCH_MODE_ROUTE_INTERRUPT_RESUME) {
          resumeInterruptedRoute("weight_interrupt_hunt_abandoned",
                                 pickup.detail);
        } else if (weightSearchMode == WEIGHT_SEARCH_MODE_SEARCH_WAYPOINT) {
          completeWeightSearch("weight_search_hunt_abandoned",
                               pickup.detail);
        } else {
          failWeightSearch(pickup.detail);
        }
      }
      break;
    }

    case WEIGHT_SEARCH_IDLE:
    default:
      break;
  }
}

bool startSearchWaypointApproach(float searchX, float searchY,
                                 float approachOriginX,
                                 float approachOriginY,
                                 const char* detail) {
  // SEARCH waypoints are approached from a short standoff distance. This gives
  // the object sensors space to observe the target before the hunt goal is
  // created.
  float robotToSearchX = searchX - robotX;
  float robotToSearchY = searchY - robotY;
  float robotToSearchM = sqrtf(robotToSearchX * robotToSearchX +
                               robotToSearchY * robotToSearchY);
  if (robotToSearchM <= WEIGHT_SEARCH_STANDOFF_M) {
    sendBluetoothEvent("weight_search_standoff_start", "inside_standoff");
    beginWeightSearch(WEIGHT_SEARCH_MODE_SEARCH_WAYPOINT, true,
                      searchX, searchY, detail);
    return true;
  }

  float approachX = searchX - approachOriginX;
  float approachY = searchY - approachOriginY;
  float approachM = sqrtf(approachX * approachX + approachY * approachY);
  if (approachM <= WEIGHT_SEARCH_STANDOFF_M) {
    sendBluetoothEvent("weight_search_standoff_start", "short_segment");
    beginWeightSearch(WEIGHT_SEARCH_MODE_SEARCH_WAYPOINT, true,
                      searchX, searchY, detail);
    return true;
  }

  float unitX = approachX / approachM;
  float unitY = approachY / approachM;
  float standoffX = searchX - unitX * WEIGHT_SEARCH_STANDOFF_M;
  float standoffY = searchY - unitY * WEIGHT_SEARCH_STANDOFF_M;
  sendBluetoothEvent("weight_search_standoff_start", "route_standoff");
  return navigationGoTo(standoffX, standoffY);
}

bool tryStartRouteWeightInterrupt(int routeIndex,
                                  float segmentStartX,
                                  float segmentStartY,
                                  float routeTargetX, float routeTargetY,
                                  bool currentActionIsSearch) {
  // Opportunistic route interrupt: if a confirmed object appears during a
  // route-owned navigation goal, pause the route and handle one target.
  NavigationStatus navigation = navigationGetStatus();
  if (weightInterruptCooldownActive() ||
      navigation.state != NAVIGATION_RUNNING ||
      !searchTargetVisible()) {
    return false;
  }

  WeightSearchMode mode = WEIGHT_SEARCH_MODE_ROUTE_INTERRUPT_RESUME;
  const float targetCrossTrackM = fabs(
    (weightSearchTarget.worldX - segmentStartX) *
      (routeTargetY - segmentStartY) -
    (weightSearchTarget.worldY - segmentStartY) *
      (routeTargetX - segmentStartX)) /
    max(0.001f, hypotf(routeTargetX - segmentStartX,
                       routeTargetY - segmentStartY));
  const float interceptDistanceM = hypotf(
    weightSearchTarget.worldX - robotX,
    weightSearchTarget.worldY - robotY);
  if (targetCrossTrackM > WEIGHT_INTERRUPT_MAX_CROSSTRACK_M ||
      interceptDistanceM * 2.0f > WEIGHT_INTERRUPT_MAX_ADDED_DISTANCE_M) {
    return false;
  }
  beginRouteWeightInterrupt(mode, routeIndex,
                            segmentStartX, segmentStartY,
                            routeTargetX, routeTargetY,
                            currentActionIsSearch
                              ? "search_waypoint_enroute_interrupt"
                              : "route_interrupt");
  return true;
}

void beginWaypointWeightSearch(float searchX, float searchY,
                               int resumeRouteIndex,
                               bool resumeValid,
                               float resumeX, float resumeY,
                               const char* detail) {
  weightRouteResume = {
    resumeValid, resumeRouteIndex, searchX, searchY,
    resumeX, resumeY, 0.0f, robotX, robotY, 0
  };
  beginWeightSearch(WEIGHT_SEARCH_MODE_SEARCH_WAYPOINT, true,
                    searchX, searchY, detail);
}
