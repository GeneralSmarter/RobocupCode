#include "../../Robot.h"
#include "../../Navigation.h"
#include "RouteMission.h"
#include "WeightSearch.h"

// The onboard route is mission data. Navigation sees only one world-frame
// point at a time and never reads this array or advances its index.
static const Waypoint ROUTE[] = {
  {1.20f, 0.00f, MISSION_ACTION_PAUSE},
  {1.20f, 0.80f, MISSION_ACTION_PAUSE},
  {0.00f, 0.80f, MISSION_ACTION_PAUSE},
  {0.00f, 0.00f, MISSION_ACTION_HOME}
};

static constexpr int ROUTE_POINT_COUNT =
  sizeof(ROUTE) / sizeof(ROUTE[0]);

static int currentRouteIndex = 0;
static unsigned long routePauseUntilMs = 0;
static unsigned long routeRetryAfterMs = 0;
static bool routeGoalPending = false;
static bool competitionModeEnabled = false;

static const char* missionActionName(MissionAction action) {
  switch (action) {
    case MISSION_ACTION_PAUSE: return "PAUSE";
    case MISSION_ACTION_SEARCH: return "SEARCH";
    case MISSION_ACTION_HOME: return "HOME";
  }
  return "UNKNOWN";
}

static bool currentActionIsSearch() {
  return currentRouteIndex < ROUTE_POINT_COUNT &&
         ROUTE[currentRouteIndex].action == MISSION_ACTION_SEARCH;
}

static void printCompletedAction(MissionAction action) {
  Serial.print("Waypoint action: ");
  Serial.println(missionActionName(action));
  if (action == MISSION_ACTION_HOME) {
    Serial.println("Reached home waypoint.");
  }
}

void resetRouteMission() {
  currentRouteIndex = 0;
  routePauseUntilMs = 0;
  routeRetryAfterMs = 0;
  routeGoalPending = false;
}

bool setCompetitionModeEnabled(bool enabled) {
  // Policy changes are allowed only while the robot is stopped. This keeps a
  // running goal's retry semantics stable until an explicit STOP/restart.
  if (robotRunEnabled || motionAuthority != MOTION_AUTHORITY_NONE ||
      navigationGetStatus().state == NAVIGATION_RUNNING ||
      lastLeftMotorUs != STOP_US || lastRightMotorUs != STOP_US ||
      isMotorCommandLeaseArmed()) {
    return false;
  }
  competitionModeEnabled = enabled;
  return true;
}

bool isCompetitionModeEnabled() {
  return competitionModeEnabled;
}

void initializeRouteMission() {
  resetRouteMission();
  initializeWeightSearch();
}

int routeMissionPointCount() {
  return ROUTE_POINT_COUNT;
}

int routeMissionDisplayIndex() {
  if (ROUTE_POINT_COUNT <= 0) {
    return 0;
  }
  if (currentRouteIndex >= ROUTE_POINT_COUNT) {
    return ROUTE_POINT_COUNT;
  }
  return currentRouteIndex + 1;
}

static bool consumeFinishedWeightSearch() {
  WeightSearchStatus search = getWeightSearchStatus();
  if (search.result == WEIGHT_SEARCH_RESULT_RUNNING ||
      search.result == WEIGHT_SEARCH_RESULT_IDLE) {
    return false;
  }

  const bool routeAlreadyRunning =
    navigationGetStatus().state == NAVIGATION_RUNNING;
  if (!routeAlreadyRunning) {
    navigationClearResult();
    routeGoalPending = false;
  }
  if (search.result == WEIGHT_SEARCH_RESULT_FAILED) {
    if (competitionModeEnabled &&
        search.origin != WEIGHT_SEARCH_ORIGIN_TEST) {
      if (search.origin == WEIGHT_SEARCH_ORIGIN_WAYPOINT) {
        // Search is an optional scoring action. A failed hunt must not end the
        // match; advance past that action and keep the route alive.
        currentRouteIndex++;
      }
      routePauseUntilMs = millis() + COMPETITION_RETRY_PAUSE_MS;
      sendBluetoothEvent(
        "competition_search_continue", "weight_search_failed");
      clearWeightSearchResult();
      return true;
    }
    clearWeightSearchResult();
    setRobotState(END_MATCH);
    return true;
  }

  if (search.origin == WEIGHT_SEARCH_ORIGIN_TEST) {
    clearWeightSearchResult();
    setRobotState(END_MATCH);
    return true;
  }

  if (search.origin == WEIGHT_SEARCH_ORIGIN_WAYPOINT) {
    currentRouteIndex++;
  }
  if (routeAlreadyRunning) {
    routeGoalPending = true;
    clearWeightSearchResult();
    return true;
  }
  routePauseUntilMs = millis() + WAYPOINT_ACTION_PAUSE_MS;
  clearWeightSearchResult();
  return true;
}

void updateRouteMission() {
  if (isWeightSearchActive()) {
    updateWeightSearch();
    if (isWeightSearchActive()) {
      return;
    }
  }
  if (consumeFinishedWeightSearch()) {
    return;
  }

  NavigationStatus navigation = navigationGetStatus();
  if (navigation.state == NAVIGATION_RUNNING) {
    if (routeGoalPending && currentRouteIndex < ROUTE_POINT_COUNT &&
        tryStartRouteWeightInterrupt(
          currentRouteIndex,
          currentRouteIndex > 0 ? ROUTE[currentRouteIndex - 1].x : robotX,
          currentRouteIndex > 0 ? ROUTE[currentRouteIndex - 1].y : robotY,
          ROUTE[currentRouteIndex].x,
          ROUTE[currentRouteIndex].y,
          currentActionIsSearch())) {
      routeGoalPending = false;
    }
    return;
  }

  if (routeGoalPending && navigation.state == NAVIGATION_FAILED) {
    if (competitionModeEnabled) {
      const unsigned long now = millis();
      if (routeRetryAfterMs == 0) {
        routeRetryAfterMs = now + COMPETITION_RETRY_PAUSE_MS;
        requestMotionStop();
        sendBluetoothEvent("competition_route_retry_wait",
                           navigation.detail);
        return;
      }
      if (now < routeRetryAfterMs) {
        requestMotionStop();
        return;
      }
      navigationClearResult();
      routeGoalPending = false;
      routeRetryAfterMs = 0;
      sendBluetoothEvent("competition_route_retry", "navigation_failed");
      return;
    }
    // Keep the typed navigation failure visible and neutral in normal mode.
    return;
  }

  if (routeGoalPending && navigation.state == NAVIGATION_REACHED) {
    const Waypoint &waypoint = ROUTE[currentRouteIndex];
    navigationClearResult();
    routeGoalPending = false;
    routeRetryAfterMs = 0;
    if (waypoint.action == MISSION_ACTION_SEARCH) {
      const bool resumeValid = currentRouteIndex + 1 < ROUTE_POINT_COUNT;
      beginWaypointWeightSearch(
                                waypoint.x, waypoint.y,
                                currentRouteIndex + 1,
                                resumeValid,
                                resumeValid ? ROUTE[currentRouteIndex + 1].x : 0.0f,
                                resumeValid ? ROUTE[currentRouteIndex + 1].y : 0.0f,
                                missionActionName(waypoint.action));
      return;
    }
    printCompletedAction(waypoint.action);
    currentRouteIndex++;
    routePauseUntilMs = millis() + WAYPOINT_ACTION_PAUSE_MS;
  }

  if (currentRouteIndex >= ROUTE_POINT_COUNT) {
    setRobotState(END_MATCH);
    return;
  }

  if (routePauseUntilMs != 0) {
    if (millis() < routePauseUntilMs) {
      requestMotionStop();
      return;
    }
    routePauseUntilMs = 0;
  }

  const Waypoint &waypoint = ROUTE[currentRouteIndex];
  Serial.print("Waypoint ");
  Serial.print(currentRouteIndex + 1);
  Serial.print(" of ");
  Serial.println(ROUTE_POINT_COUNT);

  if (waypoint.action == MISSION_ACTION_SEARCH) {
    const float originX = currentRouteIndex > 0
      ? ROUTE[currentRouteIndex - 1].x : robotX;
    const float originY = currentRouteIndex > 0
      ? ROUTE[currentRouteIndex - 1].y : robotY;
    startSearchWaypointApproach(
      waypoint.x, waypoint.y, originX, originY,
      missionActionName(waypoint.action));
    routeGoalPending = navigationGetStatus().state == NAVIGATION_RUNNING;
    return;
  }

  routeGoalPending = navigationGoTo(waypoint.x, waypoint.y);
}
