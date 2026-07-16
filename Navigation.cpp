#include "Robot.h"

// =====================================================
// Compatibility entry points for incremental navigation
// =====================================================
// Responsibility:
//   Keeps older route/state-machine calls working while the actual movement
//   logic lives in LocalPlanner.cpp.
// Interacts with:
//   StateMachine.cpp calls goToPoint() for route waypoints. LocalPlanner.cpp
//   owns startNavigationPoint(), goal lifecycle, and motor command selection.
// Control flow:
//   These functions assign work and return immediately. They do not block,
//   loop until arrival, or write motor outputs.
// Global state:
//   goToPoint() may create navigationGoal state through startNavigationPoint().
//   runWaypointAction() currently prints route-action status only.
// Existing callers and test tools may still ask to go to a point.  They now
// assign a local-planner goal and return immediately; no movement loop lives
// in this module.

// Assigns a route-owned point goal if no navigation goal is already active.
//
// Inputs:
//   targetX/targetY are world-frame metres in the +X forward, +Y left frame.
// Safety:
//   Actual path safety is handled later by updateNavigationController(),
//   publishNavigationMotion(), and MotorControl.cpp.
void goToPoint(float targetX, float targetY) {
  if (isNavigationGoalActive()) {
    return;
  }
  startNavigationPoint(targetX, targetY, NAV_OWNER_ROUTE);
}

// Handles the route action string after a waypoint completes.
//
// Current behavior:
//   HOME prints a status message; PAUSE/SEARCH are handled by StateMachine.cpp
//   before or around this compatibility hook.
void runWaypointAction(const char* action) {
  Serial.print("Waypoint action: ");
  Serial.println(action);
  if (strcmp(action, "HOME") == 0) {
    Serial.println("Reached home waypoint.");
  }
}
