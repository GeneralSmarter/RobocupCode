#include "Robot.h"

// =====================================================
// Compatibility entry points for incremental navigation
// =====================================================
// Existing callers and test tools may still ask to go to a point.  They now
// assign a local-planner goal and return immediately; no movement loop lives
// in this module.

void goToPoint(float targetX, float targetY) {
  if (isNavigationGoalActive()) {
    return;
  }
  startNavigationPoint(targetX, targetY, NAV_OWNER_ROUTE);
}

void runWaypointAction(const char* action) {
  Serial.print("Waypoint action: ");
  Serial.println(action);
  if (strcmp(action, "HOME") == 0) {
    Serial.println("Reached home waypoint.");
  }
}
