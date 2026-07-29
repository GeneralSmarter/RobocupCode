#ifndef NAVIGATION_INTERNAL_H
#define NAVIGATION_INTERNAL_H

#include "Navigation.h"

enum NavigationGoalMode {
  NAV_GOAL_NONE,
  NAV_GOAL_POINT,
  NAV_GOAL_TURN
};

enum NavigationGoalOwner {
  NAV_OWNER_ROUTE,
  NAV_OWNER_RETURN_HOME,
  NAV_OWNER_TEST_DRIVE,
  NAV_OWNER_TEST_GOTO,
  NAV_OWNER_TEST_AVOID,
  NAV_OWNER_TEST_ESCAPE,
  NAV_OWNER_TEST_TURN,
  NAV_OWNER_TEST_HUNT,
  NAV_OWNER_WEIGHT_SCAN,
  NAV_OWNER_OBJECT_HUNT
};

struct NavigationGoal {
  NavigationGoalMode mode;
  NavigationGoalOwner owner;
  MotionAuthority authority;
  bool active;
  bool completed;
  bool failed;
  float targetX;
  float targetY;
  float targetYawDeg;
  float startX;
  float startY;
  float startYawDeg;
  unsigned long startedMs;
};

struct NavigationInternalStatus {
  bool active;
  bool completed;
  bool failed;
  NavigationGoalMode mode;
  NavigationGoalOwner owner;
  MotionAuthority authority;
  PlannerStopReason stopReason;
  const char* detail;
};

// Mutable goal storage and lifecycle helpers are private to navigation.
extern NavigationGoal navigationGoal;

void initializeNavigationController();
void updateNavigationController();
bool startNavigationPoint(float targetX, float targetY,
                          NavigationGoalOwner owner);
bool startNavigationTurn(float relativeTurnDeg, NavigationGoalOwner owner);
void cancelNavigationGoal(PlannerStopReason reason, const char* detail);
NavigationInternalStatus getNavigationInternalStatus();
void clearNavigationGoalResult();
bool setEmergencyScanPolicyEnabled(bool enabled);
bool isEmergencyScanPolicyEnabled();
void clearLocalMap();

#endif
