#ifndef PLANNER_DEBUG_H
#define PLANNER_DEBUG_H

#include "../../RobotTypes.h"

struct NavigationDebugStatus {
  bool active;
  bool completed;
  bool failed;
  bool testTurnActive;
  MotionAuthority authority;
};

NavigationDebugStatus getNavigationDebugStatus();
const char* plannerStopReasonName(PlannerStopReason reason);
PlannerDebugSnapshot getPlannerDebugSnapshot();
const PlannerTelemetry& getPlannerTelemetry();
int plannerDebugMapState(float worldX, float worldY);
int plannerDebugSeedMapOccupied(float worldX, float worldY);
bool plannerDebugForceRecoveryExhaustion();

#endif
