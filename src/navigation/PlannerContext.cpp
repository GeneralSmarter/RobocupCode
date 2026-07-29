#include "../../Robot.h"
#include "PlannerContext.h"

PlannerContext plannerContext;

void initializePlannerContextDefaults() {
  plannerContext.lastReportedStopReason = PLANNER_STOP_NONE;
  plannerContext.lastFootprintRejectCellX = -1;
  plannerContext.lastFootprintRejectCellY = -1;
  plannerContext.lastCorridorRejectLeftM = -1.0f;
  plannerContext.lastCorridorRejectRightM = -1.0f;
  plannerContext.emergencyScanPolicyEnabled = PLANNER_EMERGENCY_SCAN_ENABLED;
}
