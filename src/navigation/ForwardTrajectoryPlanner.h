#ifndef FORWARD_TRAJECTORY_PLANNER_H
#define FORWARD_TRAJECTORY_PLANNER_H

#include "PlannerTypes.h"

// Internal lifecycle hook used when obstacle geometry invalidates an
// in-progress cooperative forward-planning epoch.
void resetPlannerEpoch();
void closePlannerEpoch();
void resetGeometricNoPathEvidence();
void noteGeometricNoPathEpoch();
bool currentPlannerFailureIsGeometricNoPath();
TrajectoryPlanResult selectTrajectory(float goalX, float goalY);

#endif
