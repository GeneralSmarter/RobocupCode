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
// Validates the exact matrix-hunt/feed chassis command against the same live
// fan observation envelope and swept-footprint map used by normal navigation.
// Only the latched target's small capture region is excluded from occupancy;
// all other collision evidence remains authoritative.
bool pickupTrajectoryCommandSafe(float forwardTicks, float turnTicks,
                                 const MatrixTargetObservation &target);

#endif
