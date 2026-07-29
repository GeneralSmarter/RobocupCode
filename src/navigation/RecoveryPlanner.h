#ifndef RECOVERY_PLANNER_H
#define RECOVERY_PLANNER_H

#include "PlannerTypes.h"

void resetReversePlannerEpoch();
void resetRecoveryBudget();
bool handleRecoveryExhaustion(PlannerStopReason reason, const char* detail);
bool updateReverseRecoveryBudget();
TrajectoryPlanResult selectReverseRecoveryTrajectory(float goalX,
                                                       float goalY);
void updateEmergencyRecovery();
bool canStartSafeReverse();
bool canStartEvidenceDrivenReverse();
void startEvidenceDrivenReverse(const char* trigger);
void completeEvidenceDrivenReverse();

#endif
