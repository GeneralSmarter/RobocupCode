#ifndef NAVIGATION_CONTROLLER_INTERNAL_H
#define NAVIGATION_CONTROLLER_INTERNAL_H

#include "Robot.h"
#include "NavigationInternal.h"

enum SafePivotStepResult {
  SAFE_PIVOT_STEP_PUBLISHED,
  SAFE_PIVOT_STEP_REVALIDATING,
  SAFE_PIVOT_STEP_SIDE_INVALID,
  SAFE_PIVOT_STEP_SWEEP_INVALID,
  SAFE_PIVOT_STEP_SWEEP_BLOCKED,
  SAFE_PIVOT_STEP_STUCK,
  SAFE_PIVOT_STEP_PUBLICATION_VETOED
};

bool publishNavigationMotion(float forwardSpeed, float turnSpeed);
bool ownerIsObjectHunt(NavigationGoalOwner owner);
void finishNavigationGoal(bool success, PlannerStopReason reason,
                          const char* detail);
SafePivotStepResult commandSafePivotStep(
  float turnTarget, const char* motionPlanReason,
  const char* sideRevalidatePlanReason,
  const char* sweepRevalidatePlanReason);

#endif
