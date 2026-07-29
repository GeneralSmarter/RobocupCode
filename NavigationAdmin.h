#ifndef NAVIGATION_ADMIN_H
#define NAVIGATION_ADMIN_H

#include "RobotTypes.h"

// Operator/system controls intentionally kept out of the mission-facing API.
void navigationCancelWithReason(PlannerStopReason reason, const char* detail);
bool navigationSetEmergencyScanEnabled(bool enabled);
bool navigationIsEmergencyScanEnabled();
void navigationResetMap();

#endif
