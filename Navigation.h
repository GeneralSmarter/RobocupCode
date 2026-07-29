#ifndef NAVIGATION_H
#define NAVIGATION_H

#include "RobotTypes.h"

// The complete mission-facing driving API. Mission and pickup state machines
// submit one nonblocking goal, poll its status, then decide what happens next.
bool navigationGoTo(float worldX, float worldY);
bool navigationGoToPickup(float worldX, float worldY);
bool navigationTurnBy(float relativeDegrees);
bool navigationScanTurnBy(float relativeDegrees);
void navigationCancel();
NavigationStatus navigationGetStatus();
void navigationClearResult();

#endif
