#ifndef NAVIGATION_H
#define NAVIGATION_H

#include "RobotTypes.h"

// The complete mission-facing driving API. Mission and pickup state machines
// submit one nonblocking goal, poll its status, then decide what happens next.
bool navigationGoTo(float worldX, float worldY);
bool navigationStartPickupTracking(const MatrixTargetObservation &target,
                                   const RouteResumeContext &resume);
bool navigationUpdatePickupTracking(const MatrixTargetObservation &target);
PickupTrackingStatus navigationGetPickupTrackingStatus();
bool navigationTurnBy(float relativeDegrees);
bool navigationScanTurnBy(float relativeDegrees);
void navigationCancel();
NavigationStatus navigationGetStatus();
void navigationClearResult();
bool navigationRetargetDiagnosticPoint(float worldX, float worldY);

#endif
