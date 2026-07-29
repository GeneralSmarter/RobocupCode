#ifndef PLANNER_PROGRESS_H
#define PLANNER_PROGRESS_H

#include "Robot.h"

bool routeLineFrame(float &routeLengthM, float &routeUx,
                    float &routeUy, float &routeHeadingRad);
float routeLineAlongM(float worldX, float worldY,
                      float routeUx, float routeUy);
float routeLineLateralErrorM(float worldX, float worldY,
                             float routeUx, float routeUy);
float routeLineSignedLateralErrorM(float worldX, float worldY,
                                   float routeUx, float routeUy);
bool routeLineTrackingEligible(float &routeLengthM, float &routeUx,
                               float &routeUy, float &routeHeadingRad);

#endif
