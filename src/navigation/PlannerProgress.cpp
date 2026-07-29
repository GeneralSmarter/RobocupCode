#include "Robot.h"
#include "NavigationInternal.h"
#include "PlannerProgress.h"

bool routeLineFrame(float &routeLengthM, float &routeUx,
                           float &routeUy, float &routeHeadingRad) {
  if (navigationGoal.mode != NAV_GOAL_POINT) {
    return false;
  }

  float routeDx = navigationGoal.targetX - navigationGoal.startX;
  float routeDy = navigationGoal.targetY - navigationGoal.startY;
  routeLengthM = sqrtf(routeDx * routeDx + routeDy * routeDy);
  if (routeLengthM <= WAYPOINT_TOLERANCE_M) {
    return false;
  }

  routeUx = routeDx / routeLengthM;
  routeUy = routeDy / routeLengthM;
  routeHeadingRad = atan2f(routeDy, routeDx);
  return true;
}

float routeLineAlongM(float worldX, float worldY,
                             float routeUx, float routeUy) {
  return (worldX - navigationGoal.startX) * routeUx +
         (worldY - navigationGoal.startY) * routeUy;
}

float routeLineLateralErrorM(float worldX, float worldY,
                                    float routeUx, float routeUy) {
  return fabs(-(worldX - navigationGoal.startX) * routeUy +
              (worldY - navigationGoal.startY) * routeUx);
}

float routeLineSignedLateralErrorM(float worldX, float worldY,
                                          float routeUx, float routeUy) {
  // Positive is left of the nominal start->target route frame.
  return -(worldX - navigationGoal.startX) * routeUy +
         (worldY - navigationGoal.startY) * routeUx;
}

bool routeLineTrackingEligible(float &routeLengthM, float &routeUx,
                                      float &routeUy, float &routeHeadingRad) {
  if (!routeLineFrame(routeLengthM, routeUx, routeUy, routeHeadingRad)) {
    return false;
  }

  float routeHeadingDeg = routeHeadingRad * RAD_TO_DEG;
  return fabs(wrapAngle(routeHeadingDeg - navigationHeadingDeg())) <=
    PLANNER_LINE_FOLLOW_ENABLE_HEADING_DEG;
}
