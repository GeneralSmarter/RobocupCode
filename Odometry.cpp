#include "Robot.h"

// =====================================================
// Odometry
// =====================================================
void updateOdometry() {
  long leftCount;
  long rightCount;
  readEncoderCounts(leftCount, rightCount);

  long dLeftTicks = leftCount - lastOdomLeftCount;
  long dRightTicks = rightCount - lastOdomRightCount;

  float dLeftMetres = dLeftTicks / TICKS_PER_METRE;
  float dRightMetres = dRightTicks / TICKS_PER_METRE;

  float dCentre = (dLeftMetres + dRightMetres) / 2.0;

  robotTheta = navigationHeadingDeg();
  float thetaRad = robotTheta * PI / 180.0;

  robotX += dCentre * cos(thetaRad);
  robotY += dCentre * sin(thetaRad);

  lastOdomLeftCount = leftCount;
  lastOdomRightCount = rightCount;
}
