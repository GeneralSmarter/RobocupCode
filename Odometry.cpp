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

  // navigationHeadingDeg() is CCW/left-positive. At +90 degrees, positive
  // centre travel therefore increases world +Y.
  // Cache both frames from one IMU sample so telemetry can prove the raw-to-nav
  // sign conversion without adding extra blocking I2C reads in the logger.
  lastImuClockwiseYawDeg = readImuClockwiseYawDeg();
  lastNavigationHeadingDeg = wrapAngle(
    navigationYawFromImuClockwise(lastImuClockwiseYawDeg));
  robotTheta = lastNavigationHeadingDeg;
  float thetaRad = robotTheta * PI / 180.0;

  robotX += dCentre * cos(thetaRad);
  robotY += dCentre * sin(thetaRad);

  lastOdomLeftCount = leftCount;
  lastOdomRightCount = rightCount;
}
