#include "Robot.h"

// =====================================================
// Odometry
// =====================================================
// Responsibility:
//   Integrates wheel encoder distance with IMU heading to maintain the robot's
//   estimated pose in the navigation/world frame.
// Interacts with:
//   Helpers.cpp supplies signed encoder snapshots and IMU heading conversion.
//   LocalPlanner.cpp reads robotX/robotY/robotTheta for map transforms and
//   trajectory rollout. Bluetooth.cpp reports the pose in STATUS/CSV.
// Control flow:
//   updateRobotController() calls updateOdometry() on the odometry schedule,
//   after sensor/map updates and before planning.
// Global state:
//   Reads encoder counts and IMU yaw, updates robotX/robotY in metres,
//   robotTheta in navigation degrees, lastOdom* snapshots, and heading
//   telemetry caches.

// Updates the odometry pose from encoder deltas and the latest IMU heading.
//
// LEARNING NOTE: This is a hybrid odometry approach. Encoders estimate how far
// the robot center moved; the IMU supplies the heading used to project that
// distance into world X/Y. It does not integrate encoder yaw from wheel
// difference here, so heading quality depends heavily on the BNO055.
void updateOdometry() {
  long leftCount;
  long rightCount;
  readEncoderCounts(leftCount, rightCount);

  long dLeftTicks = leftCount - lastOdomLeftCount;
  long dRightTicks = rightCount - lastOdomRightCount;

  float dLeftMetres = dLeftTicks / TICKS_PER_METRE;
  float dRightMetres = dRightTicks / TICKS_PER_METRE;

  // Average the two wheel distances to estimate travel at the midpoint
  // between the drive wheels. Units are metres because TICKS_PER_METRE has
  // already converted raw encoder ticks.
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

  // Project travelled distance into the world frame. At heading 0 deg,
  // forward motion increases +X. At +90 deg, it increases +Y.
  robotX += dCentre * cos(thetaRad);
  robotY += dCentre * sin(thetaRad);

  lastOdomLeftCount = leftCount;
  lastOdomRightCount = rightCount;
}
