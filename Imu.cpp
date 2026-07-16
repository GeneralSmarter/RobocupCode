#include "Robot.h"

// =====================================================
// IMU
// =====================================================
// Responsibility:
//   Connects to the BNO055 IMU and provides yaw readings relative to the
//   robot's boot-time/zero-time orientation.
// Interacts with:
//   setup() calls connectIMU() and zeroYaw(). Helpers.cpp converts the raw
//   clockwise yaw into navigationHeadingDeg(). Odometry.cpp and LocalPlanner.cpp
//   consume that navigation heading.
// Control flow:
//   IMU connection is blocking during setup; runtime yaw reads happen through
//   readImuClockwiseYawDeg()/navigationHeadingDeg().
// Global state:
//   Reads bno, modifies yawOffset, and indirectly feeds robotTheta through
//   Odometry.cpp.

// Blocks until the BNO055 is detected, then enables its external crystal.
//
// SAFETY: Because this runs in setup() before arming, a missing IMU prevents
// the robot from entering normal motion. What could go wrong: an unplugged or
// wrong-address IMU will keep printing retries forever.
void connectIMU() {
  while (true) {
    Serial.println("Trying to find BNO055 IMU...");

    if (bno.begin()) {
      Serial.println("BNO055 connected.");
      break;
    }

    Serial.println("BNO055 not detected. Retrying...");
    delay(1000);
  }

  delay(1000);
  bno.setExtCrystalUse(true);
  delay(500);
}

// Records the current raw BNO055 yaw as zero.
//
// Units:
//   yawOffset is degrees in the IMU's raw convention. On this robot, raw yaw
//   is clockwise/right-positive after zeroing.
void zeroYaw() {
  sensors_event_t event;
  bno.getEvent(&event);
  yawOffset = event.orientation.x;

  Serial.print("Yaw zeroed at raw heading: ");
  Serial.println(yawOffset);
}

// Returns raw IMU yaw relative to yawOffset, wrapped to [-180, +180] degrees.
//
// COORDINATE CONVENTION: This function intentionally returns the installed
// BNO055 convention, which is clockwise/right-positive. Use
// navigationHeadingDeg() for navigation/map/planner math.
float readImuClockwiseYawDeg() {
  sensors_event_t event;
  bno.getEvent(&event);

  float rawYaw = event.orientation.x;
  return wrapAngle(rawYaw - yawOffset);
}
