#ifndef TURN_CONVENTION_H
#define TURN_CONVENTION_H

// Canonical navigation convention:
//   +X forward, +Y left, +yaw/+turn counter-clockwise (robot-left).
// Wheel speeds are forward-positive, so a positive chassis turn makes the
// right wheel faster than the left wheel.
constexpr float leftWheelTargetFromChassis(float forward, float turn) {
  return forward - turn;
}

constexpr float rightWheelTargetFromChassis(float forward, float turn) {
  return forward + turn;
}

constexpr float navigationOmegaFromWheelSpeeds(float leftSpeed,
                                                float rightSpeed,
                                                float trackWidth) {
  return (rightSpeed - leftSpeed) / trackWidth;
}

// The installed BNO055 reports clockwise/right yaw as positive after zeroing.
constexpr float navigationYawFromImuClockwise(float imuClockwiseYawDeg) {
  return -imuClockwiseYawDeg;
}

static_assert(leftWheelTargetFromChassis(0.0f, 1.0f) < 0.0f,
              "Positive turn must reverse/slow the left wheel");
static_assert(rightWheelTargetFromChassis(0.0f, 1.0f) > 0.0f,
              "Positive turn must advance/speed the right wheel");
static_assert(navigationOmegaFromWheelSpeeds(-1.0f, 1.0f, 1.0f) > 0.0f,
              "Right-minus-left wheel speed must produce positive yaw");
static_assert(navigationYawFromImuClockwise(-1.0f) > 0.0f,
              "A physical left turn must produce positive navigation yaw");

#endif
