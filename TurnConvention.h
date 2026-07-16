#ifndef TURN_CONVENTION_H
#define TURN_CONVENTION_H

// =====================================================
// Canonical turn, yaw, and wheel-sign convention
// =====================================================
// Responsibility:
//   Keeps the robot's navigation sign convention in one tiny, testable place.
//   Any module converting between chassis commands, wheel speeds, IMU yaw, or
//   odometry should use these helpers or match these formulas exactly.
// Interacts with:
//   MotorControl.cpp mixes forward/turn into wheel targets, LocalPlanner.cpp
//   rolls out differential-drive arcs, Odometry.cpp integrates heading, and
//   Helpers.cpp converts raw BNO055 yaw into navigation heading.
// Control flow:
//   Header-only constexpr helpers plus static_assert invariants. No runtime
//   state is modified here.
// Global state:
//   None.

// Canonical navigation convention:
//   +X forward, +Y left, +yaw/+turn counter-clockwise (robot-left).
// Wheel speeds are forward-positive, so a positive chassis turn makes the
// right wheel faster than the left wheel.
//
// COORDINATE CONVENTION: If the robot is commanded with forward=0 and turn>0,
// it should rotate left/CCW. The left wheel target becomes negative/slower and
// the right wheel target becomes positive/faster.
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
