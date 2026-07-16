#include "Robot.h"

// =====================================================
// Encoder interrupts
// =====================================================
// Responsibility:
//   Updates the raw quadrature encoder counts for each drive wheel.
// Interacts with:
//   Hardware interrupts attached in RobotCode.ino call leftISR()/rightISR().
//   Helpers.cpp reads the volatile counts atomically in readEncoderCounts(),
//   then MotorControl.cpp and Odometry.cpp consume the signed snapshots.
// Control flow:
//   These functions run asynchronously whenever encoder channel A changes.
//   Keep them tiny: no Serial prints, no floating point, no delays.
// Global state:
//   Modifies leftRawCount and rightRawCount only. Direction sign correction is
//   deliberately deferred to readEncoderCounts() using LEFT/RIGHT_ENCODER_SIGN.

// Interrupt service routine for the physical-left wheel encoder.
//
// Inputs/outputs:
//   Reads encoder A/B pins directly and increments/decrements leftRawCount.
//   The unit is encoder ticks, not metres.
//
// DEBUGGING TIP: If forward motion produces negative signed counts, check
// LEFT_ENCODER_SIGN in RobotConfig.h before changing this ISR.
void leftISR() {
  if (digitalRead(LEFT_ENC_A) == digitalRead(LEFT_ENC_B)) {
    leftRawCount++;
  } else {
    leftRawCount--;
  }
}

// Interrupt service routine for the physical-right wheel encoder.
//
// Safety/assumptions:
//   This assumes the A/B channel relationship matches the current wiring.
//   The raw sign is allowed to differ from the left side because
//   RIGHT_ENCODER_SIGN normalizes it for the rest of the firmware.
void rightISR() {
  if (digitalRead(RIGHT_ENC_A) == digitalRead(RIGHT_ENC_B)) {
    rightRawCount++;
  } else {
    rightRawCount--;
  }
}
