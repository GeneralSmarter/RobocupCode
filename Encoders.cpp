#include "Robot.h"

// =====================================================
// Encoder interrupts
// =====================================================
void leftISR() {
  if (digitalRead(LEFT_ENC_A) == digitalRead(LEFT_ENC_B)) {
    leftRawCount++;
  } else {
    leftRawCount--;
  }
}

void rightISR() {
  if (digitalRead(RIGHT_ENC_A) == digitalRead(RIGHT_ENC_B)) {
    rightRawCount++;
  } else {
    rightRawCount--;
  }
}
