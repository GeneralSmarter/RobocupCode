#include "Robot.h"

// =====================================================
// General helpers
// =====================================================
float wrapAngle(float angle) {
  while (angle > 180.0) angle -= 360.0;
  while (angle < -180.0) angle += 360.0;
  return angle;
}

void resetEncodersAndPID() {
  noInterrupts();
  leftRawCount = 0;
  rightRawCount = 0;
  interrupts();

  lastLeftCount = 0;
  lastRightCount = 0;

  lastOdomLeftCount = 0;
  lastOdomRightCount = 0;

  leftIntegral = 0.0;
  rightIntegral = 0.0;

  lastLeftError = 0.0;
  lastRightError = 0.0;
}

void readEncoderCounts(long &leftCount, long &rightCount) {
  long leftRaw;
  long rightRaw;

  noInterrupts();
  leftRaw = leftRawCount;
  rightRaw = rightRawCount;
  interrupts();

  leftCount = LEFT_ENCODER_SIGN * leftRaw;
  rightCount = RIGHT_ENCODER_SIGN * rightRaw;
}

void printPose() {
  Serial.print("POSE  x: ");
  Serial.print(robotX, 3);
  Serial.print(" m   y: ");
  Serial.print(robotY, 3);
  Serial.print(" m   theta: ");
  Serial.print(robotTheta, 2);
  Serial.println(" deg");
}

void printCalibrationSummary() {
  Serial.println();
  Serial.println("CALIBRATION SUMMARY");
  Serial.print("Board target: Teensy 4.0");
  Serial.println();
  Serial.print("Motor us stop/min/max: ");
  Serial.print(STOP_US);
  Serial.print("/");
  Serial.print(MIN_US);
  Serial.print("/");
  Serial.println(MAX_US);
  Serial.print("Motor base L/R us: ");
  Serial.print(LEFT_BASE_US);
  Serial.print("/");
  Serial.println(RIGHT_BASE_US);
  Serial.print("Motor reverse L/R us: ");
  Serial.print(LEFT_REVERSE_US);
  Serial.print("/");
  Serial.println(RIGHT_REVERSE_US);
  Serial.print("Encoder signs L/R: ");
  Serial.print(LEFT_ENCODER_SIGN);
  Serial.print("/");
  Serial.println(RIGHT_ENCODER_SIGN);
  Serial.print("Ticks per metre: ");
  Serial.println(TICKS_PER_METRE, 2);
  Serial.print("Base target speed ticks/s: ");
  Serial.println(baseTargetSpeed, 1);
  Serial.print("Wheel PID Kp/Ki/Kd: ");
  Serial.print(Kp, 4);
  Serial.print("/");
  Serial.print(Ki, 4);
  Serial.print("/");
  Serial.println(Kd, 4);
  Serial.print("Heading gain: ");
  Serial.println(K_heading, 3);
  Serial.print("Waypoint tolerance m: ");
  Serial.println(WAYPOINT_TOLERANCE_M, 3);
  Serial.print("Front ToF stop/clear mm: ");
  Serial.print(FRONT_STOP_DISTANCE_MM);
  Serial.print("/");
  Serial.println(FRONT_CLEAR_DISTANCE_MM);
  Serial.print("Front ToF valid min/max mm: ");
  Serial.print(FRONT_VALID_MIN_MM);
  Serial.print("/");
  Serial.println(FRONT_VALID_MAX_MM);
  Serial.print("ToF stale timeout ms: ");
  Serial.println(TOF_STALE_TIMEOUT_MS);
  Serial.print("Wall follow enabled: ");
  Serial.println(ENABLE_SIDE_WALL_FOLLOW_FALLBACK ? "yes" : "no");
  Serial.print("Wall follow target mm: ");
  Serial.println(WALL_FOLLOW_TARGET_MM);
  Serial.println("Use Bluetooth command CSV ON for once-per-second machine-readable telemetry.");
}

void printWaitingForStart() {
  Serial.println("WAITING_FOR_START: open the CH9143 COM port at 115200 and send START.");
}
