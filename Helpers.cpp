#include "Robot.h"

// =====================================================
// General helpers
// =====================================================
float wrapAngle(float angle) {
  while (angle > 180.0) angle -= 360.0;
  while (angle < -180.0) angle += 360.0;
  return angle;
}

float navigationHeadingDeg() {
  // The BNO055 mounting reports a physical right turn as positive yaw. The
  // robot/map convention is +Y left with counter-clockwise-positive heading,
  // so navigation geometry must use the inverse of that feedback convention.
  return wrapAngle(navigationYawFromImuClockwise(readImuClockwiseYawDeg()));
}

void resetEncodersAndPID() {
  // Encoder totals are intentionally never reset during navigation.  Local
  // planning, odometry, and recovery all need one continuous motion history.
  // A caller that wants a fresh control segment receives new snapshots instead.
  long leftCount;
  long rightCount;
  readEncoderCounts(leftCount, rightCount);

  lastLeftCount = leftCount;
  lastRightCount = rightCount;

  lastOdomLeftCount = leftCount;
  lastOdomRightCount = rightCount;

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
  Serial.print(leftForwardBaseUs);
  Serial.print("/");
  Serial.println(rightForwardBaseUs);
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
  Serial.print("Planner effective track width m (provisional until arc-calibrated): ");
  Serial.println(EFFECTIVE_TRACK_WIDTH_M, 3);
  Serial.println("Turn convention: +yaw/+turn is CCW/left; omega=(right-left)/track");
  Serial.print("Base target speed ticks/s: ");
  Serial.println(baseTargetSpeed, 1);
  Serial.print("Wheel PID Kp/Ki/Kd: ");
  Serial.print(Kp, 4);
  Serial.print("/");
  Serial.print(Ki, 4);
  Serial.print("/");
  Serial.println(Kd, 4);
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
  Serial.println("High ToF fan right-to-left:");
  Serial.println("  0 right_outer -60 deg VL53L0X XSHUT0 addr 0x30");
  Serial.println("  1 right_inner -20 deg VL53L0X XSHUT1 addr 0x31");
  Serial.println("  2 left_inner  +20 deg VL53L0X XSHUT2 addr 0x32");
  Serial.println("  3 left_outer  +60 deg VL53L0X XSHUT3 addr 0x33");
  Serial.println("  front safety is virtual: nearest valid +/-20 deg inner beam");
  Serial.println("  outer fan beams are clearance sensors, not side-wall followers");
  Serial.print("Object VL53L1X subsystem enabled: ");
  Serial.println(OBJECT_TOF_ENABLED ? "yes" : "no");
  Serial.println("  planned object ToFs: left/right LOW+UPPER on XSHUT4-7, addr 0x34-0x37");
  Serial.println("Use Bluetooth command CSV ON for maximum-rate machine-readable telemetry.");
}

void printWaitingForStart() {
  Serial.println("WAITING_FOR_START: open the CH9143 COM port at 115200 and send START.");
}
