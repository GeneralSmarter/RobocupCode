// =====================================================
// ROBOCUP ROBOT CODE
// =====================================================
// Behavior preserved from MatrixToPathV3 Step 23.
// This sketch is split into focused modules so sensor, motion,
// navigation, and state-machine work can evolve independently.
// =====================================================

#include "Robot.h"

// =====================================================
// Setup
// =====================================================
void setup() {
  Serial.begin(115200);
  setupBluetooth();
  delay(2000);

  Wire.begin();

  leftMotor.attach(LEFT_MOTOR_PIN);
  rightMotor.attach(RIGHT_MOTOR_PIN);

  pinMode(LEFT_ENC_A, INPUT_PULLUP);
  pinMode(LEFT_ENC_B, INPUT_PULLUP);
  pinMode(RIGHT_ENC_A, INPUT_PULLUP);
  pinMode(RIGHT_ENC_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(LEFT_ENC_A), leftISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_A), rightISR, CHANGE);

  stopMotors();

  connectIMU();
  connectTOFSensors();

  Serial.println("Keep robot still. Zeroing yaw...");
  delay(2000);
  zeroYaw();

  robotX = 0.0;
  robotY = 0.0;
  robotTheta = 0.0;

  Serial.println();
  Serial.println("Step 23 modular priority-order navigation ready.");
  printCalibrationSummary();
  printPose();
  printWaitingForStart();

  delay(200);
}

// =====================================================
// Main loop
// =====================================================
void loop() {
  handleBluetoothCommands();
  updateManualDriveTimeout();

  if (isManualDriveActive()) {
    delay(20);
    return;
  }

  if (!robotRunEnabled) {
    stopMotors();
    delay(50);
    return;
  }

  runStateMachine();
  handleBluetoothCommands();
}
