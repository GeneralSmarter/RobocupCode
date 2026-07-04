// =====================================================
// ROBOCUP ROBOT CODE
// =====================================================
// V7 local-planner firmware. This sketch only performs setup and schedules
// the module-owned control loop.
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
  initializeNavigationController();

  Serial.println();
  Serial.println("V7 local-planner navigation ready.");
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

  if (robotRunEnabled && !isManualDriveActive()) {
    runStateMachine();
  } else if (!isManualDriveActive()) {
    motorStopRequested = true;
    setMotionCommand(0.0, 0.0);
  }

  updateRobotController();
}
