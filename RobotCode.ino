// =====================================================
// ROBOCUP ROBOT CODE
// =====================================================
// Responsibility:
//   Arduino entry point for the active V7 Teensy 4.0 robot firmware.
//   This file owns hardware startup and the top-level loop schedule only.
// Interacts with:
//   Robot.h for all shared objects, Bluetooth.cpp for operator commands,
//   StateMachine.cpp for mission decisions, LocalPlanner.cpp for sensing,
//   odometry and navigation updates, and MotorControl.cpp for the only
//   periodic servo output path.
// Control flow:
//   setup() initializes hardware and shared state once. loop() repeatedly
//   services safety, command input, mission logic, the controller pipeline,
//   and queued telemetry.
// Global state:
//   Initializes motors, encoder interrupts, IMU/ToF hardware, pose
//   (robotX/robotY/robotTheta), navigation controller state, timing
//   diagnostics, and motor safety watchdog state.
// =====================================================

#include "Robot.h"

// =====================================================
// Setup
// =====================================================
// Called once by the Arduino runtime after reset or upload.
//
// LEARNING NOTE: Arduino sketches do not have a visible main() here. The
// Teensy/Arduino core calls setup() once, then calls loop() forever.
//
// SAFETY: setup() attaches motors, immediately writes neutral, and starts the
// motor lease/watchdog before any navigation goal can be created. It also
// blocks until the IMU and navigation ToF sensors are connected, so the robot
// does not enter the normal control loop with missing required hardware.
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
  initializeMotorSafetyWatchdog();

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
// Called repeatedly by the Arduino runtime.
//
// CONTROL FLOW: This is the complete normal runtime order. Command parsing can
// arm/start/stop modes, the state machine can assign navigation goals, and
// updateRobotController() performs sensing, map updates, odometry, planning,
// motor control, and telemetry construction.
//
// SAFETY: The motor watchdog is serviced before command/state/controller work,
// and updateRobotController() ends at updateMotorController(), the sole
// periodic servo writer. If robotRunEnabled is false and manual drive is not
// active, this loop requests neutral instead of letting an old goal keep
// publishing commands.
void loop() {
  noteMainLoopHeartbeat();
  unsigned long phaseStartedUs = micros();
  serviceMotorSafetyWatchdog();
  recordMainLoopPhaseDuration("watchdog", phaseStartedUs);

  phaseStartedUs = micros();
  handleBluetoothCommands();
  updateManualDriveTimeout();
  recordMainLoopPhaseDuration("bluetooth_rx", phaseStartedUs);

  phaseStartedUs = micros();
  if (robotRunEnabled && !isManualDriveActive()) {
    runStateMachine();
  } else if (!isManualDriveActive()) {
    motorStopRequested = true;
    setMotionCommand(0.0, 0.0);
  }
  recordMainLoopPhaseDuration("state_machine", phaseStartedUs);

  phaseStartedUs = micros();
  updateRobotController();
  recordMainLoopPhaseDuration("robot_controller", phaseStartedUs);

  phaseStartedUs = micros();
  serviceBluetoothTelemetryTx();
  recordMainLoopPhaseDuration("bluetooth_tx", phaseStartedUs);
}
