#include "Robot.h"
#include <ctype.h>
#include <stdlib.h>
#include <string.h>

// =====================================================
// Bluetooth serial debug link on Serial2
// =====================================================
static char bluetoothCommand[64];
static int bluetoothCommandLength = 0;
static bool bluetoothAbortMotionRequested = false;
static bool bluetoothStreamEnabled = false;
static bool bluetoothCsvStreamEnabled = false;
static bool bluetoothTestArmed = false;
static bool bluetoothManualArmed = false;
static bool bluetoothManualActive = false;
static unsigned long lastManualDriveCommandMs = 0;
static unsigned long lastBluetoothTelemetryMs = 0;

const float TEST_DRIVE_MIN_METRES = 0.01;
const float TEST_DRIVE_MAX_METRES = 1.50;
const float TEST_TURN_MIN_DEG = -360.0;
const float TEST_TURN_MAX_DEG = 360.0;
const float SPEED_MIN_TICKS_PER_SEC = 300.0;
const float SPEED_MAX_TICKS_PER_SEC = 3000.0;
const float HGAIN_MIN = 0.0;
const float HGAIN_MAX = 100.0;
const float FBASE_MIN_US = STOP_US;
const float FBASE_MAX_US = MAX_US;
const float MANUAL_COMMAND_MIN = -100.0;
const float MANUAL_COMMAND_MAX = 100.0;
const unsigned long MANUAL_DRIVE_TIMEOUT_MS = 350;

static bool commandEquals(const char* command, const char* expected) {
  while (*command != '\0' && *expected != '\0') {
    if (toupper(*command) != toupper(*expected)) {
      return false;
    }

    command++;
    expected++;
  }

  return *command == '\0' && *expected == '\0';
}

static bool commandHasPrefix(const char* command, const char* prefix) {
  while (*command != '\0' && *prefix != '\0') {
    if (toupper(*command) != toupper(*prefix)) {
      return false;
    }

    command++;
    prefix++;
  }

  if (*prefix != '\0') {
    return false;
  }

  return *command == '\0' || isspace(*command);
}

static const char* commandArgument(const char* command, const char* prefix) {
  const char* argument = command + strlen(prefix);

  while (*argument != '\0' && isspace(*argument)) {
    argument++;
  }

  return argument;
}

static bool parseFloatArgument(const char* text, float &value) {
  char* endPtr = NULL;
  double parsedValue = strtod(text, &endPtr);

  if (endPtr == text) {
    return false;
  }

  if (parsedValue != parsedValue) {
    return false;
  }

  while (*endPtr != '\0' && isspace(*endPtr)) {
    endPtr++;
  }

  value = (float)parsedValue;
  return *endPtr == '\0';
}

static bool parseTwoFloatArguments(const char* text, float &first, float &second) {
  char* endPtr = NULL;
  double firstValue = strtod(text, &endPtr);

  if (endPtr == text || firstValue != firstValue) {
    return false;
  }

  while (*endPtr != '\0' && isspace(*endPtr)) {
    endPtr++;
  }

  char* secondEndPtr = NULL;
  double secondValue = strtod(endPtr, &secondEndPtr);

  if (secondEndPtr == endPtr || secondValue != secondValue) {
    return false;
  }

  while (*secondEndPtr != '\0' && isspace(*secondEndPtr)) {
    secondEndPtr++;
  }

  first = (float)firstValue;
  second = (float)secondValue;
  return *secondEndPtr == '\0';
}

static void trimCommand(char* command) {
  int length = strlen(command);

  while (length > 0 && isspace(command[length - 1])) {
    command[length - 1] = '\0';
    length--;
  }

  int start = 0;
  while (command[start] != '\0' && isspace(command[start])) {
    start++;
  }

  if (start > 0) {
    int writeIndex = 0;
    while (command[start] != '\0') {
      command[writeIndex] = command[start];
      writeIndex++;
      start++;
    }
    command[writeIndex] = '\0';
  }
}

static void printBluetoothHelp() {
  Serial2.println("Commands:");
  Serial2.println("  HELP or H      show this help");
  Serial2.println("  START          start robot navigation");
  Serial2.println("  STATUS or P    print robot status");
  Serial2.println("  STREAM ON      send status once per second");
  Serial2.println("  STREAM OFF     stop periodic status");
  Serial2.println("  CSV ON         send CSV telemetry once per second");
  Serial2.println("  CSV OFF        stop CSV telemetry");
  Serial2.println("  CAL            print calibration summary");
  Serial2.println("  SPEED <ticks>  set temporary base target speed");
  Serial2.println("  SPEED RESET    restore default base target speed");
  Serial2.println("  HGAIN <gain>   set temporary heading correction gain");
  Serial2.println("  HGAIN RESET    restore default heading correction gain");
  Serial2.println("  FBASE <l> <r>  set temporary forward motor base pulses");
  Serial2.println("  FBASE RESET    restore default forward motor base pulses");
  Serial2.println("  TEST ARM       allow one or more test motion commands");
  Serial2.println("  TEST DISARM    disable test motion commands");
  Serial2.println("  TEST DRIVE <m> drive a fixed distance at current heading");
  Serial2.println("  TEST TURN <d>  turn a fixed signed angle in degrees");
  Serial2.println("  MANUAL ARM     allow live DRIVE commands");
  Serial2.println("  MANUAL DISARM  stop and disable live DRIVE commands");
  Serial2.println("  DRIVE <f> <t>  live drive, forward/turn from -100 to 100");
  Serial2.println("  HOME           request return home");
  Serial2.println("  ZERO           zero yaw and pose");
  Serial2.println("  STOP or S      stop motors and end match");
}

void setupBluetooth() {
  Serial2.begin(BLUETOOTH_BAUD);
  bluetoothOutputEnabled = true;
  delay(100);

  Serial2.println();
  Serial2.println("CH9143 Bluetooth link ready on Serial2.");
  printBluetoothHelp();
}

static int displayWaypointIndex() {
  if (NUM_POINTS <= 0) {
    return 0;
  }

  if (currentWaypointIndex >= NUM_POINTS) {
    return NUM_POINTS;
  }

  return currentWaypointIndex + 1;
}

void sendBluetoothStatus() {
  updateTOFSensors();

  long leftCount;
  long rightCount;
  readEncoderCounts(leftCount, rightCount);

  Serial2.print("STATUS state=");
  Serial2.print(robotStateName(currentState));
  Serial2.print(" run=");
  Serial2.print(robotRunEnabled ? 1 : 0);
  Serial2.print(" testArmed=");
  Serial2.print(bluetoothTestArmed ? 1 : 0);
  Serial2.print(" manualArmed=");
  Serial2.print(bluetoothManualArmed ? 1 : 0);
  Serial2.print(" manualActive=");
  Serial2.print(bluetoothManualActive ? 1 : 0);
  Serial2.print(" waypoint=");
  Serial2.print(displayWaypointIndex());
  Serial2.print("/");
  Serial2.print(NUM_POINTS);
  Serial2.print(" x=");
  Serial2.print(robotX, 3);
  Serial2.print(" y=");
  Serial2.print(robotY, 3);
  Serial2.print(" theta=");
  Serial2.print(robotTheta, 2);
  Serial2.print(" front=");
  Serial2.print(frontDistance);
  Serial2.print(" left=");
  Serial2.print(leftDistance);
  Serial2.print(" right=");
  Serial2.print(rightDistance);
  Serial2.print(" tofValid=");
  Serial2.print(frontTofValid ? 1 : 0);
  Serial2.print("/");
  Serial2.print(leftTofValid ? 1 : 0);
  Serial2.print("/");
  Serial2.print(rightTofValid ? 1 : 0);
  Serial2.print(" encL=");
  Serial2.print(leftCount);
  Serial2.print(" encR=");
  Serial2.print(rightCount);
  Serial2.print(" blocked=");
  Serial2.print(frontBlocked ? 1 : 0);
  Serial2.print(" stuck=");
  Serial2.print((driveStuck || wheelMismatchStuck || turnStuck) ? 1 : 0);
  Serial2.print(" home=");
  Serial2.print(returnHomeRequested ? 1 : 0);
  Serial2.print(" motorL=");
  Serial2.print(lastLeftMotorUs);
  Serial2.print(" motorR=");
  Serial2.print(lastRightMotorUs);
  Serial2.print(" fBase=");
  Serial2.print(leftForwardBaseUs);
  Serial2.print("/");
  Serial2.print(rightForwardBaseUs);
  Serial2.print(" baseSpeed=");
  Serial2.print(baseTargetSpeed, 1);
  Serial2.print(" hGain=");
  Serial2.print(K_heading, 1);
  Serial2.print(" cmdForward=");
  Serial2.print(desiredForwardSpeed, 1);
  Serial2.print(" cmdTurn=");
  Serial2.println(desiredTurnSpeed, 1);
}

static void sendBluetoothCsvHeader() {
  Serial2.println("row_type,event,detail,ms,state,run,test_armed,waypoint,x_m,y_m,theta_deg,front_mm,left_mm,right_mm,front_valid,left_valid,right_valid,enc_l,enc_r,blocked,drive_stuck,wheel_mismatch,turn_stuck,home_requested,motor_l_us,motor_r_us,base_speed,cmd_forward,cmd_turn");
}

static void sendBluetoothCsvSnapshot(const char* rowType, const char* eventName, const char* eventDetail) {
  long leftCount;
  long rightCount;
  readEncoderCounts(leftCount, rightCount);

  Serial2.print(rowType);
  Serial2.print(",");
  Serial2.print(eventName);
  Serial2.print(",");
  Serial2.print(eventDetail);
  Serial2.print(",");
  Serial2.print(millis());
  Serial2.print(",");
  Serial2.print(robotStateName(currentState));
  Serial2.print(",");
  Serial2.print(robotRunEnabled ? 1 : 0);
  Serial2.print(",");
  Serial2.print(bluetoothTestArmed ? 1 : 0);
  Serial2.print(",");
  Serial2.print(displayWaypointIndex());
  Serial2.print(",");
  Serial2.print(robotX, 3);
  Serial2.print(",");
  Serial2.print(robotY, 3);
  Serial2.print(",");
  Serial2.print(robotTheta, 2);
  Serial2.print(",");
  Serial2.print(frontDistance);
  Serial2.print(",");
  Serial2.print(leftDistance);
  Serial2.print(",");
  Serial2.print(rightDistance);
  Serial2.print(",");
  Serial2.print(frontTofValid ? 1 : 0);
  Serial2.print(",");
  Serial2.print(leftTofValid ? 1 : 0);
  Serial2.print(",");
  Serial2.print(rightTofValid ? 1 : 0);
  Serial2.print(",");
  Serial2.print(leftCount);
  Serial2.print(",");
  Serial2.print(rightCount);
  Serial2.print(",");
  Serial2.print(frontBlocked ? 1 : 0);
  Serial2.print(",");
  Serial2.print(driveStuck ? 1 : 0);
  Serial2.print(",");
  Serial2.print(wheelMismatchStuck ? 1 : 0);
  Serial2.print(",");
  Serial2.print(turnStuck ? 1 : 0);
  Serial2.print(",");
  Serial2.print(returnHomeRequested ? 1 : 0);
  Serial2.print(",");
  Serial2.print(lastLeftMotorUs);
  Serial2.print(",");
  Serial2.print(lastRightMotorUs);
  Serial2.print(",");
  Serial2.print(baseTargetSpeed, 1);
  Serial2.print(",");
  Serial2.print(desiredForwardSpeed, 1);
  Serial2.print(",");
  Serial2.println(desiredTurnSpeed, 1);
}

static void sendBluetoothCsvRow() {
  sendBluetoothCsvSnapshot("telemetry", "", "");
}

void sendBluetoothEvent(const char* eventName, const char* eventDetail) {
  if (!bluetoothCsvStreamEnabled) {
    return;
  }

  sendBluetoothCsvSnapshot("event", eventName, eventDetail);
}

void sendBluetoothTelemetry() {
  if (!bluetoothStreamEnabled && !bluetoothCsvStreamEnabled) {
    return;
  }

  unsigned long now = millis();
  if (now - lastBluetoothTelemetryMs < 1000) {
    return;
  }

  lastBluetoothTelemetryMs = now;

  if (bluetoothStreamEnabled) {
    sendBluetoothStatus();
  }

  if (bluetoothCsvStreamEnabled) {
    sendBluetoothCsvRow();
  }
}

static void printFloatRangeError(const char* commandName, float minValue, float maxValue) {
  Serial2.print("ERROR ");
  Serial2.print(commandName);
  Serial2.print(" must be between ");
  Serial2.print(minValue, 2);
  Serial2.print(" and ");
  Serial2.println(maxValue, 2);
}

static int manualMotorPulseFromPercent(float percent, int forwardUs, int reverseUs) {
  percent = constrain(percent, MANUAL_COMMAND_MIN, MANUAL_COMMAND_MAX);

  if (percent >= 0.0) {
    return STOP_US + (int)((forwardUs - STOP_US) * (percent / 100.0));
  }

  return STOP_US - (int)((STOP_US - reverseUs) * (-percent / 100.0));
}

static void stopManualDrive() {
  bluetoothManualActive = false;
  setMotionCommand(0.0, 0.0);
  stopMotors();
}

static void runManualDriveCommand(float forwardPercent, float turnPercent) {
  if (!bluetoothManualArmed) {
    Serial2.println("ERROR manual drive is disarmed. Send MANUAL ARM first.");
    return;
  }

  if (forwardPercent < MANUAL_COMMAND_MIN || forwardPercent > MANUAL_COMMAND_MAX ||
      turnPercent < MANUAL_COMMAND_MIN || turnPercent > MANUAL_COMMAND_MAX) {
    printFloatRangeError("DRIVE forward/turn", MANUAL_COMMAND_MIN, MANUAL_COMMAND_MAX);
    return;
  }

  updateTOFSensors();

  if (forwardPercent > 0.0 && isRangeSensorBlocked(RANGE_FRONT)) {
    stopManualDrive();
    Serial2.println("MANUAL blocked by front sensor. Motors stopped.");
    sendBluetoothEvent("manual_drive_blocked", "front");
    return;
  }

  float leftPercent = constrain(forwardPercent + turnPercent, MANUAL_COMMAND_MIN, MANUAL_COMMAND_MAX);
  float rightPercent = constrain(forwardPercent - turnPercent, MANUAL_COMMAND_MIN, MANUAL_COMMAND_MAX);

  int leftUs = manualMotorPulseFromPercent(leftPercent, leftForwardBaseUs, LEFT_REVERSE_US);
  int rightUs = manualMotorPulseFromPercent(rightPercent, rightForwardBaseUs, RIGHT_REVERSE_US);

  setMotionCommand(forwardPercent, turnPercent);
  writeMotorUS(leftUs, rightUs);

  bluetoothManualActive = forwardPercent != 0.0 || turnPercent != 0.0;
  lastManualDriveCommandMs = millis();
}

bool isManualDriveActive() {
  return bluetoothManualActive;
}

void updateManualDriveTimeout() {
  if (!bluetoothManualActive) {
    return;
  }

  if (millis() - lastManualDriveCommandMs > MANUAL_DRIVE_TIMEOUT_MS) {
    stopManualDrive();
    Serial2.println("MANUAL timeout. Motors stopped.");
    sendBluetoothEvent("manual_drive_timeout", "watchdog");
  }
}

static bool requireBluetoothTestArm() {
  if (bluetoothTestArmed) {
    return true;
  }

  Serial2.println("ERROR test motion is disarmed. Send TEST ARM first.");
  return false;
}

static void beginBluetoothTestMotion() {
  bluetoothAbortMotionRequested = false;
  bluetoothManualArmed = false;
  bluetoothManualActive = false;
  robotRunEnabled = true;
  stoppedSafely = false;
  endMatchPrinted = false;
  setRobotState(FOLLOW_PATH);
  clearStuckFlags();
}

static void finishBluetoothTestMotion(bool aborted) {
  stopMotors();
  robotRunEnabled = false;
  stoppedSafely = true;
  endMatchPrinted = false;
  setRobotState(END_MATCH);
  bluetoothAbortMotionRequested = false;

  if (aborted) {
    Serial2.println("TEST aborted. Motors stopped.");
  } else {
    Serial2.println("TEST complete. Motors stopped.");
  }
}

static void runBluetoothTestDrive(float distanceMetres) {
  if (!requireBluetoothTestArm()) {
    return;
  }

  if (distanceMetres < TEST_DRIVE_MIN_METRES || distanceMetres > TEST_DRIVE_MAX_METRES) {
    printFloatRangeError("TEST DRIVE", TEST_DRIVE_MIN_METRES, TEST_DRIVE_MAX_METRES);
    return;
  }

  updateTOFSensors();

  if (isRangeSensorBlocked(RANGE_FRONT)) {
    Serial2.println("ERROR front sensor is blocked. TEST DRIVE refused.");
    return;
  }

  float headingDeg = readYawDeg();

  Serial2.print("OK test drive ");
  Serial2.print(distanceMetres, 3);
  Serial2.print(" m at heading ");
  Serial2.print(headingDeg, 2);
  Serial2.println(" deg.");

  beginBluetoothTestMotion();
  sendBluetoothEvent("test_drive_start", "manual");
  driveDistanceWithHeadingNoAvoid(distanceMetres, headingDeg);
  bool aborted = bluetoothAbortMotionRequested || currentState == END_MATCH;
  finishBluetoothTestMotion(aborted);
  sendBluetoothEvent(aborted ? "test_drive_abort" : "test_drive_end", "manual");
}

static void runBluetoothTestTurn(float angleDeg) {
  if (!requireBluetoothTestArm()) {
    return;
  }

  if (angleDeg < TEST_TURN_MIN_DEG || angleDeg > TEST_TURN_MAX_DEG) {
    printFloatRangeError("TEST TURN", TEST_TURN_MIN_DEG, TEST_TURN_MAX_DEG);
    return;
  }

  if (angleDeg > -TURN_TOLERANCE_DEG && angleDeg < TURN_TOLERANCE_DEG) {
    Serial2.println("ERROR TEST TURN angle is inside turn tolerance.");
    return;
  }

  Serial2.print("OK test turn ");
  Serial2.print(angleDeg, 2);
  Serial2.println(" deg.");

  beginBluetoothTestMotion();
  sendBluetoothEvent("test_turn_start", "manual");
  turnAngle(angleDeg);
  bool aborted = bluetoothAbortMotionRequested || currentState == END_MATCH;
  finishBluetoothTestMotion(aborted);
  sendBluetoothEvent(aborted ? "test_turn_abort" : "test_turn_end", "manual");
}

static void setBluetoothBaseSpeed(float speedTicksPerSecond) {
  if (speedTicksPerSecond < SPEED_MIN_TICKS_PER_SEC || speedTicksPerSecond > SPEED_MAX_TICKS_PER_SEC) {
    printFloatRangeError("SPEED", SPEED_MIN_TICKS_PER_SEC, SPEED_MAX_TICKS_PER_SEC);
    return;
  }

  baseTargetSpeed = speedTicksPerSecond;

  Serial2.print("OK base target speed set to ");
  Serial2.print(baseTargetSpeed, 1);
  Serial2.println(" ticks/s.");
  sendBluetoothEvent("speed_set", "manual");
}

static void setBluetoothHeadingGain(float headingGain) {
  if (headingGain < HGAIN_MIN || headingGain > HGAIN_MAX) {
    printFloatRangeError("HGAIN", HGAIN_MIN, HGAIN_MAX);
    return;
  }

  K_heading = headingGain;

  Serial2.print("OK heading gain set to ");
  Serial2.print(K_heading, 1);
  Serial2.println(".");
  sendBluetoothEvent("hgain_set", "manual");
}

static void setBluetoothForwardBase(float leftBaseUs, float rightBaseUs) {
  if (leftBaseUs < FBASE_MIN_US || leftBaseUs > FBASE_MAX_US ||
      rightBaseUs < FBASE_MIN_US || rightBaseUs > FBASE_MAX_US) {
    printFloatRangeError("FBASE left/right", FBASE_MIN_US, FBASE_MAX_US);
    return;
  }

  leftForwardBaseUs = (int)(leftBaseUs + 0.5);
  rightForwardBaseUs = (int)(rightBaseUs + 0.5);

  Serial2.print("OK forward base pulses set to ");
  Serial2.print(leftForwardBaseUs);
  Serial2.print("/");
  Serial2.print(rightForwardBaseUs);
  Serial2.println(" us.");
  sendBluetoothEvent("fbase_set", "manual");
}

static void zeroRobotPoseFromBluetooth() {
  stopMotors();
  zeroYaw();

  robotX = 0.0;
  robotY = 0.0;
  robotTheta = 0.0;

  resetEncodersAndPID();
  currentWaypointIndex = 0;
  stuckRecoveryCount = 0;
  returnHomeRequested = false;
  clearStuckFlags();
  robotRunEnabled = false;
  bluetoothTestArmed = false;
  bluetoothManualArmed = false;
  bluetoothManualActive = false;
  bluetoothAbortMotionRequested = true;
  stoppedSafely = true;
  endMatchPrinted = false;
  setRobotState(END_MATCH);
  Serial2.println("OK stopped and zeroed yaw, pose, encoders, and PID state.");
}

static void handleBluetoothCommandLine(char* command) {
  trimCommand(command);

  if (command[0] == '\0') {
    return;
  }

  if (commandEquals(command, "HELP") || commandEquals(command, "H")) {
    printBluetoothHelp();
    return;
  }

  if (commandEquals(command, "START")) {
    robotRunEnabled = true;
    bluetoothTestArmed = false;
    bluetoothManualArmed = false;
    bluetoothManualActive = false;
    bluetoothAbortMotionRequested = false;
    stoppedSafely = false;
    endMatchPrinted = false;
    returnHomeRequested = false;

    if (currentState == END_MATCH) {
      currentState = INIT;
      previousState = INIT;
    }

    Serial2.println("OK start requested.");
    return;
  }

  if (commandEquals(command, "STATUS") || commandEquals(command, "P")) {
    sendBluetoothStatus();
    return;
  }

  if (commandEquals(command, "STREAM ON")) {
    bluetoothStreamEnabled = true;
    lastBluetoothTelemetryMs = 0;
    Serial2.println("OK stream on.");
    return;
  }

  if (commandEquals(command, "STREAM OFF")) {
    bluetoothStreamEnabled = false;
    Serial2.println("OK stream off.");
    return;
  }

  if (commandEquals(command, "CSV ON")) {
    bluetoothCsvStreamEnabled = true;
    lastBluetoothTelemetryMs = 0;
    Serial2.println("OK csv on.");
    sendBluetoothCsvHeader();
    return;
  }

  if (commandEquals(command, "CSV OFF")) {
    bluetoothCsvStreamEnabled = false;
    Serial2.println("OK csv off.");
    return;
  }

  if (commandEquals(command, "CAL") || commandEquals(command, "CALIBRATION")) {
    printCalibrationSummary();
    return;
  }

  if (commandEquals(command, "SPEED RESET")) {
    baseTargetSpeed = DEFAULT_BASE_TARGET_SPEED;
    Serial2.print("OK base target speed reset to ");
    Serial2.print(baseTargetSpeed, 1);
    Serial2.println(" ticks/s.");
    sendBluetoothEvent("speed_reset", "manual");
    return;
  }

  if (commandEquals(command, "HGAIN RESET")) {
    K_heading = DEFAULT_HEADING_GAIN;
    Serial2.print("OK heading gain reset to ");
    Serial2.print(K_heading, 1);
    Serial2.println(".");
    sendBluetoothEvent("hgain_reset", "manual");
    return;
  }

  if (commandHasPrefix(command, "HGAIN")) {
    float requestedHeadingGain = 0.0;

    if (!parseFloatArgument(commandArgument(command, "HGAIN"), requestedHeadingGain)) {
      Serial2.println("ERROR usage: HGAIN <gain> or HGAIN RESET");
      return;
    }

    setBluetoothHeadingGain(requestedHeadingGain);
    return;
  }

  if (commandEquals(command, "FBASE RESET")) {
    leftForwardBaseUs = LEFT_BASE_US;
    rightForwardBaseUs = RIGHT_BASE_US;
    Serial2.print("OK forward base pulses reset to ");
    Serial2.print(leftForwardBaseUs);
    Serial2.print("/");
    Serial2.print(rightForwardBaseUs);
    Serial2.println(" us.");
    sendBluetoothEvent("fbase_reset", "manual");
    return;
  }

  if (commandHasPrefix(command, "FBASE")) {
    float requestedLeftBase = 0.0;
    float requestedRightBase = 0.0;

    if (!parseTwoFloatArguments(commandArgument(command, "FBASE"), requestedLeftBase, requestedRightBase)) {
      Serial2.println("ERROR usage: FBASE <left_us> <right_us> or FBASE RESET");
      return;
    }

    setBluetoothForwardBase(requestedLeftBase, requestedRightBase);
    return;
  }

  if (commandHasPrefix(command, "SPEED")) {
    float requestedSpeed = 0.0;

    if (!parseFloatArgument(commandArgument(command, "SPEED"), requestedSpeed)) {
      Serial2.println("ERROR usage: SPEED <ticks/s> or SPEED RESET");
      return;
    }

    setBluetoothBaseSpeed(requestedSpeed);
    return;
  }

  if (commandEquals(command, "TEST ARM")) {
    bluetoothTestArmed = true;
    bluetoothManualArmed = false;
    bluetoothManualActive = false;
    robotRunEnabled = false;
    bluetoothAbortMotionRequested = false;
    stopMotors();
    Serial2.println("OK test motion armed. TEST DRIVE and TEST TURN can move the robot.");
    sendBluetoothEvent("test_arm", "manual");
    return;
  }

  if (commandEquals(command, "TEST DISARM")) {
    bluetoothTestArmed = false;
    stopMotors();
    Serial2.println("OK test motion disarmed.");
    sendBluetoothEvent("test_disarm", "manual");
    return;
  }

  if (commandEquals(command, "MANUAL ARM")) {
    bluetoothManualArmed = true;
    bluetoothManualActive = false;
    bluetoothTestArmed = false;
    robotRunEnabled = false;
    bluetoothAbortMotionRequested = false;
    stopMotors();
    Serial2.println("OK manual drive armed. Use DRIVE <forward> <turn>.");
    sendBluetoothEvent("manual_arm", "manual");
    return;
  }

  if (commandEquals(command, "MANUAL DISARM")) {
    bluetoothManualArmed = false;
    stopManualDrive();
    Serial2.println("OK manual drive disarmed.");
    sendBluetoothEvent("manual_disarm", "manual");
    return;
  }

  if (commandHasPrefix(command, "DRIVE")) {
    float forwardPercent = 0.0;
    float turnPercent = 0.0;

    if (!parseTwoFloatArguments(commandArgument(command, "DRIVE"), forwardPercent, turnPercent)) {
      Serial2.println("ERROR usage: DRIVE <forward -100..100> <turn -100..100>");
      return;
    }

    runManualDriveCommand(forwardPercent, turnPercent);
    return;
  }

  if (commandHasPrefix(command, "TEST DRIVE")) {
    float distanceMetres = 0.0;

    if (!parseFloatArgument(commandArgument(command, "TEST DRIVE"), distanceMetres)) {
      Serial2.println("ERROR usage: TEST DRIVE <metres>");
      return;
    }

    runBluetoothTestDrive(distanceMetres);
    return;
  }

  if (commandHasPrefix(command, "TEST TURN")) {
    float angleDeg = 0.0;

    if (!parseFloatArgument(commandArgument(command, "TEST TURN"), angleDeg)) {
      Serial2.println("ERROR usage: TEST TURN <degrees>");
      return;
    }

    runBluetoothTestTurn(angleDeg);
    return;
  }

  if (commandEquals(command, "HOME")) {
    bluetoothTestArmed = false;
    bluetoothManualArmed = false;
    bluetoothManualActive = false;
    returnHomeRequested = true;
    Serial2.println("OK return home requested.");
    return;
  }

  if (commandEquals(command, "ZERO")) {
    zeroRobotPoseFromBluetooth();
    return;
  }

  if (commandEquals(command, "STOP") || commandEquals(command, "S")) {
    robotRunEnabled = false;
    bluetoothTestArmed = false;
    bluetoothManualArmed = false;
    bluetoothManualActive = false;
    bluetoothAbortMotionRequested = true;
    stoppedSafely = true;
    endMatchPrinted = false;
    stopMotors();
    setRobotState(END_MATCH);
    Serial2.println("OK stopped motors and set END_MATCH.");
    return;
  }

  Serial2.print("ERROR unknown command: ");
  Serial2.println(command);
  Serial2.println("Type HELP for commands.");
}

bool handleBluetoothCommands() {
  while (Serial2.available() > 0) {
    char incoming = Serial2.read();

    if (incoming == '\r' || incoming == '\n') {
      bluetoothCommand[bluetoothCommandLength] = '\0';
      handleBluetoothCommandLine(bluetoothCommand);
      bluetoothCommandLength = 0;
      continue;
    }

    if (bluetoothCommandLength < (int)sizeof(bluetoothCommand) - 1) {
      bluetoothCommand[bluetoothCommandLength] = incoming;
      bluetoothCommandLength++;
    } else {
      bluetoothCommandLength = 0;
      Serial2.println("ERROR command too long.");
    }
  }

  sendBluetoothTelemetry();
  return bluetoothAbortMotionRequested;
}
