#include "../../Robot.h"
#include "../../Navigation.h"
#include "../../NavigationAdmin.h"
#include "../../NavigationTest.h"
#include "../navigation/PlannerDebug.h"
#include "../mission/RouteMission.h"
#include "../mission/WeightSearch.h"
#include <ctype.h>
#include <stdlib.h>
#include <string.h>

// =====================================================
// Bluetooth serial debug link on Serial2
// =====================================================
// Responsibility:
//   Implements the CH9143 Serial2 operator console, command parser, runtime
//   test harnesses, manual drive interface, telemetry queue, CSV/status
//   output, and command-time tuning controls.
// Interacts with:
//   MissionController.cpp is started/stopped by START/STOP/HOME and test commands.
//   Navigation.h receives navigation/test goals. MotorControl.cpp provides
//   motion authority, safety, PID/output diagnostics, and manual/test command
//   acceptance. Fan, rear-array, and front-matrix modules provide diagnostics.
// Control flow:
//   RobotCode.ino calls handleBluetoothCommands() every loop and
//   serviceBluetoothTelemetryTx() after the controller update. Commands are
//   line-oriented text terminated by CR/LF.
// Global state:
//   Owns Bluetooth/test/manual mode flags, command buffer, telemetry queue,
//   test timers, and runtime-tunable copies of speed/base/scan-turn settings.
static char bluetoothCommand[64];
static int bluetoothCommandLength = 0;
static bool bluetoothAbortMotionRequested = false;
static bool bluetoothStreamEnabled = false;
static bool bluetoothCsvStreamEnabled = false;
static bool bluetoothCsvClosing = false;
static bool bluetoothTestArmed = false;
static bool bluetoothManualArmed = false;
static bool bluetoothManualActive = false;
static bool bluetoothSideTestActive = false;
static bool bluetoothTurnPulseTestActive = false;
static bool bluetoothArcTestActive = false;
static bool bluetoothTurnTruthActive = false;
static bool bluetoothNavigationTestActive = false;
static bool turnPulseCoasting = false;
static unsigned long lastManualDriveCommandMs = 0;
static unsigned long lastBluetoothMotionTelemetryMs = 0;
static unsigned long lastBluetoothFullTelemetryMs = 0;
static unsigned long sideTestEndMs = 0;
static unsigned long sideTestNextSampleMs = 0;
static int sideTestSampleNumber = 0;
static unsigned long turnPulseEndMs = 0;
static unsigned long turnPulseCoastEndMs = 0;
static unsigned long turnPulseNextSampleMs = 0;
static unsigned long turnPulseStartMs = 0;
static float turnPulseStartHeadingDeg = 0.0;
static float turnPulseCommandTicksPerSec = 0.0;
static unsigned long arcTestEndMs = 0;
static unsigned long arcTestStartMs = 0;
static unsigned long arcTestNextSampleMs = 0;
static float arcTestForwardTicksPerSec = 0.0;
static float arcTestTurnTicksPerSec = 0.0;
static unsigned long turnTruthStartMs = 0;
static unsigned long turnTruthNextSampleMs = 0;

// CSV/STATUS telemetry must never write synchronously to Serial2. A complete
// row is first assembled in telemetryStage, then atomically admitted to the
// bounded byte ring. serviceBluetoothTelemetryTx() drains only bytes that the
// UART reports writable and only up to a fixed per-loop budget.
static char telemetryQueue[TELEMETRY_QUEUE_CAPACITY_BYTES];
static size_t telemetryQueueHead = 0;
static size_t telemetryQueueTail = 0;
static size_t telemetryQueueBytes = 0;
static unsigned long telemetryQueuedRows = 0;
static unsigned long telemetryDroppedRows = 0;
static unsigned long telemetryRateLimitedEvents = 0;
static unsigned long telemetrySequence = 0;
static char lastTelemetryEventName[40] = "";
static char lastTelemetryEventDetail[64] = "";
static unsigned long lastTelemetryEventMs = 0;
static bool telemetryRowAssemblyActive = false;

class TelemetryStagePrint : public Print {
public:
  void reset() {
    length = 0;
    overflowed = false;
  }

  size_t write(uint8_t value) override {
    if (length >= TELEMETRY_STAGE_CAPACITY_BYTES) {
      overflowed = true;
      return 0;
    }
    buffer[length++] = (char)value;
    return 1;
  }

  bool commit(size_t maximumLength = TELEMETRY_STAGE_CAPACITY_BYTES,
              size_t reservedQueueBytes = 0) {
    if (overflowed || length == 0 || length > maximumLength ||
        length + reservedQueueBytes >
          TELEMETRY_QUEUE_CAPACITY_BYTES - telemetryQueueBytes) {
      telemetryDroppedRows++;
      reset();
      return false;
    }
    for (size_t i = 0; i < length; i++) {
      telemetryQueue[telemetryQueueHead] = buffer[i];
      telemetryQueueHead = (telemetryQueueHead + 1) % TELEMETRY_QUEUE_CAPACITY_BYTES;
    }
    telemetryQueueBytes += length;
    telemetryQueuedRows++;
    reset();
    return true;
  }

private:
  char buffer[TELEMETRY_STAGE_CAPACITY_BYTES];
  size_t length = 0;
  bool overflowed = false;
};

static TelemetryStagePrint telemetryStage;
static TelemetryStagePrint controlLineStage;

static void beginTelemetryRow() {
  // Starts building a single STATUS/CSV row in RAM. The row is committed to
  // the bounded queue only after it is complete, preventing interleaved output.
  telemetryStage.reset();
  telemetryRowAssemblyActive = true;
}

static void commitTelemetryRow(
    size_t maximumLength = TELEMETRY_STAGE_CAPACITY_BYTES) {
  telemetryStage.commit(maximumLength, TELEMETRY_EVENT_QUEUE_RESERVE_BYTES);
  telemetryRowAssemblyActive = false;
}

static void commitTelemetryEventRow() {
  telemetryStage.commit(TELEMETRY_EVENT_ROW_BUDGET_BYTES);
  telemetryRowAssemblyActive = false;
}

void serviceBluetoothTelemetryTx() {
  // Drains a small number of queued bytes per main-loop pass. This protects
  // the control loop from blocking on a slow UART or a full Bluetooth buffer.
  size_t writable = Serial2.availableForWrite();
  if (writable > TELEMETRY_TX_BUDGET_BYTES_PER_LOOP) {
    writable = TELEMETRY_TX_BUDGET_BYTES_PER_LOOP;
  }
  if (writable > telemetryQueueBytes) {
    writable = telemetryQueueBytes;
  }

  for (size_t sent = 0; sent < writable; sent++) {
    const char value = telemetryQueue[telemetryQueueTail];
    Serial2.write((uint8_t)value);
    telemetryQueueTail = (telemetryQueueTail + 1) % TELEMETRY_QUEUE_CAPACITY_BYTES;
    telemetryQueueBytes--;
    if (value == '\n' && telemetryQueuedRows > 0) {
      telemetryQueuedRows--;
    }
  }

  if (bluetoothCsvClosing && telemetryQueueBytes == 0) {
    bluetoothCsvClosing = false;
  }
}

bool isBluetoothCsvStreamEnabled() {
  // RobotSerialClass uses this to decide whether free-form Serial prints may
  // be mirrored directly or must avoid corrupting CSV output.
  return bluetoothCsvStreamEnabled || bluetoothCsvClosing;
}

// During CSV capture every Bluetooth output line uses the same bounded queue.
// This prevents acknowledgements and turn/arc truth diagnostics from cutting
// through telemetry rows or blocking behind a full UART.
class BluetoothSerialMux {
public:
  void begin(unsigned long baud) { ::Serial2.begin(baud); }
  int available() { return ::Serial2.available(); }
  int read() { return ::Serial2.read(); }

  template <typename... Args>
  size_t print(Args... args) {
    if (telemetryRowAssemblyActive) {
      return telemetryStage.print(args...);
    }
    if (isBluetoothCsvStreamEnabled()) {
      return controlLineStage.print(args...);
    }
    return ::Serial2.print(args...);
  }

  template <typename... Args>
  size_t println(Args... args) {
    if (telemetryRowAssemblyActive) {
      return telemetryStage.println(args...);
    }
    if (isBluetoothCsvStreamEnabled()) {
      const size_t written = controlLineStage.println(args...);
      controlLineStage.commit();
      return written;
    }
    return ::Serial2.println(args...);
  }

  size_t println() {
    if (telemetryRowAssemblyActive) {
      return telemetryStage.println();
    }
    if (isBluetoothCsvStreamEnabled()) {
      const size_t written = controlLineStage.println();
      controlLineStage.commit();
      return written;
    }
    return ::Serial2.println();
  }
};

static BluetoothSerialMux bluetoothSerialMux;
#define Serial2 bluetoothSerialMux

const float TEST_DRIVE_MIN_METRES = 0.01;
const float TEST_DRIVE_MAX_METRES = 1.50;
const float TEST_GOTO_MIN_COORD_M = -10.00;
const float TEST_GOTO_MAX_COORD_M = 10.00;
const float TEST_AVOID_MIN_METRES = 0.10;
const float TEST_AVOID_MAX_METRES = 2.00;
const float TEST_TURN_MIN_DEG = -360.0;
const float TEST_TURN_MAX_DEG = 360.0;
const float TEST_TURN_PULSE_MIN_SECONDS = 0.10;
const float TEST_TURN_PULSE_MAX_SECONDS = 0.80;
const float TEST_ARC_MIN_FORWARD_TPS = 300.0;
const float TEST_ARC_MAX_FORWARD_TPS = 1800.0;
const float TEST_ARC_MAX_ABS_TURN_TPS = 500.0;
const float TEST_ARC_MIN_SECONDS = 0.10;
const float TEST_ARC_MAX_SECONDS = 0.80;
const unsigned long TEST_TRUTH_FIRST_SAMPLE_MS = 120;
const unsigned long TEST_TRUTH_SAMPLE_INTERVAL_MS = 1000;
const unsigned long TEST_TURN_PULSE_SAMPLE_INTERVAL_MS = 100;
const unsigned long TEST_TURN_PULSE_COAST_MS = 1000;
const float TEST_SIDE_MIN_SECONDS = 1.0;
const float TEST_SIDE_MAX_SECONDS = 30.0;
const unsigned long TEST_SIDE_SAMPLE_INTERVAL_MS = 1000;
const float SPEED_MIN_TICKS_PER_SEC = 300.0;
const float SPEED_MAX_TICKS_PER_SEC = 3000.0;
const float FBASE_MIN_US = STOP_US;
const float FBASE_MAX_US = MAX_US;
const float SEARCHTURN_MIN_US = WEIGHT_SCAN_TURN_OFFSET_MIN_US;
const float SEARCHTURN_MAX_US = WEIGHT_SCAN_TURN_OFFSET_MAX_US;
const float MANUAL_COMMAND_MIN = -100.0;
const float MANUAL_COMMAND_MAX = 100.0;
const unsigned long MANUAL_DRIVE_TIMEOUT_MS = 350;

static bool commandEquals(const char* command, const char* expected) {
  // Case-insensitive exact command match. Commands are trimmed before routing.
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
  // Case-insensitive prefix match where the next character must be whitespace
  // or the end of the command. This prevents TEST TURN matching TEST TURNXYZ.
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
  // Parses one finite float and rejects trailing non-space text. This keeps
  // command inputs deterministic and avoids partial parses like "1.0abc".
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

static bool parseThreeFloatArguments(const char* text, float &first,
                                     float &second, float &third) {
  char* endPtr = NULL;
  double values[3];
  for (int i = 0; i < 3; i++) {
    values[i] = strtod(text, &endPtr);
    if (endPtr == text || values[i] != values[i]) {
      return false;
    }
    text = endPtr;
    while (*text != '\0' && isspace(*text)) {
      text++;
    }
  }
  if (*text != '\0') {
    return false;
  }
  first = (float)values[0];
  second = (float)values[1];
  third = (float)values[2];
  return true;
}

static void trimCommand(char* command) {
  // Mutates the command buffer in place, removing leading/trailing whitespace
  // before the handler chain sees it.
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
  Serial2.println("  BUILD          print firmware build label");
  Serial2.println("  START          start robot navigation");
  Serial2.println("  COMPETITION ON/OFF/STATUS  retry route failures until STOP");
  Serial2.println("  STATUS or P    print robot status");
  Serial2.println("  LOOP RESET     reset loop-gap diagnostics while motors are neutral");
  Serial2.println("  STREAM ON      send status as fast as the telemetry loop allows");
  Serial2.println("  STREAM OFF     stop periodic status");
  Serial2.println("  CSV ON         send CSV telemetry as fast as the telemetry loop allows");
  Serial2.println("  CSV OFF        stop CSV telemetry");
  Serial2.println("  CAL            print calibration summary");
  Serial2.println("  SPEED <ticks>  set temporary base target speed");
  Serial2.println("  SPEED RESET    restore default base target speed");
  Serial2.println("  SEARCHTURN <us>|RESET  set TEST SEARCH scan turn offset");
  Serial2.println("  FBASE <l> <r>  set temporary forward motor base pulses");
  Serial2.println("  FBASE RESET    restore default forward motor base pulses");
  Serial2.println("  ESCAPE ON/OFF/STATUS  reverse policy (trusted rear coverage required)");
  Serial2.println("  EMERGENCY ON/OFF/STATUS  ultimate scan/retry recovery policy");
  Serial2.println("  TEST ARM       allow one or more test motion commands");
  Serial2.println("  TEST DISARM    disable test motion commands");
  Serial2.println("  TEST DRIVE <m> drive a fixed distance at current heading");
  Serial2.println("  TEST GOTO <x> <y>  go to one temporary absolute waypoint");
  Serial2.println("  TEST AVOID <m> run one straight-ahead avoidance scenario");
  Serial2.println("  TEST ESCAPE <m> front-blocked reverse-recovery test");
  Serial2.println("  TEST FAN       print high fan ToF sector readings");
  Serial2.println("  TEST REAR      print three-rear-VL53L1X safety telemetry");
  Serial2.println("  TEST MATRIX    print front matrix and weight evidence");
  Serial2.println("  TEST PAYLOAD   print internal-funnel payload ToF evidence");
  Serial2.println("  TEST HUNT TARGET  print latched matrix target without moving");
  Serial2.println("  TEST HUNT      track a confirmed matrix target after TEST ARM");
  Serial2.println("  TEST FOLLOW START|STOP|STATUS  bounded matrix follow diagnostic");
  Serial2.println("  TEST SEARCH    run waypoint-style weight scan/search after TEST ARM");
  Serial2.println("  TEST SIDE <s>  sample avoidance side choice without moving");
  Serial2.println("  TEST TURN <d>  signed angle; positive is CCW/left");
  Serial2.println("  TEST ARC <forward_tps> <turn_tps> <s>  bounded wheel-PID sign diagnostic");
  Serial2.println("  TEST TURNPULSE <signed_s>  positive is CCW/left; log 1 s coast");
  Serial2.println("  TEST TURNLADDER LEFT|RIGHT  disabled until nonblocking P0-06 conversion");
  Serial2.println("  MARK <note>    print and log a test note");
  Serial2.println("  MANUAL ARM     allow live DRIVE commands");
  Serial2.println("  MANUAL DISARM  stop and disable live DRIVE commands");
  Serial2.println("  DRIVE <f> <t>  live drive; positive turn is CCW/left");
  Serial2.println("  HOME           request return home");
  Serial2.println("  ZERO           zero yaw and pose");
  Serial2.println("  STOP or S      stop motors and end match");
}

void setupBluetooth() {
  // Starts Serial2 and prints the command help. Called before setup() enables
  // sensors so boot/progress messages can be mirrored to the operator link.
  Serial2.begin(BLUETOOTH_BAUD);
  bluetoothOutputEnabled = true;
  delay(100);

  Serial2.println();
  Serial2.println("CH9143 Bluetooth link ready on Serial2.");
  printBluetoothHelp();
}

static int displayWaypointIndex() {
  return routeMissionDisplayIndex();
}

void sendBluetoothStatus() {
  // Human-readable snapshot used by STATUS/P and stream mode. It includes
  // pose, sensing, authority, motor, planner, object, timing, and telemetry
  // diagnostics in one line for saved regression oracles.
  beginTelemetryRow();
  const NavigationDebugStatus navigation = getNavigationDebugStatus();
  const PlannerTelemetry& plannerTelemetry = getPlannerTelemetry();
  long leftCount;
  long rightCount;
  readEncoderCounts(leftCount, rightCount);

  Serial2.print("STATUS state=");
  Serial2.print(robotStateName(currentState));
  Serial2.print(" build=");
  Serial2.print(ROBOT_BUILD_LABEL);
  Serial2.print(" run=");
  Serial2.print(robotRunEnabled ? 1 : 0);
  Serial2.print(" competition=");
  Serial2.print(isCompetitionModeEnabled() ? 1 : 0);
  Serial2.print(" testArmed=");
  Serial2.print(bluetoothTestArmed ? 1 : 0);
  Serial2.print(" manualArmed=");
  Serial2.print(bluetoothManualArmed ? 1 : 0);
  Serial2.print(" manualActive=");
  Serial2.print(bluetoothManualActive ? 1 : 0);
  Serial2.print(" authority=");
  Serial2.print(motionAuthorityName(motionAuthority));
  Serial2.print(" goalAuthority=");
  Serial2.print(motionAuthorityName(navigation.authority));
  Serial2.print(" safetyStop=");
  Serial2.print(isMotionSafetyStopActive() ? 1 : 0);
  Serial2.print(" safetyReason=");
  Serial2.print(motionSafetyReasonName(lastMotionSafetyReason()));
  Serial2.print(" watchdogReady=");
  Serial2.print(isMotorSafetyWatchdogReady() ? 1 : 0);
  Serial2.print(" commandLease=");
  Serial2.print(isMotorCommandLeaseArmed() ? 1 : 0);
  Serial2.print(" leaseTrips=");
  Serial2.print(motorCommandLeaseTripCount());
  Serial2.print(" loopGapMs=");
  Serial2.print(currentMainLoopGapMs());
  Serial2.print(" loopMaxMs=");
  Serial2.print(maximumMainLoopGapMs());
  Serial2.print(" loopMisses=");
  Serial2.print(mainLoopDeadlineMissCount());
  Serial2.print(" loopWorstPhase=");
  Serial2.print(maximumMainLoopPhaseName());
  Serial2.print(" loopWorstPhaseUs=");
  Serial2.print(maximumMainLoopPhaseUs());
  Serial2.print(" telemetryQueuedRows=");
  Serial2.print(telemetryQueuedRows);
  Serial2.print(" telemetryQueuedBytes=");
  Serial2.print(telemetryQueueBytes);
  Serial2.print(" telemetryDroppedRows=");
  Serial2.print(telemetryDroppedRows);
  Serial2.print(" telemetryRateLimitedEvents=");
  Serial2.print(telemetryRateLimitedEvents);
  Serial2.print(" waypoint=");
  Serial2.print(displayWaypointIndex());
  Serial2.print("/");
  Serial2.print(routeMissionPointCount());
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
  Serial2.print(" fanMm=");
  Serial2.print(getRangeSensorDistance(RANGE_RIGHT_OUTER));
  Serial2.print("/");
  Serial2.print(getRangeSensorDistance(RANGE_RIGHT_INNER));
  Serial2.print("/");
  Serial2.print(getRangeSensorDistance(RANGE_LEFT_INNER));
  Serial2.print("/");
  Serial2.print(getRangeSensorDistance(RANGE_LEFT_OUTER));
  Serial2.print(" frontVirtual=");
  Serial2.print(getRangeSensorDistance(RANGE_FRONT));
  Serial2.print(" fanValid=");
  Serial2.print(isRangeSensorValid(RANGE_RIGHT_OUTER) ? 1 : 0);
  Serial2.print("/");
  Serial2.print(isRangeSensorValid(RANGE_RIGHT_INNER) ? 1 : 0);
  Serial2.print("/");
  Serial2.print(isRangeSensorValid(RANGE_LEFT_INNER) ? 1 : 0);
  Serial2.print("/");
  Serial2.print(isRangeSensorValid(RANGE_LEFT_OUTER) ? 1 : 0);
  Serial2.print(" frontVirtualValid=");
  Serial2.print(isRangeSensorValid(RANGE_FRONT) ? 1 : 0);
  Serial2.print(" rearAggregate=");
  Serial2.print(getRangeSensorDistance(RANGE_REAR_AGGREGATE));
  Serial2.print("/");
  Serial2.print(isRangeSensorValid(RANGE_REAR_AGGREGATE) ? 1 : 0);
  Serial2.print("/");
  Serial2.print(isRangeSensorBlocked(RANGE_REAR_AGGREGATE) ? 1 : 0);
  Serial2.print(" matrixTarget=");
  Serial2.print(matrixEvidenceKindName(matrixTargetObservation.evidence));
  Serial2.print("/");
  Serial2.print(matrixTargetObservation.confirmedStatic ? 1 : 0);
  Serial2.print("/");
  Serial2.print(matrixTargetObservation.trackId);
  Serial2.print("/");
  Serial2.print(matrixTargetObservation.directGapMm, 1);
  Serial2.print(" fanAgeMs=");
  unsigned long statusNow = millis();
  Serial2.print(statusNow - rangeSensors[RANGE_RIGHT_OUTER].lastReadMs);
  Serial2.print("/");
  Serial2.print(statusNow - rangeSensors[RANGE_RIGHT_INNER].lastReadMs);
  Serial2.print("/");
  Serial2.print(statusNow - rangeSensors[RANGE_LEFT_INNER].lastReadMs);
  Serial2.print("/");
  Serial2.print(statusNow - rangeSensors[RANGE_LEFT_OUTER].lastReadMs);
  Serial2.print(" encL=");
  Serial2.print(leftCount);
  Serial2.print(" encR=");
  Serial2.print(rightCount);
  Serial2.print(" blocked=");
  Serial2.print(frontBlocked ? 1 : 0);
  Serial2.print(" escape=");
  Serial2.print(escapeBacktrackEnabled ? 1 : 0);
  Serial2.print(" emergency=");
  Serial2.print(navigationIsEmergencyScanEnabled() ? 1 : 0);
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
  Serial2.print(" searchTurnUs=");
  Serial2.print(weightScanTurnOffsetUs);
  Serial2.print(" cmdForward=");
  Serial2.print(desiredForwardSpeed, 1);
  Serial2.print(" cmdTurn=");
  Serial2.print(desiredTurnSpeed, 1);
  Serial2.print(" wheelTargetL=");
  Serial2.print(lastRequestedLeftWheelSpeed, 1);
  Serial2.print(" wheelTargetR=");
  Serial2.print(lastRequestedRightWheelSpeed, 1);
  Serial2.print(" wheelRateL=");
  Serial2.print(lastMeasuredLeftWheelSpeed, 1);
  Serial2.print(" wheelRateR=");
  Serial2.print(lastMeasuredRightWheelSpeed, 1);
  Serial2.print(" imuRawCw=");
  Serial2.print(lastImuClockwiseYawDeg, 2);
  Serial2.print(" navYaw=");
  Serial2.print(lastNavigationHeadingDeg, 2);
  Serial2.print(" motorMode=");
  Serial2.print(lastMotorOutputMode);
  Serial2.print(" plannerCandidates=");
  Serial2.print(plannerTelemetry.candidateCount);
  Serial2.print(" plannerV=");
  Serial2.print(plannerTelemetry.selectedForwardTicksPerSec, 1);
  Serial2.print(" plannerW=");
  Serial2.print(plannerTelemetry.selectedTurnTicksPerSec, 1);
  Serial2.print(" plannerClearance=");
  Serial2.print(plannerTelemetry.minimumSweptClearanceMm, 1);
  Serial2.print(" plannerStop=");
  Serial2.print(plannerStopReasonName(plannerTelemetry.stopReason));
  Serial2.print(" plannerReason=");
  Serial2.print(plannerTelemetry.planReason);
  Serial2.print(" plannerTiming=");
  Serial2.print(plannerTelemetry.plannerSliceUs);
  Serial2.print("/");
  Serial2.print(plannerTelemetry.plannerSliceMaxUs);
  Serial2.print("/");
  Serial2.print(plannerTelemetry.plannerEpochWorkUs);
  Serial2.print("/");
  Serial2.print(plannerTelemetry.plannerEpochMaxWorkUs);
  Serial2.print("/");
  Serial2.print(plannerTelemetry.plannerEpochAgeMs);
  Serial2.print("/");
  Serial2.print(plannerTelemetry.plannerCommandAgeMs);
  Serial2.print("/");
  Serial2.print(plannerTelemetry.plannerCandidatesProcessed);
  Serial2.print("/");
  Serial2.print(plannerTelemetry.plannerYieldCount);
  Serial2.print("/");
  Serial2.print(plannerTelemetry.plannerEpochActive ? 1 : 0);
  Serial2.print(" recovery=");
  Serial2.print(plannerTelemetry.globalGoalDistanceM, 2);
  Serial2.print("/");
  Serial2.print(plannerTelemetry.localGoalDistanceM, 2);
  Serial2.print("/");
  Serial2.print(plannerTelemetry.routeAlongProgressM, 2);
  Serial2.print("/");
  Serial2.print(plannerTelemetry.routeSignedLateralErrorM, 2);
  Serial2.print("/");
  Serial2.print(plannerTelemetry.obstacleProgressAgeS, 1);
  Serial2.print("/");
  Serial2.print(plannerTelemetry.cumulativeReverseDistanceM, 2);
  Serial2.print("/");
  Serial2.print(plannerTelemetry.recoveryCount);
  Serial2.print("/");
  Serial2.print(plannerTelemetry.obstacleBestProgressM, 2);
  Serial2.print(" matrixTarget=");
  Serial2.print(matrixTargetObservation.valid ? 1 : 0);
  Serial2.print("/");
  Serial2.print(matrixTargetObservation.confirmedStatic ? 1 : 0);
  Serial2.print("/");
  Serial2.print(matrixTargetObservation.worldX, 3);
  Serial2.print("/");
  Serial2.print(matrixTargetObservation.worldY, 3);
  Serial2.print("/");
  Serial2.print(matrixTargetObservation.robotXmm, 1);
  Serial2.print("/");
  Serial2.print(matrixTargetObservation.robotYmm, 1);
  Serial2.print("/");
  Serial2.println(matrixTargetObservation.trackId);
  commitTelemetryRow();
}

static void sendBluetoothCsvHeader() {
  beginTelemetryRow();
  Serial2.println(NAV_TELEMETRY_HEADER);
  commitTelemetryRow(TELEMETRY_HEADER_BUDGET_BYTES);
}

static void padSparseTelemetryRow(size_t populatedFields) {
  for (size_t field = populatedFields; field < NAV_TELEMETRY_FIELD_COUNT; field++) {
    telemetryStage.print(",");
  }
  telemetryStage.println();
}

static void finishSparseTelemetryRow(size_t populatedFields) {
  padSparseTelemetryRow(populatedFields);
  commitTelemetryRow(TELEMETRY_EVENT_ROW_BUDGET_BYTES);
}

static void finishSparseTelemetryEventRow(size_t populatedFields) {
  padSparseTelemetryRow(populatedFields);
  commitTelemetryEventRow();
}

static void sendBluetoothSchemaMetadata() {
  beginTelemetryRow();
  telemetryStage.print("meta,");
  telemetryStage.print(NAV_TELEMETRY_SCHEMA_VERSION);
  telemetryStage.print(","); telemetryStage.print(telemetrySequence++);
  telemetryStage.print(",schema,"); telemetryStage.print(ROBOT_BUILD_LABEL);
  telemetryStage.print(","); telemetryStage.print(millis());
  telemetryStage.print(","); telemetryStage.print(ROBOT_BUILD_LABEL);
  finishSparseTelemetryRow(7);
}

// Compact, versioned 10 Hz navigation evidence row. The frozen schema retains
// historical wall-oriented column names; the current receding-horizon planner
// maps its closest typed state into those compatibility fields.
static void sendBluetoothMotionRow() {
  beginTelemetryRow();
  const PlannerTelemetry& plannerTelemetry = getPlannerTelemetry();
  Serial2.print("telemetry,");
  Serial2.print(NAV_TELEMETRY_SCHEMA_VERSION);
  Serial2.print(","); Serial2.print(telemetrySequence++);
  Serial2.print(",,,"); Serial2.print(millis());
  Serial2.print(","); Serial2.print(ROBOT_BUILD_LABEL);
  Serial2.print(","); Serial2.print(robotStateName(currentState));
  Serial2.print(","); Serial2.print(robotRunEnabled ? 1 : 0);
  Serial2.print(","); Serial2.print(bluetoothTestArmed ? 1 : 0);
  Serial2.print(","); Serial2.print(robotX, 3);
  Serial2.print(","); Serial2.print(robotY, 3);
  Serial2.print(","); Serial2.print(robotTheta, 2);
  for (int i = RANGE_RIGHT_OUTER; i <= RANGE_LEFT_OUTER; i++) {
    Serial2.print(","); Serial2.print(getRangeSensorDistance((RangeSensorId)i));
  }
  for (int i = RANGE_RIGHT_OUTER; i <= RANGE_LEFT_OUTER; i++) {
    Serial2.print(","); Serial2.print(isRangeSensorValid((RangeSensorId)i) ? 1 : 0);
  }
  const unsigned long now = millis();
  for (int i = RANGE_RIGHT_OUTER; i <= RANGE_LEFT_OUTER; i++) {
    Serial2.print(","); Serial2.print(now - rangeSensors[i].lastReadMs);
  }
  Serial2.print(","); Serial2.print(frontBlocked ? 1 : 0);
  Serial2.print(","); Serial2.print(lastLeftMotorUs);
  Serial2.print(","); Serial2.print(lastRightMotorUs);
  Serial2.print(","); Serial2.print(plannerTelemetry.planReason);
  Serial2.print(","); Serial2.print(0);
  Serial2.print(","); Serial2.print(plannerTelemetry.minimumSweptClearanceMm, 1);
  Serial2.print(","); Serial2.print(plannerTelemetry.obstacleProgressAgeS, 2);
  Serial2.print(","); Serial2.print(plannerTelemetry.recoveryPlateauCount);
  Serial2.print(","); Serial2.print(plannerTelemetry.cumulativeReverseDistanceM, 3);
  Serial2.print(","); Serial2.print(plannerTelemetry.routeSignedLateralErrorM, 3);
  Serial2.print(","); Serial2.print(plannerTelemetry.selectedForwardTicksPerSec, 1);
  Serial2.print(","); Serial2.print(plannerTelemetry.selectedTurnTicksPerSec, 1);
  Serial2.print(","); Serial2.print(plannerTelemetry.minimumSweptClearanceMm, 1);
  Serial2.print(","); Serial2.print(plannerTelemetry.speedCapTicksPerSec, 1);
  Serial2.print(","); Serial2.print(plannerTelemetry.replanReason);
  Serial2.print(","); Serial2.print(plannerStopReasonName(plannerTelemetry.stopReason));
  Serial2.print(","); Serial2.print(lastRequestedLeftWheelSpeed, 1);
  Serial2.print(","); Serial2.print(lastRequestedRightWheelSpeed, 1);
  Serial2.print(","); Serial2.print(lastMeasuredLeftWheelSpeed, 1);
  Serial2.print(","); Serial2.print(lastMeasuredRightWheelSpeed, 1);
  Serial2.print(","); Serial2.print(lastImuClockwiseYawDeg, 2);
  Serial2.print(","); Serial2.print(lastNavigationHeadingDeg, 2);
  Serial2.print(","); Serial2.print(lastMotorOutputMode);
  Serial2.print(","); Serial2.print(motionAuthorityName(motionAuthority));
  Serial2.print(","); Serial2.print(motorCommandLeaseTripCount());
  Serial2.print(","); Serial2.print(maximumMainLoopGapMs());
  Serial2.print(","); Serial2.print(mainLoopDeadlineMissCount());
  Serial2.print(","); Serial2.print(maximumMainLoopPhaseName());
  Serial2.print(","); Serial2.print(maximumMainLoopPhaseUs());
  Serial2.print(","); Serial2.print(telemetryQueuedRows);
  Serial2.print(","); Serial2.print(telemetryQueueBytes);
  Serial2.print(","); Serial2.print(telemetryDroppedRows);
  Serial2.print(","); Serial2.print(telemetryRateLimitedEvents);
  Serial2.print(","); Serial2.print(plannerTelemetry.plannerSliceMaxUs);
  Serial2.print(","); Serial2.print(plannerTelemetry.plannerCommandAgeMs);
  Serial2.print(","); Serial2.print(plannerTelemetry.globalGoalDistanceM, 3);
  Serial2.println();
  commitTelemetryRow(TELEMETRY_MOTION_ROW_BUDGET_BYTES);
}

void sendBluetoothEvent(const char* eventName, const char* eventDetail) {
  // Queues a compact event row during CSV capture. Duplicate rate limiting
  // prevents a persistent fault from filling the telemetry ring every loop.
  if (!bluetoothCsvStreamEnabled) {
    return;
  }
  const unsigned long now = millis();
  if (strcmp(eventName, lastTelemetryEventName) == 0 &&
      strcmp(eventDetail, lastTelemetryEventDetail) == 0 &&
      now - lastTelemetryEventMs < TELEMETRY_DUPLICATE_EVENT_LIMIT_MS) {
    telemetryRateLimitedEvents++;
    return;
  }
  strncpy(lastTelemetryEventName, eventName, sizeof(lastTelemetryEventName) - 1);
  lastTelemetryEventName[sizeof(lastTelemetryEventName) - 1] = '\0';
  strncpy(lastTelemetryEventDetail, eventDetail, sizeof(lastTelemetryEventDetail) - 1);
  lastTelemetryEventDetail[sizeof(lastTelemetryEventDetail) - 1] = '\0';
  lastTelemetryEventMs = now;

  // Events share the versioned header and sequence. Sparse fields are padded,
  // so a partial or malformed record is detectable by width and sequence.
  beginTelemetryRow();
  telemetryStage.print("event,");
  telemetryStage.print(NAV_TELEMETRY_SCHEMA_VERSION);
  telemetryStage.print(","); telemetryStage.print(telemetrySequence++);
  telemetryStage.print(","); telemetryStage.print(eventName);
  telemetryStage.print(","); telemetryStage.print(eventDetail);
  telemetryStage.print(","); telemetryStage.print(now);
  telemetryStage.print(","); telemetryStage.print(ROBOT_BUILD_LABEL);
  finishSparseTelemetryEventRow(7);
}

void sendBluetoothTelemetry() {
  // Builds periodic telemetry rows. Transmission itself is performed later by
  // serviceBluetoothTelemetryTx() so row construction and UART draining stay
  // bounded separately.
  if (!bluetoothStreamEnabled && !bluetoothCsvStreamEnabled) {
    return;
  }

  const unsigned long now = millis();
  if (bluetoothCsvStreamEnabled &&
      now - lastBluetoothMotionTelemetryMs >= TELEMETRY_MOTION_INTERVAL_MS) {
    lastBluetoothMotionTelemetryMs = now;
    sendBluetoothMotionRow();
  }

  if (bluetoothStreamEnabled && !bluetoothCsvStreamEnabled) {
    if (now - lastBluetoothFullTelemetryMs >= TELEMETRY_FULL_INTERVAL_MS) {
      lastBluetoothFullTelemetryMs = now;
      sendBluetoothStatus();
    }
  }
}

static const char* avoidChoiceName(AvoidTurnChoice choice) {
  if (choice == AVOID_TURN_LEFT) {
    return "left";
  }
  if (choice == AVOID_TURN_RIGHT) {
    return "right";
  }
  return "none";
}

static AvoidTurnChoice sampleAvoidSideChoice(const char* &reason,
                                             AvoidSideClearance &left,
                                             AvoidSideClearance &right) {
  return evaluateAvoidTurnDirection(left, right, reason);
}

static void printSideDecisionHeader() {
  Serial2.println("side_decision,sample,ms,choice,reason,front_mm,front_valid,front_blocked,right_mm,right_valid,left_mm,left_valid,fan0_mm,fan1_mm,fan2_mm,fan3_mm,fan0_valid,fan1_valid,fan2_valid,fan3_valid,left_passable,left_inner_sweep_clearance_mm,left_outer_sweep_clearance_mm,left_score_mm,right_passable,right_inner_sweep_clearance_mm,right_outer_sweep_clearance_mm,right_score_mm");
}

static void printSideDecisionRow(int sampleNumber) {
  updateTOFSensors();

  const char* reason = "";
  AvoidSideClearance left;
  AvoidSideClearance right;
  AvoidTurnChoice choice = sampleAvoidSideChoice(reason, left, right);

  Serial2.print("side_decision,");
  Serial2.print(sampleNumber);
  Serial2.print(",");
  Serial2.print(millis());
  Serial2.print(",");
  Serial2.print(avoidChoiceName(choice));
  Serial2.print(",");
  Serial2.print(reason);
  Serial2.print(",");
  Serial2.print(getRangeSensorDistance(RANGE_FRONT));
  Serial2.print(",");
  Serial2.print(isRangeSensorValid(RANGE_FRONT) ? 1 : 0);
  Serial2.print(",");
  Serial2.print(isRangeSensorBlocked(RANGE_FRONT) ? 1 : 0);
  Serial2.print(",");
  Serial2.print(getRangeSensorDistance(RANGE_RIGHT));
  Serial2.print(",");
  Serial2.print(isRangeSensorValid(RANGE_RIGHT) ? 1 : 0);
  Serial2.print(",");
  Serial2.print(getRangeSensorDistance(RANGE_LEFT));
  Serial2.print(",");
  Serial2.print(isRangeSensorValid(RANGE_LEFT) ? 1 : 0);
  Serial2.print(",");
  Serial2.print(getRangeSensorDistance(RANGE_RIGHT_OUTER));
  Serial2.print(",");
  Serial2.print(getRangeSensorDistance(RANGE_RIGHT_INNER));
  Serial2.print(",");
  Serial2.print(getRangeSensorDistance(RANGE_LEFT_INNER));
  Serial2.print(",");
  Serial2.print(getRangeSensorDistance(RANGE_LEFT_OUTER));
  Serial2.print(",");
  Serial2.print(isRangeSensorValid(RANGE_RIGHT_OUTER) ? 1 : 0);
  Serial2.print(",");
  Serial2.print(isRangeSensorValid(RANGE_RIGHT_INNER) ? 1 : 0);
  Serial2.print(",");
  Serial2.print(isRangeSensorValid(RANGE_LEFT_INNER) ? 1 : 0);
  Serial2.print(",");
  Serial2.print(isRangeSensorValid(RANGE_LEFT_OUTER) ? 1 : 0);
  Serial2.print(",");
  Serial2.print(left.passable ? 1 : 0);
  Serial2.print(",");
  Serial2.print(left.innerSweepClearanceMm, 1);
  Serial2.print(",");
  Serial2.print(left.outerSweepClearanceMm, 1);
  Serial2.print(",");
  Serial2.print(left.scoreMm, 1);
  Serial2.print(",");
  Serial2.print(right.passable ? 1 : 0);
  Serial2.print(",");
  Serial2.print(right.innerSweepClearanceMm, 1);
  Serial2.print(",");
  Serial2.print(right.outerSweepClearanceMm, 1);
  Serial2.print(",");
  Serial2.println(right.scoreMm, 1);
}

static void printFloatRangeError(const char* commandName, float minValue, float maxValue) {
  Serial2.print("ERROR ");
  Serial2.print(commandName);
  Serial2.print(" must be between ");
  Serial2.print(minValue, 2);
  Serial2.print(" and ");
  Serial2.println(maxValue, 2);
}

static void stopManualDrive() {
  bluetoothManualActive = false;
  requestMotionStop();
  stopMotors();
}

static void runManualDriveCommand(float forwardPercent, float turnPercent) {
  // Live manual drive command. Percent inputs are scaled by baseTargetSpeed
  // into ticks/s, then submitted through the same authority/safety path as
  // autonomy.
  if (!bluetoothManualArmed || motionAuthority != MOTION_AUTHORITY_MANUAL) {
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

  if ((turnPercent > 0.0 &&
       (!isRangeSensorValid(RANGE_LEFT_INNER) || !isRangeSensorValid(RANGE_LEFT_OUTER))) ||
      (turnPercent < 0.0 &&
       (!isRangeSensorValid(RANGE_RIGHT_INNER) || !isRangeSensorValid(RANGE_RIGHT_OUTER)))) {
    stopManualDrive();
    Serial2.println("MANUAL blocked by invalid turn-side sensing. Motors stopped.");
    sendBluetoothEvent("manual_drive_blocked", "turn_side_invalid");
    return;
  }

  if (turnPercent != 0.0 && !isTurnSweepSafe()) {
    stopManualDrive();
    Serial2.println("MANUAL blocked by turn-sweep clearance. Motors stopped.");
    sendBluetoothEvent("manual_drive_blocked", "turn_sweep");
    return;
  }

  // Manual drive remains deliberately direct in intent, but it now feeds the
  // same single motor-output path as autonomous navigation.
  if (!setAuthorizedMotionCommand(MOTION_AUTHORITY_MANUAL,
                                  baseTargetSpeed * forwardPercent / 100.0,
                                  baseTargetSpeed * turnPercent / 100.0,
                                  MOTION_COMMAND_STANDARD)) {
    bluetoothManualActive = false;
    Serial2.print("MANUAL blocked by continuous safety supervisor: ");
    Serial2.println(motionSafetyReasonName(lastMotionSafetyReason()));
    return;
  }

  bluetoothManualActive = forwardPercent != 0.0 || turnPercent != 0.0;
  lastManualDriveCommandMs = millis();
}

bool isManualDriveActive() {
  return bluetoothManualActive;
}

void updateManualDriveTimeout() {
  // Deadman for live manual drive. If DRIVE commands stop arriving, neutralize
  // before a stale command can persist.
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
  // Test motion is a two-step action: TEST ARM claims TEST authority, then the
  // movement command creates a goal. This protects against accidental moves
  // from a single mistyped command.
  if (bluetoothTestArmed && motionAuthority == MOTION_AUTHORITY_TEST) {
    return true;
  }

  Serial2.println("ERROR test motion is disarmed. Send TEST ARM first.");
  return false;
}

static void beginBluetoothTestMotion() {
  // Common setup for TEST DRIVE/GOTO/AVOID/ESCAPE/TURN/HUNT/SEARCH. It enables
  // the normal state/controller loop but disables manual ownership.
  bluetoothAbortMotionRequested = false;
  bluetoothManualArmed = false;
  bluetoothManualActive = false;
  robotRunEnabled = true;
  endMatchPrinted = false;
  setRobotState(FOLLOW_PATH);
  clearStuckFlags();
}

static void recordBluetoothNavigationSubmission(bool accepted,
                                                const char* detail) {
  bluetoothNavigationTestActive = accepted;
  if (accepted) {
    return;
  }

  Serial2.println("ERROR navigation goal rejected.");
  sendBluetoothEvent("navigation_goal_rejected", detail);
  bluetoothTurnTruthActive = false;
  robotRunEnabled = false;
  requestMotionStop();
  setRobotState(END_MATCH);
}

static void runBluetoothTestDrive(float distanceMetres) {
  // TEST DRIVE creates a point goal straight ahead from the current pose.
  // distanceMetres is metres; heading comes from navigationHeadingDeg().
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

  float headingDeg = navigationHeadingDeg();
  float headingRad = headingDeg * DEG_TO_RAD;

  Serial2.print("OK test drive ");
  Serial2.print(distanceMetres, 3);
  Serial2.print(" m at heading ");
  Serial2.print(headingDeg, 2);
  Serial2.println(" deg.");

  beginBluetoothTestMotion();
  sendBluetoothEvent("test_drive_start", "manual");
  recordBluetoothNavigationSubmission(
    navigationStartTestPoint(
      robotX + cosf(headingRad) * distanceMetres,
      robotY + sinf(headingRad) * distanceMetres,
      NAVIGATION_TEST_DRIVE),
    "test_drive");
}

static void runBluetoothTestGoto(float targetX, float targetY) {
  // TEST GOTO creates an absolute world-frame point goal in metres. The field
  // UI intentionally does not call this until a safe transform/confirmation
  // flow exists.
  if (!requireBluetoothTestArm()) {
    return;
  }

  if (targetX < TEST_GOTO_MIN_COORD_M || targetX > TEST_GOTO_MAX_COORD_M ||
      targetY < TEST_GOTO_MIN_COORD_M || targetY > TEST_GOTO_MAX_COORD_M) {
    printFloatRangeError("TEST GOTO x/y", TEST_GOTO_MIN_COORD_M, TEST_GOTO_MAX_COORD_M);
    return;
  }

  Serial2.print("OK test goto x=");
  Serial2.print(targetX, 3);
  Serial2.print(" y=");
  Serial2.print(targetY, 3);
  Serial2.println(".");

  beginBluetoothTestMotion();
  sendBluetoothEvent("test_goto_start", "manual");
  recordBluetoothNavigationSubmission(
    navigationStartTestPoint(targetX, targetY, NAVIGATION_TEST_GOTO),
    "test_goto");
}

static void printMatrixHuntTarget() {
  MatrixTargetObservation target;
  const bool fresh = getMatrixTargetObservation(target);
  Serial2.print("matrix_hunt_target,valid=");
  Serial2.print(fresh ? 1 : 0);
  Serial2.print(",evidence=");
  Serial2.print(matrixEvidenceKindName(matrixTargetObservation.evidence));
  Serial2.print(",confirmed=");
  Serial2.print(matrixTargetObservation.confirmedStatic ? 1 : 0);
  Serial2.print(",track_id=");
  Serial2.print(matrixTargetObservation.trackId);
  Serial2.print(",gap_mm=");
  Serial2.print(matrixTargetObservation.directGapMm, 1);
  Serial2.print(",robot_x_mm=");
  Serial2.print(matrixTargetObservation.robotXmm, 1);
  Serial2.print(",robot_y_mm=");
  Serial2.print(matrixTargetObservation.robotYmm, 1);
  Serial2.print(",world_x_m=");
  Serial2.print(matrixTargetObservation.worldX, 3);
  Serial2.print(",world_y_m=");
  Serial2.print(matrixTargetObservation.worldY, 3);
  Serial2.print(",column=");
  Serial2.println(matrixTargetObservation.column, 2);
}

static void runBluetoothTestHunt() {
  if (!requireBluetoothTestArm()) {
    return;
  }

  MatrixTargetObservation target;
  if (!getMatrixTargetObservation(target)) {
    Serial2.println("ERROR no fresh confirmed static matrix target. Use TEST MATRIX or TEST HUNT TARGET first.");
    return;
  }

  Serial2.print("OK test hunt target x=");
  Serial2.print(target.worldX, 3);
  Serial2.print(" y=");
  Serial2.print(target.worldY, 3);
  Serial2.print(" robot_mm=");
  Serial2.print(target.robotXmm, 1);
  Serial2.print("/");
  Serial2.print(target.robotYmm, 1);
  Serial2.print(" track=");
  Serial2.print(target.trackId);
  Serial2.println(".");

  beginBluetoothTestMotion();
  sendBluetoothEvent("test_hunt_start", "matrix_static_track");
  RouteResumeContext noRoute = {};
  recordBluetoothNavigationSubmission(
    navigationStartPickupTracking(target, noRoute),
    "test_hunt");
}

static void runBluetoothTestSearch() {
  if (!requireBluetoothTestArm()) {
    return;
  }

  Serial2.println("OK test search using current pose as search waypoint.");
  beginBluetoothTestMotion();
  sendBluetoothEvent("test_search_start", "manual");
  startWeightSearchTest();
}

// TEST AVOID and TEST ESCAPE are the same action: a straight-ahead point goal
// that exercises the normal planner, not a scripted routine. They differ only
// in the diagnostic they are aimed at, which is carried by the goal kind and
// the operator-facing wording.
static void runBluetoothTestBearingPoint(float distanceMetres,
                                         const char* commandName,
                                         const char* okPrefix,
                                         const char* okSuffix,
                                         const char* startEvent,
                                         const char* detail,
                                         NavigationTestPointKind kind) {
  if (!requireBluetoothTestArm()) {
    return;
  }

  if (distanceMetres < TEST_AVOID_MIN_METRES || distanceMetres > TEST_AVOID_MAX_METRES) {
    printFloatRangeError(commandName, TEST_AVOID_MIN_METRES, TEST_AVOID_MAX_METRES);
    return;
  }

  float headingDeg = navigationHeadingDeg();
  float headingRad = headingDeg * PI / 180.0;
  float targetX = robotX + cos(headingRad) * distanceMetres;
  float targetY = robotY + sin(headingRad) * distanceMetres;

  Serial2.print(okPrefix);
  Serial2.print(distanceMetres, 3);
  Serial2.print(" m toward x=");
  Serial2.print(targetX, 3);
  Serial2.print(" y=");
  Serial2.print(targetY, 3);
  Serial2.print(" heading=");
  Serial2.print(headingDeg, 2);
  Serial2.println(okSuffix);

  beginBluetoothTestMotion();
  sendBluetoothEvent(startEvent, "manual");
  recordBluetoothNavigationSubmission(
    navigationStartTestPoint(targetX, targetY, kind), detail);
}

static void runBluetoothTestAvoid(float distanceMetres) {
  runBluetoothTestBearingPoint(
    distanceMetres, "TEST AVOID", "OK test avoid ", " deg.",
    "test_avoid_start", "test_avoid", NAVIGATION_TEST_AVOID);
}

static void runBluetoothTestEscape(float distanceMetres) {
  // Aimed at front-blocked reverse-recovery diagnostics; the reverse leg still
  // depends on trusted rear coverage, hence the extra operator note.
  runBluetoothTestBearingPoint(
    distanceMetres, "TEST ESCAPE", "OK test escape scan ",
    " deg. Reverse repositioning still requires trusted rear coverage.",
    "test_escape_start", "test_escape", NAVIGATION_TEST_ESCAPE);
}

static void runBluetoothTestSide(float durationSeconds) {
  if (durationSeconds < TEST_SIDE_MIN_SECONDS || durationSeconds > TEST_SIDE_MAX_SECONDS) {
    printFloatRangeError("TEST SIDE", TEST_SIDE_MIN_SECONDS, TEST_SIDE_MAX_SECONDS);
    return;
  }

  stopMotors();
  requestMotionStop();
  bluetoothManualActive = false;
  bluetoothAbortMotionRequested = false;

  unsigned long startMs = millis();
  sideTestEndMs = startMs + (unsigned long)(durationSeconds * 1000.0);
  sideTestNextSampleMs = startMs;
  sideTestSampleNumber = 0;
  bluetoothSideTestActive = true;

  Serial2.print("OK side decision sampler for ");
  Serial2.print(durationSeconds, 1);
  Serial2.println(" s. Move the obstacle around the fan; motors stay stopped.");
  printSideDecisionHeader();

}

static void updateBluetoothTestSide() {
  if (!bluetoothSideTestActive) {
    return;
  }

  unsigned long now = millis();
  requestMotionStop();

  if (now >= sideTestNextSampleMs) {
    printSideDecisionRow(sideTestSampleNumber++);
    sideTestNextSampleMs = now + TEST_SIDE_SAMPLE_INTERVAL_MS;
  }

  if (now >= sideTestEndMs) {
    bluetoothSideTestActive = false;
    Serial2.println("TEST SIDE complete.");
  }
}

static void runBluetoothTestTurn(float angleDeg) {
  // TEST TURN creates a relative yaw goal in degrees. Positive is CCW/left in
  // the navigation convention.
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
  turnTruthStartMs = millis();
  turnTruthNextSampleMs = turnTruthStartMs + TEST_TRUTH_FIRST_SAMPLE_MS;
  bluetoothTurnTruthActive = true;
  sendBluetoothEvent("test_turn_start", "manual");
  recordBluetoothNavigationSubmission(
    navigationStartTestTurn(angleDeg),
    "test_turn");
}

static const char* matrixFollowPhaseName(MatrixFollowDiagnosticPhase phase) {
  switch (phase) {
    case MATRIX_FOLLOW_IDLE: return "IDLE";
    case MATRIX_FOLLOW_SLOW_SCAN: return "SLOW_SCAN";
    case MATRIX_FOLLOW_ACQUIRE_STATIC_WEIGHT: return "ACQUIRE_STATIC_WEIGHT";
    case MATRIX_FOLLOW_LOCKED_TRACK: return "FOLLOW_LOCKED_TRACK";
    case MATRIX_FOLLOW_BOUNDED_REACQUIRE: return "BOUNDED_REACQUIRE";
    case MATRIX_FOLLOW_STOPPED: return "STOPPED";
  }
  return "UNKNOWN";
}

static void printMatrixFollowStatus() {
  MatrixFollowDiagnosticStatus status = getMatrixFollowDiagnosticStatus();
  Serial2.print("matrix_follow,phase=");
  Serial2.print(matrixFollowPhaseName(status.phase));
  Serial2.print(",track=");
  Serial2.print(status.trackId);
  Serial2.print(",detail=");
  Serial2.println(status.detail);
}

static void runBluetoothMatrixFollowStart() {
  if (!requireBluetoothTestArm()) return;
  beginBluetoothTestMotion();
  const bool accepted = startMatrixFollowDiagnostic();
  recordBluetoothNavigationSubmission(accepted, "matrix_follow");
  if (accepted) printMatrixFollowStatus();
}

static void printBluetoothTurnTruth(const char* phase) {
  long leftCount;
  long rightCount;
  readEncoderCounts(leftCount, rightCount);
  Serial2.print("turn_truth,"); Serial2.print(phase);
  Serial2.print(",ms="); Serial2.print(millis() - turnTruthStartMs);
  Serial2.print(";tl="); Serial2.print(lastRequestedLeftWheelSpeed, 1);
  Serial2.print(";tr="); Serial2.print(lastRequestedRightWheelSpeed, 1);
  Serial2.print(";pl="); Serial2.print(lastLeftMotorUs);
  Serial2.print(";pr="); Serial2.print(lastRightMotorUs);
  Serial2.print(";el="); Serial2.print(leftCount);
  Serial2.print(";er="); Serial2.print(rightCount);
  Serial2.print(";rl="); Serial2.print(lastMeasuredLeftWheelSpeed, 1);
  Serial2.print(";rr="); Serial2.print(lastMeasuredRightWheelSpeed, 1);
  Serial2.print(";ir="); Serial2.print(lastImuClockwiseYawDeg, 2);
  Serial2.print(";ny="); Serial2.print(lastNavigationHeadingDeg, 2);
  Serial2.print(";m="); Serial2.print(lastMotorOutputMode);
  Serial2.print(";a="); Serial2.print(motionAuthorityName(motionAuthority));
  Serial2.print(";lt="); Serial2.println(motorCommandLeaseTripCount());
}

static void updateBluetoothTurnTruth() {
  if (!bluetoothTurnTruthActive) {
    return;
  }
  const NavigationDebugStatus navigation = getNavigationDebugStatus();
  const PlannerTelemetry& plannerTelemetry = getPlannerTelemetry();
  const unsigned long now = millis();
  if (navigation.testTurnActive) {
    if (now >= turnTruthNextSampleMs) {
      printBluetoothTurnTruth("d");
      turnTruthNextSampleMs = now + TEST_TRUTH_SAMPLE_INTERVAL_MS;
    }
    return;
  }

  printBluetoothTurnTruth(navigation.completed ? "e" : "x");
  Serial2.print(navigation.completed ? "test_turn_end," : "test_turn_abort,");
  Serial2.println(plannerTelemetry.safeStopReason);
  bluetoothTurnTruthActive = false;
}

static void updateBluetoothNavigationTest() {
  if (!bluetoothNavigationTestActive) {
    return;
  }
  NavigationStatus navigation = navigationGetStatus();
  if (navigation.state == NAVIGATION_RUNNING) {
    return;
  }
  bluetoothNavigationTestActive = false;
  robotRunEnabled = false;
  requestMotionStop();
  setRobotState(END_MATCH);
  Serial.println(navigation.state == NAVIGATION_REACHED
    ? "TEST complete. Motors stopped."
    : "TEST aborted. Motors stopped.");
}

static void finishBluetoothArcTest(const char* detail) {
  long leftCount;
  long rightCount;
  readEncoderCounts(leftCount, rightCount);
  Serial2.print("arc_truth,e,ms=");
  Serial2.print(millis() - arcTestStartMs);
  Serial2.print(";f="); Serial2.print(arcTestForwardTicksPerSec, 1);
  Serial2.print(";t="); Serial2.print(arcTestTurnTicksPerSec, 1);
  Serial2.print(";tl="); Serial2.print(lastRequestedLeftWheelSpeed, 1);
  Serial2.print(";tr="); Serial2.print(lastRequestedRightWheelSpeed, 1);
  Serial2.print(";pl="); Serial2.print(lastLeftMotorUs);
  Serial2.print(";pr="); Serial2.print(lastRightMotorUs);
  Serial2.print(";el="); Serial2.print(leftCount);
  Serial2.print(";er="); Serial2.print(rightCount);
  Serial2.print(";rl="); Serial2.print(lastMeasuredLeftWheelSpeed, 1);
  Serial2.print(";rr="); Serial2.print(lastMeasuredRightWheelSpeed, 1);
  Serial2.print(";ir="); Serial2.print(lastImuClockwiseYawDeg, 2);
  Serial2.print(";ny="); Serial2.print(lastNavigationHeadingDeg, 2);
  Serial2.print(";m="); Serial2.print(lastMotorOutputMode);
  Serial2.print(";lt="); Serial2.println(motorCommandLeaseTripCount());
  bluetoothArcTestActive = false;
  bluetoothManualActive = false;
  robotRunEnabled = false;
  setAuthorizedMotionCommand(MOTION_AUTHORITY_TEST, 0.0, 0.0,
                             MOTION_COMMAND_STANDARD);
  stopMotors();
  setRobotState(END_MATCH);
  sendBluetoothEvent("test_arc_end", detail);
  Serial2.print("test_arc_end,");
  Serial2.println(detail);
  Serial2.println("TEST ARC complete; motors neutral.");
}

static void runBluetoothTestArc(float forwardTicksPerSec,
                                float turnTicksPerSec,
                                float durationSeconds) {
  // Direct bounded wheel-PID sign diagnostic. It bypasses navigation goals but
  // still claims TEST authority and uses setAuthorizedMotionCommand() so final
  // safety supervision remains active.
  if (!requireBluetoothTestArm()) {
    return;
  }
  const float leftTarget = leftWheelTargetFromChassis(
    forwardTicksPerSec, turnTicksPerSec);
  const float rightTarget = rightWheelTargetFromChassis(
    forwardTicksPerSec, turnTicksPerSec);
  if (forwardTicksPerSec < TEST_ARC_MIN_FORWARD_TPS ||
      forwardTicksPerSec > TEST_ARC_MAX_FORWARD_TPS ||
      fabs(turnTicksPerSec) > TEST_ARC_MAX_ABS_TURN_TPS ||
      durationSeconds < TEST_ARC_MIN_SECONDS ||
      durationSeconds > TEST_ARC_MAX_SECONDS ||
      leftTarget < 0.0f || rightTarget < 0.0f) {
    Serial2.println(
      "ERROR TEST ARC requires forward 300..1800 tps, |turn| <=500 tps, "
      "duration 0.10..0.80 s, and nonnegative wheel targets.");
    return;
  }

  updateTOFSensors();
  arcTestForwardTicksPerSec = forwardTicksPerSec;
  arcTestTurnTicksPerSec = turnTicksPerSec;
  arcTestStartMs = millis();
  arcTestEndMs = arcTestStartMs + (unsigned long)(durationSeconds * 1000.0f);
  arcTestNextSampleMs = arcTestStartMs + TEST_TRUTH_FIRST_SAMPLE_MS;
  bluetoothArcTestActive = true;
  // Reuse the existing direct-motion scheduler guard so RobotCode.ino does
  // not overwrite this TEST-authority command with the idle stop branch.
  bluetoothManualActive = true;
  lastManualDriveCommandMs = millis();
  bluetoothAbortMotionRequested = false;
  robotRunEnabled = false;
  clearStuckFlags();
  if (!setAuthorizedMotionCommand(MOTION_AUTHORITY_TEST,
                                  arcTestForwardTicksPerSec,
                                  arcTestTurnTicksPerSec,
                                  MOTION_COMMAND_STANDARD)) {
    bluetoothArcTestActive = false;
    Serial2.print("ERROR TEST ARC refused by continuous safety supervisor: ");
    Serial2.println(motionSafetyReasonName(lastMotionSafetyReason()));
    return;
  }

  char detail[64];
  snprintf(detail, sizeof(detail), "f=%.0f;t=%.0f;l=%.0f;r=%.0f;ms=%lu",
           forwardTicksPerSec, turnTicksPerSec, leftTarget, rightTarget,
           (unsigned long)(durationSeconds * 1000.0f));
  sendBluetoothEvent("test_arc_start", detail);
  Serial2.print("OK TEST ARC ");
  Serial2.println(detail);
}

static void updateBluetoothArcTest() {
  if (!bluetoothArcTestActive) {
    return;
  }
  if (millis() >= arcTestEndMs) {
    finishBluetoothArcTest("duration_complete");
    return;
  }
  bluetoothManualActive = true;
  lastManualDriveCommandMs = millis();
  if (!setAuthorizedMotionCommand(MOTION_AUTHORITY_TEST,
                                  arcTestForwardTicksPerSec,
                                  arcTestTurnTicksPerSec,
                                  MOTION_COMMAND_STANDARD)) {
    finishBluetoothArcTest("safety_abort");
    return;
  }
  const unsigned long now = millis();
  if (now >= arcTestNextSampleMs) {
    long leftCount;
    long rightCount;
    readEncoderCounts(leftCount, rightCount);
    Serial2.print("arc_truth,d,ms=");
    Serial2.print(now - arcTestStartMs);
    Serial2.print(";f="); Serial2.print(arcTestForwardTicksPerSec, 1);
    Serial2.print(";t="); Serial2.print(arcTestTurnTicksPerSec, 1);
    Serial2.print(";tl="); Serial2.print(lastRequestedLeftWheelSpeed, 1);
    Serial2.print(";tr="); Serial2.print(lastRequestedRightWheelSpeed, 1);
    Serial2.print(";pl="); Serial2.print(lastLeftMotorUs);
    Serial2.print(";pr="); Serial2.print(lastRightMotorUs);
    Serial2.print(";el="); Serial2.print(leftCount);
    Serial2.print(";er="); Serial2.print(rightCount);
    Serial2.print(";rl="); Serial2.print(lastMeasuredLeftWheelSpeed, 1);
    Serial2.print(";rr="); Serial2.print(lastMeasuredRightWheelSpeed, 1);
    Serial2.print(";ir="); Serial2.print(lastImuClockwiseYawDeg, 2);
    Serial2.print(";ny="); Serial2.print(lastNavigationHeadingDeg, 2);
    Serial2.print(";m="); Serial2.print(lastMotorOutputMode);
    Serial2.print(";a="); Serial2.print(motionAuthorityName(motionAuthority));
    Serial2.print(";lt="); Serial2.println(motorCommandLeaseTripCount());
    arcTestNextSampleMs = now + TEST_TRUTH_SAMPLE_INTERVAL_MS;
  }
}

static void printTurnPulseSample(const char* phase) {
  Serial2.print("turn_pulse,");
  Serial2.print(phase);
  Serial2.print(",elapsed_ms,");
  Serial2.print(millis() - turnPulseStartMs);
  Serial2.print(",nav_heading_delta_deg,");
  Serial2.print(wrapAngle(navigationHeadingDeg() - turnPulseStartHeadingDeg), 2);
  Serial2.print(",command_tps,");
  Serial2.println(turnPulseCoasting ? 0.0 : turnPulseCommandTicksPerSec, 1);
}

static void finishBluetoothTurnPulseTest() {
  float finalHeadingDeg = wrapAngle(
    navigationHeadingDeg() - turnPulseStartHeadingDeg);
  bluetoothTurnPulseTestActive = false;
  turnPulseCoasting = false;
  bluetoothManualActive = false;
  robotRunEnabled = false;
  requestMotionStop();
  setRobotState(END_MATCH);
  Serial2.print("TURNPULSE complete: final_nav_heading_delta_deg=");
  Serial2.println(finalHeadingDeg, 2);
  sendBluetoothEvent("test_turnpulse_end", "coast_logged");
}

static void runBluetoothTestTurnPulse(float signedSeconds) {
  // Physical turn calibration pulse. The sign selects direction, magnitude is
  // duration in seconds, and the coast phase is logged without continuing the
  // drive command.
  if (!requireBluetoothTestArm()) {
    return;
  }

  float durationSeconds = fabs(signedSeconds);
  if (durationSeconds < TEST_TURN_PULSE_MIN_SECONDS ||
      durationSeconds > TEST_TURN_PULSE_MAX_SECONDS) {
    printFloatRangeError("TEST TURNPULSE magnitude", TEST_TURN_PULSE_MIN_SECONDS,
                         TEST_TURN_PULSE_MAX_SECONDS);
    return;
  }

  float direction = signedSeconds >= 0.0 ? 1.0 : -1.0;
  turnPulseCommandTicksPerSec = direction * PLANNER_TURN_TARGET_SPEED;
  updateTOFSensors();
  if (!isTurnDirectionObservable(turnPulseCommandTicksPerSec) || !isTurnSweepSafe()) {
    Serial2.println("ERROR TEST TURNPULSE refused by turn sensing or sweep clearance.");
    return;
  }

  if (navigationGetStatus().state == NAVIGATION_RUNNING) {
    navigationCancelWithReason(PLANNER_STOP_ABORTED, "turnpulse_started");
  }
  bluetoothAbortMotionRequested = false;
  bluetoothManualArmed = false;
  bluetoothManualActive = true;
  robotRunEnabled = false;
  clearStuckFlags();
  turnPulseStartHeadingDeg = navigationHeadingDeg();
  unsigned long now = millis();
  turnPulseStartMs = now;
  turnPulseEndMs = now + (unsigned long)(durationSeconds * 1000.0);
  turnPulseCoastEndMs = turnPulseEndMs + TEST_TURN_PULSE_COAST_MS;
  turnPulseNextSampleMs = now;
  turnPulseCoasting = false;
  bluetoothTurnPulseTestActive = true;
  if (!setAuthorizedMotionCommand(MOTION_AUTHORITY_TEST, 0.0,
                                  turnPulseCommandTicksPerSec,
                                  MOTION_COMMAND_STANDARD)) {
    Serial2.print("ERROR TEST TURNPULSE refused by continuous safety supervisor: ");
    Serial2.println(motionSafetyReasonName(lastMotionSafetyReason()));
    finishBluetoothTurnPulseTest();
    return;
  }

  Serial2.print("OK turn pulse ");
  Serial2.print(signedSeconds, 2);
  Serial2.println(" s at full calibrated turn rate; logging 1 s coast.");
  sendBluetoothEvent("test_turnpulse_start", "calibration");
}

static void runBluetoothTestTurnLadder(const char* directionText) {
  (void)directionText;
  stopMotors();
  Serial2.println(
    "ERROR TEST TURNLADDER disabled: blocking raw pulses cannot be continuously supervised; defer conversion to P0-06.");
}

static void updateBluetoothTurnPulseTest() {
  if (!bluetoothTurnPulseTestActive) {
    return;
  }

  unsigned long now = millis();
  if (!turnPulseCoasting) {
    updateTOFSensors();
    if (!isTurnDirectionObservable(turnPulseCommandTicksPerSec) || !isTurnSweepSafe()) {
      Serial2.println("TURNPULSE aborted by turn sensing or sweep clearance.");
      finishBluetoothTurnPulseTest();
      return;
    }
    if (!setAuthorizedMotionCommand(MOTION_AUTHORITY_TEST, 0.0,
                                    turnPulseCommandTicksPerSec,
                                    MOTION_COMMAND_STANDARD)) {
      Serial2.print("TURNPULSE aborted by continuous safety supervisor: ");
      Serial2.println(motionSafetyReasonName(lastMotionSafetyReason()));
      finishBluetoothTurnPulseTest();
      return;
    }
    bluetoothManualActive = true;
    lastManualDriveCommandMs = now;
    if (now >= turnPulseEndMs) {
      turnPulseCoasting = true;
      bluetoothManualActive = false;
      requestMotionStop();
      Serial2.println("TURNPULSE drive phase complete; logging coast.");
    }
  }

  if (now >= turnPulseNextSampleMs) {
    printTurnPulseSample(turnPulseCoasting ? "coast" : "drive");
    turnPulseNextSampleMs = now + TEST_TURN_PULSE_SAMPLE_INTERVAL_MS;
  }

  if (turnPulseCoasting && now >= turnPulseCoastEndMs) {
    finishBluetoothTurnPulseTest();
  }
}

static void printBluetoothBuild() {
  Serial2.print("BUILD ");
  Serial2.println(ROBOT_BUILD_LABEL);
}

static void markBluetoothLog(const char* note) {
  if (note[0] == '\0') {
    Serial2.println("ERROR usage: MARK <note>");
    return;
  }

  char safeNote[48];
  int i = 0;
  while (note[i] != '\0' && i < (int)sizeof(safeNote) - 1) {
    safeNote[i] = note[i] == ',' ? ';' : note[i];
    i++;
  }
  safeNote[i] = '\0';

  Serial2.print("MARK ");
  Serial2.println(safeNote);
  sendBluetoothEvent("mark", safeNote);
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

static void setBluetoothSearchTurnOffset(float offsetUs) {
  if (offsetUs < SEARCHTURN_MIN_US || offsetUs > SEARCHTURN_MAX_US) {
    printFloatRangeError("SEARCHTURN", SEARCHTURN_MIN_US, SEARCHTURN_MAX_US);
    return;
  }

  weightScanTurnOffsetUs = (int)(offsetUs + 0.5f);
  Serial2.print("OK search turn offset set to ");
  Serial2.print(weightScanTurnOffsetUs);
  Serial2.println(" us.");
  sendBluetoothEvent("searchturn_set", "manual");
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

static void revokeAndCancelAllMotion(const char* detail) {
  // Revoke at the motor sink first. Any stale planner command issued during
  // cleanup is rejected before it can become a physical motor output.
  revokeMotionAuthority();
  bluetoothManualActive = false;
  bluetoothTurnPulseTestActive = false;
  bluetoothArcTestActive = false;
  bluetoothSideTestActive = false;
  bluetoothNavigationTestActive = false;
  if (navigationGetStatus().state == NAVIGATION_RUNNING) {
    navigationCancelWithReason(PLANNER_STOP_ABORTED, detail);
  }
  if (isWeightSearchActive()) {
    cancelWeightSearch(detail);
  }
  stopMotors();
}

void disarmBluetoothMotionModes() {
  bluetoothTestArmed = false;
  bluetoothManualArmed = false;
  bluetoothManualActive = false;
  bluetoothTurnPulseTestActive = false;
  bluetoothArcTestActive = false;
  bluetoothNavigationTestActive = false;
}

static void zeroRobotPoseFromBluetooth() {
  // ZERO is a full software coordinate reset: stop/revoke first, zero yaw and
  // pose, reset encoder/PID references, clear local map, and park in END_MATCH.
  revokeAndCancelAllMotion("zero_command");
  zeroYaw();

  robotX = 0.0;
  robotY = 0.0;
  robotTheta = 0.0;

  resetEncodersAndPID();
  navigationResetMap();
  resetRouteMission();
  returnHomeRequested = false;
  clearStuckFlags();
  robotRunEnabled = false;
  bluetoothTestArmed = false;
  bluetoothManualArmed = false;
  bluetoothManualActive = false;
  bluetoothAbortMotionRequested = true;
  endMatchPrinted = false;
  setRobotState(END_MATCH);
  Serial2.println("OK stopped and zeroed yaw, pose, encoder references, map, and PID state.");
}

static bool handleCoreBluetoothCommand(const char* command) {
  // Core commands affect run state or global safety: START, STOP, ZERO, HOME,
  // STATUS, BUILD, CAL, and loop timing reset.
  if (commandEquals(command, "HELP") || commandEquals(command, "H")) {
    printBluetoothHelp();
    return true;
  }

  if (commandEquals(command, "BUILD")) {
    printBluetoothBuild();
    return true;
  }

  if (commandEquals(command, "COMPETITION ON")) {
    if (!setCompetitionModeEnabled(true)) {
      Serial2.println(
        "ERROR COMPETITION requires neutral motors and no active mission; set it before START.");
      return true;
    }
    Serial2.println(
      "OK competition mode ON. Route and home failures will retry until STOP; safety gates remain active.");
    sendBluetoothEvent("competition_mode_set", "on");
    return true;
  }

  if (commandEquals(command, "COMPETITION OFF")) {
    if (!setCompetitionModeEnabled(false)) {
      Serial2.println(
        "ERROR COMPETITION requires neutral motors and no active mission; use STOP first.");
      return true;
    }
    Serial2.println("OK competition mode OFF. Typed failures remain visible and neutral.");
    sendBluetoothEvent("competition_mode_set", "off");
    return true;
  }

  if (commandEquals(command, "COMPETITION STATUS")) {
    Serial2.print("COMPETITION ");
    Serial2.println(isCompetitionModeEnabled() ? "ON" : "OFF");
    return true;
  }

  if (commandHasPrefix(command, "COMPETITION")) {
    Serial2.println(
      "ERROR usage: COMPETITION ON, COMPETITION OFF, or COMPETITION STATUS");
    return true;
  }

  if (commandEquals(command, "START")) {
    revokeAndCancelAllMotion("start_authority_transition");
    claimMotionAuthority(MOTION_AUTHORITY_MISSION);
    // A fresh mission owns the decision to clear navigation's local map.
    // Navigation itself no longer performs mission-start cleanup.
    if (currentState == INIT || currentState == END_MATCH) {
      navigationResetMap();
    }
    robotRunEnabled = true;
    bluetoothTestArmed = false;
    bluetoothManualArmed = false;
    bluetoothManualActive = false;
    bluetoothAbortMotionRequested = false;
    endMatchPrinted = false;
    returnHomeRequested = false;
    navigationClearResult();

    if (currentState == END_MATCH) {
      currentState = INIT;
    }

    Serial2.println("OK start requested.");
    return true;
  }

  if (commandEquals(command, "STATUS") || commandEquals(command, "P")) {
    sendBluetoothStatus();
    return true;
  }

  if (commandEquals(command, "LOOP RESET")) {
    if (lastLeftMotorUs != STOP_US || lastRightMotorUs != STOP_US ||
        isMotorCommandLeaseArmed()) {
      Serial2.println("ERROR LOOP RESET requires neutral motors and an inactive lease.");
      return true;
    }
    resetMainLoopTimingDiagnostics();
    Serial2.println("OK loop timing diagnostics reset.");
    return true;
  }

  if (commandEquals(command, "CAL") || commandEquals(command, "CALIBRATION")) {
    printCalibrationSummary();
    return true;
  }

  if (commandEquals(command, "HOME")) {
    bluetoothTestArmed = false;
    bluetoothManualArmed = false;
    bluetoothManualActive = false;
    bluetoothTurnPulseTestActive = false;
    if (navigationGetStatus().state == NAVIGATION_RUNNING) {
      navigationCancelWithReason(PLANNER_STOP_ABORTED, "home_requested");
      navigationClearResult();
    }
    returnHomeRequested = true;
    Serial2.println("OK return home requested.");
    return true;
  }

  if (commandEquals(command, "ZERO")) {
    zeroRobotPoseFromBluetooth();
    return true;
  }

  if (commandEquals(command, "STOP") || commandEquals(command, "S")) {
    revokeAndCancelAllMotion("stop_command");
    robotRunEnabled = false;
    bluetoothTestArmed = false;
    bluetoothManualArmed = false;
    bluetoothManualActive = false;
    bluetoothSideTestActive = false;
    bluetoothTurnPulseTestActive = false;
    bluetoothArcTestActive = false;
    bluetoothAbortMotionRequested = true;
    endMatchPrinted = false;
    stopMotors();
    setRobotState(END_MATCH);
    Serial2.println("OK stopped motors and set END_MATCH.");
    return true;
  }

  return false;
}

static bool handleStreamAndLogBluetoothCommand(const char* command) {
  // Telemetry/logging commands. CSV ON starts the schema and enables queued
  // machine-readable rows; MARK inserts an event into saved logs.
  if (commandEquals(command, "STREAM ON")) {
    bluetoothStreamEnabled = true;
    lastBluetoothFullTelemetryMs = 0;
    Serial2.println("OK stream on.");
    return true;
  }

  if (commandEquals(command, "STREAM OFF")) {
    bluetoothStreamEnabled = false;
    Serial2.println("OK stream off.");
    return true;
  }

  if (commandEquals(command, "CSV ON")) {
    if (bluetoothCsvClosing) {
      Serial2.println("ERROR csv transport is still closing.");
      return true;
    }
    bluetoothCsvStreamEnabled = true;
    telemetrySequence = 0;
    telemetryDroppedRows = 0;
    telemetryRateLimitedEvents = 0;
    lastBluetoothMotionTelemetryMs = millis();
    lastBluetoothFullTelemetryMs = millis();
    sendBluetoothCsvHeader();
    sendBluetoothSchemaMetadata();
    Serial2.println("OK csv on.");
    Serial2.println("CSV READY");
    return true;
  }

  if (commandEquals(command, "CSV OFF")) {
    if (!bluetoothCsvStreamEnabled && !bluetoothCsvClosing) {
      Serial2.println("OK csv off.");
      return true;
    }
    bluetoothCsvClosing = true;
    bluetoothCsvStreamEnabled = false;
    Serial2.println("OK csv off.");
    Serial2.println("CSV CLOSED");
    return true;
  }

  if (commandHasPrefix(command, "MARK")) {
    markBluetoothLog(commandArgument(command, "MARK"));
    return true;
  }

  return false;
}

static bool handleTuningBluetoothCommand(const char* command) {
  // Runtime tuning commands affect only this boot. They deliberately modify
  // global runtime copies rather than RobotConfig.h constants.
  if (commandEquals(command, "SPEED RESET")) {
    baseTargetSpeed = DEFAULT_BASE_TARGET_SPEED;
    Serial2.print("OK base target speed reset to ");
    Serial2.print(baseTargetSpeed, 1);
    Serial2.println(" ticks/s.");
    sendBluetoothEvent("speed_reset", "manual");
    return true;
  }

  if (commandEquals(command, "SEARCHTURN RESET")) {
    weightScanTurnOffsetUs = DEFAULT_WEIGHT_SCAN_TURN_OFFSET_US;
    Serial2.print("OK search turn offset reset to ");
    Serial2.print(weightScanTurnOffsetUs);
    Serial2.println(" us.");
    sendBluetoothEvent("searchturn_reset", "manual");
    return true;
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
    return true;
  }

  if (commandHasPrefix(command, "SEARCHTURN")) {
    float offsetUs = 0.0;
    if (!parseFloatArgument(commandArgument(command, "SEARCHTURN"), offsetUs)) {
      Serial2.println("ERROR usage: SEARCHTURN <offset_us> or SEARCHTURN RESET");
      return true;
    }
    setBluetoothSearchTurnOffset(offsetUs);
    return true;
  }

  if (commandHasPrefix(command, "FBASE")) {
    float requestedLeftBase = 0.0;
    float requestedRightBase = 0.0;

    if (!parseTwoFloatArguments(commandArgument(command, "FBASE"), requestedLeftBase, requestedRightBase)) {
      Serial2.println("ERROR usage: FBASE <left_us> <right_us> or FBASE RESET");
      return true;
    }

    setBluetoothForwardBase(requestedLeftBase, requestedRightBase);
    return true;
  }

  if (commandHasPrefix(command, "SPEED")) {
    float requestedSpeed = 0.0;

    if (!parseFloatArgument(commandArgument(command, "SPEED"), requestedSpeed)) {
      Serial2.println("ERROR usage: SPEED <ticks/s> or SPEED RESET");
      return true;
    }

    setBluetoothBaseSpeed(requestedSpeed);
    return true;
  }

  if (commandEquals(command, "ESCAPE ON")) {
    escapeBacktrackEnabled = true;
    Serial2.println(hasTrustedRearCoverage()
      ? "OK reverse repositioning enabled."
      : "OK reverse policy enabled; motion remains gated without trusted rear coverage.");
    sendBluetoothEvent("escape_set", "on");
    return true;
  }

  if (commandEquals(command, "ESCAPE OFF")) {
    escapeBacktrackEnabled = false;
    Serial2.println("OK reverse repositioning disabled. Geometric no-path will safe-stop only.");
    sendBluetoothEvent("escape_set", "off");
    return true;
  }

  if (commandEquals(command, "ESCAPE STATUS")) {
    Serial2.print("ESCAPE ");
    Serial2.println(escapeBacktrackEnabled ? "ON" : "OFF");
    return true;
  }

  if (commandHasPrefix(command, "ESCAPE")) {
    Serial2.println("ERROR usage: ESCAPE ON, ESCAPE OFF, or ESCAPE STATUS");
    return true;
  }

  if (commandEquals(command, "EMERGENCY ON")) {
    if (!navigationSetEmergencyScanEnabled(true)) {
      Serial2.println("ERROR EMERGENCY cannot change during an active navigation goal.");
      return true;
    }
    Serial2.println(hasTrustedRearCoverage()
      ? "OK ultimate scan/retry recovery enabled."
      : "OK emergency recovery policy enabled; motion remains gated without trusted rear coverage.");
    sendBluetoothEvent("emergency_set", "on");
    return true;
  }

  if (commandEquals(command, "EMERGENCY OFF")) {
    if (!navigationSetEmergencyScanEnabled(false)) {
      Serial2.println("ERROR EMERGENCY cannot change during an active navigation goal.");
      return true;
    }
    Serial2.println("OK ultimate scan/retry recovery disabled.");
    sendBluetoothEvent("emergency_set", "off");
    return true;
  }

  if (commandEquals(command, "EMERGENCY STATUS")) {
    Serial2.print("EMERGENCY ");
    Serial2.println(navigationIsEmergencyScanEnabled() ? "ON" : "OFF");
    return true;
  }

  if (commandHasPrefix(command, "EMERGENCY")) {
    Serial2.println("ERROR usage: EMERGENCY ON, EMERGENCY OFF, or EMERGENCY STATUS");
    return true;
  }

  return false;
}

static bool handleTestArmBluetoothCommand(const char* command) {
  // Arming/disarming TEST authority. Disarm cancels old goals so a previous
  // test cannot republish motion after one neutral cycle.
  if (commandEquals(command, "TEST ARM")) {
    revokeAndCancelAllMotion("test_arm_authority_transition");
    claimMotionAuthority(MOTION_AUTHORITY_TEST);
    bluetoothTestArmed = true;
    bluetoothManualArmed = false;
    bluetoothManualActive = false;
    robotRunEnabled = false;
    bluetoothAbortMotionRequested = false;
    stopMotors();
    Serial2.println("OK test motion armed. TEST DRIVE, TEST TURN, TEST GOTO, TEST AVOID, TEST ESCAPE, TEST SEARCH, and TEST HUNT can move the robot.");
    sendBluetoothEvent("test_arm", "manual");
    return true;
  }

  if (commandEquals(command, "TEST DISARM")) {
    revokeAndCancelAllMotion("test_disarmed");
    bluetoothTestArmed = false;
    bluetoothSideTestActive = false;
    bluetoothTurnPulseTestActive = false;
    bluetoothArcTestActive = false;
    // Disarming must end an already-running TEST GOTO/DRIVE/AVOID/TURN goal,
    // not merely reject the next command. Otherwise updateRobotController()
    // can re-publish an old navigation command after this one-cycle stop.
    robotRunEnabled = false;
    stopMotors();
    setRobotState(END_MATCH);
    Serial2.println("OK test motion disarmed.");
    sendBluetoothEvent("test_disarm", "manual");
    return true;
  }

  return false;
}

static bool handleManualBluetoothCommand(const char* command) {
  // Manual mode owns MOTION_AUTHORITY_MANUAL and requires a live stream of
  // DRIVE commands. It is separate from TEST and MISSION authority.
  if (commandEquals(command, "MANUAL ARM")) {
    revokeAndCancelAllMotion("manual_arm_authority_transition");
    claimMotionAuthority(MOTION_AUTHORITY_MANUAL);
    bluetoothManualArmed = true;
    bluetoothManualActive = false;
    bluetoothTestArmed = false;
    robotRunEnabled = false;
    bluetoothAbortMotionRequested = false;
    stopMotors();
    Serial2.println("OK manual drive armed. Use DRIVE <forward> <turn>.");
    sendBluetoothEvent("manual_arm", "manual");
    return true;
  }

  if (commandEquals(command, "MANUAL DISARM")) {
    revokeAndCancelAllMotion("manual_disarmed");
    bluetoothManualArmed = false;
    stopManualDrive();
    Serial2.println("OK manual drive disarmed.");
    sendBluetoothEvent("manual_disarm", "manual");
    return true;
  }

  if (commandHasPrefix(command, "DRIVE")) {
    float forwardPercent = 0.0;
    float turnPercent = 0.0;

    if (!parseTwoFloatArguments(commandArgument(command, "DRIVE"), forwardPercent, turnPercent)) {
      Serial2.println("ERROR usage: DRIVE <forward -100..100> <turn -100..100>");
      return true;
    }

    runManualDriveCommand(forwardPercent, turnPercent);
    return true;
  }

  return false;
}

// TEST commands taking exactly one float argument. Adding one is a table row;
// the parse, the usage error, and the dispatch are shared. Entries must not be
// prefixes of one another, since dispatch is prefix-matched.
struct SingleFloatTestCommand {
  const char* name;
  const char* usageArgument;
  void (*run)(float);
};

static const SingleFloatTestCommand SINGLE_FLOAT_TEST_COMMANDS[] = {
  {"TEST DRIVE", " <metres>", runBluetoothTestDrive},
  {"TEST AVOID", " <metres>", runBluetoothTestAvoid},
  {"TEST ESCAPE", " <metres>", runBluetoothTestEscape},
  {"TEST SIDE", " <seconds>", runBluetoothTestSide},
};

static const size_t SINGLE_FLOAT_TEST_COMMAND_COUNT =
  sizeof(SINGLE_FLOAT_TEST_COMMANDS) / sizeof(SINGLE_FLOAT_TEST_COMMANDS[0]);

static bool handleTestMotionBluetoothCommand(const char* command) {
  // TEST motion and diagnostics. Motion-producing commands call
  // requireBluetoothTestArm(); stationary diagnostics such as TEST FAN/OBJECT
  // and TEST SIDE do not move motors.
  if (commandHasPrefix(command, "TEST ARC")) {
    float forwardTicksPerSec = 0.0;
    float turnTicksPerSec = 0.0;
    float durationSeconds = 0.0;
    if (!parseThreeFloatArguments(commandArgument(command, "TEST ARC"),
                                  forwardTicksPerSec, turnTicksPerSec,
                                  durationSeconds)) {
      Serial2.println("ERROR usage: TEST ARC <forward_tps> <turn_tps> <seconds>");
      return true;
    }
    runBluetoothTestArc(forwardTicksPerSec, turnTicksPerSec, durationSeconds);
    return true;
  }

  // Every single-argument TEST command parses identically and differs only in
  // its usage text and handler, so they are dispatched from one table.
  for (size_t i = 0; i < SINGLE_FLOAT_TEST_COMMAND_COUNT; i++) {
    const SingleFloatTestCommand &entry = SINGLE_FLOAT_TEST_COMMANDS[i];
    if (!commandHasPrefix(command, entry.name)) {
      continue;
    }
    float value = 0.0;
    if (!parseFloatArgument(commandArgument(command, entry.name), value)) {
      Serial2.print("ERROR usage: ");
      Serial2.print(entry.name);
      Serial2.println(entry.usageArgument);
      return true;
    }
    entry.run(value);
    return true;
  }

  if (commandHasPrefix(command, "TEST GOTO")) {
    float targetX = 0.0;
    float targetY = 0.0;

    if (!parseTwoFloatArguments(commandArgument(command, "TEST GOTO"), targetX, targetY)) {
      Serial2.println("ERROR usage: TEST GOTO <x_m> <y_m>");
      return true;
    }

    runBluetoothTestGoto(targetX, targetY);
    return true;
  }

  if (commandEquals(command, "TEST FAN") || commandEquals(command, "FAN")) {
    printFanTelemetry();
    return true;
  }

  if (commandEquals(command, "TEST MATRIX") || commandEquals(command, "MATRIX") ||
      commandEquals(command, "TEST OBJECT") || commandEquals(command, "OBJECT")) {
    printMatrixTelemetry();
    return true;
  }

  if (commandEquals(command, "TEST REAR") || commandEquals(command, "REAR")) {
    printRearTofStatus();
    return true;
  }

  if (commandEquals(command, "TEST PAYLOAD") ||
      commandEquals(command, "PAYLOAD")) {
    printPayloadTofStatus();
    PickupTrackingStatus pickup = navigationGetPickupTrackingStatus();
    Serial2.print("payload_capture,attempt=");
    Serial2.print(pickup.captureAttemptId);
    Serial2.print(",evidence=");
    Serial2.println((int)pickup.payloadEvidence);
    return true;
  }

  if (commandEquals(command, "TEST HUNT TARGET")) {
    printMatrixHuntTarget();
    return true;
  }

  if (commandEquals(command, "TEST HUNT")) {
    runBluetoothTestHunt();
    return true;
  }

  if (commandEquals(command, "TEST FOLLOW START")) {
    runBluetoothMatrixFollowStart();
    return true;
  }

  if (commandEquals(command, "TEST FOLLOW STOP")) {
    stopMatrixFollowDiagnostic("operator_stop");
    printMatrixFollowStatus();
    return true;
  }

  if (commandEquals(command, "TEST FOLLOW STATUS")) {
    printMatrixFollowStatus();
    return true;
  }

  if (commandHasPrefix(command, "TEST FOLLOW")) {
    Serial2.println("ERROR usage: TEST FOLLOW START|STOP|STATUS");
    return true;
  }

  if (commandHasPrefix(command, "TEST HUNT")) {
    Serial2.println("ERROR usage: TEST HUNT or TEST HUNT TARGET");
    return true;
  }

  if (commandEquals(command, "TEST SEARCH")) {
    runBluetoothTestSearch();
    return true;
  }

  if (commandHasPrefix(command, "TEST SEARCH")) {
    Serial2.println("ERROR usage: TEST SEARCH");
    return true;
  }

  if (commandHasPrefix(command, "TEST TURNPULSE")) {
    float signedSeconds = 0.0;

    if (!parseFloatArgument(commandArgument(command, "TEST TURNPULSE"), signedSeconds)) {
      Serial2.println("ERROR usage: TEST TURNPULSE <signed_seconds>");
      return true;
    }

    runBluetoothTestTurnPulse(signedSeconds);
    return true;
  }

  if (commandHasPrefix(command, "TEST TURNLADDER")) {
    runBluetoothTestTurnLadder(commandArgument(command, "TEST TURNLADDER"));
    return true;
  }

  if (commandHasPrefix(command, "TEST TURN")) {
    float angleDeg = 0.0;

    if (!parseFloatArgument(commandArgument(command, "TEST TURN"), angleDeg)) {
      Serial2.println("ERROR usage: TEST TURN <degrees>");
      return true;
    }

    runBluetoothTestTurn(angleDeg);
    return true;
  }

  return false;
}

static void handleBluetoothCommandLine(char* command) {
  // Routes a complete line through focused handler groups. The order matters:
  // more specific command families should claim their commands before the
  // unknown-command fallback.
  trimCommand(command);

  if (command[0] == '\0') {
    return;
  }

  if (handleCoreBluetoothCommand(command) ||
      handleStreamAndLogBluetoothCommand(command) ||
      handleTuningBluetoothCommand(command) ||
      handleTestArmBluetoothCommand(command) ||
      handleManualBluetoothCommand(command) ||
      handleTestMotionBluetoothCommand(command)) {
    return;
  }

  Serial2.print("ERROR unknown command: ");
  Serial2.println(command);
  Serial2.println("Type HELP for commands.");
}

bool handleBluetoothCommands() {
  // Reads available Serial2 bytes into a fixed command buffer and dispatches
  // complete CR/LF-terminated lines. Also advances nonblocking test harnesses
  // such as TEST SIDE, TEST ARC, TEST TURNPULSE, and turn truth logging.
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

  updateBluetoothTestSide();
  updateBluetoothTurnPulseTest();
  updateBluetoothArcTest();
  updateBluetoothTurnTruth();
  updateBluetoothNavigationTest();
  return bluetoothAbortMotionRequested;
}
