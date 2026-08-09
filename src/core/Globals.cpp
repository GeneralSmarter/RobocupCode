#include "../../Robot.h"

// =====================================================
// Shared hardware objects and runtime state definitions
// =====================================================
// Responsibility:
//   Allocates the storage for the globals declared in Robot.h. This is the
//   one place where shared state actually exists; other files should treat
//   these variables as subsystem-owned data, not as convenient scratch space.
// Interacts with:
//   All modules read or update some of this state. The main ownership pattern
//   is: Encoders.cpp owns raw counts, TofSensors.cpp owns rangeSensors and
//   fan, rear-array, and front-matrix sensor state,
//   Odometry.cpp owns robotX/robotY/robotTheta updates; Navigation owns
//   navigationGoal/plannerTelemetry, MotorControl.cpp owns desired command and
//   motor authority/output diagnostics, and RouteMission.cpp owns route
//   state/waypoint progress.
// Control flow:
//   No functions run here. Initial values define the boot-time safe state:
//   stopped motors, no motion authority, no active navigation goal, and the
//   robot waiting in INIT until START or a test command changes state.
// Global state:
//   Everything in this file is global by design. Changing initial values can
//   change startup behavior, telemetry, or safety assumptions.

Servo leftMotor;
Servo rightMotor;

// Raw quadrature counts are changed inside interrupt service routines. They
// are signed later by readEncoderCounts() so the rest of the firmware sees
// forward-positive wheel ticks.
volatile long leftRawCount  = 0;
volatile long rightRawCount = 0;

// Encoder snapshots for two different consumers: MotorControl.cpp uses
// lastLeft/RightCount for wheel speed PID, while Odometry.cpp uses
// lastOdomLeft/RightCount to integrate pose.
long lastLeftCount  = 0;
long lastRightCount = 0;

long lastOdomLeftCount  = 0;
long lastOdomRightCount = 0;

int leftForwardBaseUs = LEFT_BASE_US;
int rightForwardBaseUs = RIGHT_BASE_US;
int weightScanTurnOffsetUs = DEFAULT_WEIGHT_SCAN_TURN_OFFSET_US;

float robotX = 0.0;
float robotY = 0.0;
float robotTheta = 0.0;

// Wheel-speed commands are encoder ticks per second. MotorControl.cpp converts
// them to servo microseconds using feed-forward plus PID correction.
float baseTargetSpeed = DEFAULT_BASE_TARGET_SPEED;  // encoder ticks per second

float Kp = 0.025;
float Ki = 0.004;
float Kd = 0.000;

float leftIntegral  = 0.0;
float rightIntegral = 0.0;

float lastLeftError  = 0.0;
float lastRightError = 0.0;

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

float yawOffset = 0.0;
float latestImuPitchDeg = 0.0f;
float latestImuRollDeg = 0.0f;

SX1509 io;

VL53L0X rightOuterTOF;
VL53L0X rightInnerTOF;
VL53L0X leftInnerTOF;
VL53L0X leftOuterTOF;
VL53L1X rearTofs[REAR_TOF_COUNT];

RearTofState rearTofStates[REAR_TOF_COUNT] = {
  {false, false, true, true, RANGE_NO_READING_MM,
   SENSOR_RANGE_STATUS_UNKNOWN, 0.0f, 0.0f, 0, 0, 0, 0},
  {false, false, true, true, RANGE_NO_READING_MM,
   SENSOR_RANGE_STATUS_UNKNOWN, 0.0f, 0.0f, 0, 0, 0, 0},
  {false, false, true, true, RANGE_NO_READING_MM,
   SENSOR_RANGE_STATUS_UNKNOWN, 0.0f, 0.0f, 0, 0, 0, 0}
};

FrontMatrixFrame frontMatrixFrame = {};
MatrixTargetObservation matrixTargetObservation = {
  false, false, 0, 0, 0, 0, MATRIX_EVIDENCE_NONE,
  0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 3.5f, 0.0f,
  0.0f, 0.0f
};

RangeSensorState rangeSensors[RANGE_SENSOR_COUNT] = {
  // Physical fan sensors are listed right-to-left to match RANGE_* enum
  // indices and FAN_SENSOR_GEOMETRY. The aggregate front/right/left entries
  // are derived by TofSensors.cpp and should not be treated as hardware.
  {"right_outer", (int)FAN_SENSOR_GEOMETRY[RANGE_RIGHT_OUTER].angleDeg, RANGE_NO_READING_MM, false, false, false, 0, 0, 0},
  {"right_inner", (int)FAN_SENSOR_GEOMETRY[RANGE_RIGHT_INNER].angleDeg, RANGE_NO_READING_MM, false, false, false, 0, 0, 0},
  {"left_inner", (int)FAN_SENSOR_GEOMETRY[RANGE_LEFT_INNER].angleDeg, RANGE_NO_READING_MM, false, false, false, 0, 0, 0},
  {"left_outer", (int)FAN_SENSOR_GEOMETRY[RANGE_LEFT_OUTER].angleDeg, RANGE_NO_READING_MM, false, false, false, 0, 0, 0},
  {"front_virtual", 0, RANGE_NO_READING_MM, false, false, false, 0, 0, 0},
  {"right_fan", -30, RANGE_NO_READING_MM, false, false, false, 0, 0, 0},
  {"left_fan", 30, RANGE_NO_READING_MM, false, false, false, 0, 0, 0},
  {"rear_aggregate", 180, RANGE_NO_READING_MM, false, true, true, 0, 0, 0},
  {"front_matrix_aggregate", 0, RANGE_NO_READING_MM, false, true, true, 0, 0, 0}
};

bool frontBlocked = false;
uint16_t frontDistance = RANGE_NO_READING_MM;
uint16_t leftDistance  = RANGE_NO_READING_MM;
uint16_t rightDistance = RANGE_NO_READING_MM;

bool frontTofValid = false;
bool leftTofValid = false;
bool rightTofValid = false;

int frontBlockCounter = 0;
int frontClearCounter = 0;

bool driveStuck = false;
bool wheelMismatchStuck = false;
bool turnStuck = false;

unsigned long driveStuckStartMs = 0;
unsigned long wheelMismatchStartMs = 0;
unsigned long turnCheckStartMs = 0;
float turnCheckStartYaw = 0.0;

bool returnHomeRequested = false;
bool escapeBacktrackEnabled = true;
bool robotRunEnabled = false;
bool bluetoothOutputEnabled = false;

RobotSerialClass robotSerial;

RobotState currentState = INIT;

bool endMatchPrinted = false;

float desiredForwardSpeed = 0.0;
float desiredTurnSpeed = 0.0;
float lastRequestedLeftWheelSpeed = 0.0;
float lastRequestedRightWheelSpeed = 0.0;
float lastMeasuredLeftWheelSpeed = 0.0;
float lastMeasuredRightWheelSpeed = 0.0;
float lastImuClockwiseYawDeg = 0.0;
float lastNavigationHeadingDeg = 0.0;
const char* lastMotorOutputMode = "neutral";

PlannerTelemetry plannerTelemetry = {
  // Safe neutral telemetry defaults. String pointers are static literals used
  // by STATUS/CSV; do not assign pointers to stack buffers here.
  0.0,
  0.0,
  0.0,
  -1.0,
  0.0,
  0.0,
  0.0,
  0.0,
  0.0,
  0.0,
  0.0,
  0.0,
  0.0,
  0.0,
  0.0,
  0.0,
  0,
  0,
  false,
  0,
  PLANNER_STOP_NONE,
  "idle",
  "idle",
  "",
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  false
};

bool motorStopRequested = true;
MotionAuthority motionAuthority = MOTION_AUTHORITY_NONE;
MotionAuthority motionCommandAuthority = MOTION_AUTHORITY_NONE;
MotionCommandMode motionCommandMode = MOTION_COMMAND_STANDARD;
unsigned long lastSensorUpdateMs = 0;
unsigned long lastOdometryUpdateMs = 0;
unsigned long lastPlannerUpdateMs = 0;
unsigned long lastMotorControlUpdateMs = 0;

int lastLeftMotorUs = STOP_US;
int lastRightMotorUs = STOP_US;

