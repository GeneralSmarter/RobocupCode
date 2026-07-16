#include "Robot.h"

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
//   legacy ToF globals, ObjectDetection.cpp owns object candidate state,
//   Odometry.cpp owns robotX/robotY/robotTheta updates, LocalPlanner.cpp owns
//   navigationGoal/plannerTelemetry, MotorControl.cpp owns desired command and
//   motor authority/output diagnostics, and StateMachine.cpp owns mission
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

SX1509 io;

VL53L0X rightOuterTOF;
VL53L0X rightInnerTOF;
VL53L0X leftInnerTOF;
VL53L0X leftOuterTOF;
VL53L1X objectLeftLowTOF;
VL53L1X objectLeftUpperTOF;
VL53L1X objectRightLowTOF;
VL53L1X objectRightUpperTOF;

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
  {"fake_rear_tof", (int)FAKE_REAR_TOF_GEOMETRY.angleDeg, RANGE_NO_READING_MM, false, false, false, 0, 0, 0}
};

const ObjectSensorGeometry OBJECT_SENSOR_GEOMETRY[OBJECT_TOF_COUNT] = {
  // Object sensor geometry is robot-frame millimetres. The LOW/UPPER pairs
  // help distinguish short weight-sized objects from taller obstacles, but do
  // not provide navigation safety clearance.
  {91.4,  60.6, 55.0, -20.0, 0.0, OBJECT_ROLE_LOW},    // object_left_low, XSHUT7
  {91.4,  60.6, 120.0, -20.0, 0.0, OBJECT_ROLE_UPPER}, // object_left_upper, XSHUT5
  {91.4, -60.6, 55.0,  20.0, 0.0, OBJECT_ROLE_LOW},    // object_right_low, XSHUT6
  {91.4, -60.6, 120.0,  20.0, 0.0, OBJECT_ROLE_UPPER}  // object_right_upper, XSHUT4
};

ObjectSensorState objectSensors[OBJECT_TOF_COUNT] = {
  {"object_left_low", OBJECT_ROLE_LOW, OBJECT_NO_READING_MM, false, false, false, 0, 0, 0, SENSOR_RANGE_STATUS_UNKNOWN, 0.0, 0.0},
  {"object_left_upper", OBJECT_ROLE_UPPER, OBJECT_NO_READING_MM, false, false, false, 0, 0, 0, SENSOR_RANGE_STATUS_UNKNOWN, 0.0, 0.0},
  {"object_right_low", OBJECT_ROLE_LOW, OBJECT_NO_READING_MM, false, false, false, 0, 0, 0, SENSOR_RANGE_STATUS_UNKNOWN, 0.0, 0.0},
  {"object_right_upper", OBJECT_ROLE_UPPER, OBJECT_NO_READING_MM, false, false, false, 0, 0, 0, SENSOR_RANGE_STATUS_UNKNOWN, 0.0, 0.0}
};

ObjectCandidateState objectCandidate = {
  OBJECT_CANDIDATE_DISABLED,
  "object_tof_disabled",
  false,
  0,
  OBJECT_NO_READING_MM,
  0,
  0
};

ObjectTargetEstimate objectTargetEstimate = {
  false,
  0.0,
  0.0,
  0.0,
  0.0,
  OBJECT_NO_READING_MM,
  0,
  "not_estimated",
  0
};

bool frontBlocked = false;
uint16_t frontDistance = RANGE_NO_READING_MM;
uint16_t leftDistance  = RANGE_NO_READING_MM;
uint16_t rightDistance = RANGE_NO_READING_MM;

bool frontTofValid = false;
bool leftTofValid = false;
bool rightTofValid = false;

unsigned long lastFrontTofReadMs = 0;
unsigned long lastLeftTofReadMs = 0;
unsigned long lastRightTofReadMs = 0;

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
RobotState previousState = INIT;

int currentWaypointIndex = 0;
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

NavigationGoal navigationGoal = {
  // Idle goal. startNavigationPoint() and startNavigationTurn() fill every
  // field when a mission/test command claims motion authority.
  NAV_GOAL_NONE,
  NAV_OWNER_ROUTE,
  MOTION_AUTHORITY_NONE,
  false,
  false,
  false,
  0.0,
  0.0,
  0.0,
  0.0,
  0.0,
  0.0,
  0
};

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
  0,
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
unsigned long lastSensorUpdateMs = 0;
unsigned long lastOdometryUpdateMs = 0;
unsigned long lastPlannerUpdateMs = 0;
unsigned long lastMotorControlUpdateMs = 0;

int lastLeftMotorUs = STOP_US;
int lastRightMotorUs = STOP_US;

Waypoint path[] = {
  // Default calibration route in world metres. StateMachine.cpp assigns these
  // one at a time; LocalPlanner.cpp decides the safe short arcs between them.
  {1.20, 0.00, "PAUSE"},
  {1.20, 0.80, "PAUSE"},
  {0.00, 0.80, "PAUSE"},
  {0.00, 0.00, "HOME"}
};

extern const int NUM_POINTS = sizeof(path) / sizeof(path[0]);
