#include "Robot.h"

Servo leftMotor;
Servo rightMotor;

volatile long leftRawCount  = 0;
volatile long rightRawCount = 0;

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

NavigationGoal navigationGoal = {
  NAV_GOAL_NONE,
  NAV_OWNER_ROUTE,
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
  0.0,
  0.0,
  0.0,
  -1.0,
  0.0,
  0.0,
  0,
  PLANNER_STOP_NONE,
  "idle",
  "idle",
  "",
  0
};

bool motorStopRequested = true;
unsigned long lastSensorUpdateMs = 0;
unsigned long lastOdometryUpdateMs = 0;
unsigned long lastPlannerUpdateMs = 0;
unsigned long lastMotorControlUpdateMs = 0;

int lastLeftMotorUs = STOP_US;
int lastRightMotorUs = STOP_US;

Waypoint path[] = {
  {1.20, 0.00, "PAUSE"},
  {1.20, 0.80, "PAUSE"},
  {0.00, 0.80, "PAUSE"},
  {0.00, 0.00, "HOME"}
};

extern const int NUM_POINTS = sizeof(path) / sizeof(path[0]);
