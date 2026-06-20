#ifndef ROBOT_H
#define ROBOT_H

#include <Arduino.h>
#include <Servo.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <VL53L1X.h>
#include <VL53L0X.h>
#include <SparkFunSX1509.h>

// =====================================================
// Debug
// =====================================================
const bool DEBUG_DRIVE = true;
const bool DEBUG_TURN  = false;

const char ROBOT_BUILD_LABEL[] = "V6.2b-direct-steering";

// =====================================================
// Bluetooth serial debug link
// =====================================================
const unsigned long BLUETOOTH_BAUD = 115200;

extern bool bluetoothOutputEnabled;
extern bool robotRunEnabled;

class RobotSerialClass {
public:
  void begin(unsigned long baud) {
    ::Serial.begin(baud);
  }

  template <typename... Args>
  size_t print(Args... args) {
    size_t written = ::Serial.print(args...);
    if (bluetoothOutputEnabled) {
      Serial2.print(args...);
    }
    return written;
  }

  template <typename... Args>
  size_t println(Args... args) {
    size_t written = ::Serial.println(args...);
    if (bluetoothOutputEnabled) {
      Serial2.println(args...);
    }
    return written;
  }

  size_t println() {
    size_t written = ::Serial.println();
    if (bluetoothOutputEnabled) {
      Serial2.println();
    }
    return written;
  }

  void flush() {
    ::Serial.flush();
    if (bluetoothOutputEnabled) {
      Serial2.flush();
    }
  }
};

extern RobotSerialClass robotSerial;
#define Serial robotSerial

// =====================================================
// Motors
// =====================================================
extern Servo leftMotor;
extern Servo rightMotor;

const int LEFT_MOTOR_PIN  = 0;
const int RIGHT_MOTOR_PIN = 1;

const int STOP_US = 1500;
const int MIN_US  = 1000;
const int MAX_US  = 1993;

const int LEFT_BASE_US  = 1885;
const int RIGHT_BASE_US = 1890;

const int LEFT_REVERSE_US  = 1130;
const int RIGHT_REVERSE_US = 1190;

extern int leftForwardBaseUs;
extern int rightForwardBaseUs;

// =====================================================
// Encoders
// =====================================================
const int LEFT_ENC_A  = 2;
const int LEFT_ENC_B  = 3;
const int RIGHT_ENC_A = 4;
const int RIGHT_ENC_B = 5;

const int LEFT_ENCODER_SIGN  = -1;
const int RIGHT_ENCODER_SIGN = 1;

extern volatile long leftRawCount;
extern volatile long rightRawCount;

extern long lastLeftCount;
extern long lastRightCount;

extern long lastOdomLeftCount;
extern long lastOdomRightCount;

const float TICKS_PER_METRE = 8943.20;

// =====================================================
// Pose
// =====================================================
extern float robotX;
extern float robotY;
extern float robotTheta;

// =====================================================
// Wheel speed PID
// =====================================================
extern float baseTargetSpeed;

const float DEFAULT_BASE_TARGET_SPEED = 2300.0;

extern float Kp;
extern float Ki;
extern float Kd;

extern float leftIntegral;
extern float rightIntegral;

extern float lastLeftError;
extern float lastRightError;

// =====================================================
// IMU
// =====================================================
extern Adafruit_BNO055 bno;

extern float yawOffset;
extern float K_heading;

const float DEFAULT_HEADING_GAIN = 10.0;

// =====================================================
// TOF sensors
// =====================================================
extern SX1509 io;

extern VL53L0X rightOuterTOF;
extern VL53L1X rightInnerTOF;
extern VL53L1X leftInnerTOF;
extern VL53L0X leftOuterTOF;

const byte SX1509_ADDRESS = 0x3F;

// High fan XSHUT mapping, physically numbered right-to-left on the robot.
const byte RIGHT_OUTER_XSHUT = 0;  // -60 deg, VL53L0X
const byte RIGHT_INNER_XSHUT = 1;  // -20 deg, VL53L1X
const byte LEFT_INNER_XSHUT  = 2;  // +20 deg, VL53L1X
const byte LEFT_OUTER_XSHUT  = 3;  // +60 deg, VL53L0X

const uint8_t RIGHT_OUTER_ADDRESS = 0x30;
const uint8_t RIGHT_INNER_ADDRESS = 0x31;
const uint8_t LEFT_INNER_ADDRESS  = 0x32;
const uint8_t LEFT_OUTER_ADDRESS  = 0x33;

const int FRONT_STOP_DISTANCE_MM  = 180;
const int FRONT_CLEAR_DISTANCE_MM = 230;

const int FRONT_VALID_MIN_MM = 20;
const int FRONT_VALID_MAX_MM = 4000;
const unsigned long TOF_STALE_TIMEOUT_MS = 750;

const int FRONT_BLOCK_CONFIRM_READS = 2;
const int FRONT_CLEAR_CONFIRM_READS = 3;
const unsigned long FRONT_CLEAR_SETTLE_TIMEOUT_MS = 500;

enum RangeSensorId {
  RANGE_RIGHT_OUTER,
  RANGE_RIGHT_INNER,
  RANGE_LEFT_INNER,
  RANGE_LEFT_OUTER,
  RANGE_FRONT,
  RANGE_RIGHT,
  RANGE_LEFT,
  RANGE_SENSOR_COUNT
};

// Robot-centred geometry in millimetres. The origin is the midpoint between
// the drive wheels: +X forward, +Y left. Keep future physical measurements in
// this one block so clearance logic stays relative to the chassis.
struct RobotFootprintGeometry {
  float frontExtentMm;
  float rearExtentMm;
  float leftExtentMm;
  float rightExtentMm;
};

struct FanSensorGeometry {
  float xMm;
  float yMm;
  float angleDeg;
};

const RobotFootprintGeometry ROBOT_FOOTPRINT_GEOMETRY = {
  185.0,  // front extent
  160.0,  // rear extent
  145.0,  // left extent
  145.0   // right extent
};

const FanSensorGeometry FAN_SENSOR_GEOMETRY[4] = {
  {110.0, -95.0, -60.0},  // right outer
  {150.0, -40.0, -20.0},  // right inner
  {150.0,  40.0,  20.0},  // left inner
  {110.0,  95.0,  60.0}   // left outer
};

struct RangeSensorState {
  const char* name;
  int angleDeg;
  uint16_t distanceMm;
  bool valid;
  bool stale;
  bool blocked;
  unsigned long lastReadMs;
  unsigned long timeoutCount;
  unsigned long invalidCount;
};

extern RangeSensorState rangeSensors[RANGE_SENSOR_COUNT];

extern bool frontBlocked;
extern uint16_t frontDistance;
extern uint16_t leftDistance;
extern uint16_t rightDistance;

extern bool frontTofValid;
extern bool leftTofValid;
extern bool rightTofValid;

extern unsigned long lastFrontTofReadMs;
extern unsigned long lastLeftTofReadMs;
extern unsigned long lastRightTofReadMs;

extern int frontBlockCounter;
extern int frontClearCounter;
extern unsigned long lastFrontInvalidPrintMs;

// =====================================================
// Obstacle avoidance
// =====================================================
const float AVOID_CLEARANCE_MARGIN_MM = 50.0;
const float AVOID_SCORE_TIE_MARGIN_MM = 80.0;

const float AVOID_REVERSE_DISTANCE_M = 0.10;
const float AVOID_TURN_ANGLE_DEG     = 60.0;
// A clearance check uses each fan ray against the actual circular turn
// envelope of the chassis. Keep this tied to the normal footprint margin so
// geometry changes remain relative to the robot, not a particular course.
const float AVOID_DIAGONAL_WARNING_CLEARANCE_MM = AVOID_CLEARANCE_MARGIN_MM;
const float AVOID_ESCAPE_REVERSE_STEP_M = 0.05;
const float AVOID_ESCAPE_TURN_STEP_DEG = 10.0;
const int AVOID_ESCAPE_MAX_CLEARANCE_ATTEMPTS = 2;
const float AVOID_MIN_BYPASS_DISTANCE_M = 0.25;
const float AVOID_MAX_BYPASS_DISTANCE_M = 0.55;
const float AVOID_TARGET_BYPASS_FRACTION = 0.50;
const float AVOID_REJOIN_HANDOFF_MAX_ERROR_DEG = 30.0;

const unsigned long AVOID_TIMEOUT_MS = 7000;

enum AvoidTurnChoice {
  AVOID_TURN_LEFT,
  AVOID_TURN_RIGHT,
  AVOID_TURN_NONE
};

struct AvoidSideClearance {
  bool valid;
  bool passable;
  float innerSweepClearanceMm;
  float outerSweepClearanceMm;
  float scoreMm;
};

// =====================================================
// Side wall following fallback
// =====================================================
const bool ENABLE_SIDE_WALL_FOLLOW_FALLBACK = true;

const int WALL_FOLLOW_TARGET_MM    = 280;
const int WALL_FOLLOW_VALID_MIN_MM = 120;
const int WALL_FOLLOW_VALID_MAX_MM = 700;

const float WALL_FOLLOW_K = 0.8;
const float WALL_FOLLOW_MAX_CORRECTION = 300.0;

const unsigned long WALL_FOLLOW_TIMEOUT_MS = 2500;

enum WallFollowSide {
  FOLLOW_NO_WALL,
  FOLLOW_LEFT_WALL,
  FOLLOW_RIGHT_WALL
};

// =====================================================
// Stuck detection
// =====================================================
const float STUCK_COMMAND_SPEED_MIN = 1000.0;
const float STUCK_ENCODER_SPEED_MIN = 180.0;
const unsigned long DRIVE_STUCK_TIME_MS = 900;

const float WHEEL_MISMATCH_SPEED_MIN = 700.0;
const float WHEEL_MISMATCH_RATIO = 0.45;
const float WHEEL_MISMATCH_EXPECTED_RATIO = 0.70;
const unsigned long WHEEL_MISMATCH_TIME_MS = 900;

const float TURN_STUCK_YAW_MIN_DEG = 3.0;
const unsigned long TURN_STUCK_TIME_MS = 900;

extern bool driveStuck;
extern bool wheelMismatchStuck;
extern bool turnStuck;

extern unsigned long driveStuckStartMs;
extern unsigned long wheelMismatchStartMs;
extern unsigned long turnCheckStartMs;
extern float turnCheckStartYaw;

// =====================================================
// Stuck recovery
// =====================================================
const int MAX_STUCK_RECOVERIES = 3;

const float STUCK_REVERSE_DISTANCE_M = 0.12;
const float STUCK_FORWARD_DISTANCE_M = 0.20;
const float STUCK_TURN_ANGLE_DEG = 45.0;

extern int stuckRecoveryCount;
extern bool stoppedSafely;
extern bool inRecovery;
extern bool returnHomeRequested;

// =====================================================
// Main state machine
// =====================================================
enum RobotState {
  INIT,
  FOLLOW_PATH,
  APPROACH_OBJECT,
  COLLECT_SORT,
  RETURN_HOME,
  UNLOAD,
  OBSTACLE_AVOID,
  STUCK_RECOVERY,
  END_MATCH
};

extern RobotState currentState;
extern RobotState previousState;

extern int currentWaypointIndex;
extern bool endMatchPrinted;

extern float desiredForwardSpeed;
extern float desiredTurnSpeed;

extern int lastLeftMotorUs;
extern int lastRightMotorUs;

// =====================================================
// Turn settings
// =====================================================
const float TURN_TOLERANCE_DEG = 3.0;
const float SLOW_ZONE_DEG = 30.0;

const int TURN_RIGHT_LEFT_FAST_US  = 1850;
const int TURN_RIGHT_RIGHT_FAST_US = 1150;

const int TURN_LEFT_LEFT_FAST_US  = 1150;
const int TURN_LEFT_RIGHT_FAST_US = 1850;

const int TURN_RIGHT_LEFT_SLOW_US  = 1800;
const int TURN_RIGHT_RIGHT_SLOW_US = 1200;

const int TURN_LEFT_LEFT_SLOW_US  = 1200;
const int TURN_LEFT_RIGHT_SLOW_US = 1800;

// =====================================================
// Waypoints
// =====================================================
const float WAYPOINT_TOLERANCE_M = 0.06;
const float WAYPOINT_LOOKAHEAD_M = 0.35;
const float WAYPOINT_FINAL_APPROACH_M = 0.15;
const float WAYPOINT_FINAL_SKIP_TURN_DEG = 15.0;
const float WAYPOINT_STEER_SLOW_START_DEG = 5.0;
const float WAYPOINT_PIVOT_TURN_DEG = 40.0;
const float WAYPOINT_MIN_FORWARD_SCALE = 0.20;
const float WAYPOINT_FINAL_MIN_FORWARD_SCALE = 0.40;
const float WAYPOINT_STEER_GAIN_MULTIPLIER = 2.2;
const float WAYPOINT_MAX_TURN_CORRECTION = 1400.0;
const float WAYPOINT_TURN_US_PER_DEG = 8.0;
const int WAYPOINT_MAX_TURN_US = 320;
const unsigned long WAYPOINT_ACTION_PAUSE_MS = 250;
const unsigned long WAYPOINT_HOME_PAUSE_MS = 250;
const unsigned long WAYPOINT_TURN_SETTLE_MS = 25;
const unsigned long TURN_SETTLE_MS = 75;
const unsigned long DRIVE_COMPLETE_SETTLE_MS = 50;

struct Waypoint {
  float x;
  float y;
  const char* action;
};

extern Waypoint path[];
extern const int NUM_POINTS;

// =====================================================
// Timing
// =====================================================
extern unsigned long lastTime;

// =====================================================
// Function declarations
// =====================================================
void leftISR();
void rightISR();

void goToPoint(float targetX, float targetY);
void runWaypointAction(const char* action);

void runObstacleAvoidance(float targetX, float targetY,
                          const char* trigger = "front_blocked");
AvoidTurnChoice chooseAvoidTurnDirection();
AvoidTurnChoice evaluateAvoidTurnDirection(AvoidSideClearance &left,
                                           AvoidSideClearance &right,
                                           const char* &reason);
void reverseDistanceOpenLoop(float distanceMetres);

bool isValidWallFollowReading(uint16_t distanceMm);
WallFollowSide chooseWallFollowSide();
float getWallFollowCorrection(WallFollowSide side);
void driveDistanceWithHeadingWallFallback(float distanceMetres, float targetHeadingDeg);

void driveDistanceWithHeading(float distanceMetres, float targetHeadingDeg);
void driveDistanceWithHeadingNoAvoid(float distanceMetres, float targetHeadingDeg);

void runStuckRecovery();
void updateStuckDriving(float leftTargetSpeed, float rightTargetSpeed,
                        float leftSpeed, float rightSpeed);
void resetTurnStuckCheck(float startYaw);
void updateStuckTurning(float currentYaw);
void clearStuckFlags();

void turnAngle(float relativeTurnDeg);
void turnAngleNoStuckCheck(float relativeTurnDeg);
void turnRightFast();
void turnRightSlow();
void turnLeftFast();
void turnLeftSlow();

void updateOdometry();

void connectIMU();
void zeroYaw();
float readYawDeg();

void connectTOFSensors();
void connectRightOuterTOF();
void connectRightInnerTOF();
void connectLeftInnerTOF();
void connectLeftOuterTOF();
void updateTOFSensors();
void updateFanTOFSensors();
bool isRangeSensorValid(RangeSensorId id);
bool isRangeSensorBlocked(RangeSensorId id);
uint16_t getRangeSensorDistance(RangeSensorId id);
float getFanSweepClearanceMm(RangeSensorId id);
bool getDiagonalClearanceWarning(RangeSensorId &sensorId, float &clearanceMm);
bool waitForFrontClear(unsigned long timeoutMs);
void printFanTelemetry();

bool handleEmergencyStopPriority(const char* contextLabel);
bool handleStuckPriority();
bool handleObstacleAvoidPriority(float targetX, float targetY);
bool handleReturnHomePriority();
bool handleDrivePriorities(float targetX, float targetY);

void stopMotors();
void writeMotorUS(int leftUs, int rightUs);
int updatePID(float target, float actual, float &integral, float &lastError, float dt, int baseCommand);

float wrapAngle(float angle);
void resetEncodersAndPID();
void readEncoderCounts(long &leftCount, long &rightCount);
void printPose();
void printCalibrationSummary();

void setupBluetooth();
bool handleBluetoothCommands();
void sendBluetoothStatus();
void sendBluetoothTelemetry();
void sendBluetoothEvent(const char* eventName, const char* eventDetail);
bool isManualDriveActive();
void updateManualDriveTimeout();
void printWaitingForStart();

void runStateMachine();
void runInitState();
void runFollowPathState();
void runReturnHomeState();
void runUnusedState(const char* stateName);
void runEndMatchState();
void setRobotState(RobotState newState);
const char* robotStateName(RobotState state);
void setMotionCommand(float forwardSpeed, float turnSpeed);

#endif
