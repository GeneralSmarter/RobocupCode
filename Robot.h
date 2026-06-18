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

const int LEFT_BASE_US  = 1900;
const int RIGHT_BASE_US = 1860;

const int LEFT_REVERSE_US  = 1130;
const int RIGHT_REVERSE_US = 1190;

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

const float TICKS_PER_METRE = 9479.79;

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

// =====================================================
// TOF sensors
// =====================================================
extern SX1509 io;

extern VL53L1X frontTOF;
extern VL53L0X leftTOF;
extern VL53L0X rightTOF;

const byte SX1509_ADDRESS = 0x3F;

const byte FRONT_XSHUT = 0;
const byte LEFT_XSHUT  = 1;
const byte RIGHT_XSHUT = 2;

const uint8_t LEFT_L0_ADDRESS  = 0x30;
const uint8_t RIGHT_L0_ADDRESS = 0x31;

const int FRONT_STOP_DISTANCE_MM  = 180;
const int FRONT_CLEAR_DISTANCE_MM = 230;

const int FRONT_VALID_MIN_MM = 20;
const int FRONT_VALID_MAX_MM = 4000;
const unsigned long TOF_STALE_TIMEOUT_MS = 250;

const int FRONT_BLOCK_CONFIRM_READS = 2;
const int FRONT_CLEAR_CONFIRM_READS = 3;

enum RangeSensorId {
  RANGE_FRONT,
  RANGE_LEFT,
  RANGE_RIGHT,
  RANGE_SENSOR_COUNT
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
const int AVOID_OPEN_MARGIN_MM = 80;

const float AVOID_REVERSE_DISTANCE_M = 0.10;
const float AVOID_BYPASS_DISTANCE_M  = 0.40;
const float AVOID_REJOIN_DISTANCE_M  = 0.25;
const float AVOID_TURN_ANGLE_DEG     = 60.0;

const unsigned long AVOID_TIMEOUT_MS = 7000;

enum AvoidTurnChoice {
  AVOID_TURN_LEFT,
  AVOID_TURN_RIGHT
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

void runObstacleAvoidance(float originalPathHeadingDeg);
AvoidTurnChoice chooseAvoidTurnDirection();
void reverseDistanceOpenLoop(float distanceMetres);

bool isValidWallFollowReading(uint16_t distanceMm);
WallFollowSide chooseWallFollowSide();
float getWallFollowCorrection(WallFollowSide side);
void driveDistanceWithHeadingWallFallback(float distanceMetres, float targetHeadingDeg);

void driveDistanceWithHeading(float distanceMetres, float targetHeadingDeg);
void driveDistanceWithHeadingNoAvoid(float distanceMetres, float targetHeadingDeg);

void runStuckRecovery();
void updateStuckDriving(float targetSpeed, float leftSpeed, float rightSpeed);
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
void connectLeftTOF();
void connectRightTOF();
void connectFrontTOF();
void updateTOFSensors();
void updateFrontTOF();
void updateSideTOFSensors();
bool isRangeSensorValid(RangeSensorId id);
bool isRangeSensorBlocked(RangeSensorId id);
uint16_t getRangeSensorDistance(RangeSensorId id);

bool handleEmergencyStopPriority(const char* contextLabel);
bool handleStuckPriority();
bool handleObstacleAvoidPriority(float originalPathHeadingDeg);
bool handleReturnHomePriority();
bool handleDrivePriorities(float originalPathHeadingDeg);

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
