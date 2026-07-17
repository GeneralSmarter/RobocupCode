#ifndef ROBOT_H
#define ROBOT_H

#ifdef ROBOT_HOST_SIM
#include "HostSimRobot.h"
#else

// =====================================================
// Shared firmware interface and global state declarations
// =====================================================
// Responsibility:
//   Central include file for the active firmware. It gathers Arduino/library
//   includes, project types/configuration, extern declarations for shared
//   hardware/runtime globals, and function prototypes used across modules.
// Interacts with:
//   Every RobotCode module includes this file. Globals are defined in
//   Globals.cpp, while behavior is implemented by Bluetooth.cpp,
//   StateMachine.cpp, LocalPlanner.cpp, MotorControl.cpp, TofSensors.cpp,
//   Odometry.cpp, Imu.cpp, Encoders.cpp, ObjectDetection.cpp, and helpers.
// Control flow:
//   This file does not execute code, but it exposes the contracts that let
//   RobotCode.ino schedule the system and let modules call each other without
//   owning each other's internals.
// Global state:
//   Declares motors, encoders, pose, PID state, IMU, ToF sensors, navigation
//   goals, telemetry, motion authority, route waypoints, and timing stamps.
//   Units are documented beside the owning constants/types where possible:
//   pose in metres/degrees, ranges in millimetres, speeds in encoder ticks/s,
//   and motor commands in servo microseconds.

#include <Arduino.h>
#include <Servo.h>
#include <IntervalTimer.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <VL53L0X.h>
#include <VL53L1X.h>
#include <SparkFunSX1509.h>

#include "RobotTypes.h"
#include "RobotConfig.h"
#include "TurnConvention.h"
#include "MotionSafety.h"

// =====================================================
// Bluetooth serial debug link
// =====================================================
extern bool bluetoothOutputEnabled;
extern bool robotRunEnabled;
bool isBluetoothCsvStreamEnabled();

class RobotSerialClass {
public:
  // Mirrors normal Serial output to the Bluetooth link unless CSV streaming is
  // active. CSV mode uses Bluetooth.cpp's bounded queue so free-form debug
  // prints cannot split a machine-readable telemetry row.
  void begin(unsigned long baud) {
    ::Serial.begin(baud);
  }

  template <typename... Args>
  size_t print(Args... args) {
    size_t written = ::Serial.print(args...);
    if (bluetoothOutputEnabled && !isBluetoothCsvStreamEnabled()) {
      Serial2.print(args...);
    }
    return written;
  }

  template <typename... Args>
  size_t println(Args... args) {
    size_t written = ::Serial.println(args...);
    if (bluetoothOutputEnabled && !isBluetoothCsvStreamEnabled()) {
      Serial2.println(args...);
    }
    return written;
  }

  size_t println() {
    size_t written = ::Serial.println();
    if (bluetoothOutputEnabled && !isBluetoothCsvStreamEnabled()) {
      Serial2.println();
    }
    return written;
  }

  void flush() {
    ::Serial.flush();
    if (bluetoothOutputEnabled && !isBluetoothCsvStreamEnabled()) {
      Serial2.flush();
    }
  }
};

extern RobotSerialClass robotSerial;
#define Serial robotSerial

// =====================================================
// Hardware objects and runtime state
// =====================================================
extern Servo leftMotor;
extern Servo rightMotor;

// Volatile because the encoder interrupt service routines update these counts
// asynchronously while normal loop code reads them. Always use
// readEncoderCounts() rather than reading these directly.
extern volatile long leftRawCount;
extern volatile long rightRawCount;

// Encoder snapshots for speed/PID and odometry. These are intentionally
// separate so resetting a control segment does not erase odometry history.
extern long lastLeftCount;
extern long lastRightCount;

extern long lastOdomLeftCount;
extern long lastOdomRightCount;

extern int leftForwardBaseUs;
extern int rightForwardBaseUs;
extern int weightScanTurnOffsetUs;

extern float robotX;
extern float robotY;
// Navigation/world-frame heading. Robot coordinates are +X forward, +Y left,
// so positive heading is counter-clockwise. The BNO055's raw yaw convention is
// the opposite on this installation; use navigationHeadingDeg() for mapping,
// odometry, trajectory geometry, and direct turn control.
extern float robotTheta;

extern float baseTargetSpeed;

extern float Kp;
extern float Ki;
extern float Kd;

extern float leftIntegral;
extern float rightIntegral;

extern float lastLeftError;
extern float lastRightError;

extern Adafruit_BNO055 bno;
extern float yawOffset;

extern SX1509 io;

extern VL53L0X rightOuterTOF;
extern VL53L0X rightInnerTOF;
extern VL53L0X leftInnerTOF;
extern VL53L0X leftOuterTOF;
extern VL53L1X objectLeftLowTOF;
extern VL53L1X objectLeftUpperTOF;
extern VL53L1X objectRightLowTOF;
extern VL53L1X objectRightUpperTOF;

extern RangeSensorState rangeSensors[RANGE_SENSOR_COUNT];

extern const ObjectSensorGeometry OBJECT_SENSOR_GEOMETRY[OBJECT_TOF_COUNT];
extern ObjectSensorState objectSensors[OBJECT_TOF_COUNT];
extern ObjectCandidateState objectCandidate;
extern ObjectTargetEstimate objectTargetEstimate;

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

extern bool driveStuck;
extern bool wheelMismatchStuck;
extern bool turnStuck;

extern unsigned long driveStuckStartMs;
extern unsigned long wheelMismatchStartMs;
extern unsigned long turnCheckStartMs;
extern float turnCheckStartYaw;

extern bool returnHomeRequested;

extern RobotState currentState;
extern RobotState previousState;

extern int currentWaypointIndex;
extern bool endMatchPrinted;

extern float desiredForwardSpeed;
extern float desiredTurnSpeed;
// DEBUGGING TIP: requested/measured wheel speeds are ticks/s and are published
// in STATUS/CSV to prove the command-to-encoder chain.
extern float lastRequestedLeftWheelSpeed;
extern float lastRequestedRightWheelSpeed;
extern float lastMeasuredLeftWheelSpeed;
extern float lastMeasuredRightWheelSpeed;
extern float lastImuClockwiseYawDeg;
extern float lastNavigationHeadingDeg;
extern const char* lastMotorOutputMode;

extern int lastLeftMotorUs;
extern int lastRightMotorUs;

extern NavigationGoal navigationGoal;
extern PlannerTelemetry plannerTelemetry;
extern bool motorStopRequested;
// Motion authority is the high-level owner (mission/test/manual). The command
// authority records who last submitted the current desiredForward/Turn command.
extern MotionAuthority motionAuthority;
extern MotionAuthority motionCommandAuthority;
extern bool escapeBacktrackEnabled;
extern unsigned long lastSensorUpdateMs;
extern unsigned long lastOdometryUpdateMs;
extern unsigned long lastPlannerUpdateMs;
extern unsigned long lastMotorControlUpdateMs;

extern Waypoint path[];
extern const int NUM_POINTS;

// =====================================================
// Function declarations
// =====================================================
void leftISR();
void rightISR();

void goToPoint(float targetX, float targetY);
void runWaypointAction(const char* action);

void initializeNavigationController();
void updateRobotController();
void updateNavigationController();
void startNavigationPoint(float targetX, float targetY, NavigationGoalOwner owner);
void startNavigationTurn(float relativeTurnDeg, NavigationGoalOwner owner);
void cancelNavigationGoal(PlannerStopReason reason, const char* detail);
bool isNavigationGoalActive();
bool didNavigationGoalComplete();
bool didNavigationGoalFail();
void clearNavigationGoalResult();
const char* plannerStopReasonName(PlannerStopReason reason);
bool isTurnDirectionObservable(float turnTicksPerSec);
bool isTurnSweepSafe();
void clearLocalMap();
void updateLocalMapFromSensors();
void markTraversedFreeSpace();

AvoidTurnChoice evaluateAvoidTurnDirection(AvoidSideClearance &left,
                                           AvoidSideClearance &right,
                                           const char* &reason);

void updateStuckDriving(float leftTargetSpeed, float rightTargetSpeed,
                        float leftSpeed, float rightSpeed);
void resetTurnStuckCheck(float startYaw);
void updateStuckTurning(float currentYaw);
void clearStuckFlags();

void updateOdometry();

void connectIMU();
void zeroYaw();
float readImuClockwiseYawDeg();
float navigationHeadingDeg();

void connectTOFSensors();
void connectRightOuterTOF();
void connectRightInnerTOF();
void connectLeftInnerTOF();
void connectLeftOuterTOF();
void updateTOFSensors();
void updateFanTOFSensors();
void prepareObjectTOFPinsForStartup();
void connectObjectTOFSensors();
void updateObjectTOFSensors();
void refreshObjectTargetEstimate();
void printObjectTelemetry();
bool isObjectTargetFresh();
bool isRangeSensorValid(RangeSensorId id);
bool isRangeSensorBlocked(RangeSensorId id);
bool isRangeSensorCurrent(RangeSensorId id);
bool hasTrustedRearCoverage();
uint16_t getRangeSensorDistance(RangeSensorId id);
bool isTofCloseReadingRevalidating();
float getFanSweepClearanceMm(RangeSensorId id);
bool getDiagonalClearanceWarning(RangeSensorId &sensorId, float &clearanceMm);
void printFanTelemetry();
const char* objectCandidateKindName(ObjectCandidateKind kind);
const char* objectSensorRoleName(ObjectTofRole role);

void stopMotors();
void initializeMotorSafetyWatchdog();
void serviceMotorSafetyWatchdog();
void noteMainLoopHeartbeat();
bool isMotorSafetyWatchdogReady();
bool isMotorCommandLeaseArmed();
unsigned long motorCommandLeaseTripCount();
unsigned long currentMainLoopGapMs();
unsigned long maximumMainLoopGapMs();
unsigned long mainLoopDeadlineMissCount();
void recordMainLoopPhaseDuration(const char* phase, unsigned long startedUs);
const char* maximumMainLoopPhaseName();
unsigned long maximumMainLoopPhaseUs();
void resetMainLoopTimingDiagnostics();
void revokeMotionAuthority();
bool claimMotionAuthority(MotionAuthority authority);
bool setAuthorizedMotionCommand(MotionAuthority authority, float forwardSpeed, float turnSpeed);
const char* motionAuthorityName(MotionAuthority authority);
const char* motionSafetyReasonName(MotionSafetyReason reason);
MotionSafetyReason lastMotionSafetyReason();
bool isMotionSafetyStopActive();
int updatePID(float target, float actual, float &integral, float &lastError, float dt, int baseCommand);
void updateMotorController();

float wrapAngle(float angle);
void resetEncodersAndPID();
void readEncoderCounts(long &leftCount, long &rightCount);
void printPose();
void printCalibrationSummary();

void setupBluetooth();
bool handleBluetoothCommands();
void disarmBluetoothMotionModes();
void sendBluetoothStatus();
void sendBluetoothTelemetry();
void sendBluetoothEvent(const char* eventName, const char* eventDetail);
void serviceBluetoothTelemetryTx();
bool isManualDriveActive();
void updateManualDriveTimeout();
void printWaitingForStart();

void runStateMachine();
void runInitState();
void runFollowPathState();
void runReturnHomeState();
void runUnusedState(const char* stateName);
void runEndMatchState();
void startWeightSearchTest();
bool isWeightSearchActive();
void cancelWeightSearch(const char* detail);
void setRobotState(RobotState newState);
const char* robotStateName(RobotState state);
void setMotionCommand(float forwardSpeed, float turnSpeed);

#endif  // ROBOT_HOST_SIM
#endif  // ROBOT_H
