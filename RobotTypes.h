#ifndef ROBOT_TYPES_H
#define ROBOT_TYPES_H

// =====================================================
// Shared enums and lightweight data structures
// =====================================================
// Responsibility:
//   Defines the vocabulary used by the firmware: sensor identifiers, robot
//   states, navigation goal ownership, motion authority, telemetry payloads,
//   and simple geometry records.
// Interacts with:
//   Included through Robot.h by all modules. Bluetooth.cpp prints many of
//   these fields, navigation modules consume navigation and planner structs,
//   TofSensors.cpp fills RangeSensorState, ObjectDetection.cpp fills object
//   structs, and MotorControl.cpp enforces MotionAuthority.
// Control flow:
//   No executable runtime logic except small constexpr policy checks and
//   static_assert invariants for motion authority.
// Global state:
//   These are type definitions only. Instances live in Globals.cpp and are
//   read/modified by the subsystem that owns each behavior.

#include <Arduino.h>

// Physical fan sensors come first and match FAN_SENSOR_GEOMETRY indices.
// RANGE_FRONT/RANGE_LEFT/RANGE_RIGHT are derived aggregate views; there is no
// physical front-centre ToF in this layout. RANGE_FAKE_REAR is a temporary
// compatibility name for the physical rear matrix ToF range slot.
enum RangeSensorId {
  RANGE_RIGHT_OUTER,
  RANGE_RIGHT_INNER,
  RANGE_LEFT_INNER,
  RANGE_LEFT_OUTER,
  RANGE_FRONT,
  RANGE_RIGHT,
  RANGE_LEFT,
  RANGE_FAKE_REAR,
  RANGE_SENSOR_COUNT
};

struct RobotFootprintGeometry {
  // Millimetres from the drive-wheel midpoint to each chassis edge.
  float frontExtentMm;
  float rearExtentMm;
  float leftExtentMm;
  float rightExtentMm;
};

struct FanSensorGeometry {
  // Robot-frame mounting pose in millimetres/degrees. +X is forward, +Y left.
  float xMm;
  float yMm;
  float angleDeg;
};

struct RangeSensorState {
  // A ToF reading is more than a number: validity, staleness and blocked state
  // are kept separately so unknown or old evidence can fail closed.
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

enum ObjectTofId {
  OBJECT_LEFT_LOW,
  OBJECT_LEFT_UPPER,
  OBJECT_RIGHT_LOW,
  OBJECT_RIGHT_UPPER,
  OBJECT_TOF_COUNT
};

enum ObjectTofRole {
  OBJECT_ROLE_LOW,
  OBJECT_ROLE_UPPER
};

// Object detection is advisory for search/hunt behavior, not a safety input.
enum ObjectCandidateKind {
  OBJECT_CANDIDATE_DISABLED,
  OBJECT_CANDIDATE_NONE,
  OBJECT_CANDIDATE_UNKNOWN,
  OBJECT_CANDIDATE_WEIGHT_SIZED,
  OBJECT_CANDIDATE_TALL_OBSTACLE
};

struct ObjectSensorGeometry {
  float xMm;
  float yMm;
  float zMm;
  float yawDeg;
  float pitchDeg;
  ObjectTofRole role;
};

struct ObjectSensorState {
  const char* name;
  ObjectTofRole role;
  uint16_t distanceMm;
  bool valid;
  bool stale;
  bool connected;
  unsigned long lastReadMs;
  unsigned long timeoutCount;
  unsigned long invalidCount;
  uint8_t rangeStatus;
  float signalMcps;
  float ambientMcps;
};

struct ObjectCandidateState {
  ObjectCandidateKind kind;
  const char* reason;
  bool confirmed;
  int directionHint;
  uint16_t rangeMm;
  uint8_t confirmCount;
  unsigned long lastUpdateMs;
};

struct ObjectTargetEstimate {
  // robotXmm/robotYmm are in the robot body frame; worldX/worldY are the same
  // target transformed into the odometry frame at the time of estimation.
  bool valid;
  float robotXmm;
  float robotYmm;
  float worldX;
  float worldY;
  uint16_t rangeMm;
  uint8_t sourceMask;
  const char* reason;
  unsigned long lastUpdateMs;
};

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

// Keep the historical numeric values used by saved telemetry while exposing
// only states with implemented entry, exit, and failure behavior.
enum RobotState {
  INIT = 0,
  FOLLOW_PATH = 1,
  RETURN_HOME = 4,
  END_MATCH = 8
};

// Exactly one of these may own motion at a time. MotorControl.cpp enforces it
// at command acceptance and again at the final motor writer.
enum MotionAuthority {
  MOTION_AUTHORITY_NONE,
  MOTION_AUTHORITY_MISSION,
  MOTION_AUTHORITY_TEST,
  MOTION_AUTHORITY_MANUAL
};

// Tells the final motor writer how to realise an accepted chassis command.
// Navigation selects this explicitly so motor control never reads private
// navigation-goal state.
enum MotionCommandMode {
  MOTION_COMMAND_STANDARD,
  MOTION_COMMAND_NAV_DRIVE,
  MOTION_COMMAND_NAV_TURN,
  MOTION_COMMAND_NAV_SCAN_TURN
};

constexpr bool motionAuthorityAllows(MotionAuthority active,
                                     MotionAuthority claimant) {
  return active != MOTION_AUTHORITY_NONE && active == claimant;
}

static_assert(!motionAuthorityAllows(MOTION_AUTHORITY_NONE,
                                     MOTION_AUTHORITY_NONE),
              "Disarmed authority must never permit motion");
static_assert(motionAuthorityAllows(MOTION_AUTHORITY_MISSION,
                                    MOTION_AUTHORITY_MISSION),
              "Matching mission authority must be accepted");
static_assert(!motionAuthorityAllows(MOTION_AUTHORITY_MISSION,
                                     MOTION_AUTHORITY_TEST),
              "Mismatched motion authorities must be rejected");

enum PlannerStopReason {
  PLANNER_STOP_NONE = 0,
  PLANNER_STOP_FRONT_BLOCKED = 1,
  PLANNER_STOP_FRONT_INVALID = 2,
  PLANNER_STOP_NO_SAFE_TRAJECTORY = 3,
  PLANNER_STOP_TURN_SIDE_INVALID = 4,
  PLANNER_STOP_TURN_CLEARANCE = 5,
  PLANNER_STOP_STUCK = 6,
  PLANNER_STOP_RECOVERY_TIME = 9,
  PLANNER_STOP_RECOVERY_DISTANCE = 10,
  PLANNER_STOP_RECOVERY_REPEATED = 11,
  PLANNER_STOP_RECOVERY_NO_PROGRESS = 12,
  PLANNER_STOP_RECOVERY_NO_USEFUL_OUTCOME = 13,
  PLANNER_STOP_EMERGENCY_SCAN_ABORTED = 14,
  PLANNER_STOP_EMERGENCY_RETRY_EXHAUSTED = 15,
  PLANNER_STOP_ABORTED = 16
};

enum NavigationRunState {
  NAVIGATION_IDLE,
  NAVIGATION_RUNNING,
  NAVIGATION_REACHED,
  NAVIGATION_FAILED
};

struct NavigationStatus {
  NavigationRunState state;
  PlannerStopReason stopReason;
  const char* detail;
};

struct PlannerDebugSnapshot {
  int emergencyPhase;
  bool emergencyConsumed;
  uint8_t emergencySector;
  float emergencyScanYawDeg;
  float emergencyRelocationDistanceM;
  float emergencyRotationalClearanceM;
  int obstacleBypassPhase;
  float obstacleBypassSideSign;
  float obstacleNearAlongM;
  float obstacleFarAlongM;
  float obstacleMinLateralM;
  float obstacleMaxLateralM;
  float obstacleGoalX;
  float obstacleGoalY;
};

struct PlannerTelemetry {
  // Public planner state for STATUS/CSV. Speeds are ticks/s, distances are
  // metres unless suffixed with Mm, and timings are milliseconds/microseconds
  // as named. String pointers reference static literals, not owned buffers.
  float selectedForwardTicksPerSec;
  float selectedTurnTicksPerSec;
  float selectedCurvature;
  float minimumSweptClearanceMm;
  float speedCapTicksPerSec;
  float globalGoalDistanceM;
  float localGoalDistanceM;
  float routeAlongProgressM;
  float routeSignedLateralErrorM;
  float obstacleProgressAgeS;
  float cumulativeReverseDistanceM;
  float obstacleBestProgressM;
  float recoveryCurrentClearanceM;
  float recoveryEndpointClearanceM;
  float recoveryClearanceGainM;
  float recoveryUnexploredScore;
  uint8_t recoveryCount;
  uint8_t recoveryPlateauCount;
  bool reverseRecoveryActive;
  int candidateCount;
  PlannerStopReason stopReason;
  const char* planReason;
  const char* replanReason;
  const char* safeStopReason;
  unsigned long lastPlanMs;
  unsigned long plannerSliceUs;
  unsigned long plannerSliceMaxUs;
  unsigned long plannerEpochWorkUs;
  unsigned long plannerEpochMaxWorkUs;
  unsigned long plannerEpochAgeMs;
  unsigned long plannerCommandAgeMs;
  uint8_t plannerCandidatesProcessed;
  uint8_t plannerYieldCount;
  bool plannerEpochActive;
};

enum MissionAction {
  MISSION_ACTION_PAUSE,
  MISSION_ACTION_SEARCH,
  MISSION_ACTION_HOME
};

struct Waypoint {
  // Default route point in world metres plus the action owned by RouteMission.
  float x;
  float y;
  MissionAction action;
};

#endif
