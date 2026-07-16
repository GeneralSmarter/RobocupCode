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
//   these fields, LocalPlanner.cpp consumes navigation and planner structs,
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
// physical front-centre ToF in this layout. RANGE_FAKE_REAR is temporary test
// scaffolding and must not be treated as competition-ready rear coverage.
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

// The active mission currently uses INIT, FOLLOW_PATH, RETURN_HOME and
// END_MATCH. Other labels are retained so telemetry keeps a stable schema
// while future mission phases are built.
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

enum NavigationGoalMode {
  NAV_GOAL_NONE,
  NAV_GOAL_POINT,
  NAV_GOAL_TURN
};

// Owner identifies why a goal exists. It affects completion events,
// test-vs-route cleanup, and special object-hunt finish behavior.
enum NavigationGoalOwner {
  NAV_OWNER_ROUTE,
  NAV_OWNER_RETURN_HOME,
  NAV_OWNER_TEST_DRIVE,
  NAV_OWNER_TEST_GOTO,
  NAV_OWNER_TEST_AVOID,
  NAV_OWNER_TEST_ESCAPE,
  NAV_OWNER_TEST_TURN,
  NAV_OWNER_TEST_HUNT,
  NAV_OWNER_WEIGHT_SCAN,
  NAV_OWNER_OBJECT_HUNT
};

// Exactly one of these may own motion at a time. MotorControl.cpp enforces it
// at command acceptance and again at the final motor writer.
enum MotionAuthority {
  MOTION_AUTHORITY_NONE,
  MOTION_AUTHORITY_MISSION,
  MOTION_AUTHORITY_TEST,
  MOTION_AUTHORITY_MANUAL
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
  PLANNER_STOP_NONE,
  PLANNER_STOP_FRONT_BLOCKED,
  PLANNER_STOP_FRONT_INVALID,
  PLANNER_STOP_NO_SAFE_TRAJECTORY,
  PLANNER_STOP_TURN_SIDE_INVALID,
  PLANNER_STOP_TURN_CLEARANCE,
  PLANNER_STOP_STUCK,
  PLANNER_STOP_RECOVERY_DIVERGENCE,
  PLANNER_STOP_RECOVERY_DISPLACEMENT,
  PLANNER_STOP_RECOVERY_TIME,
  PLANNER_STOP_RECOVERY_DISTANCE,
  PLANNER_STOP_RECOVERY_REPEATED,
  PLANNER_STOP_RECOVERY_NO_PROGRESS,
  PLANNER_STOP_ABORTED
};

struct NavigationGoal {
  // Active goal in world metres/degrees. For NAV_GOAL_POINT, targetX/Y are the
  // desired waypoint. For NAV_GOAL_TURN, targetYawDeg is an absolute
  // navigation-frame heading, positive CCW/left.
  NavigationGoalMode mode;
  NavigationGoalOwner owner;
  MotionAuthority authority;
  bool active;
  bool completed;
  bool failed;
  float targetX;
  float targetY;
  float targetYawDeg;
  float startX;
  float startY;
  float startYawDeg;
  unsigned long startedMs;
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
  float recoveryPhaseElapsedS;
  float cumulativeRecoveryDistanceM;
  float recoveryBestProgressM;
  uint8_t recoveryCount;
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

struct Waypoint {
  // Default route point in world metres plus an action string such as PAUSE,
  // HOME, or SEARCH. StateMachine.cpp interprets the action.
  float x;
  float y;
  const char* action;
};

#endif
