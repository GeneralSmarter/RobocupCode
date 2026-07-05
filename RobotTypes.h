#ifndef ROBOT_TYPES_H
#define ROBOT_TYPES_H

#include <Arduino.h>

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

enum PlannerStopReason {
  PLANNER_STOP_NONE,
  PLANNER_STOP_FRONT_BLOCKED,
  PLANNER_STOP_FRONT_INVALID,
  PLANNER_STOP_NO_SAFE_TRAJECTORY,
  PLANNER_STOP_TURN_SIDE_INVALID,
  PLANNER_STOP_TURN_CLEARANCE,
  PLANNER_STOP_STUCK,
  PLANNER_STOP_ABORTED
};

struct NavigationGoal {
  NavigationGoalMode mode;
  NavigationGoalOwner owner;
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
  float selectedForwardTicksPerSec;
  float selectedTurnTicksPerSec;
  float selectedCurvature;
  float minimumSweptClearanceMm;
  float speedCapTicksPerSec;
  float localGoalDistanceM;
  int candidateCount;
  PlannerStopReason stopReason;
  const char* planReason;
  const char* replanReason;
  const char* safeStopReason;
  unsigned long lastPlanMs;
};

struct Waypoint {
  float x;
  float y;
  const char* action;
};

#endif
