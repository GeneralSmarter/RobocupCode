#ifndef PLANNER_TYPES_H
#define PLANNER_TYPES_H

#include "Robot.h"

// Internal planner-only data structures. Mission and pickup code must use the
// public navigation status/debug interfaces instead of including this file.
struct LocalMapCell {
  // Evidence is deliberately separate from a simple occupied/free boolean.
  // A single noisy return can be contradicted by later free observations
  // instead of permanently poisoning the local map.
  int8_t freeEvidence;
  // Static evidence decays slowly; dynamic evidence decays quickly. Both may
  // make a cell occupied once they cross PLANNER_OBSTACLE_SCORE_THRESHOLD.
  int8_t staticEvidence;
  int8_t dynamicEvidence;
  // This is recorded only under the robot's own footprint. It is presently
  // useful for diagnostics and is the natural basis for a future non-blind
  // reverse/backtrack policy.
  uint8_t traversedEvidence;
  unsigned long lastObservedMs;
  unsigned long lastTraversedMs;
};

const int ARENA_MEMORY_BITS = ARENA_MEMORY_CELLS * ARENA_MEMORY_CELLS;
const int ARENA_MEMORY_BYTES = (ARENA_MEMORY_BITS + 7) / 8;
struct ReverseRecoveryState {
  bool active;
  bool checkingForward;
  bool forwardCheckAfterMovement;
  float startX;
  float startY;
  float currentClearanceM;
  float clearanceGainM;
  uint8_t plateauCount;
};

struct RecoveryBudget {
  bool forwardTakeoverPending;
  uint8_t attemptCount;
  float cumulativeReverseDistanceM;
  float lastReverseX;
  float lastReverseY;
  float takeoverStartGoalDistanceM;
  float takeoverStartRouteAlongM;
};

enum EmergencyRecoveryPhase {
  EMERGENCY_RECOVERY_IDLE,
  EMERGENCY_RECOVERY_SETTLE_CURRENT,
  EMERGENCY_RECOVERY_SCAN_TURN,
  EMERGENCY_RECOVERY_SCAN_DWELL,
  EMERGENCY_RECOVERY_SCAN_UNWIND,
  EMERGENCY_RECOVERY_RELOCATE,
  EMERGENCY_RECOVERY_SETTLE_RELOCATED,
  EMERGENCY_RECOVERY_RESET_RETRY,
  EMERGENCY_RECOVERY_RETRY_ACTIVE
};

struct EmergencyRecoveryState {
  EmergencyRecoveryPhase phase;
  bool consumed;
  bool retryActive;
  unsigned long startedMs;
  unsigned long phaseStartedMs;
  unsigned long relocationStartedMs;
  float startX;
  float startY;
  float lastX;
  float lastY;
  float relocationCheckpointX;
  float relocationCheckpointY;
  float relocationDistanceM;
  float bestRotationalClearanceM;
  float scanLastYawDeg;
  float scanAccumulatedDeg;
  float scanTargetAccumulatedDeg;
  int8_t scanDirection;
  uint8_t scanSector;
  unsigned long fanReadBaselineMs[4];
  uint32_t rearFrameBaseline;
};

struct ObstacleContext {
  bool active;
  float originX;
  float originY;
  float routeUx;
  float routeUy;
  float routeLengthM;
  float approachNearAlongM;
  float nearAlongM;
  float farAlongM;
  float minLateralM;
  float maxLateralM;
  float sideSign;
  float sideEscapeAlongM;
  bool sideReconsidered;
  bool progressGoalValid;
  float progressGoalX;
  float progressGoalY;
  float progressStartDistanceM;
  float progressBestDistanceM;
  unsigned long progressLastMs;
  unsigned long clearSinceMs;
};

struct ObstacleEnvelope {
  bool found;
  float nearAlongM;
  float farAlongM;
  float minLateralM;
  float maxLateralM;
};

enum CandidateRejectReason {
  CANDIDATE_REJECT_NONE,
  CANDIDATE_REJECT_TURN_OBSERVABILITY,
  CANDIDATE_REJECT_FORWARD_OBSERVATION,
  CANDIDATE_REJECT_REAR_OBSERVATION,
  CANDIDATE_REJECT_FOOTPRINT,
  CANDIDATE_REJECT_CORRIDOR,
  CANDIDATE_REJECT_CLEAR_EVIDENCE
};

enum TrajectoryPlanResult {
  TRAJECTORY_PLAN_PENDING,
  TRAJECTORY_PLAN_SUCCESS,
  TRAJECTORY_PLAN_RETRY,
  TRAJECTORY_PLAN_NO_PATH,
  TRAJECTORY_PLAN_ABORTED
};

const int PLANNER_OCCUPANCY_BITS = LOCAL_MAP_CELLS * LOCAL_MAP_CELLS;
const int PLANNER_OCCUPANCY_BYTES = (PLANNER_OCCUPANCY_BITS + 7) / 8;

struct PlannerCollisionSnapshot {
  float originX;
  float originY;
  uint8_t occupied[PLANNER_OCCUPANCY_BYTES];
  uint8_t knownClear[PLANNER_OCCUPANCY_BYTES];
};

struct PlannerEpoch {
  bool active;
  bool awaitingRevalidation;
  bool commandStoppedForAge;
  bool countersteerFallbackPass;
  unsigned long startedMs;
  unsigned long goalStartedMs;
  MotionAuthority authority;
  unsigned long accumulatedWorkUs;
  uint8_t yieldCount;
  uint8_t candidateIndex;
  float startX;
  float startY;
  float startHeadingRad;
  float goalX;
  float goalY;
  float localGoalDistanceM;
  float finalGoalDistanceM;
  float requestedSpeedCap;
  float speedCap;
  float routeHeadingRad;
  float previousSelectedTurn;
  float minimumFanClearanceMm;
  float observedRightInnerM;
  float observedLeftInnerM;
  bool rightInnerValid;
  bool leftInnerValid;
  bool rightOuterValid;
  bool leftOuterValid;
  bool finalWaypointIsLocalGoal;
  bool lineFollowActive;
  int acceptedCount;
  int rejectedTurnObservability;
  int rejectedForwardObservation;
  int rejectedFootprint;
  int rejectedCorridor;
  int skippedLinePolicy;
  float bestScore;
  float bestForward;
  float bestTurn;
  bool bestReachesGoal;
  float bestArrivalTimeS;
  PlannerCollisionSnapshot collision;
};

struct ReversePlannerEpoch {
  bool active;
  bool awaitingRevalidation;
  bool commandStoppedForAge;
  bool emergencyScanObjective;
  unsigned long startedMs;
  unsigned long goalStartedMs;
  MotionAuthority authority;
  unsigned long accumulatedWorkUs;
  uint8_t yieldCount;
  uint8_t candidateIndex;
  float startX;
  float startY;
  float startHeadingRad;
  float goalX;
  float goalY;
  float observedRearM;
  float speedCap;
  float previousSelectedTurn;
  bool rearValid;
  bool rearBlocked;
  int acceptedCount;
  int rejectedRear;
  int rejectedFootprint;
  int rejectedEvidence;
  float allowedUnknownFraction;
  float bestScore;
  float bestReverse;
  float bestTurn;
  int bestClearanceBand;
  PlannerCollisionSnapshot collision;
};

#endif
