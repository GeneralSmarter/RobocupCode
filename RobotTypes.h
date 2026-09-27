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
//   ToF modules fill RangeSensorState and immutable matrix/rear observations;
//   structs, and MotorControl.cpp enforces MotionAuthority.
// Control flow:
//   No executable runtime logic except small constexpr policy checks and
//   static_assert invariants for motion authority.
// Global state:
//   These are type definitions only. Instances live in Globals.cpp and are
//   read/modified by the subsystem that owns each behavior.

#include <Arduino.h>

// Physical fan sensors come first and match FRONT_FAN_CONFIG indices.
// RANGE_FRONT/RANGE_LEFT/RANGE_RIGHT are derived aggregate views; there is no
// physical front-centre single-zone ToF in this layout. Rear and front-matrix
// aggregate slots are live derived safety evidence, not physical sensors.
enum RangeSensorId {
  RANGE_RIGHT_OUTER,
  RANGE_RIGHT_INNER,
  RANGE_LEFT_INNER,
  RANGE_LEFT_OUTER,
  RANGE_FRONT,
  RANGE_RIGHT,
  RANGE_LEFT,
  RANGE_REAR_AGGREGATE,
  RANGE_FRONT_MATRIX_AGGREGATE,
  RANGE_SENSOR_COUNT
};

enum SensorI2cBus {
  SENSOR_I2C_PRIMARY,
  SENSOR_I2C_SECONDARY
};

struct SensorMountPose {
  float xMm;
  float yMm;
  float zMm;
  float yawDeg;
  float pitchDeg;
  float rollDeg;
};

struct FrontFanSensorConfig {
  const char* name;
  SensorMountPose mount;
  byte xshutChannel;
  uint8_t i2cAddress;
};

enum RearTofId {
  REAR_TOF_LEFT,
  REAR_TOF_CENTRE,
  REAR_TOF_RIGHT,
  REAR_TOF_COUNT
};

struct RearTofConfig {
  const char* name;
  SensorMountPose mount;
  SensorI2cBus bus;
  byte xshutChannel;
  uint8_t i2cAddress;
  uint8_t roiWidth;
  uint8_t roiHeight;
  float horizontalFovDeg;
  float verticalFovDeg;
  uint32_t timingBudgetUs;
  unsigned long samplePeriodMs;
  uint16_t validMinimumMm;
  uint16_t validMaximumMm;
  uint16_t stopDistanceMm;
  uint16_t clearDistanceMm;
};

struct RearTofState {
  bool connected;
  bool valid;
  bool stale;
  bool blocked;
  uint16_t distanceMm;
  uint8_t rangeStatus;
  float signalMcps;
  float ambientMcps;
  uint32_t sequence;
  unsigned long acquiredMs;
  unsigned long timeoutCount;
  unsigned long invalidCount;
};

struct PayloadTofConfig {
  const char* name;
  SensorMountPose mount;
  SensorI2cBus bus;
  byte xshutChannel;
  uint8_t i2cAddress;
  uint32_t timingBudgetUs;
  unsigned long samplePeriodMs;
  uint16_t validMinimumMm;
  uint16_t validMaximumMm;
  uint16_t confirmationMaximumMm;
};

struct PayloadTofObservation {
  bool connected;
  bool valid;
  bool stale;
  uint16_t distanceMm;
  uint8_t rangeStatus;
  float signalMcps;
  float ambientMcps;
  uint32_t sequence;
  unsigned long acquiredMs;
};

enum PayloadEvidenceState {
  PAYLOAD_UNKNOWN,
  PAYLOAD_CAPTURE_ENTRY_SEEN,
  PAYLOAD_PRESENT_UNCLASSIFIED
};

struct FrontMatrixConfig {
  const char* name;
  SensorMountPose mount;
  SensorI2cBus bus;
  uint8_t i2cAddress;
  uint8_t rows;
  uint8_t columns;
  float horizontalFovDeg;
  float verticalFovDeg;
  uint8_t gridRotationQuarterTurns;
  bool flipRows;
  bool flipColumns;
  uint64_t safetyCellMask;
  uint64_t perceptionCellMask;
};

struct RangeRayObservation {
  float originXmm;
  float originYmm;
  float originZmm;
  float yawDeg;
  float pitchDeg;
  uint16_t distanceMm;
  uint32_t sequence;
  unsigned long acquiredMs;
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

enum FrontMatrixCellState {
  FRONT_MATRIX_CELL_UNKNOWN,
  FRONT_MATRIX_CELL_VALID
};

struct FrontMatrixFrame {
  uint16_t distanceMm[64];
  FrontMatrixCellState cellState[64];
  bool valid;
  uint32_t sequence;
  unsigned long acquiredMs;
  float robotX;
  float robotY;
  float robotHeadingDeg;
  float pitchDeg;
  float rollDeg;
  uint8_t gridRotationQuarterTurns;
  bool flipRows;
  bool flipColumns;
};

enum MatrixEvidenceKind {
  MATRIX_EVIDENCE_NONE,
  MATRIX_EVIDENCE_WEIGHT_CANDIDATE,
  MATRIX_EVIDENCE_WALL_LIKE,
  MATRIX_EVIDENCE_DYNAMIC_LOW_OBJECT,
  MATRIX_EVIDENCE_MIXED_OR_OCCLUDED,
  MATRIX_EVIDENCE_UNKNOWN
};

struct MatrixTargetObservation {
  bool valid;
  bool confirmedStatic;
  uint32_t trackId;
  uint32_t frameSequence;
  unsigned long acquiredMs;
  uint64_t sourceCellMask;
  MatrixEvidenceKind evidence;
  float robotXmm;
  float robotYmm;
  float worldX;
  float worldY;
  float widthMm;
  float heightMm;
  float column;
  float columnError;
  float directGapMm;
  float apparentSpeedMps;
};

struct RouteResumeContext {
  bool valid;
  int routeIndex;
  float segmentStartX;
  float segmentStartY;
  float segmentEndX;
  float segmentEndY;
  float alongSegmentProgressM;
  float interruptX;
  float interruptY;
  uint32_t trackId;
};

enum PickupTrackingPhase {
  PICKUP_TRACKING_IDLE,
  PICKUP_TRACKING_FULL_SPEED,
  PICKUP_TRACKING_FINAL_APPROACH_PREDICT,
  PICKUP_TRACKING_HANDOFF_ASSUMED,
  PICKUP_TRACKING_FEEDING_UNCONFIRMED,
  PICKUP_TRACKING_FEED_COMPLETE_UNCONFIRMED,
  PICKUP_TRACKING_FAILED,
  // Appended to preserve the numeric values used by saved simulator traces.
  PICKUP_TRACKING_FEED_COMPLETE_CONFIRMED
};

enum PickupTrackingOutcome {
  PICKUP_OUTCOME_NONE,
  PICKUP_OUTCOME_RUNNING,
  PICKUP_OUTCOME_WEIGHT_FUNNEL_HANDOFF_ASSUMED,
  PICKUP_OUTCOME_FEED_COMPLETE_UNCONFIRMED,
  PICKUP_OUTCOME_TARGET_LOST,
  PICKUP_OUTCOME_FINAL_APPROACH_ESTIMATE_EXPIRED,
  PICKUP_OUTCOME_INACCESSIBLE_TARGET,
  PICKUP_OUTCOME_FEED_INTERRUPTED,
  PICKUP_OUTCOME_NAVIGATION_FAILED,
  // Appended to preserve the numeric values used by saved simulator traces.
  PICKUP_OUTCOME_FEED_COMPLETE_CONFIRMED
};

struct PickupTrackingStatus {
  PickupTrackingPhase phase;
  PickupTrackingOutcome outcome;
  uint32_t trackId;
  float bestGapMm;
  float columnError;
  float remainingFeedMm;
  bool usingPredictedGap;
  uint32_t captureAttemptId;
  PayloadEvidenceState payloadEvidence;
  const char* detail;
};

enum MatrixFollowDiagnosticPhase {
  MATRIX_FOLLOW_IDLE,
  MATRIX_FOLLOW_SLOW_SCAN,
  MATRIX_FOLLOW_ACQUIRE_STATIC_WEIGHT,
  MATRIX_FOLLOW_LOCKED_TRACK,
  MATRIX_FOLLOW_BOUNDED_REACQUIRE,
  MATRIX_FOLLOW_STOPPED
};

struct MatrixFollowDiagnosticStatus {
  MatrixFollowDiagnosticPhase phase;
  uint32_t trackId;
  const char* detail;
};

// Tells the final motor writer how to realise an accepted chassis command.
// Navigation selects this explicitly so motor control never reads private
// navigation-goal state.
enum MotionCommandMode {
  MOTION_COMMAND_STANDARD,
  MOTION_COMMAND_NAV_DRIVE,
  MOTION_COMMAND_NAV_TURN,
  MOTION_COMMAND_NAV_SCAN_TURN,
  MOTION_COMMAND_NAV_PICKUP_TRACK
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
  // Reject-reason counts from the most recently closed reverse-recovery
  // epoch. Diagnoses *why* reverse recovery cannot find a candidate, which
  // the STATUS/CSV stream does not otherwise expose.
  int reverseRejectedRear;
  int reverseRejectedFootprint;
  int reverseRejectedEvidence;
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
