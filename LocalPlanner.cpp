#include "Robot.h"

// =====================================================
// Local confidence map and receding-horizon navigation
// =====================================================
// Responsibility:
//   Owns local perception-to-motion planning: rolling occupancy evidence,
//   persistent arena memory,
//   footprint collision checks, differential-drive arc rollout/scoring, point
//   and turn navigation goals, obstacle-local forward bypass, planner telemetry,
//   and the scheduled controller pipeline.
// Interacts with:
//   TofSensors.cpp supplies fan and rear range evidence. Odometry.cpp updates
//   robot pose. StateMachine.cpp and Bluetooth.cpp create/cancel goals.
//   MotorControl.cpp is the only physical motor writer and accepts commands
//   through the planner publish wrapper and the authorized motor-command API.
// Control flow:
//   updateRobotController() is called from loop(). It schedules sensors/map,
//   odometry, updateNavigationController(), updateMotorController(), and
//   telemetry construction. Point goals use cooperative planner epochs; turn
//   goals use direct yaw feedback.
// Global state:
//   Modifies localMap, navigationGoal, plannerTelemetry, one obstacle-scoped
//   context, planner epochs, motorStopRequested/desired motion through safe
//   command APIs, and controller timing stamps.
// The rolling map preserves fresh dynamic evidence. A compact fixed-world
// layer remembers thresholded occupied and clear cells for the match while
// still allowing repeated contradictory observations to correct them.

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

// localMap is expressed in world metres, but stored as a small robot-centred
// grid. shiftedMap is a scratch buffer used only when the robot has walked far
// enough that the local grid needs to be re-centred around it.
static LocalMapCell localMap[LOCAL_MAP_CELLS][LOCAL_MAP_CELLS];
static LocalMapCell shiftedMap[LOCAL_MAP_CELLS][LOCAL_MAP_CELLS];
static bool localMapInitialized = false;
static float localMapOriginX = 0.0;
static float localMapOriginY = 0.0;
static uint32_t lastRearEvidenceFrameSequence = 0;
const int ARENA_MEMORY_BITS = ARENA_MEMORY_CELLS * ARENA_MEMORY_CELLS;
const int ARENA_MEMORY_BYTES = (ARENA_MEMORY_BITS + 7) / 8;
static int8_t arenaMemoryChallenge[ARENA_MEMORY_BITS];
static uint8_t arenaMemoryOccupied[ARENA_MEMORY_BYTES];
static uint8_t arenaMemoryKnownClear[ARENA_MEMORY_BYTES];
static bool arenaMemoryInitialized = false;
static float arenaMemoryOriginX = 0.0f;
static float arenaMemoryOriginY = 0.0f;
static PlannerStopReason lastReportedStopReason = PLANNER_STOP_NONE;
static bool turnBrakeActive = false;
static unsigned long turnBrakeUntilMs = 0;
static float turnLastCommandDirection = 0.0;
static bool pointAlignTurnActive = false;
static float pointAlignTurnDirection = 0.0;
static unsigned long turnSideInvalidSinceMs = 0;
static unsigned long turnSweepInvalidSinceMs = 0;
static bool reverseRecoveryActive = false;
static unsigned long reverseRecoveryStartedMs = 0;
static unsigned long reverseRecoveryStepCount = 0;
static unsigned long noSafeTrajectorySinceMs = 0;
static uint8_t geometricNoPathEpochCount = 0;
static bool lastForwardNoPathWasGeometric = false;
static bool candidateRejectsReported = false;
static bool reverseRecoveryRejectsReported = false;
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
static ReverseRecoveryState reverseRecoveryState = {};
struct RecoveryBudget {
  bool forwardTakeoverPending;
  uint8_t attemptCount;
  float cumulativeReverseDistanceM;
  float lastReverseX;
  float lastReverseY;
  float takeoverStartGoalDistanceM;
  float takeoverStartRouteAlongM;
};
static RecoveryBudget recoveryBudget = {};
static float lastFootprintRejectWorldX = 0.0f;
static float lastFootprintRejectWorldY = 0.0f;
static int lastFootprintRejectCellX = -1;
static int lastFootprintRejectCellY = -1;
static float lastCorridorRejectLeftM = -1.0f;
static float lastCorridorRejectRightM = -1.0f;

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

static EmergencyRecoveryState emergencyRecoveryState = {};
static bool emergencyScanPolicyEnabled = PLANNER_EMERGENCY_SCAN_ENABLED;

bool setEmergencyScanPolicyEnabled(bool enabled) {
  if (navigationGoal.active) {
    return false;
  }
  emergencyScanPolicyEnabled = enabled;
  return true;
}

bool isEmergencyScanPolicyEnabled() {
  return emergencyScanPolicyEnabled;
}

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

static ObstacleContext obstacleContext = {};

static void resetRecoveryBudget();
static bool routeLineFrame(float &routeLengthM, float &routeUx,
                           float &routeUy, float &routeHeadingRad);
static float routeLineSignedLateralErrorM(float worldX, float worldY,
                                          float routeUx, float routeUy);
static void resetObstacleContext(const char* reason);
static bool handleRecoveryExhaustion(PlannerStopReason reason,
                                     const char* detail);

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

static PlannerEpoch plannerEpoch = {};
static ReversePlannerEpoch reversePlannerEpoch = {};
static unsigned long lastPlannerCommandPublishedMs = 0;

static void resetPlannerEpoch() {
  // Clears any in-progress forward planning epoch and resets planner timing
  // telemetry. It does not cancel the navigation goal itself.
  plannerEpoch.active = false;
  plannerEpoch.awaitingRevalidation = false;
  plannerEpoch.commandStoppedForAge = false;
  plannerTelemetry.plannerEpochActive = false;
  plannerTelemetry.plannerEpochAgeMs = 0;
  plannerTelemetry.plannerCandidatesProcessed = 0;
  plannerTelemetry.plannerYieldCount = 0;
  plannerTelemetry.plannerSliceUs = 0;
  plannerTelemetry.plannerSliceMaxUs = 0;
  plannerTelemetry.plannerEpochWorkUs = 0;
  plannerTelemetry.plannerEpochMaxWorkUs = 0;
  plannerTelemetry.plannerCommandAgeMs = 0;
}

static void resetReversePlannerEpoch() {
  reversePlannerEpoch.active = false;
  reversePlannerEpoch.awaitingRevalidation = false;
  reversePlannerEpoch.commandStoppedForAge = false;
  if (!plannerEpoch.active) {
    plannerTelemetry.plannerEpochActive = false;
  }
}

static void closePlannerEpoch() {
  // Marks the current epoch complete and records its age for telemetry. The
  // chosen command, if any, has already been published before this is called.
  plannerEpoch.active = false;
  plannerEpoch.awaitingRevalidation = false;
  plannerTelemetry.plannerEpochActive = false;
  plannerTelemetry.plannerEpochAgeMs = millis() - plannerEpoch.startedMs;
}

static void resetGeometricNoPathEvidence() {
  noSafeTrajectorySinceMs = 0;
  geometricNoPathEpochCount = 0;
  lastForwardNoPathWasGeometric = false;
}

static void noteGeometricNoPathEpoch() {
  // Counts consecutive no-path epochs that were caused by geometry/footprint
  // evidence rather than stale sensors, authority loss, or policy retries.
  unsigned long now = millis();
  if (geometricNoPathEpochCount == 0) {
    noSafeTrajectorySinceMs = now;
  }
  if (geometricNoPathEpochCount < 255) {
    geometricNoPathEpochCount++;
  }
}

static bool currentPlannerFailureIsGeometricNoPath() {
  return lastForwardNoPathWasGeometric;
}

static bool areTurnSweepSensorsValid() {
  // A pivot sweeps every corner of the chassis through a circle, so it needs
  // all four fan sectors. Straight driving does not make this assumption.
  for (int i = RANGE_RIGHT_OUTER; i <= RANGE_LEFT_OUTER; i++) {
    if (!isRangeSensorValid((RangeSensorId)i)) {
      return false;
    }
  }
  return true;
}

static float clampEvidence(float value) {
  return constrain(value, -120.0, 120.0);
}

enum ArenaMemoryState {
  ARENA_MEMORY_UNKNOWN,
  ARENA_MEMORY_CLEAR,
  ARENA_MEMORY_OCCUPIED
};

static void initialiseArenaMemoryAtRobot() {
  arenaMemoryOriginX = robotX - ARENA_MEMORY_SIZE_M * 0.5f;
  arenaMemoryOriginY = robotY - ARENA_MEMORY_SIZE_M * 0.5f;
  memset(arenaMemoryChallenge, 0, sizeof(arenaMemoryChallenge));
  memset(arenaMemoryOccupied, 0, sizeof(arenaMemoryOccupied));
  memset(arenaMemoryKnownClear, 0, sizeof(arenaMemoryKnownClear));
  arenaMemoryInitialized = true;
}

static bool arenaWorldToCell(float worldX, float worldY,
                             int &cellX, int &cellY) {
  if (!arenaMemoryInitialized) {
    return false;
  }
  cellX = (int)floorf((worldX - arenaMemoryOriginX) / LOCAL_MAP_CELL_M);
  cellY = (int)floorf((worldY - arenaMemoryOriginY) / LOCAL_MAP_CELL_M);
  return cellX >= 0 && cellX < ARENA_MEMORY_CELLS &&
         cellY >= 0 && cellY < ARENA_MEMORY_CELLS;
}

static ArenaMemoryState arenaMemoryStateAtCell(int cellX, int cellY) {
  if (cellX < 0 || cellX >= ARENA_MEMORY_CELLS ||
      cellY < 0 || cellY >= ARENA_MEMORY_CELLS) {
    return ARENA_MEMORY_UNKNOWN;
  }
  int bit = cellY * ARENA_MEMORY_CELLS + cellX;
  uint8_t mask = (uint8_t)(1U << (bit & 7));
  if ((arenaMemoryOccupied[bit >> 3] & mask) != 0) {
    return ARENA_MEMORY_OCCUPIED;
  }
  return (arenaMemoryKnownClear[bit >> 3] & mask) != 0
    ? ARENA_MEMORY_CLEAR : ARENA_MEMORY_UNKNOWN;
}

static ArenaMemoryState arenaMemoryStateAtWorld(float worldX, float worldY) {
  int cellX;
  int cellY;
  return arenaWorldToCell(worldX, worldY, cellX, cellY)
    ? arenaMemoryStateAtCell(cellX, cellY) : ARENA_MEMORY_UNKNOWN;
}

static void setArenaMemoryState(int cellX, int cellY,
                                ArenaMemoryState state) {
  int bit = cellY * ARENA_MEMORY_CELLS + cellX;
  uint8_t mask = (uint8_t)(1U << (bit & 7));
  arenaMemoryOccupied[bit >> 3] &= (uint8_t)~mask;
  arenaMemoryKnownClear[bit >> 3] &= (uint8_t)~mask;
  if (state == ARENA_MEMORY_OCCUPIED) {
    arenaMemoryOccupied[bit >> 3] |= mask;
  } else if (state == ARENA_MEMORY_CLEAR) {
    arenaMemoryKnownClear[bit >> 3] |= mask;
  }
  arenaMemoryChallenge[bit] = 0;
}

static void addArenaMemoryEvidence(float worldX, float worldY, int amount) {
  int cellX;
  int cellY;
  if (amount == 0 || !arenaWorldToCell(worldX, worldY, cellX, cellY)) {
    return;
  }
  if ((unsigned int)cellX >= (unsigned int)ARENA_MEMORY_CELLS ||
      (unsigned int)cellY >= (unsigned int)ARENA_MEMORY_CELLS) {
    return;
  }

  ArenaMemoryState state = arenaMemoryStateAtCell(cellX, cellY);
  int bit = cellY * ARENA_MEMORY_CELLS + cellX;
  int challenge = arenaMemoryChallenge[bit];
  if (amount > 0) {
    if (state == ARENA_MEMORY_OCCUPIED) {
      arenaMemoryChallenge[bit] = (int8_t)min(0, challenge + amount);
      return;
    }
    challenge = challenge < 0 ? 0 : challenge;
    challenge = min(120, challenge + amount);
    if (challenge >= ARENA_MEMORY_EVIDENCE_THRESHOLD) {
      setArenaMemoryState(cellX, cellY, ARENA_MEMORY_OCCUPIED);
      return;
    }
  } else {
    if (state == ARENA_MEMORY_CLEAR) {
      arenaMemoryChallenge[bit] = (int8_t)max(0, challenge + amount);
      return;
    }
    challenge = challenge > 0 ? 0 : challenge;
    challenge = max(-120, challenge + amount);
    if (challenge <= -ARENA_MEMORY_EVIDENCE_THRESHOLD) {
      setArenaMemoryState(cellX, cellY, ARENA_MEMORY_CLEAR);
      return;
    }
  }
  arenaMemoryChallenge[bit] = (int8_t)challenge;
}

static void markArenaMemoryClear(float worldX, float worldY) {
  int cellX;
  int cellY;
  if (arenaWorldToCell(worldX, worldY, cellX, cellY)) {
    setArenaMemoryState(cellX, cellY, ARENA_MEMORY_CLEAR);
  }
}

static void initialiseMapAtRobot() {
  // Put the robot at the centre of the map. The map origin is its lower-left
  // world coordinate, not the robot pose.
  localMapOriginX = robotX - LOCAL_MAP_SIZE_M * 0.5;
  localMapOriginY = robotY - LOCAL_MAP_SIZE_M * 0.5;
  memset(localMap, 0, sizeof(localMap));
  localMapInitialized = true;
}

static bool worldToCell(float worldX, float worldY, int &cellX, int &cellY) {
  if (!localMapInitialized) {
    return false;
  }

  // Every map operation uses this one conversion. Keeping it here prevents
  // subtle disagreements about rounding at cell boundaries.
  cellX = (int)floorf((worldX - localMapOriginX) / LOCAL_MAP_CELL_M);
  cellY = (int)floorf((worldY - localMapOriginY) / LOCAL_MAP_CELL_M);
  return cellX >= 0 && cellX < LOCAL_MAP_CELLS &&
         cellY >= 0 && cellY < LOCAL_MAP_CELLS;
}

static void recenterLocalMapIfNeeded() {
  if (!localMapInitialized) {
    initialiseMapAtRobot();
    return;
  }

  float centreX = localMapOriginX + LOCAL_MAP_SIZE_M * 0.5;
  float centreY = localMapOriginY + LOCAL_MAP_SIZE_M * 0.5;
  float deltaX = robotX - centreX;
  float deltaY = robotY - centreY;

  if (fabs(deltaX) < LOCAL_MAP_RECENTER_MARGIN_M &&
      fabs(deltaY) < LOCAL_MAP_RECENTER_MARGIN_M) {
    return;
  }

  // Shift by an integer number of cells so remembered obstacles retain their
  // world positions. New space entering the map is cleared by memset().
  int shiftX = (int)roundf(deltaX / LOCAL_MAP_CELL_M);
  int shiftY = (int)roundf(deltaY / LOCAL_MAP_CELL_M);
  if (shiftX == 0 && shiftY == 0) {
    return;
  }

  memset(shiftedMap, 0, sizeof(shiftedMap));
  for (int y = 0; y < LOCAL_MAP_CELLS; y++) {
    for (int x = 0; x < LOCAL_MAP_CELLS; x++) {
      int destinationX = x - shiftX;
      int destinationY = y - shiftY;
      if (destinationX >= 0 && destinationX < LOCAL_MAP_CELLS &&
          destinationY >= 0 && destinationY < LOCAL_MAP_CELLS) {
        shiftedMap[destinationY][destinationX] = localMap[y][x];
      }
    }
  }

  memcpy(localMap, shiftedMap, sizeof(localMap));
  localMapOriginX += shiftX * LOCAL_MAP_CELL_M;
  localMapOriginY += shiftY * LOCAL_MAP_CELL_M;
}

static void decayLocalMap() {
  // The map is intentionally short-lived. This avoids pretending that
  // encoder-only odometry can support a permanent arena-scale world model.
  unsigned long now = millis();
  for (int y = 0; y < LOCAL_MAP_CELLS; y++) {
    for (int x = 0; x < LOCAL_MAP_CELLS; x++) {
      LocalMapCell &cell = localMap[y][x];
      if (cell.dynamicEvidence > 0 && now - cell.lastObservedMs > MAP_DYNAMIC_EXPIRY_MS) {
        cell.dynamicEvidence = 0;
      }
      if (cell.staticEvidence > 0 && now - cell.lastObservedMs > MAP_STATIC_EXPIRY_MS) {
        cell.staticEvidence = 0;
      }
      if (cell.traversedEvidence > 0 && now - cell.lastTraversedMs > MAP_TRAVERSED_EXPIRY_MS) {
        cell.traversedEvidence = 0;
      }
    }
  }
}

static void addFreeEvidence(float worldX, float worldY, int amount) {
  int x;
  int y;
  if (!worldToCell(worldX, worldY, x, y)) {
    return;
  }

  LocalMapCell &cell = localMap[y][x];
  // A free observation does three things: remember free space, weaken a
  // recent obstacle report strongly, and weaken older static evidence more
  // gently. This lets two different rays correct each other over time.
  cell.freeEvidence = (int8_t)clampEvidence(cell.freeEvidence + amount);
  cell.dynamicEvidence = (int8_t)clampEvidence(cell.dynamicEvidence - amount);
  cell.staticEvidence = (int8_t)clampEvidence(cell.staticEvidence - amount / 3);
  cell.lastObservedMs = millis();
  addArenaMemoryEvidence(worldX, worldY, -max(1, amount / 3));
}

static void markFreeRayEvidence(float sensorWorldX, float sensorWorldY,
                                float rayHeadingRad, float freeLengthM,
                                float halfWidthM, int evidenceAmount) {
  for (float distance = 0.0f; distance <= freeLengthM;
       distance += LOCAL_MAP_CELL_M * 0.5f) {
    float centreX = sensorWorldX + cosf(rayHeadingRad) * distance;
    float centreY = sensorWorldY + sinf(rayHeadingRad) * distance;
    for (float lateral = -halfWidthM;
         lateral <= halfWidthM;
         lateral += LOCAL_MAP_CELL_M) {
      float freeX = centreX - sinf(rayHeadingRad) * lateral;
      float freeY = centreY + cosf(rayHeadingRad) * lateral;
      addFreeEvidence(freeX, freeY, evidenceAmount);
    }
  }
}

static bool isOuterFanSensor(RangeSensorId id) {
  return id == RANGE_RIGHT_OUTER || id == RANGE_LEFT_OUTER;
}

static float fanForwardObservationDistanceM(RangeSensorId id) {
  // The planner's rollout distance is measured from the robot origin.  Inner
  // fan ranges are measured along angled beams from sensors mounted ahead of
  // that origin, so compare against the endpoint's forward projection rather
  // than the raw range.  Footprint/map checks still decide whether the
  // lateral side-wall endpoint itself is collision-free.
  if (id < RANGE_RIGHT_OUTER || id > RANGE_LEFT_OUTER ||
      !isRangeSensorValid(id)) {
    return 0.0f;
  }

  const FanSensorGeometry &sensor = FAN_SENSOR_GEOMETRY[(int)id];
  float rangeM = getRangeSensorDistance(id) / 1000.0f;
  return sensor.xMm / 1000.0f +
         rangeM * cosf(sensor.angleDeg * DEG_TO_RAD);
}

static void markDirectionalEndpointEvidence(
    float worldX, float worldY, float rayHeadingRad,
    float backUncertaintyM, float forwardUncertaintyM,
    float lateralUncertaintyM, int dynamicEvidence, int staticEvidence) {
  // A ToF reading says "an object lies somewhere near this ray endpoint".
  // It does NOT justify filling a large circular obstacle around it. The
  // uncertainty box below is aligned to the beam: range uncertainty is along
  // the beam and beam/cone uncertainty is across it.
  int radiusCells = (int)ceilf(max(max(backUncertaintyM, forwardUncertaintyM),
                                  lateralUncertaintyM) / LOCAL_MAP_CELL_M);
  int centreX;
  int centreY;
  if (!worldToCell(worldX, worldY, centreX, centreY)) {
    return;
  }

  for (int y = centreY - radiusCells; y <= centreY + radiusCells; y++) {
    for (int x = centreX - radiusCells; x <= centreX + radiusCells; x++) {
      if (x < 0 || x >= LOCAL_MAP_CELLS || y < 0 || y >= LOCAL_MAP_CELLS) {
        continue;
      }
      float dx = (x - centreX) * LOCAL_MAP_CELL_M;
      float dy = (y - centreY) * LOCAL_MAP_CELL_M;
      float alongBeamM = dx * cosf(rayHeadingRad) + dy * sinf(rayHeadingRad);
      float acrossBeamM = -dx * sinf(rayHeadingRad) + dy * cosf(rayHeadingRad);
      if (alongBeamM >= -backUncertaintyM &&
          alongBeamM <= forwardUncertaintyM &&
          fabs(acrossBeamM) <= lateralUncertaintyM) {
        LocalMapCell &cell = localMap[y][x];
        cell.dynamicEvidence = (int8_t)clampEvidence(cell.dynamicEvidence + dynamicEvidence);
        cell.staticEvidence = (int8_t)clampEvidence(cell.staticEvidence + staticEvidence);
        cell.freeEvidence = (int8_t)clampEvidence(cell.freeEvidence - dynamicEvidence / 2);
        if (cell.staticEvidence >= PLANNER_OBSTACLE_SCORE_THRESHOLD ||
            cell.dynamicEvidence >= PLANNER_OBSTACLE_SCORE_THRESHOLD) {
          // Contradictory obstacle evidence revokes persistent reverse
          // permission until a later clear observation earns it again.
          cell.freeEvidence = 0;
        }
        cell.lastObservedMs = millis();
        float cellWorldX = localMapOriginX + (x + 0.5f) * LOCAL_MAP_CELL_M;
        float cellWorldY = localMapOriginY + (y + 0.5f) * LOCAL_MAP_CELL_M;
        addArenaMemoryEvidence(cellWorldX, cellWorldY, staticEvidence);
      }
    }
  }
}

static void markEndpointEvidence(RangeSensorId id, float worldX, float worldY,
                                 float rayHeadingRad) {
  const bool outer = isOuterFanSensor(id);
  // Outer rays are useful for route choice, but are more oblique and less
  // reliable for declaring a hard forward obstruction. Their lower evidence
  // therefore needs repeated observations before cellOccupied() rejects them.
  markDirectionalEndpointEvidence(
    worldX,
    worldY,
    rayHeadingRad,
    outer ? MAP_OUTER_ENDPOINT_BACK_UNCERTAINTY_M
          : MAP_INNER_ENDPOINT_BACK_UNCERTAINTY_M,
    outer ? MAP_OUTER_ENDPOINT_FORWARD_UNCERTAINTY_M
          : MAP_INNER_ENDPOINT_FORWARD_UNCERTAINTY_M,
    outer ? MAP_OUTER_ENDPOINT_LATERAL_UNCERTAINTY_M
          : MAP_INNER_ENDPOINT_LATERAL_UNCERTAINTY_M,
    outer ? MAP_OUTER_ENDPOINT_DYNAMIC_EVIDENCE
          : MAP_INNER_ENDPOINT_DYNAMIC_EVIDENCE,
    outer ? MAP_OUTER_ENDPOINT_STATIC_EVIDENCE
          : MAP_INNER_ENDPOINT_STATIC_EVIDENCE);
}

static void transformRobotPoint(float localX, float localY, float headingRad,
                                float &worldX, float &worldY) {
  // Robot convention: +X forward, +Y left. This is the standard 2D rigid-body
  // transform from a point fixed to the chassis into the odometry world frame.
  worldX = robotX + localX * cosf(headingRad) - localY * sinf(headingRad);
  worldY = robotY + localX * sinf(headingRad) + localY * cosf(headingRad);
}

static float navigationHeadingRad() {
  return navigationHeadingDeg() * DEG_TO_RAD;
}

static void updateRearObstacleMapEvidence(float headingRad) {
  const uint32_t frameSequence = getRearObstacleFrameSequence();
  if (frameSequence == 0 || frameSequence == lastRearEvidenceFrameSequence) {
    return;
  }
  lastRearEvidenceFrameSequence = frameSequence;

  float sensorWorldX;
  float sensorWorldY;
  transformRobotPoint(REAR_MATRIX_TOF_GEOMETRY.xMm / 1000.0f,
                      REAR_MATRIX_TOF_GEOMETRY.yMm / 1000.0f,
                      headingRad, sensorWorldX, sensorWorldY);
  const float halfColumnAngleRad =
    (REAR_MATRIX_TOF_HORIZONTAL_FOV_DEG /
     REAR_MATRIX_TOF_COLUMN_COUNT * 0.5f) * DEG_TO_RAD;

  for (uint8_t column = 0; column < REAR_MATRIX_TOF_COLUMN_COUNT; column++) {
    uint16_t distanceMm;
    float robotAngleDeg;
    if (!getRearObstacleRay(column, distanceMm, robotAngleDeg)) {
      continue;
    }

    const float rangeM = distanceMm / 1000.0f;
    const float rayHeadingRad = headingRad + robotAngleDeg * DEG_TO_RAD;
    const float rangeUncertaintyM = max(
      MAP_REAR_ENDPOINT_MIN_RANGE_UNCERTAINTY_M,
      rangeM * MAP_REAR_ENDPOINT_RANGE_UNCERTAINTY_RATIO);
    const float freeLengthM = max(0.0f, rangeM - rangeUncertaintyM);
    markFreeRayEvidence(sensorWorldX, sensorWorldY, rayHeadingRad,
                        freeLengthM, MAP_REAR_FREE_RAY_HALF_WIDTH_M,
                        MAP_REAR_FREE_EVIDENCE);

    const float endpointX = sensorWorldX + cosf(rayHeadingRad) * rangeM;
    const float endpointY = sensorWorldY + sinf(rayHeadingRad) * rangeM;
    const float lateralUncertaintyM = max(
      MAP_REAR_ENDPOINT_MIN_LATERAL_UNCERTAINTY_M,
      rangeM * tanf(halfColumnAngleRad));
    markDirectionalEndpointEvidence(
      endpointX, endpointY, rayHeadingRad,
      rangeUncertaintyM, rangeUncertaintyM, lateralUncertaintyM,
      MAP_REAR_ENDPOINT_DYNAMIC_EVIDENCE,
      MAP_REAR_ENDPOINT_STATIC_EVIDENCE);
  }
}

void clearLocalMap() {
  // Used by ZERO and mission initialisation. It clears both map layers but
  // intentionally leaves pose ownership to the caller.
  initialiseMapAtRobot();
  initialiseArenaMemoryAtRobot();
  lastRearEvidenceFrameSequence = 0;
  plannerTelemetry.replanReason = "map_cleared";
}

void updateLocalMapFromSensors() {
  // This is perception, not planning. It converts the latest forward fan and
  // rear matrix rays into short-lived evidence before planning asks whether
  // arcs are safe.
  recenterLocalMapIfNeeded();
  decayLocalMap();

  const float headingRad = navigationHeadingRad();
  for (int i = RANGE_RIGHT_OUTER; i <= RANGE_LEFT_OUTER; i++) {
    RangeSensorId id = (RangeSensorId)i;
    if (!isRangeSensorValid(id)) {
      continue;
    }

    const FanSensorGeometry &sensor = FAN_SENSOR_GEOMETRY[i];
    float sensorWorldX;
    float sensorWorldY;
    transformRobotPoint(sensor.xMm / 1000.0, sensor.yMm / 1000.0,
                        headingRad, sensorWorldX, sensorWorldY);

    float rayHeading = headingRad + sensor.angleDeg * DEG_TO_RAD;
    float rangeM = getRangeSensorDistance(id) / 1000.0;
    bool outer = isOuterFanSensor(id);
    float endpointBackUncertaintyM = outer ? MAP_OUTER_ENDPOINT_BACK_UNCERTAINTY_M
                                           : MAP_INNER_ENDPOINT_BACK_UNCERTAINTY_M;
    // Space before a range return is evidence of free travel. Stop short of
    // the return by the range-direction uncertainty so we never write free
    // evidence through the obstacle itself.
    float freeLengthM = max(0.0f, rangeM - endpointBackUncertaintyM);

    // Only a narrow strip around the beam is marked free. Treating the full
    // ToF cone as free would invent visibility where an edge return could
    // hide a wall or another obstacle.
    markFreeRayEvidence(sensorWorldX, sensorWorldY, rayHeading, freeLengthM,
                        MAP_FREE_RAY_HALF_WIDTH_M, 8);

    float endpointX = sensorWorldX + cosf(rayHeading) * rangeM;
    float endpointY = sensorWorldY + sinf(rayHeading) * rangeM;
    // The endpoint becomes obstacle evidence after the free ray is drawn. The
    // ordering matters: the ray must not erase the object it just measured.
    markEndpointEvidence(id, endpointX, endpointY, rayHeading);
  }

  updateRearObstacleMapEvidence(headingRad);

}

void markTraversedFreeSpace() {
  // The robot's own footprint is durable clear evidence. Include the hard
  // planning margin where it has no contradictory obstacle evidence so the
  // same validated corridor remains usable for a later reverse traversal.
  recenterLocalMapIfNeeded();
  const float headingRad = navigationHeadingRad();
  const float actualFront = ROBOT_FOOTPRINT_GEOMETRY.frontExtentMm / 1000.0f;
  const float actualRear = ROBOT_FOOTPRINT_GEOMETRY.rearExtentMm / 1000.0f;
  const float actualLeft = ROBOT_FOOTPRINT_GEOMETRY.leftExtentMm / 1000.0f;
  const float actualRight = ROBOT_FOOTPRINT_GEOMETRY.rightExtentMm / 1000.0f;
  const float sampleStepM = LOCAL_MAP_CELL_M * 0.5f;
  for (float localX = -actualRear - PLANNER_TOTAL_HARD_CLEARANCE_M;
       localX <= actualFront + PLANNER_TOTAL_HARD_CLEARANCE_M + 0.001f;
       localX += sampleStepM) {
    for (float localY = -actualRight - PLANNER_TOTAL_HARD_CLEARANCE_M;
         localY <= actualLeft + PLANNER_TOTAL_HARD_CLEARANCE_M + 0.001f;
         localY += sampleStepM) {
      float worldX;
      float worldY;
      transformRobotPoint(localX, localY, headingRad, worldX, worldY);
      int x;
      int y;
      if (worldToCell(worldX, worldY, x, y)) {
        LocalMapCell &cell = localMap[y][x];
        bool insideActualFootprint =
          localX >= -actualRear && localX <= actualFront &&
          localY >= -actualRight && localY <= actualLeft;
        bool occupied = cell.staticEvidence >= PLANNER_OBSTACLE_SCORE_THRESHOLD ||
                        cell.dynamicEvidence >= PLANNER_OBSTACLE_SCORE_THRESHOLD;
        if (!insideActualFootprint && occupied) {
          continue;
        }
        cell.traversedEvidence = 255;
        cell.freeEvidence = 100;
        if (insideActualFootprint) {
          cell.dynamicEvidence = 0;
          markArenaMemoryClear(worldX, worldY);
        }
        cell.lastTraversedMs = millis();
      }
    }
  }
}

static bool cellOccupied(int cellX, int cellY) {
  if (cellX < 0 || cellX >= LOCAL_MAP_CELLS || cellY < 0 || cellY >= LOCAL_MAP_CELLS) {
    return true;
  }

  const LocalMapCell &cell = localMap[cellY][cellX];
  // Free evidence is intentionally not a hard permission. A cell is safe only
  // because no sufficiently strong obstacle evidence currently contradicts it.
  float worldX = localMapOriginX + (cellX + 0.5f) * LOCAL_MAP_CELL_M;
  float worldY = localMapOriginY + (cellY + 0.5f) * LOCAL_MAP_CELL_M;
  bool locallyOccupied =
    cell.staticEvidence >= PLANNER_OBSTACLE_SCORE_THRESHOLD ||
    cell.dynamicEvidence >= PLANNER_OBSTACLE_SCORE_THRESHOLD;
  bool freshlyClear = !locallyOccupied &&
    cell.freeEvidence >= PLANNER_REVERSE_CLEAR_EVIDENCE_THRESHOLD;
  return locallyOccupied ||
         (!freshlyClear &&
          arenaMemoryStateAtWorld(worldX, worldY) == ARENA_MEMORY_OCCUPIED);
}

int plannerDebugMapState(float worldX, float worldY) {
  int cellX;
  int cellY;
  ArenaMemoryState arenaState = arenaMemoryStateAtWorld(worldX, worldY);
  if (!worldToCell(worldX, worldY, cellX, cellY)) {
    if (arenaState == ARENA_MEMORY_OCCUPIED) return 2;
    if (arenaState == ARENA_MEMORY_CLEAR) return 1;
    return 0;
  }
  if (cellOccupied(cellX, cellY)) return 2;
  return localMap[cellY][cellX].freeEvidence >=
           PLANNER_REVERSE_CLEAR_EVIDENCE_THRESHOLD ||
         arenaState == ARENA_MEMORY_CLEAR
    ? 1 : 0;
}

int plannerDebugSeedMapOccupied(float worldX, float worldY) {
  int cellX;
  int cellY;
  if (!worldToCell(worldX, worldY, cellX, cellY)) {
    return 0;
  }
  LocalMapCell &cell = localMap[cellY][cellX];
  cell.dynamicEvidence = PLANNER_OBSTACLE_SCORE_THRESHOLD;
  cell.staticEvidence = PLANNER_OBSTACLE_SCORE_THRESHOLD;
  cell.freeEvidence = 0;
  cell.lastObservedMs = millis();
  addArenaMemoryEvidence(worldX, worldY, ARENA_MEMORY_EVIDENCE_THRESHOLD);
  return 1;
}

static void capturePlannerCollisionSnapshot(PlannerCollisionSnapshot &snapshot) {
  snapshot.originX = localMapOriginX;
  snapshot.originY = localMapOriginY;
  memset(snapshot.occupied, 0, sizeof(snapshot.occupied));
  memset(snapshot.knownClear, 0, sizeof(snapshot.knownClear));
  for (int y = 0; y < LOCAL_MAP_CELLS; y++) {
    for (int x = 0; x < LOCAL_MAP_CELLS; x++) {
      int bit = y * LOCAL_MAP_CELLS + x;
      if (cellOccupied(x, y)) {
        snapshot.occupied[bit >> 3] |= (uint8_t)(1U << (bit & 7));
      } else {
        float worldX = localMapOriginX + (x + 0.5f) * LOCAL_MAP_CELL_M;
        float worldY = localMapOriginY + (y + 0.5f) * LOCAL_MAP_CELL_M;
        if (localMap[y][x].freeEvidence >=
              PLANNER_REVERSE_CLEAR_EVIDENCE_THRESHOLD ||
            arenaMemoryStateAtWorld(worldX, worldY) == ARENA_MEMORY_CLEAR) {
          snapshot.knownClear[bit >> 3] |= (uint8_t)(1U << (bit & 7));
        }
      }
    }
  }
}

static int fastFloorToInt(float value) {
  int truncated = (int)value;
  return value < (float)truncated ? truncated - 1 : truncated;
}

static bool snapshotWorldOccupied(const PlannerCollisionSnapshot &snapshot,
                                  float worldX, float worldY,
                                  int *cellX = NULL, int *cellY = NULL) {
  int x = fastFloorToInt((worldX - snapshot.originX) / LOCAL_MAP_CELL_M);
  int y = fastFloorToInt((worldY - snapshot.originY) / LOCAL_MAP_CELL_M);
  if (cellX != NULL) {
    *cellX = x;
  }
  if (cellY != NULL) {
    *cellY = y;
  }
  if (x < 0 || x >= LOCAL_MAP_CELLS || y < 0 || y >= LOCAL_MAP_CELLS) {
    return true;
  }
  int bit = y * LOCAL_MAP_CELLS + x;
  return (snapshot.occupied[bit >> 3] & (uint8_t)(1U << (bit & 7))) != 0;
}

static bool snapshotWorldKnownClear(const PlannerCollisionSnapshot &snapshot,
                                    float worldX, float worldY) {
  int x = fastFloorToInt((worldX - snapshot.originX) / LOCAL_MAP_CELL_M);
  int y = fastFloorToInt((worldY - snapshot.originY) / LOCAL_MAP_CELL_M);
  if (x < 0 || x >= LOCAL_MAP_CELLS || y < 0 || y >= LOCAL_MAP_CELLS) {
    return false;
  }
  int bit = y * LOCAL_MAP_CELLS + x;
  return (snapshot.knownClear[bit >> 3] &
          (uint8_t)(1U << (bit & 7))) != 0;
}

struct InflatedFootprintSampleGrid {
  float minimumLocalX;
  float maximumLocalX;
  float minimumLocalY;
  float maximumLocalY;
  float headingCos;
  float headingSin;
  int xIntervals;
  int yIntervals;
};

static void buildInflatedFootprintSampleGrid(
    float headingRad, InflatedFootprintSampleGrid &grid) {
  const float frontM = ROBOT_FOOTPRINT_GEOMETRY.frontExtentMm / 1000.0f +
                       PLANNER_TOTAL_HARD_CLEARANCE_M;
  const float rearM = ROBOT_FOOTPRINT_GEOMETRY.rearExtentMm / 1000.0f +
                      PLANNER_TOTAL_HARD_CLEARANCE_M;
  const float leftM = ROBOT_FOOTPRINT_GEOMETRY.leftExtentMm / 1000.0f +
                      PLANNER_TOTAL_HARD_CLEARANCE_M;
  const float rightM = ROBOT_FOOTPRINT_GEOMETRY.rightExtentMm / 1000.0f +
                       PLANNER_TOTAL_HARD_CLEARANCE_M;
  grid.minimumLocalX = -rearM;
  grid.maximumLocalX = frontM;
  grid.minimumLocalY = -rightM;
  grid.maximumLocalY = leftM;
  grid.headingCos = cosf(headingRad);
  grid.headingSin = sinf(headingRad);
  grid.xIntervals = max(1, (int)ceilf(
    (frontM + rearM) / LOCAL_MAP_CELL_M));
  grid.yIntervals = max(1, (int)ceilf(
    (leftM + rightM) / LOCAL_MAP_CELL_M));
}

static void inflatedFootprintSampleWorld(
    const InflatedFootprintSampleGrid &grid,
    float worldX, float worldY, int xIndex, int yIndex,
    float &sampleWorldX, float &sampleWorldY) {
  const float localX = grid.minimumLocalX +
    (grid.maximumLocalX - grid.minimumLocalX) *
      ((float)xIndex / grid.xIntervals);
  const float localY = grid.minimumLocalY +
    (grid.maximumLocalY - grid.minimumLocalY) *
      ((float)yIndex / grid.yIntervals);
  sampleWorldX = worldX + localX * grid.headingCos -
                 localY * grid.headingSin;
  sampleWorldY = worldY + localX * grid.headingSin +
                 localY * grid.headingCos;
}

static bool footprintKnownClearOnSnapshot(
    const PlannerCollisionSnapshot &snapshot,
    float worldX, float worldY, float headingRad) {
  InflatedFootprintSampleGrid grid;
  buildInflatedFootprintSampleGrid(headingRad, grid);
  for (int xIndex = 0; xIndex <= grid.xIntervals; ++xIndex) {
    for (int yIndex = 0; yIndex <= grid.yIntervals; ++yIndex) {
      float sampleWorldX;
      float sampleWorldY;
      inflatedFootprintSampleWorld(
        grid, worldX, worldY, xIndex, yIndex,
        sampleWorldX, sampleWorldY);
      if (!snapshotWorldKnownClear(
            snapshot, sampleWorldX, sampleWorldY)) {
        return false;
      }
    }
  }
  return true;
}

static float footprintUnknownFractionOnSnapshot(
    const PlannerCollisionSnapshot &snapshot,
    float worldX, float worldY, float headingRad) {
  InflatedFootprintSampleGrid grid;
  buildInflatedFootprintSampleGrid(headingRad, grid);
  int sampleCount = 0;
  int unknownCount = 0;
  for (int xIndex = 0; xIndex <= grid.xIntervals; ++xIndex) {
    for (int yIndex = 0; yIndex <= grid.yIntervals; ++yIndex) {
      float sampleWorldX;
      float sampleWorldY;
      inflatedFootprintSampleWorld(
        grid, worldX, worldY, xIndex, yIndex,
        sampleWorldX, sampleWorldY);
      sampleCount++;
      if (!snapshotWorldKnownClear(
            snapshot, sampleWorldX, sampleWorldY)) {
        unknownCount++;
      }
    }
  }
  return sampleCount > 0
    ? (float)unknownCount / (float)sampleCount : 1.0f;
}

static float currentReverseUnknownAllowance() {
  if (!reverseRecoveryActive || reverseRecoveryStartedMs == 0) {
    return 0.0f;
  }
  float ramp = constrain(
    (millis() - reverseRecoveryStartedMs) /
      (float)PLANNER_REVERSE_UNKNOWN_RAMP_MS,
    0.0f, 1.0f);
  return ramp * PLANNER_REVERSE_MAX_UNKNOWN_FRACTION;
}

static float poseKnownClearanceM(const PlannerCollisionSnapshot &snapshot,
                                 float worldX, float worldY,
                                 float headingRad) {
  const float front = ROBOT_FOOTPRINT_GEOMETRY.frontExtentMm / 1000.0f +
                      PLANNER_TOTAL_HARD_CLEARANCE_M;
  const float rear = ROBOT_FOOTPRINT_GEOMETRY.rearExtentMm / 1000.0f +
                     PLANNER_TOTAL_HARD_CLEARANCE_M;
  const float left = ROBOT_FOOTPRINT_GEOMETRY.leftExtentMm / 1000.0f +
                     PLANNER_TOTAL_HARD_CLEARANCE_M;
  const float right = ROBOT_FOOTPRINT_GEOMETRY.rightExtentMm / 1000.0f +
                      PLANNER_TOTAL_HARD_CLEARANCE_M;
  const float headingCos = cosf(headingRad);
  const float headingSin = sinf(headingRad);
  float bestSquared = PLANNER_REVERSE_CLEARANCE_CAP_M *
                      PLANNER_REVERSE_CLEARANCE_CAP_M;
  for (int cellY = 0; cellY < LOCAL_MAP_CELLS; ++cellY) {
    for (int cellX = 0; cellX < LOCAL_MAP_CELLS; ++cellX) {
      int bit = cellY * LOCAL_MAP_CELLS + cellX;
      if ((snapshot.occupied[bit >> 3] &
           (uint8_t)(1U << (bit & 7))) == 0) {
        continue;
      }
      float cellWorldX = snapshot.originX +
        (cellX + 0.5f) * LOCAL_MAP_CELL_M;
      float cellWorldY = snapshot.originY +
        (cellY + 0.5f) * LOCAL_MAP_CELL_M;
      float dx = cellWorldX - worldX;
      float dy = cellWorldY - worldY;
      float localX = dx * headingCos + dy * headingSin;
      float localY = -dx * headingSin + dy * headingCos;
      float outsideX = localX < -rear ? -rear - localX
        : (localX > front ? localX - front : 0.0f);
      float outsideY = localY < -right ? -right - localY
        : (localY > left ? localY - left : 0.0f);
      float distanceSquared = outsideX * outsideX + outsideY * outsideY;
      if (distanceSquared < bestSquared) {
        bestSquared = distanceSquared;
      }
    }
  }
  float clearanceM = sqrtf(bestSquared) -
    LOCAL_MAP_CELL_M * 0.70710678f;
  return constrain(clearanceM, 0.0f, PLANNER_REVERSE_CLEARANCE_CAP_M);
}

static float rotationalEnvelopeClearanceM(
    const PlannerCollisionSnapshot &snapshot,
    float worldX, float worldY) {
  const float front = ROBOT_FOOTPRINT_GEOMETRY.frontExtentMm / 1000.0f +
                      PLANNER_TOTAL_HARD_CLEARANCE_M;
  const float rear = ROBOT_FOOTPRINT_GEOMETRY.rearExtentMm / 1000.0f +
                     PLANNER_TOTAL_HARD_CLEARANCE_M;
  const float left = ROBOT_FOOTPRINT_GEOMETRY.leftExtentMm / 1000.0f +
                     PLANNER_TOTAL_HARD_CLEARANCE_M;
  const float right = ROBOT_FOOTPRINT_GEOMETRY.rightExtentMm / 1000.0f +
                      PLANNER_TOTAL_HARD_CLEARANCE_M;
  const float sweepRadiusM = max(
    max(hypotf(front, left), hypotf(front, right)),
    max(hypotf(rear, left), hypotf(rear, right)));
  float bestM = PLANNER_REVERSE_CLEARANCE_CAP_M;
  for (int cellY = 0; cellY < LOCAL_MAP_CELLS; ++cellY) {
    for (int cellX = 0; cellX < LOCAL_MAP_CELLS; ++cellX) {
      int bit = cellY * LOCAL_MAP_CELLS + cellX;
      if ((snapshot.occupied[bit >> 3] &
           (uint8_t)(1U << (bit & 7))) == 0) {
        continue;
      }
      float cellWorldX = snapshot.originX +
        (cellX + 0.5f) * LOCAL_MAP_CELL_M;
      float cellWorldY = snapshot.originY +
        (cellY + 0.5f) * LOCAL_MAP_CELL_M;
      float centreDistanceM = hypotf(cellWorldX - worldX,
                                     cellWorldY - worldY);
      bestM = min(bestM, centreDistanceM - sweepRadiusM -
                         LOCAL_MAP_CELL_M * 0.70710678f);
    }
  }
  return constrain(bestM, 0.0f, PLANNER_REVERSE_CLEARANCE_CAP_M);
}

static float projectedUnexploredScore(
    const PlannerCollisionSnapshot &snapshot,
    float worldX, float worldY, float headingRad) {
  int unknownCount = 0;
  int sampleCount = 0;
  for (int sensorIndex = RANGE_RIGHT_OUTER;
       sensorIndex <= RANGE_LEFT_OUTER; ++sensorIndex) {
    const FanSensorGeometry &sensor = FAN_SENSOR_GEOMETRY[sensorIndex];
    float sensorX = worldX +
      (sensor.xMm / 1000.0f) * cosf(headingRad) -
      (sensor.yMm / 1000.0f) * sinf(headingRad);
    float sensorY = worldY +
      (sensor.xMm / 1000.0f) * sinf(headingRad) +
      (sensor.yMm / 1000.0f) * cosf(headingRad);
    float rayHeading = headingRad + sensor.angleDeg * DEG_TO_RAD;
    for (float distanceM = LOCAL_MAP_CELL_M;
         distanceM <= PLANNER_HORIZON_S;
         distanceM += LOCAL_MAP_CELL_M) {
      float x = sensorX + cosf(rayHeading) * distanceM;
      float y = sensorY + sinf(rayHeading) * distanceM;
      if (snapshotWorldOccupied(snapshot, x, y)) {
        break;
      }
      sampleCount++;
      if (!snapshotWorldKnownClear(snapshot, x, y)) {
        unknownCount++;
      }
    }
  }
  return sampleCount > 0
    ? constrain((float)unknownCount / sampleCount, 0.0f, 1.0f)
    : 0.0f;
}

static bool footprintClearOnSnapshot(const PlannerCollisionSnapshot &snapshot,
                                     float worldX, float worldY,
                                     float headingRad) {
  // Occupancy evidence is represented at map-cell centres; obstacle envelope
  // targets add the separate half-cell uncertainty allowance. Test every
  // occupied centre inside the complete inflated oriented rectangle so no
  // asymmetric edge is skipped and map-cell width is not counted twice.
  InflatedFootprintSampleGrid grid;
  buildInflatedFootprintSampleGrid(headingRad, grid);
  const float localCentreX =
    (grid.minimumLocalX + grid.maximumLocalX) * 0.5f;
  const float localCentreY =
    (grid.minimumLocalY + grid.maximumLocalY) * 0.5f;
  const float centreX = worldX + localCentreX * grid.headingCos -
                        localCentreY * grid.headingSin;
  const float centreY = worldY + localCentreX * grid.headingSin +
                        localCentreY * grid.headingCos;
  const float halfLengthM =
    (grid.maximumLocalX - grid.minimumLocalX) * 0.5f;
  const float halfWidthM =
    (grid.maximumLocalY - grid.minimumLocalY) * 0.5f;
  const float worldHalfWidthM =
    halfLengthM * fabsf(grid.headingCos) +
    halfWidthM * fabsf(grid.headingSin);
  const float worldHalfHeightM =
    halfLengthM * fabsf(grid.headingSin) +
    halfWidthM * fabsf(grid.headingCos);
  const float mapMaxX = snapshot.originX + LOCAL_MAP_SIZE_M;
  const float mapMaxY = snapshot.originY + LOCAL_MAP_SIZE_M;
  if (centreX - worldHalfWidthM < snapshot.originX ||
      centreX + worldHalfWidthM > mapMaxX ||
      centreY - worldHalfHeightM < snapshot.originY ||
      centreY + worldHalfHeightM > mapMaxY) {
    return false;
  }
  int minCellX = max(0, fastFloorToInt(
    (centreX - worldHalfWidthM - snapshot.originX) / LOCAL_MAP_CELL_M));
  int maxCellX = min(LOCAL_MAP_CELLS - 1, fastFloorToInt(
    (centreX + worldHalfWidthM - snapshot.originX) / LOCAL_MAP_CELL_M));
  int minCellY = max(0, fastFloorToInt(
    (centreY - worldHalfHeightM - snapshot.originY) / LOCAL_MAP_CELL_M));
  int maxCellY = min(LOCAL_MAP_CELLS - 1, fastFloorToInt(
    (centreY + worldHalfHeightM - snapshot.originY) / LOCAL_MAP_CELL_M));
  for (int cellY = minCellY; cellY <= maxCellY; ++cellY) {
    for (int cellX = minCellX; cellX <= maxCellX; ++cellX) {
      int bit = cellY * LOCAL_MAP_CELLS + cellX;
      if ((snapshot.occupied[bit >> 3] &
           (uint8_t)(1U << (bit & 7))) == 0) {
        continue;
      }
      float cellWorldX =
        snapshot.originX + (cellX + 0.5f) * LOCAL_MAP_CELL_M;
      float cellWorldY =
        snapshot.originY + (cellY + 0.5f) * LOCAL_MAP_CELL_M;
      float dx = cellWorldX - worldX;
      float dy = cellWorldY - worldY;
      float localX = dx * grid.headingCos + dy * grid.headingSin;
      float localY = -dx * grid.headingSin + dy * grid.headingCos;
      if (localX >= grid.minimumLocalX &&
          localX <= grid.maximumLocalX &&
          localY >= grid.minimumLocalY &&
          localY <= grid.maximumLocalY) {
        lastFootprintRejectWorldX = cellWorldX;
        lastFootprintRejectWorldY = cellWorldY;
        lastFootprintRejectCellX = cellX;
        lastFootprintRejectCellY = cellY;
        return false;
      }
    }
  }
  return true;
}

static bool turnSegmentFootprintClear(
    const PlannerCollisionSnapshot &snapshot,
    float worldX, float worldY, float startHeadingDeg,
    float direction, float sweepDeg) {
  int intervals = max(1, (int)ceilf(
    sweepDeg / PLANNER_EMERGENCY_SCAN_SWEEP_STEP_DEG));
  for (int interval = 0; interval <= intervals; ++interval) {
    float progressDeg = sweepDeg * ((float)interval / intervals);
    float headingRad = wrapAngle(
      startHeadingDeg + direction * progressDeg) * DEG_TO_RAD;
    if (!footprintClearOnSnapshot(snapshot, worldX, worldY, headingRad)) {
      return false;
    }
  }
  return true;
}

static float lateralObstacleDistanceOnSnapshot(
    const PlannerCollisionSnapshot &snapshot,
    float worldX, float worldY, float lateralX, float lateralY) {
  for (float distanceM = 0.0f;
       distanceM <= PLANNER_CORRIDOR_SIDE_SEARCH_M;
       distanceM += LOCAL_MAP_CELL_M * 0.5f) {
    if (snapshotWorldOccupied(snapshot,
                              worldX + lateralX * distanceM,
                              worldY + lateralY * distanceM)) {
      return distanceM;
    }
  }
  return -1.0f;
}

static bool isNarrowObservedCorridorOnSnapshot(
    const PlannerCollisionSnapshot &snapshot,
    float worldX, float worldY, float headingRad) {
  const float lateralX = -sinf(headingRad);
  const float lateralY = cosf(headingRad);
  float leftBoundaryM = lateralObstacleDistanceOnSnapshot(
    snapshot, worldX, worldY, lateralX, lateralY);
  float rightBoundaryM = lateralObstacleDistanceOnSnapshot(
    snapshot, worldX, worldY, -lateralX, -lateralY);
  bool narrow = leftBoundaryM >= 0.0f && rightBoundaryM >= 0.0f &&
                leftBoundaryM + rightBoundaryM <= PLANNER_CORRIDOR_MAX_WIDTH_M;
  if (narrow) {
    lastCorridorRejectLeftM = leftBoundaryM;
    lastCorridorRejectRightM = rightBoundaryM;
  }
  return narrow;
}

static void resetObstacleContext(const char* reason) {
  bool wasActive = obstacleContext.active;
  obstacleContext = {};
  if (wasActive) {
    plannerTelemetry.replanReason = reason;
    sendBluetoothEvent("obstacle_context_clear", reason);
  }
}

static void obstacleCellRoutePosition(int cellX, int cellY,
                                      float originX, float originY,
                                      float routeUx, float routeUy,
                                      float &alongM, float &lateralM) {
  float worldX = localMapOriginX + (cellX + 0.5f) * LOCAL_MAP_CELL_M;
  float worldY = localMapOriginY + (cellY + 0.5f) * LOCAL_MAP_CELL_M;
  float dx = worldX - originX;
  float dy = worldY - originY;
  alongM = dx * routeUx + dy * routeUy;
  lateralM = -dx * routeUy + dy * routeUx;
}

static void growObstacleEnvelopeFromNearbyEvidence(
    float originX, float originY,
    float routeUx, float routeUy,
    float routeLengthM,
    ObstacleEnvelope &envelope) {
  if (!envelope.found) {
    return;
  }

  // Side-facing rays observe a long wall in separate endpoint patches. Grow
  // from the retained envelope so a newly observed patch can extend the same
  // obstacle even when one 50 mm map row between them has no endpoint return.
  const float joinM = LOCAL_MAP_CELL_M * 2.75f;
  const float endpointReachM = routeLengthM +
                               ROBOT_FOOTPRINT_GEOMETRY.frontExtentMm / 1000.0f +
                               PLANNER_TOTAL_HARD_CLEARANCE_M;
  for (int pass = 0; pass < 12; pass++) {
    bool expanded = false;
    for (int y = 0; y < LOCAL_MAP_CELLS; y++) {
      for (int x = 0; x < LOCAL_MAP_CELLS; x++) {
        if (!cellOccupied(x, y)) {
          continue;
        }
        float alongM;
        float lateralM;
        obstacleCellRoutePosition(x, y, originX, originY,
                                  routeUx, routeUy, alongM, lateralM);
        if (alongM < -joinM || alongM > endpointReachM + joinM ||
            alongM < envelope.nearAlongM - joinM ||
            alongM > envelope.farAlongM + joinM ||
            lateralM < envelope.minLateralM - joinM ||
            lateralM > envelope.maxLateralM + joinM) {
          continue;
        }
        float oldNear = envelope.nearAlongM;
        float oldFar = envelope.farAlongM;
        float oldMin = envelope.minLateralM;
        float oldMax = envelope.maxLateralM;
        envelope.nearAlongM = min(envelope.nearAlongM, alongM);
        envelope.farAlongM = max(envelope.farAlongM, alongM);
        envelope.minLateralM = min(envelope.minLateralM, lateralM);
        envelope.maxLateralM = max(envelope.maxLateralM, lateralM);
        expanded = expanded || oldNear != envelope.nearAlongM ||
                   oldFar != envelope.farAlongM ||
                   oldMin != envelope.minLateralM ||
                   oldMax != envelope.maxLateralM;
      }
    }
    if (!expanded) {
      break;
    }
  }
}

static bool scanRouteObstacle(float originX, float originY,
                              float routeUx, float routeUy,
                              float routeLengthM,
                              ObstacleEnvelope &envelope) {
  envelope = {};
  float halfWidthM = max(ROBOT_FOOTPRINT_GEOMETRY.leftExtentMm,
                         ROBOT_FOOTPRINT_GEOMETRY.rightExtentMm) / 1000.0f +
                     PLANNER_TOTAL_HARD_CLEARANCE_M +
                     LOCAL_MAP_CELL_M * 0.5f;
  float endpointReachM = routeLengthM +
                         ROBOT_FOOTPRINT_GEOMETRY.frontExtentMm / 1000.0f +
                         PLANNER_TOTAL_HARD_CLEARANCE_M;
  int seedX = -1;
  int seedY = -1;
  float nearestAlongM = 1000000.0f;
  float seedLateralM = 0.0f;

  for (int y = 0; y < LOCAL_MAP_CELLS; y++) {
    for (int x = 0; x < LOCAL_MAP_CELLS; x++) {
      if (!cellOccupied(x, y)) {
        continue;
      }
      float alongM;
      float lateralM;
      obstacleCellRoutePosition(x, y, originX, originY,
                                routeUx, routeUy, alongM, lateralM);
      if (alongM >= 0.0f && alongM <= endpointReachM &&
          fabs(lateralM) <= halfWidthM && alongM < nearestAlongM) {
        nearestAlongM = alongM;
        seedLateralM = lateralM;
        seedX = x;
        seedY = y;
      }
    }
  }
  if (seedX < 0 || seedY < 0) {
    return false;
  }

  envelope.found = true;
  envelope.nearAlongM = nearestAlongM;
  envelope.farAlongM = nearestAlongM;
  envelope.minLateralM = seedLateralM;
  envelope.maxLateralM = seedLateralM;
  growObstacleEnvelopeFromNearbyEvidence(
    originX, originY, routeUx, routeUy, routeLengthM, envelope);
  return true;
}

static float obstacleTargetMinimumLateralM() {
  return
    max(ROBOT_FOOTPRINT_GEOMETRY.leftExtentMm,
        ROBOT_FOOTPRINT_GEOMETRY.rightExtentMm) / 1000.0f +
    PLANNER_TOTAL_HARD_CLEARANCE_M + LOCAL_MAP_CELL_M * 0.5f;
}

static float obstacleTargetLateralM(float sideSign,
                                    float minimumLateralM,
                                    float maximumLateralM) {
  float lateralClearanceM = obstacleTargetMinimumLateralM();
  float targetLateralM = sideSign > 0.0f
    ? maximumLateralM + lateralClearanceM
    : minimumLateralM - lateralClearanceM;
  return sideSign > 0.0f
    ? max(targetLateralM, lateralClearanceM)
    : min(targetLateralM, -lateralClearanceM);
}

static void obstacleTargetWorldPosition(
    float routeUx, float routeUy,
    float targetAlongM, float targetLateralM,
    float &targetX, float &targetY) {
  targetX = obstacleContext.originX +
            routeUx * targetAlongM - routeUy * targetLateralM;
  targetY = obstacleContext.originY +
            routeUy * targetAlongM + routeUx * targetLateralM;
}

static bool obstacleTargetPoseClear(
    const PlannerCollisionSnapshot &snapshot,
    float routeUx, float routeUy,
    float targetAlongM, float targetLateralM) {
  float targetX;
  float targetY;
  obstacleTargetWorldPosition(
    routeUx, routeUy, targetAlongM, targetLateralM,
    targetX, targetY);
  float approachHeadingRad = atan2f(targetY - robotY,
                                    targetX - robotX);
  return footprintClearOnSnapshot(
    snapshot, targetX, targetY, approachHeadingRad);
}

static bool chooseInsideBandTargetLateral(
    const PlannerCollisionSnapshot &snapshot,
    float routeUx, float routeUy, float sideSign,
    float targetAlongM, float nominalTargetLateralM,
  float &selectedTargetLateralM) {
  selectedTargetLateralM = nominalTargetLateralM;
  if (obstacleTargetPoseClear(
        snapshot, routeUx, routeUy,
        targetAlongM, nominalTargetLateralM)) {
    return true;
  }

  const float minimumSignedLateralM =
    obstacleTargetMinimumLateralM();
  const float nominalSignedLateralM =
    sideSign * nominalTargetLateralM;
  const float maximumInwardShiftM =
    nominalSignedLateralM - minimumSignedLateralM;
  if (maximumInwardShiftM <= 0.0f) {
    return false;
  }

  const float sampleStepM = LOCAL_MAP_CELL_M * 0.5f;
  const int sampleCount = max(
    1, (int)ceilf(maximumInwardShiftM / sampleStepM));
  bool foundSafeBand = false;
  float firstSafeInwardShiftM = 0.0f;
  float outerSafeLateralM = nominalTargetLateralM;
  float innerSafeLateralM = nominalTargetLateralM;
  for (int sample = 1; sample <= sampleCount; ++sample) {
    float inwardShiftM = min(
      maximumInwardShiftM, sample * sampleStepM);
    float candidateLateralM =
      nominalTargetLateralM - sideSign * inwardShiftM;
    bool clear = obstacleTargetPoseClear(
      snapshot, routeUx, routeUy,
      targetAlongM, candidateLateralM);
    if (!clear) {
      if (foundSafeBand) {
        break;
      }
      continue;
    }
    if (!foundSafeBand) {
      firstSafeInwardShiftM = inwardShiftM;
      outerSafeLateralM = candidateLateralM;
      foundSafeBand = true;
    }
    innerSafeLateralM = candidateLateralM;
  }
  if (!foundSafeBand) {
    return false;
  }
  if (firstSafeInwardShiftM <= sampleStepM + 0.001f) {
    return true;
  }

  float signedOuterM = sideSign * outerSafeLateralM;
  float signedInnerM = sideSign * innerSafeLateralM;
  float safeBandWidthM = max(0.0f, signedOuterM - signedInnerM);
  float insideBufferM = min(
    safeBandWidthM,
    PLANNER_PREFERRED_CLEARANCE_M * 0.5f);
  selectedTargetLateralM =
    innerSafeLateralM + sideSign * insideBufferM;
  return true;
}

static void obstacleSideEscapeTarget(
    const ObstacleEnvelope &envelope,
    float routeUx, float routeUy,
    float sideSign,
    float &targetX, float &targetY) {
  float frontClearM = ROBOT_FOOTPRINT_GEOMETRY.frontExtentMm / 1000.0f +
                      PLANNER_TOTAL_HARD_CLEARANCE_M +
                      LOCAL_MAP_CELL_M * 0.5f;
  float targetAlongM = max(0.05f, envelope.nearAlongM - frontClearM -
                                  PLANNER_OBSTACLE_TURN_ROOM_M);
  float targetLateralM = obstacleTargetLateralM(
    sideSign, envelope.minLateralM, envelope.maxLateralM);
  obstacleTargetWorldPosition(
    routeUx, routeUy, targetAlongM, targetLateralM,
    targetX, targetY);
}

static bool obstacleSideEscapeCorridorBlocked(
    const PlannerCollisionSnapshot &snapshot,
    const ObstacleEnvelope &envelope,
    float routeUx, float routeUy, float sideSign) {
  float targetX;
  float targetY;
  obstacleSideEscapeTarget(
    envelope, routeUx, routeUy,
    sideSign, targetX, targetY);
  float dx = targetX - robotX;
  float dy = targetY - robotY;
  float distanceM = sqrtf(dx * dx + dy * dy);
  if (distanceM <= LOCAL_MAP_CELL_M) {
    return false;
  }
  float stepM = LOCAL_MAP_CELL_M * 0.5f;
  for (float travelledM = stepM;
       travelledM <= distanceM + 0.001f;
       travelledM += stepM) {
    float progress = min(1.0f, travelledM / distanceM);
    if (snapshotWorldOccupied(
          snapshot,
          robotX + dx * progress,
          robotY + dy * progress)) {
      return true;
    }
  }
  return false;
}

static float chooseObstacleSide(const ObstacleEnvelope &envelope,
                                float routeUx, float routeUy) {
  float leftTargetM = obstacleTargetLateralM(
    1.0f, envelope.minLateralM, envelope.maxLateralM);
  float rightTargetM = obstacleTargetLateralM(
    -1.0f, envelope.minLateralM, envelope.maxLateralM);
  PlannerCollisionSnapshot collision;
  capturePlannerCollisionSnapshot(collision);
  bool leftCorridorBlocked = obstacleSideEscapeCorridorBlocked(
    collision, envelope, routeUx, routeUy, 1.0f);
  bool rightCorridorBlocked = obstacleSideEscapeCorridorBlocked(
    collision, envelope, routeUx, routeUy, -1.0f);
  if (leftCorridorBlocked != rightCorridorBlocked) {
    return leftCorridorBlocked ? -1.0f : 1.0f;
  }
  float dx = robotX - obstacleContext.originX;
  float dy = robotY - obstacleContext.originY;
  float currentLateralM = -dx * routeUy + dy * routeUx;
  float frontClearM = ROBOT_FOOTPRINT_GEOMETRY.frontExtentMm / 1000.0f +
                      PLANNER_TOTAL_HARD_CLEARANCE_M +
                      LOCAL_MAP_CELL_M * 0.5f;
  float targetAlongM = max(0.05f, envelope.nearAlongM - frontClearM -
                                  PLANNER_OBSTACLE_TURN_ROOM_M);
  float robotHeadingRad = robotTheta * PI / 180.0f;
  float headingX = cosf(robotHeadingRad);
  float headingY = sinf(robotHeadingRad);
  float leftForwardM =
    (routeUx * targetAlongM - routeUy * leftTargetM) * headingX +
    (routeUy * targetAlongM + routeUx * leftTargetM) * headingY;
  float rightForwardM =
    (routeUx * targetAlongM - routeUy * rightTargetM) * headingX +
    (routeUy * targetAlongM + routeUx * rightTargetM) * headingY;
  if (leftForwardM > 0.0f && rightForwardM <= 0.0f) {
    return 1.0f;
  }
  if (rightForwardM > 0.0f && leftForwardM <= 0.0f) {
    return -1.0f;
  }
  float leftCostM = fabs(leftTargetM - currentLateralM);
  float rightCostM = fabs(rightTargetM - currentLateralM);
  if (fabs(leftCostM - rightCostM) > LOCAL_MAP_CELL_M) {
    return leftCostM < rightCostM ? 1.0f : -1.0f;
  }
  float leftRangeM = isRangeSensorValid(RANGE_LEFT_INNER)
    ? getRangeSensorDistance(RANGE_LEFT_INNER) / 1000.0f : 0.0f;
  float rightRangeM = isRangeSensorValid(RANGE_RIGHT_INNER)
    ? getRangeSensorDistance(RANGE_RIGHT_INNER) / 1000.0f : 0.0f;
  return leftRangeM >= rightRangeM ? 1.0f : -1.0f;
}

static bool directWaypointCorridorClear(float targetX, float targetY) {
  float dx = targetX - robotX;
  float dy = targetY - robotY;
  float distanceM = sqrtf(dx * dx + dy * dy);
  if (distanceM <= WAYPOINT_TOLERANCE_M) {
    return true;
  }
  ObstacleEnvelope directObstacle;
  return !scanRouteObstacle(robotX, robotY, dx / distanceM, dy / distanceM,
                            distanceM, directObstacle);
}

static bool clipDirectSegmentToEnvelopeAxis(float start, float delta,
                                            float minimum, float maximum,
                                            float &entry, float &exit) {
  if (fabs(delta) < 0.000001f) {
    return start >= minimum && start <= maximum;
  }
  float first = (minimum - start) / delta;
  float second = (maximum - start) / delta;
  if (first > second) {
    float swap = first;
    first = second;
    second = swap;
  }
  entry = max(entry, first);
  exit = min(exit, second);
  return entry <= exit;
}

static bool directSegmentClearsRetainedObstacle(float targetX, float targetY) {
  float currentDx = robotX - obstacleContext.originX;
  float currentDy = robotY - obstacleContext.originY;
  float currentAlongM = currentDx * obstacleContext.routeUx +
                        currentDy * obstacleContext.routeUy;
  float currentLateralM = -currentDx * obstacleContext.routeUy +
                          currentDy * obstacleContext.routeUx;
  float targetDx = targetX - obstacleContext.originX;
  float targetDy = targetY - obstacleContext.originY;
  float targetAlongM = targetDx * obstacleContext.routeUx +
                       targetDy * obstacleContext.routeUy;
  float targetLateralM = -targetDx * obstacleContext.routeUy +
                         targetDy * obstacleContext.routeUx;
  float alongClearanceM =
    max(ROBOT_FOOTPRINT_GEOMETRY.frontExtentMm,
        ROBOT_FOOTPRINT_GEOMETRY.rearExtentMm) / 1000.0f +
    PLANNER_TOTAL_HARD_CLEARANCE_M + LOCAL_MAP_CELL_M * 0.5f;
  float lateralClearanceM =
    max(ROBOT_FOOTPRINT_GEOMETRY.leftExtentMm,
        ROBOT_FOOTPRINT_GEOMETRY.rightExtentMm) / 1000.0f +
    PLANNER_TOTAL_HARD_CLEARANCE_M + LOCAL_MAP_CELL_M * 0.5f;
  float entry = 0.0f;
  float exit = 1.0f;
  bool crossesAlong = clipDirectSegmentToEnvelopeAxis(
    currentAlongM, targetAlongM - currentAlongM,
    obstacleContext.nearAlongM - alongClearanceM,
    obstacleContext.farAlongM + alongClearanceM,
    entry, exit);
  bool crossesLateral = crossesAlong && clipDirectSegmentToEnvelopeAxis(
    currentLateralM, targetLateralM - currentLateralM,
    obstacleContext.minLateralM - lateralClearanceM,
    obstacleContext.maxLateralM + lateralClearanceM,
    entry, exit);
  return !crossesLateral;
}

static bool updateObstacleContext(float targetX, float targetY) {
  if (!obstacleContext.active) {
    float dx = targetX - robotX;
    float dy = targetY - robotY;
    float distanceM = sqrtf(dx * dx + dy * dy);
    if (distanceM <= WAYPOINT_TOLERANCE_M) {
      return false;
    }
    ObstacleEnvelope envelope;
    float routeUx = dx / distanceM;
    float routeUy = dy / distanceM;
    if (!scanRouteObstacle(robotX, robotY, routeUx, routeUy,
                           distanceM, envelope)) {
      return false;
    }
    obstacleContext.active = true;
    obstacleContext.originX = robotX;
    obstacleContext.originY = robotY;
    obstacleContext.routeUx = routeUx;
    obstacleContext.routeUy = routeUy;
    obstacleContext.routeLengthM = distanceM;
    // Envelope growth may join a perpendicular arena boundary behind us.
    // Keep the originally observed front face as the forward-phase gate.
    obstacleContext.approachNearAlongM = envelope.nearAlongM;
    obstacleContext.nearAlongM = envelope.nearAlongM;
    obstacleContext.farAlongM = envelope.farAlongM;
    obstacleContext.minLateralM = envelope.minLateralM;
    obstacleContext.maxLateralM = envelope.maxLateralM;
    obstacleContext.sideSign = chooseObstacleSide(envelope, routeUx, routeUy);
    obstacleContext.sideEscapeAlongM = 0.0f;
    obstacleContext.sideReconsidered = false;
    obstacleContext.clearSinceMs = 0;
    plannerTelemetry.replanReason = "obstacle_context_started";
    sendBluetoothEvent("obstacle_context_start",
                       obstacleContext.sideSign > 0.0f ? "left" : "right");
    resetPlannerEpoch();
    return true;
  }

  ObstacleEnvelope observed = {
    true,
    obstacleContext.nearAlongM,
    obstacleContext.farAlongM,
    obstacleContext.minLateralM,
    obstacleContext.maxLateralM
  };
  float previousNearAlongM = observed.nearAlongM;
  float previousFarAlongM = observed.farAlongM;
  float previousMinLateralM = observed.minLateralM;
  float previousMaxLateralM = observed.maxLateralM;
  growObstacleEnvelopeFromNearbyEvidence(
    obstacleContext.originX,
    obstacleContext.originY,
    obstacleContext.routeUx,
    obstacleContext.routeUy,
    obstacleContext.routeLengthM,
    observed);
  obstacleContext.nearAlongM = observed.nearAlongM;
  obstacleContext.farAlongM = observed.farAlongM;
  obstacleContext.minLateralM = observed.minLateralM;
  obstacleContext.maxLateralM = observed.maxLateralM;
  bool envelopeExpanded =
    observed.nearAlongM != previousNearAlongM ||
    observed.farAlongM != previousFarAlongM ||
    observed.minLateralM != previousMinLateralM ||
    observed.maxLateralM != previousMaxLateralM;
  if (envelopeExpanded && !obstacleContext.sideReconsidered) {
    PlannerCollisionSnapshot collision;
    capturePlannerCollisionSnapshot(collision);
    bool currentSideBlocked = obstacleSideEscapeCorridorBlocked(
      collision, observed,
      obstacleContext.routeUx, obstacleContext.routeUy,
      obstacleContext.sideSign);
    bool oppositeSideBlocked = obstacleSideEscapeCorridorBlocked(
      collision, observed,
      obstacleContext.routeUx, obstacleContext.routeUy,
      -obstacleContext.sideSign);
    if (currentSideBlocked && !oppositeSideBlocked) {
      obstacleContext.sideSign = -obstacleContext.sideSign;
      float switchDx = robotX - obstacleContext.originX;
      float switchDy = robotY - obstacleContext.originY;
      obstacleContext.sideEscapeAlongM =
        switchDx * obstacleContext.routeUx +
        switchDy * obstacleContext.routeUy;
      obstacleContext.sideReconsidered = true;
      obstacleContext.clearSinceMs = 0;
      plannerTelemetry.replanReason = "obstacle_side_infeasible";
      sendBluetoothEvent(
        "obstacle_side_switch",
        obstacleContext.sideSign > 0.0f ? "left" : "right");
      resetPlannerEpoch();
    }
  }

  float poseDx = robotX - obstacleContext.originX;
  float poseDy = robotY - obstacleContext.originY;
  float alongM = poseDx * obstacleContext.routeUx +
                 poseDy * obstacleContext.routeUy;
  float currentLateralM = -poseDx * obstacleContext.routeUy +
                          poseDy * obstacleContext.routeUx;
  float lateralClearanceM =
    max(ROBOT_FOOTPRINT_GEOMETRY.leftExtentMm,
        ROBOT_FOOTPRINT_GEOMETRY.rightExtentMm) / 1000.0f +
    PLANNER_TOTAL_HARD_CLEARANCE_M + LOCAL_MAP_CELL_M * 0.5f;
  float releaseLateralM = obstacleContext.sideSign > 0.0f
    ? obstacleContext.maxLateralM + lateralClearanceM
    : obstacleContext.minLateralM - lateralClearanceM;
  bool outsideObstacleSide =
    obstacleContext.sideSign * currentLateralM >=
    obstacleContext.sideSign * releaseLateralM;
  bool reachedObstacleApproach =
    alongM >= obstacleContext.approachNearAlongM -
                LOCAL_MAP_CELL_M * 0.5f;
  bool directRelease = reachedObstacleApproach &&
                       outsideObstacleSide &&
                       directSegmentClearsRetainedObstacle(targetX, targetY) &&
                       directWaypointCorridorClear(targetX, targetY);
  float rearClearM = ROBOT_FOOTPRINT_GEOMETRY.rearExtentMm / 1000.0f +
                     PLANNER_TOTAL_HARD_CLEARANCE_M +
                     LOCAL_MAP_CELL_M * 0.5f;
  bool rearPassedObstacle =
    alongM >= obstacleContext.farAlongM + rearClearM;
  if (directRelease || rearPassedObstacle) {
    if (obstacleContext.clearSinceMs == 0) {
      obstacleContext.clearSinceMs = millis();
    }
    if (millis() - obstacleContext.clearSinceMs >= 80) {
      resetObstacleContext(directRelease
        ? "direct_waypoint_corridor_clear" : "obstacle_cleared");
      resetPlannerEpoch();
      // The completed envelope owns only this obstacle. Reacquire immediately
      // so a later blocker gets a fresh envelope and independent side choice.
      return updateObstacleContext(targetX, targetY);
    }
  } else {
    obstacleContext.clearSinceMs = 0;
  }
  return true;
}

static void buildObstacleLocalGoal(float &localGoalX, float &localGoalY) {
  float rearClearM = ROBOT_FOOTPRINT_GEOMETRY.rearExtentMm / 1000.0f +
                     PLANNER_TOTAL_HARD_CLEARANCE_M +
                     LOCAL_MAP_CELL_M * 0.5f;
  float nominalTargetLateralM = obstacleTargetLateralM(
    obstacleContext.sideSign,
    obstacleContext.minLateralM,
    obstacleContext.maxLateralM);
  float poseDx = robotX - obstacleContext.originX;
  float poseDy = robotY - obstacleContext.originY;
  float currentAlongM = poseDx * obstacleContext.routeUx +
                        poseDy * obstacleContext.routeUy;
  float currentLateralM = -poseDx * obstacleContext.routeUy +
                          poseDy * obstacleContext.routeUx;
  float countersteerLeadM = obstacleContext.sideReconsidered
    ? PLANNER_OBSTACLE_RECONSIDERED_COUNTERSTEER_LEAD_M
    : PLANNER_OBSTACLE_COUNTERSTEER_LEAD_M;
  bool laterallyClear = obstacleContext.sideSign * currentLateralM >=
    obstacleContext.sideSign * nominalTargetLateralM -
      countersteerLeadM;
  float targetAlongM;
  if (laterallyClear ||
      currentAlongM >= obstacleContext.approachNearAlongM) {
    targetAlongM = obstacleContext.farAlongM + rearClearM;
  } else {
    float frontClearM = ROBOT_FOOTPRINT_GEOMETRY.frontExtentMm / 1000.0f +
                        PLANNER_TOTAL_HARD_CLEARANCE_M +
                        LOCAL_MAP_CELL_M * 0.5f;
    float approachAlongM =
      max(0.05f, obstacleContext.nearAlongM - frontClearM -
                 PLANNER_OBSTACLE_TURN_ROOM_M);
    if (obstacleContext.sideReconsidered) {
      float sideEscapeTargetAlongM = obstacleContext.sideEscapeAlongM;
      if (sideEscapeTargetAlongM >
          obstacleContext.nearAlongM + PLANNER_PREFERRED_CLEARANCE_M) {
        sideEscapeTargetAlongM =
          obstacleContext.nearAlongM - PLANNER_PREFERRED_CLEARANCE_M;
      }
      targetAlongM = min(sideEscapeTargetAlongM, approachAlongM);
    } else {
      targetAlongM = approachAlongM;
    }
  }
  PlannerCollisionSnapshot collision;
  capturePlannerCollisionSnapshot(collision);
  float targetLateralM = nominalTargetLateralM;
  chooseInsideBandTargetLateral(
    collision,
    obstacleContext.routeUx,
    obstacleContext.routeUy,
    obstacleContext.sideSign,
    targetAlongM,
    targetLateralM,
    targetLateralM);
  obstacleTargetWorldPosition(
    obstacleContext.routeUx,
    obstacleContext.routeUy,
    targetAlongM,
    targetLateralM,
    localGoalX,
    localGoalY);
}

PlannerDebugSnapshot getPlannerDebugSnapshot() {
  float obstacleGoalX = navigationGoal.targetX;
  float obstacleGoalY = navigationGoal.targetY;
  if (obstacleContext.active) {
    buildObstacleLocalGoal(obstacleGoalX, obstacleGoalY);
  }
  PlannerDebugSnapshot snapshot = {
    static_cast<int>(emergencyRecoveryState.phase),
    emergencyRecoveryState.consumed,
    emergencyRecoveryState.scanSector,
    emergencyRecoveryState.scanAccumulatedDeg,
    emergencyRecoveryState.relocationDistanceM,
    emergencyRecoveryState.bestRotationalClearanceM,
    obstacleContext.active ? 1 : 0,
    obstacleContext.active ? obstacleContext.sideSign : 0.0f,
    obstacleContext.nearAlongM,
    obstacleContext.farAlongM,
    obstacleContext.minLateralM,
    obstacleContext.maxLateralM,
    obstacleGoalX,
    obstacleGoalY
  };
  return snapshot;
}

static float minimumFanSweepClearanceMm() {
  // This is a turn-envelope measurement, not the straight-driving body
  // clearance. It is kept for turn safety and as a soft preference signal.
  float minimum = 1000000.0;
  for (int i = RANGE_RIGHT_OUTER; i <= RANGE_LEFT_OUTER; i++) {
    RangeSensorId id = (RangeSensorId)i;
    if (!isRangeSensorValid(id)) {
      continue;
    }
    minimum = min(minimum, getFanSweepClearanceMm(id));
  }
  return minimum == 1000000.0 ? -1.0 : minimum;
}

bool isTurnDirectionObservable(float turnTicksPerSec) {
  // Positive turn increases navigation heading, which is robot-left in the
  // current frame. A turn with missing sensing on the swept side is never
  // assumed safe.
  if (fabs(turnTicksPerSec) < 1.0) {
    return isRangeSensorValid(RANGE_RIGHT_INNER) &&
           isRangeSensorValid(RANGE_LEFT_INNER);
  }

  if (turnTicksPerSec > 0.0) {
    return isRangeSensorValid(RANGE_LEFT_INNER) &&
           isRangeSensorValid(RANGE_LEFT_OUTER);
  }

  return isRangeSensorValid(RANGE_RIGHT_INNER) &&
         isRangeSensorValid(RANGE_RIGHT_OUTER);
}

bool isTurnSweepSafe() {
  // A pivot has a larger swept radius than straight driving. This stricter
  // 50 mm turn margin is intentionally separate from point-goal collision
  // clearance, so a 400 mm straight corridor is not treated as turn space.
  for (int i = RANGE_RIGHT_OUTER; i <= RANGE_LEFT_OUTER; i++) {
    RangeSensorId id = (RangeSensorId)i;
    if (!isRangeSensorValid(id) ||
        getFanSweepClearanceMm(id) < AVOID_CLEARANCE_MARGIN_MM) {
      return false;
    }
  }
  return true;
}

const char* plannerStopReasonName(PlannerStopReason reason) {
  switch (reason) {
    case PLANNER_STOP_NONE: return "none";
    case PLANNER_STOP_FRONT_BLOCKED: return "front_blocked";
    case PLANNER_STOP_FRONT_INVALID: return "front_invalid";
    case PLANNER_STOP_NO_SAFE_TRAJECTORY: return "no_safe_trajectory";
    case PLANNER_STOP_TURN_SIDE_INVALID: return "turn_side_invalid";
    case PLANNER_STOP_TURN_CLEARANCE: return "turn_clearance";
    case PLANNER_STOP_STUCK: return "stuck";
    case PLANNER_STOP_RECOVERY_DIVERGENCE: return "recovery_divergence";
    case PLANNER_STOP_RECOVERY_DISPLACEMENT: return "recovery_displacement";
    case PLANNER_STOP_RECOVERY_TIME: return "recovery_time";
    case PLANNER_STOP_RECOVERY_DISTANCE: return "recovery_distance";
    case PLANNER_STOP_RECOVERY_REPEATED: return "recovery_repeated";
    case PLANNER_STOP_RECOVERY_NO_PROGRESS: return "recovery_no_progress";
    case PLANNER_STOP_RECOVERY_NO_USEFUL_OUTCOME: return "recovery_no_useful_outcome";
    case PLANNER_STOP_EMERGENCY_SCAN_ABORTED: return "emergency_scan_aborted";
    case PLANNER_STOP_EMERGENCY_RETRY_EXHAUSTED: return "emergency_retry_exhausted";
    case PLANNER_STOP_ABORTED: return "aborted";
  }
  return "unknown";
}

static bool publishNavigationMotion(float forwardSpeed, float turnSpeed) {
  // Final planner-to-motor handoff. Inputs are chassis speeds in ticks/s:
  // positive forward drives +X, positive turn is CCW/left. MotorControl.cpp
  // may still veto the command if live safety evidence changed.
  if (setAuthorizedMotionCommand(navigationGoal.authority,
                                 forwardSpeed, turnSpeed)) {
    return true;
  }

  // Keep the goal and its P0-02 authority intact so the planner can replan on
  // the next fresh snapshot. The final motor owner has already forced neutral.
  plannerTelemetry.safeStopReason =
    motionSafetyReasonName(lastMotionSafetyReason());
  plannerTelemetry.replanReason = "continuous_safety_veto";
  return false;
}

static const char* ownerEventName(NavigationGoalOwner owner, bool success) {
  switch (owner) {
    case NAV_OWNER_TEST_DRIVE: return success ? "test_drive_end" : "test_drive_abort";
    case NAV_OWNER_TEST_GOTO: return success ? "test_goto_end" : "test_goto_abort";
    case NAV_OWNER_TEST_AVOID: return success ? "test_avoid_end" : "test_avoid_abort";
    case NAV_OWNER_TEST_ESCAPE: return success ? "test_escape_end" : "test_escape_abort";
    case NAV_OWNER_TEST_TURN: return success ? "test_turn_end" : "test_turn_abort";
    case NAV_OWNER_TEST_HUNT: return success ? "test_hunt_end" : "test_hunt_abort";
    case NAV_OWNER_WEIGHT_SCAN: return success ? "weight_scan_end" : "weight_scan_abort";
    case NAV_OWNER_OBJECT_HUNT: return success ? "object_hunt_end" : "object_hunt_abort";
    default: return success ? "navigation_goal_complete" : "navigation_goal_stop";
  }
}

static bool ownerIsTest(NavigationGoalOwner owner) {
  return owner == NAV_OWNER_TEST_DRIVE || owner == NAV_OWNER_TEST_GOTO ||
         owner == NAV_OWNER_TEST_AVOID || owner == NAV_OWNER_TEST_ESCAPE ||
         owner == NAV_OWNER_TEST_TURN || owner == NAV_OWNER_TEST_HUNT;
}

static bool ownerIsObjectHunt(NavigationGoalOwner owner) {
  return owner == NAV_OWNER_TEST_HUNT || owner == NAV_OWNER_OBJECT_HUNT;
}

static void finishNavigationGoal(bool success, PlannerStopReason reason, const char* detail) {
  // This is the one exit path for both route goals and test goals. It removes
  // any pending motion command before publishing the completion/abort event.
  NavigationGoalOwner owner = navigationGoal.owner;
  if (plannerEpoch.active) {
    closePlannerEpoch();
  }
  resetReversePlannerEpoch();
  reverseRecoveryActive = false;
  reverseRecoveryState = {};
  resetRecoveryBudget();
  emergencyRecoveryState = {};
  plannerTelemetry.reverseRecoveryActive = false;
  resetGeometricNoPathEvidence();
  reverseRecoveryRejectsReported = false;
  resetObstacleContext(success ? "goal_complete" : "goal_abort");
  motorStopRequested = true;
  requestMotionStop();
  navigationGoal.active = false;
  navigationGoal.authority = MOTION_AUTHORITY_NONE;
  navigationGoal.completed = success;
  navigationGoal.failed = !success;
  plannerTelemetry.stopReason = reason;
  plannerTelemetry.safeStopReason = detail;
  plannerTelemetry.selectedForwardTicksPerSec = 0.0f;
  plannerTelemetry.selectedTurnTicksPerSec = 0.0f;
  plannerTelemetry.selectedCurvature = 0.0f;
  plannerTelemetry.plannerEpochActive = false;
  plannerTelemetry.plannerCommandAgeMs = 0;
  sendBluetoothEvent(ownerEventName(owner, success), detail);

  if (ownerIsTest(owner)) {
    robotRunEnabled = false;
    setRobotState(END_MATCH);
    Serial.println(success ? "TEST complete. Motors stopped." : "TEST aborted. Motors stopped.");
  }
}

void startNavigationPoint(float targetX, float targetY, NavigationGoalOwner owner) {
  // Creates a world-frame point goal in metres.
  //
  // Called by:
  //   goToPoint(), Bluetooth TEST DRIVE/GOTO/AVOID/ESCAPE/HUNT, and the
  //   weight-search state machine.
  //
  // Global effects:
  //   Resets planner epochs, recovery state, PID/encoder snapshots, goal
  //   telemetry, and schedules an immediate planner update.
  if (motionAuthority != MOTION_AUTHORITY_MISSION &&
      motionAuthority != MOTION_AUTHORITY_TEST) {
    stopMotors();
    plannerTelemetry.stopReason = PLANNER_STOP_ABORTED;
    plannerTelemetry.safeStopReason = "no_motion_authority";
    return;
  }
  // A point goal does not mean "drive this exact line". It means repeatedly
  // choose a short safe arc that makes progress toward this world coordinate.
  navigationGoal.mode = NAV_GOAL_POINT;
  resetPlannerEpoch();
  resetReversePlannerEpoch();
  lastPlannerCommandPublishedMs = 0;
  navigationGoal.owner = owner;
  navigationGoal.authority = motionAuthority;
  navigationGoal.active = true;
  navigationGoal.completed = false;
  navigationGoal.failed = false;
  navigationGoal.targetX = targetX;
  navigationGoal.targetY = targetY;
  navigationGoal.targetYawDeg = 0.0;
  navigationGoal.startX = robotX;
  navigationGoal.startY = robotY;
  navigationGoal.startYawDeg = navigationHeadingDeg();
  navigationGoal.startedMs = millis();
  // Keep cumulative encoder totals intact, but reset the control snapshots so
  // a previous motion segment cannot create a derivative/PID kick here.
  resetEncodersAndPID();
  // Force a plan on the next controller pass rather than waiting for the old
  // goal's 40 ms schedule phase.
  lastPlannerUpdateMs = 0;
  motorStopRequested = true;
  requestMotionStop();
  // Telemetry is reset with the goal. Without this, a previous turn command
  // can make the first point-goal CSV row look like it is steering.
  plannerTelemetry.selectedForwardTicksPerSec = 0.0;
  plannerTelemetry.selectedTurnTicksPerSec = 0.0;
  plannerTelemetry.selectedCurvature = 0.0;
  plannerTelemetry.minimumSweptClearanceMm = -1.0;
  plannerTelemetry.speedCapTicksPerSec = 0.0;
  plannerTelemetry.globalGoalDistanceM =
    sqrtf((targetX - robotX) * (targetX - robotX) +
          (targetY - robotY) * (targetY - robotY));
  plannerTelemetry.localGoalDistanceM = 0.0;
  plannerTelemetry.routeAlongProgressM = 0.0;
  plannerTelemetry.routeSignedLateralErrorM = 0.0;
  plannerTelemetry.obstacleProgressAgeS = 0.0;
  plannerTelemetry.cumulativeReverseDistanceM = 0.0;
  plannerTelemetry.obstacleBestProgressM = 0.0;
  plannerTelemetry.recoveryCurrentClearanceM = 0.0f;
  plannerTelemetry.recoveryEndpointClearanceM = 0.0f;
  plannerTelemetry.recoveryClearanceGainM = 0.0f;
  plannerTelemetry.recoveryUnexploredScore = 0.0f;
  plannerTelemetry.recoveryCount = 0;
  plannerTelemetry.recoveryPlateauCount = 0;
  plannerTelemetry.reverseRecoveryActive = false;
  plannerTelemetry.candidateCount = 0;
  plannerTelemetry.stopReason = PLANNER_STOP_NONE;
  plannerTelemetry.planReason = "goal_started";
  lastReportedStopReason = PLANNER_STOP_NONE;
  reverseRecoveryActive = false;
  reverseRecoveryState = {};
  resetRecoveryBudget();
  emergencyRecoveryState = {};
  reverseRecoveryStepCount = 0;
  resetGeometricNoPathEvidence();
  candidateRejectsReported = false;
  reverseRecoveryRejectsReported = false;
  resetObstacleContext("point_goal_start");
  motorStopRequested = false;
  plannerTelemetry.replanReason = "goal_started";
  plannerTelemetry.safeStopReason = "";
  resetTurnStuckCheck(navigationHeadingDeg());
  pointAlignTurnActive = false;
  pointAlignTurnDirection = 0.0;
  turnSideInvalidSinceMs = 0;
  turnSweepInvalidSinceMs = 0;
  sendBluetoothEvent("navigation_goal_start", "point");
}

void startNavigationTurn(float relativeTurnDeg, NavigationGoalOwner owner) {
  // Creates a relative in-place yaw goal in degrees.
  //
  // Inputs/outputs:
  //   relativeTurnDeg is positive CCW/left. The stored targetYawDeg is an
  //   absolute navigation heading wrapped to [-180, +180].
  //
  // Safety:
  //   updateTurnGoal() validates turn-side and full sweep sensing before each
  //   command and MotorControl.cpp checks again at output time.
  if (motionAuthority != MOTION_AUTHORITY_MISSION &&
      motionAuthority != MOTION_AUTHORITY_TEST) {
    stopMotors();
    plannerTelemetry.stopReason = PLANNER_STOP_ABORTED;
    plannerTelemetry.safeStopReason = "no_motion_authority";
    return;
  }
  // Turns are their own direct yaw-feedback task. They do not use the map arc
  // sampler because they are intentionally in-place and run at 20 ms.
  float startYawDeg = navigationHeadingDeg();
  resetPlannerEpoch();
  resetReversePlannerEpoch();
  lastPlannerCommandPublishedMs = 0;
  navigationGoal.mode = NAV_GOAL_TURN;
  navigationGoal.owner = owner;
  navigationGoal.authority = motionAuthority;
  navigationGoal.active = true;
  navigationGoal.completed = false;
  navigationGoal.failed = false;
  navigationGoal.targetX = robotX;
  navigationGoal.targetY = robotY;
  navigationGoal.targetYawDeg = wrapAngle(startYawDeg + relativeTurnDeg);
  navigationGoal.startX = robotX;
  navigationGoal.startY = robotY;
  navigationGoal.startYawDeg = startYawDeg;
  navigationGoal.startedMs = millis();
  resetEncodersAndPID();
  lastPlannerUpdateMs = 0;
  resetTurnStuckCheck(startYawDeg);
  turnBrakeActive = false;
  turnBrakeUntilMs = 0;
  turnLastCommandDirection = relativeTurnDeg >= 0.0 ? 1.0 : -1.0;
  turnSideInvalidSinceMs = 0;
  turnSweepInvalidSinceMs = 0;
  reverseRecoveryActive = false;
  reverseRecoveryState = {};
  resetGeometricNoPathEvidence();
  reverseRecoveryRejectsReported = false;
  motorStopRequested = true;
  requestMotionStop();
  resetObstacleContext("turn_goal_start");
  motorStopRequested = false;
  plannerTelemetry.replanReason = "turn_started";
  plannerTelemetry.safeStopReason = "";
  sendBluetoothEvent("navigation_goal_start", "turn");
}

void cancelNavigationGoal(PlannerStopReason reason, const char* detail) {
  // Public cancellation path. If a goal is active, finishNavigationGoal()
  // performs the full neutral/telemetry cleanup. If no goal is active, keep
  // telemetry honest and force neutral anyway.
  if (!navigationGoal.active) {
    navigationGoal.authority = MOTION_AUTHORITY_NONE;
    plannerTelemetry.stopReason = reason;
    plannerTelemetry.safeStopReason = detail;
    motorStopRequested = true;
    requestMotionStop();
    return;
  }
  finishNavigationGoal(false, reason, detail);
}

bool isNavigationGoalActive() {
  return navigationGoal.active;
}

bool didNavigationGoalComplete() {
  return navigationGoal.completed;
}

bool didNavigationGoalFail() {
  return navigationGoal.failed;
}

NavigationStatus getNavigationStatus() {
  NavigationStatus status = {
    navigationGoal.active,
    navigationGoal.completed,
    navigationGoal.failed,
    navigationGoal.mode,
    navigationGoal.owner,
    navigationGoal.authority,
    plannerTelemetry.stopReason,
    plannerTelemetry.safeStopReason
  };
  return status;
}

void clearNavigationGoalResult() {
  navigationGoal.completed = false;
  navigationGoal.failed = false;
}

static float calculateSpeedCapTicksPerSec(float requestedCapTicksPerSec) {
  if (!isRangeSensorValid(RANGE_FRONT)) {
    return 0.0;
  }

  float availableM = getRangeSensorDistance(RANGE_FRONT) / 1000.0 -
                     PLANNER_FRONT_SPEED_BUFFER_M;
  availableM = max(0.0f, availableM);
  // d = v * latency + v^2 / (2a). Solve this braking-distance equation for
  // v, then convert metres/s to encoder ticks/s. This cap is independent of
  // goal distance: it answers only "can the robot stop before the front
  // observation runs out?".
  float a = PLANNER_MAX_DECELERATION_MPS2;
  float latency = PLANNER_SENSING_LATENCY_S;
  float speedMps = -a * latency + sqrtf(a * a * latency * latency + 2.0 * a * availableM);
  speedMps = max(0.0f, speedMps);
  return min(requestedCapTicksPerSec, speedMps * TICKS_PER_METRE);
}


static bool epochTurnDirectionObservable(const PlannerEpoch &epoch,
                                         float turnTicks) {
  if (fabs(turnTicks) < 1.0f) {
    return epoch.rightInnerValid && epoch.leftInnerValid;
  }
  if (turnTicks > 0.0f) {
    return epoch.leftInnerValid && epoch.leftOuterValid;
  }
  return epoch.rightInnerValid && epoch.rightOuterValid;
}

static float epochObservedForwardM(const PlannerEpoch &epoch,
                                   float turnTicks) {
  if (turnTicks > 1.0f) {
    return epoch.observedLeftInnerM;
  }
  if (turnTicks < -1.0f) {
    return epoch.observedRightInnerM;
  }
  if (epoch.rightInnerValid && epoch.leftInnerValid) {
    return min(epoch.observedRightInnerM, epoch.observedLeftInnerM);
  }
  if (epoch.rightInnerValid) {
    return epoch.observedRightInnerM;
  }
  if (epoch.leftInnerValid) {
    return epoch.observedLeftInnerM;
  }
  return 0.0f;
}

static bool rolloutCandidate(const PlannerEpoch &epoch,
                             float forwardTicks, float turnTicks,
                             float goalX, float goalY,
                             float &minimumClearanceMm,
                             float &closestGoalDistanceM,
                             float &headingAtClosestGoalRad,
                             float &finalX,
                             float &finalY,
                             float &finalHeadingRad,
                             float &arrivalTimeS,
                             float &safeTravelM,
                             float &maximumUnknownFraction,
                             CandidateRejectReason &rejectReason) {
  // A candidate is a constant chassis command (forward, turn) simulated for
  // PLANNER_HORIZON_S. Positive turn is CCW/left; MotorControl later converts
  // this into a slower left wheel and faster right wheel.
  if (forwardTicks <= 0.0 ||
      !epochTurnDirectionObservable(epoch, turnTicks)) {
    rejectReason = CANDIDATE_REJECT_TURN_OBSERVABILITY;
    return false;
  }
  rejectReason = CANDIDATE_REJECT_NONE;

  // A candidate may not run its leading footprint beyond the distance that the
  // valid inner fan has actually observed.  Unknown space is not converted to
  // free space merely because it is absent from the local map.
  float observedForwardM = epochObservedForwardM(epoch, turnTicks);
  float leadingEnvelopeM = ROBOT_FOOTPRINT_GEOMETRY.frontExtentMm / 1000.0 +
                           PLANNER_TOTAL_HARD_CLEARANCE_M;

  float heading = epoch.startHeadingRad;
  float x = epoch.startX;
  float y = epoch.startY;
  // Differential-drive kinematics: command is expressed as average wheel
  // speed plus/minus a turn component, then integrated at the midpoint
  // heading to avoid the bias of a simple Euler step.
  float leftMps = leftWheelTargetFromChassis(forwardTicks, turnTicks) /
                  TICKS_PER_METRE;
  float rightMps = rightWheelTargetFromChassis(forwardTicks, turnTicks) /
                   TICKS_PER_METRE;
  float linearMps = (leftMps + rightMps) * 0.5;
  float angularRadPerSec = navigationOmegaFromWheelSpeeds(
    leftMps, rightMps, EFFECTIVE_TRACK_WIDTH_M);
  minimumClearanceMm = epoch.minimumFanClearanceMm;
  closestGoalDistanceM = sqrtf((goalX - x) * (goalX - x) +
                               (goalY - y) * (goalY - y));
  headingAtClosestGoalRad = heading;
  finalX = x;
  finalY = y;
  finalHeadingRad = heading;
  arrivalTimeS = -1.0;
  safeTravelM = 0.0f;
  maximumUnknownFraction = 0.0f;

  float elapsed = 0.0f;
  while (elapsed < PLANNER_HORIZON_S - 0.000001f) {
    float integrationStepS =
      min(PLANNER_ROLLOUT_STEP_S, PLANNER_HORIZON_S - elapsed);
    if (fabs(linearMps) > 0.000001f) {
      integrationStepS = min(
        integrationStepS,
        PLANNER_ROLLOUT_MAX_SPATIAL_STEP_M / fabs(linearMps));
    }
    if (fabs(angularRadPerSec) > 0.000001f) {
      integrationStepS = min(
        integrationStepS,
        PLANNER_ROLLOUT_MAX_HEADING_STEP_RAD / fabs(angularRadPerSec));
    }
    // Keep the start of this integration segment so terminal scoring can
    // measure the closest *continuous* approach to a point goal. Comparing
    // only discrete sample endpoints can favour a slower command simply
    // because one of its samples happens to land nearer the goal.
    float segmentStartX = x;
    float segmentStartY = y;
    float segmentStartHeading = heading;
    float midpointHeading =
      heading + angularRadPerSec * integrationStepS * 0.5f;
    x += linearMps * cosf(midpointHeading) * integrationStepS;
    y += linearMps * sinf(midpointHeading) * integrationStepS;
    heading += angularRadPerSec * integrationStepS;
    // Project the point goal onto this segment. The robot stops as soon as it
    // enters the arrival circle, so a fast candidate must not lose merely
    // because its next discrete rollout sample has passed that point.
    float segmentDx = x - segmentStartX;
    float segmentDy = y - segmentStartY;
    float segmentLengthSquared = segmentDx * segmentDx + segmentDy * segmentDy;
    float segmentProgress = 0.0;
    if (segmentLengthSquared > 0.0000001f) {
      segmentProgress = ((goalX - segmentStartX) * segmentDx +
                         (goalY - segmentStartY) * segmentDy) /
                        segmentLengthSquared;
      segmentProgress = constrain(segmentProgress, 0.0f, 1.0f);
    }
    float closestX = segmentStartX + segmentDx * segmentProgress;
    float closestY = segmentStartY + segmentDy * segmentProgress;
    float goalDistanceM = sqrtf((goalX - closestX) * (goalX - closestX) +
                                (goalY - closestY) * (goalY - closestY));
    if (goalDistanceM < closestGoalDistanceM) {
      closestGoalDistanceM = goalDistanceM;
      headingAtClosestGoalRad = segmentStartHeading +
                                (heading - segmentStartHeading) * segmentProgress;
    }
    // Arrival time is a primary point-goal objective, not an arbitrary score
    // weight. Once a safe arc can enter the arrival circle, prefer the one
    // that gets there sooner rather than a slower arc that happens to improve
    // a secondary heading score.
    if (arrivalTimeS < 0.0 && segmentLengthSquared > 0.0000001f) {
      float fromGoalX = segmentStartX - goalX;
      float fromGoalY = segmentStartY - goalY;
      float radiusSquared = WAYPOINT_TOLERANCE_M * WAYPOINT_TOLERANCE_M;
      float b = 2.0f * (fromGoalX * segmentDx + fromGoalY * segmentDy);
      float c = fromGoalX * fromGoalX + fromGoalY * fromGoalY - radiusSquared;
      float discriminant = b * b - 4.0f * segmentLengthSquared * c;
      if (discriminant >= 0.0f) {
        float entryProgress = (-b - sqrtf(discriminant)) /
                              (2.0f * segmentLengthSquared);
        if (entryProgress >= 0.0f && entryProgress <= 1.0f) {
          arrivalTimeS = elapsed + entryProgress * integrationStepS;
          if (epoch.finalWaypointIsLocalGoal) {
            float arrivalX = segmentStartX + segmentDx * entryProgress;
            float arrivalY = segmentStartY + segmentDy * entryProgress;
            float arrivalHeading = segmentStartHeading +
              (heading - segmentStartHeading) * entryProgress;
            float arrivalTravelM = linearMps * arrivalTimeS;
            if (arrivalTravelM + leadingEnvelopeM > observedForwardM) {
              rejectReason = CANDIDATE_REJECT_FORWARD_OBSERVATION;
              return false;
            }
            if (!footprintClearOnSnapshot(epoch.collision,
                                          arrivalX, arrivalY,
                                          arrivalHeading)) {
              rejectReason = CANDIDATE_REJECT_FOOTPRINT;
              return false;
            }
            finalX = arrivalX;
            finalY = arrivalY;
            finalHeadingRad = arrivalHeading;
            closestGoalDistanceM = 0.0f;
            headingAtClosestGoalRad = arrivalHeading;
            safeTravelM = arrivalTravelM;
            return true;
          }
        }
      }
    }
    float travelledM = linearMps * (elapsed + integrationStepS);
    // Unknown front space is unsafe. Even an empty local-map cell cannot make
    // this candidate legal if the inner fan has not actually observed far
    // enough ahead of the leading edge of the chassis.
    if (travelledM + leadingEnvelopeM > observedForwardM) {
      rejectReason = CANDIDATE_REJECT_FORWARD_OBSERVATION;
      return false;
    }
    if (!footprintClearOnSnapshot(epoch.collision, x, y, heading)) {
      rejectReason = CANDIDATE_REJECT_FOOTPRINT;
      return false;
    }
    maximumUnknownFraction = max(
      maximumUnknownFraction,
      footprintUnknownFractionOnSnapshot(
        epoch.collision, x, y, heading));
    // A 400 mm passage is a straight-traverse problem, not a turning-space
    // problem. Once both nearby walls are evidenced, reject arcs that would
    // keep steering inside it. The planner therefore aligns before entry or
    // stops outside, where a safe turn/backtrack remains possible.
    if (fabs(turnTicks / max(1.0f, forwardTicks)) >
          PLANNER_CORRIDOR_MAX_TURN_RATIO &&
        isNarrowObservedCorridorOnSnapshot(epoch.collision, x, y, heading)) {
      rejectReason = CANDIDATE_REJECT_CORRIDOR;
      return false;
    }
    safeTravelM = travelledM;
    elapsed += integrationStepS;
  }
  finalX = x;
  finalY = y;
  finalHeadingRad = heading;
  return true;
}

struct ForwardCandidateEvaluation {
  float clearanceMm;
  float closestGoalDistanceM;
  float headingAtClosestGoalRad;
  float finalX;
  float finalY;
  float finalHeadingRad;
  float arrivalTimeS;
  float safeTravelM;
  float maximumUnknownFraction;
  CandidateRejectReason rejectReason;
};

static bool evaluateForwardCandidate(const PlannerEpoch &epoch,
                                     float forwardTicks, float turnTicks,
                                     ForwardCandidateEvaluation &evaluation) {
  evaluation.clearanceMm = -1.0f;
  evaluation.closestGoalDistanceM = epoch.localGoalDistanceM;
  evaluation.headingAtClosestGoalRad = epoch.startHeadingRad;
  evaluation.finalX = epoch.startX;
  evaluation.finalY = epoch.startY;
  evaluation.finalHeadingRad = epoch.startHeadingRad;
  evaluation.arrivalTimeS = -1.0f;
  evaluation.safeTravelM = 0.0f;
  evaluation.maximumUnknownFraction = 0.0f;
  evaluation.rejectReason = CANDIDATE_REJECT_NONE;
  return rolloutCandidate(
    epoch, forwardTicks, turnTicks, epoch.goalX, epoch.goalY,
    evaluation.clearanceMm, evaluation.closestGoalDistanceM,
    evaluation.headingAtClosestGoalRad,
    evaluation.finalX, evaluation.finalY, evaluation.finalHeadingRad,
    evaluation.arrivalTimeS, evaluation.safeTravelM,
    evaluation.maximumUnknownFraction, evaluation.rejectReason);
}

static bool obstacleRolloutMakesRequiredLateralProgress(
    float startX, float startY, float finalX, float finalY,
    float localGoalX, float localGoalY,
    float &lateralProgressScore) {
  lateralProgressScore = 0.0f;
  if (!obstacleContext.active) {
    return true;
  }
  float startDx = startX - obstacleContext.originX;
  float startDy = startY - obstacleContext.originY;
  float currentAlongM = startDx * obstacleContext.routeUx +
                        startDy * obstacleContext.routeUy;
  float currentLateralM = -startDx * obstacleContext.routeUy +
                           startDy * obstacleContext.routeUx;
  float targetDx = localGoalX - obstacleContext.originX;
  float targetDy = localGoalY - obstacleContext.originY;
  float targetLateralM =
    -targetDx * obstacleContext.routeUy +
     targetDy * obstacleContext.routeUx;
  float countersteerLeadM = obstacleContext.sideReconsidered
    ? PLANNER_OBSTACLE_RECONSIDERED_COUNTERSTEER_LEAD_M
    : PLANNER_OBSTACLE_COUNTERSTEER_LEAD_M;
  bool needsLateralClearance =
    currentAlongM < obstacleContext.nearAlongM &&
    obstacleContext.sideSign * currentLateralM <
      obstacleContext.sideSign * targetLateralM -
        countersteerLeadM;
  if (!needsLateralClearance) {
    return true;
  }
  float finalDx = finalX - obstacleContext.originX;
  float finalDy = finalY - obstacleContext.originY;
  float finalLateralM = -finalDx * obstacleContext.routeUy +
                         finalDy * obstacleContext.routeUx;
  float signedProgressM =
    obstacleContext.sideSign * (finalLateralM - currentLateralM);
  float remainingClearanceM =
    obstacleContext.sideSign * (targetLateralM - currentLateralM);
  lateralProgressScore = constrain(
    signedProgressM / max(LOCAL_MAP_CELL_M, remainingClearanceM),
    0.0f, 1.0f);
  return signedProgressM > 0.0f;
}

static float candidateScore(float forwardTicks, float turnTicks,
                            float localGoalX, float localGoalY,
                            float plannerStartX, float plannerStartY,
                            float previousSelectedTurn,
                            float closestGoalDistanceM,
                            float headingAtClosestGoalRad,
                            float finalX,
                            float finalY,
                            float finalHeadingRad,
                            float clearanceMm,
                            bool lineFollowActive,
                            float routeStartX,
                            float routeStartY,
                            float routeHeadingRad,
                            float finalGoalDistanceM) {
  float startGoalDistanceM = sqrtf(
    (localGoalX - plannerStartX) * (localGoalX - plannerStartX) +
    (localGoalY - plannerStartY) * (localGoalY - plannerStartY));
  // A point goal is complete as soon as the chassis enters its arrival
  // circle.  Score the closest point on the rollout, rather than its final
  // point after a hypothetical 0.8 s of continued driving.  Otherwise a
  // sound fast candidate is incorrectly penalised merely for passing the
  // target after the real controller would already have stopped.
  // The score deliberately mixes several modest preferences. Hard safety was
  // already checked in rolloutCandidate(); scoring is only how we choose among
  // commands that are all legal.
  float progressScore = constrain((startGoalDistanceM - closestGoalDistanceM) /
                                  max(PLANNER_MIN_PROGRESS_M, startGoalDistanceM),
                                  -1.0, 1.0);
  float desiredHeadingDeg = atan2f(localGoalY - plannerStartY,
                                   localGoalX - plannerStartX) * 180.0 / PI;
  float headingError = fabs(wrapAngle(desiredHeadingDeg -
                                      headingAtClosestGoalRad * RAD_TO_DEG));
  float curvature = fabs(turnTicks) / max(1.0f, forwardTicks);
  // 50 mm is a *preference*, not a binary wall. A tight but hard-safe arc can
  // still win when it is the only way through a valid narrow passage.
  float clearanceScore = constrain(max(0.0f, clearanceMm) /
                                   (PLANNER_PREFERRED_CLEARANCE_M * 1000.0f),
                                   0.0f, 1.0f);
  float headingScore = 1.0 - min(1.0f, headingError / 90.0f);
  // Discourage left-right twitching between successive 40 ms replans. This is
  // intentionally a soft bias so safety/progress can always override it.
  float smoothnessScore = 1.0 - min(1.0f, fabs(turnTicks - previousSelectedTurn) /
                                          max(1.0f, baseTargetSpeed));
  const float curvaturePenaltyWeight = 0.8f;
  float lineScore = 0.0f;
  float routeHeadingScore = 0.0f;
  float nearGoalStraightPenalty = 0.0f;
  if (lineFollowActive) {
    float routeDx = cosf(routeHeadingRad);
    float routeDy = sinf(routeHeadingRad);
    float lateralErrorM = fabs(-(finalX - routeStartX) * routeDy +
                               (finalY - routeStartY) * routeDx);
    lineScore = 1.0f - min(1.0f, lateralErrorM /
                                  PLANNER_LINE_FOLLOW_LATERAL_TOLERANCE_M);
    float routeHeadingErrorDeg = fabs(wrapAngle(routeHeadingRad * RAD_TO_DEG -
                                                finalHeadingRad * RAD_TO_DEG));
    routeHeadingScore = 1.0f - min(1.0f, routeHeadingErrorDeg /
                                          PLANNER_LINE_FOLLOW_HEADING_TOLERANCE_DEG);
    if (finalGoalDistanceM <= PLANNER_NEAR_GOAL_STRAIGHTEN_DISTANCE_M) {
      nearGoalStraightPenalty = 1.4f * curvature;
    }
  }
  return 3.0 * progressScore + 2.2 * headingScore + 1.4 * clearanceScore +
         0.8 * smoothnessScore + 1.6 * lineScore + 1.0 * routeHeadingScore -
         curvaturePenaltyWeight * curvature - nearGoalStraightPenalty;
}

static bool routeLineFrame(float &routeLengthM, float &routeUx,
                           float &routeUy, float &routeHeadingRad) {
  if (navigationGoal.mode != NAV_GOAL_POINT) {
    return false;
  }

  float routeDx = navigationGoal.targetX - navigationGoal.startX;
  float routeDy = navigationGoal.targetY - navigationGoal.startY;
  routeLengthM = sqrtf(routeDx * routeDx + routeDy * routeDy);
  if (routeLengthM <= WAYPOINT_TOLERANCE_M) {
    return false;
  }

  routeUx = routeDx / routeLengthM;
  routeUy = routeDy / routeLengthM;
  routeHeadingRad = atan2f(routeDy, routeDx);
  return true;
}

static float routeLineAlongM(float worldX, float worldY,
                             float routeUx, float routeUy) {
  return (worldX - navigationGoal.startX) * routeUx +
         (worldY - navigationGoal.startY) * routeUy;
}

static float routeLineLateralErrorM(float worldX, float worldY,
                                    float routeUx, float routeUy) {
  return fabs(-(worldX - navigationGoal.startX) * routeUy +
              (worldY - navigationGoal.startY) * routeUx);
}

static float routeLineSignedLateralErrorM(float worldX, float worldY,
                                          float routeUx, float routeUy) {
  // Positive is left of the nominal start->target route frame.
  return -(worldX - navigationGoal.startX) * routeUy +
         (worldY - navigationGoal.startY) * routeUx;
}

static bool routeLineTrackingEligible(float &routeLengthM, float &routeUx,
                                      float &routeUy, float &routeHeadingRad) {
  if (!routeLineFrame(routeLengthM, routeUx, routeUy, routeHeadingRad)) {
    return false;
  }

  float routeHeadingDeg = routeHeadingRad * RAD_TO_DEG;
  return fabs(wrapAngle(routeHeadingDeg - navigationHeadingDeg())) <=
    PLANNER_LINE_FOLLOW_ENABLE_HEADING_DEG;
}

static void resetRecoveryBudget() {
  recoveryBudget = {};
  recoveryBudget.lastReverseX = robotX;
  recoveryBudget.lastReverseY = robotY;
  plannerTelemetry.obstacleProgressAgeS = 0.0f;
  plannerTelemetry.cumulativeReverseDistanceM = 0.0f;
  plannerTelemetry.obstacleBestProgressM = 0.0f;
  plannerTelemetry.recoveryCount = 0;
}

static bool emergencyRecoveryOwnerEligible(NavigationGoalOwner owner) {
  return owner == NAV_OWNER_ROUTE ||
         owner == NAV_OWNER_RETURN_HOME ||
         owner == NAV_OWNER_TEST_GOTO ||
         owner == NAV_OWNER_TEST_AVOID ||
         owner == NAV_OWNER_TEST_ESCAPE;
}

static bool emergencyRecoveryReasonEligible(PlannerStopReason reason) {
  return reason == PLANNER_STOP_RECOVERY_TIME ||
         reason == PLANNER_STOP_RECOVERY_DISTANCE ||
         reason == PLANNER_STOP_RECOVERY_REPEATED ||
         reason == PLANNER_STOP_RECOVERY_NO_PROGRESS ||
         reason == PLANNER_STOP_RECOVERY_NO_USEFUL_OUTCOME ||
         reason == PLANNER_STOP_NO_SAFE_TRAJECTORY;
}

static bool emergencySensorFramesCurrent() {
  for (int i = RANGE_RIGHT_OUTER; i <= RANGE_LEFT_OUTER; ++i) {
    if (!isRangeSensorCurrent((RangeSensorId)i)) {
      return false;
    }
  }
  return hasTrustedRearCoverage() &&
         isRangeSensorCurrent(RANGE_FAKE_REAR);
}

static bool emergencyRecoverySensorsHealthy() {
  return emergencySensorFramesCurrent() &&
         !isRangeSensorBlocked(RANGE_FAKE_REAR);
}

static bool tryBeginEmergencyRecovery(PlannerStopReason reason,
                                      const char* detail) {
  if (!emergencyScanPolicyEnabled ||
      emergencyRecoveryState.consumed ||
      navigationGoal.mode != NAV_GOAL_POINT ||
      !emergencyRecoveryOwnerEligible(navigationGoal.owner) ||
      !emergencyRecoveryReasonEligible(reason) ||
      navigationGoal.authority == MOTION_AUTHORITY_NONE ||
      navigationGoal.authority != motionAuthority ||
      driveStuck || wheelMismatchStuck || turnStuck ||
      !emergencyRecoverySensorsHealthy()) {
    return false;
  }
  if (reason == PLANNER_STOP_NO_SAFE_TRAJECTORY &&
      recoveryBudget.attemptCount == 0) {
    return false;
  }

  const unsigned long now = millis();
  resetPlannerEpoch();
  resetReversePlannerEpoch();
  reverseRecoveryActive = false;
  reverseRecoveryState = {};
  resetRecoveryBudget();
  resetGeometricNoPathEvidence();
  emergencyRecoveryState = {};
  emergencyRecoveryState.phase = EMERGENCY_RECOVERY_SETTLE_CURRENT;
  emergencyRecoveryState.consumed = true;
  emergencyRecoveryState.startedMs = now;
  emergencyRecoveryState.phaseStartedMs = now;
  emergencyRecoveryState.startX = robotX;
  emergencyRecoveryState.startY = robotY;
  emergencyRecoveryState.lastX = robotX;
  emergencyRecoveryState.lastY = robotY;
  emergencyRecoveryState.relocationCheckpointX = robotX;
  emergencyRecoveryState.relocationCheckpointY = robotY;
  emergencyRecoveryState.scanLastYawDeg = navigationHeadingDeg();
  for (int i = RANGE_RIGHT_OUTER; i <= RANGE_LEFT_OUTER; ++i) {
    emergencyRecoveryState.fanReadBaselineMs[i - RANGE_RIGHT_OUTER] =
      rangeSensors[i].lastReadMs;
  }
  emergencyRecoveryState.rearFrameBaseline =
    getRearObstacleFrameSequence();
  motorStopRequested = true;
  requestMotionStop();
  plannerTelemetry.reverseRecoveryActive = false;
  plannerTelemetry.stopReason = PLANNER_STOP_NONE;
  plannerTelemetry.planReason = "emergency_scan_settle";
  plannerTelemetry.replanReason = detail;
  plannerTelemetry.safeStopReason = "";
  sendBluetoothEvent("emergency_scan_start", detail);
  return true;
}

static bool handleRecoveryExhaustion(PlannerStopReason reason,
                                     const char* detail) {
  plannerTelemetry.replanReason = detail;
  if (emergencyRecoveryReasonEligible(reason) &&
      emergencyRecoveryState.consumed &&
      emergencyRecoveryState.retryActive) {
    finishNavigationGoal(false,
      PLANNER_STOP_EMERGENCY_RETRY_EXHAUSTED,
      "emergency_scan_retry_exhausted");
    return true;
  }
  if (tryBeginEmergencyRecovery(reason, detail)) {
    return true;
  }
  finishNavigationGoal(false, reason, detail);
  return true;
}

static bool updateReverseRecoveryBudget() {
  unsigned long now = millis();
  recoveryBudget.cumulativeReverseDistanceM += hypotf(
    robotX - recoveryBudget.lastReverseX,
    robotY - recoveryBudget.lastReverseY);
  recoveryBudget.lastReverseX = robotX;
  recoveryBudget.lastReverseY = robotY;
  plannerTelemetry.obstacleProgressAgeS = 0.0f;
  plannerTelemetry.cumulativeReverseDistanceM =
    recoveryBudget.cumulativeReverseDistanceM;
  plannerTelemetry.recoveryCount = recoveryBudget.attemptCount;

  if (reverseRecoveryStartedMs != 0 &&
      now - reverseRecoveryStartedMs >
        PLANNER_REVERSE_RECOVERY_MAX_TIME_MS) {
    return handleRecoveryExhaustion(
      PLANNER_STOP_RECOVERY_TIME,
      "reverse_recovery_time_exhausted");
  }
  if (recoveryBudget.cumulativeReverseDistanceM >
      PLANNER_RECOVERY_MAX_CUMULATIVE_REVERSE_DISTANCE_M) {
    return handleRecoveryExhaustion(
      PLANNER_STOP_RECOVERY_DISTANCE,
      "reverse_recovery_distance_exhausted");
  }
  uint8_t maximumAttempts =
    emergencyRecoveryState.retryActive ? 1 : PLANNER_RECOVERY_MAX_COUNT;
  if (recoveryBudget.attemptCount > maximumAttempts) {
    return handleRecoveryExhaustion(
      PLANNER_STOP_RECOVERY_REPEATED,
      "recovery_count_exhausted");
  }
  return false;
}

static bool obstacleProgressStalled(float localGoalX, float localGoalY,
                                    float localGoalDistanceM) {
  if (!obstacleContext.active) {
    return false;
  }

  unsigned long now = millis();
  float goalShiftM = obstacleContext.progressGoalValid
    ? hypotf(localGoalX - obstacleContext.progressGoalX,
             localGoalY - obstacleContext.progressGoalY)
    : 0.0f;
  if (!obstacleContext.progressGoalValid ||
      goalShiftM >= LOCAL_MAP_CELL_M) {
    obstacleContext.progressGoalValid = true;
    obstacleContext.progressGoalX = localGoalX;
    obstacleContext.progressGoalY = localGoalY;
    obstacleContext.progressStartDistanceM = localGoalDistanceM;
    obstacleContext.progressBestDistanceM = localGoalDistanceM;
    obstacleContext.progressLastMs = now;
  } else if (localGoalDistanceM <=
             obstacleContext.progressBestDistanceM -
               PLANNER_OBSTACLE_PROGRESS_EPSILON_M) {
    obstacleContext.progressBestDistanceM = localGoalDistanceM;
    obstacleContext.progressLastMs = now;
  }

  plannerTelemetry.obstacleProgressAgeS =
    (now - obstacleContext.progressLastMs) / 1000.0f;
  plannerTelemetry.obstacleBestProgressM = max(
    0.0f,
    obstacleContext.progressStartDistanceM -
      obstacleContext.progressBestDistanceM);
  return now - obstacleContext.progressLastMs >
    PLANNER_OBSTACLE_PROGRESS_TIMEOUT_MS;
}

static bool routeLineGoalReached(float routeLengthM, float routeUx, float routeUy,
                                 float routeHeadingRad) {
  float alongM = routeLineAlongM(robotX, robotY, routeUx, routeUy);
  float lateralErrorM = routeLineLateralErrorM(robotX, robotY, routeUx, routeUy);
  float headingErrorDeg = fabs(wrapAngle(routeHeadingRad * RAD_TO_DEG -
                                         navigationHeadingDeg()));
  return alongM >= routeLengthM - WAYPOINT_TOLERANCE_M &&
         alongM <= routeLengthM + PLANNER_LINE_FOLLOW_FINISH_OVERSHOOT_M &&
         lateralErrorM <= PLANNER_LINE_FOLLOW_FINISH_LATERAL_M &&
         headingErrorDeg <= PLANNER_LINE_FOLLOW_FINISH_HEADING_DEG;
}

enum SafePivotStepResult {
  SAFE_PIVOT_STEP_PUBLISHED,
  SAFE_PIVOT_STEP_REVALIDATING,
  SAFE_PIVOT_STEP_SIDE_INVALID,
  SAFE_PIVOT_STEP_SWEEP_INVALID,
  SAFE_PIVOT_STEP_SWEEP_BLOCKED,
  SAFE_PIVOT_STEP_STUCK,
  SAFE_PIVOT_STEP_PUBLICATION_VETOED
};

static SafePivotStepResult commandSafePivotStep(
    float turnTarget, const char* motionPlanReason,
    const char* sideRevalidatePlanReason,
    const char* sweepRevalidatePlanReason) {
  if (!isTurnDirectionObservable(turnTarget)) {
    unsigned long now = millis();
    if (turnSideInvalidSinceMs == 0) {
      turnSideInvalidSinceMs = now;
      sendBluetoothEvent("turn_side_revalidate",
                         "motors_stopped_for_fresh_sample");
    }
    motorStopRequested = true;
    requestMotionStop();
    if (now - turnSideInvalidSinceMs < PLANNER_TURN_SENSOR_REVALIDATE_MS) {
      plannerTelemetry.planReason = sideRevalidatePlanReason;
      return SAFE_PIVOT_STEP_REVALIDATING;
    }
    return SAFE_PIVOT_STEP_SIDE_INVALID;
  }
  turnSideInvalidSinceMs = 0;

  if (!areTurnSweepSensorsValid()) {
    unsigned long now = millis();
    if (turnSweepInvalidSinceMs == 0) {
      turnSweepInvalidSinceMs = now;
      sendBluetoothEvent("turn_sweep_revalidate",
                         "motors_stopped_for_fresh_sample");
    }
    motorStopRequested = true;
    requestMotionStop();
    if (now - turnSweepInvalidSinceMs < PLANNER_TURN_SENSOR_REVALIDATE_MS) {
      plannerTelemetry.planReason = sweepRevalidatePlanReason;
      return SAFE_PIVOT_STEP_REVALIDATING;
    }
    return SAFE_PIVOT_STEP_SWEEP_INVALID;
  }
  turnSweepInvalidSinceMs = 0;

  if (!isTurnSweepSafe()) {
    return SAFE_PIVOT_STEP_SWEEP_BLOCKED;
  }

  updateStuckTurning(navigationHeadingDeg());
  if (turnStuck) {
    return SAFE_PIVOT_STEP_STUCK;
  }

  plannerTelemetry.selectedForwardTicksPerSec = 0.0f;
  plannerTelemetry.selectedTurnTicksPerSec = turnTarget;
  plannerTelemetry.selectedCurvature = 0.0f;
  plannerTelemetry.minimumSweptClearanceMm = minimumFanSweepClearanceMm();
  plannerTelemetry.speedCapTicksPerSec = 0.0f;
  plannerTelemetry.candidateCount = 1;
  plannerTelemetry.stopReason = PLANNER_STOP_NONE;
  plannerTelemetry.planReason = motionPlanReason;
  motorStopRequested = false;
  return publishNavigationMotion(0.0f, turnTarget)
    ? SAFE_PIVOT_STEP_PUBLISHED
    : SAFE_PIVOT_STEP_PUBLICATION_VETOED;
}

static bool routeLineOvershootStopReached(float routeLengthM, float routeUx,
                                          float routeUy, float routeHeadingRad) {
  float alongM = routeLineAlongM(robotX, robotY, routeUx, routeUy);
  float lateralErrorM = routeLineLateralErrorM(robotX, robotY, routeUx, routeUy);
  float headingErrorDeg = fabs(wrapAngle(routeHeadingRad * RAD_TO_DEG -
                                         navigationHeadingDeg()));
  return alongM > routeLengthM + PLANNER_LINE_FOLLOW_FINISH_OVERSHOOT_M &&
         alongM <= routeLengthM + PLANNER_LINE_FOLLOW_MISSED_STOP_OVERSHOOT_M &&
         lateralErrorM <= PLANNER_LINE_FOLLOW_FINISH_LATERAL_M &&
         headingErrorDeg <= PLANNER_LINE_FOLLOW_ENABLE_HEADING_DEG;
}

static bool routeLineClearlyMissed(float routeLengthM, float routeUx, float routeUy) {
  float alongM = routeLineAlongM(robotX, robotY, routeUx, routeUy);
  float lateralErrorM = routeLineLateralErrorM(robotX, robotY, routeUx, routeUy);
  return alongM > routeLengthM + PLANNER_LINE_FOLLOW_MISSED_STOP_OVERSHOOT_M &&
         lateralErrorM <= PLANNER_LINE_FOLLOW_LATERAL_TOLERANCE_M;
}


static bool huntPickupCarryThroughActive(float routeLengthM, float routeUx,
                                         float routeUy) {
  if (!ownerIsObjectHunt(navigationGoal.owner)) {
    return false;
  }

  float alongM = routeLineAlongM(robotX, robotY, routeUx, routeUy);
  float lateralErrorM = routeLineLateralErrorM(robotX, robotY, routeUx, routeUy);
  return alongM >= routeLengthM - PLANNER_HUNT_PICKUP_CARRY_ZONE_M &&
         alongM <= routeLengthM + PLANNER_HUNT_FINISH_OVERSHOOT_M &&
         lateralErrorM <= PLANNER_HUNT_FINISH_LATERAL_M;
}


static float requestedPointGoalSpeedCap() {
  return min(baseTargetSpeed, PLANNER_FORWARD_MAX_SPEED_TPS);
}

static bool huntPickupZoneReached(float routeLengthM, float routeUx, float routeUy) {
  if (!ownerIsObjectHunt(navigationGoal.owner)) {
    return false;
  }

  float alongM = routeLineAlongM(robotX, robotY, routeUx, routeUy);
  float lateralErrorM = routeLineLateralErrorM(robotX, robotY, routeUx, routeUy);
  return alongM >= routeLengthM - PLANNER_HUNT_FINISH_TARGET_TOLERANCE_M &&
         alongM <= routeLengthM + PLANNER_HUNT_FINISH_OVERSHOOT_M &&
         lateralErrorM <= PLANNER_HUNT_FINISH_LATERAL_M;
}

static float pointAlignmentTurnErrorDeg(float headingErrorDeg) {
  return headingErrorDeg;
}

static bool commandPointAlignmentTurn(float headingErrorDeg) {
  float selectedHeadingErrorDeg = pointAlignmentTurnErrorDeg(headingErrorDeg);
  if (!pointAlignTurnActive || pointAlignTurnDirection == 0.0f) {
    pointAlignTurnActive = true;
    pointAlignTurnDirection = selectedHeadingErrorDeg >= 0.0f ? 1.0f : -1.0f;
    resetTurnStuckCheck(navigationHeadingDeg());
  }
  bool slowTurn = fabs(headingErrorDeg) < SLOW_ZONE_DEG;
  float turnTarget = pointAlignTurnDirection *
                     (slowTurn ? PLANNER_TURN_SLOW_TARGET_SPEED : PLANNER_TURN_TARGET_SPEED);

  if (!isTurnDirectionObservable(turnTarget)) {
    unsigned long now = millis();
    if (turnSideInvalidSinceMs == 0) {
      turnSideInvalidSinceMs = now;
      sendBluetoothEvent("turn_side_revalidate", "point_align_sensor_recheck");
    }
    motorStopRequested = true;
    requestMotionStop();
    if (now - turnSideInvalidSinceMs < PLANNER_TURN_SENSOR_REVALIDATE_MS) {
      plannerTelemetry.planReason = "point_align_side_revalidating";
      return true;
    }
    finishNavigationGoal(false, PLANNER_STOP_TURN_SIDE_INVALID, "point_align_side_sensor_invalid");
    return true;
  }
  turnSideInvalidSinceMs = 0;

  if (!areTurnSweepSensorsValid()) {
    unsigned long now = millis();
    if (turnSweepInvalidSinceMs == 0) {
      turnSweepInvalidSinceMs = now;
      sendBluetoothEvent("turn_sweep_revalidate", "point_align_sensor_recheck");
    }
    motorStopRequested = true;
    requestMotionStop();
    if (now - turnSweepInvalidSinceMs < PLANNER_TURN_SENSOR_REVALIDATE_MS) {
      plannerTelemetry.planReason = "point_align_sweep_revalidating";
      return true;
    }
    finishNavigationGoal(false, PLANNER_STOP_TURN_CLEARANCE, "point_align_sweep_sensor_invalid");
    return true;
  }
  turnSweepInvalidSinceMs = 0;

  if (!isTurnSweepSafe()) {
    finishNavigationGoal(false, PLANNER_STOP_TURN_CLEARANCE, "point_align_sweep_not_clear");
    return true;
  }

  updateStuckTurning(navigationHeadingDeg());
  if (turnStuck) {
    finishNavigationGoal(false, PLANNER_STOP_STUCK, "point_align_progress_stalled");
    return true;
  }

  plannerTelemetry.selectedForwardTicksPerSec = 0.0;
  plannerTelemetry.selectedTurnTicksPerSec = turnTarget;
  plannerTelemetry.selectedCurvature = 0.0;
  plannerTelemetry.minimumSweptClearanceMm = minimumFanSweepClearanceMm();
  plannerTelemetry.speedCapTicksPerSec = 0.0;
  plannerTelemetry.localGoalDistanceM = 0.0;
  plannerTelemetry.candidateCount = 1;
  plannerTelemetry.stopReason = PLANNER_STOP_NONE;
  plannerTelemetry.planReason = slowTurn ? "point_align_turn_slow" : "point_align_turn_fast";
  plannerTelemetry.replanReason = "point_target_behind";
  plannerTelemetry.safeStopReason = "";
  lastReportedStopReason = PLANNER_STOP_NONE;
  motorStopRequested = false;
  publishNavigationMotion(0.0, turnTarget);
  return true;
}

static void capturePlannerEpochView(PlannerEpoch &epoch) {
  epoch.startX = robotX;
  epoch.startY = robotY;
  epoch.startHeadingRad = navigationHeadingRad();
  epoch.rightInnerValid = isRangeSensorValid(RANGE_RIGHT_INNER);
  epoch.leftInnerValid = isRangeSensorValid(RANGE_LEFT_INNER);
  epoch.rightOuterValid = isRangeSensorValid(RANGE_RIGHT_OUTER);
  epoch.leftOuterValid = isRangeSensorValid(RANGE_LEFT_OUTER);
  epoch.observedRightInnerM = epoch.rightInnerValid
    ? fanForwardObservationDistanceM(RANGE_RIGHT_INNER) : 0.0f;
  epoch.observedLeftInnerM = epoch.leftInnerValid
    ? fanForwardObservationDistanceM(RANGE_LEFT_INNER) : 0.0f;
  epoch.minimumFanClearanceMm = minimumFanSweepClearanceMm();
  capturePlannerCollisionSnapshot(epoch.collision);
}

static void recordPlannerSlice(unsigned long sliceStartedUs) {
  unsigned long sliceUs = micros() - sliceStartedUs;
  plannerEpoch.accumulatedWorkUs += sliceUs;
  plannerTelemetry.plannerSliceUs = sliceUs;
  plannerTelemetry.plannerSliceMaxUs =
    max(plannerTelemetry.plannerSliceMaxUs, sliceUs);
  plannerTelemetry.plannerEpochWorkUs = plannerEpoch.accumulatedWorkUs;
  plannerTelemetry.plannerEpochMaxWorkUs =
    max(plannerTelemetry.plannerEpochMaxWorkUs,
        plannerEpoch.accumulatedWorkUs);
  recordMainLoopPhaseDuration("planner_slice", sliceStartedUs);
}

static void notePlannerPending() {
  plannerEpoch.yieldCount++;
  plannerTelemetry.plannerYieldCount = plannerEpoch.yieldCount;
  plannerTelemetry.plannerEpochActive = true;
  plannerTelemetry.plannerEpochAgeMs = millis() - plannerEpoch.startedMs;
  plannerTelemetry.planReason = "planner_epoch_pending";
}

static TrajectoryPlanResult failPlannerEpochNoPath(const char* safeReason,
                                                   const char* replanReason) {
  stopMotors();
  lastForwardNoPathWasGeometric =
    plannerEpoch.acceptedCount == 0 &&
    plannerEpoch.rejectedTurnObservability == 0 &&
    plannerEpoch.skippedLinePolicy == 0;
  if (plannerEpoch.acceptedCount == 0 && !candidateRejectsReported) {
    char detail[224];
    snprintf(detail, sizeof(detail),
             "turn=%d;observed=%d;footprint=%d;corridor=%d;policy=%d;fp=%.3f/%.3f@%d/%d;corr=%.2f/%.2f",
             plannerEpoch.rejectedTurnObservability,
             plannerEpoch.rejectedForwardObservation,
             plannerEpoch.rejectedFootprint,
             plannerEpoch.rejectedCorridor,
             plannerEpoch.skippedLinePolicy,
             lastFootprintRejectWorldX, lastFootprintRejectWorldY,
             lastFootprintRejectCellX, lastFootprintRejectCellY,
             lastCorridorRejectLeftM, lastCorridorRejectRightM);
    sendBluetoothEvent("planner_candidate_rejects", detail);
    candidateRejectsReported = true;
  }
  plannerTelemetry.candidateCount = plannerEpoch.acceptedCount;
  plannerTelemetry.stopReason = PLANNER_STOP_NO_SAFE_TRAJECTORY;
  plannerTelemetry.safeStopReason = safeReason;
  plannerTelemetry.replanReason = replanReason;
  closePlannerEpoch();
  return TRAJECTORY_PLAN_NO_PATH;
}

static TrajectoryPlanResult retryPlannerEpoch(PlannerStopReason stopReason,
                                              const char* safeReason,
                                              const char* replanReason) {
  // A stale snapshot or changed velocity envelope is not evidence that every
  // forward path is blocked. Stop, discard the epoch, and build a fresh one
  // without advancing the reverse-recovery debounce.
  stopMotors();
  lastForwardNoPathWasGeometric = false;
  plannerTelemetry.stopReason = stopReason;
  plannerTelemetry.safeStopReason = safeReason;
  plannerTelemetry.replanReason = replanReason;
  closePlannerEpoch();
  return TRAJECTORY_PLAN_RETRY;
}

static TrajectoryPlanResult beginPlannerEpoch(float goalX, float goalY) {
  // Captures a coherent planning snapshot for one local point goal.
  // goalX/goalY are world metres for the current local target, which may be a
  // lookahead point, side-escape waypoint, or final waypoint.
  memset(&plannerEpoch, 0, sizeof(plannerEpoch));
  lastForwardNoPathWasGeometric = false;
  plannerEpoch.active = true;
  plannerEpoch.startedMs = millis();
  plannerEpoch.goalStartedMs = navigationGoal.startedMs;
  plannerEpoch.authority = navigationGoal.authority;
  plannerEpoch.goalX = goalX;
  plannerEpoch.goalY = goalY;
  plannerEpoch.bestScore = -1000000.0f;
  capturePlannerEpochView(plannerEpoch);

  float dx = goalX - plannerEpoch.startX;
  float dy = goalY - plannerEpoch.startY;
  plannerEpoch.localGoalDistanceM = sqrtf(dx * dx + dy * dy);
  float finalDx = navigationGoal.targetX - plannerEpoch.startX;
  float finalDy = navigationGoal.targetY - plannerEpoch.startY;
  plannerEpoch.finalGoalDistanceM = sqrtf(finalDx * finalDx + finalDy * finalDy);
  plannerEpoch.finalWaypointIsLocalGoal =
    navigationGoal.mode == NAV_GOAL_POINT &&
    fabs(goalX - navigationGoal.targetX) < 0.001f &&
    fabs(goalY - navigationGoal.targetY) < 0.001f;
  float routeLengthM = 0.0f;
  float routeUx = 1.0f;
  float routeUy = 0.0f;
  plannerEpoch.lineFollowActive =
    !obstacleContext.active &&
    routeLineTrackingEligible(routeLengthM, routeUx, routeUy,
                              plannerEpoch.routeHeadingRad);
  plannerEpoch.requestedSpeedCap = requestedPointGoalSpeedCap();
  plannerEpoch.speedCap =
    calculateSpeedCapTicksPerSec(plannerEpoch.requestedSpeedCap);
  plannerEpoch.previousSelectedTurn =
    plannerTelemetry.selectedTurnTicksPerSec;

  plannerTelemetry.speedCapTicksPerSec = plannerEpoch.speedCap;
  plannerTelemetry.localGoalDistanceM = plannerEpoch.localGoalDistanceM;
  plannerTelemetry.candidateCount = 0;
  plannerTelemetry.plannerCandidatesProcessed = 0;
  plannerTelemetry.plannerYieldCount = 0;
  plannerTelemetry.plannerEpochWorkUs = 0;
  plannerTelemetry.plannerEpochAgeMs = 0;
  plannerTelemetry.plannerEpochActive = true;
  lastFootprintRejectWorldX = 0.0f;
  lastFootprintRejectWorldY = 0.0f;
  lastFootprintRejectCellX = -1;
  lastFootprintRejectCellY = -1;
  lastCorridorRejectLeftM = -1.0f;
  lastCorridorRejectRightM = -1.0f;

  if (!isRangeSensorValid(RANGE_FRONT)) {
    return retryPlannerEpoch(PLANNER_STOP_FRONT_INVALID,
                             "front_sensor_invalid",
                             "front_sensor_invalid_retry");
  }
  if (isRangeSensorBlocked(RANGE_FRONT)) {
    lastForwardNoPathWasGeometric = true;
    plannerTelemetry.stopReason = PLANNER_STOP_FRONT_BLOCKED;
    plannerTelemetry.safeStopReason = "front_blocked";
    plannerTelemetry.replanReason = "front_blocked";
    closePlannerEpoch();
    return TRAJECTORY_PLAN_NO_PATH;
  }
  if (plannerEpoch.speedCap < PLANNER_MIN_DRIVABLE_SPEED_TPS) {
    return retryPlannerEpoch(PLANNER_STOP_NO_SAFE_TRAJECTORY,
                             "speed_cap_below_drivable_min",
                             "speed_cap_retry");
  }
  return TRAJECTORY_PLAN_PENDING;
}

static TrajectoryPlanResult selectTrajectory(float goalX, float goalY) {
  // Candidate selection is a cooperative epoch. Every candidate sees one
  // immutable pose/map/sensor snapshot, but only a bounded pair is evaluated
  // per main-loop pass. An incomplete epoch can never publish or renew motion.
  if (!plannerEpoch.active) {
    unsigned long sliceStartedUs = micros();
    TrajectoryPlanResult beginResult = beginPlannerEpoch(goalX, goalY);
    recordPlannerSlice(sliceStartedUs);
    if (beginResult == TRAJECTORY_PLAN_PENDING) {
      notePlannerPending();
    }
    return beginResult;
  }

  if (plannerEpoch.goalStartedMs != navigationGoal.startedMs ||
      plannerEpoch.authority != navigationGoal.authority) {
    resetPlannerEpoch();
    return TRAJECTORY_PLAN_ABORTED;
  }

  unsigned long now = millis();
  plannerTelemetry.plannerEpochAgeMs = now - plannerEpoch.startedMs;
  plannerTelemetry.plannerCommandAgeMs = lastPlannerCommandPublishedMs == 0
    ? 0 : now - lastPlannerCommandPublishedMs;
  if (!plannerEpoch.commandStoppedForAge && isMotorCommandLeaseArmed() &&
      plannerTelemetry.plannerCommandAgeMs >= PLANNER_COMMAND_MAX_AGE_MS) {
    stopMotors();
    plannerEpoch.commandStoppedForAge = true;
    plannerTelemetry.safeStopReason = "planner_command_age_guard";
  }
  if (now - plannerEpoch.startedMs > PLANNER_EPOCH_MAX_AGE_MS) {
    stopMotors();
    closePlannerEpoch();
    finishNavigationGoal(false, PLANNER_STOP_ABORTED, "planner_epoch_timeout");
    return TRAJECTORY_PLAN_ABORTED;
  }

  if (!plannerEpoch.awaitingRevalidation) {
    const int totalCandidates = 2 * PLANNER_CURVATURE_SAMPLES;
    unsigned long sliceStartedUs = micros();
    uint8_t processedThisSlice = 0;
    while (plannerEpoch.candidateIndex < totalCandidates) {
      if (processedThisSlice > 0 &&
          (processedThisSlice >= PLANNER_MAX_CANDIDATES_PER_SLICE ||
           micros() - sliceStartedUs >= PLANNER_SLICE_BUDGET_US)) {
        break;
      }
      int candidateIndex = plannerEpoch.candidateIndex++;
      plannerTelemetry.plannerCandidatesProcessed =
        plannerEpoch.candidateIndex;
      int speedIndex = candidateIndex / PLANNER_CURVATURE_SAMPLES;
      int curvatureIndex = candidateIndex % PLANNER_CURVATURE_SAMPLES;
      float normalized = -1.0f +
        (2.0f * curvatureIndex) / (PLANNER_CURVATURE_SAMPLES - 1);
      float requestedForward = speedIndex == 0
        ? plannerEpoch.speedCap
        : PLANNER_MIN_DRIVABLE_SPEED_TPS;
      float forward = min(requestedForward, 3000.0f /
        (1.0f + fabs(normalized * PLANNER_MAX_TURN_RATIO)));
      float turn = forward * normalized * PLANNER_MAX_TURN_RATIO;

      bool turnPointsToObstacleGoal = true;
      bool turnIsGentleObstacleCountersteer = true;
      if (obstacleContext.active) {
        float signedTurnRatio = turn / max(1.0f, forward);
        float desiredHeadingDeg = atan2f(
          plannerEpoch.goalY - plannerEpoch.startY,
          plannerEpoch.goalX - plannerEpoch.startX) * RAD_TO_DEG;
        float localHeadingErrorDeg = wrapAngle(
          desiredHeadingDeg -
          plannerEpoch.startHeadingRad * RAD_TO_DEG);
        float turnTowardGoalRatio =
          (localHeadingErrorDeg > 0.0f ? 1.0f : -1.0f) *
          signedTurnRatio;
        turnPointsToObstacleGoal =
          fabs(localHeadingErrorDeg) <= 10.0f ||
          turnTowardGoalRatio >= 0.15f;
        turnIsGentleObstacleCountersteer =
          turnTowardGoalRatio >=
            -PLANNER_OBSTACLE_COUNTERSTEER_MAX_RATIO;
      }
      if ((!plannerEpoch.countersteerFallbackPass &&
           !turnPointsToObstacleGoal) ||
          (plannerEpoch.countersteerFallbackPass &&
           (turnPointsToObstacleGoal ||
            !turnIsGentleObstacleCountersteer))) {
        continue;
      }

      if (plannerEpoch.lineFollowActive &&
          plannerEpoch.finalGoalDistanceM <=
            PLANNER_NEAR_GOAL_STRAIGHTEN_DISTANCE_M &&
          fabs(turn / max(1.0f, forward)) >
            PLANNER_LINE_FOLLOW_NEAR_GOAL_MAX_TURN_RATIO) {
        plannerEpoch.skippedLinePolicy++;
        continue;
      }
      
      if (forward < PLANNER_MIN_DRIVABLE_SPEED_TPS) {
        continue;
      }
      processedThisSlice++;

      ForwardCandidateEvaluation evaluation;
      bool rolloutAccepted = evaluateForwardCandidate(
        plannerEpoch, forward, turn, evaluation);
      if (!rolloutAccepted && speedIndex == 0 &&
          evaluation.rejectReason != CANDIDATE_REJECT_TURN_OBSERVABILITY) {
        // For a fixed turn ratio, speed changes how far the same geometric arc
        // is traversed during the horizon. Reuse the last fully checked prefix
        // as this curve's computed safe speed instead of selecting a separate
        // context-specific speed tier.
        float safePrefixSpeed = evaluation.safeTravelM * TICKS_PER_METRE /
                                PLANNER_HORIZON_S - 1.0f;
        if (safePrefixSpeed >= PLANNER_MIN_DRIVABLE_SPEED_TPS &&
            safePrefixSpeed < forward - 0.5f) {
          forward = safePrefixSpeed;
          turn = forward * normalized * PLANNER_MAX_TURN_RATIO;
          rolloutAccepted = evaluateForwardCandidate(
            plannerEpoch, forward, turn, evaluation);
        }
      }
      if (rolloutAccepted && speedIndex == 0 &&
          obstacleContext.active) {
        float clearanceConfidence = constrain(
          evaluation.clearanceMm /
            (PLANNER_PREFERRED_CLEARANCE_M * 1000.0f),
          0.0f, 1.0f);
        float evidenceConfidence = constrain(
          1.0f - evaluation.maximumUnknownFraction, 0.0f, 1.0f);
        float speedConfidence = recoveryBudget.forwardTakeoverPending
          ? 0.0f
          : min(clearanceConfidence, evidenceConfidence);
        float confidenceSpeed = PLANNER_MIN_DRIVABLE_SPEED_TPS +
          (forward - PLANNER_MIN_DRIVABLE_SPEED_TPS) * speedConfidence;
        if (confidenceSpeed < forward - 0.5f) {
          forward = confidenceSpeed;
          turn = forward * normalized * PLANNER_MAX_TURN_RATIO;
          rolloutAccepted = evaluateForwardCandidate(
            plannerEpoch, forward, turn, evaluation);
        }
      }
      if (!rolloutAccepted) {
        if (evaluation.rejectReason == CANDIDATE_REJECT_TURN_OBSERVABILITY) {
          plannerEpoch.rejectedTurnObservability++;
        } else if (evaluation.rejectReason ==
                   CANDIDATE_REJECT_FORWARD_OBSERVATION) {
          plannerEpoch.rejectedForwardObservation++;
        } else if (evaluation.rejectReason == CANDIDATE_REJECT_FOOTPRINT) {
          plannerEpoch.rejectedFootprint++;
        } else if (evaluation.rejectReason == CANDIDATE_REJECT_CORRIDOR) {
          plannerEpoch.rejectedCorridor++;
        }
        continue;
      }
      float obstacleLateralProgressScore = 0.0f;
      if (!obstacleRolloutMakesRequiredLateralProgress(
            plannerEpoch.startX, plannerEpoch.startY,
            evaluation.finalX, evaluation.finalY,
            plannerEpoch.goalX, plannerEpoch.goalY,
            obstacleLateralProgressScore)) {
        continue;
      }
      if (plannerEpoch.countersteerFallbackPass &&
          obstacleLateralProgressScore <
            PLANNER_OBSTACLE_COUNTERSTEER_PROGRESS_FRACTION) {
        continue;
      }

      plannerEpoch.acceptedCount++;
      plannerTelemetry.candidateCount = plannerEpoch.acceptedCount;
      float score = candidateScore(
        forward, turn, plannerEpoch.goalX, plannerEpoch.goalY,
        plannerEpoch.startX, plannerEpoch.startY,
        plannerEpoch.previousSelectedTurn,
        evaluation.closestGoalDistanceM,
        evaluation.headingAtClosestGoalRad,
        evaluation.finalX, evaluation.finalY,
        evaluation.finalHeadingRad, evaluation.clearanceMm,
        plannerEpoch.lineFollowActive,
        navigationGoal.startX, navigationGoal.startY,
        plannerEpoch.routeHeadingRad, plannerEpoch.finalGoalDistanceM);
      if (recoveryBudget.forwardTakeoverPending) {
        // After reverse has deliberately spent route progress, prefer a
        // decisive clearance gain before resuming ordinary goal scoring.
        score += 3.0f * obstacleLateralProgressScore;
      }
      bool reachesGoal = evaluation.arrivalTimeS >= 0.0f;
      bool betterArrival =
        plannerEpoch.finalWaypointIsLocalGoal &&
        !plannerEpoch.lineFollowActive && reachesGoal &&
        (!plannerEpoch.bestReachesGoal ||
         evaluation.arrivalTimeS <
           plannerEpoch.bestArrivalTimeS - 0.0001f);
      bool equalArrivalClass =
        !plannerEpoch.finalWaypointIsLocalGoal ||
        plannerEpoch.lineFollowActive ||
        (reachesGoal == plannerEpoch.bestReachesGoal &&
         (!reachesGoal ||
          fabs(evaluation.arrivalTimeS -
               plannerEpoch.bestArrivalTimeS) <= 0.0001f));
      if (betterArrival ||
          (equalArrivalClass && score > plannerEpoch.bestScore)) {
        plannerEpoch.bestScore = score;
        plannerEpoch.bestForward = forward;
        plannerEpoch.bestTurn = turn;
        plannerEpoch.bestReachesGoal = reachesGoal;
        plannerEpoch.bestArrivalTimeS = evaluation.arrivalTimeS;
      }
    }
    recordPlannerSlice(sliceStartedUs);
    if (plannerEpoch.candidateIndex < totalCandidates) {
      notePlannerPending();
      return TRAJECTORY_PLAN_PENDING;
    }
    if (plannerEpoch.acceptedCount == 0 &&
        obstacleContext.active &&
        !plannerEpoch.countersteerFallbackPass) {
      plannerEpoch.countersteerFallbackPass = true;
      plannerEpoch.candidateIndex = 0;
      plannerTelemetry.replanReason =
        "obstacle_countersteer_fallback";
      notePlannerPending();
      return TRAJECTORY_PLAN_PENDING;
    }
    plannerEpoch.awaitingRevalidation = true;
    notePlannerPending();
    return TRAJECTORY_PLAN_PENDING;
  }

  // The winner was selected from a coherent older snapshot. Re-run that one
  // command against the newest pose/map/sensors before it may reach the motor
  // owner. A changed hazard therefore invalidates publication, never safety.
  unsigned long sliceStartedUs = micros();
  if (plannerEpoch.acceptedCount == 0) {
    recordPlannerSlice(sliceStartedUs);
    return failPlannerEpochNoPath("no_footprint_safe_arc",
                                  "all_arc_candidates_rejected");
  }
  capturePlannerEpochView(plannerEpoch);
  if (!isRangeSensorValid(RANGE_FRONT)) {
    recordPlannerSlice(sliceStartedUs);
    return retryPlannerEpoch(PLANNER_STOP_FRONT_INVALID,
                             "front_sensor_invalid_revalidate",
                             "winner_revalidation_retry");
  }
  if (isRangeSensorBlocked(RANGE_FRONT)) {
    recordPlannerSlice(sliceStartedUs);
    lastForwardNoPathWasGeometric = true;
    plannerTelemetry.stopReason = PLANNER_STOP_FRONT_BLOCKED;
    plannerTelemetry.safeStopReason = "front_blocked_revalidate";
    plannerTelemetry.replanReason = "winner_revalidation_failed";
    closePlannerEpoch();
    return TRAJECTORY_PLAN_NO_PATH;
  }
  float freshSpeedCap =
    calculateSpeedCapTicksPerSec(plannerEpoch.requestedSpeedCap);
  if (freshSpeedCap < PLANNER_MIN_DRIVABLE_SPEED_TPS) {
    recordPlannerSlice(sliceStartedUs);
    return retryPlannerEpoch(PLANNER_STOP_NO_SAFE_TRAJECTORY,
                             "winner_speed_cap_below_drivable_min",
                             "winner_speed_cap_retry");
  }
  float publishForward = plannerEpoch.bestForward;
  float publishTurn = plannerEpoch.bestTurn;
  if (publishForward > freshSpeedCap + 0.5f) {
    float speedScale = freshSpeedCap / publishForward;
    publishForward = freshSpeedCap;
    publishTurn *= speedScale;
  }
  ForwardCandidateEvaluation evaluation;
  bool winnerStillSafe = evaluateForwardCandidate(
    plannerEpoch, publishForward, publishTurn, evaluation);
  recordPlannerSlice(sliceStartedUs);
  if (!winnerStillSafe) {
    return retryPlannerEpoch(PLANNER_STOP_NO_SAFE_TRAJECTORY,
                             "winner_revalidation_rejected",
                             "winner_revalidation_retry");
  }
  float obstacleLateralProgressScore = 0.0f;
  if (!obstacleRolloutMakesRequiredLateralProgress(
        plannerEpoch.startX, plannerEpoch.startY,
        evaluation.finalX, evaluation.finalY,
        plannerEpoch.goalX, plannerEpoch.goalY,
        obstacleLateralProgressScore)) {
    return retryPlannerEpoch(PLANNER_STOP_NO_SAFE_TRAJECTORY,
                             "winner_obstacle_progress_rejected",
                             "winner_revalidation_retry");
  }

  plannerTelemetry.selectedForwardTicksPerSec = publishForward;
  plannerTelemetry.selectedTurnTicksPerSec = publishTurn;
  plannerTelemetry.selectedCurvature = publishTurn /
    max(1.0f, publishForward);
  plannerTelemetry.minimumSweptClearanceMm = evaluation.clearanceMm;
  plannerTelemetry.candidateCount = plannerEpoch.acceptedCount;
  plannerTelemetry.stopReason = PLANNER_STOP_NONE;
  plannerTelemetry.planReason = "best_safe_arc_revalidated";
  plannerTelemetry.replanReason = "local_goal_visible";
  plannerTelemetry.safeStopReason = "";
  lastReportedStopReason = PLANNER_STOP_NONE;
  candidateRejectsReported = false;
  motorStopRequested = false;
  bool published = publishNavigationMotion(publishForward, publishTurn);
  if (!published) {
    return retryPlannerEpoch(PLANNER_STOP_NO_SAFE_TRAJECTORY,
                             "winner_publication_vetoed",
                             "winner_publication_retry");
  }
  lastPlannerCommandPublishedMs = millis();
  plannerTelemetry.plannerCommandAgeMs = 0;
  plannerTelemetry.lastPlanMs = lastPlannerCommandPublishedMs;
  lastPlannerUpdateMs = lastPlannerCommandPublishedMs;
  closePlannerEpoch();
  return TRAJECTORY_PLAN_SUCCESS;
}

static void reportPlannerStopIfChanged() {
  // Avoid spamming the Bluetooth stream every 40 ms while the same safe-stop
  // condition persists. A changed reason is an event worth investigating.
  if (plannerTelemetry.stopReason == lastReportedStopReason) {
    return;
  }
  lastReportedStopReason = plannerTelemetry.stopReason;
  sendBluetoothEvent("planner_safe_stop", plannerStopReasonName(plannerTelemetry.stopReason));
}


static float forwardContinuationQuality(
    const PlannerCollisionSnapshot &snapshot,
    float startX, float startY, float startHeadingRad,
    float goalX, float goalY) {
  const float startDistanceM = sqrtf((goalX - startX) * (goalX - startX) +
                                     (goalY - startY) * (goalY - startY));
  float bestQuality = 0.0f;
  const float normalizedTurns[] = {-1.0f, -0.5f, 0.0f, 0.5f, 1.0f};
  for (unsigned int index = 0;
       index < sizeof(normalizedTurns) / sizeof(normalizedTurns[0]); ++index) {
    float forwardTicks = PLANNER_MIN_DRIVABLE_SPEED_TPS;
    float turnTicks = forwardTicks * normalizedTurns[index] *
                      PLANNER_MAX_TURN_RATIO;
    float leftMps = leftWheelTargetFromChassis(forwardTicks, turnTicks) /
                    TICKS_PER_METRE;
    float rightMps = rightWheelTargetFromChassis(forwardTicks, turnTicks) /
                     TICKS_PER_METRE;
    float linearMps = (leftMps + rightMps) * 0.5f;
    float angularRadPerSec = navigationOmegaFromWheelSpeeds(
      leftMps, rightMps, EFFECTIVE_TRACK_WIDTH_M);
    float x = startX;
    float y = startY;
    float heading = startHeadingRad;
    bool safe = true;
    for (float elapsed = 0.0f; elapsed < PLANNER_HORIZON_S;
         elapsed += PLANNER_ROLLOUT_STEP_S) {
      float midpointHeading = heading +
        angularRadPerSec * PLANNER_ROLLOUT_STEP_S * 0.5f;
      x += linearMps * cosf(midpointHeading) * PLANNER_ROLLOUT_STEP_S;
      y += linearMps * sinf(midpointHeading) * PLANNER_ROLLOUT_STEP_S;
      heading += angularRadPerSec * PLANNER_ROLLOUT_STEP_S;
      if (!footprintKnownClearOnSnapshot(snapshot, x, y, heading)) {
        safe = false;
        break;
      }
    }
    if (!safe) {
      continue;
    }
    float finalDistanceM = sqrtf((goalX - x) * (goalX - x) +
                                 (goalY - y) * (goalY - y));
    float progress = constrain((startDistanceM - finalDistanceM) /
                               max(PLANNER_MIN_PROGRESS_M, startDistanceM),
                               0.0f, 1.0f);
    float desiredHeadingDeg = atan2f(goalY - y, goalX - x) * RAD_TO_DEG;
    float headingErrorDeg = fabs(wrapAngle(
      desiredHeadingDeg - heading * RAD_TO_DEG));
    float headingQuality = 1.0f - min(1.0f, headingErrorDeg / 90.0f);
    bestQuality = max(bestQuality, 0.7f * progress + 0.3f * headingQuality);
  }
  return bestQuality;
}

static float calculateReverseRecoverySpeedCapTicksPerSec() {
  if (!hasTrustedRearCoverage() ||
      !isRangeSensorCurrent(RANGE_FAKE_REAR) ||
      isRangeSensorBlocked(RANGE_FAKE_REAR)) {
    return 0.0f;
  }

  float availableM = getRangeSensorDistance(RANGE_FAKE_REAR) / 1000.0f -
                     PLANNER_TOTAL_HARD_CLEARANCE_M -
                     PLANNER_REVERSE_RECOVERY_REAR_BUFFER_M;
  availableM = max(0.0f, availableM);
  float a = PLANNER_MAX_DECELERATION_MPS2;
  float latency = PLANNER_SENSING_LATENCY_S;
  float speedMps = -a * latency + sqrtf(a * a * latency * latency + 2.0f * a * availableM);
  speedMps = max(0.0f, speedMps);
  return min(PLANNER_REVERSE_RECOVERY_MAX_SPEED_TPS,
             min(baseTargetSpeed, speedMps * TICKS_PER_METRE));
}

static bool rolloutReverseRecoveryCandidate(const ReversePlannerEpoch &epoch,
                                            float reverseTicks, float turnTicks,
                                            float goalX, float goalY,
                                            float &rearClearanceMm,
                                            float &endpointClearanceM,
                                            float &sweepClearanceM,
                                            float &forwardQuality,
                                            float &unexploredScore,
                                            float &finalX,
                                            float &finalY,
                                            float &finalHeadingRad,
                                            CandidateRejectReason &rejectReason) {
  if (reverseTicks >= 0.0f) {
    rejectReason = CANDIDATE_REJECT_REAR_OBSERVATION;
    return false;
  }
  const float leftTicks = leftWheelTargetFromChassis(reverseTicks, turnTicks);
  const float rightTicks = rightWheelTargetFromChassis(reverseTicks, turnTicks);
  if (leftTicks >= 0.0f || rightTicks >= 0.0f) {
    rejectReason = CANDIDATE_REJECT_REAR_OBSERVATION;
    return false;
  }
  if (!epoch.rearValid || epoch.rearBlocked) {
    rejectReason = CANDIDATE_REJECT_REAR_OBSERVATION;
    return false;
  }

  float observedRearM = epoch.observedRearM;
  float trailingEnvelopeM = PLANNER_TOTAL_HARD_CLEARANCE_M +
                            PLANNER_REVERSE_RECOVERY_REAR_BUFFER_M;
  float heading = epoch.startHeadingRad;
  float x = epoch.startX;
  float y = epoch.startY;
  float leftMps = leftWheelTargetFromChassis(reverseTicks, turnTicks) /
                  TICKS_PER_METRE;
  float rightMps = rightWheelTargetFromChassis(reverseTicks, turnTicks) /
                   TICKS_PER_METRE;
  float linearMps = (leftMps + rightMps) * 0.5f;
  float angularRadPerSec = navigationOmegaFromWheelSpeeds(
    leftMps, rightMps, EFFECTIVE_TRACK_WIDTH_M);
  if (leftMps >= 0.0f || rightMps >= 0.0f) {
    rejectReason = CANDIDATE_REJECT_REAR_OBSERVATION;
    return false;
  }
  float travelledRearM = 0.0f;
  sweepClearanceM = PLANNER_REVERSE_CLEARANCE_CAP_M;

  for (float elapsed = 0.0f; elapsed < PLANNER_HORIZON_S; elapsed += PLANNER_ROLLOUT_STEP_S) {
    float midpointHeading = heading + angularRadPerSec * PLANNER_ROLLOUT_STEP_S * 0.5f;
    x += linearMps * cosf(midpointHeading) * PLANNER_ROLLOUT_STEP_S;
    y += linearMps * sinf(midpointHeading) * PLANNER_ROLLOUT_STEP_S;
    heading += angularRadPerSec * PLANNER_ROLLOUT_STEP_S;
    travelledRearM = -linearMps * (elapsed + PLANNER_ROLLOUT_STEP_S);

    if (travelledRearM + trailingEnvelopeM > observedRearM) {
      rejectReason = CANDIDATE_REJECT_REAR_OBSERVATION;
      return false;
    }
    // Reverse recovery uses the exact same inflated hard-footprint predicate
    // as forward planning. Direction-specific recovery logic may constrain a
    // candidate further, but it may never weaken collision rejection.
    if (!footprintClearOnSnapshot(epoch.collision, x, y, heading)) {
      rejectReason = CANDIDATE_REJECT_FOOTPRINT;
      return false;
    }
    if (footprintUnknownFractionOnSnapshot(epoch.collision, x, y, heading) >
        epoch.allowedUnknownFraction) {
      rejectReason = CANDIDATE_REJECT_CLEAR_EVIDENCE;
      return false;
    }
    sweepClearanceM = min(sweepClearanceM,
      poseKnownClearanceM(epoch.collision, x, y, heading));
  }

  rearClearanceMm = max(0.0f, (observedRearM - travelledRearM - trailingEnvelopeM) * 1000.0f);
  endpointClearanceM = poseKnownClearanceM(epoch.collision, x, y, heading);
  forwardQuality = forwardContinuationQuality(epoch.collision,
    x, y, heading, goalX, goalY);
  unexploredScore = projectedUnexploredScore(epoch.collision, x, y, heading);
  finalX = x;
  finalY = y;
  finalHeadingRad = heading;
  rejectReason = CANDIDATE_REJECT_NONE;
  return true;
}

static float reverseRecoveryScore(float reverseTicks, float turnTicks,
                                  float previousSelectedTurn,
                                  float sweepClearanceM,
                                  float forwardQuality,
                                  float unexploredScore) {
  float curvature = fabs(turnTicks) / max(1.0f, fabs(reverseTicks));
  float smoothnessScore = 1.0f - min(1.0f, fabs(turnTicks - previousSelectedTurn) /
                                          max(1.0f, baseTargetSpeed));
  float sweepScore = constrain(sweepClearanceM /
    PLANNER_REVERSE_CLEARANCE_CAP_M, 0.0f, 1.0f);
  float efficiencyScore = 0.5f * smoothnessScore +
    0.5f * (1.0f - min(1.0f, curvature /
      PLANNER_REVERSE_RECOVERY_MAX_TURN_RATIO));
  return PLANNER_REVERSE_FORWARD_QUALITY_WEIGHT * forwardQuality +
         PLANNER_REVERSE_UNEXPLORED_WEIGHT * unexploredScore +
         PLANNER_REVERSE_SWEEP_CLEARANCE_WEIGHT * sweepScore +
         PLANNER_REVERSE_EFFICIENCY_WEIGHT * efficiencyScore;
}

static void captureReversePlannerEpochView(ReversePlannerEpoch &epoch) {
  epoch.startX = robotX;
  epoch.startY = robotY;
  epoch.startHeadingRad = navigationHeadingRad();
  epoch.rearValid = hasTrustedRearCoverage() &&
                    isRangeSensorCurrent(RANGE_FAKE_REAR);
  epoch.rearBlocked = isRangeSensorBlocked(RANGE_FAKE_REAR);
  epoch.observedRearM = epoch.rearValid
    ? getRangeSensorDistance(RANGE_FAKE_REAR) / 1000.0f : 0.0f;
  epoch.allowedUnknownFraction = currentReverseUnknownAllowance();
  capturePlannerCollisionSnapshot(epoch.collision);
}

static void recordReversePlannerSlice(unsigned long sliceStartedUs) {
  unsigned long sliceUs = micros() - sliceStartedUs;
  reversePlannerEpoch.accumulatedWorkUs += sliceUs;
  plannerTelemetry.plannerSliceUs = sliceUs;
  plannerTelemetry.plannerSliceMaxUs =
    max(plannerTelemetry.plannerSliceMaxUs, sliceUs);
  plannerTelemetry.plannerEpochWorkUs =
    reversePlannerEpoch.accumulatedWorkUs;
  plannerTelemetry.plannerEpochMaxWorkUs =
    max(plannerTelemetry.plannerEpochMaxWorkUs,
        reversePlannerEpoch.accumulatedWorkUs);
  recordMainLoopPhaseDuration("reverse_planner_slice", sliceStartedUs);
}

static void closeReversePlannerEpoch() {
  reversePlannerEpoch.active = false;
  reversePlannerEpoch.awaitingRevalidation = false;
  plannerTelemetry.plannerEpochActive = false;
  plannerTelemetry.plannerEpochAgeMs =
    millis() - reversePlannerEpoch.startedMs;
}

static TrajectoryPlanResult retryReversePlannerEpoch(const char* safeReason,
                                                     const char* replanReason) {
  stopMotors();
  plannerTelemetry.stopReason = PLANNER_STOP_NO_SAFE_TRAJECTORY;
  plannerTelemetry.safeStopReason = safeReason;
  plannerTelemetry.replanReason = replanReason;
  closeReversePlannerEpoch();
  return TRAJECTORY_PLAN_RETRY;
}

static TrajectoryPlanResult beginReversePlannerEpoch(float goalX, float goalY) {
  // Captures the snapshot used to evaluate reverse recovery arcs. In the
  // simulator, rear evidence comes from the field-raycast rear channel.
  memset(&reversePlannerEpoch, 0, sizeof(reversePlannerEpoch));
  reversePlannerEpoch.active = true;
  reversePlannerEpoch.startedMs = millis();
  reversePlannerEpoch.goalStartedMs = navigationGoal.startedMs;
  reversePlannerEpoch.authority = navigationGoal.authority;
  reversePlannerEpoch.emergencyScanObjective =
    emergencyRecoveryState.phase == EMERGENCY_RECOVERY_RELOCATE;
  reversePlannerEpoch.goalX = goalX;
  reversePlannerEpoch.goalY = goalY;
  reversePlannerEpoch.bestScore = -1000000.0f;
  reversePlannerEpoch.bestClearanceBand = -1;
  reversePlannerEpoch.previousSelectedTurn =
    plannerTelemetry.selectedTurnTicksPerSec;
  captureReversePlannerEpochView(reversePlannerEpoch);
  reversePlannerEpoch.speedCap =
    calculateReverseRecoverySpeedCapTicksPerSec();
  plannerTelemetry.speedCapTicksPerSec = reversePlannerEpoch.speedCap;
  plannerTelemetry.localGoalDistanceM = sqrtf(
    (goalX - reversePlannerEpoch.startX) *
      (goalX - reversePlannerEpoch.startX) +
    (goalY - reversePlannerEpoch.startY) *
      (goalY - reversePlannerEpoch.startY));
  plannerTelemetry.candidateCount = 0;
  plannerTelemetry.plannerCandidatesProcessed = 0;
  plannerTelemetry.plannerYieldCount = 0;
  plannerTelemetry.plannerEpochWorkUs = 0;
  plannerTelemetry.plannerEpochAgeMs = 0;
  plannerTelemetry.plannerEpochActive = true;
  if (reversePlannerEpoch.speedCap <
      PLANNER_REVERSE_RECOVERY_MIN_SPEED_TPS) {
    plannerTelemetry.stopReason = PLANNER_STOP_NO_SAFE_TRAJECTORY;
    plannerTelemetry.safeStopReason = "rear_path_unavailable";
    plannerTelemetry.replanReason = "trusted_rear_unavailable";
    closeReversePlannerEpoch();
    return TRAJECTORY_PLAN_NO_PATH;
  }
  return TRAJECTORY_PLAN_PENDING;
}

static TrajectoryPlanResult selectReverseRecoveryTrajectory(float goalX,
                                                             float goalY) {
  // Cooperative reverse-arc sampler. It mirrors selectTrajectory(): evaluate
  // a bounded number of candidates per loop, then revalidate the winner before
  // publication.
  if (!reversePlannerEpoch.active) {
    unsigned long sliceStartedUs = micros();
    TrajectoryPlanResult beginResult =
      beginReversePlannerEpoch(goalX, goalY);
    recordReversePlannerSlice(sliceStartedUs);
    if (beginResult == TRAJECTORY_PLAN_PENDING) {
      reversePlannerEpoch.yieldCount++;
      plannerTelemetry.plannerYieldCount = reversePlannerEpoch.yieldCount;
      plannerTelemetry.planReason = "reverse_planner_epoch_pending";
    }
    return beginResult;
  }
  if (reversePlannerEpoch.goalStartedMs != navigationGoal.startedMs ||
      reversePlannerEpoch.authority != navigationGoal.authority) {
    resetReversePlannerEpoch();
    return TRAJECTORY_PLAN_ABORTED;
  }

  unsigned long now = millis();
  plannerTelemetry.plannerEpochAgeMs =
    now - reversePlannerEpoch.startedMs;
  plannerTelemetry.plannerCommandAgeMs = lastPlannerCommandPublishedMs == 0
    ? 0 : now - lastPlannerCommandPublishedMs;
  if (!reversePlannerEpoch.commandStoppedForAge &&
      isMotorCommandLeaseArmed() &&
      plannerTelemetry.plannerCommandAgeMs >= PLANNER_COMMAND_MAX_AGE_MS) {
    stopMotors();
    reversePlannerEpoch.commandStoppedForAge = true;
    plannerTelemetry.safeStopReason = "planner_command_age_guard";
  }
  if (now - reversePlannerEpoch.startedMs > PLANNER_EPOCH_MAX_AGE_MS) {
    stopMotors();
    closeReversePlannerEpoch();
    finishNavigationGoal(false, PLANNER_STOP_ABORTED,
                         "reverse_planner_epoch_timeout");
    return TRAJECTORY_PLAN_ABORTED;
  }

  if (!reversePlannerEpoch.awaitingRevalidation) {
    const int totalCandidates =
      2 * PLANNER_REVERSE_RECOVERY_CURVATURE_SAMPLES;
    unsigned long sliceStartedUs = micros();
    uint8_t processedThisSlice = 0;
    while (reversePlannerEpoch.candidateIndex < totalCandidates) {
      if (processedThisSlice > 0 &&
          (processedThisSlice >= PLANNER_MAX_CANDIDATES_PER_SLICE ||
           micros() - sliceStartedUs >= PLANNER_SLICE_BUDGET_US)) {
        break;
      }
      int candidateIndex = reversePlannerEpoch.candidateIndex++;
      processedThisSlice++;
      plannerTelemetry.plannerCandidatesProcessed =
        reversePlannerEpoch.candidateIndex;
      int speedIndex = candidateIndex /
        PLANNER_REVERSE_RECOVERY_CURVATURE_SAMPLES;
      int curvatureIndex = candidateIndex %
        PLANNER_REVERSE_RECOVERY_CURVATURE_SAMPLES;
      float speedScale = speedIndex == 0
        ? 1.0f : PLANNER_REVERSE_RECOVERY_MIN_SPEED_SCALE;
      float reverseMagnitude = max(
        PLANNER_REVERSE_RECOVERY_MIN_SPEED_TPS,
        reversePlannerEpoch.speedCap * speedScale);
      reverseMagnitude = min(reverseMagnitude,
                             PLANNER_REVERSE_RECOVERY_MAX_SPEED_TPS);
      float normalized = -1.0f + (2.0f * curvatureIndex) /
        (PLANNER_REVERSE_RECOVERY_CURVATURE_SAMPLES - 1);
      float reverseTicks = -reverseMagnitude;
      float turnTicks = reverseMagnitude * normalized *
                        PLANNER_REVERSE_RECOVERY_MAX_TURN_RATIO;
      float rearClearanceMm = -1.0f;
      float endpointClearanceM = 0.0f;
      float sweepClearanceM = 0.0f;
      float forwardQuality = 0.0f;
      float unexploredScore = 0.0f;
      float finalX = reversePlannerEpoch.startX;
      float finalY = reversePlannerEpoch.startY;
      float finalHeadingRad = reversePlannerEpoch.startHeadingRad;
      CandidateRejectReason rejectReason = CANDIDATE_REJECT_NONE;
      if (!rolloutReverseRecoveryCandidate(
            reversePlannerEpoch, reverseTicks, turnTicks,
            reversePlannerEpoch.goalX, reversePlannerEpoch.goalY,
            rearClearanceMm, endpointClearanceM, sweepClearanceM,
            forwardQuality, unexploredScore,
            finalX, finalY, finalHeadingRad, rejectReason)) {
        if (rejectReason == CANDIDATE_REJECT_REAR_OBSERVATION) {
          reversePlannerEpoch.rejectedRear++;
        } else if (rejectReason == CANDIDATE_REJECT_FOOTPRINT) {
          reversePlannerEpoch.rejectedFootprint++;
        } else if (rejectReason == CANDIDATE_REJECT_CLEAR_EVIDENCE) {
          reversePlannerEpoch.rejectedEvidence++;
        }
        continue;
      }
      reversePlannerEpoch.acceptedCount++;
      plannerTelemetry.candidateCount =
        reversePlannerEpoch.acceptedCount;
      float rotationalClearanceM = endpointClearanceM;
      if (reversePlannerEpoch.emergencyScanObjective) {
        rotationalClearanceM = rotationalEnvelopeClearanceM(
          reversePlannerEpoch.collision, finalX, finalY);
      }
      float score = reverseRecoveryScore(
        reverseTicks, turnTicks,
        reversePlannerEpoch.previousSelectedTurn,
        sweepClearanceM, forwardQuality, unexploredScore);
      if (reversePlannerEpoch.emergencyScanObjective) {
        score += constrain(
          rotationalClearanceM / PLANNER_REVERSE_CLEARANCE_CAP_M,
          0.0f, 1.0f);
      }
      float selectionClearanceM =
        reversePlannerEpoch.emergencyScanObjective
          ? rotationalClearanceM : endpointClearanceM;
      int clearanceBand = (int)floorf(
        min(selectionClearanceM, PLANNER_REVERSE_CLEARANCE_CAP_M) /
        PLANNER_REVERSE_CLEARANCE_BAND_M + 0.0001f);
      if (clearanceBand > reversePlannerEpoch.bestClearanceBand ||
          (clearanceBand == reversePlannerEpoch.bestClearanceBand &&
           score > reversePlannerEpoch.bestScore)) {
        reversePlannerEpoch.bestClearanceBand = clearanceBand;
        reversePlannerEpoch.bestScore = score;
        reversePlannerEpoch.bestReverse = reverseTicks;
        reversePlannerEpoch.bestTurn = turnTicks;
      }
    }
    recordReversePlannerSlice(sliceStartedUs);
    if (reversePlannerEpoch.candidateIndex < totalCandidates) {
      reversePlannerEpoch.yieldCount++;
      plannerTelemetry.plannerYieldCount = reversePlannerEpoch.yieldCount;
      plannerTelemetry.planReason = "reverse_planner_epoch_pending";
      return TRAJECTORY_PLAN_PENDING;
    }
    reversePlannerEpoch.awaitingRevalidation = true;
    reversePlannerEpoch.yieldCount++;
    plannerTelemetry.plannerYieldCount = reversePlannerEpoch.yieldCount;
    plannerTelemetry.planReason = "reverse_planner_revalidating";
    return TRAJECTORY_PLAN_PENDING;
  }

  unsigned long sliceStartedUs = micros();
  if (reversePlannerEpoch.acceptedCount == 0) {
    recordReversePlannerSlice(sliceStartedUs);
    const bool waitingForEvidence =
      reversePlannerEpoch.rejectedEvidence > 0 &&
      reversePlannerEpoch.rearValid &&
      !reversePlannerEpoch.rearBlocked;
    plannerTelemetry.stopReason = PLANNER_STOP_NO_SAFE_TRAJECTORY;
    plannerTelemetry.safeStopReason = waitingForEvidence
      ? "reverse_waiting_for_evidence" : "no_reverse_recovery_arc";
    plannerTelemetry.replanReason = waitingForEvidence
      ? "reverse_unknown_allowance_ramp" : "no_reverse_arc";
    if (!reverseRecoveryRejectsReported) {
      char detail[64];
      snprintf(detail, sizeof(detail), "rear=%d;footprint=%d;evidence=%d",
               reversePlannerEpoch.rejectedRear,
               reversePlannerEpoch.rejectedFootprint,
               reversePlannerEpoch.rejectedEvidence);
      sendBluetoothEvent("reverse_recovery_rejects", detail);
      reverseRecoveryRejectsReported = true;
    }
    closeReversePlannerEpoch();
    return waitingForEvidence
      ? TRAJECTORY_PLAN_RETRY : TRAJECTORY_PLAN_NO_PATH;
  }

  captureReversePlannerEpochView(reversePlannerEpoch);
  float freshSpeedCap = calculateReverseRecoverySpeedCapTicksPerSec();
  if (freshSpeedCap < PLANNER_REVERSE_RECOVERY_MIN_SPEED_TPS) {
    recordReversePlannerSlice(sliceStartedUs);
    return retryReversePlannerEpoch("reverse_speed_cap_below_drivable_min",
                                    "reverse_speed_cap_retry");
  }
  float publishReverse = reversePlannerEpoch.bestReverse;
  float publishTurn = reversePlannerEpoch.bestTurn;
  float selectedMagnitude = fabs(publishReverse);
  if (selectedMagnitude > freshSpeedCap + 0.5f) {
    float speedScale = freshSpeedCap / selectedMagnitude;
    publishReverse = -freshSpeedCap;
    publishTurn *= speedScale;
  }
  float rearClearanceMm = -1.0f;
  float endpointClearanceM = 0.0f;
  float sweepClearanceM = 0.0f;
  float forwardQuality = 0.0f;
  float unexploredScore = 0.0f;
  float finalX = reversePlannerEpoch.startX;
  float finalY = reversePlannerEpoch.startY;
  float finalHeadingRad = reversePlannerEpoch.startHeadingRad;
  CandidateRejectReason rejectReason = CANDIDATE_REJECT_NONE;
  bool winnerStillSafe = rolloutReverseRecoveryCandidate(
      reversePlannerEpoch,
      publishReverse, publishTurn,
      reversePlannerEpoch.goalX, reversePlannerEpoch.goalY,
      rearClearanceMm, endpointClearanceM, sweepClearanceM,
      forwardQuality, unexploredScore,
      finalX, finalY, finalHeadingRad, rejectReason);
  recordReversePlannerSlice(sliceStartedUs);
  if (!winnerStillSafe) {
    return retryReversePlannerEpoch("reverse_winner_revalidation_rejected",
                                    "reverse_winner_revalidation_retry");
  }

  reverseRecoveryState.currentClearanceM = poseKnownClearanceM(
    reversePlannerEpoch.collision,
    reversePlannerEpoch.startX, reversePlannerEpoch.startY,
    reversePlannerEpoch.startHeadingRad);
  reverseRecoveryState.clearanceGainM = endpointClearanceM -
    reverseRecoveryState.currentClearanceM;
  if (reversePlannerEpoch.emergencyScanObjective) {
    float rotationalClearanceM = rotationalEnvelopeClearanceM(
      reversePlannerEpoch.collision, finalX, finalY);
    emergencyRecoveryState.bestRotationalClearanceM = max(
      emergencyRecoveryState.bestRotationalClearanceM,
      rotationalClearanceM);
  }
  if (reverseRecoveryState.clearanceGainM <=
      PLANNER_REVERSE_CLEARANCE_GAIN_M) {
    if (reverseRecoveryState.plateauCount <
        PLANNER_REVERSE_PLATEAU_EPOCHS) {
      reverseRecoveryState.plateauCount++;
    }
  } else {
    reverseRecoveryState.plateauCount = 0;
  }
  plannerTelemetry.recoveryCurrentClearanceM =
    reverseRecoveryState.currentClearanceM;
  plannerTelemetry.recoveryEndpointClearanceM = endpointClearanceM;
  plannerTelemetry.recoveryClearanceGainM =
    reverseRecoveryState.clearanceGainM;
  plannerTelemetry.recoveryUnexploredScore = unexploredScore;
  plannerTelemetry.recoveryPlateauCount =
    reverseRecoveryState.plateauCount;

  reverseRecoveryStepCount++;
  plannerTelemetry.selectedForwardTicksPerSec =
    publishReverse;
  plannerTelemetry.selectedTurnTicksPerSec = publishTurn;
  plannerTelemetry.selectedCurvature = publishTurn /
    max(1.0f, fabs(publishReverse));
  plannerTelemetry.minimumSweptClearanceMm =
    min(rearClearanceMm, sweepClearanceM * 1000.0f);
  plannerTelemetry.stopReason = PLANNER_STOP_NONE;
  plannerTelemetry.planReason = "reverse_recovery_arc_revalidated";
  plannerTelemetry.replanReason = "no_forward_path";
  plannerTelemetry.safeStopReason = "";
  lastReportedStopReason = PLANNER_STOP_NONE;
  reverseRecoveryRejectsReported = false;
  motorStopRequested = false;
  if (!publishNavigationMotion(publishReverse, publishTurn)) {
    return retryReversePlannerEpoch("reverse_winner_publication_vetoed",
                                    "reverse_winner_publication_retry");
  }
  lastPlannerCommandPublishedMs = millis();
  plannerTelemetry.plannerCommandAgeMs = 0;
  plannerTelemetry.lastPlanMs = lastPlannerCommandPublishedMs;
  lastPlannerUpdateMs = lastPlannerCommandPublishedMs;
  closeReversePlannerEpoch();
  if (reverseRecoveryStepCount == 1 ||
      (reverseRecoveryStepCount % 20) == 0) {
    sendBluetoothEvent("reverse_recovery_step", "arc_selected");
  }
  return TRAJECTORY_PLAN_SUCCESS;
}

static void recordEmergencySensorBaselines() {
  for (int i = RANGE_RIGHT_OUTER; i <= RANGE_LEFT_OUTER; ++i) {
    emergencyRecoveryState.fanReadBaselineMs[i - RANGE_RIGHT_OUTER] =
      rangeSensors[i].lastReadMs;
  }
  emergencyRecoveryState.rearFrameBaseline =
    getRearObstacleFrameSequence();
}

static bool emergencySensorsAdvancedSinceBaseline() {
  if (!emergencySensorFramesCurrent()) {
    return false;
  }
  for (int i = RANGE_RIGHT_OUTER; i <= RANGE_LEFT_OUTER; ++i) {
    if (rangeSensors[i].lastReadMs <=
        emergencyRecoveryState
          .fanReadBaselineMs[i - RANGE_RIGHT_OUTER]) {
      return false;
    }
  }
  uint32_t rearSequence = getRearObstacleFrameSequence();
  return rearSequence != 0 &&
         rearSequence != emergencyRecoveryState.rearFrameBaseline;
}

static void abortEmergencyRecovery(const char* detail) {
  motorStopRequested = true;
  requestMotionStop();
  sendBluetoothEvent("emergency_scan_abort", detail);
  finishNavigationGoal(false, PLANNER_STOP_EMERGENCY_SCAN_ABORTED, detail);
}

static int8_t chooseEmergencyScanDirection() {
  float leftClearanceMm = min(
    getFanSweepClearanceMm(RANGE_LEFT_INNER),
    getFanSweepClearanceMm(RANGE_LEFT_OUTER));
  float rightClearanceMm = min(
    getFanSweepClearanceMm(RANGE_RIGHT_INNER),
    getFanSweepClearanceMm(RANGE_RIGHT_OUTER));
  return leftClearanceMm >= rightClearanceMm ? 1 : -1;
}

static void startEmergencyScanAtCurrentPose() {
  resetReversePlannerEpoch();
  reverseRecoveryActive = false;
  reverseRecoveryState = {};
  plannerTelemetry.reverseRecoveryActive = false;
  emergencyRecoveryState.phase = EMERGENCY_RECOVERY_SCAN_TURN;
  emergencyRecoveryState.phaseStartedMs = millis();
  emergencyRecoveryState.scanDirection = chooseEmergencyScanDirection();
  emergencyRecoveryState.scanSector = 0;
  emergencyRecoveryState.scanLastYawDeg = navigationHeadingDeg();
  emergencyRecoveryState.scanAccumulatedDeg = 0.0f;
  emergencyRecoveryState.scanTargetAccumulatedDeg =
    PLANNER_EMERGENCY_SCAN_STEP_DEG;
  turnSideInvalidSinceMs = 0;
  turnSweepInvalidSinceMs = 0;
  resetTurnStuckCheck(navigationHeadingDeg());
  plannerTelemetry.planReason = "emergency_scan_turn";
  plannerTelemetry.replanReason = "stationary_scan";
}

static void enterEmergencyRelocation(const char* detail) {
  motorStopRequested = true;
  requestMotionStop();
  resetReversePlannerEpoch();
  if (emergencyRecoveryState.relocationStartedMs == 0) {
    emergencyRecoveryState.relocationStartedMs = millis();
    emergencyRecoveryState.relocationCheckpointX = robotX;
    emergencyRecoveryState.relocationCheckpointY = robotY;
    sendBluetoothEvent("emergency_relocation_start", detail);
  }
  emergencyRecoveryState.phase = EMERGENCY_RECOVERY_RELOCATE;
  emergencyRecoveryState.phaseStartedMs = millis();
  reverseRecoveryActive = true;
  reverseRecoveryStartedMs = millis();
  reverseRecoveryStepCount = 0;
  reverseRecoveryState = {};
  plannerTelemetry.reverseRecoveryActive = true;
  plannerTelemetry.planReason = "emergency_relocation";
  plannerTelemetry.replanReason = detail;
}

static void enterEmergencyScanUnwind(const char* detail) {
  motorStopRequested = true;
  requestMotionStop();
  resetReversePlannerEpoch();
  emergencyRecoveryState.phase =
    EMERGENCY_RECOVERY_SCAN_UNWIND;
  emergencyRecoveryState.phaseStartedMs = millis();
  emergencyRecoveryState.scanLastYawDeg =
    navigationHeadingDeg();
  turnSideInvalidSinceMs = 0;
  turnSweepInvalidSinceMs = 0;
  resetTurnStuckCheck(navigationHeadingDeg());
  plannerTelemetry.planReason = "emergency_scan_unwind";
  plannerTelemetry.replanReason = detail;
  sendBluetoothEvent("emergency_scan_blocked", detail);
}

static void enterEmergencySensorSettle(EmergencyRecoveryPhase phase,
                                       const char* planReason) {
  motorStopRequested = true;
  requestMotionStop();
  resetReversePlannerEpoch();
  reverseRecoveryActive = false;
  plannerTelemetry.reverseRecoveryActive = false;
  emergencyRecoveryState.phase = phase;
  emergencyRecoveryState.phaseStartedMs = millis();
  recordEmergencySensorBaselines();
  plannerTelemetry.planReason = planReason;
}

static void completeEmergencyScanAndPrepareRetry() {
  motorStopRequested = true;
  requestMotionStop();
  reverseRecoveryActive = false;
  reverseRecoveryState = {};
  resetPlannerEpoch();
  resetReversePlannerEpoch();
  resetGeometricNoPathEvidence();
  resetRecoveryBudget();
  resetObstacleContext("emergency_scan_retry");
  pointAlignTurnActive = false;
  pointAlignTurnDirection = 0.0f;
  turnBrakeActive = false;
  turnSideInvalidSinceMs = 0;
  turnSweepInvalidSinceMs = 0;
  resetTurnStuckCheck(navigationHeadingDeg());
  emergencyRecoveryState.phase = EMERGENCY_RECOVERY_RETRY_ACTIVE;
  emergencyRecoveryState.retryActive = true;
  emergencyRecoveryState.phaseStartedMs = millis();
  lastPlannerUpdateMs = 0;
  plannerTelemetry.reverseRecoveryActive = false;
  plannerTelemetry.stopReason = PLANNER_STOP_NONE;
  plannerTelemetry.planReason = "emergency_retry_ready";
  plannerTelemetry.replanReason = "scan_complete";
  plannerTelemetry.safeStopReason = "";
  sendBluetoothEvent("emergency_scan_complete", "full_revolution");
  sendBluetoothEvent("emergency_retry_start", "original_goal");
}

static void updateEmergencyRecovery() {
  EmergencyRecoveryState &state = emergencyRecoveryState;
  const unsigned long now = millis();
  float translationM = hypotf(robotX - state.lastX, robotY - state.lastY);
  state.lastX = robotX;
  state.lastY = robotY;
  state.relocationDistanceM += translationM;
  plannerTelemetry.cumulativeReverseDistanceM =
    state.relocationDistanceM;

  if (now - state.startedMs > PLANNER_EMERGENCY_TOTAL_TIMEOUT_MS) {
    abortEmergencyRecovery("emergency_total_timeout");
    return;
  }

  if (state.phase == EMERGENCY_RECOVERY_SETTLE_CURRENT) {
    motorStopRequested = true;
    requestMotionStop();
    if (emergencySensorsAdvancedSinceBaseline()) {
      startEmergencyScanAtCurrentPose();
      return;
    }
    if (now - state.phaseStartedMs >
        PLANNER_EMERGENCY_SENSOR_WAIT_MS) {
      abortEmergencyRecovery("emergency_initial_sensor_timeout");
    }
    return;
  }

  if (state.phase == EMERGENCY_RECOVERY_SCAN_TURN) {
    if (!emergencySensorFramesCurrent()) {
      abortEmergencyRecovery("emergency_scan_sensor_unhealthy");
      return;
    }
    if (isRangeSensorBlocked(RANGE_FAKE_REAR)) {
      enterEmergencyScanUnwind("scan_rear_became_blocked");
      return;
    }
    float headingDeg = navigationHeadingDeg();
    float yawDeltaDeg = wrapAngle(headingDeg - state.scanLastYawDeg);
    state.scanLastYawDeg = headingDeg;
    float directedProgressDeg = yawDeltaDeg * state.scanDirection;
    if (fabs(yawDeltaDeg) <= PLANNER_EMERGENCY_SCAN_STEP_DEG) {
      state.scanAccumulatedDeg = max(
        0.0f, state.scanAccumulatedDeg + directedProgressDeg);
    }
    float remainingDeg = state.scanTargetAccumulatedDeg -
                         state.scanAccumulatedDeg;
    if (remainingDeg <= TURN_TOLERANCE_DEG) {
      motorStopRequested = true;
      requestMotionStop();
      state.phase = EMERGENCY_RECOVERY_SCAN_DWELL;
      state.phaseStartedMs = now;
      recordEmergencySensorBaselines();
      plannerTelemetry.planReason = "emergency_scan_dwell";
      return;
    }
    if (now - state.phaseStartedMs >
        PLANNER_EMERGENCY_SECTOR_TIMEOUT_MS) {
      abortEmergencyRecovery("emergency_scan_sector_timeout");
      return;
    }

    PlannerCollisionSnapshot snapshot;
    capturePlannerCollisionSnapshot(snapshot);
    if (!turnSegmentFootprintClear(
          snapshot, robotX, robotY, headingDeg,
          state.scanDirection, remainingDeg)) {
      enterEmergencyScanUnwind("scan_segment_map_blocked");
      return;
    }

    float turnTarget = state.scanDirection *
      PLANNER_TURN_SLOW_TARGET_SPEED;
    SafePivotStepResult pivotResult = commandSafePivotStep(
      turnTarget,
      "emergency_scan_turn",
      "emergency_scan_side_revalidating",
      "emergency_scan_sweep_revalidating");
    if (pivotResult == SAFE_PIVOT_STEP_SWEEP_BLOCKED) {
      enterEmergencyScanUnwind("scan_live_sweep_blocked");
    } else if (pivotResult == SAFE_PIVOT_STEP_SIDE_INVALID ||
               pivotResult == SAFE_PIVOT_STEP_SWEEP_INVALID) {
      abortEmergencyRecovery("emergency_scan_sensor_invalid");
    } else if (pivotResult == SAFE_PIVOT_STEP_STUCK) {
      abortEmergencyRecovery("emergency_scan_turn_stalled");
    } else if (pivotResult ==
               SAFE_PIVOT_STEP_PUBLICATION_VETOED) {
      abortEmergencyRecovery("emergency_scan_motion_vetoed");
    }
    return;
  }

  if (state.phase == EMERGENCY_RECOVERY_SCAN_UNWIND) {
    if (!emergencySensorFramesCurrent()) {
      abortEmergencyRecovery("emergency_unwind_sensor_unhealthy");
      return;
    }
    float headingDeg = navigationHeadingDeg();
    float yawDeltaDeg = wrapAngle(headingDeg - state.scanLastYawDeg);
    state.scanLastYawDeg = headingDeg;
    float directedProgressDeg = yawDeltaDeg * state.scanDirection;
    if (fabs(yawDeltaDeg) <= PLANNER_EMERGENCY_SCAN_STEP_DEG) {
      state.scanAccumulatedDeg = max(
        0.0f, state.scanAccumulatedDeg + directedProgressDeg);
    }
    if (state.scanAccumulatedDeg <= TURN_TOLERANCE_DEG) {
      if (!emergencyRecoverySensorsHealthy()) {
        abortEmergencyRecovery("emergency_unwind_rear_blocked");
        return;
      }
      enterEmergencyRelocation("partial_scan_unwound");
      return;
    }
    if (now - state.phaseStartedMs >
        PLANNER_EMERGENCY_SECTOR_TIMEOUT_MS *
          PLANNER_EMERGENCY_SCAN_SECTORS) {
      abortEmergencyRecovery("emergency_scan_unwind_timeout");
      return;
    }

    PlannerCollisionSnapshot snapshot;
    capturePlannerCollisionSnapshot(snapshot);
    if (!turnSegmentFootprintClear(
          snapshot, robotX, robotY, headingDeg,
          -state.scanDirection, state.scanAccumulatedDeg)) {
      abortEmergencyRecovery("emergency_scan_unwind_map_blocked");
      return;
    }
    float turnTarget = -state.scanDirection *
      PLANNER_TURN_SLOW_TARGET_SPEED;
    SafePivotStepResult pivotResult = commandSafePivotStep(
      turnTarget,
      "emergency_scan_unwind",
      "emergency_unwind_side_revalidating",
      "emergency_unwind_sweep_revalidating");
    if (pivotResult == SAFE_PIVOT_STEP_SIDE_INVALID ||
        pivotResult == SAFE_PIVOT_STEP_SWEEP_INVALID) {
      abortEmergencyRecovery("emergency_scan_unwind_sensor_invalid");
    } else if (pivotResult == SAFE_PIVOT_STEP_SWEEP_BLOCKED) {
      abortEmergencyRecovery("emergency_scan_unwind_sweep_blocked");
    } else if (pivotResult == SAFE_PIVOT_STEP_STUCK) {
      abortEmergencyRecovery("emergency_scan_unwind_stalled");
    } else if (pivotResult ==
               SAFE_PIVOT_STEP_PUBLICATION_VETOED) {
      abortEmergencyRecovery("emergency_scan_unwind_motion_vetoed");
    }
    return;
  }

  if (state.phase == EMERGENCY_RECOVERY_SCAN_DWELL) {
    motorStopRequested = true;
    requestMotionStop();
    if (emergencySensorsAdvancedSinceBaseline()) {
      state.scanSector++;
      char detail[48];
      snprintf(detail, sizeof(detail), "sector=%u;yaw=%.1f",
               state.scanSector, state.scanAccumulatedDeg);
      sendBluetoothEvent("emergency_scan_sector", detail);
      if (state.scanSector >= PLANNER_EMERGENCY_SCAN_SECTORS) {
        state.phase = EMERGENCY_RECOVERY_RESET_RETRY;
        state.phaseStartedMs = now;
        completeEmergencyScanAndPrepareRetry();
        return;
      }
      state.scanTargetAccumulatedDeg =
        (state.scanSector + 1) * PLANNER_EMERGENCY_SCAN_STEP_DEG;
      state.phase = EMERGENCY_RECOVERY_SCAN_TURN;
      state.phaseStartedMs = now;
      turnSideInvalidSinceMs = 0;
      turnSweepInvalidSinceMs = 0;
      resetTurnStuckCheck(navigationHeadingDeg());
      plannerTelemetry.planReason = "emergency_scan_turn";
      return;
    }
    if (now - state.phaseStartedMs >
        PLANNER_EMERGENCY_SENSOR_WAIT_MS) {
      abortEmergencyRecovery("emergency_scan_dwell_sensor_timeout");
    }
    return;
  }

  if (state.phase == EMERGENCY_RECOVERY_RELOCATE) {
    if (!emergencyRecoverySensorsHealthy()) {
      abortEmergencyRecovery("emergency_relocation_sensor_unhealthy");
      return;
    }
    if (state.relocationDistanceM >
        PLANNER_EMERGENCY_MAX_RELOCATION_M) {
      abortEmergencyRecovery("emergency_relocation_distance_exhausted");
      return;
    }
    if (state.relocationStartedMs != 0 &&
        now - state.relocationStartedMs >
          PLANNER_EMERGENCY_RELOCATE_TIMEOUT_MS) {
      abortEmergencyRecovery("emergency_relocation_timeout");
      return;
    }
    float checkpointDistanceM = hypotf(
      robotX - state.relocationCheckpointX,
      robotY - state.relocationCheckpointY);
    if (checkpointDistanceM >=
        PLANNER_EMERGENCY_RECHECK_DISTANCE_M) {
      state.relocationCheckpointX = robotX;
      state.relocationCheckpointY = robotY;
      sendBluetoothEvent("emergency_relocation_checkpoint",
                         "stationary_reobservation");
      enterEmergencySensorSettle(
        EMERGENCY_RECOVERY_SETTLE_RELOCATED,
        "emergency_relocation_settle");
      return;
    }

    float goalDx = navigationGoal.targetX - robotX;
    float goalDy = navigationGoal.targetY - robotY;
    float goalDistanceM = hypotf(goalDx, goalDy);
    float lookaheadM = min(goalDistanceM, WAYPOINT_LOOKAHEAD_M);
    float recoveryGoalX = goalDistanceM > 0.001f
      ? robotX + goalDx / goalDistanceM * lookaheadM : robotX;
    float recoveryGoalY = goalDistanceM > 0.001f
      ? robotY + goalDy / goalDistanceM * lookaheadM : robotY;
    TrajectoryPlanResult reverseResult =
      selectReverseRecoveryTrajectory(recoveryGoalX, recoveryGoalY);
    if (reverseResult == TRAJECTORY_PLAN_NO_PATH) {
      abortEmergencyRecovery("emergency_relocation_no_safe_arc");
    } else if (reverseResult == TRAJECTORY_PLAN_SUCCESS) {
      plannerTelemetry.planReason = "emergency_relocation_arc";
      plannerTelemetry.replanReason = "seek_scan_clearance";
    }
    return;
  }

  if (state.phase == EMERGENCY_RECOVERY_SETTLE_RELOCATED) {
    motorStopRequested = true;
    requestMotionStop();
    if (emergencySensorsAdvancedSinceBaseline()) {
      PlannerCollisionSnapshot snapshot;
      capturePlannerCollisionSnapshot(snapshot);
      float rotationalClearanceM = rotationalEnvelopeClearanceM(
        snapshot, robotX, robotY);
      state.bestRotationalClearanceM = max(
        state.bestRotationalClearanceM, rotationalClearanceM);
      if (rotationalClearanceM >=
          PLANNER_EMERGENCY_TARGET_CLEARANCE_M) {
        startEmergencyScanAtCurrentPose();
      } else {
        enterEmergencyRelocation("scan_pose_clearance_insufficient");
      }
      return;
    }
    if (now - state.phaseStartedMs >
        PLANNER_EMERGENCY_SENSOR_WAIT_MS) {
      abortEmergencyRecovery("emergency_relocation_sensor_timeout");
    }
  }
}


static bool canStartSafeReverse() {
  return navigationGoal.mode == NAV_GOAL_POINT &&
         escapeBacktrackEnabled &&
         hasTrustedRearCoverage() &&
         isRangeSensorCurrent(RANGE_FAKE_REAR) &&
         !isRangeSensorBlocked(RANGE_FAKE_REAR);
}

static bool canStartEvidenceDrivenReverse() {
  return canStartSafeReverse() &&
         geometricNoPathEpochCount >=
           PLANNER_REVERSE_MIN_GEOMETRIC_NO_PATH_EPOCHS &&
         noSafeTrajectorySinceMs != 0 &&
         millis() - noSafeTrajectorySinceMs >=
           PLANNER_NO_PATH_BACKTRACK_DELAY_MS;
}

static void startEvidenceDrivenReverse(const char* trigger) {
  resetPlannerEpoch();
  resetReversePlannerEpoch();
  reverseRecoveryState = {};
  reverseRecoveryState.active = true;
  reverseRecoveryState.startX = robotX;
  reverseRecoveryState.startY = robotY;
  reverseRecoveryActive = true;
  reverseRecoveryStartedMs = millis();
  reverseRecoveryStepCount = 0;
  recoveryBudget.forwardTakeoverPending = false;
  recoveryBudget.lastReverseX = robotX;
  recoveryBudget.lastReverseY = robotY;
  if (recoveryBudget.attemptCount < 255) {
    recoveryBudget.attemptCount++;
  }
  obstacleContext.progressGoalValid = false;
  plannerTelemetry.recoveryCount = recoveryBudget.attemptCount;
  plannerTelemetry.reverseRecoveryActive = true;
  plannerTelemetry.recoveryPlateauCount = 0;
  plannerTelemetry.planReason = "reverse_reposition_start";
  plannerTelemetry.replanReason = trigger;
  plannerTelemetry.safeStopReason = "";
  sendBluetoothEvent("reverse_recovery_start", trigger);
}

static void completeEvidenceDrivenReverse() {
  resetReversePlannerEpoch();
  reverseRecoveryState = {};
  reverseRecoveryActive = false;
  recoveryBudget.forwardTakeoverPending = true;
  recoveryBudget.takeoverStartGoalDistanceM = hypotf(
    navigationGoal.targetX - robotX,
    navigationGoal.targetY - robotY);
  float routeLengthM;
  float routeUx;
  float routeUy;
  float routeHeadingRad;
  recoveryBudget.takeoverStartRouteAlongM =
    routeLineFrame(routeLengthM, routeUx, routeUy, routeHeadingRad)
      ? routeLineAlongM(robotX, robotY, routeUx, routeUy)
      : 0.0f;
  obstacleContext.progressGoalValid = false;
  plannerTelemetry.reverseRecoveryActive = false;
  plannerTelemetry.obstacleProgressAgeS = 0.0f;
  plannerTelemetry.recoveryPlateauCount = 0;
  resetGeometricNoPathEvidence();
  sendBluetoothEvent("reverse_recovery_end", "forward_takeover");
}

static void updatePointGoal() {
  float dx = navigationGoal.targetX - robotX;
  float dy = navigationGoal.targetY - robotY;
  float distanceM = sqrtf(dx * dx + dy * dy);
  float arrivalToleranceM = ownerIsObjectHunt(navigationGoal.owner)
                              ? PLANNER_HUNT_FINISH_TARGET_TOLERANCE_M
                              : WAYPOINT_TOLERANCE_M;
  if (distanceM <= arrivalToleranceM) {
    finishNavigationGoal(true, PLANNER_STOP_NONE, "waypoint_reached");
    return;
  }

  float routeLengthM = 0.0f;
  float routeUx = 1.0f;
  float routeUy = 0.0f;
  float routeHeadingRad = 0.0f;
  bool routeFrameValid =
    routeLineFrame(routeLengthM, routeUx, routeUy, routeHeadingRad);
  bool routeLineEligible =
    routeFrameValid &&
    fabs(wrapAngle(routeHeadingRad * RAD_TO_DEG - navigationHeadingDeg())) <=
      PLANNER_LINE_FOLLOW_ENABLE_HEADING_DEG;

  if (routeFrameValid && huntPickupZoneReached(routeLengthM, routeUx, routeUy)) {
    finishNavigationGoal(true, PLANNER_STOP_NONE, "hunt_pickup_zone_reached");
    return;
  }

  if (emergencyRecoveryState.phase != EMERGENCY_RECOVERY_IDLE &&
      emergencyRecoveryState.phase !=
        EMERGENCY_RECOVERY_RETRY_ACTIVE) {
    updateEmergencyRecovery();
    return;
  }

  if (reverseRecoveryState.active) {
    float lookaheadM = min(distanceM, WAYPOINT_LOOKAHEAD_M);
    float recoveryGoalX = robotX + (dx / distanceM) * lookaheadM;
    float recoveryGoalY = robotY + (dy / distanceM) * lookaheadM;
    if (updateReverseRecoveryBudget()) {
      return;
    }

    float reverseSegmentDistanceM = sqrtf(
      (robotX - reverseRecoveryState.startX) *
        (robotX - reverseRecoveryState.startX) +
      (robotY - reverseRecoveryState.startY) *
        (robotY - reverseRecoveryState.startY));
    if (!reverseRecoveryState.checkingForward &&
        reverseSegmentDistanceM >=
          PLANNER_REVERSE_FORWARD_RECHECK_DISTANCE_M) {
      stopMotors();
      reverseRecoveryState.checkingForward = true;
      reverseRecoveryState.forwardCheckAfterMovement = true;
      resetReversePlannerEpoch();
      plannerTelemetry.planReason = "reverse_distance_checkpoint";
      plannerTelemetry.replanReason = "forward_takeover_check";
      return;
    }

    if (reverseRecoveryState.checkingForward) {
      bool takeoverAvoidance = updateObstacleContext(
        navigationGoal.targetX, navigationGoal.targetY);
      if (takeoverAvoidance) {
        buildObstacleLocalGoal(recoveryGoalX, recoveryGoalY);
      }
      TrajectoryPlanResult forwardResult =
        selectTrajectory(recoveryGoalX, recoveryGoalY);
      if (forwardResult == TRAJECTORY_PLAN_PENDING ||
          forwardResult == TRAJECTORY_PLAN_ABORTED ||
          forwardResult == TRAJECTORY_PLAN_RETRY) {
        return;
      }
      if (forwardResult == TRAJECTORY_PLAN_SUCCESS) {
        completeEvidenceDrivenReverse();
        return;
      }
      if (!reverseRecoveryState.forwardCheckAfterMovement) {
        handleRecoveryExhaustion(
          PLANNER_STOP_RECOVERY_NO_USEFUL_OUTCOME,
          "reverse_corridor_without_forward_path");
        return;
      }
      stopMotors();
      reverseRecoveryState.checkingForward = false;
      reverseRecoveryState.forwardCheckAfterMovement = false;
      reverseRecoveryState.startX = robotX;
      reverseRecoveryState.startY = robotY;
      reverseRecoveryState.plateauCount = 0;
      resetPlannerEpoch();
      resetReversePlannerEpoch();
      plannerTelemetry.planReason = "reverse_checkpoint_continuing";
      plannerTelemetry.replanReason = "forward_takeover_not_ready";
      return;
    }

    TrajectoryPlanResult reverseResult =
      selectReverseRecoveryTrajectory(recoveryGoalX, recoveryGoalY);
    if (reverseResult == TRAJECTORY_PLAN_PENDING ||
        reverseResult == TRAJECTORY_PLAN_SUCCESS ||
        reverseResult == TRAJECTORY_PLAN_RETRY ||
        reverseResult == TRAJECTORY_PLAN_ABORTED) {
      return;
    }
    // Exhausting the remembered reverse corridor may mean the repositioning
    // succeeded. Stop and give the normal forward planner one fresh,
    // revalidated takeover attempt before declaring recovery impossible.
    stopMotors();
    reverseRecoveryState.checkingForward = true;
    reverseRecoveryState.forwardCheckAfterMovement = false;
    resetReversePlannerEpoch();
    plannerTelemetry.planReason = "reverse_corridor_exhausted";
    plannerTelemetry.replanReason = "forward_takeover_check";
    return;
  }

  bool huntCarryThroughActive =
    routeLineEligible && huntPickupCarryThroughActive(routeLengthM, routeUx, routeUy);
  bool avoidanceActive = updateObstacleContext(navigationGoal.targetX,
                                               navigationGoal.targetY);
  float targetHeadingDeg = atan2f(dy, dx) * RAD_TO_DEG;
  float targetHeadingErrorDeg = wrapAngle(targetHeadingDeg - navigationHeadingDeg());
  if (!plannerEpoch.active && !avoidanceActive &&
      !huntCarryThroughActive &&
      distanceM > PLANNER_FINAL_BLOCKED_ACCEPTANCE_M &&
      fabs(targetHeadingErrorDeg) > PLANNER_POINT_ALIGN_START_DEG) {
    commandPointAlignmentTurn(targetHeadingErrorDeg);
    return;
  }
  pointAlignTurnActive = false;
  pointAlignTurnDirection = 0.0;

  float localGoalX = 0.0f;
  float localGoalY = 0.0f;
  if (avoidanceActive) {
    buildObstacleLocalGoal(localGoalX, localGoalY);
    plannerTelemetry.planReason = "obstacle_local_bypass";
  } else {
    float lookaheadM = min(distanceM, WAYPOINT_LOOKAHEAD_M);
    localGoalX = robotX + (dx / distanceM) * lookaheadM;
    localGoalY = robotY + (dy / distanceM) * lookaheadM;
    plannerTelemetry.planReason = "direct_waypoint";
  }

  float localGoalDistanceM = sqrtf(
    (localGoalX - robotX) * (localGoalX - robotX) +
    (localGoalY - robotY) * (localGoalY - robotY));
  float routeAlongM = routeFrameValid
    ? routeLineAlongM(robotX, robotY, routeUx, routeUy) : 0.0f;
  plannerTelemetry.globalGoalDistanceM = distanceM;
  plannerTelemetry.localGoalDistanceM = localGoalDistanceM;
  plannerTelemetry.routeAlongProgressM = routeAlongM;
  plannerTelemetry.routeSignedLateralErrorM = routeFrameValid
    ? routeLineSignedLateralErrorM(robotX, robotY, routeUx, routeUy) : 0.0f;
  plannerTelemetry.cumulativeReverseDistanceM =
    recoveryBudget.cumulativeReverseDistanceM;
  plannerTelemetry.recoveryCount = recoveryBudget.attemptCount;

  bool madeNetTakeoverProgress =
    distanceM <= recoveryBudget.takeoverStartGoalDistanceM -
                   PLANNER_RECOVERY_TAKEOVER_PROGRESS_M ||
    (routeFrameValid &&
     routeAlongM >= recoveryBudget.takeoverStartRouteAlongM +
                      PLANNER_RECOVERY_TAKEOVER_PROGRESS_M);
  if (recoveryBudget.forwardTakeoverPending && !avoidanceActive &&
      madeNetTakeoverProgress) {
    resetRecoveryBudget();
  }
  if (avoidanceActive &&
      obstacleProgressStalled(localGoalX, localGoalY, localGoalDistanceM)) {
    motorStopRequested = true;
    requestMotionStop();
    if (canStartSafeReverse()) {
      startEvidenceDrivenReverse("obstacle_local_goal_stalled");
    } else {
      handleRecoveryExhaustion(
        PLANNER_STOP_RECOVERY_NO_PROGRESS,
        "obstacle_local_goal_progress_stalled");
    }
    return;
  }

  if (!avoidanceActive && routeLineEligible &&
      routeLineGoalReached(routeLengthM, routeUx, routeUy, routeHeadingRad)) {
    finishNavigationGoal(true, PLANNER_STOP_NONE, "route_line_reached");
    return;
  }
  if (!avoidanceActive && routeLineEligible &&
      routeLineOvershootStopReached(routeLengthM, routeUx, routeUy, routeHeadingRad)) {
    finishNavigationGoal(true, PLANNER_STOP_NONE, "route_line_overshoot_reached");
    return;
  }
  if (!avoidanceActive && routeLineEligible &&
      routeLineClearlyMissed(routeLengthM, routeUx, routeUy)) {
    finishNavigationGoal(false, PLANNER_STOP_ABORTED, "route_line_missed");
    return;
  }

  if (driveStuck || wheelMismatchStuck) {
    plannerTelemetry.replanReason = "wheel_progress_stalled";
    finishNavigationGoal(false, PLANNER_STOP_STUCK, "drive_progress_stalled");
    return;
  }

  TrajectoryPlanResult trajectoryResult =
    selectTrajectory(localGoalX, localGoalY);
  if (trajectoryResult == TRAJECTORY_PLAN_PENDING ||
      trajectoryResult == TRAJECTORY_PLAN_ABORTED) {
    return;
  }
  if (trajectoryResult == TRAJECTORY_PLAN_RETRY) {
    resetGeometricNoPathEvidence();
    return;
  }
  if (trajectoryResult == TRAJECTORY_PLAN_SUCCESS) {
    resetGeometricNoPathEvidence();
    return;
  }

  motorStopRequested = true;
  requestMotionStop();
  reportPlannerStopIfChanged();
  if (!currentPlannerFailureIsGeometricNoPath()) {
    resetGeometricNoPathEvidence();
    return;
  }
  if (!avoidanceActive &&
      distanceM <= PLANNER_FINAL_BLOCKED_ACCEPTANCE_M) {
    PlannerCollisionSnapshot currentCollision;
    capturePlannerCollisionSnapshot(currentCollision);
    if (footprintClearOnSnapshot(currentCollision, robotX, robotY,
                                 navigationHeadingRad())) {
      // The rejected rollout may extend toward an obstacle beyond the point,
      // but no motion is needed from this already-safe bounded final pose.
      finishNavigationGoal(true, PLANNER_STOP_NONE, "final_blocked_reached");
      return;
    }
  }
  noteGeometricNoPathEpoch();
  if (canStartEvidenceDrivenReverse()) {
    startEvidenceDrivenReverse("persistent_geometric_no_path");
    return;
  }
  if (millis() - noSafeTrajectorySinceMs >= PLANNER_NO_PATH_ABORT_MS) {
    handleRecoveryExhaustion(
      PLANNER_STOP_NO_SAFE_TRAJECTORY,
      avoidanceActive
        ? "obstacle_has_no_safe_forward_bypass"
        : "no_safe_forward_trajectory");
  }
}

static void updateTurnGoal() {
  // Turn control is intentionally simpler than point planning: command a
  // calibrated direction, observe yaw, slow inside 30 degrees, then brake.
  if (turnBrakeActive) {
    // Motors are briefly commanded opposite the last turn direction to cancel
    // measured coast. The pulse is safety-gated by the same sweep checks used
    // for the normal pivot.
    if (millis() < turnBrakeUntilMs) {
      float brakeTarget = -turnLastCommandDirection * PLANNER_TURN_SLOW_TARGET_SPEED;
      plannerTelemetry.selectedForwardTicksPerSec = 0.0;
      plannerTelemetry.selectedTurnTicksPerSec = brakeTarget;
      plannerTelemetry.selectedCurvature = 0.0;
      plannerTelemetry.minimumSweptClearanceMm = minimumFanSweepClearanceMm();
      plannerTelemetry.speedCapTicksPerSec = 0.0;
      plannerTelemetry.candidateCount = 1;
      plannerTelemetry.stopReason = PLANNER_STOP_NONE;
      plannerTelemetry.planReason = "calibrated_turn_brake";
      motorStopRequested = false;
      publishNavigationMotion(0.0, brakeTarget);
      return;
    }

    turnBrakeActive = false;
    sendBluetoothEvent("turn_brake_end", "pulse_complete");
    finishNavigationGoal(true, PLANNER_STOP_NONE, "turn_reached");
    return;
  }

  // wrapAngle gives the shortest signed route to target heading in [-180, +180].
  float error = wrapAngle(navigationGoal.targetYawDeg - navigationHeadingDeg());
  if (fabs(error) <= TURN_TOLERANCE_DEG) {
    float brakeTarget = -turnLastCommandDirection * PLANNER_TURN_SLOW_TARGET_SPEED;
    if (isTurnDirectionObservable(brakeTarget) && isTurnSweepSafe()) {
      unsigned long brakePulseMs = turnLastCommandDirection > 0.0
        ? PLANNER_TURN_LEFT_BRAKE_PULSE_MS
        : PLANNER_TURN_RIGHT_BRAKE_PULSE_MS;
      turnBrakeActive = true;
      turnBrakeUntilMs = millis() + brakePulseMs;
      motorStopRequested = false;
      publishNavigationMotion(0.0, brakeTarget);
      sendBluetoothEvent("turn_brake_start", "calibrated_counterturn");
      return;
    }
    finishNavigationGoal(true, PLANNER_STOP_NONE, "turn_reached_brake_unavailable");
    return;
  }

  bool slowTurn = fabs(error) < SLOW_ZONE_DEG;
  turnLastCommandDirection = error > 0.0 ? 1.0 : -1.0;
  float turnTarget = (error > 0.0 ? 1.0 : -1.0) *
                     (slowTurn ? PLANNER_TURN_SLOW_TARGET_SPEED : PLANNER_TURN_TARGET_SPEED);
  SafePivotStepResult pivotResult = commandSafePivotStep(
    turnTarget,
    slowTurn ? "calibrated_turn_slow" : "calibrated_turn_fast",
    "turn_side_revalidating",
    "turn_sweep_revalidating");
  if (pivotResult == SAFE_PIVOT_STEP_SIDE_INVALID) {
    finishNavigationGoal(false, PLANNER_STOP_TURN_SIDE_INVALID,
                         "turn_side_sensor_invalid");
  } else if (pivotResult == SAFE_PIVOT_STEP_SWEEP_INVALID) {
    finishNavigationGoal(false, PLANNER_STOP_TURN_CLEARANCE,
                         "turn_sweep_sensor_invalid");
  } else if (pivotResult == SAFE_PIVOT_STEP_SWEEP_BLOCKED) {
    finishNavigationGoal(false, PLANNER_STOP_TURN_CLEARANCE,
                         "turn_sweep_not_clear");
  } else if (pivotResult == SAFE_PIVOT_STEP_STUCK) {
    finishNavigationGoal(false, PLANNER_STOP_STUCK,
                         "turn_progress_stalled");
  }
}

void updateNavigationController() {
  if (!navigationGoal.active) {
    return;
  }
  if (navigationGoal.authority == MOTION_AUTHORITY_NONE ||
      navigationGoal.authority != motionAuthority) {
    cancelNavigationGoal(PLANNER_STOP_ABORTED, "motion_authority_revoked");
    return;
  }

  unsigned long now = millis();
  // Map-based arc selection runs at 40 ms, but a pivot turn is a direct
  // yaw feedback task.  Run it alongside the 20 ms motor/sensor schedule so
  // it does not spend another 80 ms driving after crossing its tolerance.
  // Arc rollout is comparatively expensive and can run at 40 ms. A turn is a
  // simple yaw threshold problem, so it is sampled at the 20 ms motor cadence
  // to reduce overshoot at the tolerance boundary.
  unsigned long updateIntervalMs = navigationGoal.mode == NAV_GOAL_TURN
    ? MOTOR_CONTROL_INTERVAL_MS
    : PLANNER_UPDATE_INTERVAL_MS;
  bool plannerEpochPending =
    navigationGoal.mode == NAV_GOAL_POINT &&
    (plannerEpoch.active || reversePlannerEpoch.active);
  if (!plannerEpochPending && now - lastPlannerUpdateMs < updateIntervalMs) {
    return;
  }
  if (!plannerEpochPending) {
    lastPlannerUpdateMs = now;
    if (navigationGoal.mode == NAV_GOAL_TURN) {
      plannerTelemetry.lastPlanMs = now;
    }
  }
  if (navigationGoal.mode == NAV_GOAL_POINT) {
    updatePointGoal();
  } else if (navigationGoal.mode == NAV_GOAL_TURN) {
    updateTurnGoal();
  }
}

void initializeNavigationController() {
  // Called once after yaw/pose initialisation. From this point forward the map
  // moves with the robot rather than being re-created every route waypoint.
  clearLocalMap();
  resetPlannerEpoch();
  resetReversePlannerEpoch();
  lastPlannerCommandPublishedMs = 0;
  resetEncodersAndPID();
  resetObstacleContext("controller_initialised");
  plannerTelemetry.planReason = "initialised";
}

void updateRobotController() {
  // This is the timing backbone of autonomous motion. Its order is deliberate:
  // sense/map first, take immediate safety action second, update odometry,
  // select a new command if safe, then let the sole motor writer apply it.
  unsigned long now = millis();
  bool immediateSafetyStop = false;
  unsigned long phaseStartedUs = 0;
  if (now - lastSensorUpdateMs >= SENSOR_UPDATE_INTERVAL_MS) {
    lastSensorUpdateMs = now;
    phaseStartedUs = micros();
    updateTOFSensors();
    recordMainLoopPhaseDuration("fan_tof", phaseStartedUs);
    phaseStartedUs = micros();
    updateLocalMapFromSensors();
    recordMainLoopPhaseDuration("local_map", phaseStartedUs);

    if (navigationGoal.active && isTofCloseReadingRevalidating()) {
      // A sudden close return is plausible collision evidence but also a known
      // ToF/I2C failure mode. Pause for one confirmation sample; do not map it
      // as a wall unless the sensor repeats it.
      immediateSafetyStop = true;
      motorStopRequested = true;
      requestMotionStop();
      plannerTelemetry.stopReason = PLANNER_STOP_NONE;
      plannerTelemetry.safeStopReason = "tof_close_revalidating";
      plannerTelemetry.replanReason = "tof_close_revalidating";
    }

    if (!immediateSafetyStop && navigationGoal.active &&
        isRangeSensorBlocked(RANGE_FRONT)) {
      immediateSafetyStop = true;
      motorStopRequested = true;
      requestMotionStop();
      plannerTelemetry.stopReason = PLANNER_STOP_FRONT_BLOCKED;
      plannerTelemetry.safeStopReason = "front_blocked";
      plannerTelemetry.replanReason = "front_blocked";
      reportPlannerStopIfChanged();
      if (navigationGoal.mode == NAV_GOAL_POINT) {
        // Keep the immediate neutral command, then allow the point planner to
        // publish only if a fresh forward arc passes the same sensor/map veto.
        immediateSafetyStop = false;
      } else if (navigationGoal.mode == NAV_GOAL_TURN) {
        cancelNavigationGoal(PLANNER_STOP_FRONT_BLOCKED, "front_blocked_during_turn");
      }
    }

    RangeSensorId diagonalSensor;
    float diagonalClearanceMm = 0.0;
    if (!immediateSafetyStop && navigationGoal.active &&
        getDiagonalClearanceWarning(diagonalSensor, diagonalClearanceMm)) {
      // This is the fast, current-pose guard. It uses chassis-footprint
      // clearance rather than turn radius, so it can stop an imminent clip
      // without wrongly forbidding a pre-aligned narrow straight passage.
      immediateSafetyStop = true;
      motorStopRequested = true;
      requestMotionStop();
      plannerTelemetry.stopReason = PLANNER_STOP_NO_SAFE_TRAJECTORY;
      plannerTelemetry.safeStopReason = "diagonal_clearance";
      plannerTelemetry.replanReason = "diagonal_clearance";
      reportPlannerStopIfChanged();
      if (navigationGoal.mode == NAV_GOAL_TURN) {
        cancelNavigationGoal(PLANNER_STOP_TURN_CLEARANCE, "diagonal_clearance_during_turn");
      }
    }
  }

  if (now - lastOdometryUpdateMs >= ODOMETRY_UPDATE_INTERVAL_MS) {
    lastOdometryUpdateMs = now;
    phaseStartedUs = micros();
    updateOdometry();
    recordMainLoopPhaseDuration("odometry_imu", phaseStartedUs);
    // Update pose before marking the footprint; otherwise the free patch would
    // lag behind the physical chassis by one odometry period.
    markTraversedFreeSpace();
  }

  if (!immediateSafetyStop) {
    // An immediate safety stop owns this cycle. The planner must not overwrite
    // it with a fresh arc until another controller pass has seen new evidence.
    phaseStartedUs = micros();
    updateNavigationController();
    recordMainLoopPhaseDuration("planner", phaseStartedUs);
  }
  // No other file writes servo pulses during normal navigation.
  phaseStartedUs = micros();
  updateMotorController();
  recordMainLoopPhaseDuration("motor_writer", phaseStartedUs);
  phaseStartedUs = micros();
  sendBluetoothTelemetry();
  recordMainLoopPhaseDuration("telemetry_build", phaseStartedUs);
}
