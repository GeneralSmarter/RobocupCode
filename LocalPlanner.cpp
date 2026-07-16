#include "Robot.h"

// =====================================================
// Local confidence map and receding-horizon navigation
// =====================================================
// Responsibility:
//   Owns local perception-to-motion planning: short-lived occupancy evidence,
//   footprint collision checks, differential-drive arc rollout/scoring, point
//   and turn navigation goals, obstacle-local forward bypass, planner telemetry,
//   and the scheduled controller pipeline.
// Interacts with:
//   TofSensors.cpp supplies fan/fake-rear range evidence. Odometry.cpp updates
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
// The map is intentionally local.  It gives the planner enough memory to
// avoid immediately re-entering a wall after a turn without pretending that
// odometry alone can support a permanent arena map.

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
static PlannerStopReason lastReportedStopReason = PLANNER_STOP_NONE;
static int recentBlockedTurnDirection = 0;
static unsigned long recentBlockedTurnMs = 0;
static bool turnBrakeActive = false;
static unsigned long turnBrakeUntilMs = 0;
static float turnLastCommandDirection = 0.0;
static bool pointAlignTurnActive = false;
static float pointAlignTurnDirection = 0.0;
static unsigned long turnSideInvalidSinceMs = 0;
static unsigned long turnSweepInvalidSinceMs = 0;
// Reverse recovery is an in-goal recovery mode. It keeps the original point
// target, tries normal forward planning first on every planner tick, and uses
// RANGE_FAKE_REAR to sample trusted reverse arcs only while no forward path is
// available.
enum ObstacleBypassPhase {
  BYPASS_IDLE,
  BYPASS_SIDE_ESCAPE,
  BYPASS_FRONT_WALL_REVERSE,
  BYPASS_POST_REVERSE_ESCAPE,
  BYPASS_ROUTE_REJOIN,
  BYPASS_FINAL_APPROACH
};

static bool reverseRecoveryActive = false;
static float reverseRecoveryStartX = 0.0;
static float reverseRecoveryStartY = 0.0;
static unsigned long reverseRecoveryStartedMs = 0;
static unsigned long reverseRecoveryStepCount = 0;
static bool reverseSurveySettling = false;
static unsigned long reverseSurveySettleStartedMs = 0;
static bool reverseSurveyReadyReported = false;
static bool reverseSurveyReadyToTryForward = false;
static bool reverseSurveyForcedByMaxRetreat = false;
static float reverseSurveyRequiredRetreatM = PLANNER_REVERSE_SURVEY_MIN_REVERSE_M;
static unsigned long noSafeTrajectorySinceMs = 0;
static uint8_t geometricNoPathEpochCount = 0;
static bool lastForwardNoPathWasGeometric = false;
static bool candidateRejectsReported = false;
static bool reverseRecoveryRejectsReported = false;
static ObstacleBypassPhase obstacleBypassPhase = BYPASS_IDLE;
// Sensor geometry is observational and may change on every refresh.  The
// active side is a manoeuvre contract: once non-zero it is changed only by a
// stopped reset followed by a new, stable observation.
static float obstacleObservedPreferredSideSign = 0.0f;
static float obstacleObservedCandidateSideSign = 0.0f;
static uint8_t obstacleObservedStableCount = 0;
static unsigned long obstacleObservedLastUpdateMs = 0;
static float obstacleBypassSideSign = 0.0f;
static float obstacleBypassMaxOutwardM = 0.0f;
static float obstacleBypassTargetOutwardM = 0.0f;
static unsigned long obstacleBypassPhaseStartedMs = 0;
static bool recoveryLivenessActive = false;
static float recoveryLivenessStartX = 0.0f;
static float recoveryLivenessStartY = 0.0f;
static float recoveryLivenessLastX = 0.0f;
static float recoveryLivenessLastY = 0.0f;
static float recoveryInitialGoalDistanceM = 0.0f;
static float recoveryBestGoalDistanceM = 0.0f;
static float recoveryStartRouteAlongM = 0.0f;
static float recoveryBestRouteAlongM = 0.0f;
static float recoveryProgressCheckpointGoalDistanceM = 0.0f;
static float recoveryProgressCheckpointRouteAlongM = 0.0f;
static float recoveryCumulativeDistanceM = 0.0f;
static float recoveryBestProgressM = 0.0f;
static uint8_t recoveryAttemptCount = 0;
static unsigned long recoveryLastProgressMs = 0;
static bool clearanceEscapeLocalGoalActive = false;
static bool clearanceEscapeRouteDetourActive = false;
static bool postReverseEscapeActive = false;
static float postReverseEscapeSideSign = 0.0f;
static unsigned long postReverseEscapeStartedMs = 0;
static float lastClearanceEscapeSideSign = 0.0f;
static unsigned long lastClearanceEscapeMs = 0;
static float sideEscapeAdaptiveExtraM = 0.0f;
static float sideEscapeAdaptiveSideSign = 0.0f;
static uint8_t sideEscapeAdaptiveFailureCount = 0;
static unsigned long sideEscapeAdaptiveLastBumpMs = 0;
static float lastFootprintRejectWorldX = 0.0f;
static float lastFootprintRejectWorldY = 0.0f;
static int lastFootprintRejectCellX = -1;
static int lastFootprintRejectCellY = -1;
static float lastCorridorRejectLeftM = -1.0f;
static float lastCorridorRejectRightM = -1.0f;

struct ObstacleContext {
  bool active;
  float originX;
  float originY;
  float routeUx;
  float routeUy;
  float routeLengthM;
  float nearAlongM;
  float farAlongM;
  float minLateralM;
  float maxLateralM;
  float sideSign;
  unsigned long startedMs;
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

struct SideEscapeEvidence {
  bool sensorsValid;
  bool closeSideEvidence;
  bool sidePreferenceValid;
  float rightM;
  float leftM;
  float restrictedM;
  float triggerM;
  float sideSign;
};

static bool readSideEscapeEvidence(SideEscapeEvidence &evidence);
static void bumpSideEscapeAdaptiveExtra(float sideSign, const char* reason);
static const char* obstacleBypassPhaseName(ObstacleBypassPhase phase);
static void setObstacleBypassPhase(ObstacleBypassPhase phase, const char* reason);
static bool routeLineFrame(float &routeLengthM, float &routeUx,
                           float &routeUy, float &routeHeadingRad);
static float routeLineSignedLateralErrorM(float worldX, float worldY,
                                          float routeUx, float routeUy);
static void resetObstacleContext(const char* reason);

enum ReverseSurveyDecision {
  REVERSE_SURVEY_REVERSE,
  REVERSE_SURVEY_HOLD,
  REVERSE_SURVEY_FORWARD
};

enum CandidateRejectReason {
  CANDIDATE_REJECT_NONE,
  CANDIDATE_REJECT_TURN_OBSERVABILITY,
  CANDIDATE_REJECT_FORWARD_OBSERVATION,
  CANDIDATE_REJECT_REAR_OBSERVATION,
  CANDIDATE_REJECT_FOOTPRINT,
  CANDIDATE_REJECT_CORRIDOR
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
};

struct PlannerEpoch {
  bool active;
  bool awaitingRevalidation;
  bool commandStoppedForAge;
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
  float bestClearance;
  bool bestReachesGoal;
  float bestArrivalTimeS;
  PlannerCollisionSnapshot collision;
};

struct ReversePlannerEpoch {
  bool active;
  bool awaitingRevalidation;
  bool commandStoppedForAge;
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
  float bestScore;
  float bestReverse;
  float bestTurn;
  float bestRearClearance;
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

static void markEndpointEvidence(RangeSensorId id, float worldX, float worldY,
                                 float rayHeadingRad) {
  // A ToF reading says "an object lies somewhere near this ray endpoint".
  // It does NOT justify filling a large circular obstacle around it. The
  // uncertainty box below is aligned to the beam: range uncertainty is along
  // the beam and beam/cone uncertainty is across it.
  bool outer = isOuterFanSensor(id);
  float backUncertaintyM = outer ? MAP_OUTER_ENDPOINT_BACK_UNCERTAINTY_M
                                 : MAP_INNER_ENDPOINT_BACK_UNCERTAINTY_M;
  float forwardUncertaintyM = outer ? MAP_OUTER_ENDPOINT_FORWARD_UNCERTAINTY_M
                                    : MAP_INNER_ENDPOINT_FORWARD_UNCERTAINTY_M;
  float lateralUncertaintyM = outer ? MAP_OUTER_ENDPOINT_LATERAL_UNCERTAINTY_M
                                    : MAP_INNER_ENDPOINT_LATERAL_UNCERTAINTY_M;
  int dynamicEvidence = outer ? MAP_OUTER_ENDPOINT_DYNAMIC_EVIDENCE
                              : MAP_INNER_ENDPOINT_DYNAMIC_EVIDENCE;
  int staticEvidence = outer ? MAP_OUTER_ENDPOINT_STATIC_EVIDENCE
                             : MAP_INNER_ENDPOINT_STATIC_EVIDENCE;
  // Outer rays are useful for route choice, but are more oblique and less
  // reliable for declaring a hard forward obstruction. Their lower evidence
  // therefore needs repeated observations before cellOccupied() rejects them.
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
        cell.lastObservedMs = millis();
      }
    }
  }
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

void clearLocalMap() {
  // Used by ZERO and initialisation. It intentionally does not reset pose;
  // callers decide whether the robot's coordinate system should also reset.
  initialiseMapAtRobot();
  plannerTelemetry.replanReason = "map_cleared";
}

void updateLocalMapFromSensors() {
  // This is perception, not planning. It converts the latest four ranges into
  // short-lived world evidence before the planner asks whether arcs are safe.
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

    for (float distance = 0.0; distance <= freeLengthM; distance += LOCAL_MAP_CELL_M * 0.5) {
      float centreX = sensorWorldX + cosf(rayHeading) * distance;
      float centreY = sensorWorldY + sinf(rayHeading) * distance;
      // Only a narrow strip around the beam is marked free. Treating the full
      // ToF cone as free would invent visibility where an edge return could
      // hide a wall or another obstacle.
      for (float lateral = -MAP_FREE_RAY_HALF_WIDTH_M;
           lateral <= MAP_FREE_RAY_HALF_WIDTH_M;
           lateral += LOCAL_MAP_CELL_M) {
        float freeX = centreX - sinf(rayHeading) * lateral;
        float freeY = centreY + cosf(rayHeading) * lateral;
        addFreeEvidence(freeX, freeY, 8);
      }
    }

    float endpointX = sensorWorldX + cosf(rayHeading) * rangeM;
    float endpointY = sensorWorldY + sinf(rayHeading) * rangeM;
    // The endpoint becomes obstacle evidence after the free ray is drawn. The
    // ordering matters: the ray must not erase the object it just measured.
    markEndpointEvidence(id, endpointX, endpointY, rayHeading);
  }

  if (isRangeSensorValid(RANGE_FAKE_REAR)) {
    float sensorWorldX;
    float sensorWorldY;
    transformRobotPoint(FAKE_REAR_TOF_GEOMETRY.xMm / 1000.0f,
                        FAKE_REAR_TOF_GEOMETRY.yMm / 1000.0f,
                        headingRad, sensorWorldX, sensorWorldY);

    float rayHeading = headingRad + FAKE_REAR_TOF_GEOMETRY.angleDeg * DEG_TO_RAD;
    float rangeM = getRangeSensorDistance(RANGE_FAKE_REAR) / 1000.0f;
    float rearHalfWidthM = max(MAP_FREE_RAY_HALF_WIDTH_M,
      max(ROBOT_FOOTPRINT_GEOMETRY.leftExtentMm,
          ROBOT_FOOTPRINT_GEOMETRY.rightExtentMm) / 1000.0f +
      PLANNER_TOTAL_HARD_CLEARANCE_M);

    float freeLengthM = min(rangeM, LOCAL_MAP_SIZE_M * 0.5f);
    for (float distance = 0.0f; distance <= freeLengthM; distance += LOCAL_MAP_CELL_M * 0.5f) {
      float centreX = sensorWorldX + cosf(rayHeading) * distance;
      float centreY = sensorWorldY + sinf(rayHeading) * distance;
      for (float lateral = -rearHalfWidthM;
           lateral <= rearHalfWidthM;
           lateral += LOCAL_MAP_CELL_M) {
        float freeX = centreX - sinf(rayHeading) * lateral;
        float freeY = centreY + cosf(rayHeading) * lateral;
        addFreeEvidence(freeX, freeY, 10);
      }
    }

    if (getRangeSensorDistance(RANGE_FAKE_REAR) < FAKE_REAR_TOF_DISTANCE_MM) {
      float endpointX = sensorWorldX + cosf(rayHeading) * rangeM;
      float endpointY = sensorWorldY + sinf(rayHeading) * rangeM;
      markEndpointEvidence(RANGE_FAKE_REAR, endpointX, endpointY, rayHeading);
    }
  }
}

void markTraversedFreeSpace() {
  // The robot's own recent footprint is stronger evidence than a distant ToF
  // ray. Sample a 3x3 set over the chassis rather than a single centre point
  // so the map remembers the whole swept rectangle as recently traversed.
  recenterLocalMapIfNeeded();
  const float headingRad = navigationHeadingRad();
  const float sampleX[] = {
    -ROBOT_FOOTPRINT_GEOMETRY.rearExtentMm / 1000.0f,
    0.0,
    ROBOT_FOOTPRINT_GEOMETRY.frontExtentMm / 1000.0f
  };
  const float sampleY[] = {
    -ROBOT_FOOTPRINT_GEOMETRY.rightExtentMm / 1000.0f,
    0.0,
    ROBOT_FOOTPRINT_GEOMETRY.leftExtentMm / 1000.0f
  };

  for (unsigned int ix = 0; ix < sizeof(sampleX) / sizeof(sampleX[0]); ix++) {
    for (unsigned int iy = 0; iy < sizeof(sampleY) / sizeof(sampleY[0]); iy++) {
      float worldX;
      float worldY;
      transformRobotPoint(sampleX[ix], sampleY[iy], headingRad, worldX, worldY);
      int x;
      int y;
      if (worldToCell(worldX, worldY, x, y)) {
        LocalMapCell &cell = localMap[y][x];
        cell.traversedEvidence = 255;
        cell.freeEvidence = 100;
        cell.dynamicEvidence = 0;
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
  return cell.staticEvidence >= PLANNER_OBSTACLE_SCORE_THRESHOLD ||
         cell.dynamicEvidence >= PLANNER_OBSTACLE_SCORE_THRESHOLD;
}

static bool worldOccupied(float worldX, float worldY) {
  int x;
  int y;
  if (!worldToCell(worldX, worldY, x, y)) {
    return true;
  }
  return cellOccupied(x, y);
}

static void capturePlannerCollisionSnapshot(PlannerCollisionSnapshot &snapshot) {
  snapshot.originX = localMapOriginX;
  snapshot.originY = localMapOriginY;
  memset(snapshot.occupied, 0, sizeof(snapshot.occupied));
  for (int y = 0; y < LOCAL_MAP_CELLS; y++) {
    for (int x = 0; x < LOCAL_MAP_CELLS; x++) {
      if (cellOccupied(x, y)) {
        int bit = y * LOCAL_MAP_CELLS + x;
        snapshot.occupied[bit >> 3] |= (uint8_t)(1U << (bit & 7));
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

static bool footprintClearOnSnapshot(const PlannerCollisionSnapshot &snapshot,
                                     float worldX, float worldY,
                                     float headingRad) {
  // Tests one predicted robot pose against an immutable occupancy snapshot.
  // This lets a planner epoch evaluate many candidates without the map
  // changing halfway through the comparison.
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

  // Preserve the original complete 50 mm lattice while replacing repeated
  // map-object lookups with one immutable occupancy bitset lookup.
  for (float localX = -rear; localX <= front + 0.001f;
       localX += LOCAL_MAP_CELL_M) {
    float pointX = worldX + localX * headingCos + right * headingSin;
    float pointY = worldY + localX * headingSin - right * headingCos;
    const float stepX = -LOCAL_MAP_CELL_M * headingSin;
    const float stepY = LOCAL_MAP_CELL_M * headingCos;
    for (float localY = -right; localY <= left + 0.001f;
         localY += LOCAL_MAP_CELL_M) {
      int cellX;
      int cellY;
      if (snapshotWorldOccupied(snapshot, pointX, pointY, &cellX, &cellY)) {
        lastFootprintRejectWorldX = pointX;
        lastFootprintRejectWorldY = pointY;
        lastFootprintRejectCellX = cellX;
        lastFootprintRejectCellY = cellY;
        return false;
      }
      pointX += stepX;
      pointY += stepY;
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

static bool footprintClear(float worldX, float worldY, float headingRad) {
  // This is the planner's hard collision test. The preferred 50 mm clearance
  // is deliberately NOT added here; it influences scoring instead so a tight
  // but physically plausible 400 mm straight passage can remain selectable.
  const float front = ROBOT_FOOTPRINT_GEOMETRY.frontExtentMm / 1000.0 +
                      PLANNER_TOTAL_HARD_CLEARANCE_M;
  const float rear = ROBOT_FOOTPRINT_GEOMETRY.rearExtentMm / 1000.0 +
                     PLANNER_TOTAL_HARD_CLEARANCE_M;
  const float left = ROBOT_FOOTPRINT_GEOMETRY.leftExtentMm / 1000.0 +
                     PLANNER_TOTAL_HARD_CLEARANCE_M;
  const float right = ROBOT_FOOTPRINT_GEOMETRY.rightExtentMm / 1000.0 +
                      PLANNER_TOTAL_HARD_CLEARANCE_M;
  // Every point belongs to the same predicted pose. Hoisting this basis out
  // of the nested sample loops removes thousands of redundant sin/cos calls
  // per planning cycle without changing a single collision sample.
  const float headingCos = cosf(headingRad);
  const float headingSin = sinf(headingRad);

  // Sample every map cell across the inflated rectangle, not merely the
  // corners.  A narrow panel therefore cannot slip between sparse footprint
  // sample points during a curved rollout.
  for (float localX = -rear; localX <= front + 0.001; localX += LOCAL_MAP_CELL_M) {
    for (float localY = -right; localY <= left + 0.001; localY += LOCAL_MAP_CELL_M) {
      float pointX = worldX + localX * headingCos - localY * headingSin;
      float pointY = worldY + localX * headingSin + localY * headingCos;
      if (worldOccupied(pointX, pointY)) {
        lastFootprintRejectWorldX = pointX;
        lastFootprintRejectWorldY = pointY;
        worldToCell(pointX, pointY, lastFootprintRejectCellX, lastFootprintRejectCellY);
        return false;
      }
    }
  }
  return true;
}

static void resetObstacleContext(const char* reason) {
  bool wasActive = obstacleContext.active;
  obstacleContext = {};
  obstacleBypassSideSign = 0.0f;
  obstacleBypassPhase = BYPASS_IDLE;
  obstacleBypassPhaseStartedMs = 0;
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

  // Grow the connected observed envelope in route coordinates. This derives
  // the bypass from the obstacle evidence itself instead of a fixed travel
  // distance, while tolerating the small gaps produced by separate ToF rays.
  const float joinM = LOCAL_MAP_CELL_M * 1.75f;
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
        if (alongM < envelope.nearAlongM - joinM ||
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
  return true;
}

static float chooseObstacleSide(const ObstacleEnvelope &envelope,
                                float routeUx, float routeUy) {
  float lateralClearanceM =
    max(ROBOT_FOOTPRINT_GEOMETRY.leftExtentMm,
        ROBOT_FOOTPRINT_GEOMETRY.rightExtentMm) / 1000.0f +
    PLANNER_TOTAL_HARD_CLEARANCE_M + LOCAL_MAP_CELL_M * 0.5f;
  float leftTargetM = envelope.maxLateralM + lateralClearanceM;
  float rightTargetM = envelope.minLateralM - lateralClearanceM;
  float dx = robotX - obstacleContext.originX;
  float dy = robotY - obstacleContext.originY;
  float currentLateralM = -dx * routeUy + dy * routeUx;
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
    obstacleContext.nearAlongM = envelope.nearAlongM;
    obstacleContext.farAlongM = envelope.farAlongM;
    obstacleContext.minLateralM = envelope.minLateralM;
    obstacleContext.maxLateralM = envelope.maxLateralM;
    obstacleContext.sideSign = chooseObstacleSide(envelope, routeUx, routeUy);
    obstacleContext.startedMs = millis();
    obstacleContext.clearSinceMs = 0;
    obstacleBypassSideSign = obstacleContext.sideSign;
    obstacleBypassPhase = BYPASS_SIDE_ESCAPE;
    obstacleBypassPhaseStartedMs = millis();
    plannerTelemetry.replanReason = "obstacle_context_started";
    sendBluetoothEvent("obstacle_context_start",
                       obstacleContext.sideSign > 0.0f ? "left" : "right");
    resetPlannerEpoch();
    return true;
  }

  ObstacleEnvelope observed;
  if (scanRouteObstacle(obstacleContext.originX,
                        obstacleContext.originY,
                        obstacleContext.routeUx,
                        obstacleContext.routeUy,
                        obstacleContext.routeLengthM,
                        observed)) {
    const float joinM = LOCAL_MAP_CELL_M * 2.0f;
    bool sameObstacle = observed.nearAlongM <= obstacleContext.farAlongM + joinM &&
                        observed.farAlongM >= obstacleContext.nearAlongM - joinM;
    if (sameObstacle) {
      obstacleContext.nearAlongM = min(obstacleContext.nearAlongM,
                                       observed.nearAlongM);
      obstacleContext.farAlongM = max(obstacleContext.farAlongM,
                                      observed.farAlongM);
      obstacleContext.minLateralM = min(obstacleContext.minLateralM,
                                        observed.minLateralM);
      obstacleContext.maxLateralM = max(obstacleContext.maxLateralM,
                                        observed.maxLateralM);
    }
  }

  float poseDx = robotX - obstacleContext.originX;
  float poseDy = robotY - obstacleContext.originY;
  float alongM = poseDx * obstacleContext.routeUx +
                 poseDy * obstacleContext.routeUy;
  float rearClearM = ROBOT_FOOTPRINT_GEOMETRY.rearExtentMm / 1000.0f +
                     PLANNER_TOTAL_HARD_CLEARANCE_M +
                     LOCAL_MAP_CELL_M * 0.5f;
  if (alongM >= obstacleContext.farAlongM + rearClearM) {
    if (obstacleContext.clearSinceMs == 0) {
      obstacleContext.clearSinceMs = millis();
    }
    if (millis() - obstacleContext.clearSinceMs >= 80) {
      resetObstacleContext("obstacle_cleared");
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
  float lateralClearanceM =
    max(ROBOT_FOOTPRINT_GEOMETRY.leftExtentMm,
        ROBOT_FOOTPRINT_GEOMETRY.rightExtentMm) / 1000.0f +
    PLANNER_TOTAL_HARD_CLEARANCE_M + LOCAL_MAP_CELL_M * 0.5f;
  float rearClearM = ROBOT_FOOTPRINT_GEOMETRY.rearExtentMm / 1000.0f +
                     PLANNER_TOTAL_HARD_CLEARANCE_M +
                     LOCAL_MAP_CELL_M * 0.5f;
  float targetLateralM = obstacleContext.sideSign > 0.0f
    ? obstacleContext.maxLateralM + lateralClearanceM
    : obstacleContext.minLateralM - lateralClearanceM;
  if (obstacleContext.sideSign > 0.0f) {
    targetLateralM = max(targetLateralM, lateralClearanceM);
  } else {
    targetLateralM = min(targetLateralM, -lateralClearanceM);
  }
  float poseDx = robotX - obstacleContext.originX;
  float poseDy = robotY - obstacleContext.originY;
  float currentAlongM = poseDx * obstacleContext.routeUx +
                        poseDy * obstacleContext.routeUy;
  float currentLateralM = -poseDx * obstacleContext.routeUy +
                          poseDy * obstacleContext.routeUx;
  bool laterallyClear = obstacleContext.sideSign * currentLateralM >=
    obstacleContext.sideSign * targetLateralM -
      PLANNER_OBSTACLE_COUNTERSTEER_LEAD_M;
  float targetAlongM;
  if (laterallyClear || currentAlongM >= obstacleContext.nearAlongM) {
    targetAlongM = obstacleContext.farAlongM + rearClearM;
  } else {
    float frontClearM = ROBOT_FOOTPRINT_GEOMETRY.frontExtentMm / 1000.0f +
                        PLANNER_TOTAL_HARD_CLEARANCE_M +
                        LOCAL_MAP_CELL_M * 0.5f;
    targetAlongM = max(0.05f, obstacleContext.nearAlongM - frontClearM -
                              PLANNER_OBSTACLE_TURN_ROOM_M);
  }
  localGoalX = obstacleContext.originX +
               obstacleContext.routeUx * targetAlongM -
               obstacleContext.routeUy * targetLateralM;
  localGoalY = obstacleContext.originY +
               obstacleContext.routeUy * targetAlongM +
               obstacleContext.routeUx * targetLateralM;
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
  resetGeometricNoPathEvidence();
  reverseRecoveryRejectsReported = false;
  resetObstacleContext(success ? "goal_complete" : "goal_abort");
  motorStopRequested = true;
  setMotionCommand(0.0, 0.0);
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
  setMotionCommand(0.0, 0.0);
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
  plannerTelemetry.recoveryPhaseElapsedS = 0.0;
  plannerTelemetry.cumulativeRecoveryDistanceM = 0.0;
  plannerTelemetry.recoveryBestProgressM = 0.0;
  plannerTelemetry.recoveryCount = 0;
  plannerTelemetry.candidateCount = 0;
  plannerTelemetry.stopReason = PLANNER_STOP_NONE;
  plannerTelemetry.planReason = "goal_started";
  lastReportedStopReason = PLANNER_STOP_NONE;
  recentBlockedTurnDirection = 0;
  recentBlockedTurnMs = 0;
  reverseRecoveryActive = false;
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
  resetGeometricNoPathEvidence();
  reverseRecoveryRejectsReported = false;
  motorStopRequested = true;
  setMotionCommand(0.0, 0.0);
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
    setMotionCommand(0.0, 0.0);
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

#if 0
static float clearanceEscapeUrgency() {
  if (!clearanceEscapeLocalGoalActive) {
    return 0.0f;
  }

  float urgency = 0.0f;
  if (isRangeSensorValid(RANGE_FRONT)) {
    float frontM = getRangeSensorDistance(RANGE_FRONT) / 1000.0f;
    float spanM = max(0.001f, PLANNER_ESCAPE_SPEED_LIMIT_START_M -
                                PLANNER_ESCAPE_SPEED_LIMIT_FULL_M);
    urgency = max(urgency,
                  constrain((PLANNER_ESCAPE_SPEED_LIMIT_START_M - frontM) / spanM,
                            0.0f, 1.0f));
  }

  SideEscapeEvidence evidence;
  if (readSideEscapeEvidence(evidence) && evidence.closeSideEvidence) {
    float sideSpanM = max(0.001f, evidence.triggerM - PLANNER_PREFERRED_CLEARANCE_M);
    urgency = max(urgency,
                  constrain((evidence.triggerM - evidence.restrictedM) / sideSpanM,
                            0.0f, 1.0f));
  }

  if (postReverseEscapeActive) {
    urgency = max(urgency, 0.85f);
  }

  return urgency;
}

static float applyClearanceEscapeSpeedCap(float speedCap) {
  float urgency = clearanceEscapeUrgency();
  if (urgency <= 0.0f) {
    return speedCap;
  }

  float escapeCap = speedCap -
    (speedCap - PLANNER_ESCAPE_MIN_SPEED_TPS) * urgency;
  return max(PLANNER_MIN_DRIVABLE_SPEED_TPS, min(speedCap, escapeCap));
}
#endif

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

  for (float elapsed = 0.0; elapsed < PLANNER_HORIZON_S; elapsed += PLANNER_ROLLOUT_STEP_S) {
    // Keep the start of this integration segment so terminal scoring can
    // measure the closest *continuous* approach to a point goal.  Comparing
    // only the 100 ms sample endpoints can favour a slower command simply
    // because one of its samples happens to land nearer the goal.
    float segmentStartX = x;
    float segmentStartY = y;
    float segmentStartHeading = heading;
    float midpointHeading = heading + angularRadPerSec * PLANNER_ROLLOUT_STEP_S * 0.5;
    x += linearMps * cosf(midpointHeading) * PLANNER_ROLLOUT_STEP_S;
    y += linearMps * sinf(midpointHeading) * PLANNER_ROLLOUT_STEP_S;
    heading += angularRadPerSec * PLANNER_ROLLOUT_STEP_S;
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
          arrivalTimeS = elapsed + entryProgress * PLANNER_ROLLOUT_STEP_S;
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
            return true;
          }
        }
      }
    }
    float travelledM = linearMps * (elapsed + PLANNER_ROLLOUT_STEP_S);
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
  }
  finalX = x;
  finalY = y;
  finalHeadingRad = heading;
  return true;
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
  float headingScore = 1.0 - min(1.0f, headingError / 90.0);
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

static void resetRecoveryLivenessState() {
  recoveryLivenessActive = false;
  recoveryLivenessStartX = robotX;
  recoveryLivenessStartY = robotY;
  recoveryLivenessLastX = robotX;
  recoveryLivenessLastY = robotY;
  recoveryInitialGoalDistanceM = 0.0f;
  recoveryBestGoalDistanceM = 0.0f;
  recoveryStartRouteAlongM = 0.0f;
  recoveryBestRouteAlongM = 0.0f;
  recoveryProgressCheckpointGoalDistanceM = 0.0f;
  recoveryProgressCheckpointRouteAlongM = 0.0f;
  recoveryCumulativeDistanceM = 0.0f;
  recoveryBestProgressM = 0.0f;
  recoveryAttemptCount = 0;
  recoveryLastProgressMs = 0;
}

static bool abortRecoveryLiveness(PlannerStopReason reason,
                                  const char* detail) {
  // finishNavigationGoal() neutralizes the command before publishing the
  // typed abort.  This monitor never supplies a success result.
  plannerTelemetry.replanReason = detail;
  finishNavigationGoal(false, reason, detail);
  return true;
}

static bool updateRecoveryLiveness(float globalGoalDistanceM,
                                   float localGoalDistanceM,
                                   bool routeFrameValid,
                                   float routeUx, float routeUy) {
  float routeAlongM = routeFrameValid
    ? routeLineAlongM(robotX, robotY, routeUx, routeUy) : 0.0f;
  float signedLateralM = routeFrameValid
    ? routeLineSignedLateralErrorM(robotX, robotY, routeUx, routeUy) : 0.0f;
  unsigned long now = millis();

  plannerTelemetry.globalGoalDistanceM = globalGoalDistanceM;
  plannerTelemetry.localGoalDistanceM = localGoalDistanceM;
  plannerTelemetry.routeAlongProgressM = routeAlongM;
  plannerTelemetry.routeSignedLateralErrorM = signedLateralM;

  bool mustSupervise = obstacleBypassPhase != BYPASS_IDLE ||
                       clearanceEscapeLocalGoalActive ||
                       postReverseEscapeActive || reverseRecoveryActive;
  if (!mustSupervise) {
    return false;
  }

  if (!recoveryLivenessActive) {
    recoveryLivenessActive = true;
    recoveryLivenessStartX = robotX;
    recoveryLivenessStartY = robotY;
    recoveryLivenessLastX = robotX;
    recoveryLivenessLastY = robotY;
    recoveryInitialGoalDistanceM = globalGoalDistanceM;
    recoveryBestGoalDistanceM = globalGoalDistanceM;
    recoveryStartRouteAlongM = routeAlongM;
    recoveryBestRouteAlongM = routeAlongM;
    recoveryProgressCheckpointGoalDistanceM = globalGoalDistanceM;
    recoveryProgressCheckpointRouteAlongM = routeAlongM;
    recoveryLastProgressMs = now;
    if (obstacleBypassPhaseStartedMs == 0) {
      obstacleBypassPhaseStartedMs = now;
    }
    sendBluetoothEvent("recovery_liveness_start", "original_goal_supervision");
  }

  recoveryCumulativeDistanceM +=
    sqrtf((robotX - recoveryLivenessLastX) * (robotX - recoveryLivenessLastX) +
          (robotY - recoveryLivenessLastY) * (robotY - recoveryLivenessLastY));
  recoveryLivenessLastX = robotX;
  recoveryLivenessLastY = robotY;
  recoveryBestGoalDistanceM = min(recoveryBestGoalDistanceM, globalGoalDistanceM);
  recoveryBestRouteAlongM = max(recoveryBestRouteAlongM, routeAlongM);
  recoveryBestProgressM = max(recoveryInitialGoalDistanceM - recoveryBestGoalDistanceM,
                              recoveryBestRouteAlongM - recoveryStartRouteAlongM);

  if (globalGoalDistanceM <= recoveryProgressCheckpointGoalDistanceM -
                               PLANNER_RECOVERY_PROGRESS_EPSILON_M ||
      routeAlongM >= recoveryProgressCheckpointRouteAlongM +
                       PLANNER_RECOVERY_PROGRESS_EPSILON_M) {
    recoveryProgressCheckpointGoalDistanceM =
      min(recoveryProgressCheckpointGoalDistanceM, globalGoalDistanceM);
    recoveryProgressCheckpointRouteAlongM =
      max(recoveryProgressCheckpointRouteAlongM, routeAlongM);
    recoveryLastProgressMs = now;
  }

  plannerTelemetry.recoveryPhaseElapsedS =
    obstacleBypassPhaseStartedMs == 0
      ? 0.0f : (now - obstacleBypassPhaseStartedMs) / 1000.0f;
  plannerTelemetry.cumulativeRecoveryDistanceM = recoveryCumulativeDistanceM;
  plannerTelemetry.recoveryBestProgressM = recoveryBestProgressM;
  plannerTelemetry.recoveryCount = recoveryAttemptCount;

  if (globalGoalDistanceM - recoveryBestGoalDistanceM >
      PLANNER_RECOVERY_MAX_GOAL_DIVERGENCE_M) {
    return abortRecoveryLiveness(PLANNER_STOP_RECOVERY_DIVERGENCE,
                                 "recovery_goal_divergence");
  }
  if (fabs(signedLateralM) >
      PLANNER_RECOVERY_MAX_LATERAL_DISPLACEMENT_M) {
    return abortRecoveryLiveness(PLANNER_STOP_RECOVERY_DISPLACEMENT,
                                 "recovery_lateral_displacement");
  }
  if (obstacleBypassPhaseStartedMs != 0 &&
      now - obstacleBypassPhaseStartedMs >
        PLANNER_RECOVERY_MAX_PHASE_TIME_MS) {
    return abortRecoveryLiveness(PLANNER_STOP_RECOVERY_TIME,
                                 "recovery_phase_time_exhausted");
  }
  if (recoveryCumulativeDistanceM >
      PLANNER_RECOVERY_MAX_CUMULATIVE_DISTANCE_M) {
    return abortRecoveryLiveness(PLANNER_STOP_RECOVERY_DISTANCE,
                                 "recovery_distance_exhausted");
  }
  if (recoveryAttemptCount > PLANNER_RECOVERY_MAX_COUNT) {
    return abortRecoveryLiveness(PLANNER_STOP_RECOVERY_REPEATED,
                                 "recovery_count_exhausted");
  }
  if (recoveryLastProgressMs != 0 &&
      now - recoveryLastProgressMs >
        PLANNER_RECOVERY_NO_PROGRESS_TIMEOUT_MS) {
    return abortRecoveryLiveness(PLANNER_STOP_RECOVERY_NO_PROGRESS,
                                 "recovery_progress_stalled");
  }
  return false;
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

#if 0
static bool sideEscapeRejoinActive(float routeLengthM, float routeUx, float routeUy) {
  if (!clearanceEscapeRouteDetourActive || routeLengthM <= 0.0f) {
    return false;
  }

  float alongM = routeLineAlongM(robotX, robotY, routeUx, routeUy);
  if (obstacleBypassSideSign != 0.0f &&
      obstacleBypassTargetOutwardM >= PLANNER_FRONT_WALL_BYPASS_OUTWARD_M - 0.001f) {
    float outwardM = obstacleBypassSideSign *
      routeLineSignedLateralErrorM(robotX, robotY, routeUx, routeUy);
    if (outwardM < obstacleBypassTargetOutwardM) {
      return true;
    }
  }
  float finalApproachM = PLANNER_SIDE_ESCAPE_FINAL_APPROACH_M;
  if (obstacleBypassTargetOutwardM >= PLANNER_FRONT_WALL_BYPASS_OUTWARD_M - 0.001f) {
    finalApproachM = min(finalApproachM, 0.10f);
  }
  return alongM < routeLengthM - finalApproachM;
}

static void clearSideEscapeDetourIfActive(const char* reason) {
  if (!clearanceEscapeRouteDetourActive) {
    return;
  }
  setObstacleBypassPhase(BYPASS_FINAL_APPROACH, reason);
  clearanceEscapeRouteDetourActive = false;
  sendBluetoothEvent("side_escape_final_approach", reason);
}
#endif

static bool huntPickupCarryThroughActive(float routeLengthM, float routeUx,
                                         float routeUy) {
  if (!ownerIsObjectHunt(navigationGoal.owner)) {
    return false;
  }

  float alongM = routeLineAlongM(robotX, robotY, routeUx, routeUy);
  float lateralErrorM = routeLineLateralErrorM(robotX, robotY, routeUx, routeUy);
  return alongM >= routeLengthM - PLANNER_HUNT_PICKUP_BOOST_ZONE_M &&
         alongM <= routeLengthM + PLANNER_HUNT_FINISH_OVERSHOOT_M &&
         lateralErrorM <= PLANNER_HUNT_FINISH_LATERAL_M;
}

#if 0
static bool obstacleBypassSpeedLimited() {
  return clearanceEscapeLocalGoalActive ||
         postReverseEscapeActive ||
         obstacleBypassPhase == BYPASS_SIDE_ESCAPE ||
         obstacleBypassPhase == BYPASS_FRONT_WALL_REVERSE;
}
#endif

static float requestedPointGoalSpeedCap(bool routeLineActive,
                                        float routeLengthM, float routeUx,
                                        float routeUy) {
  float requestedCap = baseTargetSpeed;
  if (obstacleContext.active) {
    requestedCap = min(requestedCap, PLANNER_OBSTACLE_MAX_SPEED_TPS);
  }
  if (routeLineActive && huntPickupCarryThroughActive(routeLengthM, routeUx, routeUy)) {
    requestedCap = min(PLANNER_HUNT_PICKUP_MAX_SPEED_TPS,
                       baseTargetSpeed + PLANNER_HUNT_PICKUP_BOOST_TPS);
  }
  return requestedCap;
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

static void buildRouteLineLocalGoal(float routeLengthM, float routeUx,
                                    float routeUy, float &localGoalX,
                                    float &localGoalY) {
  float alongM = routeLineAlongM(robotX, robotY, routeUx, routeUy);
  float lookaheadAlongM = constrain(alongM + WAYPOINT_LOOKAHEAD_M,
                                    0.0f, routeLengthM);
  localGoalX = navigationGoal.startX + routeUx * lookaheadAlongM;
  localGoalY = navigationGoal.startY + routeUy * lookaheadAlongM;
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
    setMotionCommand(0.0, 0.0);
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
    setMotionCommand(0.0, 0.0);
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
  plannerEpoch.bestClearance = -1.0f;
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
  plannerEpoch.requestedSpeedCap = requestedPointGoalSpeedCap(
    plannerEpoch.lineFollowActive, routeLengthM, routeUx, routeUy);
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
    const int totalCandidates =
      PLANNER_SPEED_SAMPLES * PLANNER_CURVATURE_SAMPLES;
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
      float speedScale = speedIndex == 0 ? 1.0f : PLANNER_MIN_SPEED_SCALE;
      float normalized = -1.0f +
        (2.0f * curvatureIndex) / (PLANNER_CURVATURE_SAMPLES - 1);
      float forward = max(PLANNER_MIN_DRIVABLE_SPEED_TPS,
                          plannerEpoch.speedCap * speedScale);
      forward = min(forward, 3000.0f /
        (1.0f + fabs(normalized * PLANNER_MAX_TURN_RATIO)));
      float turn = forward * normalized * PLANNER_MAX_TURN_RATIO;

      if (obstacleContext.active) {
        float signedTurnRatio = turn / max(1.0f, forward);
        float poseDx = plannerEpoch.startX - obstacleContext.originX;
        float poseDy = plannerEpoch.startY - obstacleContext.originY;
        float currentAlongM = poseDx * obstacleContext.routeUx +
                              poseDy * obstacleContext.routeUy;
        float currentLateralM = -poseDx * obstacleContext.routeUy +
                                poseDy * obstacleContext.routeUx;
        float lateralClearanceM =
          max(ROBOT_FOOTPRINT_GEOMETRY.leftExtentMm,
              ROBOT_FOOTPRINT_GEOMETRY.rightExtentMm) / 1000.0f +
          PLANNER_TOTAL_HARD_CLEARANCE_M + LOCAL_MAP_CELL_M * 0.5f;
        float targetLateralM = obstacleContext.sideSign > 0.0f
          ? obstacleContext.maxLateralM + lateralClearanceM
          : obstacleContext.minLateralM - lateralClearanceM;
        if (obstacleContext.sideSign > 0.0f) {
          targetLateralM = max(targetLateralM, lateralClearanceM);
        } else {
          targetLateralM = min(targetLateralM, -lateralClearanceM);
        }
        float routeHeadingDeg = atan2f(obstacleContext.routeUy,
                                       obstacleContext.routeUx) * RAD_TO_DEG;
        float outwardHeadingDeg = obstacleContext.sideSign * wrapAngle(
          plannerEpoch.startHeadingRad * RAD_TO_DEG - routeHeadingDeg);
        bool needsLateralClearance = currentAlongM < obstacleContext.nearAlongM &&
          obstacleContext.sideSign * currentLateralM <
            obstacleContext.sideSign * targetLateralM -
              PLANNER_OBSTACLE_COUNTERSTEER_LEAD_M;
        if (needsLateralClearance && outwardHeadingDeg < 35.0f &&
            obstacleContext.sideSign * signedTurnRatio <
              PLANNER_OBSTACLE_MIN_OUTWARD_TURN_RATIO) {
          continue;
        }
        float desiredHeadingDeg = atan2f(plannerEpoch.goalY - plannerEpoch.startY,
                                         plannerEpoch.goalX - plannerEpoch.startX) *
                                  RAD_TO_DEG;
        float localHeadingErrorDeg = wrapAngle(
          desiredHeadingDeg - plannerEpoch.startHeadingRad * RAD_TO_DEG);
        if (fabs(localHeadingErrorDeg) > 10.0f &&
            (localHeadingErrorDeg > 0.0f ? 1.0f : -1.0f) * signedTurnRatio <
              0.15f) {
          continue;
        }
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

      float clearanceMm = -1.0f;
      float closestGoalDistanceM = plannerEpoch.localGoalDistanceM;
      float headingAtClosestGoalRad = plannerEpoch.startHeadingRad;
      float finalX = plannerEpoch.startX;
      float finalY = plannerEpoch.startY;
      float finalHeadingRad = plannerEpoch.startHeadingRad;
      float arrivalTimeS = -1.0f;
      CandidateRejectReason rejectReason = CANDIDATE_REJECT_NONE;
      if (!rolloutCandidate(plannerEpoch, forward, turn,
                            plannerEpoch.goalX, plannerEpoch.goalY,
                            clearanceMm, closestGoalDistanceM,
                            headingAtClosestGoalRad, finalX, finalY,
                            finalHeadingRad, arrivalTimeS, rejectReason)) {
        if (rejectReason == CANDIDATE_REJECT_TURN_OBSERVABILITY) {
          plannerEpoch.rejectedTurnObservability++;
        } else if (rejectReason == CANDIDATE_REJECT_FORWARD_OBSERVATION) {
          plannerEpoch.rejectedForwardObservation++;
        } else if (rejectReason == CANDIDATE_REJECT_FOOTPRINT) {
          plannerEpoch.rejectedFootprint++;
        } else if (rejectReason == CANDIDATE_REJECT_CORRIDOR) {
          plannerEpoch.rejectedCorridor++;
        }
        continue;
      }

      plannerEpoch.acceptedCount++;
      plannerTelemetry.candidateCount = plannerEpoch.acceptedCount;
      float score = candidateScore(
        forward, turn, plannerEpoch.goalX, plannerEpoch.goalY,
        plannerEpoch.startX, plannerEpoch.startY,
        plannerEpoch.previousSelectedTurn,
        closestGoalDistanceM, headingAtClosestGoalRad,
        finalX, finalY, finalHeadingRad, clearanceMm,
        plannerEpoch.lineFollowActive,
        navigationGoal.startX, navigationGoal.startY,
        plannerEpoch.routeHeadingRad, plannerEpoch.finalGoalDistanceM);
      bool reachesGoal = arrivalTimeS >= 0.0f;
      bool betterArrival =
        plannerEpoch.finalWaypointIsLocalGoal &&
        !plannerEpoch.lineFollowActive && reachesGoal &&
        (!plannerEpoch.bestReachesGoal ||
         arrivalTimeS < plannerEpoch.bestArrivalTimeS - 0.0001f);
      bool equalArrivalClass =
        !plannerEpoch.finalWaypointIsLocalGoal ||
        plannerEpoch.lineFollowActive ||
        (reachesGoal == plannerEpoch.bestReachesGoal &&
         (!reachesGoal ||
          fabs(arrivalTimeS - plannerEpoch.bestArrivalTimeS) <= 0.0001f));
      if (betterArrival || (equalArrivalClass && score > plannerEpoch.bestScore)) {
        plannerEpoch.bestScore = score;
        plannerEpoch.bestForward = forward;
        plannerEpoch.bestTurn = turn;
        plannerEpoch.bestClearance = clearanceMm;
        plannerEpoch.bestReachesGoal = reachesGoal;
        plannerEpoch.bestArrivalTimeS = arrivalTimeS;
      }
    }
    recordPlannerSlice(sliceStartedUs);
    if (plannerEpoch.candidateIndex < totalCandidates) {
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
  float clearanceMm = -1.0f;
  float closestGoalDistanceM = sqrtf(
    (plannerEpoch.goalX - plannerEpoch.startX) *
      (plannerEpoch.goalX - plannerEpoch.startX) +
    (plannerEpoch.goalY - plannerEpoch.startY) *
      (plannerEpoch.goalY - plannerEpoch.startY));
  float headingAtClosestGoalRad = plannerEpoch.startHeadingRad;
  float finalX = plannerEpoch.startX;
  float finalY = plannerEpoch.startY;
  float finalHeadingRad = plannerEpoch.startHeadingRad;
  float arrivalTimeS = -1.0f;
  CandidateRejectReason rejectReason = CANDIDATE_REJECT_NONE;
  bool winnerStillSafe = rolloutCandidate(
    plannerEpoch, publishForward, publishTurn,
    plannerEpoch.goalX, plannerEpoch.goalY, clearanceMm,
    closestGoalDistanceM, headingAtClosestGoalRad,
    finalX, finalY, finalHeadingRad, arrivalTimeS, rejectReason);
  recordPlannerSlice(sliceStartedUs);
  if (!winnerStillSafe) {
    return retryPlannerEpoch(PLANNER_STOP_NO_SAFE_TRAJECTORY,
                             "winner_revalidation_rejected",
                             "winner_revalidation_retry");
  }

  plannerTelemetry.selectedForwardTicksPerSec = publishForward;
  plannerTelemetry.selectedTurnTicksPerSec = publishTurn;
  plannerTelemetry.selectedCurvature = publishTurn /
    max(1.0f, publishForward);
  plannerTelemetry.minimumSweptClearanceMm = clearanceMm;
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

#if 0
// Retained temporarily as a readable comparison with RobotCode - Copy (3).
// The forward-only obstacle-context planner below does not compile or call
// reverse recovery, side-memory latching, adaptive escape, route rejoin, or
// final-approach behavior.
static bool canStartReverseRecovery() {
  return navigationGoal.mode == NAV_GOAL_POINT &&
         escapeBacktrackEnabled &&
         !reverseRecoveryActive &&
         geometricNoPathEpochCount >=
           PLANNER_REVERSE_MIN_GEOMETRIC_NO_PATH_EPOCHS &&
         (obstacleBypassSideSign != 0.0f ||
          obstacleObservedPreferredSideSign != 0.0f) &&
         plannerTelemetry.stopReason != PLANNER_STOP_FRONT_INVALID &&
         isRangeSensorValid(RANGE_FAKE_REAR);
}

static void resetReverseSurveyState() {
  reverseSurveySettling = false;
  reverseSurveySettleStartedMs = 0;
  reverseSurveyReadyReported = false;
  reverseSurveyReadyToTryForward = false;
  reverseSurveyForcedByMaxRetreat = false;
  reverseSurveyRequiredRetreatM = PLANNER_REVERSE_SURVEY_MIN_REVERSE_M;
}

static void startReverseRecovery() {
  // Enters reverse recovery after repeated geometric no-forward-path evidence.
  // It preserves the original point goal and uses reverse planner epochs only
  // until a forward arc becomes available again.
  resetReversePlannerEpoch();
  float adaptiveSideSign = postReverseEscapeActive
    ? postReverseEscapeSideSign
    : lastClearanceEscapeSideSign;
  if (adaptiveSideSign != 0.0f &&
      millis() - lastClearanceEscapeMs <=
        PLANNER_SIDE_ESCAPE_ADAPT_MEMORY_MS) {
    bumpSideEscapeAdaptiveExtra(adaptiveSideSign, "reentered_recovery");
  }

  reverseRecoveryActive = true;
  if (recoveryAttemptCount < 255) {
    recoveryAttemptCount++;
  }
  plannerTelemetry.recoveryCount = recoveryAttemptCount;
  reverseRecoveryStartX = robotX;
  reverseRecoveryStartY = robotY;
  reverseRecoveryStartedMs = millis();
  reverseRecoveryStepCount = 0;
  reverseRecoveryRejectsReported = false;
  resetReverseSurveyState();
  resetGeometricNoPathEvidence();
  if (obstacleBypassSideSign == 0.0f) {
    obstacleBypassSideSign = obstacleObservedPreferredSideSign;
    lastClearanceEscapeSideSign = obstacleBypassSideSign;
    lastClearanceEscapeMs = millis();
    clearanceEscapeRouteDetourActive = true;
  }
  obstacleBypassTargetOutwardM =
    max(obstacleBypassTargetOutwardM, PLANNER_FRONT_WALL_BYPASS_OUTWARD_M);
  setObstacleBypassPhase(BYPASS_FRONT_WALL_REVERSE, "reverse_started");
  resetEncodersAndPID();
  plannerTelemetry.planReason = "reverse_recovery_start";
  plannerTelemetry.replanReason = "no_forward_path";
  plannerTelemetry.safeStopReason = "";
  sendBluetoothEvent("reverse_recovery_start", "no_forward_path");
}

static void endReverseRecovery(const char* detail) {
  if (!reverseRecoveryActive) {
    return;
  }
  reverseRecoveryActive = false;
  resetReversePlannerEpoch();
  reverseRecoveryRejectsReported = false;
  resetReverseSurveyState();
  sendBluetoothEvent("reverse_recovery_end", detail);
}

static float sideEscapeTriggerM() {
  return WAYPOINT_LOOKAHEAD_M +
         ROBOT_FOOTPRINT_GEOMETRY.frontExtentMm / 1000.0f +
         PLANNER_PREFERRED_CLEARANCE_M;
}

static const char* sideEscapeName(float sideSign) {
  if (sideSign < 0.0f) {
    return "right";
  }
  if (sideSign > 0.0f) {
    return "left";
  }
  return "unknown";
}

static const char* obstacleBypassPhaseName(ObstacleBypassPhase phase) {
  switch (phase) {
    case BYPASS_IDLE: return "idle";
    case BYPASS_SIDE_ESCAPE: return "side_escape";
    case BYPASS_FRONT_WALL_REVERSE: return "front_wall_reverse";
    case BYPASS_POST_REVERSE_ESCAPE: return "post_reverse_escape";
    case BYPASS_ROUTE_REJOIN: return "route_rejoin";
    case BYPASS_FINAL_APPROACH: return "final_approach";
  }
  return "unknown";
}

static void setObstacleBypassPhase(ObstacleBypassPhase phase, const char* reason) {
  if (obstacleBypassPhase == phase) {
    return;
  }

  char detail[128];
  snprintf(detail, sizeof(detail),
           "from=%s;to=%s;reason=%s;observed=%.0f;latched=%.0f;out=%.2f;target=%.2f",
           obstacleBypassPhaseName(obstacleBypassPhase),
           obstacleBypassPhaseName(phase),
           reason,
           obstacleObservedPreferredSideSign,
           obstacleBypassSideSign,
           obstacleBypassMaxOutwardM,
           obstacleBypassTargetOutwardM);
  obstacleBypassPhase = phase;
  obstacleBypassPhaseStartedMs = millis();
  sendBluetoothEvent("obstacle_bypass_phase", detail);
}

static void resetPostReverseEscapeState() {
  postReverseEscapeActive = false;
  postReverseEscapeSideSign = 0.0f;
  postReverseEscapeStartedMs = 0;
}

static void resetSideEscapeAdaptation() {
  sideEscapeAdaptiveExtraM = 0.0f;
  sideEscapeAdaptiveSideSign = 0.0f;
  sideEscapeAdaptiveFailureCount = 0;
  sideEscapeAdaptiveLastBumpMs = 0;
}

static void resetClearanceEscapeMemory(const char* reason) {
  if (obstacleBypassPhase != BYPASS_IDLE || obstacleBypassSideSign != 0.0f) {
    char detail[128];
    snprintf(detail, sizeof(detail),
             "phase=%s;reason=%s;observed=%.0f;latched=%.0f",
             obstacleBypassPhaseName(obstacleBypassPhase), reason,
             obstacleObservedPreferredSideSign, obstacleBypassSideSign);
    sendBluetoothEvent("obstacle_bypass_reset", detail);
  }
  obstacleBypassPhase = BYPASS_IDLE;
  obstacleObservedPreferredSideSign = 0.0f;
  obstacleObservedCandidateSideSign = 0.0f;
  obstacleObservedStableCount = 0;
  obstacleObservedLastUpdateMs = 0;
  obstacleBypassSideSign = 0.0f;
  obstacleBypassMaxOutwardM = 0.0f;
  obstacleBypassTargetOutwardM = 0.0f;
  obstacleBypassPhaseStartedMs = 0;
  lastClearanceEscapeSideSign = 0.0f;
  lastClearanceEscapeMs = 0;
  clearanceEscapeRouteDetourActive = false;
  resetSideEscapeAdaptation();
}

static void updateObservedPreferredSide(const SideEscapeEvidence &evidence) {
  unsigned long now = millis();
  // readSideEscapeEvidence() is used by several decisions in one planner tick;
  // count at most one observation per millisecond so repeated reads cannot
  // manufacture stability.
  if (obstacleObservedLastUpdateMs == now) {
    return;
  }
  obstacleObservedLastUpdateMs = now;

  if (!evidence.sensorsValid || !evidence.closeSideEvidence) {
    obstacleObservedCandidateSideSign = 0.0f;
    obstacleObservedStableCount = 0;
    obstacleObservedPreferredSideSign = 0.0f;
    return;
  }

  float observedSideSign = evidence.sidePreferenceValid ? evidence.sideSign : 0.0f;
  if (observedSideSign == 0.0f && recentBlockedTurnDirection != 0 &&
      now - recentBlockedTurnMs <= PLANNER_SIDE_ESCAPE_MEMORY_MS) {
    // A tied front-wall view has no geometric open-side winner.  The current
    // blocked-turn observation supplies a deterministic fallback, but it is
    // still required to repeat before it becomes a latch candidate.
    observedSideSign = -recentBlockedTurnDirection;
  }
  if (observedSideSign == 0.0f) {
    obstacleObservedCandidateSideSign = 0.0f;
    obstacleObservedStableCount = 0;
    obstacleObservedPreferredSideSign = 0.0f;
    return;
  }

  if (observedSideSign != obstacleObservedCandidateSideSign) {
    obstacleObservedCandidateSideSign = observedSideSign;
    obstacleObservedStableCount = 1;
  } else if (obstacleObservedStableCount < 2) {
    obstacleObservedStableCount++;
  }
  obstacleObservedPreferredSideSign = obstacleObservedStableCount >= 2
    ? obstacleObservedCandidateSideSign : 0.0f;
}

static bool readSideEscapeEvidence(SideEscapeEvidence &evidence) {
  // Reads the four fan rays into a compact side-escape summary. sideSign is
  // -1 for robot-right and +1 for robot-left in the navigation frame.
  evidence.sensorsValid =
    isRangeSensorValid(RANGE_RIGHT_INNER) &&
    isRangeSensorValid(RANGE_LEFT_INNER) &&
    isRangeSensorValid(RANGE_RIGHT_OUTER) &&
    isRangeSensorValid(RANGE_LEFT_OUTER);
  evidence.rightM = -1.0f;
  evidence.leftM = -1.0f;
  evidence.restrictedM = -1.0f;
  evidence.triggerM = sideEscapeTriggerM();
  evidence.closeSideEvidence = false;
  evidence.sidePreferenceValid = false;
  evidence.sideSign = 0.0f;

  if (!evidence.sensorsValid) {
    updateObservedPreferredSide(evidence);
    return false;
  }

  float rightInnerM = getRangeSensorDistance(RANGE_RIGHT_INNER) / 1000.0f;
  float leftInnerM = getRangeSensorDistance(RANGE_LEFT_INNER) / 1000.0f;
  float rightOuterM = getRangeSensorDistance(RANGE_RIGHT_OUTER) / 1000.0f;
  float leftOuterM = getRangeSensorDistance(RANGE_LEFT_OUTER) / 1000.0f;
  evidence.rightM = min(rightInnerM, rightOuterM);
  evidence.leftM = min(leftInnerM, leftOuterM);
  evidence.restrictedM = min(evidence.rightM, evidence.leftM);
  evidence.closeSideEvidence = evidence.restrictedM < evidence.triggerM;
  evidence.sidePreferenceValid =
    fabs(evidence.rightM - evidence.leftM) >= PLANNER_SIDE_ESCAPE_TIE_M;
  evidence.sideSign = evidence.rightM > evidence.leftM ? -1.0f : 1.0f;
  updateObservedPreferredSide(evidence);
  return true;
}

static bool chooseSideEscapeSign(const SideEscapeEvidence &evidence,
                                 bool allowRecentMemory,
                                 float &sideSign) {
  sideSign = 0.0f;
  if (!evidence.closeSideEvidence) {
    return false;
  }

  if (obstacleBypassSideSign != 0.0f) {
    sideSign = obstacleBypassSideSign;
    return true;
  }

  if (obstacleObservedPreferredSideSign != 0.0f) {
    sideSign = obstacleObservedPreferredSideSign;
    return true;
  }

  if (allowRecentMemory &&
      lastClearanceEscapeSideSign != 0.0f &&
      millis() - lastClearanceEscapeMs <= PLANNER_SIDE_ESCAPE_MEMORY_MS) {
    sideSign = lastClearanceEscapeSideSign;
    return true;
  }

  return false;
}

static void rememberClearanceEscapeSide(float sideSign) {
  if (sideSign == 0.0f) {
    return;
  }

  if (obstacleBypassSideSign != 0.0f &&
      sideSign * obstacleBypassSideSign < 0.0f) {
    char detail[112];
    snprintf(detail, sizeof(detail),
             "reason=implicit_flip_rejected;phase=%s;observed=%.0f;latched=%.0f",
             obstacleBypassPhaseName(obstacleBypassPhase),
             obstacleObservedPreferredSideSign, obstacleBypassSideSign);
    sendBluetoothEvent("obstacle_bypass_side_rejected", detail);
    sideSign = obstacleBypassSideSign;
  }

  if (sideEscapeAdaptiveSideSign != 0.0f &&
      sideSign * sideEscapeAdaptiveSideSign < 0.0f) {
    resetSideEscapeAdaptation();
  }
  lastClearanceEscapeSideSign = sideSign;
  lastClearanceEscapeMs = millis();
  clearanceEscapeRouteDetourActive = true;
  if (obstacleBypassSideSign == 0.0f) {
    obstacleBypassSideSign = sideSign;
  }
  obstacleBypassTargetOutwardM =
    max(obstacleBypassTargetOutwardM, PLANNER_SIDE_ESCAPE_MIN_OUTWARD_M);

  float routeLengthM = 0.0f;
  float routeUx = 1.0f;
  float routeUy = 0.0f;
  float routeHeadingRad = 0.0f;
  if (routeLineFrame(routeLengthM, routeUx, routeUy, routeHeadingRad)) {
    float lateralM = routeLineSignedLateralErrorM(robotX, robotY, routeUx, routeUy);
    obstacleBypassMaxOutwardM =
      max(obstacleBypassMaxOutwardM, max(0.0f, sideSign * lateralM));
  }

  if (postReverseEscapeActive) {
    setObstacleBypassPhase(BYPASS_POST_REVERSE_ESCAPE, "post_reverse_escape");
  } else if (obstacleBypassPhase == BYPASS_IDLE ||
             obstacleBypassPhase == BYPASS_FINAL_APPROACH ||
             obstacleBypassPhase == BYPASS_ROUTE_REJOIN) {
    setObstacleBypassPhase(BYPASS_SIDE_ESCAPE, "side_evidence");
  }
}

static float sideEscapeAdaptiveExtraFor(float sideSign) {
  if (sideSign == 0.0f ||
      sideEscapeAdaptiveSideSign == 0.0f ||
      sideSign * sideEscapeAdaptiveSideSign <= 0.0f ||
      sideEscapeAdaptiveExtraM <= 0.0f) {
    return 0.0f;
  }

  if (millis() - sideEscapeAdaptiveLastBumpMs >
      PLANNER_SIDE_ESCAPE_ADAPT_MEMORY_MS) {
    resetSideEscapeAdaptation();
    return 0.0f;
  }

  return sideEscapeAdaptiveExtraM;
}

static void bumpSideEscapeAdaptiveExtra(float sideSign, const char* reason) {
  if (sideSign == 0.0f) {
    return;
  }

  unsigned long now = millis();
  if (sideEscapeAdaptiveSideSign != 0.0f &&
      sideSign * sideEscapeAdaptiveSideSign > 0.0f &&
      now - sideEscapeAdaptiveLastBumpMs <
        PLANNER_SIDE_ESCAPE_ADAPT_BUMP_COOLDOWN_MS) {
    return;
  }

  if (sideEscapeAdaptiveSideSign == 0.0f ||
      sideSign * sideEscapeAdaptiveSideSign < 0.0f) {
    sideEscapeAdaptiveExtraM = 0.0f;
    sideEscapeAdaptiveFailureCount = 0;
  }

  sideEscapeAdaptiveSideSign = sideSign;
  sideEscapeAdaptiveExtraM =
    min(PLANNER_SIDE_ESCAPE_ADAPT_MAX_EXTRA_M,
        sideEscapeAdaptiveExtraM + PLANNER_SIDE_ESCAPE_ADAPT_STEP_M);
  if (sideEscapeAdaptiveFailureCount < 255) {
    sideEscapeAdaptiveFailureCount++;
  }
  sideEscapeAdaptiveLastBumpMs = now;
  rememberClearanceEscapeSide(sideSign);

  char detail[96];
  snprintf(detail, sizeof(detail), "reason=%s;side=%s;extra=%.2f;failures=%u",
           reason, sideEscapeName(sideSign), sideEscapeAdaptiveExtraM,
           (unsigned int)sideEscapeAdaptiveFailureCount);
  sendBluetoothEvent("side_escape_widen", detail);
}

static void sendPostReverseEscapeEvent(const char* eventName,
                                       const char* reason,
                                       float sideSign,
                                       const SideEscapeEvidence &evidence) {
  char detail[120];
  snprintf(detail, sizeof(detail), "%s;side=%s;right=%.2f;left=%.2f;extra=%.2f",
           reason, sideEscapeName(sideSign), evidence.rightM, evidence.leftM,
           sideEscapeAdaptiveExtraFor(sideSign));
  sendBluetoothEvent(eventName, detail);
}

static float reverseRecoveryRetreatDistanceM() {
  return sqrtf((robotX - reverseRecoveryStartX) *
               (robotX - reverseRecoveryStartX) +
               (robotY - reverseRecoveryStartY) *
               (robotY - reverseRecoveryStartY));
}

static float reverseSurveyFrontDistanceM() {
  if (!isRangeSensorValid(RANGE_FRONT)) {
    return -1.0f;
  }
  return getRangeSensorDistance(RANGE_FRONT) / 1000.0f;
}

static void sendReverseSurveyEvent(const char* eventName,
                                   const char* reason,
                                   float retreatM,
                                   float frontM,
                                   bool forced,
                                   const SideEscapeEvidence &evidence) {
  char detail[144];
  snprintf(detail, sizeof(detail),
           "reason=%s;retreat=%.2f;required=%.2f;front=%.2f;side=%.2f;right=%.2f;left=%.2f;forced=%d",
           reason,
           retreatM,
           reverseSurveyRequiredRetreatM,
           frontM,
           evidence.restrictedM,
           evidence.rightM,
           evidence.leftM,
           forced ? 1 : 0);
  sendBluetoothEvent(eventName, detail);
}

static bool reverseSurveyObservationReady(SideEscapeEvidence &evidence,
                                          float &retreatM,
                                          float &frontM,
                                          bool &forced) {
  retreatM = reverseRecoveryRetreatDistanceM();
  frontM = reverseSurveyFrontDistanceM();
  forced = retreatM >= PLANNER_REVERSE_SURVEY_MAX_REVERSE_M;
  bool frontReady = frontM >= PLANNER_REVERSE_SURVEY_FRONT_CLEAR_M;
  bool sideReady = readSideEscapeEvidence(evidence) &&
                   evidence.restrictedM >= PLANNER_REVERSE_SURVEY_SIDE_CLEAR_M;
  bool retreatReady = retreatM >= reverseSurveyRequiredRetreatM;
  return forced || (retreatReady && frontReady && sideReady);
}

static void holdReverseSurvey(const char* planReason,
                              const char* replanReason) {
  plannerTelemetry.selectedForwardTicksPerSec = 0.0;
  plannerTelemetry.selectedTurnTicksPerSec = 0.0;
  plannerTelemetry.selectedCurvature = 0.0;
  plannerTelemetry.minimumSweptClearanceMm = minimumFanSweepClearanceMm();
  plannerTelemetry.speedCapTicksPerSec = 0.0;
  plannerTelemetry.candidateCount = 0;
  plannerTelemetry.stopReason = PLANNER_STOP_NONE;
  plannerTelemetry.planReason = planReason;
  plannerTelemetry.replanReason = replanReason;
  plannerTelemetry.safeStopReason = "";
  motorStopRequested = true;
  setMotionCommand(0.0, 0.0);
}

static ReverseSurveyDecision updateReverseSurveyDecision() {
  reverseSurveyReadyToTryForward = false;
  reverseSurveyForcedByMaxRetreat = false;

  SideEscapeEvidence evidence = {};
  float retreatM = 0.0f;
  float frontM = -1.0f;
  bool forced = false;
  if (!reverseSurveyObservationReady(evidence, retreatM, frontM, forced)) {
    reverseSurveySettling = false;
    reverseSurveyReadyReported = false;
    return REVERSE_SURVEY_REVERSE;
  }

  unsigned long now = millis();
  if (!reverseSurveySettling) {
    reverseSurveySettling = true;
    reverseSurveySettleStartedMs = now;
    reverseSurveyReadyReported = false;
    reverseSurveyForcedByMaxRetreat = forced;
    sendReverseSurveyEvent("reverse_survey_settle",
                           forced ? "max_retreat" : "observation_pose",
                           retreatM,
                           frontM,
                           forced,
                           evidence);
    holdReverseSurvey("reverse_survey_settle",
                      "waiting_for_fresh_side_fan");
    return REVERSE_SURVEY_HOLD;
  }

  if (now - reverseSurveySettleStartedMs < PLANNER_REVERSE_SURVEY_SETTLE_MS) {
    reverseSurveyForcedByMaxRetreat = forced;
    holdReverseSurvey("reverse_survey_settle",
                      "waiting_for_fresh_side_fan");
    return REVERSE_SURVEY_HOLD;
  }

  reverseSurveyReadyToTryForward = true;
  reverseSurveyForcedByMaxRetreat = forced;
  if (!reverseSurveyReadyReported) {
    sendReverseSurveyEvent("reverse_survey_ready",
                           forced ? "max_retreat" : "fresh_observation",
                           retreatM,
                           frontM,
                           forced,
                           evidence);
    reverseSurveyReadyReported = true;
  }
  return REVERSE_SURVEY_FORWARD;
}

static void requestReverseSurveyRetry(const char* reason) {
  float retreatM = reverseRecoveryRetreatDistanceM();
  reverseSurveyRequiredRetreatM =
    min(PLANNER_REVERSE_SURVEY_MAX_REVERSE_M,
        max(reverseSurveyRequiredRetreatM,
            retreatM + PLANNER_REVERSE_SURVEY_RETRY_REVERSE_M));
  reverseSurveySettling = false;
  reverseSurveySettleStartedMs = 0;
  reverseSurveyReadyReported = false;
  reverseSurveyReadyToTryForward = false;
  reverseSurveyForcedByMaxRetreat = false;

  char detail[96];
  snprintf(detail, sizeof(detail), "reason=%s;retreat=%.2f;required=%.2f",
           reason, retreatM, reverseSurveyRequiredRetreatM);
  sendBluetoothEvent("reverse_survey_retry", detail);
}

static float calculateReverseRecoverySpeedCapTicksPerSec() {
  if (!isRangeSensorValid(RANGE_FAKE_REAR) ||
      isRangeSensorBlocked(RANGE_FAKE_REAR)) {
    return 0.0f;
  }

  float availableM = getRangeSensorDistance(RANGE_FAKE_REAR) / 1000.0f -
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
                                            float &finalGoalDistanceM,
                                            float &finalHeadingErrorDeg,
                                            float &finalRouteLateralErrorM,
                                            CandidateRejectReason &rejectReason) {
  if (reverseTicks >= 0.0f) {
    rejectReason = CANDIDATE_REJECT_REAR_OBSERVATION;
    return false;
  }
  if (!epoch.rearValid || epoch.rearBlocked) {
    rejectReason = CANDIDATE_REJECT_REAR_OBSERVATION;
    return false;
  }

  float observedRearM = epoch.observedRearM;
  float trailingEnvelopeM = ROBOT_FOOTPRINT_GEOMETRY.rearExtentMm / 1000.0f +
                            PLANNER_TOTAL_HARD_CLEARANCE_M +
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
  float travelledRearM = 0.0f;
  finalRouteLateralErrorM = 0.0f;

  float routeLengthM = 0.0f;
  float routeUx = 1.0f;
  float routeUy = 0.0f;
  float routeHeadingRad = 0.0f;
  bool hasRouteFrame = routeLineFrame(routeLengthM, routeUx, routeUy, routeHeadingRad);

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
  }

  float goalDx = goalX - x;
  float goalDy = goalY - y;
  finalGoalDistanceM = sqrtf(goalDx * goalDx + goalDy * goalDy);
  float desiredHeadingDeg = atan2f(goalDy, goalDx) * RAD_TO_DEG;
  finalHeadingErrorDeg = fabs(wrapAngle(desiredHeadingDeg - heading * RAD_TO_DEG));
  rearClearanceMm = max(0.0f, (observedRearM - travelledRearM - trailingEnvelopeM) * 1000.0f);
  if (hasRouteFrame) {
    finalRouteLateralErrorM = routeLineLateralErrorM(x, y, routeUx, routeUy);
  }
  rejectReason = CANDIDATE_REJECT_NONE;
  return true;
}

static float reverseRecoveryScore(float reverseTicks, float turnTicks,
                                  float plannerStartX, float plannerStartY,
                                  float previousSelectedTurn,
                                  float rearClearanceMm,
                                  float finalGoalDistanceM,
                                  float finalHeadingErrorDeg,
                                  float finalRouteLateralErrorM) {
  float startGoalDistanceM = sqrtf(
    (navigationGoal.targetX - plannerStartX) *
      (navigationGoal.targetX - plannerStartX) +
    (navigationGoal.targetY - plannerStartY) *
      (navigationGoal.targetY - plannerStartY));
  float progressScore = constrain((startGoalDistanceM - finalGoalDistanceM) /
                                  max(PLANNER_MIN_PROGRESS_M, startGoalDistanceM),
                                  -1.0f, 1.0f);
  float headingScore = 1.0f - min(1.0f, finalHeadingErrorDeg / 120.0f);
  float clearanceScore = constrain(rearClearanceMm /
                                   (PLANNER_PREFERRED_CLEARANCE_M * 1000.0f),
                                   0.0f, 1.0f);
  float curvature = fabs(turnTicks) / max(1.0f, fabs(reverseTicks));
  float smoothnessScore = 1.0f - min(1.0f, fabs(turnTicks - previousSelectedTurn) /
                                          max(1.0f, baseTargetSpeed));
  float routeScore = 1.0f - min(1.0f, finalRouteLateralErrorM /
                                      PLANNER_LINE_FOLLOW_LATERAL_TOLERANCE_M);
  float reverseProgressWeight = reverseSurveyReadyToTryForward ? 0.5f : 0.15f;
  float reverseCurvaturePenalty = reverseSurveyReadyToTryForward ? 0.9f : 2.0f;
  return 1.6f * headingScore + 1.4f * clearanceScore +
         0.8f * smoothnessScore + 0.8f * routeScore +
         reverseProgressWeight * progressScore -
         reverseCurvaturePenalty * curvature;
}

static void captureReversePlannerEpochView(ReversePlannerEpoch &epoch) {
  epoch.startX = robotX;
  epoch.startY = robotY;
  epoch.startHeadingRad = navigationHeadingRad();
  epoch.rearValid = isRangeSensorValid(RANGE_FAKE_REAR);
  epoch.rearBlocked = isRangeSensorBlocked(RANGE_FAKE_REAR);
  epoch.observedRearM = epoch.rearValid
    ? getRangeSensorDistance(RANGE_FAKE_REAR) / 1000.0f : 0.0f;
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
  // current test build, rear evidence comes from RANGE_FAKE_REAR.
  memset(&reversePlannerEpoch, 0, sizeof(reversePlannerEpoch));
  reversePlannerEpoch.active = true;
  reversePlannerEpoch.startedMs = millis();
  reversePlannerEpoch.goalStartedMs = navigationGoal.startedMs;
  reversePlannerEpoch.authority = navigationGoal.authority;
  reversePlannerEpoch.goalX = goalX;
  reversePlannerEpoch.goalY = goalY;
  reversePlannerEpoch.bestScore = -1000000.0f;
  reversePlannerEpoch.bestRearClearance = -1.0f;
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
    plannerTelemetry.replanReason = "fake_rear_tof_unavailable";
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
      float finalGoalDistanceM = 0.0f;
      float finalHeadingErrorDeg = 180.0f;
      float finalRouteLateralErrorM = 0.0f;
      CandidateRejectReason rejectReason = CANDIDATE_REJECT_NONE;
      if (!rolloutReverseRecoveryCandidate(
            reversePlannerEpoch, reverseTicks, turnTicks,
            reversePlannerEpoch.goalX, reversePlannerEpoch.goalY,
            rearClearanceMm, finalGoalDistanceM, finalHeadingErrorDeg,
            finalRouteLateralErrorM, rejectReason)) {
        if (rejectReason == CANDIDATE_REJECT_REAR_OBSERVATION) {
          reversePlannerEpoch.rejectedRear++;
        } else if (rejectReason == CANDIDATE_REJECT_FOOTPRINT) {
          reversePlannerEpoch.rejectedFootprint++;
        }
        continue;
      }
      reversePlannerEpoch.acceptedCount++;
      plannerTelemetry.candidateCount =
        reversePlannerEpoch.acceptedCount;
      float score = reverseRecoveryScore(
        reverseTicks, turnTicks,
        reversePlannerEpoch.startX, reversePlannerEpoch.startY,
        reversePlannerEpoch.previousSelectedTurn,
        rearClearanceMm, finalGoalDistanceM, finalHeadingErrorDeg,
        finalRouteLateralErrorM);
      if (score > reversePlannerEpoch.bestScore) {
        reversePlannerEpoch.bestScore = score;
        reversePlannerEpoch.bestReverse = reverseTicks;
        reversePlannerEpoch.bestTurn = turnTicks;
        reversePlannerEpoch.bestRearClearance = rearClearanceMm;
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
    plannerTelemetry.stopReason = PLANNER_STOP_NO_SAFE_TRAJECTORY;
    plannerTelemetry.safeStopReason = "no_reverse_recovery_arc";
    plannerTelemetry.replanReason = "no_reverse_arc";
    if (!reverseRecoveryRejectsReported) {
      char detail[64];
      snprintf(detail, sizeof(detail), "rear=%d;footprint=%d",
               reversePlannerEpoch.rejectedRear,
               reversePlannerEpoch.rejectedFootprint);
      sendBluetoothEvent("reverse_recovery_rejects", detail);
      reverseRecoveryRejectsReported = true;
    }
    closeReversePlannerEpoch();
    return TRAJECTORY_PLAN_NO_PATH;
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
  float finalGoalDistanceM = 0.0f;
  float finalHeadingErrorDeg = 180.0f;
  float finalRouteLateralErrorM = 0.0f;
  CandidateRejectReason rejectReason = CANDIDATE_REJECT_NONE;
  bool winnerStillSafe = rolloutReverseRecoveryCandidate(
      reversePlannerEpoch,
      publishReverse, publishTurn,
      reversePlannerEpoch.goalX, reversePlannerEpoch.goalY,
      rearClearanceMm, finalGoalDistanceM, finalHeadingErrorDeg,
      finalRouteLateralErrorM, rejectReason);
  recordReversePlannerSlice(sliceStartedUs);
  if (!winnerStillSafe) {
    return retryReversePlannerEpoch("reverse_winner_revalidation_rejected",
                                    "reverse_winner_revalidation_retry");
  }

  reverseRecoveryStepCount++;
  plannerTelemetry.selectedForwardTicksPerSec =
    publishReverse;
  plannerTelemetry.selectedTurnTicksPerSec = publishTurn;
  plannerTelemetry.selectedCurvature = publishTurn /
    max(1.0f, fabs(publishReverse));
  plannerTelemetry.minimumSweptClearanceMm = rearClearanceMm;
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

static void buildSideEscapeLocalGoal(float sideSign,
                                     float extraLateralM,
                                     float &localGoalX,
                                     float &localGoalY) {
  // Robot +Y is left. An open right side must therefore create a negative-Y
  // local target (and vice versa). On routed point goals, express that offset
  // in the original route frame so the escape remains a forward bypass instead
  // of rotating with the chassis and walking sideways away from the waypoint.
  float lateralOffsetM = (sideSign > 0.0f
    ? ROBOT_FOOTPRINT_GEOMETRY.leftExtentMm
    : ROBOT_FOOTPRINT_GEOMETRY.rightExtentMm) / 1000.0f +
    PLANNER_PREFERRED_CLEARANCE_M + extraLateralM +
    sideEscapeAdaptiveExtraFor(sideSign);
  lateralOffsetM = max(lateralOffsetM, obstacleBypassTargetOutwardM);

  float routeLengthM = 0.0f;
  float routeUx = 1.0f;
  float routeUy = 0.0f;
  float routeHeadingRad = 0.0f;
  if (routeLineFrame(routeLengthM, routeUx, routeUy, routeHeadingRad)) {
    float alongM = routeLineAlongM(robotX, robotY, routeUx, routeUy);
    float lookaheadAlongM = constrain(alongM + WAYPOINT_LOOKAHEAD_M,
                                      0.0f, routeLengthM);
    float currentLateralM =
      routeLineSignedLateralErrorM(robotX, robotY, routeUx, routeUy);
    float targetLateralM = sideSign * lateralOffsetM;
    float currentOutwardM = sideSign * currentLateralM;
    if (currentOutwardM > lateralOffsetM) {
      // Already wider than the planned bypass lane. Hold that lateral offset
      // until the route-rejoin/final-approach phase rather than steering back
      // inward while the wall edge may still be beside the chassis.
      targetLateralM = currentLateralM;
      obstacleBypassMaxOutwardM =
        max(obstacleBypassMaxOutwardM, max(0.0f, currentOutwardM));
    }
    localGoalX = navigationGoal.startX + routeUx * lookaheadAlongM -
                 routeUy * targetLateralM;
    localGoalY = navigationGoal.startY + routeUy * lookaheadAlongM +
                 routeUx * targetLateralM;
    return;
  }

  float headingRad = navigationHeadingRad();
  localGoalX = robotX + WAYPOINT_LOOKAHEAD_M * cosf(headingRad) -
               sideSign * lateralOffsetM * sinf(headingRad);
  localGoalY = robotY + WAYPOINT_LOOKAHEAD_M * sinf(headingRad) +
               sideSign * lateralOffsetM * cosf(headingRad);
}

static bool buildClearanceEscapeLocalGoal(float &localGoalX, float &localGoalY) {
  // Nearby side evidence on either the inner or outer rays can describe an
  // observable side exit. Give the arc sampler that exit as its temporary
  // local goal instead of scoring only progress along the blocked route line.
  if (obstacleBypassPhase == BYPASS_FINAL_APPROACH) {
    return false;
  }

  SideEscapeEvidence evidence;
  if (!readSideEscapeEvidence(evidence)) {
    return false;
  }

  float sideSign = 0.0f;
  if (!chooseSideEscapeSign(evidence, true, sideSign)) {
    return false;
  }

  rememberClearanceEscapeSide(sideSign);
  buildSideEscapeLocalGoal(sideSign, 0.0f, localGoalX, localGoalY);
  plannerTelemetry.replanReason = "clearance_escape_waypoint";
  return true;
}

static bool sideEscapeShouldBypassPointAlignment() {
  if (postReverseEscapeActive) {
    return true;
  }

  if (obstacleBypassPhase == BYPASS_SIDE_ESCAPE ||
      obstacleBypassPhase == BYPASS_FRONT_WALL_REVERSE ||
      obstacleBypassPhase == BYPASS_ROUTE_REJOIN) {
    return true;
  }

  if (lastClearanceEscapeSideSign != 0.0f &&
      millis() - lastClearanceEscapeMs <= PLANNER_SIDE_ESCAPE_MEMORY_MS) {
    return true;
  }

  SideEscapeEvidence evidence;
  if (!readSideEscapeEvidence(evidence)) {
    return false;
  }

  float sideSign = 0.0f;
  return chooseSideEscapeSign(evidence, true, sideSign);
}

static bool buildPostReverseEscapeLocalGoal(float &localGoalX,
                                            float &localGoalY,
                                            bool allowPendingStart,
                                            bool &pendingStart,
                                            float &pendingSideSign,
                                            SideEscapeEvidence &pendingEvidence) {
  pendingStart = false;
  SideEscapeEvidence evidence;
  if (!readSideEscapeEvidence(evidence)) {
    if (postReverseEscapeActive) {
      sendPostReverseEscapeEvent("post_reverse_escape_unavailable",
                                 "side_sensor_invalid",
                                 postReverseEscapeSideSign,
                                 evidence);
    }
    resetPostReverseEscapeState();
    return false;
  }

  unsigned long now = millis();
  if (!evidence.closeSideEvidence &&
      (!postReverseEscapeActive ||
       now - postReverseEscapeStartedMs >= PLANNER_POST_REVERSE_ESCAPE_MIN_MS)) {
    if (postReverseEscapeActive) {
      sendPostReverseEscapeEvent("post_reverse_escape_end",
                                 "side_clear",
                                 postReverseEscapeSideSign,
                                 evidence);
      setObstacleBypassPhase(BYPASS_ROUTE_REJOIN, "side_clear");
    }
    resetPostReverseEscapeState();
    return false;
  }

  if (postReverseEscapeActive &&
      now - postReverseEscapeStartedMs > PLANNER_POST_REVERSE_ESCAPE_TIMEOUT_MS) {
    bumpSideEscapeAdaptiveExtra(postReverseEscapeSideSign, "post_timeout");
    sendPostReverseEscapeEvent("post_reverse_escape_unavailable",
                               "timeout",
                               postReverseEscapeSideSign,
                               evidence);
    setObstacleBypassPhase(BYPASS_ROUTE_REJOIN, "post_timeout");
    resetPostReverseEscapeState();
    return false;
  }

  float sideSign = postReverseEscapeActive
    ? postReverseEscapeSideSign : obstacleBypassSideSign;
  if (!postReverseEscapeActive &&
      (sideSign == 0.0f || !evidence.closeSideEvidence)) {
    return false;
  }

  if (!postReverseEscapeActive) {
    if (!allowPendingStart) {
      return false;
    }
    pendingStart = true;
    pendingSideSign = sideSign;
    pendingEvidence = evidence;
  }

  buildSideEscapeLocalGoal(sideSign,
                           PLANNER_POST_REVERSE_ESCAPE_EXTRA_LATERAL_M,
                           localGoalX,
                           localGoalY);
  rememberClearanceEscapeSide(sideSign);
  plannerTelemetry.replanReason = "post_reverse_escape_waypoint";
  return true;
}

static void startPostReverseEscape(const SideEscapeEvidence &evidence) {
  postReverseEscapeActive = true;
  postReverseEscapeSideSign = obstacleBypassSideSign;
  postReverseEscapeStartedMs = millis();
  setObstacleBypassPhase(BYPASS_POST_REVERSE_ESCAPE, "post_reverse_start");
  rememberClearanceEscapeSide(obstacleBypassSideSign);
  sendPostReverseEscapeEvent("post_reverse_escape_start",
                             "forward_path_found",
                             obstacleBypassSideSign,
                             evidence);
}
#endif

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

  bool huntCarryThroughActive =
    routeLineEligible && huntPickupCarryThroughActive(routeLengthM, routeUx, routeUy);
  bool avoidanceActive = updateObstacleContext(navigationGoal.targetX,
                                               navigationGoal.targetY);
  float targetHeadingDeg = atan2f(dy, dx) * RAD_TO_DEG;
  float targetHeadingErrorDeg = wrapAngle(targetHeadingDeg - navigationHeadingDeg());
  if (!plannerEpoch.active && !avoidanceActive &&
      !huntCarryThroughActive &&
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
  setMotionCommand(PLANNER_DEFAULT_SAFE_STOP_SPEED_MPS, 0.0);
  reportPlannerStopIfChanged();
  if (!currentPlannerFailureIsGeometricNoPath()) {
    resetGeometricNoPathEvidence();
    return;
  }
  noteGeometricNoPathEpoch();
  if (millis() - noSafeTrajectorySinceMs >= PLANNER_NO_PATH_ABORT_MS) {
    finishNavigationGoal(false, PLANNER_STOP_NO_SAFE_TRAJECTORY,
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
  if (!isTurnDirectionObservable(turnTarget)) {
    // A missing ray could be a single bad sample. Stop before turning and give
    // the fan a short stationary revalidation window rather than guessing.
    unsigned long now = millis();
    if (turnSideInvalidSinceMs == 0) {
      turnSideInvalidSinceMs = now;
      sendBluetoothEvent("turn_side_revalidate", "motors_stopped_for_fresh_sample");
    }
    motorStopRequested = true;
    setMotionCommand(0.0, 0.0);
    if (now - turnSideInvalidSinceMs < PLANNER_TURN_SENSOR_REVALIDATE_MS) {
      plannerTelemetry.planReason = "turn_side_revalidating";
      return;
    }
    finishNavigationGoal(false, PLANNER_STOP_TURN_SIDE_INVALID, "turn_side_sensor_invalid");
    return;
  }
  turnSideInvalidSinceMs = 0;

  if (!areTurnSweepSensorsValid()) {
    // Direction sensing proves the chosen side, while this all-fan check proves
    // the complete rotating chassis envelope. Both are needed for a pivot.
    unsigned long now = millis();
    if (turnSweepInvalidSinceMs == 0) {
      turnSweepInvalidSinceMs = now;
      sendBluetoothEvent("turn_sweep_revalidate", "motors_stopped_for_fresh_sample");
    }
    motorStopRequested = true;
    setMotionCommand(0.0, 0.0);
    if (now - turnSweepInvalidSinceMs < PLANNER_TURN_SENSOR_REVALIDATE_MS) {
      plannerTelemetry.planReason = "turn_sweep_revalidating";
      return;
    }
    finishNavigationGoal(false, PLANNER_STOP_TURN_CLEARANCE, "turn_sweep_sensor_invalid");
    return;
  }
  turnSweepInvalidSinceMs = 0;

  if (!isTurnSweepSafe()) {
    finishNavigationGoal(false, PLANNER_STOP_TURN_CLEARANCE, "turn_sweep_not_clear");
    return;
  }

  updateStuckTurning(navigationHeadingDeg());
  if (turnStuck) {
    finishNavigationGoal(false, PLANNER_STOP_STUCK, "turn_progress_stalled");
    return;
  }

  plannerTelemetry.selectedForwardTicksPerSec = 0.0;
  plannerTelemetry.selectedTurnTicksPerSec = turnTarget;
  plannerTelemetry.selectedCurvature = 0.0;
  plannerTelemetry.minimumSweptClearanceMm = minimumFanSweepClearanceMm();
  plannerTelemetry.speedCapTicksPerSec = 0.0;
  plannerTelemetry.candidateCount = 1;
  plannerTelemetry.stopReason = PLANNER_STOP_NONE;
  plannerTelemetry.planReason = slowTurn ? "calibrated_turn_slow" : "calibrated_turn_fast";
  motorStopRequested = false;
  publishNavigationMotion(0.0, turnTarget);
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
    plannerEpoch.active;
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
      setMotionCommand(0.0, 0.0);
      plannerTelemetry.stopReason = PLANNER_STOP_NONE;
      plannerTelemetry.safeStopReason = "tof_close_revalidating";
      plannerTelemetry.replanReason = "tof_close_revalidating";
    }

    if (!immediateSafetyStop && navigationGoal.active &&
        isRangeSensorBlocked(RANGE_FRONT)) {
      immediateSafetyStop = true;
      motorStopRequested = true;
      setMotionCommand(0.0, 0.0);
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
      setMotionCommand(0.0, 0.0);
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
