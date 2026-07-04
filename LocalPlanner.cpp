#include "Robot.h"

// =====================================================
// Local confidence map and receding-horizon navigation
// =====================================================
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
static bool reverseRecoveryActive = false;
static float reverseRecoveryStartX = 0.0;
static float reverseRecoveryStartY = 0.0;
static unsigned long reverseRecoveryStartedMs = 0;
static unsigned long reverseRecoveryStepCount = 0;
static unsigned long noSafeTrajectorySinceMs = 0;
static bool candidateRejectsReported = false;
static bool reverseRecoveryRejectsReported = false;
static bool clearanceEscapeLocalGoalActive = false;
static float lastFootprintRejectWorldX = 0.0f;
static float lastFootprintRejectWorldY = 0.0f;
static int lastFootprintRejectCellX = -1;
static int lastFootprintRejectCellY = -1;
static float lastCorridorRejectLeftM = -1.0f;
static float lastCorridorRejectRightM = -1.0f;

enum CandidateRejectReason {
  CANDIDATE_REJECT_NONE,
  CANDIDATE_REJECT_TURN_OBSERVABILITY,
  CANDIDATE_REJECT_FORWARD_OBSERVATION,
  CANDIDATE_REJECT_REAR_OBSERVATION,
  CANDIDATE_REJECT_FOOTPRINT,
  CANDIDATE_REJECT_CORRIDOR
};

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

static float lateralObstacleDistanceM(float worldX, float worldY,
                                      float headingRad, float sideSign) {
  // Cast sideways through the map from a predicted robot pose. This is not a
  // wall follower; it is only a local question: are both corridor boundaries
  // close enough that steering must be restrained?
  const float lateralX = -sinf(headingRad) * sideSign;
  const float lateralY = cosf(headingRad) * sideSign;
  for (float distanceM = 0.0;
       distanceM <= PLANNER_CORRIDOR_SIDE_SEARCH_M;
       distanceM += LOCAL_MAP_CELL_M * 0.5) {
    if (worldOccupied(worldX + lateralX * distanceM,
                      worldY + lateralY * distanceM)) {
      return distanceM;
    }
  }
  return -1.0;
}

static bool isNarrowObservedCorridor(float worldX, float worldY, float headingRad) {
  float leftBoundaryM = lateralObstacleDistanceM(worldX, worldY, headingRad, 1.0);
  float rightBoundaryM = lateralObstacleDistanceM(worldX, worldY, headingRad, -1.0);
  // Require *both* boundaries. One nearby obstacle is an open-side bypass,
  // not a corridor, and should still allow a normal avoiding arc.
  bool narrow = leftBoundaryM >= 0.0 && rightBoundaryM >= 0.0 &&
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

  // Sample every map cell across the inflated rectangle, not merely the
  // corners.  A narrow panel therefore cannot slip between sparse footprint
  // sample points during a curved rollout.
  for (float localX = -rear; localX <= front + 0.001; localX += LOCAL_MAP_CELL_M) {
    for (float localY = -right; localY <= left + 0.001; localY += LOCAL_MAP_CELL_M) {
      float pointX = worldX + localX * cosf(headingRad) - localY * sinf(headingRad);
      float pointY = worldY + localX * sinf(headingRad) + localY * cosf(headingRad);
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
    case PLANNER_STOP_ABORTED: return "aborted";
  }
  return "unknown";
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
  reverseRecoveryActive = false;
  noSafeTrajectorySinceMs = 0;
  reverseRecoveryRejectsReported = false;
  navigationGoal.active = false;
  navigationGoal.completed = success;
  navigationGoal.failed = !success;
  plannerTelemetry.stopReason = reason;
  plannerTelemetry.safeStopReason = detail;
  motorStopRequested = true;
  setMotionCommand(0.0, 0.0);
  sendBluetoothEvent(ownerEventName(owner, success), detail);

  if (ownerIsTest(owner)) {
    robotRunEnabled = false;
    setRobotState(END_MATCH);
    Serial.println(success ? "TEST complete. Motors stopped." : "TEST aborted. Motors stopped.");
  }
}

void startNavigationPoint(float targetX, float targetY, NavigationGoalOwner owner) {
  // A point goal does not mean "drive this exact line". It means repeatedly
  // choose a short safe arc that makes progress toward this world coordinate.
  navigationGoal.mode = NAV_GOAL_POINT;
  navigationGoal.owner = owner;
  navigationGoal.active = true;
  navigationGoal.completed = false;
  navigationGoal.failed = false;
  navigationGoal.targetX = targetX;
  navigationGoal.targetY = targetY;
  navigationGoal.targetYawDeg = 0.0;
  navigationGoal.startX = robotX;
  navigationGoal.startY = robotY;
  navigationGoal.startYawDeg = readYawDeg();
  navigationGoal.startedMs = millis();
  // Keep cumulative encoder totals intact, but reset the control snapshots so
  // a previous motion segment cannot create a derivative/PID kick here.
  resetEncodersAndPID();
  // Force a plan on the next controller pass rather than waiting for the old
  // goal's 40 ms schedule phase.
  lastPlannerUpdateMs = 0;
  motorStopRequested = false;
  // Telemetry is reset with the goal. Without this, a previous turn command
  // can make the first point-goal CSV row look like it is steering.
  plannerTelemetry.selectedForwardTicksPerSec = 0.0;
  plannerTelemetry.selectedTurnTicksPerSec = 0.0;
  plannerTelemetry.selectedCurvature = 0.0;
  plannerTelemetry.minimumSweptClearanceMm = -1.0;
  plannerTelemetry.speedCapTicksPerSec = 0.0;
  plannerTelemetry.localGoalDistanceM = 0.0;
  plannerTelemetry.candidateCount = 0;
  plannerTelemetry.stopReason = PLANNER_STOP_NONE;
  plannerTelemetry.planReason = "goal_started";
  lastReportedStopReason = PLANNER_STOP_NONE;
  recentBlockedTurnDirection = 0;
  recentBlockedTurnMs = 0;
  reverseRecoveryActive = false;
  reverseRecoveryStepCount = 0;
  noSafeTrajectorySinceMs = 0;
  candidateRejectsReported = false;
  reverseRecoveryRejectsReported = false;
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
  // Turns are their own direct yaw-feedback task. They do not use the map arc
  // sampler because they are intentionally in-place and run at 20 ms.
  float startYawDeg = navigationHeadingDeg();
  navigationGoal.mode = NAV_GOAL_TURN;
  navigationGoal.owner = owner;
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
  noSafeTrajectorySinceMs = 0;
  reverseRecoveryRejectsReported = false;
  motorStopRequested = false;
  plannerTelemetry.replanReason = "turn_started";
  plannerTelemetry.safeStopReason = "";
  sendBluetoothEvent("navigation_goal_start", "turn");
}

void cancelNavigationGoal(PlannerStopReason reason, const char* detail) {
  if (!navigationGoal.active) {
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

static float observedForwardDistanceForCandidate(float turnTicks) {
  // The virtual front is the conservative minimum of both inner rays, which
  // is right for a straight path.  A committed arc, however, moves its
  // leading footprint toward the side it is turning into.  Do not let an
  // obstacle observed only on the opposite inner ray veto that escape arc
  // when the turning-side inner ray is valid and open.
  if (turnTicks > 1.0f) {
    return fanForwardObservationDistanceM(RANGE_RIGHT_INNER);
  }
  if (turnTicks < -1.0f) {
    return fanForwardObservationDistanceM(RANGE_LEFT_INNER);
  }

  float observedM = 1000000.0f;
  bool hasObservation = false;
  if (isRangeSensorValid(RANGE_RIGHT_INNER)) {
    observedM = min(observedM, fanForwardObservationDistanceM(RANGE_RIGHT_INNER));
    hasObservation = true;
  }
  if (isRangeSensorValid(RANGE_LEFT_INNER)) {
    observedM = min(observedM, fanForwardObservationDistanceM(RANGE_LEFT_INNER));
    hasObservation = true;
  }

  return hasObservation ? observedM : getRangeSensorDistance(RANGE_FRONT) / 1000.0f;
}

static float clearanceEscapeUrgency() {
  if (!clearanceEscapeLocalGoalActive || !isRangeSensorValid(RANGE_FRONT)) {
    return 0.0f;
  }

  float frontM = getRangeSensorDistance(RANGE_FRONT) / 1000.0f;
  float spanM = max(0.001f, PLANNER_ESCAPE_SPEED_LIMIT_START_M -
                              PLANNER_ESCAPE_SPEED_LIMIT_FULL_M);
  return constrain((PLANNER_ESCAPE_SPEED_LIMIT_START_M - frontM) / spanM,
                   0.0f, 1.0f);
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

static bool rolloutCandidate(float forwardTicks, float turnTicks,
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
  // PLANNER_HORIZON_S. Positive turn means left wheel faster/right wheel
  // slower; MotorControl later converts this into two wheel targets.
  if (forwardTicks <= 0.0 || !isTurnDirectionObservable(turnTicks)) {
    rejectReason = CANDIDATE_REJECT_TURN_OBSERVABILITY;
    return false;
  }
  rejectReason = CANDIDATE_REJECT_NONE;

  // A candidate may not run its leading footprint beyond the distance that the
  // valid inner fan has actually observed.  Unknown space is not converted to
  // free space merely because it is absent from the local map.
  float observedForwardM = observedForwardDistanceForCandidate(turnTicks);
  float leadingEnvelopeM = ROBOT_FOOTPRINT_GEOMETRY.frontExtentMm / 1000.0 +
                           PLANNER_TOTAL_HARD_CLEARANCE_M;

  float heading = navigationHeadingRad();
  float x = robotX;
  float y = robotY;
  // Differential-drive kinematics: command is expressed as average wheel
  // speed plus/minus a turn component, then integrated at the midpoint
  // heading to avoid the bias of a simple Euler step.
  float leftMps = (forwardTicks + turnTicks) / TICKS_PER_METRE;
  float rightMps = (forwardTicks - turnTicks) / TICKS_PER_METRE;
  float linearMps = (leftMps + rightMps) * 0.5;
  // Positive turn ticks command a physical right turn. Navigation heading is
  // counter-clockwise-positive, so its angular rate is the opposite sign.
  float angularRadPerSec = (leftMps - rightMps) / EFFECTIVE_TRACK_WIDTH_M;
  minimumClearanceMm = minimumFanSweepClearanceMm();
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
    if (!footprintClear(x, y, heading)) {
      rejectReason = CANDIDATE_REJECT_FOOTPRINT;
      return false;
    }
    // A 400 mm passage is a straight-traverse problem, not a turning-space
    // problem. Once both nearby walls are evidenced, reject arcs that would
    // keep steering inside it. The planner therefore aligns before entry or
    // stops outside, where a safe turn/backtrack remains possible.
    if (isNarrowObservedCorridor(x, y, heading) &&
        fabs(turnTicks / max(1.0f, forwardTicks)) > PLANNER_CORRIDOR_MAX_TURN_RATIO) {
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
                            float closestGoalDistanceM,
                            float headingAtClosestGoalRad,
                            float finalX,
                            float finalY,
                            float finalHeadingRad,
                            float clearanceMm,
                            float escapeUrgency,
                            float desiredEscapeTurnSign,
                            bool lineFollowActive,
                            float routeStartX,
                            float routeStartY,
                            float routeHeadingRad,
                            float finalGoalDistanceM) {
  float startGoalDistanceM = sqrtf((localGoalX - robotX) * (localGoalX - robotX) +
                                   (localGoalY - robotY) * (localGoalY - robotY));
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
  float desiredHeadingDeg = atan2f(localGoalY - robotY, localGoalX - robotX) * 180.0 / PI;
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
  float smoothnessScore = 1.0 - min(1.0f, fabs(turnTicks - plannerTelemetry.selectedTurnTicksPerSec) /
                                          max(1.0f, baseTargetSpeed));
  float repeatedHeadingPenalty = 0.0;
  if (recentBlockedTurnDirection != 0 &&
      millis() - recentBlockedTurnMs < MAP_DYNAMIC_EXPIRY_MS &&
      ((turnTicks > 0.0 && recentBlockedTurnDirection > 0) ||
       (turnTicks < 0.0 && recentBlockedTurnDirection < 0))) {
    repeatedHeadingPenalty = 1.0;
  }
  float escapeTurnScore = 0.0;
  if (escapeUrgency > 0.0f && desiredEscapeTurnSign != 0.0f &&
      fabs(turnTicks) > 1.0f) {
    float turnSign = turnTicks > 0.0f ? 1.0f : -1.0f;
    float turnMagnitude = constrain(curvature / PLANNER_MAX_TURN_RATIO,
                                    0.0f, 1.0f);
    escapeTurnScore = escapeUrgency *
      (turnSign == desiredEscapeTurnSign ? 1.8f * turnMagnitude : -2.0f);
  }
  float curvaturePenaltyWeight = 0.8f * (1.0f - 0.75f * escapeUrgency);
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
         curvaturePenaltyWeight * curvature - nearGoalStraightPenalty +
         escapeTurnScore - repeatedHeadingPenalty;
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

static bool routeLineTrackingEligible(float &routeLengthM, float &routeUx,
                                      float &routeUy, float &routeHeadingRad) {
  if (!routeLineFrame(routeLengthM, routeUx, routeUy, routeHeadingRad)) {
    return false;
  }

  float routeHeadingDeg = routeHeadingRad * RAD_TO_DEG;
  return fabs(wrapAngle(routeHeadingDeg - navigationHeadingDeg())) <=
    PLANNER_LINE_FOLLOW_ENABLE_HEADING_DEG;
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

static float requestedPointGoalSpeedCap(bool routeLineActive,
                                        float routeLengthM, float routeUx,
                                        float routeUy) {
  float requestedCap = baseTargetSpeed;
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
  if (fabs(headingErrorDeg) < PLANNER_POINT_ALIGN_BEHIND_DEG) {
    return headingErrorDeg;
  }
  if (!isRangeSensorValid(RANGE_RIGHT_OUTER) ||
      !isRangeSensorValid(RANGE_RIGHT_INNER) ||
      !isRangeSensorValid(RANGE_LEFT_INNER) ||
      !isRangeSensorValid(RANGE_LEFT_OUTER)) {
    return headingErrorDeg;
  }

  float rightSideMm = min((float)getRangeSensorDistance(RANGE_RIGHT_OUTER),
                          (float)getRangeSensorDistance(RANGE_RIGHT_INNER));
  float leftSideMm = min((float)getRangeSensorDistance(RANGE_LEFT_INNER),
                         (float)getRangeSensorDistance(RANGE_LEFT_OUTER));
  if (rightSideMm > leftSideMm + PLANNER_POINT_ALIGN_SIDE_TIE_MM) {
    return -fabs(headingErrorDeg);
  }
  if (leftSideMm > rightSideMm + PLANNER_POINT_ALIGN_SIDE_TIE_MM) {
    return fabs(headingErrorDeg);
  }
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
  setMotionCommand(0.0, turnTarget);
  return true;
}

static bool selectTrajectory(float goalX, float goalY) {
  // This function is the heart of point navigation. It does not execute a
  // manoeuvre: it evaluates a small menu of constant arcs and publishes the
  // best command for MotorControl to hold until the next replan.
  float dx = goalX - robotX;
  float dy = goalY - robotY;
  float distanceM = sqrtf(dx * dx + dy * dy);
  float finalDx = navigationGoal.targetX - robotX;
  float finalDy = navigationGoal.targetY - robotY;
  float finalGoalDistanceM = sqrtf(finalDx * finalDx + finalDy * finalDy);
  bool finalWaypointIsLocalGoal =
    navigationGoal.mode == NAV_GOAL_POINT &&
    fabs(goalX - navigationGoal.targetX) < 0.001f &&
    fabs(goalY - navigationGoal.targetY) < 0.001f;
  float routeLengthM = 0.0f;
  float routeUx = 1.0f;
  float routeUy = 0.0f;
  float routeHeadingRad = 0.0f;
  bool lineFollowActive =
    !clearanceEscapeLocalGoalActive &&
    routeLineTrackingEligible(routeLengthM, routeUx, routeUy, routeHeadingRad);
  float requestedSpeedCap = requestedPointGoalSpeedCap(lineFollowActive,
                                                       routeLengthM,
                                                       routeUx,
                                                       routeUy);
  float speedCap = applyClearanceEscapeSpeedCap(
    calculateSpeedCapTicksPerSec(requestedSpeedCap));
  plannerTelemetry.speedCapTicksPerSec = speedCap;
  plannerTelemetry.localGoalDistanceM = distanceM;
  plannerTelemetry.candidateCount = 0;

  // These checks are early exits because no score can make an unobserved or
  // physically blocked forward path safe.
  if (!isRangeSensorValid(RANGE_FRONT)) {
    plannerTelemetry.stopReason = PLANNER_STOP_FRONT_INVALID;
    plannerTelemetry.safeStopReason = "front_sensor_invalid";
    plannerTelemetry.replanReason = "front_sensor_invalid";
    return false;
  }

  if (isRangeSensorBlocked(RANGE_FRONT)) {
    plannerTelemetry.stopReason = PLANNER_STOP_FRONT_BLOCKED;
    plannerTelemetry.safeStopReason = "front_blocked";
    plannerTelemetry.replanReason = "front_blocked";
    return false;
  }

  if (speedCap < PLANNER_MIN_DRIVABLE_SPEED_TPS) {
    plannerTelemetry.stopReason = isRangeSensorBlocked(RANGE_FRONT)
                                    ? PLANNER_STOP_FRONT_BLOCKED
                                    : PLANNER_STOP_NO_SAFE_TRAJECTORY;
    plannerTelemetry.safeStopReason = "speed_cap_below_drivable_min";
    plannerTelemetry.replanReason = "speed_cap_below_drivable_min";
    return false;
  }

  float bestScore = -1000000.0;
  float bestForward = 0.0;
  float bestTurn = 0.0;
  float bestClearance = -1.0;
  bool bestReachesGoal = false;
  float bestArrivalTimeS = 0.0;
  int rejectedTurnObservability = 0;
  int rejectedForwardObservation = 0;
  int rejectedFootprint = 0;
  int rejectedCorridor = 0;
  lastFootprintRejectWorldX = 0.0f;
  lastFootprintRejectWorldY = 0.0f;
  lastFootprintRejectCellX = -1;
  lastFootprintRejectCellY = -1;
  lastCorridorRejectLeftM = -1.0f;
  lastCorridorRejectRightM = -1.0f;
  float escapeUrgency = clearanceEscapeUrgency();
  float desiredEscapeTurnSign = 0.0f;
  if (clearanceEscapeLocalGoalActive) {
    float headingRad = navigationHeadingRad();
    float goalBodyY = -dx * sinf(headingRad) + dy * cosf(headingRad);
    if (goalBodyY < -0.02f) {
      desiredEscapeTurnSign = -1.0f;  // negative turn reduces navigation heading toward robot-right
    } else if (goalBodyY > 0.02f) {
      desiredEscapeTurnSign = 1.0f;   // positive turn increases navigation heading toward robot-left
    }
  }

  // The menu is intentionally small enough for a 40 ms Teensy cycle:
  // full/minimum drivable speed x a symmetric set of curvatures.
  for (int speedIndex = 0; speedIndex < PLANNER_SPEED_SAMPLES; speedIndex++) {
    float speedScale = speedIndex == 0 ? 1.0 : PLANNER_MIN_SPEED_SCALE;
    for (int curvatureIndex = 0; curvatureIndex < PLANNER_CURVATURE_SAMPLES; curvatureIndex++) {
      // normalized spans -1 (maximum left curve) through 0 (straight) to
      // +1 increases navigation heading; -1 decreases it. The sign convention
      // is measured from the current motor wiring/IMU frame.
      float normalized = -1.0 + (2.0 * curvatureIndex) / (PLANNER_CURVATURE_SAMPLES - 1);
      float forward = max(PLANNER_MIN_DRIVABLE_SPEED_TPS, speedCap * speedScale);
      float turn = forward * normalized * PLANNER_MAX_TURN_RATIO;
      forward = min(forward, 3000.0 / (1.0 + fabs(normalized * PLANNER_MAX_TURN_RATIO)));
      turn = forward * normalized * PLANNER_MAX_TURN_RATIO;
      if (lineFollowActive &&
          finalGoalDistanceM <= PLANNER_NEAR_GOAL_STRAIGHTEN_DISTANCE_M &&
          fabs(turn / max(1.0f, forward)) > PLANNER_LINE_FOLLOW_NEAR_GOAL_MAX_TURN_RATIO) {
        continue;
      }

      if (forward < PLANNER_MIN_DRIVABLE_SPEED_TPS) {
        continue;
      }

      float clearanceMm = -1.0;
      float closestGoalDistanceM = distanceM;
      float headingAtClosestGoalRad = navigationHeadingRad();
      float finalX = robotX;
      float finalY = robotY;
      float finalHeadingRad = navigationHeadingRad();
      float arrivalTimeS = -1.0;
      CandidateRejectReason rejectReason = CANDIDATE_REJECT_NONE;
      if (!rolloutCandidate(forward, turn, goalX, goalY, clearanceMm,
                            closestGoalDistanceM, headingAtClosestGoalRad,
                            finalX, finalY, finalHeadingRad, arrivalTimeS,
                            rejectReason)) {
        if (rejectReason == CANDIDATE_REJECT_TURN_OBSERVABILITY) {
          rejectedTurnObservability++;
        } else if (rejectReason == CANDIDATE_REJECT_FORWARD_OBSERVATION) {
          rejectedForwardObservation++;
        } else if (rejectReason == CANDIDATE_REJECT_FOOTPRINT) {
          rejectedFootprint++;
        } else if (rejectReason == CANDIDATE_REJECT_CORRIDOR) {
          rejectedCorridor++;
        }
        continue;
      }

      // Candidate count is useful in CSV: zero means every sampled command
      // failed a hard safety test; a low count means the map is constraining.
      plannerTelemetry.candidateCount++;
      float score = candidateScore(forward, turn, goalX, goalY,
                                   closestGoalDistanceM, headingAtClosestGoalRad,
                                   finalX, finalY, finalHeadingRad,
                                   clearanceMm, escapeUrgency,
                                   desiredEscapeTurnSign, lineFollowActive,
                                   navigationGoal.startX, navigationGoal.startY,
                                   routeHeadingRad, finalGoalDistanceM);
      bool reachesGoal = arrivalTimeS >= 0.0f;
      bool betterArrival = finalWaypointIsLocalGoal && !lineFollowActive && reachesGoal &&
                           (!bestReachesGoal || arrivalTimeS < bestArrivalTimeS - 0.0001f);
      bool equalArrivalClass = !finalWaypointIsLocalGoal || lineFollowActive ||
                               (reachesGoal == bestReachesGoal &&
                                (!reachesGoal || fabs(arrivalTimeS - bestArrivalTimeS) <= 0.0001f));
      if (betterArrival || (equalArrivalClass && score > bestScore)) {
        bestScore = score;
        bestForward = forward;
        bestTurn = turn;
        bestClearance = clearanceMm;
        bestReachesGoal = reachesGoal;
        bestArrivalTimeS = arrivalTimeS;
      }
    }
  }

  if (plannerTelemetry.candidateCount == 0) {
    float targetHeadingDeg = atan2f(goalY - robotY, goalX - robotX) * RAD_TO_DEG;
    float targetHeadingErrorDeg = fabs(wrapAngle(targetHeadingDeg - navigationHeadingDeg()));
    float observedForwardM = observedForwardDistanceForCandidate(0.0f);
    float leadingEnvelopeM = ROBOT_FOOTPRINT_GEOMETRY.frontExtentMm / 1000.0f +
                             PLANNER_TOTAL_HARD_CLEARANCE_M;
    bool straightSqueezeAllowed =
      rejectedFootprint > 0 &&
      rejectedForwardObservation == 0 &&
      rejectedCorridor == 0 &&
      targetHeadingErrorDeg <= PLANNER_CORRIDOR_SQUEEZE_HEADING_DEG &&
      observedForwardM >= leadingEnvelopeM +
                            min(distanceM, PLANNER_CORRIDOR_SQUEEZE_MIN_OBSERVED_M);
    if (straightSqueezeAllowed) {
      float squeezeForward = min(speedCap, PLANNER_CORRIDOR_SQUEEZE_SPEED_TPS);
      if (squeezeForward >= PLANNER_MIN_DRIVABLE_SPEED_TPS) {
        plannerTelemetry.selectedForwardTicksPerSec = squeezeForward;
        plannerTelemetry.selectedTurnTicksPerSec = 0.0;
        plannerTelemetry.selectedCurvature = 0.0;
        plannerTelemetry.minimumSweptClearanceMm = minimumFanSweepClearanceMm();
        plannerTelemetry.speedCapTicksPerSec = speedCap;
        plannerTelemetry.localGoalDistanceM = distanceM;
        plannerTelemetry.candidateCount = 1;
        plannerTelemetry.stopReason = PLANNER_STOP_NONE;
        plannerTelemetry.planReason = "corridor_squeeze_straight";
        plannerTelemetry.replanReason = "footprint_side_squeeze";
        plannerTelemetry.safeStopReason = "";
        lastReportedStopReason = PLANNER_STOP_NONE;
        candidateRejectsReported = false;
        motorStopRequested = false;
        setMotionCommand(squeezeForward, 0.0);
        return true;
      }
    }

    if (!candidateRejectsReported) {
      char detail[192];
      snprintf(detail, sizeof(detail),
               "turn=%d;observed=%d;footprint=%d;corridor=%d;fp=%.3f/%.3f@%d/%d;corr=%.2f/%.2f",
               rejectedTurnObservability, rejectedForwardObservation,
               rejectedFootprint, rejectedCorridor,
               lastFootprintRejectWorldX, lastFootprintRejectWorldY,
               lastFootprintRejectCellX, lastFootprintRejectCellY,
               lastCorridorRejectLeftM, lastCorridorRejectRightM);
      sendBluetoothEvent("planner_candidate_rejects", detail);
      candidateRejectsReported = true;
    }
    plannerTelemetry.stopReason = PLANNER_STOP_NO_SAFE_TRAJECTORY;
    plannerTelemetry.safeStopReason = "no_footprint_safe_arc";
    plannerTelemetry.replanReason = "all_arc_candidates_rejected";
    return false;
  }

  // Publish one chassis command. MotorControl owns the actual servo pulses and
  // will turn (forward, turn) into individual left/right wheel targets.
  plannerTelemetry.selectedForwardTicksPerSec = bestForward;
  plannerTelemetry.selectedTurnTicksPerSec = bestTurn;
  plannerTelemetry.selectedCurvature = bestTurn / max(1.0f, bestForward);
  plannerTelemetry.minimumSweptClearanceMm = bestClearance;
  plannerTelemetry.stopReason = PLANNER_STOP_NONE;
  plannerTelemetry.planReason = "best_safe_arc";
  plannerTelemetry.replanReason = "local_goal_visible";
  plannerTelemetry.safeStopReason = "";
  lastReportedStopReason = PLANNER_STOP_NONE;
  candidateRejectsReported = false;
  motorStopRequested = false;
  setMotionCommand(bestForward, bestTurn);
  return true;
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

static bool canStartReverseRecovery() {
  return navigationGoal.mode == NAV_GOAL_POINT &&
         escapeBacktrackEnabled &&
         !reverseRecoveryActive &&
         plannerTelemetry.stopReason != PLANNER_STOP_FRONT_INVALID &&
         isRangeSensorValid(RANGE_FAKE_REAR);
}

static void startReverseRecovery() {
  reverseRecoveryActive = true;
  reverseRecoveryStartX = robotX;
  reverseRecoveryStartY = robotY;
  reverseRecoveryStartedMs = millis();
  reverseRecoveryStepCount = 0;
  reverseRecoveryRejectsReported = false;
  noSafeTrajectorySinceMs = 0;
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
  reverseRecoveryRejectsReported = false;
  sendBluetoothEvent("reverse_recovery_end", detail);
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

static bool rolloutReverseRecoveryCandidate(float reverseTicks, float turnTicks,
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
  if (!isRangeSensorValid(RANGE_FAKE_REAR) ||
      isRangeSensorBlocked(RANGE_FAKE_REAR)) {
    rejectReason = CANDIDATE_REJECT_REAR_OBSERVATION;
    return false;
  }

  float observedRearM = getRangeSensorDistance(RANGE_FAKE_REAR) / 1000.0f;
  float trailingEnvelopeM = ROBOT_FOOTPRINT_GEOMETRY.rearExtentMm / 1000.0f +
                            PLANNER_TOTAL_HARD_CLEARANCE_M +
                            PLANNER_REVERSE_RECOVERY_REAR_BUFFER_M;
  float heading = navigationHeadingRad();
  float x = robotX;
  float y = robotY;
  float leftMps = (reverseTicks + turnTicks) / TICKS_PER_METRE;
  float rightMps = (reverseTicks - turnTicks) / TICKS_PER_METRE;
  float linearMps = (leftMps + rightMps) * 0.5f;
  float angularRadPerSec = (leftMps - rightMps) / EFFECTIVE_TRACK_WIDTH_M;
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
    if (!footprintClear(x, y, heading)) {
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
                                  float rearClearanceMm,
                                  float finalGoalDistanceM,
                                  float finalHeadingErrorDeg,
                                  float finalRouteLateralErrorM) {
  float startGoalDistanceM = sqrtf((navigationGoal.targetX - robotX) *
                                   (navigationGoal.targetX - robotX) +
                                   (navigationGoal.targetY - robotY) *
                                   (navigationGoal.targetY - robotY));
  float progressScore = constrain((startGoalDistanceM - finalGoalDistanceM) /
                                  max(PLANNER_MIN_PROGRESS_M, startGoalDistanceM),
                                  -1.0f, 1.0f);
  float headingScore = 1.0f - min(1.0f, finalHeadingErrorDeg / 120.0f);
  float clearanceScore = constrain(rearClearanceMm /
                                   (PLANNER_PREFERRED_CLEARANCE_M * 1000.0f),
                                   0.0f, 1.0f);
  float curvature = fabs(turnTicks) / max(1.0f, fabs(reverseTicks));
  float smoothnessScore = 1.0f - min(1.0f, fabs(turnTicks - plannerTelemetry.selectedTurnTicksPerSec) /
                                          max(1.0f, baseTargetSpeed));
  float routeScore = 1.0f - min(1.0f, finalRouteLateralErrorM /
                                      PLANNER_LINE_FOLLOW_LATERAL_TOLERANCE_M);
  return 1.6f * headingScore + 1.4f * clearanceScore +
         0.8f * smoothnessScore + 0.8f * routeScore +
         0.5f * progressScore - 0.9f * curvature;
}

static bool selectReverseRecoveryTrajectory(float goalX, float goalY) {
  float speedCap = calculateReverseRecoverySpeedCapTicksPerSec();
  plannerTelemetry.speedCapTicksPerSec = speedCap;
  plannerTelemetry.localGoalDistanceM = sqrtf((goalX - robotX) * (goalX - robotX) +
                                              (goalY - robotY) * (goalY - robotY));
  plannerTelemetry.candidateCount = 0;

  if (speedCap < PLANNER_REVERSE_RECOVERY_MIN_SPEED_TPS) {
    plannerTelemetry.stopReason = PLANNER_STOP_NO_SAFE_TRAJECTORY;
    plannerTelemetry.safeStopReason = "rear_path_unavailable";
    plannerTelemetry.replanReason = "fake_rear_tof_unavailable";
    return false;
  }

  float bestScore = -1000000.0f;
  float bestReverse = 0.0f;
  float bestTurn = 0.0f;
  float bestRearClearance = -1.0f;
  int rejectedRear = 0;
  int rejectedFootprint = 0;

  for (int speedIndex = 0; speedIndex < 2; speedIndex++) {
    float speedScale = speedIndex == 0 ? 1.0f : PLANNER_REVERSE_RECOVERY_MIN_SPEED_SCALE;
    float reverseMagnitude = max(PLANNER_REVERSE_RECOVERY_MIN_SPEED_TPS,
                                 speedCap * speedScale);
    reverseMagnitude = min(reverseMagnitude, PLANNER_REVERSE_RECOVERY_MAX_SPEED_TPS);
    for (int curvatureIndex = 0;
         curvatureIndex < PLANNER_REVERSE_RECOVERY_CURVATURE_SAMPLES;
         curvatureIndex++) {
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
      if (!rolloutReverseRecoveryCandidate(reverseTicks, turnTicks, goalX, goalY,
                                           rearClearanceMm, finalGoalDistanceM,
                                           finalHeadingErrorDeg,
                                           finalRouteLateralErrorM,
                                           rejectReason)) {
        if (rejectReason == CANDIDATE_REJECT_REAR_OBSERVATION) {
          rejectedRear++;
        } else if (rejectReason == CANDIDATE_REJECT_FOOTPRINT) {
          rejectedFootprint++;
        }
        continue;
      }

      plannerTelemetry.candidateCount++;
      float score = reverseRecoveryScore(reverseTicks, turnTicks,
                                         rearClearanceMm, finalGoalDistanceM,
                                         finalHeadingErrorDeg,
                                         finalRouteLateralErrorM);
      if (score > bestScore) {
        bestScore = score;
        bestReverse = reverseTicks;
        bestTurn = turnTicks;
        bestRearClearance = rearClearanceMm;
      }
    }
  }

  if (plannerTelemetry.candidateCount == 0) {
    plannerTelemetry.stopReason = PLANNER_STOP_NO_SAFE_TRAJECTORY;
    plannerTelemetry.safeStopReason = "no_reverse_recovery_arc";
    plannerTelemetry.replanReason = "no_reverse_arc";
    if (!reverseRecoveryRejectsReported) {
      char detail[64];
      snprintf(detail, sizeof(detail), "rear=%d;footprint=%d",
               rejectedRear, rejectedFootprint);
      sendBluetoothEvent("reverse_recovery_rejects", detail);
      reverseRecoveryRejectsReported = true;
    }
    return false;
  }

  reverseRecoveryStepCount++;
  plannerTelemetry.selectedForwardTicksPerSec = bestReverse;
  plannerTelemetry.selectedTurnTicksPerSec = bestTurn;
  plannerTelemetry.selectedCurvature = bestTurn / max(1.0f, fabs(bestReverse));
  plannerTelemetry.minimumSweptClearanceMm = bestRearClearance;
  plannerTelemetry.stopReason = PLANNER_STOP_NONE;
  plannerTelemetry.planReason = "reverse_recovery_arc";
  plannerTelemetry.replanReason = "no_forward_path";
  plannerTelemetry.safeStopReason = "";
  lastReportedStopReason = PLANNER_STOP_NONE;
  reverseRecoveryRejectsReported = false;
  motorStopRequested = false;
  setMotionCommand(bestReverse, bestTurn);
  if (reverseRecoveryStepCount == 1 ||
      (reverseRecoveryStepCount % 20) == 0) {
    sendBluetoothEvent("reverse_recovery_step", "arc_selected");
  }
  return true;
}

static bool buildClearanceEscapeLocalGoal(float &localGoalX, float &localGoalY) {
  // A single nearby inner ray and an open opposite inner ray describe an
  // observable side exit. Give the arc sampler that exit as its temporary
  // local goal instead of continuing to score only progress along the blocked
  // global target line. The normal footprint/map checks still decide whether
  // any candidate is safe.
  if (!isRangeSensorValid(RANGE_RIGHT_INNER) ||
      !isRangeSensorValid(RANGE_LEFT_INNER)) {
    return false;
  }

  float rightInnerM = getRangeSensorDistance(RANGE_RIGHT_INNER) / 1000.0f;
  float leftInnerM = getRangeSensorDistance(RANGE_LEFT_INNER) / 1000.0f;
  float restrictedM = min(rightInnerM, leftInnerM);
  float escapeTriggerM = WAYPOINT_LOOKAHEAD_M +
    ROBOT_FOOTPRINT_GEOMETRY.frontExtentMm / 1000.0f +
    PLANNER_PREFERRED_CLEARANCE_M;
  if (restrictedM >= escapeTriggerM || fabs(rightInnerM - leftInnerM) < LOCAL_MAP_CELL_M) {
    return false;
  }

  if (!isRangeSensorValid(RANGE_RIGHT_OUTER) ||
      !isRangeSensorValid(RANGE_LEFT_OUTER)) {
    return false;
  }

  float rightOuterM = getRangeSensorDistance(RANGE_RIGHT_OUTER) / 1000.0f;
  float leftOuterM = getRangeSensorDistance(RANGE_LEFT_OUTER) / 1000.0f;
  float rightSideM = min(rightInnerM, rightOuterM);
  float leftSideM = min(leftInnerM, leftOuterM);
  if (fabs(rightSideM - leftSideM) < LOCAL_MAP_CELL_M) {
    return false;
  }

  // Robot +Y is left. An open right side must therefore create a negative-Y
  // local target (and vice versa). Use both inner and outer rays so a close
  // outer wall cannot be hidden by an open inner return.
  float sideSign = rightSideM > leftSideM ? -1.0f : 1.0f;
  float lateralOffsetM = (sideSign > 0.0f
    ? ROBOT_FOOTPRINT_GEOMETRY.leftExtentMm
    : ROBOT_FOOTPRINT_GEOMETRY.rightExtentMm) / 1000.0f +
    PLANNER_PREFERRED_CLEARANCE_M;
  float headingRad = navigationHeadingRad();
  localGoalX = robotX + WAYPOINT_LOOKAHEAD_M * cosf(headingRad) -
               sideSign * lateralOffsetM * sinf(headingRad);
  localGoalY = robotY + WAYPOINT_LOOKAHEAD_M * sinf(headingRad) +
               sideSign * lateralOffsetM * cosf(headingRad);
  plannerTelemetry.replanReason = "clearance_escape_waypoint";
  return true;
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

  // Object pickup is complete by position, not by final heading. Once the
  // robot has driven through the hunt pickup zone, stop before the generic
  // point-goal alignment cleanup can rotate the chassis away from the pickup.
  if (routeFrameValid && huntPickupZoneReached(routeLengthM, routeUx, routeUy)) {
    finishNavigationGoal(true, PLANNER_STOP_NONE, "hunt_pickup_zone_reached");
    return;
  }

  bool huntCarryThroughActive =
    routeLineEligible && huntPickupCarryThroughActive(routeLengthM, routeUx, routeUy);
  float targetHeadingDeg = atan2f(dy, dx) * RAD_TO_DEG;
  float targetHeadingErrorDeg = wrapAngle(targetHeadingDeg - navigationHeadingDeg());
  if (!reverseRecoveryActive &&
      !huntCarryThroughActive &&
      fabs(targetHeadingErrorDeg) > PLANNER_POINT_ALIGN_START_DEG) {
    commandPointAlignmentTurn(targetHeadingErrorDeg);
    return;
  }
  pointAlignTurnActive = false;
  pointAlignTurnDirection = 0.0;

  // Long routes are planned toward a nearby point on the target line. This
  // keeps the local map relevant and lets every 40 ms replan react to new
  // obstacle evidence without needing a global A* route. When the point is
  // mostly ahead, track the original start->target route line rather than
  // chasing the final coordinate from the current drifted pose. This turns
  // ordinary physical sideways error into a controlled line-follow problem.
  float localGoalX = 0.0f;
  float localGoalY = 0.0f;
  if (routeLineEligible) {
    buildRouteLineLocalGoal(routeLengthM, routeUx, routeUy, localGoalX, localGoalY);
  } else {
    float lookaheadM = min(distanceM, WAYPOINT_LOOKAHEAD_M);
    localGoalX = robotX + (dx / distanceM) * lookaheadM;
    localGoalY = robotY + (dy / distanceM) * lookaheadM;
  }

  clearanceEscapeLocalGoalActive = buildClearanceEscapeLocalGoal(localGoalX, localGoalY);

  // For mostly-forward point goals, the useful physical task is crossing the
  // target plane while staying close to the requested route. A strict 60 mm
  // centre-point circle is retained above, but it is too brittle as the only
  // success condition once the drivetrain drifts a little at speed.
  if (!clearanceEscapeLocalGoalActive && routeLineEligible &&
      routeLineGoalReached(routeLengthM, routeUx, routeUy, routeHeadingRad)) {
    finishNavigationGoal(true, PLANNER_STOP_NONE, "route_line_reached");
    return;
  }
  if (!clearanceEscapeLocalGoalActive && routeLineEligible &&
      routeLineOvershootStopReached(routeLengthM, routeUx, routeUy, routeHeadingRad)) {
    finishNavigationGoal(true, PLANNER_STOP_NONE, "route_line_overshoot_reached");
    return;
  }
  if (!clearanceEscapeLocalGoalActive && routeLineEligible &&
      routeLineClearlyMissed(routeLengthM, routeUx, routeUy)) {
    finishNavigationGoal(false, PLANNER_STOP_ABORTED, "route_line_missed");
    return;
  }

  if (driveStuck || wheelMismatchStuck) {
    plannerTelemetry.replanReason = "wheel_progress_stalled";
    finishNavigationGoal(false, PLANNER_STOP_STUCK, "drive_progress_stalled");
    return;
  }

  if (selectTrajectory(localGoalX, localGoalY)) {
    if (reverseRecoveryActive) {
      endReverseRecovery("forward_path_found");
    }
    // Any safe forward arc breaks the no-path streak. The recovery condition
    // is therefore genuinely "no forward path exists", not merely "an
    // obstacle was briefly visible".
    noSafeTrajectorySinceMs = 0;
    return;
  }

  if (reverseRecoveryActive) {
    if (!selectReverseRecoveryTrajectory(localGoalX, localGoalY)) {
      motorStopRequested = true;
      setMotionCommand(PLANNER_DEFAULT_SAFE_STOP_SPEED_MPS, 0.0);
      reportPlannerStopIfChanged();
    }
    return;
  }

  // A first failure is a safe stop. The debounce filters a one-frame
  // map/sensor disagreement before reverse recovery becomes eligible.
  motorStopRequested = true;
  setMotionCommand(PLANNER_DEFAULT_SAFE_STOP_SPEED_MPS, 0.0);
  reportPlannerStopIfChanged();
  unsigned long now = millis();
  if (noSafeTrajectorySinceMs == 0) {
    noSafeTrajectorySinceMs = now;
  }
  if (now - noSafeTrajectorySinceMs >= PLANNER_NO_PATH_BACKTRACK_DELAY_MS &&
      canStartReverseRecovery()) {
    startReverseRecovery();
    if (!selectReverseRecoveryTrajectory(localGoalX, localGoalY)) {
      motorStopRequested = true;
      setMotionCommand(PLANNER_DEFAULT_SAFE_STOP_SPEED_MPS, 0.0);
      reportPlannerStopIfChanged();
    }
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
      setMotionCommand(0.0, brakeTarget);
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
        ? PLANNER_TURN_RIGHT_BRAKE_PULSE_MS
        : PLANNER_TURN_LEFT_BRAKE_PULSE_MS;
      turnBrakeActive = true;
      turnBrakeUntilMs = millis() + brakePulseMs;
      motorStopRequested = false;
      setMotionCommand(0.0, brakeTarget);
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
  setMotionCommand(0.0, turnTarget);
}

void updateNavigationController() {
  if (!navigationGoal.active) {
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
  if (now - lastPlannerUpdateMs < updateIntervalMs) {
    return;
  }
  lastPlannerUpdateMs = now;
  plannerTelemetry.lastPlanMs = now;
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
  resetEncodersAndPID();
  plannerTelemetry.planReason = "initialised";
}

void updateRobotController() {
  // This is the timing backbone of autonomous motion. Its order is deliberate:
  // sense/map first, take immediate safety action second, update odometry,
  // select a new command if safe, then let the sole motor writer apply it.
  unsigned long now = millis();
  bool immediateSafetyStop = false;
  if (now - lastSensorUpdateMs >= SENSOR_UPDATE_INTERVAL_MS) {
    lastSensorUpdateMs = now;
    updateTOFSensors();
    updateLocalMapFromSensors();

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

    // Reverse recovery owns rearward motion while no forward arc exists. Front
    // blocked evidence remains the trigger for recovery, but it must not
    // cancel the trusted rear-arc command that is moving the chassis out.
    if (!immediateSafetyStop && navigationGoal.active && !reverseRecoveryActive &&
        isRangeSensorBlocked(RANGE_FRONT)) {
      // Remember the side that looked tighter. If that same turn direction is
      // sampled later, candidateScore penalises it to prevent repeated retries
      // into the obstruction.
      recentBlockedTurnDirection =
        getRangeSensorDistance(RANGE_RIGHT_INNER) <= getRangeSensorDistance(RANGE_LEFT_INNER)
          ? 1 : -1;
      recentBlockedTurnMs = now;
      immediateSafetyStop = true;
      motorStopRequested = true;
      setMotionCommand(0.0, 0.0);
      plannerTelemetry.stopReason = PLANNER_STOP_FRONT_BLOCKED;
      plannerTelemetry.safeStopReason = "front_blocked";
      plannerTelemetry.replanReason = "front_blocked";
      reportPlannerStopIfChanged();
      if (navigationGoal.mode == NAV_GOAL_POINT) {
        // Keep the emergency motor stop, but let point-goal planning observe
        // the persistent blocked-front state. selectTrajectory() still
        // rejects all forward arcs while front_blocked is true; after the
        // no-path debounce this can enter reverse-arc recovery
        // instead of freezing forever at the first blocked reading.
        immediateSafetyStop = false;
      } else if (navigationGoal.mode == NAV_GOAL_TURN) {
        cancelNavigationGoal(PLANNER_STOP_FRONT_BLOCKED, "front_blocked_during_turn");
      }
    }

    RangeSensorId diagonalSensor;
    float diagonalClearanceMm = 0.0;
    if (!immediateSafetyStop && navigationGoal.active && !reverseRecoveryActive &&
        getDiagonalClearanceWarning(diagonalSensor, diagonalClearanceMm)) {
      // This is the fast, current-pose guard. It uses chassis-footprint
      // clearance rather than turn radius, so it can stop an imminent clip
      // without wrongly forbidding a pre-aligned narrow straight passage.
      recentBlockedTurnDirection =
        FAN_SENSOR_GEOMETRY[(int)diagonalSensor].angleDeg < 0.0 ? 1 : -1;
      recentBlockedTurnMs = now;
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
    updateOdometry();
    // Update pose before marking the footprint; otherwise the free patch would
    // lag behind the physical chassis by one odometry period.
    markTraversedFreeSpace();
  }

  if (!immediateSafetyStop) {
    // An immediate safety stop owns this cycle. The planner must not overwrite
    // it with a fresh arc until another controller pass has seen new evidence.
    updateNavigationController();
  }
  // No other file writes servo pulses during normal navigation.
  updateMotorController();
  sendBluetoothTelemetry();
}
