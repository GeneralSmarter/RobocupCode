#include "../../Robot.h"
#include "PlannerContext.h"
#include "PlannerMap.h"


static float clampEvidence(float value) {
  return constrain(value, -120.0, 120.0);
}

enum ArenaMemoryState {
  ARENA_MEMORY_UNKNOWN,
  ARENA_MEMORY_CLEAR,
  ARENA_MEMORY_OCCUPIED
};

static void initialiseArenaMemoryAtRobot() {
  plannerContext.arenaMemoryOriginX = robotX - ARENA_MEMORY_SIZE_M * 0.5f;
  plannerContext.arenaMemoryOriginY = robotY - ARENA_MEMORY_SIZE_M * 0.5f;
  memset(plannerContext.arenaMemoryChallenge, 0, sizeof(plannerContext.arenaMemoryChallenge));
  memset(plannerContext.arenaMemoryOccupied, 0, sizeof(plannerContext.arenaMemoryOccupied));
  memset(plannerContext.arenaMemoryKnownClear, 0, sizeof(plannerContext.arenaMemoryKnownClear));
  plannerContext.arenaMemoryInitialized = true;
}

static bool arenaWorldToCell(float worldX, float worldY,
                             int &cellX, int &cellY) {
  if (!plannerContext.arenaMemoryInitialized) {
    return false;
  }
  cellX = (int)floorf((worldX - plannerContext.arenaMemoryOriginX) / LOCAL_MAP_CELL_M);
  cellY = (int)floorf((worldY - plannerContext.arenaMemoryOriginY) / LOCAL_MAP_CELL_M);
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
  if ((plannerContext.arenaMemoryOccupied[bit >> 3] & mask) != 0) {
    return ARENA_MEMORY_OCCUPIED;
  }
  return (plannerContext.arenaMemoryKnownClear[bit >> 3] & mask) != 0
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
  plannerContext.arenaMemoryOccupied[bit >> 3] &= (uint8_t)~mask;
  plannerContext.arenaMemoryKnownClear[bit >> 3] &= (uint8_t)~mask;
  if (state == ARENA_MEMORY_OCCUPIED) {
    plannerContext.arenaMemoryOccupied[bit >> 3] |= mask;
  } else if (state == ARENA_MEMORY_CLEAR) {
    plannerContext.arenaMemoryKnownClear[bit >> 3] |= mask;
  }
  plannerContext.arenaMemoryChallenge[bit] = 0;
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
  int challenge = plannerContext.arenaMemoryChallenge[bit];
  if (amount > 0) {
    if (state == ARENA_MEMORY_OCCUPIED) {
      plannerContext.arenaMemoryChallenge[bit] = (int8_t)min(0, challenge + amount);
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
      plannerContext.arenaMemoryChallenge[bit] = (int8_t)max(0, challenge + amount);
      return;
    }
    challenge = challenge > 0 ? 0 : challenge;
    challenge = max(-120, challenge + amount);
    if (challenge <= -ARENA_MEMORY_EVIDENCE_THRESHOLD) {
      setArenaMemoryState(cellX, cellY, ARENA_MEMORY_CLEAR);
      return;
    }
  }
  plannerContext.arenaMemoryChallenge[bit] = (int8_t)challenge;
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
  plannerContext.localMapOriginX = robotX - LOCAL_MAP_SIZE_M * 0.5;
  plannerContext.localMapOriginY = robotY - LOCAL_MAP_SIZE_M * 0.5;
  memset(plannerContext.localMap, 0, sizeof(plannerContext.localMap));
  plannerContext.localMapInitialized = true;
}

bool plannerMapWorldToCell(float worldX, float worldY, int &cellX, int &cellY) {
  if (!plannerContext.localMapInitialized) {
    return false;
  }

  // Every map operation uses this one conversion. Keeping it here prevents
  // subtle disagreements about rounding at cell boundaries.
  cellX = (int)floorf((worldX - plannerContext.localMapOriginX) / LOCAL_MAP_CELL_M);
  cellY = (int)floorf((worldY - plannerContext.localMapOriginY) / LOCAL_MAP_CELL_M);
  return cellX >= 0 && cellX < LOCAL_MAP_CELLS &&
         cellY >= 0 && cellY < LOCAL_MAP_CELLS;
}

static void recenterLocalMapIfNeeded() {
  if (!plannerContext.localMapInitialized) {
    initialiseMapAtRobot();
    return;
  }

  float centreX = plannerContext.localMapOriginX + LOCAL_MAP_SIZE_M * 0.5;
  float centreY = plannerContext.localMapOriginY + LOCAL_MAP_SIZE_M * 0.5;
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

  memset(plannerContext.shiftedMap, 0, sizeof(plannerContext.shiftedMap));
  for (int y = 0; y < LOCAL_MAP_CELLS; y++) {
    for (int x = 0; x < LOCAL_MAP_CELLS; x++) {
      int destinationX = x - shiftX;
      int destinationY = y - shiftY;
      if (destinationX >= 0 && destinationX < LOCAL_MAP_CELLS &&
          destinationY >= 0 && destinationY < LOCAL_MAP_CELLS) {
        plannerContext.shiftedMap[destinationY][destinationX] = plannerContext.localMap[y][x];
      }
    }
  }

  memcpy(plannerContext.localMap, plannerContext.shiftedMap, sizeof(plannerContext.localMap));
  plannerContext.localMapOriginX += shiftX * LOCAL_MAP_CELL_M;
  plannerContext.localMapOriginY += shiftY * LOCAL_MAP_CELL_M;
}

static void decayLocalMap() {
  // The map is intentionally short-lived. This avoids pretending that
  // encoder-only odometry can support a permanent arena-scale world model.
  unsigned long now = millis();
  for (int y = 0; y < LOCAL_MAP_CELLS; y++) {
    for (int x = 0; x < LOCAL_MAP_CELLS; x++) {
      LocalMapCell &cell = plannerContext.localMap[y][x];
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
  if (!plannerMapWorldToCell(worldX, worldY, x, y)) {
    return;
  }

  LocalMapCell &cell = plannerContext.localMap[y][x];
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

float plannerMapFanForwardObservationDistanceM(RangeSensorId id) {
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
  if (!plannerMapWorldToCell(worldX, worldY, centreX, centreY)) {
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
        LocalMapCell &cell = plannerContext.localMap[y][x];
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
        float cellWorldX = plannerContext.localMapOriginX + (x + 0.5f) * LOCAL_MAP_CELL_M;
        float cellWorldY = plannerContext.localMapOriginY + (y + 0.5f) * LOCAL_MAP_CELL_M;
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
  // therefore needs repeated observations before plannerMapCellOccupied() rejects them.
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

float plannerNavigationHeadingRad() {
  return navigationHeadingDeg() * DEG_TO_RAD;
}

static void updateRearObstacleMapEvidence(float headingRad) {
  const uint32_t frameSequence = getRearObstacleFrameSequence();
  if (frameSequence == 0 || frameSequence == plannerContext.lastRearEvidenceFrameSequence) {
    return;
  }
  plannerContext.lastRearEvidenceFrameSequence = frameSequence;

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
  plannerContext.lastRearEvidenceFrameSequence = 0;
  plannerTelemetry.replanReason = "map_cleared";
}

void updateLocalMapFromSensors() {
  // This is perception, not planning. It converts the latest forward fan and
  // rear matrix rays into short-lived evidence before planning asks whether
  // arcs are safe.
  recenterLocalMapIfNeeded();
  decayLocalMap();

  const float headingRad = plannerNavigationHeadingRad();
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
  const float headingRad = plannerNavigationHeadingRad();
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
      if (plannerMapWorldToCell(worldX, worldY, x, y)) {
        LocalMapCell &cell = plannerContext.localMap[y][x];
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

bool plannerMapCellOccupied(int cellX, int cellY) {
  if (cellX < 0 || cellX >= LOCAL_MAP_CELLS || cellY < 0 || cellY >= LOCAL_MAP_CELLS) {
    return true;
  }

  const LocalMapCell &cell = plannerContext.localMap[cellY][cellX];
  // Free evidence is intentionally not a hard permission. A cell is safe only
  // because no sufficiently strong obstacle evidence currently contradicts it.
  float worldX = plannerContext.localMapOriginX + (cellX + 0.5f) * LOCAL_MAP_CELL_M;
  float worldY = plannerContext.localMapOriginY + (cellY + 0.5f) * LOCAL_MAP_CELL_M;
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
  if (!plannerMapWorldToCell(worldX, worldY, cellX, cellY)) {
    if (arenaState == ARENA_MEMORY_OCCUPIED) return 2;
    if (arenaState == ARENA_MEMORY_CLEAR) return 1;
    return 0;
  }
  if (plannerMapCellOccupied(cellX, cellY)) return 2;
  return plannerContext.localMap[cellY][cellX].freeEvidence >=
           PLANNER_REVERSE_CLEAR_EVIDENCE_THRESHOLD ||
         arenaState == ARENA_MEMORY_CLEAR
    ? 1 : 0;
}

int plannerDebugSeedMapOccupied(float worldX, float worldY) {
  int cellX;
  int cellY;
  if (!plannerMapWorldToCell(worldX, worldY, cellX, cellY)) {
    return 0;
  }
  LocalMapCell &cell = plannerContext.localMap[cellY][cellX];
  cell.dynamicEvidence = PLANNER_OBSTACLE_SCORE_THRESHOLD;
  cell.staticEvidence = PLANNER_OBSTACLE_SCORE_THRESHOLD;
  cell.freeEvidence = 0;
  cell.lastObservedMs = millis();
  addArenaMemoryEvidence(worldX, worldY, ARENA_MEMORY_EVIDENCE_THRESHOLD);
  return 1;
}

void plannerMapCaptureCollisionSnapshot(PlannerCollisionSnapshot &snapshot) {
  snapshot.originX = plannerContext.localMapOriginX;
  snapshot.originY = plannerContext.localMapOriginY;
  memset(snapshot.occupied, 0, sizeof(snapshot.occupied));
  memset(snapshot.knownClear, 0, sizeof(snapshot.knownClear));
  for (int y = 0; y < LOCAL_MAP_CELLS; y++) {
    for (int x = 0; x < LOCAL_MAP_CELLS; x++) {
      int bit = y * LOCAL_MAP_CELLS + x;
      if (plannerMapCellOccupied(x, y)) {
        snapshot.occupied[bit >> 3] |= (uint8_t)(1U << (bit & 7));
      } else {
        float worldX = plannerContext.localMapOriginX + (x + 0.5f) * LOCAL_MAP_CELL_M;
        float worldY = plannerContext.localMapOriginY + (y + 0.5f) * LOCAL_MAP_CELL_M;
        if (plannerContext.localMap[y][x].freeEvidence >=
              PLANNER_REVERSE_CLEAR_EVIDENCE_THRESHOLD ||
            arenaMemoryStateAtWorld(worldX, worldY) == ARENA_MEMORY_CLEAR) {
          snapshot.knownClear[bit >> 3] |= (uint8_t)(1U << (bit & 7));
        }
      }
    }
  }
}
void plannerMapCellCenter(int cellX, int cellY,
                          float &worldX, float &worldY) {
  worldX = plannerContext.localMapOriginX + (cellX + 0.5f) * LOCAL_MAP_CELL_M;
  worldY = plannerContext.localMapOriginY + (cellY + 0.5f) * LOCAL_MAP_CELL_M;
}
