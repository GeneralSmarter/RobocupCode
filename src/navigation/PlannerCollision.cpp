#include "../../Robot.h"
#include "PlannerContext.h"
#include "PlannerCollision.h"

static int fastFloorToInt(float value) {
  int truncated = (int)value;
  return value < (float)truncated ? truncated - 1 : truncated;
}

bool snapshotWorldOccupied(const PlannerCollisionSnapshot &snapshot,
                                  float worldX, float worldY,
                                  int *cellX, int *cellY) {
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

bool footprintKnownClearOnSnapshot(
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

float footprintUnknownFractionOnSnapshot(
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

float currentReverseUnknownAllowance() {
  if (!plannerContext.reverseRecoveryActive || plannerContext.reverseRecoveryStartedMs == 0) {
    return 0.0f;
  }
  float ramp = constrain(
    (millis() - plannerContext.reverseRecoveryStartedMs) /
      (float)PLANNER_REVERSE_UNKNOWN_RAMP_MS,
    0.0f, 1.0f);
  return ramp * PLANNER_REVERSE_MAX_UNKNOWN_FRACTION;
}

float poseKnownClearanceM(const PlannerCollisionSnapshot &snapshot,
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

float rotationalEnvelopeClearanceM(
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

float projectedUnexploredScore(
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

bool footprintClearOnSnapshot(const PlannerCollisionSnapshot &snapshot,
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
        plannerContext.lastFootprintRejectWorldX = cellWorldX;
        plannerContext.lastFootprintRejectWorldY = cellWorldY;
        plannerContext.lastFootprintRejectCellX = cellX;
        plannerContext.lastFootprintRejectCellY = cellY;
        return false;
      }
    }
  }
  return true;
}

bool turnSegmentFootprintClear(
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

bool isNarrowObservedCorridorOnSnapshot(
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
    plannerContext.lastCorridorRejectLeftM = leftBoundaryM;
    plannerContext.lastCorridorRejectRightM = rightBoundaryM;
  }
  return narrow;
}

float minimumFanSweepClearanceMm() {
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
  // The matrix is supplemental: an unavailable frame cannot establish clear
  // space, but a current close obstacle cell is an additional pivot veto.
  if (isRangeSensorCurrent(RANGE_FRONT_MATRIX_AGGREGATE) &&
      isRangeSensorBlocked(RANGE_FRONT_MATRIX_AGGREGATE)) {
    return false;
  }
  return true;
}
