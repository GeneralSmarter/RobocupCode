#ifndef REAR_OBSTACLE_POLICY_H
#define REAR_OBSTACLE_POLICY_H

#include <stddef.h>
#include <stdint.h>
#include <math.h>

struct RearObstacleFrameSummary {
  uint16_t minimumDistanceMm;
  uint8_t validCellCount;
};

struct RearObstacleHysteresisState {
  bool blocked;
  uint8_t consecutiveClearFrames;
};

constexpr uint8_t rearObstacleRawIndexForLogical(uint8_t row, uint8_t col) {
  return (uint8_t)((7U - row) * 8U + (7U - col));
}

constexpr bool isRearObstacleCellValid(uint16_t distanceMm,
                                       uint16_t validMinimumMm,
                                       uint16_t validMaximumMm) {
  return distanceMm >= validMinimumMm && distanceMm <= validMaximumMm;
}

inline RearObstacleFrameSummary aggregateRearObstacleFrame(
    const uint8_t *payload,
    size_t payloadLength,
    uint16_t *logicalDistancesMm,
    uint16_t validMinimumMm,
    uint16_t validMaximumMm,
    uint8_t activeFirstRow,
    uint8_t activeRowCount) {
  RearObstacleFrameSummary summary = {0, 0};
  if (payload == NULL || payloadLength != 128U) {
    return summary;
  }

  uint16_t minimumDistanceMm = UINT16_MAX;
  for (uint8_t row = 0; row < 8U; row++) {
    for (uint8_t col = 0; col < 8U; col++) {
      const uint8_t logicalIndex = (uint8_t)(row * 8U + col);
      const uint8_t rawIndex = rearObstacleRawIndexForLogical(row, col);
      const size_t byteIndex = (size_t)rawIndex * 2U;
      const uint16_t distanceMm =
        (uint16_t)payload[byteIndex] |
        ((uint16_t)payload[byteIndex + 1U] << 8U);

      if (logicalDistancesMm != NULL) {
        logicalDistancesMm[logicalIndex] = distanceMm;
      }

      const bool activeRow =
        row >= activeFirstRow &&
        (uint16_t)row < (uint16_t)activeFirstRow + activeRowCount;
      if (!activeRow ||
          !isRearObstacleCellValid(distanceMm,
                                   validMinimumMm,
                                   validMaximumMm)) {
        continue;
      }

      summary.validCellCount++;
      if (distanceMm < minimumDistanceMm) {
        minimumDistanceMm = distanceMm;
      }
    }
  }

  if (summary.validCellCount > 0U) {
    summary.minimumDistanceMm = minimumDistanceMm;
  }
  return summary;
}

inline uint8_t aggregateRearObstacleColumnRays(
    const uint16_t *logicalDistancesMm,
    uint16_t *columnDistancesMm,
    bool *columnValid,
    uint16_t validMinimumMm,
    uint16_t validMaximumMm,
    uint8_t activeFirstRow,
    uint8_t activeRowCount,
    float verticalFieldOfViewDeg) {
  if (logicalDistancesMm == NULL || columnDistancesMm == NULL ||
      columnValid == NULL) {
    return 0;
  }

  uint8_t validColumnCount = 0;
  const float rowAngleStepDeg = verticalFieldOfViewDeg / 8.0f;
  for (uint8_t col = 0; col < 8U; col++) {
    columnDistancesMm[col] = 0;
    columnValid[col] = false;
    uint16_t minimumProjectedDistanceMm = UINT16_MAX;

    for (uint8_t row = 0; row < 8U; row++) {
      const bool activeRow =
        row >= activeFirstRow &&
        (uint16_t)row < (uint16_t)activeFirstRow + activeRowCount;
      const uint16_t distanceMm = logicalDistancesMm[row * 8U + col];
      if (!activeRow ||
          !isRearObstacleCellValid(distanceMm,
                                   validMinimumMm,
                                   validMaximumMm)) {
        continue;
      }

      const float verticalAngleDeg =
        verticalFieldOfViewDeg * 0.5f - (row + 0.5f) * rowAngleStepDeg;
      const uint16_t projectedDistanceMm = (uint16_t)(
        distanceMm * cosf(verticalAngleDeg * 0.01745329252f));
      if (projectedDistanceMm < minimumProjectedDistanceMm) {
        minimumProjectedDistanceMm = projectedDistanceMm;
      }
    }

    if (minimumProjectedDistanceMm != UINT16_MAX) {
      columnDistancesMm[col] = minimumProjectedDistanceMm;
      columnValid[col] = true;
      validColumnCount++;
    }
  }
  return validColumnCount;
}

constexpr RearObstacleHysteresisState updateRearObstacleHysteresis(
    RearObstacleHysteresisState state,
    bool frameValid,
    uint16_t minimumDistanceMm,
    uint16_t stopDistanceMm,
    uint16_t clearDistanceMm,
    uint8_t clearConfirmFrames) {
  return !frameValid || minimumDistanceMm <= stopDistanceMm
    ? RearObstacleHysteresisState{true, 0}
    : minimumDistanceMm >= clearDistanceMm
      ? RearObstacleHysteresisState{
          state.consecutiveClearFrames + 1U >= clearConfirmFrames
            ? false : state.blocked,
          state.consecutiveClearFrames < clearConfirmFrames
            ? (uint8_t)(state.consecutiveClearFrames + 1U)
            : clearConfirmFrames}
      : RearObstacleHysteresisState{state.blocked, 0};
}

static_assert(rearObstacleRawIndexForLogical(0, 0) == 63,
              "Rear matrix must be rolled 180 degrees");
static_assert(rearObstacleRawIndexForLogical(7, 7) == 0,
              "Rear matrix roll must map both axes");
static_assert(!isRearObstacleCellValid(0, 20, 3999) &&
              !isRearObstacleCellValid(4000, 20, 3999) &&
              isRearObstacleCellValid(20, 20, 3999) &&
              isRearObstacleCellValid(3999, 20, 3999),
              "Rear matrix valid range changed unexpectedly");
static_assert(updateRearObstacleHysteresis({false, 2}, true, 250,
                                           250, 300, 3).blocked,
              "Rear obstacle at the stop threshold must block immediately");
static_assert(updateRearObstacleHysteresis({true, 0}, true, 275,
                                           250, 300, 3).blocked,
              "The rear hysteresis band must retain the blocked state");
static_assert(!updateRearObstacleHysteresis(
                updateRearObstacleHysteresis(
                  updateRearObstacleHysteresis({true, 0}, true, 300,
                                               250, 300, 3),
                  true, 300, 250, 300, 3),
                true, 300, 250, 300, 3).blocked,
              "Rear clearance must require three consecutive clear frames");

#endif
