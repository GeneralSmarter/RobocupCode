#ifndef FRONT_MATRIX_POLICY_H
#define FRONT_MATRIX_POLICY_H

#include <stdint.h>
#include <math.h>

constexpr uint8_t frontMatrixRotateIndex(uint8_t row, uint8_t column,
                                         uint8_t quarterTurns,
                                         bool flipRows,
                                         bool flipColumns) {
  uint8_t r = flipRows ? (uint8_t)(7U - row) : row;
  uint8_t c = flipColumns ? (uint8_t)(7U - column) : column;
  return quarterTurns == 0 ? (uint8_t)(r * 8U + c)
    : quarterTurns == 1 ? (uint8_t)(c * 8U + (7U - r))
    : quarterTurns == 2 ? (uint8_t)((7U - r) * 8U + (7U - c))
    : (uint8_t)((7U - c) * 8U + r);
}

constexpr bool frontMatrixDistanceValid(uint16_t distanceMm,
                                        uint16_t validMinimumMm,
                                        uint16_t validMaximumMm) {
  return distanceMm >= validMinimumMm && distanceMm <= validMaximumMm;
}

inline float frontMatrixHorizontalAngleDeg(uint8_t column, float fovDeg) {
  return fovDeg * 0.5f - (column + 0.5f) * (fovDeg / 8.0f);
}

inline float frontMatrixVerticalAngleDeg(uint8_t row, float fovDeg) {
  return fovDeg * 0.5f - (row + 0.5f) * (fovDeg / 8.0f);
}

inline float frontMatrixHorizontalRangeMm(uint16_t distanceMm,
                                          float verticalAngleDeg) {
  return distanceMm * cosf(verticalAngleDeg * 0.01745329252f);
}

inline float frontMatrixHitHeightMm(uint16_t distanceMm,
                                    float sensorHeightMm,
                                    float verticalAngleDeg) {
  return sensorHeightMm +
         distanceMm * sinf(verticalAngleDeg * 0.01745329252f);
}

static_assert(frontMatrixRotateIndex(0, 0, 0, false, false) == 0,
              "Identity matrix transform must preserve the first cell");
static_assert(frontMatrixRotateIndex(0, 0, 1, false, false) == 7,
              "Quarter-turn transform must rotate the first cell");
static_assert(frontMatrixRotateIndex(0, 0, 2, false, false) == 63,
              "Half-turn transform must rotate both axes");
static_assert(frontMatrixRotateIndex(0, 0, 0, true, true) == 63,
              "Explicit row/column flips must replace implicit mounting fixes");

#endif
