#include "../../Robot.h"
#include "FrontMatrixPolicy.h"

// Front SEN0628 I/O driver: connection, frame polling, and the
// column-clearance/weight-perception rays consumed by the planner.
namespace {

const uint8_t FRONT_MATRIX_COMMAND_SET_MODE = 1;
const uint8_t FRONT_MATRIX_COMMAND_ALL_DATA = 2;
const uint8_t FRONT_MATRIX_STATUS_SUCCESS = 0x53;
const uint8_t FRONT_MATRIX_STATUS_FAILED = 0x63;
const uint8_t FRONT_MATRIX_STATUS_PENDING = 0xFF;
const uint8_t FRONT_MATRIX_PACKET_HEAD = 0x55;
const uint8_t FRONT_MATRIX_MODE_8X8 = 8;
const uint16_t FRONT_MATRIX_FRAME_BYTES = 128;
const uint8_t FRONT_MATRIX_I2C_CHUNK_BYTES = 32;
const float FRONT_MATRIX_MIN_OBSTACLE_HEIGHT_MM = 20.0f;

enum FrontMatrixIoState {
  FRONT_MATRIX_IO_DISCONNECTED,
  FRONT_MATRIX_IO_WAIT_MODE_STATUS,
  FRONT_MATRIX_IO_MODE_SETTLING,
  FRONT_MATRIX_IO_IDLE,
  FRONT_MATRIX_IO_WAIT_FRAME_STATUS,
  FRONT_MATRIX_IO_READ_PAYLOAD
};

FrontMatrixIoState frontMatrixIoState = FRONT_MATRIX_IO_DISCONNECTED;
bool frontMatrixConnected = false;
bool frontMatrixInitialized = false;
bool frontMatrixResponseSuccess = false;
uint8_t frontMatrixExpectedCommand = 0;
uint16_t frontMatrixResponseLength = 0;
uint16_t frontMatrixResponseOffset = 0;
uint8_t frontMatrixPayload[FRONT_MATRIX_FRAME_BYTES] = {0};
uint16_t frontMatrixColumnDistanceMm[8] = {0};
bool frontMatrixColumnValid[8] = {false};
uint8_t frontMatrixValidCellCount = 0;
uint8_t frontMatrixSafetyCellCount = 0;
uint8_t frontMatrixClearFrames = 0;
unsigned long frontMatrixLastRequestMs = 0;
unsigned long frontMatrixResponseStartedMs = 0;
unsigned long frontMatrixModeSettlingStartedMs = 0;
unsigned long frontMatrixReconnectAttemptMs = 0;
unsigned long frontMatrixReadDurationUs = 0;
unsigned long frontMatrixReadAccumulatorUs = 0;

void addFrontMatrixIoDuration(unsigned long startedUs) {
  frontMatrixReadAccumulatorUs += micros() - startedUs;
}

void publishFrontMatrixUnavailable(bool timeout) {
  RangeSensorState &sensor = rangeSensors[RANGE_FRONT_MATRIX_AGGREGATE];
  sensor.distanceMm = RANGE_NO_READING_MM;
  sensor.valid = false;
  sensor.stale = true;
  // Supplemental mode: missing matrix evidence cannot prove clearance, but it
  // does not override the independent front fan with a synthetic stop.
  sensor.blocked = false;
  frontMatrixValidCellCount = 0;
  frontMatrixSafetyCellCount = 0;
  frontMatrixClearFrames = 0;
  for (uint8_t col = 0; col < 8; col++) {
    frontMatrixColumnDistanceMm[col] = 0;
    frontMatrixColumnValid[col] = false;
  }
  if (timeout) sensor.timeoutCount++;
  else sensor.invalidCount++;
}

bool frontMatrixWritePacket(const uint8_t *packet, uint8_t length) {
  const unsigned long startedUs = micros();
  Wire1.beginTransmission(FRONT_MATRIX_CONFIG.i2cAddress);
  Wire1.write(packet, length);
  const uint8_t result = Wire1.endTransmission();
  addFrontMatrixIoDuration(startedUs);
  return result == 0;
}

bool frontMatrixReadBytes(uint8_t *destination, uint8_t length) {
  const unsigned long startedUs = micros();
  const uint8_t received = Wire1.requestFrom(
    FRONT_MATRIX_CONFIG.i2cAddress, length);
  if (received != length) {
    while (Wire1.available()) Wire1.read();
    addFrontMatrixIoDuration(startedUs);
    return false;
  }
  for (uint8_t i = 0; i < length; i++) {
    destination[i] = (uint8_t)Wire1.read();
  }
  addFrontMatrixIoDuration(startedUs);
  return true;
}

void disconnectFrontMatrix(bool timeout) {
  publishFrontMatrixUnavailable(timeout);
  frontMatrixConnected = false;
  frontMatrixInitialized = false;
  frontMatrixIoState = FRONT_MATRIX_IO_DISCONNECTED;
  frontMatrixReconnectAttemptMs = millis();
  frontMatrixReadDurationUs = frontMatrixReadAccumulatorUs;
  frontMatrixReadAccumulatorUs = 0;
}

bool sendFrontMatrixModeCommand() {
  const uint8_t packet[] = {
    FRONT_MATRIX_PACKET_HEAD, 0, 5, FRONT_MATRIX_COMMAND_SET_MODE,
    0, 0, 0, FRONT_MATRIX_MODE_8X8
  };
  if (!frontMatrixWritePacket(packet, sizeof(packet))) return false;
  frontMatrixExpectedCommand = FRONT_MATRIX_COMMAND_SET_MODE;
  frontMatrixResponseStartedMs = millis();
  frontMatrixIoState = FRONT_MATRIX_IO_WAIT_MODE_STATUS;
  return true;
}

bool sendFrontMatrixFrameCommand() {
  const uint8_t packet[] = {
    FRONT_MATRIX_PACKET_HEAD, 0, 1, FRONT_MATRIX_COMMAND_ALL_DATA
  };
  frontMatrixReadAccumulatorUs = 0;
  if (!frontMatrixWritePacket(packet, sizeof(packet))) return false;
  frontMatrixExpectedCommand = FRONT_MATRIX_COMMAND_ALL_DATA;
  frontMatrixResponseStartedMs = millis();
  frontMatrixLastRequestMs = frontMatrixResponseStartedMs;
  frontMatrixIoState = FRONT_MATRIX_IO_WAIT_FRAME_STATUS;
  return true;
}

bool readFrontMatrixResponseHeader() {
  uint8_t header[3];
  if (!frontMatrixReadBytes(header, sizeof(header)) ||
      header[0] != frontMatrixExpectedCommand) return false;
  frontMatrixResponseLength =
    (uint16_t)header[1] | ((uint16_t)header[2] << 8U);
  frontMatrixResponseOffset = 0;
  return frontMatrixResponseLength <= FRONT_MATRIX_FRAME_BYTES;
}

void publishFrontMatrixFrame() {
  FrontMatrixFrame next = {};
  next.valid = true;
  next.sequence = frontMatrixFrame.sequence + 1U;
  if (next.sequence == 0) next.sequence = 1;
  next.acquiredMs = millis();
  next.robotX = robotX;
  next.robotY = robotY;
  next.robotHeadingDeg = navigationHeadingDeg();
  getLatestImuAttitude(next.pitchDeg, next.rollDeg);
  next.gridRotationQuarterTurns =
    FRONT_MATRIX_CONFIG.gridRotationQuarterTurns;
  next.flipRows = FRONT_MATRIX_CONFIG.flipRows;
  next.flipColumns = FRONT_MATRIX_CONFIG.flipColumns;
  frontMatrixValidCellCount = 0;
  frontMatrixSafetyCellCount = 0;
  uint16_t minimumSafetyDistanceMm = UINT16_MAX;
  for (uint8_t col = 0; col < 8; col++) {
    frontMatrixColumnDistanceMm[col] = 0;
    frontMatrixColumnValid[col] = false;
  }

  for (uint8_t rawRow = 0; rawRow < 8; rawRow++) {
    for (uint8_t rawCol = 0; rawCol < 8; rawCol++) {
      const uint8_t rawIndex = rawRow * 8U + rawCol;
      const uint16_t byteIndex = rawIndex * 2U;
      const uint16_t distanceMm =
        (uint16_t)frontMatrixPayload[byteIndex] |
        ((uint16_t)frontMatrixPayload[byteIndex + 1U] << 8U);
      const uint8_t logicalIndex = frontMatrixRotateIndex(
        rawRow, rawCol, FRONT_MATRIX_CONFIG.gridRotationQuarterTurns,
        FRONT_MATRIX_CONFIG.flipRows, FRONT_MATRIX_CONFIG.flipColumns);
      next.distanceMm[logicalIndex] = distanceMm;
      const bool valid = frontMatrixDistanceValid(
        distanceMm, FRONT_MATRIX_TOF_VALID_MIN_MM,
        FRONT_MATRIX_TOF_VALID_MAX_MM);
      next.cellState[logicalIndex] = valid
        ? FRONT_MATRIX_CELL_VALID : FRONT_MATRIX_CELL_UNKNOWN;
      if (!valid) continue;
      frontMatrixValidCellCount++;

      const uint8_t row = logicalIndex / 8U;
      const uint8_t col = logicalIndex % 8U;
      if ((FRONT_MATRIX_CONFIG.safetyCellMask &
           (UINT64_C(1) << logicalIndex)) == 0) continue;
      if (matrixCellBelongsToActivePickup(logicalIndex)) continue;
      const float verticalDeg = frontMatrixVerticalAngleDeg(
        row, FRONT_MATRIX_CONFIG.verticalFovDeg);
      const float hitHeightMm = frontMatrixHitHeightMm(
        distanceMm, FRONT_MATRIX_CONFIG.mount.zMm, verticalDeg);
      if (hitHeightMm < FRONT_MATRIX_MIN_OBSTACLE_HEIGHT_MM) continue;
      const uint16_t horizontalMm = (uint16_t)frontMatrixHorizontalRangeMm(
        distanceMm, verticalDeg);
      frontMatrixSafetyCellCount++;
      if (!frontMatrixColumnValid[col] ||
          horizontalMm < frontMatrixColumnDistanceMm[col]) {
        frontMatrixColumnValid[col] = true;
        frontMatrixColumnDistanceMm[col] = horizontalMm;
      }
      if (horizontalMm < minimumSafetyDistanceMm) {
        minimumSafetyDistanceMm = horizontalMm;
      }
    }
  }

  frontMatrixFrame = next;
  RangeSensorState &sensor = rangeSensors[RANGE_FRONT_MATRIX_AGGREGATE];
  sensor.lastReadMs = next.acquiredMs;
  sensor.stale = false;
  sensor.valid = frontMatrixSafetyCellCount > 0;
  sensor.distanceMm = sensor.valid ? minimumSafetyDistanceMm
                                   : RANGE_NO_READING_MM;
  if (sensor.valid &&
      minimumSafetyDistanceMm <= FRONT_MATRIX_TOF_STOP_DISTANCE_MM) {
    sensor.blocked = true;
    frontMatrixClearFrames = 0;
  } else if (sensor.valid &&
             minimumSafetyDistanceMm >= FRONT_MATRIX_TOF_CLEAR_DISTANCE_MM) {
    if (frontMatrixClearFrames < FRONT_MATRIX_TOF_CLEAR_CONFIRM_FRAMES) {
      frontMatrixClearFrames++;
    }
    if (frontMatrixClearFrames >= FRONT_MATRIX_TOF_CLEAR_CONFIRM_FRAMES) {
      sensor.blocked = false;
    }
  } else {
    frontMatrixClearFrames = 0;
  }
  updateMatrixWeightDetection();
  frontMatrixReadDurationUs = frontMatrixReadAccumulatorUs;
  frontMatrixReadAccumulatorUs = 0;
}

void finishFrontMatrixResponse() {
  if (frontMatrixExpectedCommand == FRONT_MATRIX_COMMAND_SET_MODE) {
    if (!frontMatrixResponseSuccess) {
      disconnectFrontMatrix(false);
      return;
    }
    frontMatrixModeSettlingStartedMs = millis();
    frontMatrixIoState = FRONT_MATRIX_IO_MODE_SETTLING;
    return;
  }
  if (!frontMatrixResponseSuccess ||
      frontMatrixResponseLength != FRONT_MATRIX_FRAME_BYTES) {
    publishFrontMatrixUnavailable(false);
  } else {
    publishFrontMatrixFrame();
  }
  frontMatrixIoState = FRONT_MATRIX_IO_IDLE;
}

bool readFrontMatrixPayloadChunk() {
  const uint16_t remaining =
    frontMatrixResponseLength - frontMatrixResponseOffset;
  const uint8_t chunkLength = remaining > FRONT_MATRIX_I2C_CHUNK_BYTES
    ? FRONT_MATRIX_I2C_CHUNK_BYTES : (uint8_t)remaining;
  if (chunkLength == 0) {
    finishFrontMatrixResponse();
    return true;
  }
  if (!frontMatrixReadBytes(
        frontMatrixPayload + frontMatrixResponseOffset, chunkLength)) {
    return false;
  }
  frontMatrixResponseOffset += chunkLength;
  if (frontMatrixResponseOffset == frontMatrixResponseLength) {
    finishFrontMatrixResponse();
  }
  return true;
}

bool pollFrontMatrixResponseStatus() {
  uint8_t status = FRONT_MATRIX_STATUS_PENDING;
  if (!frontMatrixReadBytes(&status, 1)) return false;
  if (status == FRONT_MATRIX_STATUS_PENDING) return true;
  if (status != FRONT_MATRIX_STATUS_SUCCESS &&
      status != FRONT_MATRIX_STATUS_FAILED) return false;
  frontMatrixResponseSuccess = status == FRONT_MATRIX_STATUS_SUCCESS;
  if (!readFrontMatrixResponseHeader()) return false;
  frontMatrixIoState = FRONT_MATRIX_IO_READ_PAYLOAD;
  return readFrontMatrixPayloadChunk();
}

void expireFrontMatrixFrame(unsigned long now) {
  RangeSensorState &sensor = rangeSensors[RANGE_FRONT_MATRIX_AGGREGATE];
  if (sensor.lastReadMs > 0 &&
      now - sensor.lastReadMs > FRONT_MATRIX_TOF_STALE_TIMEOUT_MS) {
    publishFrontMatrixUnavailable(true);
  }
}

}  // namespace

void connectFrontMatrixSensor() {
  publishFrontMatrixUnavailable(false);
  Wire1.begin();
  frontMatrixConnected = true;
  if (!sendFrontMatrixModeCommand()) disconnectFrontMatrix(false);
}

void updateFrontMatrixSensor() {
  const unsigned long now = millis();
  expireFrontMatrixFrame(now);
  switch (frontMatrixIoState) {
    case FRONT_MATRIX_IO_DISCONNECTED:
      if (now - frontMatrixReconnectAttemptMs >=
          FRONT_MATRIX_TOF_RECONNECT_INTERVAL_MS) {
        frontMatrixReconnectAttemptMs = now;
        frontMatrixConnected = true;
        if (!sendFrontMatrixModeCommand()) disconnectFrontMatrix(false);
      }
      break;
    case FRONT_MATRIX_IO_WAIT_MODE_STATUS:
      if (now - frontMatrixResponseStartedMs >
          FRONT_MATRIX_TOF_MODE_TIMEOUT_MS) disconnectFrontMatrix(true);
      else if (!pollFrontMatrixResponseStatus()) disconnectFrontMatrix(false);
      break;
    case FRONT_MATRIX_IO_MODE_SETTLING:
      if (now - frontMatrixModeSettlingStartedMs >=
          FRONT_MATRIX_TOF_MODE_SETTLE_MS) {
        frontMatrixInitialized = true;
        frontMatrixIoState = FRONT_MATRIX_IO_IDLE;
      }
      break;
    case FRONT_MATRIX_IO_IDLE:
      if (now - frontMatrixLastRequestMs >=
          FRONT_MATRIX_TOF_FRAME_INTERVAL_MS &&
          !sendFrontMatrixFrameCommand()) disconnectFrontMatrix(false);
      break;
    case FRONT_MATRIX_IO_WAIT_FRAME_STATUS:
      if (now - frontMatrixResponseStartedMs >
          FRONT_MATRIX_TOF_RESPONSE_TIMEOUT_MS) disconnectFrontMatrix(true);
      else if (!pollFrontMatrixResponseStatus()) disconnectFrontMatrix(false);
      break;
    case FRONT_MATRIX_IO_READ_PAYLOAD:
      if (now - frontMatrixResponseStartedMs >
          FRONT_MATRIX_TOF_RESPONSE_TIMEOUT_MS) disconnectFrontMatrix(true);
      else if (!readFrontMatrixPayloadChunk()) disconnectFrontMatrix(false);
      break;
  }
}

bool getFrontMatrixFrame(FrontMatrixFrame &frame) {
  if (!frontMatrixConnected || !frontMatrixInitialized ||
      !frontMatrixFrame.valid ||
      frontMatrixFrame.sequence == 0 ||
      millis() - frontMatrixFrame.acquiredMs >
        FRONT_MATRIX_TOF_STALE_TIMEOUT_MS) return false;
  frame = frontMatrixFrame;
  return true;
}

bool getFrontMatrixRay(uint8_t column, RangeRayObservation &observation) {
  if (column >= 8 || !frontMatrixColumnValid[column] ||
      frontMatrixFrame.sequence == 0 ||
      millis() - frontMatrixFrame.acquiredMs >
        FRONT_MATRIX_TOF_STALE_TIMEOUT_MS) return false;
  const float yaw = FRONT_MATRIX_CONFIG.mount.yawDeg +
    frontMatrixHorizontalAngleDeg(
      column, FRONT_MATRIX_CONFIG.horizontalFovDeg);
  observation = {
    FRONT_MATRIX_CONFIG.mount.xMm, FRONT_MATRIX_CONFIG.mount.yMm,
    FRONT_MATRIX_CONFIG.mount.zMm, yaw,
    FRONT_MATRIX_CONFIG.mount.pitchDeg,
    frontMatrixColumnDistanceMm[column], frontMatrixFrame.sequence,
    frontMatrixFrame.acquiredMs
  };
  return true;
}

void printFrontMatrixStatus() {
  const RangeSensorState &sensor =
    rangeSensors[RANGE_FRONT_MATRIX_AGGREGATE];
  Serial2.print("front_matrix,connected=");
  Serial2.print(frontMatrixConnected ? 1 : 0);
  Serial2.print(",initialized=");
  Serial2.print(frontMatrixInitialized ? 1 : 0);
  Serial2.print(",sequence="); Serial2.print(frontMatrixFrame.sequence);
  Serial2.print(",age_ms=");
  Serial2.print(frontMatrixFrame.acquiredMs > 0
    ? millis() - frontMatrixFrame.acquiredMs : 0);
  Serial2.print(",valid_cells="); Serial2.print(frontMatrixValidCellCount);
  Serial2.print(",safety_cells="); Serial2.print(frontMatrixSafetyCellCount);
  Serial2.print(",minimum_mm="); Serial2.print(sensor.distanceMm);
  Serial2.print(",blocked="); Serial2.print(sensor.blocked ? 1 : 0);
  Serial2.print(",read_us="); Serial2.println(frontMatrixReadDurationUs);
}
