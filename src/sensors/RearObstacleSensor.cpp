#include "Robot.h"
#include "RearObstaclePolicy.h"

// The SEN0628 packet stream is serviced in small steps from the normal 20 ms
// sensor schedule. No call waits for a complete 128-byte matrix response.
namespace {

const uint8_t REAR_TOF_COMMAND_SET_MODE = 1;
const uint8_t REAR_TOF_COMMAND_ALL_DATA = 2;
const uint8_t REAR_TOF_STATUS_SUCCESS = 0x53;
const uint8_t REAR_TOF_STATUS_FAILED = 0x63;
const uint8_t REAR_TOF_STATUS_PENDING = 0xFF;
const uint8_t REAR_TOF_PACKET_HEAD = 0x55;
const uint8_t REAR_TOF_MATRIX_8X8 = 8;
const uint16_t REAR_TOF_FRAME_BYTES = 128;
const uint8_t REAR_TOF_I2C_CHUNK_BYTES = 32;

enum RearObstacleIoState {
  REAR_IO_DISCONNECTED,
  REAR_IO_WAIT_MODE_STATUS,
  REAR_IO_MODE_SETTLING,
  REAR_IO_IDLE,
  REAR_IO_WAIT_FRAME_STATUS,
  REAR_IO_READ_PAYLOAD
};

RearObstacleIoState rearIoState = REAR_IO_DISCONNECTED;
bool rearConnected = false;
bool rearInitialized = false;
bool rearResponseSuccess = false;
uint8_t rearExpectedCommand = 0;
uint16_t rearResponseLength = 0;
uint16_t rearResponseOffset = 0;
uint8_t rearResponsePayload[REAR_TOF_FRAME_BYTES] = {0};
uint16_t rearLogicalDistancesMm[64] = {0};
uint16_t rearColumnDistancesMm[8] = {0};
bool rearColumnValid[8] = {false};
uint32_t rearFrameSequence = 0;
uint8_t rearValidCellCount = 0;
uint8_t rearValidColumnCount = 0;
uint16_t rearMinimumDistanceMm = 0;
unsigned long rearLastRequestMs = 0;
unsigned long rearResponseStartedMs = 0;
unsigned long rearModeSettlingStartedMs = 0;
unsigned long rearReconnectAttemptMs = 0;
unsigned long rearReadDurationUs = 0;
unsigned long rearReadAccumulatorUs = 0;
RearObstacleHysteresisState rearHysteresis = {true, 0};

void addRearIoDuration(unsigned long startedUs) {
  rearReadAccumulatorUs += micros() - startedUs;
}

void publishRearUnavailable(bool timeout) {
  RangeSensorState &sensor = rangeSensors[RANGE_FAKE_REAR];
  sensor.distanceMm = RANGE_NO_READING_MM;
  sensor.valid = false;
  sensor.stale = true;
  sensor.blocked = true;
  rearHysteresis.blocked = true;
  rearHysteresis.consecutiveClearFrames = 0;
  rearValidCellCount = 0;
  rearValidColumnCount = 0;
  rearMinimumDistanceMm = 0;
  for (uint8_t col = 0; col < REAR_MATRIX_TOF_COLUMN_COUNT; col++) {
    rearColumnDistancesMm[col] = 0;
    rearColumnValid[col] = false;
  }
  if (timeout) {
    sensor.timeoutCount++;
  } else {
    sensor.invalidCount++;
  }
}

bool rearWritePacket(const uint8_t *packet, uint8_t length) {
  const unsigned long startedUs = micros();
  Wire1.beginTransmission(REAR_MATRIX_TOF_I2C_ADDRESS);
  Wire1.write(packet, length);
  const uint8_t result = Wire1.endTransmission();
  addRearIoDuration(startedUs);
  return result == 0;
}

bool rearReadBytes(uint8_t *destination, uint8_t length) {
  const unsigned long startedUs = micros();
  const uint8_t received =
    Wire1.requestFrom(REAR_MATRIX_TOF_I2C_ADDRESS, length);
  if (received != length) {
    while (Wire1.available()) {
      Wire1.read();
    }
    addRearIoDuration(startedUs);
    return false;
  }

  for (uint8_t i = 0; i < length; i++) {
    destination[i] = (uint8_t)Wire1.read();
  }
  addRearIoDuration(startedUs);
  return true;
}

void disconnectRearSensor(bool timeout) {
  publishRearUnavailable(timeout);
  rearConnected = false;
  rearInitialized = false;
  rearIoState = REAR_IO_DISCONNECTED;
  rearReconnectAttemptMs = millis();
  rearReadDurationUs = rearReadAccumulatorUs;
  rearReadAccumulatorUs = 0;
}

bool sendRearModeCommand() {
  const uint8_t packet[] = {
    REAR_TOF_PACKET_HEAD, 0, 5, REAR_TOF_COMMAND_SET_MODE,
    0, 0, 0, REAR_TOF_MATRIX_8X8
  };
  if (!rearWritePacket(packet, sizeof(packet))) {
    return false;
  }

  rearExpectedCommand = REAR_TOF_COMMAND_SET_MODE;
  rearResponseStartedMs = millis();
  rearIoState = REAR_IO_WAIT_MODE_STATUS;
  return true;
}

bool sendRearFrameCommand() {
  const uint8_t packet[] = {
    REAR_TOF_PACKET_HEAD, 0, 1, REAR_TOF_COMMAND_ALL_DATA
  };
  rearReadAccumulatorUs = 0;
  if (!rearWritePacket(packet, sizeof(packet))) {
    return false;
  }

  rearExpectedCommand = REAR_TOF_COMMAND_ALL_DATA;
  rearResponseStartedMs = millis();
  rearLastRequestMs = rearResponseStartedMs;
  rearIoState = REAR_IO_WAIT_FRAME_STATUS;
  return true;
}

bool readRearResponseHeader() {
  uint8_t header[3];
  if (!rearReadBytes(header, sizeof(header))) {
    return false;
  }

  if (header[0] != rearExpectedCommand) {
    return false;
  }

  rearResponseLength =
    (uint16_t)header[1] | ((uint16_t)header[2] << 8U);
  rearResponseOffset = 0;
  if (rearResponseLength > REAR_TOF_FRAME_BYTES) {
    return false;
  }
  return true;
}

void publishRearFrame() {
  RearObstacleFrameSummary summary = aggregateRearObstacleFrame(
    rearResponsePayload,
    rearResponseLength,
    rearLogicalDistancesMm,
    REAR_MATRIX_TOF_VALID_MIN_MM,
    REAR_MATRIX_TOF_VALID_MAX_MM,
    REAR_MATRIX_TOF_ACTIVE_FIRST_ROW,
    REAR_MATRIX_TOF_ACTIVE_ROW_COUNT);
  rearReadDurationUs = rearReadAccumulatorUs;
  rearReadAccumulatorUs = 0;

  if (summary.validCellCount == 0) {
    publishRearUnavailable(false);
    return;
  }

  const uint8_t validColumns = aggregateRearObstacleColumnRays(
    rearLogicalDistancesMm,
    rearColumnDistancesMm,
    rearColumnValid,
    REAR_MATRIX_TOF_VALID_MIN_MM,
    REAR_MATRIX_TOF_VALID_MAX_MM,
    REAR_MATRIX_TOF_ACTIVE_FIRST_ROW,
    REAR_MATRIX_TOF_ACTIVE_ROW_COUNT,
    REAR_MATRIX_TOF_VERTICAL_FOV_DEG);
  if (validColumns == 0) {
    publishRearUnavailable(false);
    return;
  }
  rearValidColumnCount = validColumns;

  RangeSensorState &sensor = rangeSensors[RANGE_FAKE_REAR];
  rearHysteresis = updateRearObstacleHysteresis(
    rearHysteresis,
    true,
    summary.minimumDistanceMm,
    REAR_MATRIX_TOF_STOP_DISTANCE_MM,
    REAR_MATRIX_TOF_CLEAR_DISTANCE_MM,
    REAR_MATRIX_TOF_CLEAR_CONFIRM_FRAMES);
  rearValidCellCount = summary.validCellCount;
  rearMinimumDistanceMm = summary.minimumDistanceMm;
  sensor.distanceMm = summary.minimumDistanceMm;
  sensor.valid = true;
  sensor.stale = false;
  sensor.blocked = rearHysteresis.blocked;
  sensor.lastReadMs = millis();
  rearFrameSequence++;
  if (rearFrameSequence == 0) {
    rearFrameSequence++;
  }
}

void finishRearResponse() {
  if (rearExpectedCommand == REAR_TOF_COMMAND_SET_MODE) {
    if (!rearResponseSuccess) {
      disconnectRearSensor(false);
      return;
    }
    rearModeSettlingStartedMs = millis();
    rearIoState = REAR_IO_MODE_SETTLING;
    return;
  }

  if (!rearResponseSuccess || rearResponseLength != REAR_TOF_FRAME_BYTES) {
    publishRearUnavailable(false);
  } else {
    publishRearFrame();
  }
  rearIoState = REAR_IO_IDLE;
}

bool readRearPayloadChunk() {
  const uint16_t remaining = rearResponseLength - rearResponseOffset;
  const uint8_t chunkLength = remaining > REAR_TOF_I2C_CHUNK_BYTES
    ? REAR_TOF_I2C_CHUNK_BYTES
    : (uint8_t)remaining;
  if (chunkLength == 0) {
    finishRearResponse();
    return true;
  }

  if (!rearReadBytes(rearResponsePayload + rearResponseOffset, chunkLength)) {
    return false;
  }
  rearResponseOffset += chunkLength;
  if (rearResponseOffset == rearResponseLength) {
    finishRearResponse();
  }
  return true;
}

bool pollRearResponseStatus() {
  uint8_t status = REAR_TOF_STATUS_PENDING;
  if (!rearReadBytes(&status, 1)) {
    return false;
  }
  if (status == REAR_TOF_STATUS_PENDING) {
    return true;
  }
  if (status != REAR_TOF_STATUS_SUCCESS &&
      status != REAR_TOF_STATUS_FAILED) {
    return false;
  }

  rearResponseSuccess = status == REAR_TOF_STATUS_SUCCESS;
  if (!readRearResponseHeader()) {
    return false;
  }
  rearIoState = REAR_IO_READ_PAYLOAD;
  return readRearPayloadChunk();
}

void expireRearFrameIfStale(unsigned long now) {
  RangeSensorState &sensor = rangeSensors[RANGE_FAKE_REAR];
  if (sensor.valid && now - sensor.lastReadMs > REAR_MATRIX_TOF_STALE_TIMEOUT_MS) {
    publishRearUnavailable(true);
  }
}

}  // namespace

void connectRearObstacleSensor() {
  RangeSensorState &sensor = rangeSensors[RANGE_FAKE_REAR];
  sensor.distanceMm = RANGE_NO_READING_MM;
  sensor.valid = false;
  sensor.stale = true;
  sensor.blocked = true;
  rearHysteresis = {true, 0};

  Wire1.begin();
  rearReadAccumulatorUs = 0;
  rearConnected = true;
  if (!sendRearModeCommand()) {
    disconnectRearSensor(false);
  }
}

void updateRearObstacleSensor() {
  const unsigned long now = millis();
  expireRearFrameIfStale(now);

  switch (rearIoState) {
    case REAR_IO_DISCONNECTED:
      if (now - rearReconnectAttemptMs >= REAR_MATRIX_TOF_RECONNECT_INTERVAL_MS) {
        rearReconnectAttemptMs = now;
        rearReadAccumulatorUs = 0;
        rearConnected = true;
        if (!sendRearModeCommand()) {
          disconnectRearSensor(false);
        }
      }
      break;

    case REAR_IO_WAIT_MODE_STATUS:
      if (now - rearResponseStartedMs > REAR_MATRIX_TOF_MODE_TIMEOUT_MS) {
        disconnectRearSensor(true);
      } else if (!pollRearResponseStatus()) {
        disconnectRearSensor(false);
      }
      break;

    case REAR_IO_MODE_SETTLING:
      if (now - rearModeSettlingStartedMs >= REAR_MATRIX_TOF_MODE_SETTLE_MS) {
        rearInitialized = true;
        rearIoState = REAR_IO_IDLE;
      }
      break;

    case REAR_IO_IDLE:
      if (now - rearLastRequestMs >= REAR_MATRIX_TOF_FRAME_INTERVAL_MS &&
          !sendRearFrameCommand()) {
        disconnectRearSensor(false);
      }
      break;

    case REAR_IO_WAIT_FRAME_STATUS:
      if (now - rearResponseStartedMs > REAR_MATRIX_TOF_RESPONSE_TIMEOUT_MS) {
        disconnectRearSensor(true);
      } else if (!pollRearResponseStatus()) {
        disconnectRearSensor(false);
      }
      break;

    case REAR_IO_READ_PAYLOAD:
      if (now - rearResponseStartedMs > REAR_MATRIX_TOF_RESPONSE_TIMEOUT_MS) {
        disconnectRearSensor(true);
      } else if (!readRearPayloadChunk()) {
        disconnectRearSensor(false);
      }
      break;
  }
}

bool hasTrustedRearCoverage() {
  const RangeSensorState &sensor = rangeSensors[RANGE_FAKE_REAR];
  return rearConnected && rearInitialized && sensor.valid && !sensor.stale &&
         millis() - sensor.lastReadMs <= REAR_MATRIX_TOF_STALE_TIMEOUT_MS;
}

bool getRearObstacleRay(uint8_t column, uint16_t &distanceMm,
                        float &robotAngleDeg) {
  if (column >= REAR_MATRIX_TOF_COLUMN_COUNT ||
      !hasTrustedRearCoverage() || !rearColumnValid[column]) {
    return false;
  }

  const float columnWidthDeg =
    REAR_MATRIX_TOF_HORIZONTAL_FOV_DEG / REAR_MATRIX_TOF_COLUMN_COUNT;
  const float sensorViewOffsetDeg =
    REAR_MATRIX_TOF_HORIZONTAL_FOV_DEG * 0.5f -
    (column + 0.5f) * columnWidthDeg;
  distanceMm = rearColumnDistancesMm[column];
  // Column zero is image-left. On a rear-facing sensor that points toward
  // robot-right, which is the positive offset from the 180-degree rear axis.
  robotAngleDeg = REAR_MATRIX_TOF_GEOMETRY.angleDeg + sensorViewOffsetDeg;
  return true;
}

uint32_t getRearObstacleFrameSequence() {
  return rearFrameSequence;
}

void printRearObstacleStatus() {
  const RangeSensorState &sensor = rangeSensors[RANGE_FAKE_REAR];
  const unsigned long ageMs = sensor.lastReadMs > 0
    ? millis() - sensor.lastReadMs : 0;
  Serial2.print("rear_matrix_tof,connected=");
  Serial2.print(rearConnected ? 1 : 0);
  Serial2.print(",age_ms=");
  Serial2.print(ageMs);
  Serial2.print(",valid_cells=");
  Serial2.print(rearValidCellCount);
  Serial2.print(",valid_rays=");
  Serial2.print(rearValidColumnCount);
  Serial2.print(",minimum_mm=");
  Serial2.print(rearMinimumDistanceMm);
  Serial2.print(",blocked=");
  Serial2.print(sensor.blocked ? 1 : 0);
  Serial2.print(",read_us=");
  Serial2.println(rearReadDurationUs);
}
