#include "../../Robot.h"
#include "RearTofPolicy.h"
#include <limits.h>

namespace {

uint8_t nextRearTofToUpdate = 0;
uint8_t nextRearTofToReconnect = 0;
unsigned long lastReconnectAttemptMs = 0;
uint32_t rearAggregateSequence = 0;
uint32_t lastPublishedRearSequence[REAR_TOF_COUNT] = {0, 0, 0};
int8_t pendingReconnectId = -1;
unsigned long pendingReconnectPowerOnMs = 0;
RearTofHysteresisState rearHysteresis[REAR_TOF_COUNT] = {
  {true, 0, 0}, {true, 0, 0}, {true, 0, 0}
};

bool rearStateCurrent(RearTofId id, unsigned long now) {
  const RearTofState &state = rearTofStates[id];
  return state.connected && state.valid && !state.stale &&
         now - state.acquiredMs <= REAR_TOF_STALE_TIMEOUT_MS;
}

void publishRearAggregate() {
  const unsigned long now = millis();
  RangeSensorState &aggregate = rangeSensors[RANGE_REAR_AGGREGATE];
  bool allCurrent = true;
  bool anyBlocked = false;
  uint16_t minimumMm = RANGE_NO_READING_MM;
  unsigned long oldestMs = ULONG_MAX;
  unsigned long newestMs = 0;

  for (uint8_t i = 0; i < REAR_TOF_COUNT; i++) {
    const RearTofState &state = rearTofStates[i];
    if (!rearStateCurrent((RearTofId)i, now)) {
      allCurrent = false;
    }
    if (state.blocked) {
      anyBlocked = true;
    }
    if (state.valid && state.distanceMm < minimumMm) {
      minimumMm = state.distanceMm;
    }
    if (state.acquiredMs > 0 && state.acquiredMs < oldestMs) {
      oldestMs = state.acquiredMs;
    }
    if (state.acquiredMs > newestMs) {
      newestMs = state.acquiredMs;
    }
  }

  const bool coherent = allCurrent && oldestMs != ULONG_MAX &&
    rearTofSamplesCoherent(oldestMs, newestMs, REAR_TOF_MAX_SAMPLE_SKEW_MS);
  aggregate.distanceMm = minimumMm;
  aggregate.valid = coherent;
  aggregate.stale = !coherent;
  aggregate.blocked = !coherent || anyBlocked;
  aggregate.lastReadMs = coherent ? oldestMs : newestMs;
  bool allChannelsFreshForFrame = coherent;
  for (uint8_t i = 0; i < REAR_TOF_COUNT; i++) {
    allChannelsFreshForFrame = allChannelsFreshForFrame &&
      rearTofStates[i].sequence != lastPublishedRearSequence[i];
  }
  if (allChannelsFreshForFrame) {
    rearAggregateSequence++;
    if (rearAggregateSequence == 0) {
      rearAggregateSequence = 1;
    }
    for (uint8_t i = 0; i < REAR_TOF_COUNT; i++) {
      lastPublishedRearSequence[i] = rearTofStates[i].sequence;
    }
  }
}

void markRearUnavailable(RearTofId id, bool timeout) {
  RearTofState &state = rearTofStates[id];
  state.valid = false;
  state.stale = true;
  state.blocked = true;
  state.distanceMm = RANGE_NO_READING_MM;
  state.rangeStatus = SENSOR_RANGE_STATUS_UNKNOWN;
  if (timeout) {
    state.timeoutCount++;
  } else {
    state.invalidCount++;
  }
  rearHysteresis[id].blocked = true;
  rearHysteresis[id].consecutiveClearSamples = 0;
}

bool finishRearTofConnection(RearTofId id) {
  const RearTofConfig &config = REAR_TOF_CONFIG[id];
  RearTofState &state = rearTofStates[id];
  VL53L1X &sensor = rearTofs[id];

  sensor.setTimeout(50);
  if (!sensor.init()) {
    io.digitalWrite(config.xshutChannel, LOW);
    state.connected = false;
    markRearUnavailable(id, false);
    return false;
  }

  sensor.setAddress(config.i2cAddress);
  sensor.setDistanceMode(VL53L1X::Short);
  sensor.setMeasurementTimingBudget(config.timingBudgetUs);
  sensor.setROISize(config.roiWidth, config.roiHeight);
  sensor.startContinuous(config.samplePeriodMs);
  state.connected = true;
  state.valid = false;
  state.stale = true;
  state.blocked = true;
  state.distanceMm = RANGE_NO_READING_MM;
  state.rangeStatus = SENSOR_RANGE_STATUS_UNKNOWN;
  return true;
}

bool connectRearTofAtStartup(RearTofId id) {
  io.digitalWrite(REAR_TOF_CONFIG[id].xshutChannel, HIGH);
  delay(10);
  return finishRearTofConnection(id);
}

void expireRearSamples(unsigned long now) {
  for (uint8_t i = 0; i < REAR_TOF_COUNT; i++) {
    RearTofState &state = rearTofStates[i];
    if (state.valid && now - state.acquiredMs > REAR_TOF_STALE_TIMEOUT_MS) {
      state.valid = false;
      state.stale = true;
      state.blocked = true;
      rearHysteresis[i].blocked = true;
      rearHysteresis[i].consecutiveClearSamples = 0;
    }
  }
}

}  // namespace

void prepareRearTofPinsForStartup() {
  for (uint8_t i = 0; i < REAR_TOF_COUNT; i++) {
    io.pinMode(REAR_TOF_CONFIG[i].xshutChannel, OUTPUT);
    io.digitalWrite(REAR_TOF_CONFIG[i].xshutChannel, LOW);
  }
}

void connectRearTofArray() {
  prepareRearTofPinsForStartup();
  for (uint8_t i = 0; i < REAR_TOF_COUNT; i++) {
    Serial.print("Starting ");
    Serial.print(REAR_TOF_CONFIG[i].name);
    Serial.println(" VL53L1X...");
    const bool connected = connectRearTofAtStartup((RearTofId)i);
    Serial.print(REAR_TOF_CONFIG[i].name);
    Serial.println(connected ? " connected." : " unavailable; rear blocked.");
  }
  publishRearAggregate();
}

void updateRearTofArray() {
  const unsigned long now = millis();
  expireRearSamples(now);

  RearTofId id = (RearTofId)nextRearTofToUpdate;
  nextRearTofToUpdate = (nextRearTofToUpdate + 1U) % REAR_TOF_COUNT;
  RearTofState &state = rearTofStates[id];
  VL53L1X &sensor = rearTofs[id];

  if (state.connected && sensor.dataReady()) {
    const uint16_t distanceMm = sensor.read(false);
    if (sensor.timeoutOccurred()) {
      markRearUnavailable(id, true);
    } else {
      VL53L1X::RangingData &data = sensor.ranging_data;
      state.sequence++;
      if (state.sequence == 0) {
        state.sequence = 1;
      }
      state.distanceMm = distanceMm;
      state.rangeStatus = data.range_status;
      state.signalMcps = data.peak_signal_count_rate_MCPS;
      state.ambientMcps = data.ambient_count_rate_MCPS;
      state.acquiredMs = now;
      state.valid = data.range_status == VL53L1X::RangeValid &&
        distanceMm >= REAR_TOF_CONFIG[id].validMinimumMm &&
        distanceMm <= REAR_TOF_CONFIG[id].validMaximumMm;
      state.stale = false;
      if (!state.valid) {
        state.invalidCount++;
      }
      rearHysteresis[id] = updateRearTofHysteresis(
        rearHysteresis[id], state.valid, distanceMm, state.sequence,
        REAR_TOF_CONFIG[id].stopDistanceMm,
        REAR_TOF_CONFIG[id].clearDistanceMm,
        REAR_TOF_CLEAR_CONFIRM_SAMPLES);
      state.blocked = rearHysteresis[id].blocked;
    }
  }

  if (now - lastReconnectAttemptMs >= REAR_TOF_RECONNECT_INTERVAL_MS) {
    lastReconnectAttemptMs = now;
    RearTofId reconnectId = (RearTofId)nextRearTofToReconnect;
    nextRearTofToReconnect =
      (nextRearTofToReconnect + 1U) % REAR_TOF_COUNT;
    if (!rearTofStates[reconnectId].connected && pendingReconnectId < 0) {
      io.digitalWrite(REAR_TOF_CONFIG[reconnectId].xshutChannel, HIGH);
      pendingReconnectId = (int8_t)reconnectId;
      pendingReconnectPowerOnMs = now;
    }
  }

  // Runtime reconnection is a two-step cooperative operation. The sensor gets
  // its required reset-release interval without delaying the control loop.
  if (pendingReconnectId >= 0 && now - pendingReconnectPowerOnMs >= 10) {
    const RearTofId reconnectId = (RearTofId)pendingReconnectId;
    finishRearTofConnection(reconnectId);
    pendingReconnectId = -1;
  }

  publishRearAggregate();
}

bool hasTrustedRearCoverage() {
  const RangeSensorState &aggregate = rangeSensors[RANGE_REAR_AGGREGATE];
  return aggregate.valid && !aggregate.stale && !aggregate.blocked &&
         millis() - aggregate.lastReadMs <= REAR_TOF_STALE_TIMEOUT_MS;
}

bool getRearTofRay(RearTofId id, RangeRayObservation &observation) {
  if (id >= REAR_TOF_COUNT || !rearStateCurrent(id, millis())) {
    return false;
  }
  const RearTofConfig &config = REAR_TOF_CONFIG[id];
  const RearTofState &state = rearTofStates[id];
  observation = {config.mount.xMm, config.mount.yMm, config.mount.zMm,
                 config.mount.yawDeg, config.mount.pitchDeg,
                 state.distanceMm, state.sequence, state.acquiredMs};
  return true;
}

uint32_t getRearObstacleFrameSequence() {
  return rearAggregateSequence;
}

void printRearTofStatus() {
  unsigned long oldestMs = ULONG_MAX;
  unsigned long newestMs = 0;
  Serial2.println("rear_id,name,x_mm,y_mm,z_mm,yaw_deg,xshut,address,distance_mm,status,valid,stale,blocked,age_ms,sequence,signal_mcps,ambient_mcps");
  for (uint8_t i = 0; i < REAR_TOF_COUNT; i++) {
    const RearTofConfig &config = REAR_TOF_CONFIG[i];
    const RearTofState &state = rearTofStates[i];
    Serial2.print(i); Serial2.print(",");
    Serial2.print(config.name); Serial2.print(",");
    Serial2.print(config.mount.xMm, 3); Serial2.print(",");
    Serial2.print(config.mount.yMm, 3); Serial2.print(",");
    Serial2.print(config.mount.zMm, 3); Serial2.print(",");
    Serial2.print(config.mount.yawDeg, 1); Serial2.print(",");
    Serial2.print(config.xshutChannel); Serial2.print(",0x");
    Serial2.print(config.i2cAddress, HEX); Serial2.print(",");
    Serial2.print(state.distanceMm); Serial2.print(",");
    Serial2.print(state.rangeStatus); Serial2.print(",");
    Serial2.print(state.valid ? 1 : 0); Serial2.print(",");
    Serial2.print(state.stale ? 1 : 0); Serial2.print(",");
    Serial2.print(state.blocked ? 1 : 0); Serial2.print(",");
    Serial2.print(state.acquiredMs > 0 ? millis() - state.acquiredMs : 0);
    Serial2.print(","); Serial2.print(state.sequence); Serial2.print(",");
    Serial2.print(state.signalMcps, 3); Serial2.print(",");
    Serial2.println(state.ambientMcps, 3);
    if (state.acquiredMs > 0 && state.acquiredMs < oldestMs) oldestMs = state.acquiredMs;
    if (state.acquiredMs > newestMs) newestMs = state.acquiredMs;
  }
  Serial2.print("rear_aggregate,valid=");
  Serial2.print(rangeSensors[RANGE_REAR_AGGREGATE].valid ? 1 : 0);
  Serial2.print(",blocked=");
  Serial2.print(rangeSensors[RANGE_REAR_AGGREGATE].blocked ? 1 : 0);
  Serial2.print(",minimum_mm=");
  Serial2.print(rangeSensors[RANGE_REAR_AGGREGATE].distanceMm);
  Serial2.print(",sample_skew_ms=");
  Serial2.println(oldestMs == ULONG_MAX ? 0 : newestMs - oldestMs);
}
