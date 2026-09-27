#include "../../Robot.h"

// The internal-funnel ToF publishes payload evidence only. It is intentionally
// absent from RangeSensorId and is never read by mapping, navigation safety, or
// MotorControl.cpp.
namespace {

unsigned long lastPayloadReconnectAttemptMs = 0;
unsigned long payloadReconnectPowerOnMs = 0;
bool payloadReconnectPending = false;

void markPayloadUnavailable(bool timeout) {
  io.digitalWrite(PAYLOAD_TOF_CONFIG.xshutChannel, LOW);
  payloadTofObservation.connected = false;
  payloadTofObservation.valid = false;
  payloadTofObservation.stale = true;
  payloadTofObservation.distanceMm = RANGE_NO_READING_MM;
  payloadTofObservation.rangeStatus = SENSOR_RANGE_STATUS_UNKNOWN;
  if (timeout) {
    sendBluetoothEvent("payload_tof_timeout", "payload_unknown");
  }
}

bool finishPayloadTofConnection() {
  payloadTof.setTimeout(50);
  if (!payloadTof.init()) {
    io.digitalWrite(PAYLOAD_TOF_CONFIG.xshutChannel, LOW);
    markPayloadUnavailable(false);
    return false;
  }
  payloadTof.setAddress(PAYLOAD_TOF_CONFIG.i2cAddress);
  payloadTof.setDistanceMode(VL53L1X::Short);
  payloadTof.setMeasurementTimingBudget(PAYLOAD_TOF_CONFIG.timingBudgetUs);
  payloadTof.startContinuous(PAYLOAD_TOF_CONFIG.samplePeriodMs);
  payloadTofObservation.connected = true;
  payloadTofObservation.valid = false;
  payloadTofObservation.stale = true;
  payloadTofObservation.distanceMm = RANGE_NO_READING_MM;
  payloadTofObservation.rangeStatus = SENSOR_RANGE_STATUS_UNKNOWN;
  return true;
}

}  // namespace

void preparePayloadTofPinForStartup() {
  io.pinMode(PAYLOAD_TOF_CONFIG.xshutChannel, OUTPUT);
  io.digitalWrite(PAYLOAD_TOF_CONFIG.xshutChannel, LOW);
}

void connectPayloadTofSensor() {
  preparePayloadTofPinForStartup();
  io.digitalWrite(PAYLOAD_TOF_CONFIG.xshutChannel, HIGH);
  delay(10);
  Serial.println("Starting internal-funnel payload VL53L1X...");
  const bool connected = finishPayloadTofConnection();
  Serial.println(connected
    ? "Internal-funnel payload ToF connected."
    : "Internal-funnel payload ToF unavailable; payload remains unknown.");
}

void updatePayloadTofSensor() {
  const unsigned long now = millis();
  if (payloadTofObservation.connected && payloadTof.dataReady()) {
    const uint16_t distanceMm = payloadTof.read(false);
    if (payloadTof.timeoutOccurred()) {
      markPayloadUnavailable(true);
    } else {
      VL53L1X::RangingData &data = payloadTof.ranging_data;
      payloadTofObservation.sequence++;
      if (payloadTofObservation.sequence == 0) {
        payloadTofObservation.sequence = 1;
      }
      payloadTofObservation.distanceMm = distanceMm;
      payloadTofObservation.rangeStatus = data.range_status;
      payloadTofObservation.signalMcps = data.peak_signal_count_rate_MCPS;
      payloadTofObservation.ambientMcps = data.ambient_count_rate_MCPS;
      payloadTofObservation.acquiredMs = now;
      payloadTofObservation.valid =
        data.range_status == VL53L1X::RangeValid &&
        distanceMm >= PAYLOAD_TOF_CONFIG.validMinimumMm &&
        distanceMm <= PAYLOAD_TOF_CONFIG.validMaximumMm;
      payloadTofObservation.stale = false;
    }
  }

  if (payloadTofObservation.connected &&
      payloadTofObservation.acquiredMs > 0 &&
      now - payloadTofObservation.acquiredMs > PAYLOAD_TOF_STALE_TIMEOUT_MS) {
    payloadTofObservation.valid = false;
    payloadTofObservation.stale = true;
  }

  if (!payloadTofObservation.connected && !payloadReconnectPending &&
      now - lastPayloadReconnectAttemptMs >=
        PAYLOAD_TOF_RECONNECT_INTERVAL_MS) {
    lastPayloadReconnectAttemptMs = now;
    io.digitalWrite(PAYLOAD_TOF_CONFIG.xshutChannel, HIGH);
    payloadReconnectPowerOnMs = now;
    payloadReconnectPending = true;
  }
  if (payloadReconnectPending && now - payloadReconnectPowerOnMs >= 10) {
    finishPayloadTofConnection();
    payloadReconnectPending = false;
  }
}

bool getPayloadTofObservation(PayloadTofObservation &observation) {
  const unsigned long now = millis();
  observation = payloadTofObservation;
  observation.stale = !observation.connected || observation.stale ||
    observation.acquiredMs == 0 ||
    now - observation.acquiredMs > PAYLOAD_TOF_STALE_TIMEOUT_MS;
  observation.valid = observation.valid && !observation.stale;
  return observation.valid;
}

void printPayloadTofStatus() {
  PayloadTofObservation observation;
  getPayloadTofObservation(observation);
  Serial2.print("payload_tof,x_mm=");
  Serial2.print(PAYLOAD_TOF_CONFIG.mount.xMm, 1);
  Serial2.print(",y_mm=");
  Serial2.print(PAYLOAD_TOF_CONFIG.mount.yMm, 1);
  Serial2.print(",z_mm=");
  Serial2.print(PAYLOAD_TOF_CONFIG.mount.zMm, 1);
  Serial2.print(",xshut=");
  Serial2.print(PAYLOAD_TOF_CONFIG.xshutChannel);
  Serial2.print(",address=0x");
  Serial2.print(PAYLOAD_TOF_CONFIG.i2cAddress, HEX);
  Serial2.print(",connected=");
  Serial2.print(observation.connected ? 1 : 0);
  Serial2.print(",valid=");
  Serial2.print(observation.valid ? 1 : 0);
  Serial2.print(",stale=");
  Serial2.print(observation.stale ? 1 : 0);
  Serial2.print(",distance_mm=");
  Serial2.print(observation.distanceMm);
  Serial2.print(",status=");
  Serial2.print(observation.rangeStatus);
  Serial2.print(",sequence=");
  Serial2.print(observation.sequence);
  Serial2.print(",age_ms=");
  Serial2.print(observation.acquiredMs > 0
    ? millis() - observation.acquiredMs : 0);
  Serial2.print(",signal_mcps=");
  Serial2.print(observation.signalMcps, 3);
  Serial2.print(",ambient_mcps=");
  Serial2.println(observation.ambientMcps, 3);
}
