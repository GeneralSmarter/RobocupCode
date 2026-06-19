#include "Robot.h"

// =====================================================
// TOF sensors
// =====================================================
const uint16_t TOF_NO_READING_MM = 9999;
const uint16_t L0X_VALID_MAX_MM = 8191;

static void updateFrontBlockState();

static bool isPhysicalFanSensor(RangeSensorId id) {
  return id == RANGE_RIGHT_OUTER ||
         id == RANGE_RIGHT_INNER ||
         id == RANGE_LEFT_INNER ||
         id == RANGE_LEFT_OUTER;
}

static const char* fanModelName(RangeSensorId id) {
  if (id == RANGE_RIGHT_OUTER || id == RANGE_LEFT_OUTER) {
    return "VL53L0X";
  }

  return "VL53L1X";
}

static byte fanXshutPin(RangeSensorId id) {
  switch (id) {
    case RANGE_RIGHT_OUTER:
      return RIGHT_OUTER_XSHUT;
    case RANGE_RIGHT_INNER:
      return RIGHT_INNER_XSHUT;
    case RANGE_LEFT_INNER:
      return LEFT_INNER_XSHUT;
    case RANGE_LEFT_OUTER:
      return LEFT_OUTER_XSHUT;
    default:
      return 255;
  }
}

static uint8_t fanI2cAddress(RangeSensorId id) {
  switch (id) {
    case RANGE_RIGHT_OUTER:
      return RIGHT_OUTER_ADDRESS;
    case RANGE_RIGHT_INNER:
      return RIGHT_INNER_ADDRESS;
    case RANGE_LEFT_INNER:
      return LEFT_INNER_ADDRESS;
    case RANGE_LEFT_OUTER:
      return LEFT_OUTER_ADDRESS;
    default:
      return 0;
  }
}

static bool isValidL1XDistance(uint16_t distanceMm) {
  return distanceMm >= FRONT_VALID_MIN_MM && distanceMm <= FRONT_VALID_MAX_MM;
}

static bool isValidL0XDistance(uint16_t distanceMm) {
  return distanceMm >= FRONT_VALID_MIN_MM && distanceMm <= L0X_VALID_MAX_MM;
}

static unsigned long maxReadTime(unsigned long a, unsigned long b) {
  return a > b ? a : b;
}

static bool isUsableAggregateReading(const RangeSensorState &sensor) {
  if (sensor.valid) {
    return true;
  }

  // VL53L1X can report very small values when an object is pressed close to
  // the beam. Treat that as unsafe clearance even though it is below the
  // normal calibrated distance window.
  return sensor.distanceMm > 0 && sensor.distanceMm < FRONT_STOP_DISTANCE_MM;
}

static void combineFanAggregateSensor(RangeSensorId aggregateId, RangeSensorId innerId, RangeSensorId outerId) {
  RangeSensorState &aggregate = rangeSensors[aggregateId];
  RangeSensorState &inner = rangeSensors[innerId];
  RangeSensorState &outer = rangeSensors[outerId];

  bool innerUsable = isUsableAggregateReading(inner);
  bool outerUsable = isUsableAggregateReading(outer);
  bool anyValid = innerUsable || outerUsable;
  uint16_t nearestMm = TOF_NO_READING_MM;

  if (innerUsable && inner.distanceMm < nearestMm) {
    nearestMm = inner.distanceMm;
  }

  if (outerUsable && outer.distanceMm < nearestMm) {
    nearestMm = outer.distanceMm;
  }

  aggregate.distanceMm = anyValid ? nearestMm : TOF_NO_READING_MM;
  aggregate.valid = anyValid;
  aggregate.stale = !anyValid && (inner.stale || outer.stale);
  aggregate.lastReadMs = maxReadTime(inner.lastReadMs, outer.lastReadMs);
  aggregate.timeoutCount = inner.timeoutCount + outer.timeoutCount;
  aggregate.invalidCount = inner.invalidCount + outer.invalidCount;
}

static void updateFanAggregateSensors() {
  combineFanAggregateSensor(RANGE_RIGHT, RANGE_RIGHT_INNER, RANGE_RIGHT_OUTER);
  combineFanAggregateSensor(RANGE_FRONT, RANGE_RIGHT_INNER, RANGE_LEFT_INNER);
  combineFanAggregateSensor(RANGE_LEFT, RANGE_LEFT_INNER, RANGE_LEFT_OUTER);
}

static void syncLegacyTofGlobals() {
  updateFanAggregateSensors();

  frontDistance = rangeSensors[RANGE_FRONT].distanceMm;
  leftDistance = rangeSensors[RANGE_LEFT].distanceMm;
  rightDistance = rangeSensors[RANGE_RIGHT].distanceMm;

  frontTofValid = rangeSensors[RANGE_FRONT].valid;
  leftTofValid = rangeSensors[RANGE_LEFT].valid;
  rightTofValid = rangeSensors[RANGE_RIGHT].valid;

  lastFrontTofReadMs = rangeSensors[RANGE_FRONT].lastReadMs;
  lastLeftTofReadMs = rangeSensors[RANGE_LEFT].lastReadMs;
  lastRightTofReadMs = rangeSensors[RANGE_RIGHT].lastReadMs;

  frontBlocked = rangeSensors[RANGE_FRONT].blocked;
}

static void setRangeSensorReading(RangeSensorId id, uint16_t distanceMm, bool valid) {
  RangeSensorState &sensor = rangeSensors[id];
  sensor.distanceMm = distanceMm;
  sensor.valid = valid;
  sensor.stale = false;

  if (valid) {
    sensor.lastReadMs = millis();
  } else {
    sensor.invalidCount++;
  }

  syncLegacyTofGlobals();
}

static void markRangeSensorTimeout(RangeSensorId id) {
  RangeSensorState &sensor = rangeSensors[id];
  sensor.distanceMm = TOF_NO_READING_MM;
  sensor.valid = false;
  sensor.stale = false;
  sensor.timeoutCount++;

  syncLegacyTofGlobals();
  sendBluetoothEvent("tof_timeout", sensor.name);
}

static bool isTofStale(bool valid, unsigned long lastReadMs, unsigned long now) {
  return valid && (now - lastReadMs > TOF_STALE_TIMEOUT_MS);
}

static void updateTofStaleFlags() {
  unsigned long now = millis();

  for (int i = 0; i < RANGE_SENSOR_COUNT; i++) {
    RangeSensorId id = (RangeSensorId)i;

    if (!isPhysicalFanSensor(id)) {
      continue;
    }

    RangeSensorState &sensor = rangeSensors[id];
    if (!isTofStale(sensor.valid, sensor.lastReadMs, now)) {
      continue;
    }

    stopMotors();
    sensor.valid = false;
    sensor.stale = true;

    Serial.print(sensor.name);
    Serial.println(" TOF stale. Motors stopped.");

    syncLegacyTofGlobals();
    sendBluetoothEvent("tof_stale", sensor.name);
  }

  updateFrontBlockState();
}

static void connectL0XFanSensor(VL53L0X &sensor, RangeSensorId id) {
  while (true) {
    Serial.print("Starting ");
    Serial.print(rangeSensors[id].name);
    Serial.println(" VL53L0X...");

    io.digitalWrite(fanXshutPin(id), HIGH);
    delay(100);

    sensor.setTimeout(100);

    if (sensor.init()) {
      sensor.setAddress(fanI2cAddress(id));
      sensor.startContinuous(50);
      setRangeSensorReading(id, TOF_NO_READING_MM, false);

      Serial.print(rangeSensors[id].name);
      Serial.print(" VL53L0X connected at 0x");
      Serial.println(fanI2cAddress(id), HEX);
      break;
    }

    Serial.print(rangeSensors[id].name);
    Serial.println(" VL53L0X not detected. Retrying...");
    io.digitalWrite(fanXshutPin(id), LOW);
    delay(1000);
  }
}

static void connectL1XFanSensor(VL53L1X &sensor, RangeSensorId id) {
  while (true) {
    Serial.print("Starting ");
    Serial.print(rangeSensors[id].name);
    Serial.println(" VL53L1X...");

    io.digitalWrite(fanXshutPin(id), HIGH);
    delay(100);

    sensor.setTimeout(100);

    if (sensor.init()) {
      sensor.setAddress(fanI2cAddress(id));
      sensor.setDistanceMode(VL53L1X::Long);
      sensor.setMeasurementTimingBudget(50000);
      sensor.startContinuous(50);
      setRangeSensorReading(id, TOF_NO_READING_MM, false);

      Serial.print(rangeSensors[id].name);
      Serial.print(" VL53L1X connected at 0x");
      Serial.println(fanI2cAddress(id), HEX);
      break;
    }

    Serial.print(rangeSensors[id].name);
    Serial.println(" VL53L1X not detected. Retrying...");
    io.digitalWrite(fanXshutPin(id), LOW);
    delay(1000);
  }
}

void connectTOFSensors() {
  while (true) {
    Serial.println("Connecting to SX1509...");

    if (io.begin(SX1509_ADDRESS)) {
      Serial.println("SX1509 connected.");
      break;
    }

    Serial.println("SX1509 not detected. Retrying...");
    delay(1000);
  }

  const byte fanXshutPins[] = {
    RIGHT_OUTER_XSHUT,
    RIGHT_INNER_XSHUT,
    LEFT_INNER_XSHUT,
    LEFT_OUTER_XSHUT
  };

  for (unsigned int i = 0; i < sizeof(fanXshutPins) / sizeof(fanXshutPins[0]); i++) {
    io.pinMode(fanXshutPins[i], OUTPUT);
    io.digitalWrite(fanXshutPins[i], LOW);
  }

  delay(100);

  connectRightOuterTOF();
  connectRightInnerTOF();
  connectLeftInnerTOF();
  connectLeftOuterTOF();
}

void connectRightOuterTOF() {
  connectL0XFanSensor(rightOuterTOF, RANGE_RIGHT_OUTER);
}

void connectRightInnerTOF() {
  connectL1XFanSensor(rightInnerTOF, RANGE_RIGHT_INNER);
}

void connectLeftInnerTOF() {
  connectL1XFanSensor(leftInnerTOF, RANGE_LEFT_INNER);
}

void connectLeftOuterTOF() {
  connectL0XFanSensor(leftOuterTOF, RANGE_LEFT_OUTER);
}

static void updateFrontBlockState() {
  if (!rangeSensors[RANGE_FRONT].valid) {
    frontBlockCounter = 0;
    frontClearCounter = 0;
    if (!rangeSensors[RANGE_FRONT].blocked) {
      rangeSensors[RANGE_FRONT].blocked = true;
      syncLegacyTofGlobals();
      Serial.println("FRONT BLOCKED: virtual front fan invalid.");
      sendBluetoothEvent("front_blocked", "invalid");
    }
    return;
  }

  if (!rangeSensors[RANGE_FRONT].blocked) {
    if (rangeSensors[RANGE_FRONT].distanceMm < FRONT_STOP_DISTANCE_MM) {
      frontBlockCounter++;
    } else {
      frontBlockCounter = 0;
    }

    if (frontBlockCounter >= FRONT_BLOCK_CONFIRM_READS) {
      rangeSensors[RANGE_FRONT].blocked = true;
      syncLegacyTofGlobals();
      frontBlockCounter = 0;
      frontClearCounter = 0;
      Serial.println("FRONT BLOCKED: emergency stop active.");
      sendBluetoothEvent("front_blocked", "confirmed");
    }
  } else {
    if (rangeSensors[RANGE_FRONT].distanceMm > FRONT_CLEAR_DISTANCE_MM) {
      frontClearCounter++;
    } else {
      frontClearCounter = 0;
    }

    if (frontClearCounter >= FRONT_CLEAR_CONFIRM_READS) {
      rangeSensors[RANGE_FRONT].blocked = false;
      syncLegacyTofGlobals();
      frontClearCounter = 0;
      frontBlockCounter = 0;
      Serial.println("FRONT CLEAR: movement allowed.");
      sendBluetoothEvent("front_clear", "confirmed");
    }
  }
}

static void updateL1XFanSensor(RangeSensorId id, VL53L1X &sensor, void (*reconnectSensor)()) {
  uint16_t rawDistance = sensor.read();

  if (sensor.timeoutOccurred()) {
    stopMotors();
    markRangeSensorTimeout(id);
    Serial.print(rangeSensors[id].name);
    Serial.println(" TOF timeout. Reconnecting...");
    reconnectSensor();
    return;
  }

  bool validReading = isValidL1XDistance(rawDistance);
  setRangeSensorReading(id, rawDistance, validReading);
}

static void updateL0XFanSensor(RangeSensorId id, VL53L0X &sensor, void (*reconnectSensor)()) {
  uint16_t rawDistance = sensor.readRangeContinuousMillimeters();

  if (sensor.timeoutOccurred()) {
    stopMotors();
    markRangeSensorTimeout(id);
    Serial.print(rangeSensors[id].name);
    Serial.println(" TOF timeout. Reconnecting...");
    reconnectSensor();
    return;
  }

  setRangeSensorReading(id, rawDistance, isValidL0XDistance(rawDistance));
}

void updateTOFSensors() {
  updateFanTOFSensors();
  updateTofStaleFlags();
}

void updateFanTOFSensors() {
  updateL0XFanSensor(RANGE_RIGHT_OUTER, rightOuterTOF, connectRightOuterTOF);
  updateL1XFanSensor(RANGE_RIGHT_INNER, rightInnerTOF, connectRightInnerTOF);
  updateL1XFanSensor(RANGE_LEFT_INNER, leftInnerTOF, connectLeftInnerTOF);
  updateL0XFanSensor(RANGE_LEFT_OUTER, leftOuterTOF, connectLeftOuterTOF);
  updateFrontBlockState();
}

bool isRangeSensorValid(RangeSensorId id) {
  return rangeSensors[id].valid;
}

bool isRangeSensorBlocked(RangeSensorId id) {
  return rangeSensors[id].blocked;
}

uint16_t getRangeSensorDistance(RangeSensorId id) {
  return rangeSensors[id].distanceMm;
}

bool waitForFrontClear(unsigned long timeoutMs) {
  unsigned long startMs = millis();

  while (millis() - startMs <= timeoutMs) {
    if (handleBluetoothCommands()) {
      stopMotors();
      return false;
    }

    updateTOFSensors();

    if (!isRangeSensorBlocked(RANGE_FRONT)) {
      return true;
    }

    delay(50);
  }

  updateTOFSensors();
  return !isRangeSensorBlocked(RANGE_FRONT);
}

void printFanTelemetry() {
  updateTOFSensors();

  Serial2.println("fan_idx,name,model,angle_deg,xshut,address,distance_mm,valid,blocked,stale,timeouts,invalids");

  for (int i = RANGE_RIGHT_OUTER; i <= RANGE_LEFT_OUTER; i++) {
    RangeSensorId id = (RangeSensorId)i;
    RangeSensorState &sensor = rangeSensors[id];

    Serial2.print(i);
    Serial2.print(",");
    Serial2.print(sensor.name);
    Serial2.print(",");
    Serial2.print(fanModelName(id));
    Serial2.print(",");
    Serial2.print(sensor.angleDeg);
    Serial2.print(",");
    Serial2.print(fanXshutPin(id));
    Serial2.print(",0x");
    Serial2.print(fanI2cAddress(id), HEX);
    Serial2.print(",");
    Serial2.print(sensor.distanceMm);
    Serial2.print(",");
    Serial2.print(sensor.valid ? 1 : 0);
    Serial2.print(",");
    Serial2.print(sensor.blocked ? 1 : 0);
    Serial2.print(",");
    Serial2.print(sensor.stale ? 1 : 0);
    Serial2.print(",");
    Serial2.print(sensor.timeoutCount);
    Serial2.print(",");
    Serial2.println(sensor.invalidCount);
  }

  Serial2.print("fan_aggregate,right_mm=");
  Serial2.print(getRangeSensorDistance(RANGE_RIGHT));
  Serial2.print(",right_valid=");
  Serial2.print(isRangeSensorValid(RANGE_RIGHT) ? 1 : 0);
  Serial2.print(",front_virtual_mm=");
  Serial2.print(getRangeSensorDistance(RANGE_FRONT));
  Serial2.print(",front_virtual_valid=");
  Serial2.print(isRangeSensorValid(RANGE_FRONT) ? 1 : 0);
  Serial2.print(",left_mm=");
  Serial2.print(getRangeSensorDistance(RANGE_LEFT));
  Serial2.print(",left_valid=");
  Serial2.println(isRangeSensorValid(RANGE_LEFT) ? 1 : 0);
}
