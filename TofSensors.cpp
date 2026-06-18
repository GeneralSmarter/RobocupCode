#include "Robot.h"

// =====================================================
// TOF sensors
// =====================================================
static void syncLegacyTofGlobals() {
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
  sensor.distanceMm = 9999;
  sensor.valid = false;
  sensor.stale = false;
  sensor.timeoutCount++;

  if (id == RANGE_FRONT) {
    sensor.blocked = true;
  }

  syncLegacyTofGlobals();
  sendBluetoothEvent("tof_timeout", sensor.name);
}

static bool isTofStale(bool valid, unsigned long lastReadMs, unsigned long now) {
  return valid && (now - lastReadMs > TOF_STALE_TIMEOUT_MS);
}

static void updateTofStaleFlags() {
  unsigned long now = millis();

  if (isTofStale(frontTofValid, lastFrontTofReadMs, now)) {
    stopMotors();
    rangeSensors[RANGE_FRONT].valid = false;
    rangeSensors[RANGE_FRONT].stale = true;
    rangeSensors[RANGE_FRONT].blocked = true;
    syncLegacyTofGlobals();
    Serial.println("Front TOF stale. Motors stopped and front marked blocked.");
    sendBluetoothEvent("tof_stale", rangeSensors[RANGE_FRONT].name);
  }

  if (isTofStale(leftTofValid, lastLeftTofReadMs, now)) {
    stopMotors();
    rangeSensors[RANGE_LEFT].valid = false;
    rangeSensors[RANGE_LEFT].stale = true;
    syncLegacyTofGlobals();
    Serial.println("Left TOF stale. Motors stopped.");
    sendBluetoothEvent("tof_stale", rangeSensors[RANGE_LEFT].name);
  }

  if (isTofStale(rightTofValid, lastRightTofReadMs, now)) {
    stopMotors();
    rangeSensors[RANGE_RIGHT].valid = false;
    rangeSensors[RANGE_RIGHT].stale = true;
    syncLegacyTofGlobals();
    Serial.println("Right TOF stale. Motors stopped.");
    sendBluetoothEvent("tof_stale", rangeSensors[RANGE_RIGHT].name);
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

  io.pinMode(FRONT_XSHUT, OUTPUT);
  io.pinMode(LEFT_XSHUT, OUTPUT);
  io.pinMode(RIGHT_XSHUT, OUTPUT);

  io.digitalWrite(FRONT_XSHUT, LOW);
  io.digitalWrite(LEFT_XSHUT, LOW);
  io.digitalWrite(RIGHT_XSHUT, LOW);
  delay(100);

  connectLeftTOF();
  connectRightTOF();
  connectFrontTOF();
}

void connectLeftTOF() {
  while (true) {
    Serial.println("Starting LEFT VL53L0X...");

    io.digitalWrite(LEFT_XSHUT, HIGH);
    delay(100);

    leftTOF.setTimeout(100);

    if (leftTOF.init()) {
      leftTOF.setAddress(LEFT_L0_ADDRESS);
      leftTOF.startContinuous(50);
      setRangeSensorReading(RANGE_LEFT, 9999, false);

      Serial.print("LEFT VL53L0X connected at 0x");
      Serial.println(LEFT_L0_ADDRESS, HEX);
      break;
    }

    Serial.println("LEFT VL53L0X not detected. Retrying...");
    io.digitalWrite(LEFT_XSHUT, LOW);
    delay(1000);
  }
}

void connectRightTOF() {
  while (true) {
    Serial.println("Starting RIGHT VL53L0X...");

    io.digitalWrite(RIGHT_XSHUT, HIGH);
    delay(100);

    rightTOF.setTimeout(100);

    if (rightTOF.init()) {
      rightTOF.setAddress(RIGHT_L0_ADDRESS);
      rightTOF.startContinuous(50);
      setRangeSensorReading(RANGE_RIGHT, 9999, false);

      Serial.print("RIGHT VL53L0X connected at 0x");
      Serial.println(RIGHT_L0_ADDRESS, HEX);
      break;
    }

    Serial.println("RIGHT VL53L0X not detected. Retrying...");
    io.digitalWrite(RIGHT_XSHUT, LOW);
    delay(1000);
  }
}

void connectFrontTOF() {
  while (true) {
    Serial.println("Starting FRONT VL53L1X...");

    io.digitalWrite(FRONT_XSHUT, HIGH);
    delay(100);

    if (frontTOF.init()) {
      Serial.println("FRONT VL53L1X connected.");

      frontTOF.setDistanceMode(VL53L1X::Long);
      frontTOF.setMeasurementTimingBudget(50000);
      frontTOF.startContinuous(50);
      setRangeSensorReading(RANGE_FRONT, 9999, false);

      break;
    }

    Serial.println("FRONT VL53L1X not detected. Retrying...");
    io.digitalWrite(FRONT_XSHUT, LOW);
    delay(1000);
  }
}

void updateTOFSensors() {
  updateFrontTOF();
  updateSideTOFSensors();
  updateTofStaleFlags();
}

void updateFrontTOF() {
  uint16_t rawDistance = frontTOF.read();

  if (frontTOF.timeoutOccurred()) {
    stopMotors();
    markRangeSensorTimeout(RANGE_FRONT);
    Serial.println("Front TOF timeout. Reconnecting...");
    connectFrontTOF();
    frontBlockCounter = 0;
    frontClearCounter = 0;
    return;
  }

  bool validFrontReading = rawDistance >= FRONT_VALID_MIN_MM && rawDistance <= FRONT_VALID_MAX_MM;

  if (!validFrontReading) {
    setRangeSensorReading(RANGE_FRONT, rawDistance, false);
    frontBlockCounter = 0;
    frontClearCounter = 0;

    if (millis() - lastFrontInvalidPrintMs > 1000) {
      Serial.print("Front TOF invalid reading ignored: ");
      Serial.print(rawDistance);
      Serial.println(" mm");
      lastFrontInvalidPrintMs = millis();
    }

    return;
  }

  setRangeSensorReading(RANGE_FRONT, rawDistance, true);

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

void updateSideTOFSensors() {
  uint16_t rawLeftDistance = leftTOF.readRangeContinuousMillimeters();
  if (leftTOF.timeoutOccurred()) {
    stopMotors();
    markRangeSensorTimeout(RANGE_LEFT);
    Serial.println("Left TOF timeout. Reconnecting...");
    connectLeftTOF();
  } else {
    setRangeSensorReading(RANGE_LEFT, rawLeftDistance, true);
  }

  uint16_t rawRightDistance = rightTOF.readRangeContinuousMillimeters();
  if (rightTOF.timeoutOccurred()) {
    stopMotors();
    markRangeSensorTimeout(RANGE_RIGHT);
    Serial.println("Right TOF timeout. Reconnecting...");
    connectRightTOF();
  } else {
    setRangeSensorReading(RANGE_RIGHT, rawRightDistance, true);
  }
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
