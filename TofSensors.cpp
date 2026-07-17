#include "Robot.h"

// =====================================================
// TOF sensors
// =====================================================
// Responsibility:
//   Owns the high forward VL53L0X navigation fan, derived legacy aggregate
//   readings, front-block debounce, stale/timeout handling, and the temporary
//   fake rear ToF scaffold.
// Interacts with:
//   RobotCode.ino calls connectTOFSensors() during setup. LocalPlanner.cpp and
//   MotorControl.cpp read rangeSensors through the accessor functions.
//   Bluetooth.cpp prints raw fan/aggregate telemetry. ObjectDetection.cpp owns
//   the separate VL53L1X object sensors.
// Control flow:
//   updateRobotController() calls updateTOFSensors() on a 20 ms schedule.
//   Sensor reads are polled for readiness before using the library read call
//   so the control loop does not block behind four long ranging waits.
// Global state:
//   Modifies rangeSensors, legacy front/left/right globals, frontBlocked,
//   front debounce counters, pending-close revalidation state, and fake rear
//   validity/blocked fields.
const uint16_t TOF_NO_READING_MM = RANGE_NO_READING_MM;
const uint16_t L0X_VALID_MAX_MM = 8191;
static uint16_t pendingCloseDistanceMm[RANGE_SENSOR_COUNT] = {0};
static uint8_t pendingCloseReadCount[RANGE_SENSOR_COUNT] = {0};
static bool tofCloseReadingRevalidating = false;

static void updateFrontBlockState();
static void updateFakeRearTofSensor();

static bool isPhysicalFanSensor(RangeSensorId id) {
  return id == RANGE_RIGHT_OUTER ||
         id == RANGE_RIGHT_INNER ||
         id == RANGE_LEFT_INNER ||
         id == RANGE_LEFT_OUTER;
}

static const char* fanModelName(RangeSensorId id) {
  return "VL53L0X";
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
      return INVALID_XSHUT_PIN;
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

static bool isValidL0XDistance(uint16_t distanceMm) {
  return distanceMm >= FRONT_VALID_MIN_MM && distanceMm <= L0X_VALID_MAX_MM;
}

static bool isValidFakeRearTofDistance(uint16_t distanceMm) {
  return distanceMm >= FAKE_REAR_TOF_VALID_MIN_MM &&
         distanceMm <= FAKE_REAR_TOF_VALID_MAX_MM;
}

static unsigned long maxReadTime(unsigned long a, unsigned long b) {
  return a > b ? a : b;
}

static bool isUsableAggregateReading(const RangeSensorState &sensor) {
  if (sensor.valid) {
    return true;
  }

  // A ToF can report very small values when an object is pressed close to the
  // beam. Treat that as unsafe clearance even though it is below the normal
  // calibrated distance window.
  return sensor.distanceMm > 0 && sensor.distanceMm < FRONT_STOP_DISTANCE_MM;
}

static void combineFanAggregateSensor(RangeSensorId aggregateId, RangeSensorId innerId, RangeSensorId outerId) {
  // Legacy front/left/right telemetry is derived from the physical fan. The
  // planner itself uses the individual rays, but this aggregate preserves a
  // simple nearest-obstacle view for front-stop and operator telemetry.
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
  // Inputs:
  //   distanceMm is a raw millimetre range from a ToF channel or synthetic
  //   fake rear source. valid means it falls inside that sensor's calibrated
  //   usable range, not necessarily that the path is safe.
  //
  // Global effects:
  //   Updates rangeSensors[id], pending close-read filters, legacy aggregate
  //   front/left/right state, and frontBlocked through sync/update calls.
  RangeSensorState &sensor = rangeSensors[id];

  // A fan beam cannot physically move hundreds of millimetres closer in one
  // 20 ms sample while the robot is moving at its permitted speed.  Hold a
  // sudden close return until a second, similar reading confirms it.  Motion
  // is stopped during that one-sample recheck, so this removes I2C/ToF
  // outliers without treating them as free space.
  // Only a very large inward jump is delayed. A gradual approach to a real
  // wall remains responsive; the filter is aimed at isolated impossible drops.
  if (valid && sensor.valid &&
      distanceMm + TOF_SUDDEN_CLOSE_DROP_MM < sensor.distanceMm) {
    uint8_t index = (uint8_t)id;
    uint16_t pending = pendingCloseDistanceMm[index];
    if (pendingCloseReadCount[index] == 0 ||
        abs((int)distanceMm - (int)pending) > TOF_CLOSE_CONFIRM_TOLERANCE_MM) {
      pendingCloseDistanceMm[index] = distanceMm;
      pendingCloseReadCount[index] = 1;
    } else {
      pendingCloseReadCount[index]++;
    }

    if (pendingCloseReadCount[index] < TOF_CLOSE_CONFIRM_READS) {
      tofCloseReadingRevalidating = true;
      return;
    }
  }

  pendingCloseReadCount[(uint8_t)id] = 0;
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
  // A timeout means the channel did not provide a trustworthy measurement in
  // time. Mark it invalid and let the safety policy treat unknown space as
  // blocked where that direction is needed.
  RangeSensorState &sensor = rangeSensors[id];
  pendingCloseReadCount[(uint8_t)id] = 0;
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
  // Called every sensor cycle. A stale valid reading is worse than no reading
  // because it can look like fresh clearance. Stop first, then mark invalid.
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
  // Brings up one physical VL53L0X. All sensors share I2C, so each one is
  // held in reset with XSHUT until it can be assigned a unique address.
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

void connectTOFSensors() {
  // Initializes the SX1509 GPIO expander, holds every ToF in reset, then
  // starts each fan sensor one at a time. This runs before the robot can move.
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
  prepareObjectTOFPinsForStartup();

  delay(100);

  connectRightOuterTOF();
  connectRightInnerTOF();
  connectLeftInnerTOF();
  connectLeftOuterTOF();
  connectObjectTOFSensors();
  updateFakeRearTofSensor();
}

void connectRightOuterTOF() {
  connectL0XFanSensor(rightOuterTOF, RANGE_RIGHT_OUTER);
}

void connectRightInnerTOF() {
  connectL0XFanSensor(rightInnerTOF, RANGE_RIGHT_INNER);
}

void connectLeftInnerTOF() {
  connectL0XFanSensor(leftInnerTOF, RANGE_LEFT_INNER);
}

void connectLeftOuterTOF() {
  connectL0XFanSensor(leftOuterTOF, RANGE_LEFT_OUTER);
}

static void updateFrontBlockState() {
  // The virtual front is the nearer inner +/-20 degree ray. It has its own
  // debounce so a genuine front obstacle is not lost between local-map updates.
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

static void updateFakeRearTofSensor() {
  // SAFETY: This is explicit unsafe test scaffolding, not real rear sensing.
  // It keeps the current recovery API wired during software tests but cannot
  // prove physical rear clearance.
  setRangeSensorReading(RANGE_FAKE_REAR,
                        FAKE_REAR_TOF_DISTANCE_MM,
                        isValidFakeRearTofDistance(FAKE_REAR_TOF_DISTANCE_MM));
  RangeSensorState &sensor = rangeSensors[RANGE_FAKE_REAR];
  sensor.blocked = sensor.valid &&
                   sensor.distanceMm < FAKE_REAR_TOF_STOP_DISTANCE_MM;
  if (sensor.valid && sensor.distanceMm > FAKE_REAR_TOF_CLEAR_DISTANCE_MM) {
    sensor.blocked = false;
  }
  sensor.stale = false;
  syncLegacyTofGlobals();
}

bool hasTrustedRearCoverage() {
  // The installed build still exposes RANGE_FAKE_REAR for diagnostics only.
  // It must never satisfy reverse motion safety or planner evidence.
  return false;
}

static bool isL0XFanSampleReady(VL53L0X &sensor) {
  return (sensor.readReg(VL53L0X::RESULT_INTERRUPT_STATUS) & 0x07) != 0;
}

static void updateL0XFanSensor(RangeSensorId id, VL53L0X &sensor) {
  // Pololu's range-read call waits until a sample is ready. Poll its public
  // interrupt-status register first so four fan channels cannot serialize four
  // 100 ms measurement waits while an old motor command remains latched.
  if (!isL0XFanSampleReady(sensor)) {
    return;
  }
  uint16_t rawDistance = sensor.readRangeContinuousMillimeters();

  if (sensor.timeoutOccurred()) {
    stopMotors();
    markRangeSensorTimeout(id);
    Serial.print(rangeSensors[id].name);
    Serial.println(" TOF timeout. Channel marked degraded; motion policy will not assume clearance.");
    return;
  }

  setRangeSensorReading(id, rawDistance, isValidL0XDistance(rawDistance));
}

void updateTOFSensors() {
  // Full range update used by the controller. Object ToFs are updated here as
  // well so object candidates are based on the same main-loop schedule.
  updateFanTOFSensors();
  updateObjectTOFSensors();
  updateTofStaleFlags();
}

void updateFanTOFSensors() {
  // Updates the four navigation fan rays and the derived fake rear/front
  // state. The pending-close flag is reset each cycle and set only if a sudden
  // close sample is waiting for confirmation.
  tofCloseReadingRevalidating = false;
  updateL0XFanSensor(RANGE_RIGHT_OUTER, rightOuterTOF);
  updateL0XFanSensor(RANGE_RIGHT_INNER, rightInnerTOF);
  updateL0XFanSensor(RANGE_LEFT_INNER, leftInnerTOF);
  updateL0XFanSensor(RANGE_LEFT_OUTER, leftOuterTOF);
  updateFakeRearTofSensor();
  updateFrontBlockState();
}

bool isRangeSensorValid(RangeSensorId id) {
  return rangeSensors[id].valid;
}

bool isRangeSensorBlocked(RangeSensorId id) {
  return rangeSensors[id].blocked;
}

bool isRangeSensorCurrent(RangeSensorId id) {
  // "Current" means valid, not stale, and younger than the stale timeout.
  // MotorControl.cpp uses this stricter check before allowing motion.
  const RangeSensorState &sensor = rangeSensors[id];
  return sensor.valid && !sensor.stale &&
         millis() - sensor.lastReadMs <= TOF_STALE_TIMEOUT_MS;
}

bool isTofCloseReadingRevalidating() {
  return tofCloseReadingRevalidating;
}

uint16_t getRangeSensorDistance(RangeSensorId id) {
  return rangeSensors[id].distanceMm;
}

static float getRobotTurnSweepRadiusMm() {
  // Radius from drive-wheel midpoint to the farthest chassis corner. During a
  // pivot, every corner can sweep this circle, so turn safety uses this radius
  // rather than only the front edge.
  const float frontLeft = sqrtf(ROBOT_FOOTPRINT_GEOMETRY.frontExtentMm * ROBOT_FOOTPRINT_GEOMETRY.frontExtentMm +
                                ROBOT_FOOTPRINT_GEOMETRY.leftExtentMm * ROBOT_FOOTPRINT_GEOMETRY.leftExtentMm);
  const float frontRight = sqrtf(ROBOT_FOOTPRINT_GEOMETRY.frontExtentMm * ROBOT_FOOTPRINT_GEOMETRY.frontExtentMm +
                                 ROBOT_FOOTPRINT_GEOMETRY.rightExtentMm * ROBOT_FOOTPRINT_GEOMETRY.rightExtentMm);
  const float rearLeft = sqrtf(ROBOT_FOOTPRINT_GEOMETRY.rearExtentMm * ROBOT_FOOTPRINT_GEOMETRY.rearExtentMm +
                               ROBOT_FOOTPRINT_GEOMETRY.leftExtentMm * ROBOT_FOOTPRINT_GEOMETRY.leftExtentMm);
  const float rearRight = sqrtf(ROBOT_FOOTPRINT_GEOMETRY.rearExtentMm * ROBOT_FOOTPRINT_GEOMETRY.rearExtentMm +
                                ROBOT_FOOTPRINT_GEOMETRY.rightExtentMm * ROBOT_FOOTPRINT_GEOMETRY.rightExtentMm);
  return max(max(frontLeft, frontRight), max(rearLeft, rearRight));
}

float getFanSweepClearanceMm(RangeSensorId id) {
  // This measures clearance to the largest chassis corner during an in-place
  // pivot. It is intentionally more conservative than straight body clearance.
  if (!isPhysicalFanSensor(id) || !isRangeSensorValid(id)) {
    return -1000000.0;
  }

  const FanSensorGeometry &sensor = FAN_SENSOR_GEOMETRY[(int)id];
  const float rangeMm = (float)getRangeSensorDistance(id);
  const float angleRad = sensor.angleDeg * DEG_TO_RAD;
  const float endpointX = sensor.xMm + rangeMm * cosf(angleRad);
  const float endpointY = sensor.yMm + rangeMm * sinf(angleRad);
  return sqrtf(endpointX * endpointX + endpointY * endpointY) -
         getRobotTurnSweepRadiusMm();
}

static float getFanFootprintClearanceMm(RangeSensorId id) {
  // For immediate straight-driving protection, measure distance from the ToF
  // endpoint to the current rectangular body, not to the turn circle. This is
  // why a narrow straight corridor can remain legal while a pivot inside it is
  // still rejected by getFanSweepClearanceMm().
  if (!isPhysicalFanSensor(id) || !isRangeSensorValid(id)) {
    return -1000000.0;
  }

  const FanSensorGeometry &sensor = FAN_SENSOR_GEOMETRY[(int)id];
  const float angleRad = sensor.angleDeg * DEG_TO_RAD;
  const float rangeMm = (float)getRangeSensorDistance(id);
  const float pointX = sensor.xMm + rangeMm * cosf(angleRad);
  const float pointY = sensor.yMm + rangeMm * sinf(angleRad);
  const float front = ROBOT_FOOTPRINT_GEOMETRY.frontExtentMm;
  const float rear = ROBOT_FOOTPRINT_GEOMETRY.rearExtentMm;
  const float left = ROBOT_FOOTPRINT_GEOMETRY.leftExtentMm;
  const float right = ROBOT_FOOTPRINT_GEOMETRY.rightExtentMm;

  float outsideX = 0.0;
  if (pointX > front) {
    outsideX = pointX - front;
  } else if (pointX < -rear) {
    outsideX = -rear - pointX;
  }

  float outsideY = 0.0;
  if (pointY > left) {
    outsideY = pointY - left;
  } else if (pointY < -right) {
    outsideY = -right - pointY;
  }

  return sqrtf(outsideX * outsideX + outsideY * outsideY);
}

bool getDiagonalClearanceWarning(RangeSensorId &sensorId, float &clearanceMm) {
  // Returns true when a current fan endpoint is too close to the present
  // rectangular footprint. This is an immediate current-pose warning used by
  // the safety supervisor before the planner has time to roll out a new arc.
  sensorId = RANGE_SENSOR_COUNT;
  clearanceMm = 1000000.0;

  for (int i = RANGE_RIGHT_OUTER; i <= RANGE_LEFT_OUTER; i++) {
    RangeSensorId id = (RangeSensorId)i;
    if (!isRangeSensorValid(id)) {
      // A very small nonzero ToF return is outside the normal calibrated
      // window but is evidence of immediate contact, never clear space.
      if (getRangeSensorDistance(id) > 0 &&
          getRangeSensorDistance(id) < FRONT_STOP_DISTANCE_MM) {
        sensorId = id;
        clearanceMm = -1000000.0;
        return true;
      }
      continue;
    }

    // Point-goal safety protects the chassis as it is now. Turn clearance is
    // evaluated separately by isTurnSweepSafe(); using the turn envelope here
    // would falsely reject a straight, pre-aligned 400 mm corridor.
    const float clearance = getFanFootprintClearanceMm(id);
    if (clearance < clearanceMm) {
      clearanceMm = clearance;
      sensorId = id;
    }
  }

  return sensorId != RANGE_SENSOR_COUNT &&
         clearanceMm < PLANNER_TOTAL_HARD_CLEARANCE_M * 1000.0f;
}

void printFanTelemetry() {
  // Stationary diagnostic used by TEST FAN. It refreshes sensors once, then
  // prints raw fan channels plus derived aggregate readings to the Bluetooth
  // link. It does not command motion.
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
  Serial2.print(isRangeSensorValid(RANGE_LEFT) ? 1 : 0);
  Serial2.print(",fake_rear_mm=");
  Serial2.print(getRangeSensorDistance(RANGE_FAKE_REAR));
  Serial2.print(",fake_rear_valid=");
  Serial2.print(isRangeSensorValid(RANGE_FAKE_REAR) ? 1 : 0);
  Serial2.print(",fake_rear_blocked=");
  Serial2.println(isRangeSensorBlocked(RANGE_FAKE_REAR) ? 1 : 0);
}
