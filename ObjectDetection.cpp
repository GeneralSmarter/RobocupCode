#include "Robot.h"

// =====================================================
// Object / weight detection scaffold
// =====================================================
// Responsibility:
//   Owns the VL53L1X object/weight ToF subsystem, object candidate
//   classification, and conversion from low-sensor endpoints to a world-frame
//   pickup target.
// Interacts with:
//   TofSensors.cpp starts object pins as part of ToF setup. StateMachine.cpp
//   uses objectCandidate/objectTargetEstimate for search and hunt behavior.
//   Bluetooth.cpp exposes TEST OBJECT, TEST HUNT TARGET, TEST HUNT, and CSV
//   fields. LocalPlanner.cpp treats hunt targets as ordinary point goals.
// Control flow:
//   updateObjectTOFSensors() updates one VL53L1X channel per call to avoid a
//   long sensor burst in the main loop. refreshObjectTargetEstimate() performs
//   one bounded sweep for command-time diagnostics.
// Global state:
//   Modifies objectSensors, objectCandidate, objectTargetEstimate, and the
//   nextObjectSensorToUpdate round-robin index.
// This subsystem is intentionally separate from the high VL53L0X navigation
// fan. It can identify object candidates for slow approach, but it does not
// provide travel-safety clearance to the V7 local planner.

static uint8_t nextObjectSensorToUpdate = 0;

static byte objectXshutPin(ObjectTofId id) {
  switch (id) {
    case OBJECT_LEFT_LOW:
      return OBJECT_LEFT_LOW_XSHUT;
    case OBJECT_LEFT_UPPER:
      return OBJECT_LEFT_UPPER_XSHUT;
    case OBJECT_RIGHT_LOW:
      return OBJECT_RIGHT_LOW_XSHUT;
    case OBJECT_RIGHT_UPPER:
      return OBJECT_RIGHT_UPPER_XSHUT;
    default:
      return INVALID_XSHUT_PIN;
  }
}

static uint8_t objectI2cAddress(ObjectTofId id) {
  switch (id) {
    case OBJECT_LEFT_LOW:
      return OBJECT_LEFT_LOW_ADDRESS;
    case OBJECT_LEFT_UPPER:
      return OBJECT_LEFT_UPPER_ADDRESS;
    case OBJECT_RIGHT_LOW:
      return OBJECT_RIGHT_LOW_ADDRESS;
    case OBJECT_RIGHT_UPPER:
      return OBJECT_RIGHT_UPPER_ADDRESS;
    default:
      return 0;
  }
}

static VL53L1X* objectTof(ObjectTofId id) {
  switch (id) {
    case OBJECT_LEFT_LOW:
      return &objectLeftLowTOF;
    case OBJECT_LEFT_UPPER:
      return &objectLeftUpperTOF;
    case OBJECT_RIGHT_LOW:
      return &objectRightLowTOF;
    case OBJECT_RIGHT_UPPER:
      return &objectRightUpperTOF;
    default:
      return NULL;
  }
}

const char* objectSensorRoleName(ObjectTofRole role) {
  switch (role) {
    case OBJECT_ROLE_LOW:
      return "LOW";
    case OBJECT_ROLE_UPPER:
      return "UPPER";
  }
  return "UNKNOWN";
}

const char* objectCandidateKindName(ObjectCandidateKind kind) {
  switch (kind) {
    case OBJECT_CANDIDATE_DISABLED:
      return "disabled";
    case OBJECT_CANDIDATE_NONE:
      return "none";
    case OBJECT_CANDIDATE_UNKNOWN:
      return "unknown";
    case OBJECT_CANDIDATE_WEIGHT_SIZED:
      return "weight_sized";
    case OBJECT_CANDIDATE_TALL_OBSTACLE:
      return "tall_obstacle";
  }
  return "unknown";
}

static bool isValidObjectDistance(uint16_t distanceMm) {
  return distanceMm >= OBJECT_TOF_VALID_MIN_MM &&
         distanceMm <= OBJECT_TOF_VALID_MAX_MM;
}

static bool isObjectLowNear(ObjectTofId id) {
  const ObjectSensorState &sensor = objectSensors[id];
  return sensor.valid &&
         sensor.distanceMm >= OBJECT_CANDIDATE_MIN_MM &&
         sensor.distanceMm <= OBJECT_CANDIDATE_MAX_MM;
}

static bool isObjectUpperClearForLow(ObjectTofId upperId, uint16_t lowDistanceMm) {
  // If the upper sensor is absent/weak, the system currently treats the low
  // return as potentially weight-sized. This is useful for bring-up but is a
  // known classifier limitation: invalid upper data is not proof of material.
  const ObjectSensorState &upper = objectSensors[upperId];
  if (!upper.valid) {
    return true;
  }
  if (upper.signalMcps < OBJECT_UPPER_STRONG_SIGNAL_MCPS) {
    return true;
  }
  return upper.distanceMm > lowDistanceMm + OBJECT_UPPER_CLEAR_DELTA_MM;
}

static bool isObjectUpperNear(ObjectTofId upperId, uint16_t lowDistanceMm) {
  const ObjectSensorState &upper = objectSensors[upperId];
  if (!upper.valid) {
    return false;
  }
  if (upper.signalMcps < OBJECT_UPPER_STRONG_SIGNAL_MCPS) {
    return false;
  }
  return upper.distanceMm <= lowDistanceMm + OBJECT_UPPER_CLEAR_DELTA_MM;
}

static void invalidateObjectTargetEstimate(const char* reason) {
  objectTargetEstimate.valid = false;
  objectTargetEstimate.robotXmm = 0.0;
  objectTargetEstimate.robotYmm = 0.0;
  objectTargetEstimate.worldX = 0.0;
  objectTargetEstimate.worldY = 0.0;
  objectTargetEstimate.rangeMm = OBJECT_NO_READING_MM;
  objectTargetEstimate.sourceMask = 0;
  objectTargetEstimate.reason = reason;
  objectTargetEstimate.lastUpdateMs = millis();
}

static void transformObjectTargetToWorld(float robotXmm, float robotYmm,
                                         float &worldX, float &worldY) {
  // Converts a robot-body-frame point (millimetres) into odometry/world metres.
  // Uses the same +X forward, +Y left convention as the local planner.
  float headingRad = navigationHeadingDeg() * DEG_TO_RAD;
  float localX = robotXmm / 1000.0;
  float localY = robotYmm / 1000.0;
  worldX = robotX + localX * cosf(headingRad) - localY * sinf(headingRad);
  worldY = robotY + localX * sinf(headingRad) + localY * cosf(headingRad);
}

static bool accumulateObjectLowEndpoint(ObjectTofId id, uint8_t sourceBit,
                                        float &sumXmm, float &sumYmm,
                                        uint8_t &sourceCount,
                                        uint8_t &sourceMask,
                                        uint16_t &rangeMm) {
  // Projects one low sensor's range along its mounted yaw angle to estimate
  // where the detected object sits in the robot frame.
  if (!isObjectLowNear(id)) {
    return false;
  }

  const ObjectSensorGeometry &geometry = OBJECT_SENSOR_GEOMETRY[id];
  float yawRad = geometry.yawDeg * DEG_TO_RAD;
  float distanceMm = (float)objectSensors[id].distanceMm;
  float endpointXmm = geometry.xMm + distanceMm * cosf(yawRad);
  float endpointYmm = geometry.yMm + distanceMm * sinf(yawRad);

  sumXmm += endpointXmm;
  sumYmm += endpointYmm;
  sourceCount++;
  sourceMask |= sourceBit;
  rangeMm = min(rangeMm, objectSensors[id].distanceMm);
  return true;
}

static void updateObjectTargetEstimateFromCandidate(const char* reason) {
  // Only confirmed weight-sized candidates produce a hunt target. The target
  // is biased forward by OBJECT_PICKUP_OVERSHOOT_MM so a hunt goal carries the
  // intake through the estimated object point instead of stopping short.
  if (objectCandidate.kind != OBJECT_CANDIDATE_WEIGHT_SIZED ||
      !objectCandidate.confirmed) {
    invalidateObjectTargetEstimate(reason);
    return;
  }

  float sumXmm = 0.0;
  float sumYmm = 0.0;
  uint8_t sourceCount = 0;
  uint8_t sourceMask = 0;
  uint16_t rangeMm = OBJECT_NO_READING_MM;

  accumulateObjectLowEndpoint(OBJECT_LEFT_LOW, OBJECT_TARGET_SOURCE_LEFT_LOW,
                              sumXmm, sumYmm, sourceCount, sourceMask, rangeMm);
  accumulateObjectLowEndpoint(OBJECT_RIGHT_LOW, OBJECT_TARGET_SOURCE_RIGHT_LOW,
                              sumXmm, sumYmm, sourceCount, sourceMask, rangeMm);

  if (sourceCount == 0) {
    invalidateObjectTargetEstimate("weight_no_low_target");
    return;
  }

  float targetXmm = sumXmm / sourceCount;
  float targetYmm = sumYmm / sourceCount;
  targetXmm += OBJECT_PICKUP_OVERSHOOT_MM;

  objectTargetEstimate.valid = true;
  objectTargetEstimate.robotXmm = targetXmm;
  objectTargetEstimate.robotYmm = targetYmm;
  transformObjectTargetToWorld(objectTargetEstimate.robotXmm,
                               objectTargetEstimate.robotYmm,
                               objectTargetEstimate.worldX,
                               objectTargetEstimate.worldY);
  objectTargetEstimate.rangeMm = rangeMm;
  objectTargetEstimate.sourceMask = sourceMask;
  objectTargetEstimate.reason = reason;
  objectTargetEstimate.lastUpdateMs = millis();
}

static void setCandidate(ObjectCandidateKind kind, const char* reason,
                         bool confirmed, int directionHint, uint16_t rangeMm) {
  objectCandidate.kind = kind;
  objectCandidate.reason = reason;
  objectCandidate.confirmed = confirmed;
  objectCandidate.directionHint = directionHint;
  objectCandidate.rangeMm = rangeMm;
  objectCandidate.lastUpdateMs = millis();
  updateObjectTargetEstimateFromCandidate(reason);
}

static void resetObjectSensorStates(const char* reason) {
  for (int i = 0; i < OBJECT_TOF_COUNT; i++) {
    ObjectSensorState &sensor = objectSensors[i];
    sensor.distanceMm = OBJECT_NO_READING_MM;
    sensor.valid = false;
    sensor.stale = false;
    sensor.connected = false;
    sensor.lastReadMs = 0;
    sensor.rangeStatus = SENSOR_RANGE_STATUS_UNKNOWN;
    sensor.signalMcps = 0.0;
    sensor.ambientMcps = 0.0;
  }

  objectCandidate.confirmCount = 0;
  setCandidate(OBJECT_CANDIDATE_DISABLED, reason, false, 0, OBJECT_NO_READING_MM);
}

static void updateObjectCandidate() {
  // Classification rule of thumb:
  //   low near + upper clear/weak -> weight-sized candidate
  //   low near + upper near/strong -> tall obstacle
  //   low near + unclear upper evidence -> unknown
  // The result is advisory and should not be used as a safety clearance.
  if (!OBJECT_TOF_ENABLED) {
    objectCandidate.confirmCount = 0;
    setCandidate(OBJECT_CANDIDATE_DISABLED, "object_tof_disabled", false, 0, OBJECT_NO_READING_MM);
    return;
  }

  const bool leftLowNear = isObjectLowNear(OBJECT_LEFT_LOW);
  const bool rightLowNear = isObjectLowNear(OBJECT_RIGHT_LOW);

  if (!leftLowNear && !rightLowNear) {
    objectCandidate.confirmCount = 0;
    setCandidate(OBJECT_CANDIDATE_NONE, "no_low_near_return", false, 0, OBJECT_NO_READING_MM);
    return;
  }

  const bool leftWeightLike =
    leftLowNear &&
    isObjectUpperClearForLow(OBJECT_LEFT_UPPER, objectSensors[OBJECT_LEFT_LOW].distanceMm);
  const bool rightWeightLike =
    rightLowNear &&
    isObjectUpperClearForLow(OBJECT_RIGHT_UPPER, objectSensors[OBJECT_RIGHT_LOW].distanceMm);

  const bool leftTallLike =
    leftLowNear &&
    isObjectUpperNear(OBJECT_LEFT_UPPER, objectSensors[OBJECT_LEFT_LOW].distanceMm);
  const bool rightTallLike =
    rightLowNear &&
    isObjectUpperNear(OBJECT_RIGHT_UPPER, objectSensors[OBJECT_RIGHT_LOW].distanceMm);

  if (leftTallLike || rightTallLike) {
    objectCandidate.confirmCount = 0;
    uint16_t rangeMm = OBJECT_NO_READING_MM;
    if (leftTallLike) {
      rangeMm = min(rangeMm, objectSensors[OBJECT_LEFT_LOW].distanceMm);
    }
    if (rightTallLike) {
      rangeMm = min(rangeMm, objectSensors[OBJECT_RIGHT_LOW].distanceMm);
    }
    setCandidate(OBJECT_CANDIDATE_TALL_OBSTACLE, "low_and_upper_near", false, 0, rangeMm);
    return;
  }

  if (!leftWeightLike && !rightWeightLike) {
    objectCandidate.confirmCount = 0;
    setCandidate(OBJECT_CANDIDATE_UNKNOWN, "low_upper_geometry_unclear", false, 0, OBJECT_NO_READING_MM);
    return;
  }

  uint16_t rangeMm = OBJECT_NO_READING_MM;
  int directionHint = 0;
  if (leftWeightLike) {
    rangeMm = min(rangeMm, objectSensors[OBJECT_LEFT_LOW].distanceMm);
    directionHint -= 1;
  }
  if (rightWeightLike) {
    rangeMm = min(rangeMm, objectSensors[OBJECT_RIGHT_LOW].distanceMm);
    directionHint += 1;
  }

  if (objectCandidate.kind == OBJECT_CANDIDATE_WEIGHT_SIZED &&
      objectCandidate.confirmCount < OBJECT_CANDIDATE_CONFIRM_READS) {
    objectCandidate.confirmCount++;
  } else if (objectCandidate.kind != OBJECT_CANDIDATE_WEIGHT_SIZED) {
    objectCandidate.confirmCount = 1;
  }

  const bool confirmed = objectCandidate.confirmCount >= OBJECT_CANDIDATE_CONFIRM_READS;
  setCandidate(OBJECT_CANDIDATE_WEIGHT_SIZED, "low_near_upper_clear", confirmed,
               directionHint, rangeMm);
}

static void markObjectSensorTimeout(ObjectTofId id) {
  ObjectSensorState &sensor = objectSensors[id];
  sensor.distanceMm = OBJECT_NO_READING_MM;
  sensor.valid = false;
  sensor.stale = false;
  sensor.timeoutCount++;
  sensor.rangeStatus = SENSOR_RANGE_STATUS_UNKNOWN;
  sensor.signalMcps = 0.0;
  sensor.ambientMcps = 0.0;
  sendBluetoothEvent("object_tof_timeout", sensor.name);
}

static void setObjectSensorReading(ObjectTofId id, uint16_t distanceMm,
                                   bool valid, uint8_t rangeStatus, float signalMcps,
                                   float ambientMcps) {
  ObjectSensorState &sensor = objectSensors[id];
  sensor.distanceMm = distanceMm;
  sensor.valid = valid;
  sensor.stale = false;
  sensor.rangeStatus = rangeStatus;
  sensor.signalMcps = signalMcps;
  sensor.ambientMcps = ambientMcps;

  if (valid) {
    sensor.lastReadMs = millis();
  } else {
    sensor.invalidCount++;
  }
}

static void updateObjectStaleFlags() {
  unsigned long now = millis();
  for (int i = 0; i < OBJECT_TOF_COUNT; i++) {
    ObjectSensorState &sensor = objectSensors[i];
    if (!sensor.connected || !sensor.valid) {
      continue;
    }
    if (now - sensor.lastReadMs <= OBJECT_TOF_STALE_TIMEOUT_MS) {
      continue;
    }
    sensor.valid = false;
    sensor.stale = true;
    sendBluetoothEvent("object_tof_stale", sensor.name);
  }
}

static void connectObjectSensor(ObjectTofId id) {
  // Object ToFs are optional bring-up sensors. A missing sensor degrades the
  // object subsystem and emits telemetry, but it does not block navigation
  // startup because these sensors are not part of the motion safety kernel.
  ObjectSensorState &state = objectSensors[id];
  VL53L1X* sensor = objectTof(id);
  if (sensor == NULL) {
    return;
  }

  Serial.print("Starting ");
  Serial.print(state.name);
  Serial.println(" VL53L1X...");

  io.digitalWrite(objectXshutPin(id), HIGH);
  delay(100);

  sensor->setTimeout(100);
  if (!sensor->init()) {
    state.connected = false;
    state.valid = false;
    state.timeoutCount++;
    Serial.print(state.name);
    Serial.println(" VL53L1X not detected. Object sensing degraded.");
    sendBluetoothEvent("object_tof_missing", state.name);
    io.digitalWrite(objectXshutPin(id), LOW);
    return;
  }

  sensor->setAddress(objectI2cAddress(id));
  sensor->setDistanceMode(VL53L1X::Short);
  sensor->setROISize(OBJECT_TOF_ROI_WIDTH, OBJECT_TOF_ROI_HEIGHT);
  sensor->setROICenter(OBJECT_TOF_ROI_CENTER_SPAD);
  sensor->setMeasurementTimingBudget(OBJECT_TOF_TIMING_BUDGET_US);
  sensor->startContinuous(OBJECT_TOF_SAMPLE_PERIOD_MS);

  state.connected = true;
  state.valid = false;
  state.stale = false;
  state.distanceMm = OBJECT_NO_READING_MM;
  state.lastReadMs = 0;
  state.rangeStatus = SENSOR_RANGE_STATUS_UNKNOWN;
  state.signalMcps = 0.0;
  state.ambientMcps = 0.0;

  Serial.print(state.name);
  Serial.print(" VL53L1X connected at 0x");
  Serial.println(objectI2cAddress(id), HEX);
}

void prepareObjectTOFPinsForStartup() {
  // Keeps object sensors quiet on the shared I2C bus until connectObjectSensor
  // assigns addresses, or permanently if the subsystem is disabled.
  if (!OBJECT_TOF_ENABLED && !OBJECT_TOF_HOLD_DISABLED_IN_RESET) {
    return;
  }

  for (int i = 0; i < OBJECT_TOF_COUNT; i++) {
    byte pin = objectXshutPin((ObjectTofId)i);
    io.pinMode(pin, OUTPUT);
    io.digitalWrite(pin, LOW);
  }
}

void connectObjectTOFSensors() {
  // Called after the SX1509 expander is connected. Each VL53L1X is started
  // independently so one missing object sensor does not prevent others from
  // reporting.
  resetObjectSensorStates("object_tof_disabled");

  if (!OBJECT_TOF_ENABLED) {
    Serial.println("Object VL53L1X ToFs disabled by configuration.");
    return;
  }

  for (int i = 0; i < OBJECT_TOF_COUNT; i++) {
    connectObjectSensor((ObjectTofId)i);
  }

  objectCandidate.confirmCount = 0;
  setCandidate(OBJECT_CANDIDATE_NONE, "object_tof_ready", false, 0, OBJECT_NO_READING_MM);
}

void updateObjectTOFSensors() {
  // Nonblocking round-robin update. Each call checks at most one object sensor
  // for dataReady(); if no sample is ready, the function returns quickly.
  if (!OBJECT_TOF_ENABLED) {
    return;
  }

  updateObjectStaleFlags();

  ObjectTofId id = (ObjectTofId)nextObjectSensorToUpdate;
  nextObjectSensorToUpdate = (nextObjectSensorToUpdate + 1) % OBJECT_TOF_COUNT;

  ObjectSensorState &state = objectSensors[id];
  VL53L1X* sensor = objectTof(id);
  if (sensor == NULL || !state.connected) {
    updateObjectCandidate();
    return;
  }

  if (!sensor->dataReady()) {
    updateObjectCandidate();
    return;
  }

  uint16_t distanceMm = sensor->read(false);
  if (sensor->timeoutOccurred()) {
    markObjectSensorTimeout(id);
    updateObjectCandidate();
    return;
  }

  VL53L1X::RangingData &data = sensor->ranging_data;
  bool valid = data.range_status == VL53L1X::RangeValid &&
               isValidObjectDistance(distanceMm);
  setObjectSensorReading(id, distanceMm, valid, data.range_status,
                         data.peak_signal_count_rate_MCPS,
                         data.ambient_count_rate_MCPS);
  updateObjectCandidate();
}

void refreshObjectTargetEstimate() {
  // One bounded readiness sweep. updateObjectTOFSensors() already checks
  // dataReady() and uses read(false), so no measurement wait or delay is needed.
  for (int i = 0; i < OBJECT_TOF_COUNT; i++) {
    updateObjectTOFSensors();
  }
}

void printObjectTelemetry() {
  // Stationary diagnostic used by TEST OBJECT. It refreshes one object-sensor
  // pass and prints candidate, target, raw distance, validity, range status,
  // signal rate, and ambient rate.
  updateObjectTOFSensors();

  Serial2.print("object_summary,enabled=");
  Serial2.print(OBJECT_TOF_ENABLED ? 1 : 0);
  Serial2.print(",candidate=");
  Serial2.print(objectCandidateKindName(objectCandidate.kind));
  Serial2.print(",confirmed=");
  Serial2.print(objectCandidate.confirmed ? 1 : 0);
  Serial2.print(",direction_hint=");
  Serial2.print(objectCandidate.directionHint);
  Serial2.print(",range_mm=");
  Serial2.print(objectCandidate.rangeMm);
  Serial2.print(",reason=");
  Serial2.println(objectCandidate.reason);

  Serial2.print("object_target,valid=");
  Serial2.print(objectTargetEstimate.valid ? 1 : 0);
  Serial2.print(",fresh=");
  Serial2.print(isObjectTargetFresh() ? 1 : 0);
  Serial2.print(",robot_x_mm=");
  Serial2.print(objectTargetEstimate.robotXmm, 1);
  Serial2.print(",robot_y_mm=");
  Serial2.print(objectTargetEstimate.robotYmm, 1);
  Serial2.print(",world_x_m=");
  Serial2.print(objectTargetEstimate.worldX, 3);
  Serial2.print(",world_y_m=");
  Serial2.print(objectTargetEstimate.worldY, 3);
  Serial2.print(",range_mm=");
  Serial2.print(objectTargetEstimate.rangeMm);
  Serial2.print(",sources=");
  Serial2.print(objectTargetEstimate.sourceMask);
  Serial2.print(",reason=");
  Serial2.println(objectTargetEstimate.reason);

  Serial2.println("object_idx,name,role,xshut,address,distance_mm,valid,stale,connected,age_ms,timeouts,invalids,range_status,signal_mcps,ambient_mcps");
  unsigned long now = millis();
  for (int i = 0; i < OBJECT_TOF_COUNT; i++) {
    ObjectTofId id = (ObjectTofId)i;
    ObjectSensorState &sensor = objectSensors[id];
    Serial2.print(i);
    Serial2.print(",");
    Serial2.print(sensor.name);
    Serial2.print(",");
    Serial2.print(objectSensorRoleName(sensor.role));
    Serial2.print(",");
    Serial2.print(objectXshutPin(id));
    Serial2.print(",0x");
    Serial2.print(objectI2cAddress(id), HEX);
    Serial2.print(",");
    Serial2.print(sensor.distanceMm);
    Serial2.print(",");
    Serial2.print(sensor.valid ? 1 : 0);
    Serial2.print(",");
    Serial2.print(sensor.stale ? 1 : 0);
    Serial2.print(",");
    Serial2.print(sensor.connected ? 1 : 0);
    Serial2.print(",");
    Serial2.print(sensor.lastReadMs == 0 ? SENSOR_AGE_NOT_REPORTED_MS : now - sensor.lastReadMs);
    Serial2.print(",");
    Serial2.print(sensor.timeoutCount);
    Serial2.print(",");
    Serial2.print(sensor.invalidCount);
    Serial2.print(",");
    Serial2.print(sensor.rangeStatus);
    Serial2.print(",");
    Serial2.print(sensor.signalMcps, 6);
    Serial2.print(",");
    Serial2.println(sensor.ambientMcps, 6);
  }
}

bool isObjectTargetFresh() {
  // A target must be both valid and recent. The robot should not hunt an old
  // pose estimate after it has moved or the object has disappeared.
  return objectTargetEstimate.valid &&
         millis() - objectTargetEstimate.lastUpdateMs <= OBJECT_TARGET_STALE_TIMEOUT_MS;
}
