#include "Robot.h"

// =====================================================
// IMU
// =====================================================
void connectIMU() {
  while (true) {
    Serial.println("Trying to find BNO055 IMU...");

    if (bno.begin()) {
      Serial.println("BNO055 connected.");
      break;
    }

    Serial.println("BNO055 not detected. Retrying...");
    delay(1000);
  }

  delay(1000);
  bno.setExtCrystalUse(true);
  delay(500);
}

void zeroYaw() {
  sensors_event_t event;
  bno.getEvent(&event);
  yawOffset = event.orientation.x;

  Serial.print("Yaw zeroed at raw heading: ");
  Serial.println(yawOffset);
}

float readYawDeg() {
  sensors_event_t event;
  bno.getEvent(&event);

  float rawYaw = event.orientation.x;
  return wrapAngle(rawYaw - yawOffset);
}
