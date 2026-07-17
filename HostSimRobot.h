#ifndef HOST_SIM_ROBOT_H
#define HOST_SIM_ROBOT_H

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <type_traits>

using byte = uint8_t;

unsigned long millis();
unsigned long micros();

template <typename A, typename B, typename C>
constexpr auto constrain(A value, B low, C high) {
  using T = std::common_type_t<A, B, C>;
  const T converted = static_cast<T>(value);
  return converted < static_cast<T>(low) ? static_cast<T>(low)
    : (converted > static_cast<T>(high) ? static_cast<T>(high) : converted);
}

template <typename A, typename B>
constexpr auto min(A left, B right) {
  using T = std::common_type_t<A, B>;
  return static_cast<T>(left) < static_cast<T>(right)
    ? static_cast<T>(left) : static_cast<T>(right);
}

template <typename A, typename B>
constexpr auto max(A left, B right) {
  using T = std::common_type_t<A, B>;
  return static_cast<T>(left) > static_cast<T>(right)
    ? static_cast<T>(left) : static_cast<T>(right);
}

#include "RobotTypes.h"
#include "RobotConfig.h"
#include "TurnConvention.h"
#include "MotionSafety.h"

class HostSimSerial {
public:
  template <typename... Args> size_t print(Args...) { return 0; }
  template <typename... Args> size_t println(Args...) { return 0; }
  size_t println() { return 0; }
};

extern HostSimSerial hostSimSerial;
#define Serial hostSimSerial

extern float robotX;
extern float robotY;
extern float robotTheta;
extern float baseTargetSpeed;
extern float desiredForwardSpeed;
extern float desiredTurnSpeed;
extern bool frontBlocked;
extern uint16_t frontDistance;
extern uint16_t leftDistance;
extern uint16_t rightDistance;
extern bool driveStuck;
extern bool wheelMismatchStuck;
extern bool turnStuck;
extern bool escapeBacktrackEnabled;
extern bool motorStopRequested;
extern bool robotRunEnabled;
extern RobotState currentState;
extern NavigationGoal navigationGoal;
extern PlannerTelemetry plannerTelemetry;
extern MotionAuthority motionAuthority;
extern MotionAuthority motionCommandAuthority;
extern RangeSensorState rangeSensors[RANGE_SENSOR_COUNT];
extern ObjectCandidateState objectCandidate;
extern ObjectTargetEstimate objectTargetEstimate;
extern unsigned long lastSensorUpdateMs;
extern unsigned long lastOdometryUpdateMs;
extern unsigned long lastPlannerUpdateMs;
extern unsigned long lastMotorControlUpdateMs;

float navigationHeadingDeg();
float wrapAngle(float angle);
void resetEncodersAndPID();
void resetTurnStuckCheck(float startYaw);
void updateStuckTurning(float currentYaw);
void updateOdometry();
void updateTOFSensors();
void updateMotorController();
void stopMotors();
void sendBluetoothTelemetry();
void sendBluetoothEvent(const char* eventName, const char* eventDetail);
void setRobotState(RobotState newState);
void setMotionCommand(float forwardSpeed, float turnSpeed);
bool setAuthorizedMotionCommand(MotionAuthority authority,
                                float forwardSpeed,
                                float turnSpeed);
bool isMotorCommandLeaseArmed();
MotionSafetyReason lastMotionSafetyReason();
const char* motionSafetyReasonName(MotionSafetyReason reason);
void recordMainLoopPhaseDuration(const char* phase, unsigned long startedUs);
bool isRangeSensorValid(RangeSensorId id);
bool isRangeSensorBlocked(RangeSensorId id);
bool hasTrustedRearCoverage();
uint16_t getRangeSensorDistance(RangeSensorId id);
bool isTofCloseReadingRevalidating();
float getFanSweepClearanceMm(RangeSensorId id);
bool getDiagonalClearanceWarning(RangeSensorId &sensorId, float &clearanceMm);

#endif
