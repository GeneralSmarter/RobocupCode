#include "Robot.h"

// =====================================================
// Motor and PID helpers
// =====================================================
void stopMotors() {
  setMotionCommand(0.0, 0.0);
  writeMotorUS(STOP_US, STOP_US);
}

void writeMotorUS(int leftUs, int rightUs) {
  leftUs = constrain(leftUs, MIN_US, MAX_US);
  rightUs = constrain(rightUs, MIN_US, MAX_US);

  lastLeftMotorUs = leftUs;
  lastRightMotorUs = rightUs;

  leftMotor.writeMicroseconds(leftUs);
  rightMotor.writeMicroseconds(rightUs);
}

int updatePID(float target, float actual, float &integral, float &lastError, float dt, int baseCommand) {
  float error = target - actual;

  integral += error * dt;
  integral = constrain(integral, -3000.0, 3000.0);

  float derivative = (error - lastError) / dt;
  float correction = Kp * error + Ki * integral + Kd * derivative;

  lastError = error;

  int command = baseCommand + correction;

  command = constrain(command, baseCommand - 150, baseCommand + 150);
  command = constrain(command, MIN_US, MAX_US);

  return command;
}
