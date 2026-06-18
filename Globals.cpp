#include "Robot.h"

Servo leftMotor;
Servo rightMotor;

volatile long leftRawCount  = 0;
volatile long rightRawCount = 0;

long lastLeftCount  = 0;
long lastRightCount = 0;

long lastOdomLeftCount  = 0;
long lastOdomRightCount = 0;

float robotX = 0.0;
float robotY = 0.0;
float robotTheta = 0.0;

float baseTargetSpeed = DEFAULT_BASE_TARGET_SPEED;  // encoder ticks per second

float Kp = 0.025;
float Ki = 0.004;
float Kd = 0.000;

float leftIntegral  = 0.0;
float rightIntegral = 0.0;

float lastLeftError  = 0.0;
float lastRightError = 0.0;

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

float yawOffset = 0.0;
float K_heading = 10.0;

SX1509 io;

VL53L1X frontTOF;
VL53L0X leftTOF;
VL53L0X rightTOF;

RangeSensorState rangeSensors[RANGE_SENSOR_COUNT] = {
  {"front", 0, 9999, false, false, false, 0, 0, 0},
  {"left", 45, 9999, false, false, false, 0, 0, 0},
  {"right", -45, 9999, false, false, false, 0, 0, 0}
};

bool frontBlocked = false;
uint16_t frontDistance = 9999;
uint16_t leftDistance  = 9999;
uint16_t rightDistance = 9999;

bool frontTofValid = false;
bool leftTofValid = false;
bool rightTofValid = false;

unsigned long lastFrontTofReadMs = 0;
unsigned long lastLeftTofReadMs = 0;
unsigned long lastRightTofReadMs = 0;

int frontBlockCounter = 0;
int frontClearCounter = 0;
unsigned long lastFrontInvalidPrintMs = 0;

bool driveStuck = false;
bool wheelMismatchStuck = false;
bool turnStuck = false;

unsigned long driveStuckStartMs = 0;
unsigned long wheelMismatchStartMs = 0;
unsigned long turnCheckStartMs = 0;
float turnCheckStartYaw = 0.0;

int stuckRecoveryCount = 0;
bool stoppedSafely = false;
bool inRecovery = false;
bool returnHomeRequested = false;
bool robotRunEnabled = false;
bool bluetoothOutputEnabled = false;

RobotSerialClass robotSerial;

RobotState currentState = INIT;
RobotState previousState = INIT;

int currentWaypointIndex = 0;
bool endMatchPrinted = false;

float desiredForwardSpeed = 0.0;
float desiredTurnSpeed = 0.0;

int lastLeftMotorUs = STOP_US;
int lastRightMotorUs = STOP_US;

Waypoint path[] = {
  {0.50, 0.00, "PAUSE"},
  {0.50, 0.50, "PAUSE"},
  {0.00, 0.50, "PAUSE"},
  {0.00, 0.00, "HOME"}
};

extern const int NUM_POINTS = sizeof(path) / sizeof(path[0]);

unsigned long lastTime = 0;
