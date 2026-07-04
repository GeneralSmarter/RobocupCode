#ifndef ROBOT_H
#define ROBOT_H

#include <Arduino.h>
#include <Servo.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <VL53L0X.h>
#include <VL53L1X.h>
#include <SparkFunSX1509.h>

// =====================================================
// Debug
// =====================================================
const bool DEBUG_DRIVE = true;
const bool DEBUG_TURN  = false;

const char ROBOT_BUILD_LABEL[] = "V7-local-planner";

// =====================================================
// Bluetooth serial debug link
// =====================================================
const unsigned long BLUETOOTH_BAUD = 115200;

extern bool bluetoothOutputEnabled;
extern bool robotRunEnabled;

class RobotSerialClass {
public:
  void begin(unsigned long baud) {
    ::Serial.begin(baud);
  }

  template <typename... Args>
  size_t print(Args... args) {
    size_t written = ::Serial.print(args...);
    if (bluetoothOutputEnabled) {
      Serial2.print(args...);
    }
    return written;
  }

  template <typename... Args>
  size_t println(Args... args) {
    size_t written = ::Serial.println(args...);
    if (bluetoothOutputEnabled) {
      Serial2.println(args...);
    }
    return written;
  }

  size_t println() {
    size_t written = ::Serial.println();
    if (bluetoothOutputEnabled) {
      Serial2.println();
    }
    return written;
  }

  void flush() {
    ::Serial.flush();
    if (bluetoothOutputEnabled) {
      Serial2.flush();
    }
  }
};

extern RobotSerialClass robotSerial;
#define Serial robotSerial

// =====================================================
// Motors
// =====================================================
extern Servo leftMotor;
extern Servo rightMotor;

const int LEFT_MOTOR_PIN  = 0;
const int RIGHT_MOTOR_PIN = 1;

const int STOP_US = 1500;
const int MIN_US  = 1000;
const int MAX_US  = 1993;

const int LEFT_BASE_US  = 1935;
const int RIGHT_BASE_US = 1870;

const int LEFT_REVERSE_US  = 1130;
const int RIGHT_REVERSE_US = 1190;

extern int leftForwardBaseUs;
extern int rightForwardBaseUs;

// =====================================================
// Encoders
// =====================================================
const int LEFT_ENC_A  = 2;
const int LEFT_ENC_B  = 3;
const int RIGHT_ENC_A = 4;
const int RIGHT_ENC_B = 5;

const int LEFT_ENCODER_SIGN  = -1;
const int RIGHT_ENCODER_SIGN = 1;

extern volatile long leftRawCount;
extern volatile long rightRawCount;

extern long lastLeftCount;
extern long lastRightCount;

extern long lastOdomLeftCount;
extern long lastOdomRightCount;

const float TICKS_PER_METRE = 9125.0;

// The distance between the left and right track centre lines must be measured
// on the robot before narrow-gap planning is accepted.  The provisional value
// is deliberately kept in one visible configuration value so that the planner
// model and its test logs cannot silently diverge.
const float EFFECTIVE_TRACK_WIDTH_M = 0.224;

// =====================================================
// Pose
// =====================================================
extern float robotX;
extern float robotY;
// Navigation/world-frame heading. Robot coordinates are +X forward, +Y left,
// so positive heading is counter-clockwise. The BNO055's raw yaw convention is
// the opposite on this installation; use navigationHeadingDeg() for mapping,
// odometry, trajectory geometry, and direct turn control.
extern float robotTheta;

// =====================================================
// Wheel speed PID
// =====================================================
extern float baseTargetSpeed;

const float DEFAULT_BASE_TARGET_SPEED = 2600.0;

extern float Kp;
extern float Ki;
extern float Kd;

extern float leftIntegral;
extern float rightIntegral;

extern float lastLeftError;
extern float lastRightError;

// =====================================================
// IMU
// =====================================================
extern Adafruit_BNO055 bno;

extern float yawOffset;

// =====================================================
// TOF sensors
// =====================================================
extern SX1509 io;

extern VL53L0X rightOuterTOF;
extern VL53L0X rightInnerTOF;
extern VL53L0X leftInnerTOF;
extern VL53L0X leftOuterTOF;
extern VL53L1X objectLeftLowTOF;
extern VL53L1X objectLeftUpperTOF;
extern VL53L1X objectRightLowTOF;
extern VL53L1X objectRightUpperTOF;

const uint16_t RANGE_NO_READING_MM = 9999;
const uint16_t OBJECT_NO_READING_MM = RANGE_NO_READING_MM;
const unsigned long SENSOR_AGE_NOT_REPORTED_MS = 999999;
const uint8_t SENSOR_RANGE_STATUS_UNKNOWN = 255;
const byte INVALID_XSHUT_PIN = 255;

const byte SX1509_ADDRESS = 0x3F;

// High fan XSHUT mapping, physically numbered right-to-left on the robot.
const byte RIGHT_OUTER_XSHUT = 0;  // -60 deg, VL53L0X
const byte RIGHT_INNER_XSHUT = 1;  // -20 deg, VL53L0X
const byte LEFT_INNER_XSHUT  = 2;  // +20 deg, VL53L0X
const byte LEFT_OUTER_XSHUT  = 3;  // +60 deg, VL53L0X

const uint8_t RIGHT_OUTER_ADDRESS = 0x30;
const uint8_t RIGHT_INNER_ADDRESS = 0x31;
const uint8_t LEFT_INNER_ADDRESS  = 0x32;
const uint8_t LEFT_OUTER_ADDRESS  = 0x33;

// Object/weight ToF scaffold. The physical VL53L1X layout is now measured and
// enabled for bring-up. When disabled, keep the reserved XSHUT pins low so
// unconfigured object sensors stay off the shared I2C bus.
const bool OBJECT_TOF_ENABLED = true;
const bool OBJECT_TOF_HOLD_DISABLED_IN_RESET = true;
const byte OBJECT_LEFT_UPPER_XSHUT  = 5;
const byte OBJECT_RIGHT_UPPER_XSHUT = 4;
const byte OBJECT_LEFT_LOW_XSHUT    = 7;
const byte OBJECT_RIGHT_LOW_XSHUT   = 6;
const uint8_t OBJECT_LEFT_LOW_ADDRESS    = 0x34;
const uint8_t OBJECT_LEFT_UPPER_ADDRESS  = 0x35;
const uint8_t OBJECT_RIGHT_LOW_ADDRESS   = 0x36;
const uint8_t OBJECT_RIGHT_UPPER_ADDRESS = 0x37;
const uint32_t OBJECT_TOF_TIMING_BUDGET_US = 50000;
const unsigned long OBJECT_TOF_SAMPLE_PERIOD_MS = 60;
const int OBJECT_TOF_VALID_MIN_MM = 40;
const int OBJECT_TOF_VALID_MAX_MM = 4000;
const unsigned long OBJECT_TOF_STALE_TIMEOUT_MS = 750;
const uint16_t OBJECT_CANDIDATE_MIN_MM = 60;
const uint16_t OBJECT_CANDIDATE_MAX_MM = 1500;
const uint16_t OBJECT_UPPER_CLEAR_DELTA_MM = 80;
const float OBJECT_UPPER_STRONG_SIGNAL_MCPS = 4.0;
const uint8_t OBJECT_CANDIDATE_CONFIRM_READS = 3;
const unsigned long OBJECT_TARGET_STALE_TIMEOUT_MS = 1000;
const uint8_t OBJECT_TARGET_SOURCE_LEFT_LOW = 0x01;
const uint8_t OBJECT_TARGET_SOURCE_RIGHT_LOW = 0x02;
const float OBJECT_PICKUP_OVERSHOOT_MM = 150.0;
const float WEIGHT_SEARCH_SWEEP_DEG = 30.0;
const float WEIGHT_SEARCH_CONFIRM_TURN_MIN_DEG = 5.0;
const float WEIGHT_SEARCH_CONFIRM_TURN_MAX_DEG = 35.0;
const float WEIGHT_SEARCH_STANDOFF_M = 0.25;
const unsigned long WEIGHT_SEARCH_SETTLE_MS = 200;
const unsigned long WEIGHT_SEARCH_CONFIRM_MS = 300;
const unsigned long WEIGHT_SEARCH_HUNT_TIMEOUT_MS = 5000;
const unsigned long WEIGHT_INTERRUPT_COOLDOWN_MS = 1000;
const float WEIGHT_SEARCH_MAX_ROUTE_DEVIATION_M = 0.85;

const int FRONT_STOP_DISTANCE_MM  = 180;
const int FRONT_CLEAR_DISTANCE_MM = 230;

const int FRONT_VALID_MIN_MM = 20;
const int FRONT_VALID_MAX_MM = 8191;
const unsigned long TOF_STALE_TIMEOUT_MS = 750;

const int FRONT_BLOCK_CONFIRM_READS = 2;
const int FRONT_CLEAR_CONFIRM_READS = 3;
const unsigned long FRONT_CLEAR_SETTLE_TIMEOUT_MS = 500;

const uint16_t FAKE_REAR_TOF_DISTANCE_MM = 4000;
const int FAKE_REAR_TOF_VALID_MIN_MM = 20;
const int FAKE_REAR_TOF_VALID_MAX_MM = 4000;
const int FAKE_REAR_TOF_STOP_DISTANCE_MM = 180;
const int FAKE_REAR_TOF_CLEAR_DISTANCE_MM = 230;

enum RangeSensorId {
  RANGE_RIGHT_OUTER,
  RANGE_RIGHT_INNER,
  RANGE_LEFT_INNER,
  RANGE_LEFT_OUTER,
  RANGE_FRONT,
  RANGE_RIGHT,
  RANGE_LEFT,
  RANGE_FAKE_REAR,
  RANGE_SENSOR_COUNT
};

// Robot-centred geometry in millimetres. The origin is the midpoint between
// the drive wheels: +X forward, +Y left. Keep future physical measurements in
// this one block so clearance logic stays relative to the chassis.
struct RobotFootprintGeometry {
  float frontExtentMm;
  float rearExtentMm;
  float leftExtentMm;
  float rightExtentMm;
};

struct FanSensorGeometry {
  float xMm;
  float yMm;
  float angleDeg;
};

const RobotFootprintGeometry ROBOT_FOOTPRINT_GEOMETRY = {
  123.0,  // front extent
  138.0,  // rear extent
  90.5,   // left extent
  90.5    // right extent
};

const FanSensorGeometry FAN_SENSOR_GEOMETRY[4] = {
  {95.0,  -67.0, -60.0},  // right outer
  {115.0, -30.0, -20.0},  // right inner
  {115.0,  30.0,  20.0},  // left inner
  {95.0,   67.0,  60.0}   // left outer
};

const FanSensorGeometry FAKE_REAR_TOF_GEOMETRY = {
  -ROBOT_FOOTPRINT_GEOMETRY.rearExtentMm,
  0.0,
  180.0
};

struct RangeSensorState {
  const char* name;
  int angleDeg;
  uint16_t distanceMm;
  bool valid;
  bool stale;
  bool blocked;
  unsigned long lastReadMs;
  unsigned long timeoutCount;
  unsigned long invalidCount;
};

extern RangeSensorState rangeSensors[RANGE_SENSOR_COUNT];

enum ObjectTofId {
  OBJECT_LEFT_LOW,
  OBJECT_LEFT_UPPER,
  OBJECT_RIGHT_LOW,
  OBJECT_RIGHT_UPPER,
  OBJECT_TOF_COUNT
};

enum ObjectTofRole {
  OBJECT_ROLE_LOW,
  OBJECT_ROLE_UPPER
};

enum ObjectCandidateKind {
  OBJECT_CANDIDATE_DISABLED,
  OBJECT_CANDIDATE_NONE,
  OBJECT_CANDIDATE_UNKNOWN,
  OBJECT_CANDIDATE_WEIGHT_SIZED,
  OBJECT_CANDIDATE_TALL_OBSTACLE
};

struct ObjectSensorGeometry {
  float xMm;
  float yMm;
  float zMm;
  float yawDeg;
  float pitchDeg;
  ObjectTofRole role;
};

struct ObjectSensorState {
  const char* name;
  ObjectTofRole role;
  uint16_t distanceMm;
  bool valid;
  bool stale;
  bool connected;
  unsigned long lastReadMs;
  unsigned long timeoutCount;
  unsigned long invalidCount;
  uint8_t rangeStatus;
  float signalMcps;
  float ambientMcps;
};

struct ObjectCandidateState {
  ObjectCandidateKind kind;
  const char* reason;
  bool confirmed;
  int directionHint;
  uint16_t rangeMm;
  uint8_t confirmCount;
  unsigned long lastUpdateMs;
};

struct ObjectTargetEstimate {
  bool valid;
  float robotXmm;
  float robotYmm;
  float worldX;
  float worldY;
  uint16_t rangeMm;
  uint8_t sourceMask;
  const char* reason;
  unsigned long lastUpdateMs;
};

extern const ObjectSensorGeometry OBJECT_SENSOR_GEOMETRY[OBJECT_TOF_COUNT];
extern ObjectSensorState objectSensors[OBJECT_TOF_COUNT];
extern ObjectCandidateState objectCandidate;
extern ObjectTargetEstimate objectTargetEstimate;

extern bool frontBlocked;
extern uint16_t frontDistance;
extern uint16_t leftDistance;
extern uint16_t rightDistance;

extern bool frontTofValid;
extern bool leftTofValid;
extern bool rightTofValid;

extern unsigned long lastFrontTofReadMs;
extern unsigned long lastLeftTofReadMs;
extern unsigned long lastRightTofReadMs;

extern int frontBlockCounter;
extern int frontClearCounter;

// =====================================================
// Obstacle avoidance
// =====================================================
const float AVOID_CLEARANCE_MARGIN_MM =50.0;
const float AVOID_SCORE_TIE_MARGIN_MM = 80.0;

enum AvoidTurnChoice {
  AVOID_TURN_LEFT,
  AVOID_TURN_RIGHT,
  AVOID_TURN_NONE
};

struct AvoidSideClearance {
  bool valid;
  bool passable;
  float innerSweepClearanceMm;
  float outerSweepClearanceMm;
  float scoreMm;
};

// =====================================================
// Stuck detection
// =====================================================
const float STUCK_COMMAND_SPEED_MIN = 1000.0;
const float STUCK_ENCODER_SPEED_MIN = 180.0;
const unsigned long DRIVE_STUCK_TIME_MS = 900;

const float WHEEL_MISMATCH_SPEED_MIN = 700.0;
const float WHEEL_MISMATCH_RATIO = 0.45;
const float WHEEL_MISMATCH_EXPECTED_RATIO = 0.70;
const unsigned long WHEEL_MISMATCH_TIME_MS = 900;

const float TURN_STUCK_YAW_MIN_DEG = 3.0;
const unsigned long TURN_STUCK_TIME_MS = 900;

extern bool driveStuck;
extern bool wheelMismatchStuck;
extern bool turnStuck;

extern unsigned long driveStuckStartMs;
extern unsigned long wheelMismatchStartMs;
extern unsigned long turnCheckStartMs;
extern float turnCheckStartYaw;

extern bool returnHomeRequested;

// =====================================================
// Main state machine
// =====================================================
enum RobotState {
  INIT,
  FOLLOW_PATH,
  APPROACH_OBJECT,
  COLLECT_SORT,
  RETURN_HOME,
  UNLOAD,
  OBSTACLE_AVOID,
  STUCK_RECOVERY,
  END_MATCH
};

extern RobotState currentState;
extern RobotState previousState;

extern int currentWaypointIndex;
extern bool endMatchPrinted;

extern float desiredForwardSpeed;
extern float desiredTurnSpeed;

extern int lastLeftMotorUs;
extern int lastRightMotorUs;

// =====================================================
// Turn settings
// =====================================================
const float TURN_TOLERANCE_DEG = 3.0;
const float SLOW_ZONE_DEG = 30.0;

const int TURN_RIGHT_LEFT_FAST_US  = 1850;
const int TURN_RIGHT_RIGHT_FAST_US = 1150;

const int TURN_LEFT_LEFT_FAST_US  = 1150;
const int TURN_LEFT_RIGHT_FAST_US = 1850;

const int TURN_RIGHT_LEFT_SLOW_US  = 1800;
const int TURN_RIGHT_RIGHT_SLOW_US = 1200;

const int TURN_LEFT_LEFT_SLOW_US  = 1200;
const int TURN_LEFT_RIGHT_SLOW_US = 1800;

const int WEIGHT_SCAN_TURN_OFFSET_MIN_US = 120;
const int WEIGHT_SCAN_TURN_OFFSET_MAX_US = 300;
const int DEFAULT_WEIGHT_SCAN_TURN_OFFSET_US = 280;
extern int weightScanTurnOffsetUs;

// =====================================================
// Waypoints
// =====================================================
const float WAYPOINT_TOLERANCE_M = 0.06;
const float WAYPOINT_LOOKAHEAD_M = 0.35;
const unsigned long WAYPOINT_ACTION_PAUSE_MS = 250;

// =====================================================
// Scheduled local navigation
// =====================================================
// All values below are safety limits or starting points for hardware-led
// calibration, not course-specific manoeuvre constants.
const unsigned long SENSOR_UPDATE_INTERVAL_MS = 20;
const unsigned long ODOMETRY_UPDATE_INTERVAL_MS = 20;
const unsigned long PLANNER_UPDATE_INTERVAL_MS = 40;
const unsigned long MOTOR_CONTROL_INTERVAL_MS = 20;

const int LOCAL_MAP_CELLS = 60;
const float LOCAL_MAP_CELL_M = 0.05;
const float LOCAL_MAP_SIZE_M = LOCAL_MAP_CELLS * LOCAL_MAP_CELL_M;
const float LOCAL_MAP_RECENTER_MARGIN_M = 0.60;
const float MAP_FREE_RAY_HALF_WIDTH_M = 0.035;
// Endpoint evidence is directional: a range return is uncertain along the
// beam and across its cone, not uniformly in a 100 mm circle. Outer beams are
// deliberately lower-confidence because they are more oblique and are used
// primarily to guide, rather than veto, a forward route.
const float MAP_INNER_ENDPOINT_BACK_UNCERTAINTY_M = 0.035;
const float MAP_INNER_ENDPOINT_FORWARD_UNCERTAINTY_M = 0.035;
const float MAP_INNER_ENDPOINT_LATERAL_UNCERTAINTY_M = 0.025;
const int MAP_INNER_ENDPOINT_DYNAMIC_EVIDENCE = 22;
const int MAP_INNER_ENDPOINT_STATIC_EVIDENCE = 3;
const float MAP_OUTER_ENDPOINT_BACK_UNCERTAINTY_M = 0.045;
const float MAP_OUTER_ENDPOINT_FORWARD_UNCERTAINTY_M = 0.045;
const float MAP_OUTER_ENDPOINT_LATERAL_UNCERTAINTY_M = 0.050;
const int MAP_OUTER_ENDPOINT_DYNAMIC_EVIDENCE = 8;
const int MAP_OUTER_ENDPOINT_STATIC_EVIDENCE = 1;
const unsigned long MAP_DYNAMIC_EXPIRY_MS = 1800;
const unsigned long MAP_STATIC_EXPIRY_MS = 10000;
const unsigned long MAP_TRAVERSED_EXPIRY_MS = 5000;

const int PLANNER_CURVATURE_SAMPLES = 13;
const int PLANNER_SPEED_SAMPLES = 2;
const float PLANNER_HORIZON_S = 0.80;
const float PLANNER_ROLLOUT_STEP_S = 0.10;
const float PLANNER_MAX_TURN_RATIO = 0.65;
const float PLANNER_MIN_SPEED_SCALE = 0.45;
// Lowest speed at which the current drivetrain has demonstrated sustained
// motion.  Below this, stopping is safer and more truthful than planning a
// trajectory the motors cannot execute.
const float PLANNER_MIN_DRIVABLE_SPEED_TPS = 1500.0;
const float PLANNER_OBSTACLE_SCORE_THRESHOLD = 20.0;
// Collision proof and preferred running room are intentionally separate.
// The hard budget must fit the actual body through a 400 mm straight passage;
// the 50 mm preferred value still biases arcs toward the centre when room
// exists.
const float PLANNER_COLLISION_CLEARANCE_M = 0.020;
const float PLANNER_MODEL_UNCERTAINTY_M = 0.010;
const float PLANNER_TOTAL_HARD_CLEARANCE_M =
  PLANNER_COLLISION_CLEARANCE_M + PLANNER_MODEL_UNCERTAINTY_M;
const float PLANNER_PREFERRED_CLEARANCE_M = 0.050;
const float PLANNER_MIN_PROGRESS_M = 0.03;
const float PLANNER_FRONT_SPEED_BUFFER_M = 0.06;
const float PLANNER_MAX_DECELERATION_MPS2 = 0.60;
const float PLANNER_SENSING_LATENCY_S = 0.12;
// At higher base speeds the robot must commit to a side escape before the
// virtual front reaches the emergency stop band. This cap applies only while
// an asymmetric inner-fan clearance escape is active; open-field driving still
// uses the requested base speed.
const float PLANNER_ESCAPE_SPEED_LIMIT_START_M = 0.48;
const float PLANNER_ESCAPE_SPEED_LIMIT_FULL_M = 0.28;
const float PLANNER_ESCAPE_MIN_SPEED_TPS = PLANNER_MIN_DRIVABLE_SPEED_TPS;
const float PLANNER_DEFAULT_SAFE_STOP_SPEED_MPS = 0.0;
const float PLANNER_TURN_TARGET_SPEED = 1050.0;
// This marker selects the original, physically calibrated slow-turn pulse
// pair inside the single motor-output path.
const float PLANNER_TURN_SLOW_TARGET_SPEED = 800.0;
// Turn-pulse coast calibration is directional. The positive/right turn
// counter-pulse is shorter because 40 ms drove its settled heading past the
// target, while the negative/left turn settles correctly at 40 ms.
const unsigned long PLANNER_TURN_RIGHT_BRAKE_PULSE_MS = 20;
const unsigned long PLANNER_TURN_LEFT_BRAKE_PULSE_MS = 40;
const unsigned long PLANNER_TURN_SENSOR_REVALIDATE_MS = 120;
// A narrow corridor can be traversed only when the robot enters aligned. The
// test is map-based and activates only after both lateral boundaries are
// observed nearby.
const float PLANNER_CORRIDOR_SIDE_SEARCH_M = 0.35;
const float PLANNER_CORRIDOR_MAX_WIDTH_M = 0.48;
const float PLANNER_CORRIDOR_MAX_TURN_RATIO = 0.12;
// When a point goal is mostly ahead and the robot is not doing a clearance
// escape, prefer the start-to-target line over pretty-but-drunken arcs.  This
// is deliberately route-shape scoring only; obstacle safety still comes from
// the hard footprint/fan checks above.
const float PLANNER_LINE_FOLLOW_ENABLE_HEADING_DEG = 45.0;
const float PLANNER_LINE_FOLLOW_LATERAL_TOLERANCE_M = 0.18;
const float PLANNER_LINE_FOLLOW_HEADING_TOLERANCE_DEG = 45.0;
const float PLANNER_NEAR_GOAL_STRAIGHTEN_DISTANCE_M = 0.35;
const float PLANNER_LINE_FOLLOW_NEAR_GOAL_MAX_TURN_RATIO = 0.25;
// A mostly-forward point goal is considered complete when the robot crosses
// the target plane inside this route corridor. The exact 60 mm point circle
// still applies to all goals; this band only prevents physical drift from
// turning a successful straight traverse into an endless point chase.
const float PLANNER_LINE_FOLLOW_FINISH_LATERAL_M = 0.16;
// Route-plane completion must also be nearly aligned so a gap traverse does
// not stop with the rear corner still sweeping close to the obstacle.
const float PLANNER_LINE_FOLLOW_FINISH_HEADING_DEG = 10.0;
// Do not let route-plane completion become "anywhere beyond the target" after
// the robot has overshot and later re-aligned. Finish only in this bounded
// along-track window around the target plane.
const float PLANNER_LINE_FOLLOW_FINISH_OVERSHOOT_M = 0.20;
// Object pickup is not a precision waypoint. TEST HUNT should still drive
// through almost all of the 150 mm carry distance, then finish without doing a
// tidy point-goal heading cleanup.
const float PLANNER_HUNT_FINISH_TARGET_TOLERANCE_M = 0.025;
const float PLANNER_HUNT_FINISH_LATERAL_M = 0.22;
const float PLANNER_HUNT_FINISH_OVERSHOOT_M = 0.35;
// Once the robot reaches the estimated weight point, add a small hunt-only
// speed boost for the forward carry-through. The braking/observation cap still
// wins if forward space is not proven clear.
const float PLANNER_HUNT_PICKUP_BOOST_ZONE_M =
  OBJECT_PICKUP_OVERSHOOT_MM / 1000.0;
const float PLANNER_HUNT_PICKUP_BOOST_TPS = 250.0;
const float PLANNER_HUNT_PICKUP_MAX_SPEED_TPS = 2850.0;
// If a gap traverse crosses the target plane while still a little too angled
// for the strict finish gate, stop in this short post-window instead of
// chasing the clamped lookahead point indefinitely. Beyond this window, abort
// the point goal as missed so the robot cannot run away from a passed target.
const float PLANNER_LINE_FOLLOW_MISSED_STOP_OVERSHOOT_M = 0.45;
// Point goals are forward-arc goals. If a new point lies behind the chassis,
// rotate in place first until the target is inside the arc planner's useful
// forward field. This prevents TEST GOTO 0 0 from driving farther away after a
// gap traverse that ended beyond the requested target.
const float PLANNER_POINT_ALIGN_START_DEG = 30.0;
const float PLANNER_POINT_ALIGN_BEHIND_DEG = 90.0;
const float PLANNER_POINT_ALIGN_SIDE_TIE_MM = 50.0;
const float PLANNER_CORRIDOR_SQUEEZE_HEADING_DEG = 25.0;
const float PLANNER_CORRIDOR_SQUEEZE_SPEED_TPS = 1500.0;
const float PLANNER_CORRIDOR_SQUEEZE_MIN_OBSERVED_M = 0.20;
// Reverse recovery is an explicit no-forward-path recovery. For this testing
// build, RANGE_FAKE_REAR is trusted as a real rear clearance sensor.
const unsigned long PLANNER_NO_PATH_BACKTRACK_DELAY_MS = 250;
const float PLANNER_REVERSE_RECOVERY_MAX_SPEED_TPS = 1700.0;
const float PLANNER_REVERSE_RECOVERY_MIN_SPEED_TPS = PLANNER_MIN_DRIVABLE_SPEED_TPS;
const float PLANNER_REVERSE_RECOVERY_MIN_SPEED_SCALE = 0.75;
const float PLANNER_REVERSE_RECOVERY_MAX_TURN_RATIO = 0.60;
const int PLANNER_REVERSE_RECOVERY_CURVATURE_SAMPLES = 9;
const float PLANNER_REVERSE_RECOVERY_REAR_BUFFER_M = 0.06;
const uint16_t TOF_SUDDEN_CLOSE_DROP_MM = 400;
const uint16_t TOF_CLOSE_CONFIRM_TOLERANCE_MM = 200;
const uint8_t TOF_CLOSE_CONFIRM_READS = 2;

enum NavigationGoalMode {
  NAV_GOAL_NONE,
  NAV_GOAL_POINT,
  NAV_GOAL_TURN
};

enum NavigationGoalOwner {
  NAV_OWNER_ROUTE,
  NAV_OWNER_RETURN_HOME,
  NAV_OWNER_TEST_DRIVE,
  NAV_OWNER_TEST_GOTO,
  NAV_OWNER_TEST_AVOID,
  NAV_OWNER_TEST_ESCAPE,
  NAV_OWNER_TEST_TURN,
  NAV_OWNER_TEST_HUNT,
  NAV_OWNER_WEIGHT_SCAN,
  NAV_OWNER_OBJECT_HUNT
};

enum PlannerStopReason {
  PLANNER_STOP_NONE,
  PLANNER_STOP_FRONT_BLOCKED,
  PLANNER_STOP_FRONT_INVALID,
  PLANNER_STOP_NO_SAFE_TRAJECTORY,
  PLANNER_STOP_TURN_SIDE_INVALID,
  PLANNER_STOP_TURN_CLEARANCE,
  PLANNER_STOP_STUCK,
  PLANNER_STOP_ABORTED
};

struct NavigationGoal {
  NavigationGoalMode mode;
  NavigationGoalOwner owner;
  bool active;
  bool completed;
  bool failed;
  float targetX;
  float targetY;
  float targetYawDeg;
  float startX;
  float startY;
  float startYawDeg;
  unsigned long startedMs;
};

struct PlannerTelemetry {
  float selectedForwardTicksPerSec;
  float selectedTurnTicksPerSec;
  float selectedCurvature;
  float minimumSweptClearanceMm;
  float speedCapTicksPerSec;
  float localGoalDistanceM;
  int candidateCount;
  PlannerStopReason stopReason;
  const char* planReason;
  const char* replanReason;
  const char* safeStopReason;
  unsigned long lastPlanMs;
};

extern NavigationGoal navigationGoal;
extern PlannerTelemetry plannerTelemetry;
extern bool motorStopRequested;
extern bool escapeBacktrackEnabled;
extern unsigned long lastSensorUpdateMs;
extern unsigned long lastOdometryUpdateMs;
extern unsigned long lastPlannerUpdateMs;
extern unsigned long lastMotorControlUpdateMs;

struct Waypoint {
  float x;
  float y;
  const char* action;
};

extern Waypoint path[];
extern const int NUM_POINTS;

// =====================================================
// Function declarations
// =====================================================
void leftISR();
void rightISR();

void goToPoint(float targetX, float targetY);
void runWaypointAction(const char* action);

void initializeNavigationController();
void updateRobotController();
void updateNavigationController();
void startNavigationPoint(float targetX, float targetY, NavigationGoalOwner owner);
void startNavigationTurn(float relativeTurnDeg, NavigationGoalOwner owner);
void cancelNavigationGoal(PlannerStopReason reason, const char* detail);
bool isNavigationGoalActive();
bool didNavigationGoalComplete();
bool didNavigationGoalFail();
void clearNavigationGoalResult();
const char* plannerStopReasonName(PlannerStopReason reason);
bool isTurnDirectionObservable(float turnTicksPerSec);
bool isTurnSweepSafe();
void clearLocalMap();
void updateLocalMapFromSensors();
void markTraversedFreeSpace();

AvoidTurnChoice evaluateAvoidTurnDirection(AvoidSideClearance &left,
                                           AvoidSideClearance &right,
                                           const char* &reason);

void updateStuckDriving(float leftTargetSpeed, float rightTargetSpeed,
                        float leftSpeed, float rightSpeed);
void resetTurnStuckCheck(float startYaw);
void updateStuckTurning(float currentYaw);
void clearStuckFlags();

void updateOdometry();

void connectIMU();
void zeroYaw();
float readYawDeg();
float navigationHeadingDeg();

void connectTOFSensors();
void connectRightOuterTOF();
void connectRightInnerTOF();
void connectLeftInnerTOF();
void connectLeftOuterTOF();
void updateTOFSensors();
void updateFanTOFSensors();
void prepareObjectTOFPinsForStartup();
void connectObjectTOFSensors();
void updateObjectTOFSensors();
void refreshObjectTargetEstimate();
void printObjectTelemetry();
bool isObjectTargetFresh();
bool isRangeSensorValid(RangeSensorId id);
bool isRangeSensorBlocked(RangeSensorId id);
uint16_t getRangeSensorDistance(RangeSensorId id);
bool isTofCloseReadingRevalidating();
float getFanSweepClearanceMm(RangeSensorId id);
bool getDiagonalClearanceWarning(RangeSensorId &sensorId, float &clearanceMm);
void printFanTelemetry();
const char* objectCandidateKindName(ObjectCandidateKind kind);
const char* objectSensorRoleName(ObjectTofRole role);

void stopMotors();
void writeMotorUS(int leftUs, int rightUs);
int updatePID(float target, float actual, float &integral, float &lastError, float dt, int baseCommand);
void updateMotorController();

float wrapAngle(float angle);
void resetEncodersAndPID();
void readEncoderCounts(long &leftCount, long &rightCount);
void printPose();
void printCalibrationSummary();

void setupBluetooth();
bool handleBluetoothCommands();
void sendBluetoothStatus();
void sendBluetoothTelemetry();
void sendBluetoothEvent(const char* eventName, const char* eventDetail);
bool isManualDriveActive();
void updateManualDriveTimeout();
void printWaitingForStart();

void runStateMachine();
void runInitState();
void runFollowPathState();
void runReturnHomeState();
void runUnusedState(const char* stateName);
void runEndMatchState();
void startWeightSearchTest();
bool isWeightSearchActive();
void cancelWeightSearch(const char* detail);
void setRobotState(RobotState newState);
const char* robotStateName(RobotState state);
void setMotionCommand(float forwardSpeed, float turnSpeed);

#endif
