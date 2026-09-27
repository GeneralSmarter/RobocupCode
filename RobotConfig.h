#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H

// =====================================================
// Calibration, geometry, timing, and safety constants
// =====================================================
// Responsibility:
//   Holds the firmware's tunable constants and hardware pin/address mapping.
//   Values here define real robot behavior: motor pulses, encoder scale,
//   sensor addresses, chassis geometry, planner margins, timing budgets,
//   and recovery limits.
// Interacts with:
//   All behavior modules read this file through Robot.h. MotorControl.cpp uses
//   motor/PID/timing values, and the fan/rear/matrix modules use sensor
//   configuration, navigation modules use geometry/planner/recovery values,
//   and Bluetooth.cpp exposes selected tuning knobs at runtime.
// Control flow:
//   No functions run here. static_assert checks catch configuration contracts
//   at compile time, such as planner command age staying shorter than the
//   motor command lease.
// Global state:
//   Constants are read-only at runtime except for runtime copies such as
//   baseTargetSpeed, leftForwardBaseUs, rightForwardBaseUs, and
//   weightScanTurnOffsetUs, which Bluetooth commands can adjust for this boot.
//
// COORDINATE CONVENTION: Robot geometry is measured from the midpoint between
// the drive wheels: +X forward, +Y robot-left, angles positive CCW/left.
// Units are encoded in names: mm, m, ms, us, ticks/s, and degrees.

#include <Arduino.h>
#include "RobotTypes.h"

// =====================================================
// Debug and build label
// =====================================================
const char ROBOT_BUILD_LABEL[] = "V7-permanent-obstacles-0720a";

// =====================================================
// Bluetooth serial debug link
// =====================================================
const unsigned long BLUETOOTH_BAUD = 115200;
// Telemetry is staged as a complete row and then copied atomically into this
// bounded queue.  The transmitter only writes bytes already reported free by
// HardwareSerial and never exceeds the per-loop budget, so logging cannot
// hold the control loop behind a full UART buffer.
const size_t TELEMETRY_QUEUE_CAPACITY_BYTES = 4096;
const size_t TELEMETRY_STAGE_CAPACITY_BYTES = 1792;
const size_t TELEMETRY_TX_BUDGET_BYTES_PER_LOOP = 48;
const size_t TELEMETRY_EVENT_QUEUE_RESERVE_BYTES = 768;
const unsigned long TELEMETRY_DUPLICATE_EVENT_LIMIT_MS = 250;
const unsigned long TELEMETRY_MOTION_INTERVAL_MS = 100;
const unsigned long TELEMETRY_FULL_INTERVAL_MS = 1000;
const char NAV_TELEMETRY_SCHEMA_VERSION[] = "3";

// The CSV column list, and the single place it is written. Bluetooth.cpp emits
// this verbatim as the header row, and sendBluetoothMotionRow() must print its
// values in exactly this order. The host side keeps the matching canonical list
// in SerialCommandUI/navigation_evidence.py.
//
// Some column names are frozen history: the wall_* group predates the current
// receding-horizon planner, which maps its closest typed state into them.
// Renaming any column is a schema change and needs NAV_TELEMETRY_SCHEMA_VERSION
// bumped plus the host list and saved regression logs updated.
constexpr char NAV_TELEMETRY_HEADER[] =
  "row_type,schema_version,seq,event,detail,ms,build,state,run,test_armed,"
  "x_m,y_m,theta_deg,fan0_mm,fan1_mm,fan2_mm,fan3_mm,fan0_valid,fan1_valid,"
  "fan2_valid,fan3_valid,fan0_age_ms,fan1_age_ms,fan2_age_ms,fan3_age_ms,"
  "blocked,motor_l_us,motor_r_us,wall_phase,wall_bypass_side,wall_nearest_mm,"
  "wall_phase_elapsed_s,wall_end_reads,wall_distance_past_end_m,"
  "route_lateral_error_m,planner_v_tps,planner_w_tps,"
  "planner_min_clearance_mm,planner_speed_cap_tps,planner_arc_result,"
  "planner_stop,wheel_target_l_tps,wheel_target_r_tps,wheel_rate_l_tps,"
  "wheel_rate_r_tps,imu_raw_cw_deg,nav_yaw_deg,motor_mode,motion_authority,"
  "lease_trips,loop_max_ms,loop_misses,loop_worst_phase,loop_worst_phase_us,"
  "telemetry_queued_rows,telemetry_queued_bytes,telemetry_dropped_rows,"
  "telemetry_rate_limited_events,planner_slice_max_us,planner_command_age_ms,"
  "planner_global_goal_distance_m";

// Derived, not hand-maintained: a miscount here used to be possible only to
// catch downstream, because the count and the header were written separately.
constexpr size_t navTelemetryFieldCount(const char* header) {
  size_t fields = 1;
  for (const char* cursor = header; *cursor != '\0'; cursor++) {
    if (*cursor == ',') {
      fields++;
    }
  }
  return fields;
}

constexpr size_t NAV_TELEMETRY_FIELD_COUNT =
  navTelemetryFieldCount(NAV_TELEMETRY_HEADER);

static_assert(navTelemetryFieldCount(NAV_TELEMETRY_HEADER) == 61,
              "Telemetry schema v3 has 61 columns; bump the schema version and "
              "update navigation_evidence.py before changing the column count");

// 115200 baud with 8N1 framing carries at most 11,520 bytes/s. Keep normal
// logging below 60% so navigation rows and event bursts cannot
// saturate the transport used by the control loop.
const size_t TELEMETRY_HEADER_BUDGET_BYTES = 1536;
const size_t TELEMETRY_MOTION_ROW_BUDGET_BYTES = 660;
const size_t TELEMETRY_EVENT_ROW_BUDGET_BYTES = 320;
const size_t TELEMETRY_EVENT_RESERVE_BYTES_PER_SECOND = 300;
const size_t TELEMETRY_DESIGNED_BYTES_PER_SECOND =
  TELEMETRY_MOTION_ROW_BUDGET_BYTES * (1000 / TELEMETRY_MOTION_INTERVAL_MS) +
  TELEMETRY_EVENT_RESERVE_BYTES_PER_SECOND;
static_assert(
  TELEMETRY_DESIGNED_BYTES_PER_SECOND * 100 <=
    (BLUETOOTH_BAUD / 10) * 60,
  "Telemetry design exceeds 60 percent of Bluetooth UART capacity");

// =====================================================
// Motors
// =====================================================
// Physically re-proven wheels-up on 2026-07-14. Servo output 0 drives the
// robot-right wheel and output 1 drives the robot-left wheel.
const int LEFT_MOTOR_PIN  = 1;
const int RIGHT_MOTOR_PIN = 0;

const int STOP_US = 1500;
const int MIN_US  = 1000;
const int MAX_US  = 1993;

// These calibrations stay with the physical wheel/output they were measured
// on; they move with the corrected side mapping above.
const int LEFT_BASE_US  = 1870;
const int RIGHT_BASE_US = 1935;

const int LEFT_REVERSE_US  = 1190;
const int RIGHT_REVERSE_US = 1130;

// =====================================================
// Encoders and drivetrain calibration
// =====================================================
// Encoder connector 2/3 follows the physical-right wheel; connector 4/5
// follows the physical-left wheel. Both were proven forward-positive with
// isolated wheel commands in the marked Task-01 logs.
const int LEFT_ENC_A  = 4;
const int LEFT_ENC_B  = 5;
const int RIGHT_ENC_A = 2;
const int RIGHT_ENC_B = 3;

// The installed encoders count negative while their physical wheels move
// forward. Normalize both channels here so the PID and odometry share the
// canonical forward-positive convention.
const int LEFT_ENCODER_SIGN  = -1;
const int RIGHT_ENCODER_SIGN = 1;

const float TICKS_PER_METRE = 9125.0;

// The distance between the left and right track centre lines must be measured
// on the robot before narrow-gap planning is accepted.  The provisional value
// is deliberately kept in one visible configuration value so that the planner
// model and its test logs cannot silently diverge.
const float EFFECTIVE_TRACK_WIDTH_M = 0.224;

// =====================================================
// Wheel speed PID
// =====================================================
const float DEFAULT_BASE_TARGET_SPEED = 2600.0f;

// =====================================================
// TOF sensors
// =====================================================
const uint16_t RANGE_NO_READING_MM = 9999;
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

const float WEIGHT_SEARCH_SWEEP_DEG = 30.0;
const float WEIGHT_SEARCH_CONFIRM_TURN_MIN_DEG = 5.0;
const float WEIGHT_SEARCH_CONFIRM_TURN_MAX_DEG = 35.0;
const float WEIGHT_SEARCH_STANDOFF_M = 0.25;
const unsigned long WEIGHT_SEARCH_SETTLE_MS = 200;
const unsigned long WEIGHT_SEARCH_CONFIRM_MS = 300;
const unsigned long WEIGHT_SEARCH_HUNT_TIMEOUT_MS = 5000;
const unsigned long WEIGHT_INTERRUPT_COOLDOWN_MS = 1000;
// Mission-level retry handoff. A retry never bypasses navigation safety; it
// only prevents a competition route from terminating on one typed failure.
const unsigned long COMPETITION_RETRY_PAUSE_MS = 750;
const float MATRIX_FOLLOW_SCAN_DEG = 30.0f;
const float MATRIX_FOLLOW_MAX_TARGET_TRAVEL_M = 0.50f;
const unsigned long MATRIX_FOLLOW_LOSS_STOP_MS = 300;
const unsigned long MATRIX_FOLLOW_REACQUIRE_TIMEOUT_MS = 2500;

// Three rear VL53L1X sensors use XSHUT7/5/6. XSHUT4 and address 0x37 are
// assigned to the independent internal-funnel payload-confirmation ToF below.
const uint8_t REAR_TOF_ROI_WIDTH = 16;
const uint8_t REAR_TOF_ROI_HEIGHT = 16;
const uint32_t REAR_TOF_TIMING_BUDGET_US = 50000;
const unsigned long REAR_TOF_SAMPLE_PERIOD_MS = 60;
const unsigned long REAR_TOF_STALE_TIMEOUT_MS = 250;
const unsigned long REAR_TOF_RECONNECT_INTERVAL_MS = 1000;
const unsigned long REAR_TOF_MAX_SAMPLE_SKEW_MS = 140;
const uint8_t REAR_TOF_CLEAR_CONFIRM_SAMPLES = 3;
const uint16_t REAR_TOF_VALID_MIN_MM = 40;
const uint16_t REAR_TOF_VALID_MAX_MM = 2000;
// Provisional software thresholds retained only as an initial conservative
// starting point; physical acceptance must replace them from saved stopping
// distance and uncertainty evidence.
const uint16_t REAR_TOF_STOP_DISTANCE_MM = 250;
const uint16_t REAR_TOF_CLEAR_DISTANCE_MM = 300;

constexpr RearTofConfig REAR_TOF_CONFIG[REAR_TOF_COUNT] = {
  {"rear_left", {-107.860f, 95.340f, 180.000f, 150.0f, 0.0f, 0.0f},
   SENSOR_I2C_PRIMARY, 7, 0x34, 16, 16, 27.0f, 27.0f,
   REAR_TOF_TIMING_BUDGET_US,
   REAR_TOF_SAMPLE_PERIOD_MS, REAR_TOF_VALID_MIN_MM, REAR_TOF_VALID_MAX_MM,
   REAR_TOF_STOP_DISTANCE_MM, REAR_TOF_CLEAR_DISTANCE_MM},
  {"rear_centre", {-112.860f, 0.000f, 180.000f, 180.0f, 0.0f, 0.0f},
   SENSOR_I2C_PRIMARY, 5, 0x35, 16, 16, 27.0f, 27.0f,
   REAR_TOF_TIMING_BUDGET_US,
   REAR_TOF_SAMPLE_PERIOD_MS, REAR_TOF_VALID_MIN_MM, REAR_TOF_VALID_MAX_MM,
   REAR_TOF_STOP_DISTANCE_MM, REAR_TOF_CLEAR_DISTANCE_MM},
  {"rear_right", {-107.860f, -95.340f, 180.000f, 210.0f, 0.0f, 0.0f},
   SENSOR_I2C_PRIMARY, 6, 0x36, 16, 16, 27.0f, 27.0f,
   REAR_TOF_TIMING_BUDGET_US,
   REAR_TOF_SAMPLE_PERIOD_MS, REAR_TOF_VALID_MIN_MM, REAR_TOF_VALID_MAX_MM,
   REAR_TOF_STOP_DISTANCE_MM, REAR_TOF_CLEAR_DISTANCE_MM}
};

static_assert(REAR_TOF_COUNT == 3, "Exactly three rear ToFs are required");
static_assert(REAR_TOF_ROI_WIDTH == 16 && REAR_TOF_ROI_HEIGHT == 16,
              "Rear VL53L1X sensors must use their full 16x16 ROI");
static_assert(REAR_TOF_CONFIG[0].xshutChannel !=
                REAR_TOF_CONFIG[1].xshutChannel &&
              REAR_TOF_CONFIG[0].xshutChannel !=
                REAR_TOF_CONFIG[2].xshutChannel &&
              REAR_TOF_CONFIG[1].xshutChannel !=
                REAR_TOF_CONFIG[2].xshutChannel,
              "Rear XSHUT channels must be unique");
static_assert(REAR_TOF_CONFIG[0].i2cAddress !=
                REAR_TOF_CONFIG[1].i2cAddress &&
              REAR_TOF_CONFIG[0].i2cAddress !=
                REAR_TOF_CONFIG[2].i2cAddress &&
              REAR_TOF_CONFIG[1].i2cAddress !=
                REAR_TOF_CONFIG[2].i2cAddress,
              "Rear I2C addresses must be unique on the primary bus");
constexpr bool sensorGeometryFinite(float value) {
  return value == value && value > -100000.0f && value < 100000.0f;
}
static_assert(REAR_TOF_CONFIG[0].bus == SENSOR_I2C_PRIMARY &&
                REAR_TOF_CONFIG[1].bus == SENSOR_I2C_PRIMARY &&
                REAR_TOF_CONFIG[2].bus == SENSOR_I2C_PRIMARY,
              "All rear VL53L1X sensors must remain on the primary bus");
static_assert(REAR_TOF_CONFIG[0].mount.xMm == REAR_TOF_CONFIG[2].mount.xMm &&
                REAR_TOF_CONFIG[0].mount.yMm ==
                  -REAR_TOF_CONFIG[2].mount.yMm &&
                REAR_TOF_CONFIG[0].mount.yawDeg +
                  REAR_TOF_CONFIG[2].mount.yawDeg == 360.0f,
              "Rear left/right geometry must remain mirrored");
static_assert(sensorGeometryFinite(REAR_TOF_CONFIG[0].mount.xMm) &&
                sensorGeometryFinite(REAR_TOF_CONFIG[0].mount.yMm) &&
                sensorGeometryFinite(REAR_TOF_CONFIG[1].mount.xMm) &&
                sensorGeometryFinite(REAR_TOF_CONFIG[2].mount.yMm),
              "Rear geometry must be finite");

// Internal-funnel payload confirmation. This channel is deliberately absent
// from RangeSensorId: it is not navigation, map, collision, or motor-safety
// evidence. The approximately 10 mm desired confirmation point is represented
// by a provisional <=15 mm band until physical traces establish the sensor's
// dependable short-range behavior.
const unsigned long PAYLOAD_TOF_SAMPLE_PERIOD_MS = 50;
const unsigned long PAYLOAD_TOF_STALE_TIMEOUT_MS = 250;
const unsigned long PAYLOAD_TOF_RECONNECT_INTERVAL_MS = 1000;
const uint8_t PAYLOAD_TOF_CONFIRM_SAMPLES = 3;
constexpr PayloadTofConfig PAYLOAD_TOF_CONFIG = {
  "payload_bottom", {-38.8f, 0.0f, 25.0f, 0.0f, 0.0f, 0.0f},
  SENSOR_I2C_PRIMARY, 4, 0x37, 20000,
  PAYLOAD_TOF_SAMPLE_PERIOD_MS, 4, 250, 15
};
static_assert(PAYLOAD_TOF_CONFIG.bus == SENSOR_I2C_PRIMARY,
              "Payload ToF must remain on the primary I2C bus");
static_assert(PAYLOAD_TOF_CONFIG.confirmationMaximumMm >=
                PAYLOAD_TOF_CONFIG.validMinimumMm &&
              PAYLOAD_TOF_CONFIG.confirmationMaximumMm <=
                PAYLOAD_TOF_CONFIG.validMaximumMm,
              "Payload confirmation band must be inside the valid range");
static_assert(REAR_TOF_CONFIG[0].xshutChannel !=
                PAYLOAD_TOF_CONFIG.xshutChannel &&
              REAR_TOF_CONFIG[1].xshutChannel !=
                PAYLOAD_TOF_CONFIG.xshutChannel &&
              REAR_TOF_CONFIG[2].xshutChannel !=
                PAYLOAD_TOF_CONFIG.xshutChannel &&
              REAR_TOF_CONFIG[0].i2cAddress !=
                PAYLOAD_TOF_CONFIG.i2cAddress &&
              REAR_TOF_CONFIG[1].i2cAddress !=
                PAYLOAD_TOF_CONFIG.i2cAddress &&
              REAR_TOF_CONFIG[2].i2cAddress !=
                PAYLOAD_TOF_CONFIG.i2cAddress,
              "Payload and rear ToF identities must remain unique");

const int FRONT_STOP_DISTANCE_MM  = 180;
const int FRONT_CLEAR_DISTANCE_MM = 230;

const int FRONT_VALID_MIN_MM = 20;
const int FRONT_VALID_MAX_MM = 8191;
const unsigned long TOF_STALE_TIMEOUT_MS = 750;

const int FRONT_BLOCK_CONFIRM_READS = 2;
const int FRONT_CLEAR_CONFIRM_READS = 3;

// Front SEN0628 configuration. Raw row zero is the physically lowest-looking
// row on the installed module. Flip rows before applying the vertical FoV so
// ground returns are evaluated as downward rays and discarded by the existing
// height filter, rather than being mistaken for close upright obstacles.
// The matrix begins as supplemental blocking evidence: valid close cells can
// veto motion, while unknown/no-return cells never establish known-clear
// space.
const uint8_t FRONT_MATRIX_TOF_I2C_ADDRESS = 0x33;
const uint16_t FRONT_MATRIX_TOF_VALID_MIN_MM = 20;
const uint16_t FRONT_MATRIX_TOF_VALID_MAX_MM = 3999;
const uint16_t FRONT_MATRIX_TOF_STOP_DISTANCE_MM = 180;
const uint16_t FRONT_MATRIX_TOF_CLEAR_DISTANCE_MM = 230;
const uint8_t FRONT_MATRIX_TOF_CLEAR_CONFIRM_FRAMES = 3;
const unsigned long FRONT_MATRIX_TOF_FRAME_INTERVAL_MS = 100;
const unsigned long FRONT_MATRIX_TOF_STALE_TIMEOUT_MS = 250;
const unsigned long FRONT_MATRIX_TOF_RESPONSE_TIMEOUT_MS = 200;
const unsigned long FRONT_MATRIX_TOF_MODE_TIMEOUT_MS = 1000;
const unsigned long FRONT_MATRIX_TOF_MODE_SETTLE_MS = 5000;
const unsigned long FRONT_MATRIX_TOF_RECONNECT_INTERVAL_MS = 1000;
constexpr uint64_t FRONT_MATRIX_ALL_CELLS_MASK = UINT64_MAX;
// The matrix's forward wall evidence is deliberately limited to the two
// central cells in its top logical row (row 0, columns 3 and 4). The physical
// module is still being characterized, so off-centre/low cells must not turn
// broad floor or chassis returns into a forward wall veto. The independent
// four-ray fan remains the primary forward safety coverage.
constexpr uint64_t FRONT_MATRIX_WALL_MASK =
  (UINT64_C(1) << 3) | (UINT64_C(1) << 4);
constexpr uint64_t FRONT_MATRIX_BOTTOM_HALF_MASK = UINT64_C(0xFFFFFFFF00000000);
// The lower half remains reserved for short weight-sized-object perception and
// does not establish collision evidence.
constexpr uint8_t FRONT_MATRIX_MAP_COLUMN_MASK = 0xFF;
constexpr FrontMatrixConfig FRONT_MATRIX_CONFIG = {
  "front_matrix", {125.0f, 0.0f, 96.0f, 0.0f, 0.0f, 0.0f}, //was 67.140f
  SENSOR_I2C_SECONDARY, FRONT_MATRIX_TOF_I2C_ADDRESS,
  8, 8, 60.0f, 60.0f, 0, true, false,
  FRONT_MATRIX_WALL_MASK, FRONT_MATRIX_BOTTOM_HALF_MASK
};
static_assert(FRONT_MATRIX_CONFIG.rows == 8 &&
                FRONT_MATRIX_CONFIG.columns == 8,
              "Front matrix must publish an 8x8 frame");
static_assert(FRONT_MATRIX_CONFIG.gridRotationQuarterTurns < 4,
              "Front matrix grid rotation must be 0/90/180/270 degrees");
static_assert(FRONT_MATRIX_CONFIG.bus == SENSOR_I2C_SECONDARY,
              "Front matrix must remain on Wire1/secondary bus");
static_assert((FRONT_MATRIX_WALL_MASK & FRONT_MATRIX_BOTTOM_HALF_MASK) == 0,
              "Matrix wall and weight zones must not overlap");
static_assert(FRONT_MATRIX_CONFIG.safetyCellMask == FRONT_MATRIX_WALL_MASK &&
                FRONT_MATRIX_CONFIG.perceptionCellMask ==
                  FRONT_MATRIX_BOTTOM_HALF_MASK,
              "Only the top-centre matrix cells are wall evidence; lower rows are weight-only");
static_assert(FRONT_MATRIX_MAP_COLUMN_MASK != 0,
              "At least one front matrix column must seed obstacle endpoints");
static_assert(sensorGeometryFinite(FRONT_MATRIX_CONFIG.mount.xMm) &&
                sensorGeometryFinite(FRONT_MATRIX_CONFIG.mount.yMm) &&
                sensorGeometryFinite(FRONT_MATRIX_CONFIG.mount.zMm),
              "Front matrix geometry must be finite");

// Matrix perception/hunt starting values. Only the lower matrix half feeds
// this classifier; wall/ramp classification is intentionally disabled until
// the front-matrix mounting is physically characterized.
const float MATRIX_WEIGHT_MIN_WIDTH_MM = 25.0f;
const float MATRIX_WEIGHT_MAX_WIDTH_MM = 85.0f;
const float MATRIX_WEIGHT_MIN_HEIGHT_MM = 35.0f;
const float MATRIX_WEIGHT_MAX_HEIGHT_MM = 95.0f;
const float MATRIX_WEIGHT_MIN_DEPTH_STEP_MM = 80.0f;
// A returned zone is a cone, not a pencil ray. Adjacent supporting cells
// overlap; this provisional factor avoids counting both complete cones as
// independent target width until stationary calibration supplies a fit.
const float MATRIX_CELL_EFFECTIVE_WIDTH_FRACTION = 0.67f;
const float MATRIX_MAX_STATIC_TARGET_SPEED_MPS = 0.08f;
const float MATRIX_TRACK_POSITION_NOISE_MM = 50.0f;
const float MATRIX_TRACK_ASSOCIATION_DISTANCE_MM = 180.0f;
const unsigned long MATRIX_TRACK_STALE_TIMEOUT_MS = 500;
const uint8_t MATRIX_STATIC_CONFIRM_FRAMES = 3;
const uint8_t MATRIX_MOVING_CONFIRM_FRAMES = 2;
const uint8_t MATRIX_TARGET_LOSS_CONFIRM_FRAMES = 2;
const float WEIGHT_HUNT_CENTER_DEADBAND_COLUMNS = 1.25f;
const float WEIGHT_HUNT_STEERING_GAIN_TPS_PER_COLUMN = 500.0f;
const float WEIGHT_HUNT_MAX_SPEED_TPS = 2600.0f;
const float WEIGHT_HUNT_MAX_TURN_TPS = 1000.0f;
constexpr float MATRIX_PICKUP_HANDOFF_GAP_MM = 30.0f;
const float MATRIX_FINAL_APPROACH_ARM_GAP_MM = 80.0f;
const float MATRIX_FINAL_APPROACH_MAX_PREDICTION_MM = 100.0f;
const unsigned long MATRIX_FINAL_APPROACH_MAX_PREDICTION_MS = 600;
const float MATRIX_HANDOFF_MAX_ERROR_COLUMNS = 1.5f;
const float PICKUP_MIN_FORWARD_FEED_DISTANCE_MM = 150.0f;
const unsigned long PICKUP_FEED_TIMEOUT_MS = 2500;
const float PICKUP_FEED_MAX_TURN_TPS = 600.0f;
const float WEIGHT_INTERRUPT_MAX_CROSSTRACK_M = 0.85f;
const float WEIGHT_INTERRUPT_MAX_ADDED_DISTANCE_M = 1.70f;
static_assert(MATRIX_PICKUP_HANDOFF_GAP_MM > 0.0f,
              "Matrix handoff gap must be positive");

// Robot-centred geometry in millimetres. The origin is the midpoint between
// the drive wheels: +X forward, +Y left. Keep future physical measurements in
// this one block so clearance logic stays relative to the chassis.
constexpr RobotFootprintGeometry ROBOT_FOOTPRINT_GEOMETRY = {
  122.850f,  // front extent
  122.850f,  // rear extent
  114.000f,  // left extent
  114.000f   // right extent
};

constexpr FrontFanSensorConfig FRONT_FAN_CONFIG[4] = {
  {"right_outer", {66.453f, -103.571f, 180.0f, -60.0f, 0.0f, 0.0f},
   RIGHT_OUTER_XSHUT, RIGHT_OUTER_ADDRESS},
  {"right_inner", {109.298f, -29.063f, 180.0f, -20.0f, 0.0f, 0.0f},
   RIGHT_INNER_XSHUT, RIGHT_INNER_ADDRESS},
  {"left_inner", {109.298f, 29.063f, 180.0f, 20.0f, 0.0f, 0.0f},
   LEFT_INNER_XSHUT, LEFT_INNER_ADDRESS},
  {"left_outer", {66.453f, 103.571f, 180.0f, 60.0f, 0.0f, 0.0f},
   LEFT_OUTER_XSHUT, LEFT_OUTER_ADDRESS}
};
static_assert(FRONT_FAN_CONFIG[0].mount.xMm ==
                FRONT_FAN_CONFIG[3].mount.xMm &&
              FRONT_FAN_CONFIG[0].mount.yMm ==
                -FRONT_FAN_CONFIG[3].mount.yMm &&
              FRONT_FAN_CONFIG[1].mount.xMm ==
                FRONT_FAN_CONFIG[2].mount.xMm &&
              FRONT_FAN_CONFIG[1].mount.yMm ==
                -FRONT_FAN_CONFIG[2].mount.yMm,
              "Front fan mounts must remain mirrored");
static_assert(FRONT_FAN_CONFIG[0].xshutChannel !=
                FRONT_FAN_CONFIG[1].xshutChannel &&
              FRONT_FAN_CONFIG[1].xshutChannel !=
                FRONT_FAN_CONFIG[2].xshutChannel &&
              FRONT_FAN_CONFIG[2].xshutChannel !=
                FRONT_FAN_CONFIG[3].xshutChannel,
              "Front fan XSHUT channels must remain unique");
static_assert(FRONT_FAN_CONFIG[0].xshutChannel !=
                PAYLOAD_TOF_CONFIG.xshutChannel &&
              FRONT_FAN_CONFIG[1].xshutChannel !=
                PAYLOAD_TOF_CONFIG.xshutChannel &&
              FRONT_FAN_CONFIG[2].xshutChannel !=
                PAYLOAD_TOF_CONFIG.xshutChannel &&
              FRONT_FAN_CONFIG[3].xshutChannel !=
                PAYLOAD_TOF_CONFIG.xshutChannel &&
              FRONT_FAN_CONFIG[0].i2cAddress !=
                PAYLOAD_TOF_CONFIG.i2cAddress &&
              FRONT_FAN_CONFIG[1].i2cAddress !=
                PAYLOAD_TOF_CONFIG.i2cAddress &&
              FRONT_FAN_CONFIG[2].i2cAddress !=
                PAYLOAD_TOF_CONFIG.i2cAddress &&
              FRONT_FAN_CONFIG[3].i2cAddress !=
                PAYLOAD_TOF_CONFIG.i2cAddress,
              "Payload and front-fan ToF identities must remain unique");

constexpr FanSensorGeometry FAN_SENSOR_GEOMETRY[4] = {
  {FRONT_FAN_CONFIG[0].mount.xMm, FRONT_FAN_CONFIG[0].mount.yMm,
   FRONT_FAN_CONFIG[0].mount.yawDeg},
  {FRONT_FAN_CONFIG[1].mount.xMm, FRONT_FAN_CONFIG[1].mount.yMm,
   FRONT_FAN_CONFIG[1].mount.yawDeg},
  {FRONT_FAN_CONFIG[2].mount.xMm, FRONT_FAN_CONFIG[2].mount.yMm,
   FRONT_FAN_CONFIG[2].mount.yawDeg},
  {FRONT_FAN_CONFIG[3].mount.xMm, FRONT_FAN_CONFIG[3].mount.yMm,
   FRONT_FAN_CONFIG[3].mount.yawDeg}
};

static_assert(ROBOT_FOOTPRINT_GEOMETRY.frontExtentMm == 122.850f &&
                ROBOT_FOOTPRINT_GEOMETRY.rearExtentMm == 122.850f &&
                ROBOT_FOOTPRINT_GEOMETRY.leftExtentMm == 114.000f &&
                ROBOT_FOOTPRINT_GEOMETRY.rightExtentMm == 114.000f,
              "Declared chassis footprint must remain 228.0 x 245.7 mm");

// =====================================================
// Obstacle avoidance
// =====================================================
const float AVOID_CLEARANCE_MARGIN_MM =50.0;
const float AVOID_SCORE_TIE_MARGIN_MM = 80.0;

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

// =====================================================
// Waypoints
// =====================================================
// Leave a small discrete-control allowance around the 60 mm arrival circle.
// Without it a minimum-drivable-speed arc can miss the sampled boundary by a
// few millimetres, overshoot, and perform a full turn-back despite having
// already passed safely through the intended waypoint region.
const float WAYPOINT_TOLERANCE_M = 0.065;
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
const unsigned long MAIN_LOOP_DEADLINE_MS = 60;
const unsigned long MOTOR_COMMAND_LEASE_MS = 150;
const unsigned long MOTOR_COMMAND_WATCHDOG_TICK_MS = 10;
// A point-plan epoch is serviced cooperatively. No one slice may consume the
// whole main-loop deadline, and an old complete command is neutralized before
// the independent 150 ms motor lease can expire.
// The forward planner evaluates 26 candidates. Physical traces showed that
// two candidates per slice stretched an epoch beyond the 120 ms command-age
// guard, producing repeated drive/neutral pulses before the wheels could build
// useful speed. Process one complete curvature band per slice; the measured
// worst two-candidate slice was about 3 ms, so the 25 ms budget remains below
// the 60 ms main-loop deadline while allowing a full band to finish.
const unsigned long PLANNER_SLICE_BUDGET_US = 25000;
const uint8_t PLANNER_MAX_CANDIDATES_PER_SLICE = 13;
const unsigned long PLANNER_COMMAND_MAX_AGE_MS = 120;
const unsigned long PLANNER_EPOCH_MAX_AGE_MS = 300;

static_assert(MOTOR_COMMAND_LEASE_MS % MOTOR_COMMAND_WATCHDOG_TICK_MS == 0,
              "Motor command lease must be an integer number of watchdog ticks");
static_assert(PLANNER_COMMAND_MAX_AGE_MS < MOTOR_COMMAND_LEASE_MS,
              "Planner command age guard must stop before the motor lease");
static_assert(PLANNER_SLICE_BUDGET_US < MAIN_LOOP_DEADLINE_MS * 1000UL,
              "Planner slice budget must remain below the loop deadline");

const int LOCAL_MAP_CELLS = 60;
const float LOCAL_MAP_CELL_M = 0.05;
const float LOCAL_MAP_SIZE_M = LOCAL_MAP_CELLS * LOCAL_MAP_CELL_M;
const float LOCAL_MAP_RECENTER_MARGIN_M = 0.60;
// Persistent arena memory uses the same 50 mm cells as the rolling planner
// map. A 12 m square centred on the pose at map reset contains every point in
// the 4.9 m x 2.4 m arena regardless of the robot's starting orientation.
const int ARENA_MEMORY_CELLS = 240;
const float ARENA_MEMORY_SIZE_M = ARENA_MEMORY_CELLS * LOCAL_MAP_CELL_M;
const int ARENA_MEMORY_EVIDENCE_THRESHOLD = 20;
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
// Rear matrix cells are collapsed into one ray per column. Their evidence is
// deliberately comparable to an outer fan ray: useful after repetition, but
// not enough for one noisy frame to create a hard obstacle by itself.
const float MAP_REAR_FREE_RAY_HALF_WIDTH_M = 0.020;
const int MAP_REAR_FREE_EVIDENCE = 4;
const float MAP_REAR_ENDPOINT_MIN_RANGE_UNCERTAINTY_M = 0.025;
const float MAP_REAR_ENDPOINT_RANGE_UNCERTAINTY_RATIO = 0.06;
const float MAP_REAR_ENDPOINT_MIN_LATERAL_UNCERTAINTY_M = 0.025;
const int MAP_REAR_ENDPOINT_DYNAMIC_EVIDENCE = 8;
const int MAP_REAR_ENDPOINT_STATIC_EVIDENCE = 1;
const unsigned long MAP_DYNAMIC_EXPIRY_MS = 1800;
const unsigned long MAP_STATIC_EXPIRY_MS = 10000;
const unsigned long MAP_TRAVERSED_EXPIRY_MS = 5000;

const int PLANNER_CURVATURE_SAMPLES = 13;
static_assert(PLANNER_MAX_CANDIDATES_PER_SLICE >= PLANNER_CURVATURE_SAMPLES,
              "Planner must finish at least one curvature band per slice");
const float PLANNER_HORIZON_S = 0.80;
const float PLANNER_ROLLOUT_STEP_S = 0.10;
// Keep collision sampling tied to arena geometry rather than command speed.
// A rollout may use a shorter time step so successive footprint checks move
// no more than half a map cell. The ten-degree angular bound is a secondary
// guard for future configurations; current wheel limits are already tighter.
const float PLANNER_ROLLOUT_MAX_SPATIAL_STEP_M = LOCAL_MAP_CELL_M * 0.5f;
const float PLANNER_ROLLOUT_MAX_HEADING_STEP_RAD = 0.174533f;
const float PLANNER_MAX_TURN_RATIO = 0.65;
// One forward ceiling applies to every point-planner context. Each sampled
// curve derives a lower safe speed from braking, wheel and swept-path limits.
const float PLANNER_FORWARD_MAX_SPEED_TPS = 2600.0f;
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
// If both outer fan endpoints are close to the body while both inner rays see
// a long clear centre, the returns contradict a safely open forward corridor:
// an edge can occupy the blind wedge between rays. Treat only that paired
// signature as a pinch instead of inflating every endpoint clearance.
const float PLANNER_FAN_PINCH_OUTER_CLEARANCE_M = 0.080f;
const float PLANNER_FAN_PINCH_INNER_OPEN_RANGE_M = 0.600f;
const float PLANNER_PREFERRED_CLEARANCE_M = 0.050;
const float PLANNER_MIN_PROGRESS_M = 0.03;
const float PLANNER_FRONT_SPEED_BUFFER_M = 0.06;
const float PLANNER_MAX_DECELERATION_MPS2 = 0.60;
const float PLANNER_SENSING_LATENCY_S = 0.12;
// A transient invalid frame remains fail-closed and may recover. A required
// front aggregate that stays invalid beyond this bound terminates the active
// point goal neutral instead of leaving navigation running forever.
const unsigned long PLANNER_FRONT_INVALID_ABORT_MS = 2500;
const float PLANNER_TURN_TARGET_SPEED = 1050.0;
// This marker selects the original, physically calibrated slow-turn pulse
// pair inside the single motor-output path.
const float PLANNER_TURN_SLOW_TARGET_SPEED = 800.0;
// Turn-pulse coast calibration is physical-direction-specific. After a right
// turn, a 20 ms left counter-pulse is sufficient; after a left turn, the right
// counter-pulse needs 40 ms. Command sign is canonical: positive is left/CCW.
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
// If a gap traverse crosses the target plane while still a little too angled
// for the strict finish gate, stop in this short post-window instead of
// chasing the clamped lookahead point indefinitely. Beyond this window, abort
// the point goal as missed so the robot cannot run away from a passed target.
const float PLANNER_LINE_FOLLOW_MISSED_STOP_OVERSHOOT_M = 0.45;
// If final approach is already close to the waypoint but every forward arc is
// blocked by the obstacle/wall beyond it, accept the safe reachable pose rather
// than repeatedly reversing into the same pocket. This is deliberately much
// tighter than the rejected broad detour-plane finish.
const float PLANNER_FINAL_BLOCKED_ACCEPTANCE_M = 0.16;
// Point goals are forward-arc goals. If a new point lies behind the chassis,
// rotate in place first until the target is inside the arc planner's useful
// forward field. This prevents TEST GOTO 0 0 from driving farther away after a
// gap traverse that ended beyond the requested target.
const float PLANNER_POINT_ALIGN_START_DEG = 30.0;
// Reverse recovery is an explicit no-forward-path recovery. The range-channel
// identifier is legacy; hasTrustedRearCoverage() is the non-bypassable
// capability gate for planner publication and final motor output.
const unsigned long PLANNER_NO_PATH_BACKTRACK_DELAY_MS = 250;
// Abort when neither forward planning nor trusted-rear recovery can make
// useful progress within this interval.
const unsigned long PLANNER_NO_PATH_ABORT_MS = 1200;
const float PLANNER_OBSTACLE_TURN_ROOM_M = 0.12;
const float PLANNER_OBSTACLE_COUNTERSTEER_LEAD_M = 0.10;
const float PLANNER_OBSTACLE_RECONSIDERED_COUNTERSTEER_LEAD_M = 0.20;
const float PLANNER_OBSTACLE_SIDE_COST_TIE_M = LOCAL_MAP_CELL_M * 0.25f;
// Outer-fan ranges are directly comparable as opposite route-side evidence
// only close to route alignment, or once the chassis is clearly anti-aligned
// and each ray has been transformed into the route frame.
const float PLANNER_SIDE_RANGE_COMPARABLE_HEADING_DEG = 10.0f;
const float PLANNER_SIDE_RANGE_ANTI_ALIGNED_DEG = 120.0f;
// Reconsider a latched side only after the chassis is at least facing the
// route half-plane; earlier sparse-map growth is orientation-biased.
const float PLANNER_SIDE_RECONSIDER_HEADING_DEG = 90.0f;
const float PLANNER_OBSTACLE_STAGE_TRANSITION_TOLERANCE_M = LOCAL_MAP_CELL_M;
// If the normal outward stage has no geometric path, one noisy occupied-cell
// shift must not trap the robot just short of the along-wall stage. This only
// changes the local target; the complete swept-footprint proof remains intact.
const float PLANNER_OBSTACLE_STAGE_NO_PATH_TOLERANCE_M =
  LOCAL_MAP_CELL_M * 1.5f;
const float PLANNER_OBSTACLE_ALIGN_FALLBACK_DEG =
  PLANNER_POINT_ALIGN_START_DEG * 0.5f;
const float PLANNER_BROAD_OBSTACLE_WIDTH_MULTIPLIER = 2.0f;
const float PLANNER_BROAD_OBSTACLE_EXTRA_CLEARANCE_M =
  PLANNER_COLLISION_CLEARANCE_M * 0.5f;
// The normal local-goal turn policy remains the first planner pass. Only when
// it finds no trajectory may a gentle opposite-sign rollout qualify by making
// a measured fraction of the remaining outward clearance.
const float PLANNER_OBSTACLE_COUNTERSTEER_PROGRESS_FRACTION = 0.05;
const float PLANNER_OBSTACLE_COUNTERSTEER_MAX_RATIO = 0.15;
const uint8_t PLANNER_REVERSE_MIN_GEOMETRIC_NO_PATH_EPOCHS = 2;
const float PLANNER_REVERSE_RECOVERY_MAX_SPEED_TPS = 1700.0;
const float PLANNER_REVERSE_RECOVERY_MIN_SPEED_TPS = PLANNER_MIN_DRIVABLE_SPEED_TPS;
const float PLANNER_REVERSE_RECOVERY_MIN_SPEED_SCALE = 0.75;
const float PLANNER_REVERSE_RECOVERY_MAX_TURN_RATIO = 0.60;
const int PLANNER_REVERSE_RECOVERY_CURVATURE_SAMPLES = PLANNER_CURVATURE_SAMPLES;
const float PLANNER_REVERSE_RECOVERY_REAR_BUFFER_M = 0.06;
const int PLANNER_REVERSE_CLEAR_EVIDENCE_THRESHOLD = 20;
// Reverse recovery may gradually tolerate small gaps between otherwise clear
// rear rays. Occupied cells, stale/blocked rear coverage and the hard
// footprint margin remain non-bypassable.
const unsigned long PLANNER_REVERSE_UNKNOWN_RAMP_MS = 3000;
constexpr float PLANNER_REVERSE_MAX_UNKNOWN_FRACTION = 0.30;
const float PLANNER_REVERSE_FORWARD_RECHECK_DISTANCE_M = 0.12;
static_assert(PLANNER_REVERSE_UNKNOWN_RAMP_MS > 0,
              "Reverse unknown allowance ramp must be nonzero");
static_assert(PLANNER_REVERSE_MAX_UNKNOWN_FRACTION >= 0.0 &&
                PLANNER_REVERSE_MAX_UNKNOWN_FRACTION <= 1.0,
              "Reverse unknown allowance must be a fraction");
const float PLANNER_REVERSE_CLEARANCE_CAP_M = 0.50;
const float PLANNER_REVERSE_CLEARANCE_BAND_M = LOCAL_MAP_CELL_M;
const float PLANNER_REVERSE_CLEARANCE_GAIN_M = LOCAL_MAP_CELL_M;
const uint8_t PLANNER_REVERSE_PLATEAU_EPOCHS = 3;
const float PLANNER_REVERSE_FORWARD_QUALITY_WEIGHT = 0.50;
const float PLANNER_REVERSE_UNEXPLORED_WEIGHT = 0.30;
const float PLANNER_REVERSE_SWEEP_CLEARANCE_WEIGHT = 0.15;
const float PLANNER_REVERSE_EFFICIENCY_WEIGHT = 0.05;

// Reverse recovery owns reverse-only resource limits. Forward obstacle
// bypasses are instead supervised against their active local goal.
const unsigned long PLANNER_REVERSE_RECOVERY_MAX_TIME_MS = 12000;
const float PLANNER_RECOVERY_MAX_CUMULATIVE_REVERSE_DISTANCE_M = 2.40;
// The 120 mm forward recheck can require several short repositioning arcs to
// build lateral clearance around a close obstacle. Attempts and cumulative
// reverse distance remain active across every handoff.
const uint8_t PLANNER_RECOVERY_MAX_COUNT = 6;
// A safe forward bypass must improve its current local objective by one map
// cell during each rolling window. Target changes start a fresh window.
const unsigned long PLANNER_OBSTACLE_PROGRESS_TIMEOUT_MS = 6000;
const float PLANNER_OBSTACLE_PROGRESS_EPSILON_M = LOCAL_MAP_CELL_M;
const float PLANNER_RECOVERY_TAKEOVER_PROGRESS_M = LOCAL_MAP_CELL_M;

// Ultimate recovery is deliberately opt-in. It may run only after ordinary
// recovery has exhausted an eligible resource or progress bound, and it never weakens the
// existing footprint, sensor-freshness, authority, or motor-safety gates.
const bool PLANNER_EMERGENCY_SCAN_ENABLED = false;
constexpr uint8_t PLANNER_EMERGENCY_SCAN_SECTORS = 12;
constexpr float PLANNER_EMERGENCY_SCAN_STEP_DEG = 30.0f;
const float PLANNER_EMERGENCY_SCAN_SWEEP_STEP_DEG = 5.0f;
const unsigned long PLANNER_EMERGENCY_SENSOR_WAIT_MS = 500;
const unsigned long PLANNER_EMERGENCY_SECTOR_TIMEOUT_MS = 2500;
const unsigned long PLANNER_EMERGENCY_RELOCATE_TIMEOUT_MS = 8000;
const unsigned long PLANNER_EMERGENCY_TOTAL_TIMEOUT_MS = 20000;
const float PLANNER_EMERGENCY_MAX_RELOCATION_M = 0.80f;
const float PLANNER_EMERGENCY_RECHECK_DISTANCE_M = 0.12f;
const float PLANNER_EMERGENCY_TARGET_CLEARANCE_M = 0.08f;
static_assert(PLANNER_EMERGENCY_SCAN_SECTORS *
                PLANNER_EMERGENCY_SCAN_STEP_DEG == 360.0f,
              "Emergency scan sectors must cover exactly one revolution");
static_assert(PLANNER_EMERGENCY_SENSOR_WAIT_MS <
                PLANNER_EMERGENCY_SECTOR_TIMEOUT_MS,
              "Emergency scan sensor wait must fit inside a sector");
static_assert(PLANNER_EMERGENCY_RELOCATE_TIMEOUT_MS <
                PLANNER_EMERGENCY_TOTAL_TIMEOUT_MS,
              "Emergency relocation must fit inside the total bound");
const uint16_t TOF_SUDDEN_CLOSE_DROP_MM = 400;
const uint16_t TOF_CLOSE_CONFIRM_TOLERANCE_MM = 200;
const uint8_t TOF_CLOSE_CONFIRM_READS = 2;

#endif
