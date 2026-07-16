#ifndef MOTION_SAFETY_H
#define MOTION_SAFETY_H

// =====================================================
// Pure motion-safety policy helpers
// =====================================================
// Responsibility:
//   Defines why the final motor writer may reject motion and provides
//   constexpr policy functions for lease and direction-aware sensor safety.
// Interacts with:
//   MotorControl.cpp gathers live evidence, calls these policies, and turns
//   any non-clear result into neutral motor output plus telemetry.
// Control flow:
//   No hardware access happens here. The functions are pure so the policy can
//   be checked by static_assert and by host-side tests without a Teensy.
// Global state:
//   None. Callers provide all evidence as parameters.

enum MotionSafetyReason {
  MOTION_SAFETY_CLEAR,
  MOTION_SAFETY_FRONT_INVALID,
  MOTION_SAFETY_FRONT_BLOCKED,
  MOTION_SAFETY_CLOSE_REVALIDATING,
  MOTION_SAFETY_DIAGONAL_CLEARANCE,
  MOTION_SAFETY_REAR_INVALID,
  MOTION_SAFETY_REAR_BLOCKED,
  MOTION_SAFETY_TURN_SIDE_INVALID,
  MOTION_SAFETY_TURN_SWEEP_BLOCKED,
  MOTION_SAFETY_WATCHDOG_UNAVAILABLE,
  MOTION_SAFETY_COMMAND_LEASE_EXPIRED
};

constexpr MotionSafetyReason motorLeasePolicy(bool nonNeutral,
                                               bool watchdogReady,
                                               bool leaseArmed) {
  return !nonNeutral
           ? MOTION_SAFETY_CLEAR
         : !watchdogReady
           ? MOTION_SAFETY_WATCHDOG_UNAVAILABLE
         : !leaseArmed
           ? MOTION_SAFETY_COMMAND_LEASE_EXPIRED
         : MOTION_SAFETY_CLEAR;
}

struct MotionSafetyEvidence {
  // Each boolean is already direction-filtered by the caller. For example,
  // turnSideValid means the side being swept by the requested turn is valid,
  // not merely that some side sensor exists.
  bool frontValid;
  bool frontClear;
  bool closeReadingStable;
  bool diagonalClear;
  bool rearValid;
  bool rearClear;
  bool turnSideValid;
  bool turnSweepClear;
};

// Pure policy kept separate from sensor access so the direction contract is
// compile-time checkable. Compound motion must satisfy every relevant branch.
//
// SAFETY: Unknown evidence fails closed for the direction that needs it.
// Neutral motion deliberately remains available so cleanup can always write
// STOP_US even when sensors or watchdog setup are unhealthy.
constexpr MotionSafetyReason motionSafetyPolicy(
    bool needsForward, bool needsReverse, bool needsTurnSide,
    bool needsTurnSweep, MotionSafetyEvidence evidence) {
  return needsForward && !evidence.frontValid
           ? MOTION_SAFETY_FRONT_INVALID
         : needsForward && !evidence.frontClear
           ? MOTION_SAFETY_FRONT_BLOCKED
         : needsForward && !evidence.closeReadingStable
           ? MOTION_SAFETY_CLOSE_REVALIDATING
         : needsForward && !evidence.diagonalClear
           ? MOTION_SAFETY_DIAGONAL_CLEARANCE
         : needsReverse && !evidence.rearValid
           ? MOTION_SAFETY_REAR_INVALID
         : needsReverse && !evidence.rearClear
           ? MOTION_SAFETY_REAR_BLOCKED
         : needsTurnSide && !evidence.turnSideValid
           ? MOTION_SAFETY_TURN_SIDE_INVALID
         : needsTurnSweep && !evidence.turnSweepClear
           ? MOTION_SAFETY_TURN_SWEEP_BLOCKED
         : MOTION_SAFETY_CLEAR;
}

constexpr MotionSafetyEvidence MOTION_SAFETY_ALL_CLEAR = {
  true, true, true, true, true, true, true, true
};

static_assert(motionSafetyPolicy(false, false, false, false,
                                 {false, false, false, false,
                                  false, false, false, false}) ==
                MOTION_SAFETY_CLEAR,
              "Neutral must remain available without safety evidence");
static_assert(motionSafetyPolicy(true, false, false, false,
                                 {false, true, true, true,
                                  true, true, true, true}) ==
                MOTION_SAFETY_FRONT_INVALID,
              "Unknown forward space must block forward motion");
static_assert(motionSafetyPolicy(false, true, false, false,
                                 {true, true, true, true,
                                  false, true, true, true}) ==
                MOTION_SAFETY_REAR_INVALID,
              "Unknown rear space must block reverse motion");
static_assert(motionSafetyPolicy(false, false, true, true,
                                 {true, true, true, true,
                                  true, true, false, true}) ==
                MOTION_SAFETY_TURN_SIDE_INVALID,
              "Unknown swept side must block turning motion");
static_assert(motionSafetyPolicy(true, false, true, false,
                                 MOTION_SAFETY_ALL_CLEAR) ==
                MOTION_SAFETY_CLEAR,
              "A fully evidenced forward arc must remain allowed");
static_assert(motorLeasePolicy(false, false, false) == MOTION_SAFETY_CLEAR,
              "Neutral output must remain available without a watchdog");
static_assert(motorLeasePolicy(true, false, false) ==
                MOTION_SAFETY_WATCHDOG_UNAVAILABLE,
              "Non-neutral output must fail closed without a watchdog");
static_assert(motorLeasePolicy(true, true, false) ==
                MOTION_SAFETY_COMMAND_LEASE_EXPIRED,
              "Non-neutral output must require a live command lease");
static_assert(motorLeasePolicy(true, true, true) == MOTION_SAFETY_CLEAR,
              "A live lease must permit otherwise-safe motion");

#endif
