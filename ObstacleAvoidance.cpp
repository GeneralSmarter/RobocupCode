#include "Robot.h"

// =====================================================
// Compatibility diagnostics for the local planner
// =====================================================
// Obstacle avoidance is no longer a scripted manoeuvre.  The only decision
// retained here is the stationary fan diagnostic used by TEST SIDE.

static void assessAvoidSideClearance(AvoidTurnChoice choice,
                                     AvoidSideClearance &assessment) {
  const RangeSensorId innerId = choice == AVOID_TURN_LEFT ? RANGE_LEFT_INNER : RANGE_RIGHT_INNER;
  const RangeSensorId outerId = choice == AVOID_TURN_LEFT ? RANGE_LEFT_OUTER : RANGE_RIGHT_OUTER;

  assessment.valid = isRangeSensorValid(innerId) && isRangeSensorValid(outerId);
  assessment.passable = false;
  assessment.innerSweepClearanceMm = -1.0;
  assessment.outerSweepClearanceMm = -1.0;
  assessment.scoreMm = -1.0;
  if (!assessment.valid) {
    return;
  }

  assessment.innerSweepClearanceMm = getFanSweepClearanceMm(innerId);
  assessment.outerSweepClearanceMm = getFanSweepClearanceMm(outerId);
  assessment.scoreMm = min(assessment.innerSweepClearanceMm,
                           assessment.outerSweepClearanceMm);
  assessment.passable = assessment.scoreMm >= AVOID_CLEARANCE_MARGIN_MM;
}

AvoidTurnChoice evaluateAvoidTurnDirection(AvoidSideClearance &left,
                                           AvoidSideClearance &right,
                                           const char* &reason) {
  assessAvoidSideClearance(AVOID_TURN_LEFT, left);
  assessAvoidSideClearance(AVOID_TURN_RIGHT, right);

  if (left.passable && !right.passable) {
    reason = "only_left_footprint_safe";
    return AVOID_TURN_LEFT;
  }
  if (!left.passable && right.passable) {
    reason = "only_right_footprint_safe";
    return AVOID_TURN_RIGHT;
  }
  if (!left.passable && !right.passable) {
    reason = (!left.valid && !right.valid) ? "no_valid_side_corridor" : "no_footprint_safe_corridor";
    return AVOID_TURN_NONE;
  }
  if (left.scoreMm > right.scoreMm + AVOID_SCORE_TIE_MARGIN_MM) {
    reason = "left_more_footprint_clearance";
    return AVOID_TURN_LEFT;
  }
  if (right.scoreMm > left.scoreMm + AVOID_SCORE_TIE_MARGIN_MM) {
    reason = "right_more_footprint_clearance";
    return AVOID_TURN_RIGHT;
  }
  reason = "similar_footprint_clearance_default_right";
  return AVOID_TURN_RIGHT;
}
