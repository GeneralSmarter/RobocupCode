#ifndef PLANNER_EPOCH_LIFECYCLE_H
#define PLANNER_EPOCH_LIFECYCLE_H

#include <string.h>

#include "../../Robot.h"
#include "NavigationInternal.h"
#include "PlannerContext.h"

// Shared cooperative-epoch scaffolding for forward and reverse planning.
//
// Both directions run the same lifecycle: capture one immutable snapshot,
// evaluate a bounded number of candidates per main-loop pass, then revalidate
// the winner against fresh sensors before publication. The *algorithms* differ
// substantially — which sensors gate a candidate, how arcs are generated, how
// they are scored — and are deliberately left separate. What lives here is only
// the lifecycle, which was previously written twice and is exactly where a fix
// applied to one direction would silently leave the other unguarded.
//
// Templated on the epoch type rather than unified behind a common base struct:
// PlannerEpoch and ReversePlannerEpoch already use the same field names for all
// of this, so templates share the code without rewriting several hundred field
// accesses across two safety-critical files.

// Zeroes an epoch and stamps the ownership every epoch carries: which goal and
// which authority it was planned for. plannerEpochOwnerChanged() later compares
// against exactly these, so construction and invalidation stay in step.
// Direction-specific fields are filled by the caller afterwards.
template <typename Epoch>
void beginPlannerEpochState(Epoch &epoch, float goalX, float goalY) {
  memset(&epoch, 0, sizeof(epoch));
  epoch.active = true;
  epoch.startedMs = millis();
  epoch.goalStartedMs = navigationGoal.startedMs;
  epoch.authority = navigationGoal.authority;
  epoch.goalX = goalX;
  epoch.goalY = goalY;
  epoch.bestScore = -1000000.0f;
}

// Per-epoch counters published in STATUS/CSV. Note the slice/work *maxima* are
// deliberately not reset here: resetPlannerEpoch() owns those, because they are
// reported across an entire goal rather than per epoch.
inline void resetPlannerEpochTelemetry() {
  plannerTelemetry.candidateCount = 0;
  plannerTelemetry.plannerCandidatesProcessed = 0;
  plannerTelemetry.plannerYieldCount = 0;
  plannerTelemetry.plannerEpochWorkUs = 0;
  plannerTelemetry.plannerEpochAgeMs = 0;
  plannerTelemetry.plannerEpochActive = true;
}

// Marks an epoch complete and records its age. Any chosen command has already
// been published by the time this runs.
template <typename Epoch>
void closePlannerEpochState(Epoch &epoch) {
  epoch.active = false;
  epoch.awaitingRevalidation = false;
  plannerTelemetry.plannerEpochActive = false;
  plannerTelemetry.plannerEpochAgeMs = millis() - epoch.startedMs;
}

// Accumulates one slice of planning work into the epoch and the timing
// telemetry published by STATUS/CSV.
template <typename Epoch>
void recordPlannerEpochSlice(Epoch &epoch, unsigned long sliceStartedUs,
                             const char* phaseName) {
  const unsigned long sliceUs = micros() - sliceStartedUs;
  epoch.accumulatedWorkUs += sliceUs;
  plannerTelemetry.plannerSliceUs = sliceUs;
  plannerTelemetry.plannerSliceMaxUs =
    max(plannerTelemetry.plannerSliceMaxUs, sliceUs);
  plannerTelemetry.plannerEpochWorkUs = epoch.accumulatedWorkUs;
  plannerTelemetry.plannerEpochMaxWorkUs =
    max(plannerTelemetry.plannerEpochMaxWorkUs, epoch.accumulatedWorkUs);
  recordMainLoopPhaseDuration(phaseName, sliceStartedUs);
}

// A new goal, or a change of motion authority, invalidates the snapshot
// outright: the epoch was planned for something that is no longer current.
template <typename Epoch>
bool plannerEpochOwnerChanged(const Epoch &epoch) {
  return epoch.goalStartedMs != navigationGoal.startedMs ||
         epoch.authority != navigationGoal.authority;
}

// Applies the two age guards every epoch shares. Returns true when the epoch
// has been abandoned, in which case the caller must return
// TRAJECTORY_PLAN_ABORTED without publishing anything.
//
// SAFETY: the command-age guard forces neutral once the last published command
// is older than PLANNER_COMMAND_MAX_AGE_MS while the motor lease is armed, so a
// stalled planner cannot keep an old command alive. The epoch-age guard then
// fails the whole goal rather than planning from a stale snapshot.
template <typename Epoch>
bool servicePlannerEpochAgeGuards(Epoch &epoch, const char* timeoutDetail) {
  const unsigned long now = millis();
  plannerTelemetry.plannerEpochAgeMs = now - epoch.startedMs;
  plannerTelemetry.plannerCommandAgeMs =
    plannerContext.lastPlannerCommandPublishedMs == 0
      ? 0 : now - plannerContext.lastPlannerCommandPublishedMs;
  if (!epoch.commandStoppedForAge && isMotorCommandLeaseArmed() &&
      plannerTelemetry.plannerCommandAgeMs >= PLANNER_COMMAND_MAX_AGE_MS) {
    stopMotors();
    epoch.commandStoppedForAge = true;
    plannerTelemetry.safeStopReason = "planner_command_age_guard";
  }
  if (now - epoch.startedMs > PLANNER_EPOCH_MAX_AGE_MS) {
    stopMotors();
    closePlannerEpochState(epoch);
    finishNavigationGoal(false, PLANNER_STOP_ABORTED, timeoutDetail);
    return true;
  }
  return false;
}

// One candidate-evaluation slice is bounded by both a candidate count and a
// wall-clock budget, so a slow slice cannot overrun the main-loop deadline.
// Always evaluates at least one candidate so an epoch cannot stall.
inline bool plannerSliceBudgetReached(uint8_t processedThisSlice,
                                      unsigned long sliceStartedUs) {
  return processedThisSlice > 0 &&
         (processedThisSlice >= PLANNER_MAX_CANDIDATES_PER_SLICE ||
          micros() - sliceStartedUs >= PLANNER_SLICE_BUDGET_US);
}

#endif
