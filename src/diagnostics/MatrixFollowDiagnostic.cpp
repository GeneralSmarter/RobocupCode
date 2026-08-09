#include "../../Robot.h"
#include "../../Navigation.h"
#include "../../NavigationTest.h"

namespace {

MatrixFollowDiagnosticStatus followStatus = {
  MATRIX_FOLLOW_IDLE, 0, "idle"
};
unsigned long phaseStartedMs = 0;
unsigned long targetLastSeenMs = 0;
float lockedStartWorldX = 0.0f;
float lockedStartWorldY = 0.0f;
float lastColumnError = 0.0f;
bool scanReversed = false;

void setPhase(MatrixFollowDiagnosticPhase phase, const char* detail) {
  followStatus.phase = phase;
  followStatus.detail = detail;
  phaseStartedMs = millis();
}

void beginReacquire() {
  navigationCancel();
  setPhase(MATRIX_FOLLOW_BOUNDED_REACQUIRE,
           "target_lost_translation_stopped");
  const float direction = lastColumnError >= 0.0f ? 1.0f : -1.0f;
  navigationStartTestTurn(direction * MATRIX_FOLLOW_SCAN_DEG);
}

}  // namespace

bool startMatrixFollowDiagnostic() {
  if (followStatus.phase != MATRIX_FOLLOW_IDLE &&
      followStatus.phase != MATRIX_FOLLOW_STOPPED) return false;
  followStatus.trackId = 0;
  scanReversed = false;
  setPhase(MATRIX_FOLLOW_SLOW_SCAN, "explicit_start");
  return navigationStartTestTurn(MATRIX_FOLLOW_SCAN_DEG);
}

void stopMatrixFollowDiagnostic(const char* detail) {
  navigationCancel();
  requestMotionStop();
  setPhase(MATRIX_FOLLOW_STOPPED, detail);
}

void updateMatrixFollowDiagnostic() {
  if (followStatus.phase == MATRIX_FOLLOW_IDLE ||
      followStatus.phase == MATRIX_FOLLOW_STOPPED) return;

  MatrixTargetObservation target;
  const bool hasTarget = getMatrixTargetObservation(target);
  const unsigned long now = millis();

  if ((followStatus.phase == MATRIX_FOLLOW_SLOW_SCAN ||
       followStatus.phase == MATRIX_FOLLOW_ACQUIRE_STATIC_WEIGHT) &&
      hasTarget) {
    navigationCancel();
    followStatus.trackId = target.trackId;
    lockedStartWorldX = target.worldX;
    lockedStartWorldY = target.worldY;
    targetLastSeenMs = now;
    lastColumnError = target.columnError;
    setPhase(MATRIX_FOLLOW_LOCKED_TRACK, "static_track_latched");
    if (!navigationStartTestPoint(target.worldX, target.worldY,
                                  NAVIGATION_TEST_GOTO)) {
      stopMatrixFollowDiagnostic("follow_navigation_rejected");
    }
    return;
  }

  if (followStatus.phase == MATRIX_FOLLOW_SLOW_SCAN ||
      followStatus.phase == MATRIX_FOLLOW_ACQUIRE_STATIC_WEIGHT) {
    NavigationStatus status = navigationGetStatus();
    if (status.state != NAVIGATION_RUNNING) {
      navigationClearResult();
      if (!scanReversed) {
        scanReversed = true;
        setPhase(MATRIX_FOLLOW_ACQUIRE_STATIC_WEIGHT, "reverse_scan");
        navigationStartTestTurn(-2.0f * MATRIX_FOLLOW_SCAN_DEG);
      } else {
        stopMatrixFollowDiagnostic("no_static_weight_in_scan_bound");
      }
    }
    return;
  }

  if (followStatus.phase == MATRIX_FOLLOW_LOCKED_TRACK) {
    if (hasTarget && target.trackId == followStatus.trackId) {
      targetLastSeenMs = now;
      lastColumnError = target.columnError;
      if (hypotf(target.worldX - lockedStartWorldX,
                 target.worldY - lockedStartWorldY) >
          MATRIX_FOLLOW_MAX_TARGET_TRAVEL_M) {
        stopMatrixFollowDiagnostic("locked_track_motion_bound_exceeded");
        return;
      }
      if (!navigationRetargetDiagnosticPoint(target.worldX, target.worldY) &&
          navigationGetStatus().state != NAVIGATION_RUNNING) {
        navigationClearResult();
        navigationStartTestPoint(target.worldX, target.worldY,
                                 NAVIGATION_TEST_GOTO);
      }
    } else if (now - targetLastSeenMs >= MATRIX_FOLLOW_LOSS_STOP_MS) {
      beginReacquire();
    }
    return;
  }

  if (followStatus.phase == MATRIX_FOLLOW_BOUNDED_REACQUIRE) {
    if (hasTarget && target.trackId == followStatus.trackId) {
      navigationCancel();
      targetLastSeenMs = now;
      lastColumnError = target.columnError;
      setPhase(MATRIX_FOLLOW_LOCKED_TRACK, "same_track_reacquired");
      navigationStartTestPoint(target.worldX, target.worldY,
                               NAVIGATION_TEST_GOTO);
      return;
    }
    if (now - phaseStartedMs >= MATRIX_FOLLOW_REACQUIRE_TIMEOUT_MS) {
      stopMatrixFollowDiagnostic("reacquire_timeout");
    }
  }
}

MatrixFollowDiagnosticStatus getMatrixFollowDiagnosticStatus() {
  return followStatus;
}

bool isMatrixFollowDiagnosticActive() {
  return followStatus.phase != MATRIX_FOLLOW_IDLE &&
         followStatus.phase != MATRIX_FOLLOW_STOPPED;
}
