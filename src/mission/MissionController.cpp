#include "../../Robot.h"
#include "../../Navigation.h"
#include "RouteMission.h"
#include "WeightSearch.h"

// =====================================================
// Mission controller
// =====================================================
// Responsibility:
//   Owns high-level mission progression: boot init, waypoint route following,
//   return-home handling, and END_MATCH cleanup.
// Interacts with:
//   Navigation.h is the only driving interface. WeightSearch.cpp owns the
//   object-search sub-state-machine. MotorControl.cpp remains the only
//   periodic motor-output owner.
// Control flow:
//   RobotCode.ino calls updateMissionController() only while autonomous execution is
//   enabled. This file assigns goals and observes typed results; it never
//   writes servo pulses directly.

static bool endMatchSafetyTransitionActive = false;
static bool returnHomeGoalPending = false;
static unsigned long returnHomeRetryAfterMs = 0;

void updateMissionController() {
  // Dispatches only implemented mission states. Obstacle avoidance and
  // recovery are internal navigation behavior, not mission states.
  switch (currentState) {
    case INIT:
      runInitState();
      break;
    case FOLLOW_PATH:
      runFollowPathState();
      break;
    case RETURN_HOME:
      runReturnHomeState();
      break;
    case END_MATCH:
      runEndMatchState();
      break;
  }
}

void runInitState() {
  // Resets mission-owned latches and assigns the first active state. It does
  // not reset yaw/pose; ZERO owns coordinate reset explicitly.
  requestMotionStop();
  motorStopRequested = true;
  initializeRouteMission();
  returnHomeGoalPending = false;
  returnHomeRetryAfterMs = 0;
  endMatchPrinted = false;
  navigationClearResult();

  Serial.println();
  Serial.println("INIT complete. Starting FOLLOW_PATH.");
  setRobotState(FOLLOW_PATH);
}

void runFollowPathState() {
  if (returnHomeRequested) {
    if (navigationGetStatus().state == NAVIGATION_RUNNING) {
      navigationCancel();
    }
    if (isWeightSearchActive()) {
      cancelWeightSearch("return_home_requested");
    }
    navigationClearResult();
    returnHomeGoalPending = false;
    setRobotState(RETURN_HOME);
    return;
  }
  updateRouteMission();
}

void runReturnHomeState() {
  // Assigns a single navigation point goal at world origin and waits for its
  // result. Normal mode leaves failure visible and neutral; competition mode
  // resubmits the same goal after a neutral handoff.
  NavigationStatus navigation = navigationGetStatus();
  if (!returnHomeGoalPending && navigation.state == NAVIGATION_IDLE) {
    Serial.println(
      "RETURN_HOME: assigning local navigation goal x=0.000 y=0.000");
    returnHomeGoalPending = navigationGoTo(0.0f, 0.0f);
    return;
  }

  if (returnHomeGoalPending && navigation.state == NAVIGATION_FAILED) {
    if (!isCompetitionModeEnabled()) {
      // Preserve the typed failure and neutral output for normal operation.
      return;
    }
    const unsigned long now = millis();
    if (returnHomeRetryAfterMs == 0) {
      returnHomeRetryAfterMs = now + COMPETITION_RETRY_PAUSE_MS;
      requestMotionStop();
      sendBluetoothEvent("competition_home_retry_wait", navigation.detail);
      return;
    }
    if (now < returnHomeRetryAfterMs) {
      requestMotionStop();
      return;
    }
    navigationClearResult();
    returnHomeGoalPending = false;
    returnHomeRetryAfterMs = 0;
    sendBluetoothEvent("competition_home_retry", "navigation_failed");
    return;
  }

  if (returnHomeGoalPending && navigation.state == NAVIGATION_REACHED) {
    Serial.println("RETURN_HOME complete.");
    navigationClearResult();
    returnHomeGoalPending = false;
    returnHomeRetryAfterMs = 0;
    setRobotState(END_MATCH);
  }
}

void runEndMatchState() {
  // END_MATCH is the normal stopped terminal state.
  requestMotionStop();
  robotRunEnabled = false;

  if (!endMatchPrinted) {
    Serial.println();
    Serial.println("END_MATCH. Robot stopped.");
    printPose();
    printWaitingForStart();
    endMatchPrinted = true;
  }
}

static void enforceEndMatchMotionSafety() {
  if (endMatchSafetyTransitionActive) {
    stopMotors();
    return;
  }

  endMatchSafetyTransitionActive = true;
  // Revoke first so cleanup cannot allow an old goal to republish a command.
  revokeMotionAuthority();
  disarmBluetoothMotionModes();
  if (navigationGetStatus().state == NAVIGATION_RUNNING) {
    navigationCancel();
  }
  if (isWeightSearchActive()) {
    cancelWeightSearch("end_match");
  }
  robotRunEnabled = false;
  stopMotors();
  endMatchSafetyTransitionActive = false;
}

void setRobotState(RobotState newState) {
  if (newState == END_MATCH) {
    enforceEndMatchMotionSafety();
  }
  if (currentState != newState) {
    Serial.print("STATE: ");
    Serial.print(robotStateName(currentState));
    Serial.print(" -> ");
    Serial.println(robotStateName(newState));
  }
  currentState = newState;
}

const char* robotStateName(RobotState state) {
  switch (state) {
    case INIT: return "INIT";
    case FOLLOW_PATH: return "FOLLOW_PATH";
    case RETURN_HOME: return "RETURN_HOME";
    case END_MATCH: return "END_MATCH";
  }
  return "UNKNOWN";
}

void requestMotionStop() {
  stopMotors();
}
