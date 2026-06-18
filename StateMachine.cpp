#include "Robot.h"

// =====================================================
// Main state machine
// =====================================================
void runStateMachine() {
  switch (currentState) {
    case INIT:
      runInitState();
      break;

    case FOLLOW_PATH:
      runFollowPathState();
      break;

    case APPROACH_OBJECT:
      runUnusedState("APPROACH_OBJECT");
      break;

    case COLLECT_SORT:
      runUnusedState("COLLECT_SORT");
      break;

    case RETURN_HOME:
      runReturnHomeState();
      break;

    case UNLOAD:
      runUnusedState("UNLOAD");
      break;

    case OBSTACLE_AVOID:
      // OBSTACLE_AVOID is entered from the front TOF override inside normal driving.
      // The existing tested avoidance routine still handles the actual sequence.
      setRobotState(FOLLOW_PATH);
      break;

    case STUCK_RECOVERY:
      // STUCK_RECOVERY is entered from stuck detection inside driving/turning.
      // The existing tested recovery routine still handles the actual sequence.
      setRobotState(FOLLOW_PATH);
      break;

    case END_MATCH:
      runEndMatchState();
      break;
  }
}

void runInitState() {
  setMotionCommand(0.0, 0.0);
  stopMotors();

  currentWaypointIndex = 0;
  endMatchPrinted = false;

  Serial.println();
  Serial.println("INIT complete. Starting FOLLOW_PATH.");

  setRobotState(FOLLOW_PATH);
}

void runFollowPathState() {
  setMotionCommand(baseTargetSpeed, 0.0);

  if (handleReturnHomePriority()) {
    return;
  }

  if (currentWaypointIndex >= NUM_POINTS) {
    setRobotState(END_MATCH);
    return;
  }

  Serial.println();
  Serial.print("Waypoint ");
  Serial.print(currentWaypointIndex + 1);
  Serial.print(" of ");
  Serial.println(NUM_POINTS);

  goToPoint(path[currentWaypointIndex].x, path[currentWaypointIndex].y);
  runWaypointAction(path[currentWaypointIndex].action);

  currentWaypointIndex++;

  if (currentWaypointIndex >= NUM_POINTS) {
    setRobotState(END_MATCH);
  }
}

void runReturnHomeState() {
  setMotionCommand(baseTargetSpeed, 0.0);

  Serial.println();
  Serial.println("RETURN_HOME: going to x=0.000 y=0.000");

  goToPoint(0.0, 0.0);

  Serial.println("RETURN_HOME complete.");
  setRobotState(END_MATCH);
}

void runUnusedState(const char* stateName) {
  setMotionCommand(0.0, 0.0);
  stopMotors();

  Serial.print(stateName);
  Serial.println(" not implemented yet. Returning to FOLLOW_PATH.");

  setRobotState(FOLLOW_PATH);
}

void runEndMatchState() {
  handleBluetoothCommands();
  setMotionCommand(0.0, 0.0);
  stopMotors();
  robotRunEnabled = false;

  if (!endMatchPrinted) {
    Serial.println();
    Serial.println("END_MATCH. Robot stopped.");
    printPose();
    printWaitingForStart();
    endMatchPrinted = true;
  }

  delay(1000);
}

void setRobotState(RobotState newState) {
  if (currentState != newState) {
    previousState = currentState;

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
    case APPROACH_OBJECT: return "APPROACH_OBJECT";
    case COLLECT_SORT: return "COLLECT_SORT";
    case RETURN_HOME: return "RETURN_HOME";
    case UNLOAD: return "UNLOAD";
    case OBSTACLE_AVOID: return "OBSTACLE_AVOID";
    case STUCK_RECOVERY: return "STUCK_RECOVERY";
    case END_MATCH: return "END_MATCH";
  }

  return "UNKNOWN";
}

void setMotionCommand(float forwardSpeed, float turnSpeed) {
  desiredForwardSpeed = forwardSpeed;
  desiredTurnSpeed = turnSpeed;
}
