#ifndef PLANNER_CONTEXT_H
#define PLANNER_CONTEXT_H

#include "PlannerTypes.h"

// Single statically allocated owner of all mutable local-planner state.
struct PlannerContext {
  // localMap is expressed in world metres, but stored as a small robot-centred
  // grid. shiftedMap is a scratch buffer used only when the robot has walked
  // far enough that the local grid needs to be re-centred around it.
  LocalMapCell localMap[LOCAL_MAP_CELLS][LOCAL_MAP_CELLS];
  LocalMapCell shiftedMap[LOCAL_MAP_CELLS][LOCAL_MAP_CELLS];
  bool localMapInitialized;
  float localMapOriginX;
  float localMapOriginY;
  uint32_t lastRearEvidenceFrameSequence;
  int8_t arenaMemoryChallenge[ARENA_MEMORY_BITS];
  uint8_t arenaMemoryOccupied[ARENA_MEMORY_BYTES];
  uint8_t arenaMemoryKnownClear[ARENA_MEMORY_BYTES];
  bool arenaMemoryInitialized;
  float arenaMemoryOriginX;
  float arenaMemoryOriginY;
  PlannerStopReason lastReportedStopReason;
  bool turnBrakeActive;
  unsigned long turnBrakeUntilMs;
  float turnLastCommandDirection;
  bool pointAlignTurnActive;
  float pointAlignTurnDirection;
  unsigned long turnSideInvalidSinceMs;
  unsigned long turnSweepInvalidSinceMs;
  bool reverseRecoveryActive;
  unsigned long reverseRecoveryStartedMs;
  unsigned long reverseRecoveryStepCount;
  unsigned long noSafeTrajectorySinceMs;
  uint8_t geometricNoPathEpochCount;
  bool lastForwardNoPathWasGeometric;
  bool candidateRejectsReported;
  bool reverseRecoveryRejectsReported;
  ReverseRecoveryState reverseRecoveryState;
  RecoveryBudget recoveryBudget;
  float lastFootprintRejectWorldX;
  float lastFootprintRejectWorldY;
  int lastFootprintRejectCellX;
  int lastFootprintRejectCellY;
  float lastCorridorRejectLeftM;
  float lastCorridorRejectRightM;
  EmergencyRecoveryState emergencyRecoveryState;
  bool emergencyScanPolicyEnabled;
  ObstacleContext obstacleContext;
  PlannerEpoch plannerEpoch;
  ReversePlannerEpoch reversePlannerEpoch;
  unsigned long lastPlannerCommandPublishedMs;
};

extern PlannerContext plannerContext;
extern PlannerTelemetry plannerTelemetry;
extern unsigned long lastPlannerUpdateMs;

void initializePlannerContextDefaults();

#endif
