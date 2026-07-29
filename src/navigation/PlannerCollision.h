#ifndef PLANNER_COLLISION_H
#define PLANNER_COLLISION_H

#include "PlannerTypes.h"

bool snapshotWorldOccupied(const PlannerCollisionSnapshot &snapshot,
                           float worldX, float worldY,
                           int *cellX = NULL, int *cellY = NULL);
bool footprintKnownClearOnSnapshot(
  const PlannerCollisionSnapshot &snapshot,
  float worldX, float worldY, float headingRad);
float footprintUnknownFractionOnSnapshot(
  const PlannerCollisionSnapshot &snapshot,
  float worldX, float worldY, float headingRad);
float currentReverseUnknownAllowance();
float poseKnownClearanceM(const PlannerCollisionSnapshot &snapshot,
                          float worldX, float worldY, float headingRad);
float rotationalEnvelopeClearanceM(
  const PlannerCollisionSnapshot &snapshot, float worldX, float worldY);
float projectedUnexploredScore(
  const PlannerCollisionSnapshot &snapshot,
  float worldX, float worldY, float headingRad);
bool footprintClearOnSnapshot(const PlannerCollisionSnapshot &snapshot,
                              float worldX, float worldY,
                              float headingRad);
bool turnSegmentFootprintClear(
  const PlannerCollisionSnapshot &snapshot,
  float worldX, float worldY, float startHeadingDeg,
  float direction, float sweepDeg);
bool isNarrowObservedCorridorOnSnapshot(
  const PlannerCollisionSnapshot &snapshot,
  float worldX, float worldY, float headingRad);

float minimumFanSweepClearanceMm();
bool isTurnDirectionObservable(float turnTicksPerSec);
bool isTurnSweepSafe();

#endif
