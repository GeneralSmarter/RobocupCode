#ifndef PLANNER_MAP_H
#define PLANNER_MAP_H

#include "PlannerTypes.h"

void clearLocalMap();
void updateLocalMapFromSensors();
void markTraversedFreeSpace();

void plannerMapCaptureCollisionSnapshot(PlannerCollisionSnapshot &snapshot);
bool plannerMapWorldToCell(float worldX, float worldY,
                           int &cellX, int &cellY);
bool plannerMapCellOccupied(int cellX, int cellY);
void plannerMapCellCenter(int cellX, int cellY,
                          float &worldX, float &worldY);
float plannerMapFanForwardObservationDistanceM(RangeSensorId id);
float plannerNavigationHeadingRad();

#endif
