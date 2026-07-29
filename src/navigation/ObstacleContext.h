#ifndef OBSTACLE_CONTEXT_H
#define OBSTACLE_CONTEXT_H

#include "Robot.h"

void resetObstacleContext(const char* reason);
bool updateObstacleContext(float targetX, float targetY);
void buildObstacleLocalGoal(float &localGoalX, float &localGoalY);

#endif
