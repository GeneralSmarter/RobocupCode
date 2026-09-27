#ifndef ROUTE_MISSION_H
#define ROUTE_MISSION_H

void initializeRouteMission();
void updateRouteMission();
void resetRouteMission();
int routeMissionDisplayIndex();
int routeMissionPointCount();
bool setCompetitionModeEnabled(bool enabled);
bool isCompetitionModeEnabled();

#endif
