#ifndef WEIGHT_SEARCH_H
#define WEIGHT_SEARCH_H

void initializeWeightSearch();
void startWeightSearchTest();
bool isWeightSearchActive();
void cancelWeightSearch(const char* detail);
void updateWeightSearch();
bool tryStartRouteWeightInterrupt(float routeTargetX, float routeTargetY,
                                  bool currentActionIsSearch);
bool startSearchWaypointApproach(float searchX, float searchY,
                                 float approachOriginX,
                                 float approachOriginY,
                                 const char* detail);
void beginWaypointWeightSearch(float searchX, float searchY,
                               const char* detail);

enum WeightSearchResult {
  WEIGHT_SEARCH_RESULT_IDLE,
  WEIGHT_SEARCH_RESULT_RUNNING,
  WEIGHT_SEARCH_RESULT_COMPLETED,
  WEIGHT_SEARCH_RESULT_FAILED
};

enum WeightSearchOrigin {
  WEIGHT_SEARCH_ORIGIN_NONE,
  WEIGHT_SEARCH_ORIGIN_TEST,
  WEIGHT_SEARCH_ORIGIN_WAYPOINT,
  WEIGHT_SEARCH_ORIGIN_ROUTE_INTERRUPT
};

struct WeightSearchStatus {
  WeightSearchResult result;
  WeightSearchOrigin origin;
  const char* detail;
};

WeightSearchStatus getWeightSearchStatus();
void clearWeightSearchResult();

#endif
