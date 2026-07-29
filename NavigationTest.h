#ifndef NAVIGATION_TEST_H
#define NAVIGATION_TEST_H

enum NavigationTestPointKind {
  NAVIGATION_TEST_DRIVE,
  NAVIGATION_TEST_GOTO,
  NAVIGATION_TEST_AVOID,
  NAVIGATION_TEST_ESCAPE,
  NAVIGATION_TEST_PICKUP
};

// Explicit test/simulator adapter. Production mission code uses Navigation.h.
bool navigationStartTestPoint(float worldX, float worldY,
                              NavigationTestPointKind kind);
bool navigationStartTestTurn(float relativeDegrees);
bool navigationStartPointForSimulator(float worldX, float worldY,
                                      int ownerCode);
bool navigationSimulatorOwnerUsesMissionAuthority(int ownerCode);
void navigationResetForSimulator();
void navigationUpdateForSimulator();

#endif
