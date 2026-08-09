#ifndef REAR_TOF_POLICY_H
#define REAR_TOF_POLICY_H

#include <stdint.h>

struct RearTofHysteresisState {
  bool blocked;
  uint8_t consecutiveClearSamples;
  uint32_t lastSequence;
};

constexpr RearTofHysteresisState updateRearTofHysteresis(
    RearTofHysteresisState state,
    bool valid,
    uint16_t distanceMm,
    uint32_t sequence,
    uint16_t stopDistanceMm,
    uint16_t clearDistanceMm,
    uint8_t clearConfirmSamples) {
  return sequence == state.lastSequence
    ? state
    : !valid || distanceMm <= stopDistanceMm
      ? RearTofHysteresisState{true, 0, sequence}
      : distanceMm >= clearDistanceMm
        ? RearTofHysteresisState{
            state.consecutiveClearSamples + 1U >= clearConfirmSamples
              ? false : state.blocked,
            state.consecutiveClearSamples < clearConfirmSamples
              ? (uint8_t)(state.consecutiveClearSamples + 1U)
              : clearConfirmSamples,
            sequence}
        : RearTofHysteresisState{state.blocked, 0, sequence};
}

constexpr bool rearTofSamplesCoherent(unsigned long oldestMs,
                                      unsigned long newestMs,
                                      unsigned long maximumSkewMs) {
  return newestMs >= oldestMs && newestMs - oldestMs <= maximumSkewMs;
}

static_assert(updateRearTofHysteresis(
                {false, 0, 1}, true, 200, 2, 250, 300, 3).blocked,
              "A close rear sample must block immediately");
static_assert(updateRearTofHysteresis(
                {true, 1, 4}, true, 350, 4, 250, 300, 3)
                .consecutiveClearSamples == 1,
              "Repeated rear samples must not advance clear confirmation");
static_assert(!updateRearTofHysteresis(
                updateRearTofHysteresis(
                  updateRearTofHysteresis({true, 0, 0}, true, 350, 1,
                                          250, 300, 3),
                  true, 350, 2, 250, 300, 3),
                true, 350, 3, 250, 300, 3).blocked,
              "Three distinct clear rear samples must clear hysteresis");

#endif
