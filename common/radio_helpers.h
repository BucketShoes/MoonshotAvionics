// common/radio_helpers.h — Inline helpers shared by rocket and base.
// Pure logic over the constants in radio_config.h.

#ifndef COMMON_RADIO_HELPERS_H
#define COMMON_RADIO_HELPERS_H

#include <stdint.h>
#include "radio_config.h"

// ===================== CHANNEL ↔ FREQUENCY =====================

static inline float channelToFreqMHz(uint8_t ch) {
  if (ch < 64) return 915.2f + ch * 0.2f;
  if (ch < 72) return 915.9f + (ch - 64) * 1.6f;
  return 0.0f;
}

static inline float channelToBwKHz(uint8_t ch) {
  return (ch < 64) ? 125.0f : 500.0f;
}

// ===================== TX RATE ENCODING =====================
// Positive = Hz, negative = seconds-between-packets, 0 = disabled. +1 and -1 both = 1Hz.

static inline unsigned long txRateToIntervalUs(int8_t rate) {
  if (rate == 0) return 0;
  if (rate > 0)  return 1'000'000UL / (unsigned long)rate;
  return (unsigned long)(-(int)rate) * 1'000'000UL;
}

#endif // COMMON_RADIO_HELPERS_H
