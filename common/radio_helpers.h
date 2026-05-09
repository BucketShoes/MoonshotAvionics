// common/radio_helpers.h — Inline implementations shared by rocket and base.
// Pure logic over the constants in radio_config.h. No state of its own apart from hopSeq[]
// (which lives in each firmware's radio.cpp; this file just operates on it).

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

// ===================== HOP SEQUENCE DERIVATION =====================
// Fisher-Yates shuffle of channels 0–63 driven by HOP_SEED. After shuffling, if cmdChannel
// landed in the first NUM_HOP_CHANNELS entries it is swapped with entry [NUM_HOP_CHANNELS],
// so changing cmdChannel only perturbs one position rather than reshuffling everything.
// Both firmwares must call this with the same cmdChannel to stay in sync.

inline void rederiveHopSequence(uint8_t cmdChannel) {
  uint8_t pool[64];
  for (uint8_t i = 0; i < 64; i++) pool[i] = i;  //TODO: @@@@@ fixed channel/fewer channels for debugging %1

  uint32_t lcg = 0;
  for (uint8_t i = 0; i < HOP_SEED_LEN; i++) lcg = (lcg << 8) ^ HOP_SEED[i];

  for (uint8_t i = 63; i > 0; i--) {
    lcg = lcg * 1664525UL + 1013904223UL;  // Numerical Recipes LCG
    //Turned off actual hopping shuffle for testing
    uint8_t j = (uint8_t)(lcg >> 16) % (i + 1); 
    uint8_t tmp = pool[i]; pool[i] = pool[j]; pool[j] = tmp;
  }

  for (uint8_t i = 0; i < NUM_HOP_CHANNELS; i++) {
    if (pool[i] == cmdChannel) {
      uint8_t tmp = pool[i];
      pool[i] = pool[NUM_HOP_CHANNELS];
      pool[NUM_HOP_CHANNELS] = tmp;
      break;
    }
  }

  for (uint8_t i = 0; i < NUM_HOP_CHANNELS; i++) hopSeq[i] = pool[i];
}

// Returns the hop channel for a given slot index. Caller must have already
// called rederiveHopSequence() with the current cmdChannel.
inline uint8_t hopChannel(uint32_t slotIndex) {
  return hopSeq[slotIndex % NUM_HOP_CHANNELS];
}

// Find which hop position the currently-tuned channel sits at. Returns 0xFF if not found
// (e.g. command channel, backhaul channel — anything not in hopSeq[]).
inline uint8_t hopIndexOf(uint8_t channel) {
  for (uint8_t i = 0; i < NUM_HOP_CHANNELS; i++) {
    if (hopSeq[i] == channel) return i;
  }
  return 0xFF;
}

// Returns a string describing a WindowMode for logging.
inline const char* windowModeName(uint8_t win) {
  switch (win) {
    case 0:  return "TELEM";
    case 1:  return "CMD";
    case 2:  return "OFF";
    case 3:  return "LR";
    case 4:  return "FINDME";
    case 5:  return "BACKHAUL";
    case 9:  return "CONTINUE";
    default: return "?";
  }
}

// ===================== TX RATE ENCODING =====================
// Positive = Hz, negative = seconds-between-packets, 0 = disabled. +1 and -1 both = 1Hz.

static inline unsigned long txRateToIntervalUs(int8_t rate) {
  if (rate == 0) return 0;
  if (rate > 0)  return 1'000'000UL / (unsigned long)rate;
  return (unsigned long)(-(int)rate) * 1'000'000UL;
}

#endif // COMMON_RADIO_HELPERS_H
