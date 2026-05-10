// radio.h — RadioLib-based LoRa link for the rocket.
// Continuous RX with DIO1 IRQ; non-blocking TX when telemetry is due.
// No slot scheduling, no hopping, no scan/anchor logic — that lived in the
// previous custom-driver design and has been removed.

#ifndef RADIO_H
#define RADIO_H

#include <Arduino.h>
#include <SPI.h>
#include <esp_timer.h>
#include "config.h"

// ===================== RADIO STATE =====================

enum RadioState {
  RADIO_OFF,         // not yet initialised
  RADIO_RX,          // listening (default)
  RADIO_TX,          // transmit in flight, waiting for TxDone IRQ
};

// ===================== HARDWARE OBJECTS =====================

extern SPIClass loraSPI;

// ===================== RADIO STATE =====================

extern bool       loraReady;
extern RadioState radioState;

// ===================== ACTIVE RADIO CONFIG =====================
// Loaded from NVS at boot, updated by CMD_SET_RADIO. radioApplyConfig() reads
// these globals and pushes them to the chip.

extern uint8_t activeChannel;
extern uint8_t activeSF;
extern int8_t  activePower;
extern float   activeFreqMHz;
extern float   activeBwKHz;

// ===================== STATS =====================

extern uint32_t txCount;
extern uint32_t rxCount;
extern uint32_t txFailCount;
extern uint32_t rxFailCount;
extern int64_t  lastValidCmdUs;     // esp_timer_get_time() of last accepted command

// ===================== PUBLIC API =====================

// Derive activeFreqMHz and activeBwKHz from activeChannel.
void updateActiveFreqBw();

// True if a verified command was heard within ROCKET_NO_BASE_HEARD_THRESHOLD_US.
inline bool radioInSync() {
  return (lastValidCmdUs != 0 &&
          (esp_timer_get_time() - lastValidCmdUs) < (int64_t)ROCKET_NO_BASE_HEARD_THRESHOLD_US);
}

// Initialise SX1262 via RadioLib, attach DIO1 IRQ, and start continuous RX.
// Returns true on success. Blocking ≤ ~50 ms; only call from setup().
bool radioInit();

// Push activeChannel/SF/Power/Freq/Bw to the chip (non-blocking; sub-ms).
// Safe to call any time the radio is not mid-TX. Re-arms RX after applying.
void radioApplyConfig();

// Try to transmit a packet. Returns true if TX started, false if busy/error.
// On true: state becomes RADIO_TX; radioPoll() flips back to RADIO_RX on TxDone IRQ.
bool radioStartTransmit(const uint8_t* pkt, size_t len);

// Pump the radio state machine: read RX packets, handle TxDone, restart RX.
// Call every loop iteration. Worst-case ~1 ms (SPI read of one packet).
void radioPoll();

#endif // RADIO_H
