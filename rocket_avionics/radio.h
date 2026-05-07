// radio.h — LoRa radio hardware, TX/RX state machine, and slot-clock sync.
// Owns the SX1262 via sx126x_driver, all radio state, and slot-clock timing.

#ifndef RADIO_H
#define RADIO_H

#include <SPI.h>
#include "radio_hal.h"    // sx126x_hal_context_t, dio1Fired, dio1TimestampUs()
#include "sx126x.h"        // sx126x_driver API
#include "config.h"        // includes ../common/radio_config.h

// ===================== RADIO STATE =====================

enum RadioState {
  RADIO_OFF,         // not yet initialised
  RADIO_STANDBY,     // sx126x in STDBY_RC, BUSY low, ready for commands
  RADIO_RX_ACTIVE,   // sx126x_set_rx called, waiting for DIO1
  RADIO_TX_ACTIVE,   // sx126x_set_tx called, waiting for DIO1
};

// ===================== HARDWARE OBJECTS =====================

extern SPIClass             loraSPI;
extern sx126x_hal_context_t radioCtx;

// ===================== RADIO STATE =====================

extern bool       loraReady;
extern RadioState radioState;

// ===================== ACTIVE RADIO CONFIG =====================
// Loaded from NVS at boot, updated by CMD_SET_RADIO.

extern uint8_t activeChannel;  // runtime command channel (from NVM)
extern uint8_t activeSF;
extern int8_t  activePower;
extern float   activeFreqMHz;
extern float   activeBwKHz;

// ===================== SLOT CLOCK STATE =====================

extern unsigned long syncAnchorUs;
extern uint32_t      syncSeedSlotIndex;  // slot_index value at anchor time
extern unsigned long lastValidCmdUs;     // timestamp of last received valid command

// ===================== MODULATION SNAPSHOT =====================
// Saved at the start of each RX or TX operation. Used for airtime calculations.
// Must not be inferred from current slot type — settings can change between start and done.

extern sx126x_mod_params_lora_t savedModParams;
extern sx126x_pkt_params_lora_t savedPktParams;
extern bool                      savedIsLR;  // true if saved params use implicit header

// ===================== STATS =====================

extern uint16_t delayedTxCount;
extern uint16_t invalidRxCount;

// ===================== PUBLIC API =====================

// Derive activeFreqMHz and activeBwKHz from activeChannel.
void updateActiveFreqBw();

// Returns true if a valid command was received within ROCKET_NO_BASE_HEARD_THRESHOLD_US.
inline bool radioInSync() {
  return (lastValidCmdUs != 0 &&
          (micros() - lastValidCmdUs) < ROCKET_NO_BASE_HEARD_THRESHOLD_US);
}

// Returns current slot index (pure calculation from anchor — no stored variable).
inline uint32_t radioGetSlotIndex() {
  return (uint32_t)((micros() - syncAnchorUs) / SLOT_DURATION_US) + syncSeedSlotIndex;
}

// Initialise radio hardware.
bool radioInit();

// *** BLOCKING — INIT ONLY ***
// Apply radio parameters from NVS or CMD_SET_RADIO. Radio must be in RADIO_STANDBY.
// Contains DO_NOT_CALL_WHILE_ARMED_radioWaitBusy_WARNING_LONG_BLOCKING calls (up to 100ms each).
void radioApplyConfig_BLOCKING();

// Start windowed RX with a timeout in raw RTC steps (15.625 µs per tick).
// Saves modulation params for later airtime calc. isLR = true for implicit-header mode.
void radioStartRxTimeout(uint32_t timeoutRtcSteps,
                         const sx126x_mod_params_lora_t& modParams,
                         const sx126x_pkt_params_lora_t& pktParams,
                         bool isLR);

// Start async TX. Returns true if TX started.
bool radioStartTx(const uint8_t* pkt, size_t len,
                  const sx126x_mod_params_lora_t& modParams,
                  const sx126x_pkt_params_lora_t& pktParams,
                  bool isLR);

// Put radio in standby.
void radioStandby();

// Build the 3-byte on-air core for the 0xBB long-range packet.
size_t buildLRPacketCore(uint8_t* buf);

// Main non-blocking radio update: slot-based TX/RX scheduling.
void nonblockingRadio();

#endif // RADIO_H
