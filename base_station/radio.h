// base_station/radio.h — LoRa radio, slot clock, and sync for base station.

#ifndef BS_RADIO_H
#define BS_RADIO_H

#include <Arduino.h>
#include <SPI.h>
#include "radio_hal.h"    // sx126x_hal_context_t, dio1Fired, dio1TimestampUs()
#include "sx126x.h"        // sx126x_driver API
#include "../common/radio_config.h"

// ===================== PIN / PROTOCOL CONSTANTS =====================

#define LORA_NSS_PIN  8
#define LORA_SCK_PIN  9
#define LORA_MOSI_PIN 10
#define LORA_MISO_PIN 11
#define LORA_RST_PIN  12
#define LORA_BUSY_PIN 13
#define LORA_DIO1_PIN 14

#define LED_PIN       18

// LED_MODE: 0 = logic-driven (default, follows radio state machine decisions).
// 1 = BUSY-pin-driven (directly mirrors SX1262 BUSY pin state for direct radio observation).
#define LED_MODE_LOGIC 0
#define LED_MODE_BUSY  1
#define LED_MODE       LED_MODE_LOGIC

#define LORA_CR       5      // 4/5 coding rate
#define LORA_PREAMBLE 8      // preamble symbols

#define PKT_LONGRANGE  0xBB
// WIN_LR slot config. BW follows activeChannel (same as normal slots).
// SF is separate — reduce for bench testing to shorten air time.
#define LORA_LR_SF    11
#define LORA_LR_CR    SX126X_LORA_CR_4_5

#define FAVORITE_ROCKET_DEVICE_ID  0x92

// ===================== RADIO TIMING CONSTANTS =====================

// Base station RX: start listening this many µs before the slot boundary.
#define BS_RX_EARLY_US          20'000UL
// Base station TX: start command TX this many µs after the slot boundary.
#define BS_CMD_TX_OFFSET_US      5'000UL
// If remaining time in an RX slot is below this threshold, skip starting RX (count as overrun).
#define BS_RX_MIN_REMAINING_US  60'000UL
// Rocket telem RX window (synced). Base needs to cover the full slot minus margins.
#define BS_RX_TIMEOUT_US        (SLOT_DURATION_US - 20'000UL)
// Maximum consecutive unexpected slot overruns before forcing radio standby.
#define BS_OVERRUN_MAX          3

#define LORA_RX_BOOSTED true

// ===================== DEBUG LOG FLAGS =====================
#define LOG_RX_DONE    true
#define LOG_RX_TIMEOUT false
#define LOG_RX_START   false
#define LOG_TX_START   true
#define LOG_TX_DONE    true

// ===================== RADIO STATE =====================

enum BsRadioState { BS_RADIO_STANDBY, BS_RADIO_RX_ACTIVE, BS_RADIO_TX_ACTIVE };

extern SPIClass             bsLoraSPI;
extern sx126x_hal_context_t bsRadioCtx;
extern BsRadioState         bsRadioState;
extern bool                 bsLoraReady;

// ===================== ACTIVE RADIO CONFIG =====================

#define DEFAULT_CHANNEL  3
#define DEFAULT_SF       9
#define DEFAULT_POWER    -9

extern uint8_t activeChannel;  // runtime command channel (from NVM); also used for TX power/SF
extern uint8_t activeSF;
extern int8_t  activePower;
extern float   activeFreqMHz;
extern float   activeBwKHz;

#define DEFAULT_BH_CHANNEL 67
#define DEFAULT_BH_SF      5
#define DEFAULT_BH_POWER   -9

extern uint8_t bhChannel;
extern uint8_t bhSF;
extern int8_t  bhPower;

// ===================== PASSIVE SYNC STATE MACHINE =====================
// The base station acquires slot timing by listening, not by sending a sync command.
// States:
//   SCAN_SEARCHING — continuous RX on HOP_SEQUENCE[2], waiting for first valid 0xAF
//   SCAN_CANDIDATE — following candidate anchor, waiting for confirming packet (30s timeout)
//   SCAN_LOCKED    — running normal slotted operation with confirmed anchor
//
// "In sync" is a live condition, not a stored boolean:
//   bsInSync() returns true if a valid telem packet was received within ±50ms of its
//   expected slot start, within the last 125 seconds.

enum BsScanState {
  SCAN_SEARCHING,
  SCAN_CANDIDATE,
  SCAN_LOCKED,
};

extern BsScanState bsScanState;

// Anchor and slot offset for the running clock.
// candidateAnchorUs / candidateSeedSlot are live during SCAN_CANDIDATE; committed on lock.
// backupAnchorUs / backupSeedSlot hold the pre-scan values for revert on failure.
extern unsigned long bsSyncAnchorUs;
extern uint32_t      bsSyncSeedSlotIndex;  // slot_index value at anchor time

extern unsigned long bsCandidateAnchorUs;
extern uint32_t      bsCandidateSeedSlotIndex;
extern float         bsCandidateDriftEmaUs;  // drift accumulated during candidate phase

extern unsigned long bsBackupAnchorUs;
extern uint32_t      bsBackupSeedSlotIndex;
extern float         bsBackupDriftEmaUs;

// Drift EMA applied to bsSyncAnchorUs on each WIN_TELEM RxDone.
extern float bsDriftEmaUs;

// Scan timers (millis)
extern unsigned long bsScanStartMs;       // when current SEARCHING phase started
extern unsigned long bsCandidateStartMs;  // when CANDIDATE phase started

#define BS_SCAN_TOTAL_MS      60'000UL   // total time in SEARCHING before reverting
#define BS_CANDIDATE_TIMEOUT_MS 30'000UL // time in CANDIDATE before restarting search

// Timing quality tracking for bsInSync()
extern unsigned long bsLastGoodTelemUs;    // micros() of last telem with |error| < 50ms
extern uint32_t      bsLastTelemErrorUs;   // |rxStart - expected slot start| of last telem packet

#define BS_IN_SYNC_TIMING_US    50'000UL    // max timing error to count as "in sync"
#define BS_IN_SYNC_TIMEOUT_US   125'000'000UL // 125 seconds

inline bool bsInSync() {
  return (bsLastGoodTelemUs != 0) &&
         ((unsigned long)(micros() - bsLastGoodTelemUs) < BS_IN_SYNC_TIMEOUT_US) &&
         (bsLastTelemErrorUs < BS_IN_SYNC_TIMING_US);
}

// ===================== SLOT CLOCK (derived) =====================

// Current slot index derived from anchor.
inline uint32_t bsGetSlotIndex() {
  return (uint32_t)((micros() - bsSyncAnchorUs) / SLOT_DURATION_US) + bsSyncSeedSlotIndex;
}

// ===================== OTHER STATE =====================

// Background RSSI EMA — noise floor estimate.
extern float bsBgRssiEma;

// Track when last rocket telemetry was received (millis).
extern unsigned long bsLastTelemRxMs;

// Track when last command was sent (millis).
extern unsigned long bsLastCmdSentMs;

// Set by bsHandleRadio() when WIN_CMD slot is ready for TX dispatch.
// main.cpp calls dispatchCmdTx() then clears this flag.
extern bool bsWinCmdReady;

// Modulation params saved at the start of each RX/TX operation.
// Used for airtime calculation at RxDone — do NOT infer from current slot type.
extern sx126x_mod_params_lora_t bsSavedModParams;
extern sx126x_pkt_params_lora_t bsSavedPktParams;
extern bool                      bsSavedIsLR;  // true if saved params are LR (implicit header)

// ===================== PUBLIC API =====================

void bsUpdateActiveFreqBw();

// Initialise radio hardware. Call once in setup() after SPI is started.
bool bsRadioInit();

// Apply current activeChannel/activeSF/activePower to hardware. Radio must be in standby.
// BLOCKING — init only. Contains DO_NOT_CALL_WHILE_ARMED_radioWaitBusy_WARNING_LONG_BLOCKING calls.
void bsRadioApplyConfig_BLOCKING();

// Start RX with a specific timeout in RTC steps (15.625µs per step).
// Saves modulation params for later airtime calc. isLR = true for implicit-header mode.
void bsRadioStartRxTimeout(uint32_t timeoutRtcSteps, const sx126x_mod_params_lora_t& modParams,
                            const sx126x_pkt_params_lora_t& pktParams, bool isLR);

// Start TX. Returns true if TX started.
bool bsRadioStartTx(const uint8_t* pkt, size_t len);

// Put radio in standby.
void bsRadioStandby();

// Trigger a passive scan (e.g. from CMD_SCAN). Saves current anchor as backup.
void bsTriggerScan();

// Main radio update. Call every loop iteration.
void bsHandleRadio();

// Build CMD_PING packet. Updates nonce and writes to NVS.
// Returns packet length (always 17).
size_t bsBuildPingCmdPacket(uint8_t *buf);

// Callback invoked from bsHandleRadio() when a good LoRa packet is received.
// Implemented in main.cpp.
void bsOnPacketReceived(const uint8_t* buf, size_t len, float snrF, float rssiF,
                        uint32_t slotIndex, uint8_t seqIdx, uint8_t win,
                        uint32_t timeOnAirMs, float driftEmaUs);

#endif // BS_RADIO_H
