// base_station/radio.h — LoRa radio, slot clock, and passive sync for base station.
// Anything that must match the rocket lives in ../common/radio_config.h.
// Inline helpers (channel table, hop derivation) live in ../common/radio_helpers.h.

#ifndef BS_RADIO_H
#define BS_RADIO_H

#include <Arduino.h>
#include <SPI.h>
#include <esp_timer.h>     // esp_timer_get_time() — int64 µs since boot
#include "radio_hal.h"
#include "sx126x.h"
#include "../common/radio_config.h"
#include "../common/radio_helpers.h"

// ===================== PIN DEFINITIONS =====================

#define LORA_NSS_PIN  8
#define LORA_SCK_PIN  9
#define LORA_MOSI_PIN 10
#define LORA_MISO_PIN 11
#define LORA_RST_PIN  12
#define LORA_BUSY_PIN 13
#define LORA_DIO1_PIN 14

#define LED_PIN       18

// LED_MODE: 0 = logic-driven; 1 = mirrors BUSY pin directly.
#define LED_MODE_LOGIC 0
#define LED_MODE_BUSY  1
#define LED_MODE       LED_MODE_LOGIC

// ===================== BASE-LOCAL =====================

#define FAVORITE_ROCKET_DEVICE_ID  ROCKET_DEVICE_ID

// ===================== DEBUG LOG FLAGS =====================
// Local debug switches — may differ between rocket and base.

#define LOG_RX_DONE    true
#define LOG_RX_TIMEOUT true
#define LOG_RX_START   true
#define LOG_TX_START   true
#define LOG_TX_DONE    true

// ===================== RADIO STATE =====================

enum BsRadioState { BS_RADIO_STANDBY, BS_RADIO_RX_ACTIVE, BS_RADIO_TX_ACTIVE };

extern SPIClass             bsLoraSPI;
extern sx126x_hal_context_t bsRadioCtx;
extern BsRadioState         bsRadioState;
extern bool                 bsLoraReady;

// ===================== ACTIVE RADIO CONFIG =====================

extern uint8_t activeChannel;
extern uint8_t activeSF;
extern int8_t  activePower;
extern float   activeFreqMHz;
extern float   activeBwKHz;

extern uint8_t bhChannel;
extern uint8_t bhSF;
extern int8_t  bhPower;

// ===================== PASSIVE SYNC STATE MACHINE =====================
// SCAN_SEARCHING — continuous RX on hopSeq[2] until first valid 0xAF
// SCAN_CANDIDATE — following candidate anchor, 30s to confirm
// SCAN_LOCKED    — normal slotted operation
//
// "In sync" is a live condition (bsInSync()), not a stored boolean.

enum BsScanState {
  SCAN_SEARCHING,
  SCAN_CANDIDATE,
  SCAN_LOCKED,
};

extern BsScanState bsScanState;

extern int64_t bsSyncAnchorUs;
extern int64_t bsSyncSeedSlotIndex;

extern int64_t bsCandidateAnchorUs;
extern int64_t bsCandidateSeedSlotIndex;
extern float   bsCandidateDriftEmaUs;

extern int64_t bsBackupAnchorUs;
extern int64_t bsBackupSeedSlotIndex;
extern float   bsBackupDriftEmaUs;

extern float bsDriftEmaUs;

extern unsigned long bsScanStartMs;
extern unsigned long bsCandidateStartMs;

extern int64_t  bsLastGoodTelemUs;
extern uint32_t bsLastTelemErrorUs;

inline bool bsInSync() {
  return (bsLastGoodTelemUs != 0) &&
         ((esp_timer_get_time() - bsLastGoodTelemUs) < (int64_t)BS_IN_SYNC_TIMEOUT_US) &&
         (bsLastTelemErrorUs < BS_IN_SYNC_TIMING_US);
}

inline int64_t bsGetSlotIndex() {
  return ((esp_timer_get_time() - bsSyncAnchorUs) / (int64_t)SLOT_DURATION_US) + bsSyncSeedSlotIndex;
}

// ===================== OTHER STATE =====================

extern float bsBgRssiEma;

extern unsigned long bsLastTelemRxMs;
extern unsigned long bsLastCmdSentMs;

// Set by bsHandleRadio() when WIN_CMD slot is ready for TX dispatch.
extern bool bsWinCmdReady;

// Modulation params saved at the start of each RX/TX. Used for airtime calc at RxDone.
extern sx126x_mod_params_lora_t bsSavedModParams;
extern sx126x_pkt_params_lora_t bsSavedPktParams;
extern bool                      bsSavedIsLR;

// ===================== PUBLIC API =====================

void bsUpdateActiveFreqBw();
bool bsRadioInit();
void bsRadioApplyConfig_BLOCKING();

void bsRadioStartRxTimeout(uint32_t timeoutRtcSteps, const sx126x_mod_params_lora_t& modParams,
                            const sx126x_pkt_params_lora_t& pktParams, bool isLR);
bool bsRadioStartTx(const uint8_t* pkt, size_t len);

void bsRadioStandby();
void bsTriggerScan();
void bsHandleRadio();

size_t bsBuildPingCmdPacket(uint8_t *buf);

void bsOnPacketReceived(const uint8_t* buf, size_t len, float snrF, float rssiF,
                        int64_t slotIndex, uint8_t seqIdx, uint8_t win,
                        uint32_t timeOnAirMs, float driftEmaUs);

#endif // BS_RADIO_H
