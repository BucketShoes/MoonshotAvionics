// radio.cpp — RadioLib-based LoRa link for the rocket. See radio.h.

#include <RadioLib.h>
#include "radio.h"
#include "commands.h"
#include "globals.h"
#include "../common/radio_helpers.h"

// LED brightness levels, also used by main.cpp via radioLedOnRX/TX/Off helpers.
static inline void ledOnTX() { ledcWrite(LED_PIN, 64); }
static inline void ledOnRX() { ledcWrite(LED_PIN, 5); }
static inline void ledOff()  { ledcWrite(LED_PIN, 0); }

// ===================== HARDWARE / GLOBALS =====================

SPIClass    loraSPI(FSPI);
bool        loraReady = false;
RadioState  radioState = RADIO_OFF;

uint8_t  activeChannel = DEFAULT_CHANNEL;
uint8_t  activeSF      = DEFAULT_SF;
int8_t   activePower   = DEFAULT_POWER;
float    activeFreqMHz = 0.0f;
float    activeBwKHz   = 0.0f;

uint32_t txCount        = 0;
uint32_t rxCount        = 0;
uint32_t txFailCount    = 0;
uint32_t rxFailCount    = 0;
uint16_t delayedTxCount = 0;
uint16_t invalidRxCount = 0;
bool     lrBeaconEnabled = true;
int64_t  lastValidCmdUs = 0;

// ===================== RADIOLIB MODULE =====================
// SX1262(NSS, DIO1, RST, BUSY, SPI). RadioLib drives BUSY internally.

static SX1262 radio = new Module(LORA_NSS_PIN, LORA_DIO1_PIN, LORA_RST_PIN, LORA_BUSY_PIN, loraSPI);

// ===================== DIO1 FLAG =====================

static volatile bool    dio1Flag = false;
static volatile int64_t dio1Time = 0;   // esp_timer_get_time() at IRQ; ISR-safe.

IRAM_ATTR static void onDio1() { dio1Time = esp_timer_get_time(); dio1Flag = true; }

// ===================== HELPERS =====================

void updateActiveFreqBw() {
  activeFreqMHz = channelToFreqMHz(activeChannel);
  activeBwKHz   = channelToBwKHz(activeChannel);
}

// ===================== INIT =====================

bool radioInit() {
  updateActiveFreqBw();

  int st = radio.begin(activeFreqMHz, activeBwKHz, activeSF,
                       LORA_CR, LORA_SYNCWORD, activePower, LORA_PREAMBLE);
  if (st != RADIOLIB_ERR_NONE) {
    Serial.print("radio.begin failed: "); Serial.println(st);
    loraReady = false;
    radioState = RADIO_OFF;
    return false;
  }

  radio.setDio1Action(onDio1);
  st = radio.startReceive();
  if (st != RADIOLIB_ERR_NONE) {
    Serial.print("radio.startReceive failed: "); Serial.println(st);
    radioState = RADIO_OFF;
    return false;
  }

  loraReady  = true;
  radioState = RADIO_RX;
  ledOnRX();
  return true;
}

// ===================== APPLY CONFIG =====================

void radioApplyConfig() {
  if (!loraReady) return;
  updateActiveFreqBw();

  radio.standby();
  radio.setFrequency(activeFreqMHz);
  radio.setBandwidth(activeBwKHz);
  radio.setSpreadingFactor(activeSF);
  radio.setOutputPower(activePower);
  radio.setCodingRate(LORA_CR);
  radio.setSyncWord(LORA_SYNCWORD);

  dio1Flag = false;
  int st = radio.startReceive();
  if (st == RADIOLIB_ERR_NONE) { radioState = RADIO_RX; ledOnRX(); }
  else                         { radioState = RADIO_OFF; ledOff(); }
}

// ===================== RX-BUSY CHECK =====================
// Returns true if the SX1262 currently has preamble or header sync on an
// incoming packet. Used by the TX scheduler to defer so we don't clobber
// an in-flight reception with our own transmit.

bool radioRxBusy() {
  if (!loraReady) return false;
  if (radioState != RADIO_RX) return false;
  uint32_t irq = radio.getIrqFlags();
  return (irq & (RADIOLIB_SX126X_IRQ_PREAMBLE_DETECTED |
                 RADIOLIB_SX126X_IRQ_SYNC_WORD_VALID |
                 RADIOLIB_SX126X_IRQ_HEADER_VALID)) != 0;
}

// ===================== TX =====================

static unsigned long txStartedMs = 0;
// Watchdog: longest possible airtime is LR (SF12 implicit 3 bytes ≈ 660ms);
// give 2x headroom. Normal-mod TX is much faster but we use one bound for both.
#define RADIO_TX_WATCHDOG_MS  1500

bool radioStartTransmit(const uint8_t* pkt, size_t len, bool forceThroughBusy) {
  if (!loraReady) return false;
  if (radioState == RADIO_TX) return false;

  // Re-check immediately before issuing standby — the SX1262 latches PREAMBLE/
  // SYNC/HEADER IRQ flags as soon as it sees them, even though they aren't
  // mapped to DIO1. If any are set, a packet is currently arriving and we must
  // not standby (which would abort the in-flight RX and produce nothing).
  // Skipped when forceThroughBusy is set (telem overdue, see scheduler).
  if (!forceThroughBusy && radioRxBusy()) return false;

  radio.standby();
  dio1Flag = false;
  int st = radio.startTransmit((uint8_t*)pkt, len);
  if (st != RADIOLIB_ERR_NONE) {
    txFailCount++;
    radio.startReceive();
    radioState = RADIO_RX;
    ledOnRX();
    return false;
  }
  radioState = RADIO_TX;
  txStartedMs = millis();
  ledOnTX();
  return true;
}

// ===================== LR (LONG-RANGE) BEACON =====================
// Non-blocking SF12 implicit-header transmit. State machine:
//   start: standby → switch to LR mod → startTransmit → state=RADIO_TX_LR
//   poll on TxDone: switch back to normal mod → startReceive → state=RADIO_RX
// LR airtime (~660ms) is way over the loop budget, so it MUST be async.
// Caller (scheduler) gates frequency and disable.

bool radioStartLRTransmit(const uint8_t* payload3) {
  if (!loraReady) return false;
  if (radioState != RADIO_RX) return false;     // mid-TX or off
  if (radioRxBusy()) return false;              // packet currently arriving

  radio.standby();
  radio.setSpreadingFactor(LORA_LR_SF);
  radio.implicitHeader(LORA_LR_IMPLICIT_LEN);

  dio1Flag = false;
  int st = radio.startTransmit((uint8_t*)payload3, LORA_LR_IMPLICIT_LEN);
  if (st != RADIOLIB_ERR_NONE) {
    txFailCount++;
    // Restore normal mod and RX before returning.
    radio.setSpreadingFactor(activeSF);
    radio.explicitHeader();
    radio.startReceive();
    radioState = RADIO_RX;
    ledOnRX();
    return false;
  }
  radioState  = RADIO_TX_LR;
  txStartedMs = millis();
  ledOnTX();
  return true;
}

// ===================== POLL =====================

void radioPoll() {
  if (!loraReady) return;

  // TX watchdog — recover from missed TxDone IRQ (wedged radio). Covers both
  // normal-mod TX and LR TX. On LR-mode wedge we also restore normal mod.
  if ((radioState == RADIO_TX || radioState == RADIO_TX_LR) &&
      (millis() - txStartedMs) > RADIO_TX_WATCHDOG_MS) {
    Serial.println("radio: TX watchdog fired — forcing standby+RX");
    txFailCount++;
    radio.standby();
    if (radioState == RADIO_TX_LR) {
      radio.setSpreadingFactor(activeSF);
      radio.explicitHeader();
    }
    dio1Flag = false;
    int st = radio.startReceive();
    if (st == RADIOLIB_ERR_NONE) { radioState = RADIO_RX; ledOnRX(); }
    else                         { radioState = RADIO_OFF; ledOff(); }
    return;
  }

  if (!dio1Flag) return;
  dio1Flag = false;

  if (radioState == RADIO_TX || radioState == RADIO_TX_LR) {
    bool wasLR = (radioState == RADIO_TX_LR);
    radio.finishTransmit();
    txCount++;
    if (wasLR) {
      radio.setSpreadingFactor(activeSF);
      radio.explicitHeader();
    }
    int st = radio.startReceive();
    if (st == RADIOLIB_ERR_NONE) { radioState = RADIO_RX; ledOnRX(); }
    else                         { radioState = RADIO_OFF; ledOff(); }
    return;
  }

  // RX path
  size_t len = radio.getPacketLength();
  uint8_t buf[256];
  if (len == 0 || len > sizeof(buf)) {
    rxFailCount++;
    radio.startReceive();
    radioState = RADIO_RX;
    ledOnRX();
    return;
  }
  int st = radio.readData(buf, len);
  float rssi = radio.getRSSI();
  float snr  = radio.getSNR();

  // Re-arm RX before processing so we don't miss the next packet.
  radio.startReceive();
  radioState = RADIO_RX;
  ledOnRX();

  if (st == RADIOLIB_ERR_NONE) {
    rxCount++;
    processReceivedPacket(buf, len, rssi, snr);
  } else {
    rxFailCount++;
    if (st == RADIOLIB_ERR_CRC_MISMATCH) invalidRxCount++;   // spec 0x0C
  }
}
