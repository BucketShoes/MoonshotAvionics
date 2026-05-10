// base_station/radio.cpp — RadioLib-based LoRa link for base station. See radio.h.

#include <RadioLib.h>
#include <mbedtls/md.h>
#include <Preferences.h>
#include "radio.h"
#include "secrets.h"

// ===================== LED HELPERS =====================

static inline void ledOnTX() { ledcWrite(LED_PIN, 64); }
static inline void ledOnRX() { ledcWrite(LED_PIN, 5); }
static inline void ledOff()  { ledcWrite(LED_PIN, 0); }

// ===================== HARDWARE / GLOBALS =====================

SPIClass     bsLoraSPI(FSPI);
bool         bsLoraReady = false;
BsRadioState bsRadioState = BS_RADIO_OFF;

uint8_t  activeChannel = DEFAULT_CHANNEL;
uint8_t  activeSF      = DEFAULT_SF;
int8_t   activePower   = DEFAULT_POWER;
float    activeFreqMHz = 0.0f;
float    activeBwKHz   = 0.0f;

uint8_t  bhChannel = DEFAULT_BH_CHANNEL;
uint8_t  bhSF      = DEFAULT_BH_SF;
int8_t   bhPower   = DEFAULT_BH_POWER;

uint32_t bsTxCount     = 0;
uint32_t bsRxCount     = 0;
uint32_t bsTxFailCount = 0;
uint32_t bsRxFailCount = 0;
unsigned long bsLastTelemRxMs = 0;
unsigned long bsLastCmdSentMs = 0;

// ===================== RADIOLIB MODULE =====================

static SX1262 radio = new Module(LORA_NSS_PIN, LORA_DIO1_PIN, LORA_RST_PIN, LORA_BUSY_PIN, bsLoraSPI);

// ===================== DIO1 FLAG =====================

static volatile bool dio1Flag = false;

IRAM_ATTR static void onDio1() { dio1Flag = true; }

// ===================== HELPERS =====================

void bsUpdateActiveFreqBw() {
  activeFreqMHz = channelToFreqMHz(activeChannel);
  activeBwKHz   = channelToBwKHz(activeChannel);
}

// ===================== INIT =====================

bool bsRadioInit() {
  bsUpdateActiveFreqBw();

  int st = radio.begin(activeFreqMHz, activeBwKHz, activeSF,
                       LORA_CR, LORA_SYNCWORD, activePower, LORA_PREAMBLE);
  if (st != RADIOLIB_ERR_NONE) {
    Serial.print("bs radio.begin failed: "); Serial.println(st);
    bsLoraReady = false;
    bsRadioState = BS_RADIO_OFF;
    return false;
  }

  radio.setDio1Action(onDio1);
  st = radio.startReceive();
  if (st != RADIOLIB_ERR_NONE) {
    Serial.print("bs radio.startReceive failed: "); Serial.println(st);
    bsRadioState = BS_RADIO_OFF;
    return false;
  }

  bsLoraReady  = true;
  bsRadioState = BS_RADIO_RX;
  ledOnRX();
  return true;
}

// ===================== APPLY CONFIG =====================

void bsRadioApplyConfig() {
  if (!bsLoraReady) return;
  bsUpdateActiveFreqBw();

  radio.standby();
  radio.setFrequency(activeFreqMHz);
  radio.setBandwidth(activeBwKHz);
  radio.setSpreadingFactor(activeSF);
  radio.setOutputPower(activePower);
  radio.setCodingRate(LORA_CR);
  radio.setSyncWord(LORA_SYNCWORD);

  dio1Flag = false;
  int st = radio.startReceive();
  if (st == RADIOLIB_ERR_NONE) { bsRadioState = BS_RADIO_RX; ledOnRX(); }
  else                         { bsRadioState = BS_RADIO_OFF; ledOff(); }
}

// ===================== RX-BUSY CHECK =====================

bool bsRadioRxBusy() {
  if (!bsLoraReady) return false;
  if (bsRadioState != BS_RADIO_RX) return false;
  uint32_t irq = radio.getIrqFlags();
  return (irq & (RADIOLIB_SX126X_IRQ_PREAMBLE_DETECTED |
                 RADIOLIB_SX126X_IRQ_SYNC_WORD_VALID |
                 RADIOLIB_SX126X_IRQ_HEADER_VALID)) != 0;
}

// ===================== TX =====================

static unsigned long bsTxStartedMs = 0;
#define BS_TX_WATCHDOG_MS  1500   // longer than any sane airtime; recovers wedged TX

bool bsRadioStartTx(const uint8_t* pkt, size_t len, bool forceThroughBusy) {
  if (!bsLoraReady) return false;
  if (bsRadioState == BS_RADIO_TX) return false;

  // Atomic with the standby below: if a packet is currently arriving (latched
  // PREAMBLE/SYNC/HEADER IRQ flag), do NOT standby — that would abort the RX.
  // Skipped when forceThroughBusy is set (caller has decided to give up).
  if (!forceThroughBusy && bsRadioRxBusy()) return false;

  radio.standby();
  dio1Flag = false;
  int st = radio.startTransmit((uint8_t*)pkt, len);
  if (st != RADIOLIB_ERR_NONE) {
    bsTxFailCount++;
    radio.startReceive();
    bsRadioState = BS_RADIO_RX;
    ledOnRX();
    return false;
  }
  bsRadioState = BS_RADIO_TX;
  bsTxStartedMs = millis();
  ledOnTX();
  bsLastCmdSentMs = millis();
  return true;
}

// ===================== POLL =====================

void bsRadioPoll() {
  if (!bsLoraReady) return;

  // TX watchdog — recover from missed TxDone IRQ (wedged radio).
  if (bsRadioState == BS_RADIO_TX && (millis() - bsTxStartedMs) > BS_TX_WATCHDOG_MS) {
    Serial.println("bs radio: TX watchdog fired — forcing standby+RX");
    bsTxFailCount++;
    radio.standby();
    dio1Flag = false;
    int st = radio.startReceive();
    if (st == RADIOLIB_ERR_NONE) { bsRadioState = BS_RADIO_RX; ledOnRX(); }
    else                         { bsRadioState = BS_RADIO_OFF; ledOff(); }
    return;
  }

  if (!dio1Flag) return;
  dio1Flag = false;

  if (bsRadioState == BS_RADIO_TX) {
    radio.finishTransmit();
    bsTxCount++;
    int st = radio.startReceive();
    if (st == RADIOLIB_ERR_NONE) { bsRadioState = BS_RADIO_RX; ledOnRX(); }
    else                         { bsRadioState = BS_RADIO_OFF; ledOff(); }
    return;
  }

  // RX path
  size_t len = radio.getPacketLength();
  uint8_t buf[256];
  if (len == 0 || len > sizeof(buf)) {
    bsRxFailCount++;
    radio.startReceive();
    bsRadioState = BS_RADIO_RX;
    ledOnRX();
    return;
  }
  int st = radio.readData(buf, len);
  float rssi = radio.getRSSI();
  float snr  = radio.getSNR();

  radio.startReceive();
  bsRadioState = BS_RADIO_RX;
  ledOnRX();

  if (st == RADIOLIB_ERR_NONE) {
    bsRxCount++;
    if (len >= 1 && buf[0] == PKT_TELEMETRY) bsLastTelemRxMs = millis();
    bsOnPacketReceived(buf, len, snr, rssi);
  } else {
    bsRxFailCount++;
  }
}

// ===================== PING BUILDER =====================
// Defined in main.cpp (uses Preferences `bsNvs` and `highestNonce`).
