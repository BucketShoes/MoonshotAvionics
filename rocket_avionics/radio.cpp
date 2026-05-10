// radio.cpp — RadioLib-based LoRa link for the rocket. See radio.h.

#include <RadioLib.h>
#include "radio.h"
#include "commands.h"
#include "globals.h"
#include "../common/radio_helpers.h"

// ===================== HARDWARE / GLOBALS =====================

SPIClass    loraSPI(FSPI);
bool        loraReady = false;
RadioState  radioState = RADIO_OFF;

uint8_t  activeChannel = DEFAULT_CHANNEL;
uint8_t  activeSF      = DEFAULT_SF;
int8_t   activePower   = DEFAULT_POWER;
float    activeFreqMHz = 0.0f;
float    activeBwKHz   = 0.0f;

uint32_t txCount      = 0;
uint32_t rxCount      = 0;
uint32_t txFailCount  = 0;
uint32_t rxFailCount  = 0;
int64_t  lastValidCmdUs = 0;

// ===================== RADIOLIB MODULE =====================
// SX1262(NSS, DIO1, RST, BUSY, SPI). RadioLib drives BUSY internally.

static SX1262 radio = new Module(LORA_NSS_PIN, LORA_DIO1_PIN, LORA_RST_PIN, LORA_BUSY_PIN, loraSPI);

// ===================== DIO1 FLAG =====================

static volatile bool dio1Flag = false;

IRAM_ATTR static void onDio1() { dio1Flag = true; }

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
  radioState = (st == RADIOLIB_ERR_NONE) ? RADIO_RX : RADIO_OFF;
}

// ===================== TX =====================

bool radioStartTransmit(const uint8_t* pkt, size_t len) {
  if (!loraReady) return false;
  if (radioState == RADIO_TX) return false;

  radio.standby();
  dio1Flag = false;
  int st = radio.startTransmit((uint8_t*)pkt, len);
  if (st != RADIOLIB_ERR_NONE) {
    txFailCount++;
    radio.startReceive();
    radioState = RADIO_RX;
    return false;
  }
  radioState = RADIO_TX;
  return true;
}

// ===================== POLL =====================

void radioPoll() {
  if (!loraReady) return;
  if (!dio1Flag) return;
  dio1Flag = false;

  if (radioState == RADIO_TX) {
    radio.finishTransmit();
    txCount++;
    int st = radio.startReceive();
    radioState = (st == RADIOLIB_ERR_NONE) ? RADIO_RX : RADIO_OFF;
    return;
  }

  // RX path
  size_t len = radio.getPacketLength();
  uint8_t buf[256];
  if (len == 0 || len > sizeof(buf)) {
    rxFailCount++;
    radio.startReceive();
    radioState = RADIO_RX;
    return;
  }
  int st = radio.readData(buf, len);
  float rssi = radio.getRSSI();
  float snr  = radio.getSNR();

  // Re-arm RX before processing so we don't miss the next packet.
  radio.startReceive();
  radioState = RADIO_RX;

  if (st == RADIOLIB_ERR_NONE) {
    rxCount++;
    int8_t rssi8 = (int8_t)constrain((int)rssi, -128, 127);
    int8_t snr4  = (int8_t)constrain((int)(snr * 4.0f), -128, 127);
    processReceivedPacket(buf, len, rssi8, snr4);
  } else {
    rxFailCount++;
  }
}
