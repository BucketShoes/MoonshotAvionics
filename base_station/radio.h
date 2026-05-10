// base_station/radio.h — RadioLib-based LoRa link for the base station.
// Continuous RX with DIO1 IRQ; non-blocking TX when a command is queued.

#ifndef BS_RADIO_H
#define BS_RADIO_H

#include <Arduino.h>
#include <SPI.h>
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

// ===================== BASE-LOCAL =====================

#define FAVORITE_ROCKET_DEVICE_ID  ROCKET_DEVICE_ID

// ===================== RADIO STATE =====================

enum BsRadioState { BS_RADIO_OFF, BS_RADIO_RX, BS_RADIO_TX };

extern SPIClass     bsLoraSPI;
extern BsRadioState bsRadioState;
extern bool         bsLoraReady;

// ===================== ACTIVE RADIO CONFIG =====================

extern uint8_t activeChannel;
extern uint8_t activeSF;
extern int8_t  activePower;
extern float   activeFreqMHz;
extern float   activeBwKHz;

extern uint8_t bhChannel;
extern uint8_t bhSF;
extern int8_t  bhPower;

// ===================== STATS =====================

extern uint32_t bsTxCount;
extern uint32_t bsRxCount;
extern uint32_t bsTxFailCount;
extern uint32_t bsRxFailCount;
extern unsigned long bsLastTelemRxMs;
extern unsigned long bsLastCmdSentMs;

// ===================== PUBLIC API =====================

void bsUpdateActiveFreqBw();
bool bsRadioInit();
void bsRadioApplyConfig();

// Try to TX a packet. Returns true on success; state becomes BS_RADIO_TX
// and bsRadioPoll() flips back to BS_RADIO_RX on TxDone IRQ.
bool bsRadioStartTx(const uint8_t* pkt, size_t len);

// Pump the radio: handle RxDone / TxDone IRQs, restart RX. Call every loop.
void bsRadioPoll();

// Build a CMD_PING packet (HMAC + nonce). Returns 17.
size_t bsBuildPingCmdPacket(uint8_t* buf);

// Implemented by base_station/main.cpp — called by bsRadioPoll() on each
// successful RxDone with the bytes + signal info.
void bsOnPacketReceived(const uint8_t* buf, size_t len, float snrF, float rssiF);

#endif // BS_RADIO_H
