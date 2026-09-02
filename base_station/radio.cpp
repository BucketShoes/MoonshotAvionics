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

// ===================== FEM CONTROL (boards with an external front-end) =====================
// Supply (VFEM_Ctrl) comes up first, then the FEM logic pins — the datasheet
// requires supply before control. After that the enable line is held HIGH
// permanently and only LORA_FEM_TX_PIN moves: HIGH during TX, LOW during RX.
// Which physical FEM line that is differs per revision (CPS on V4.1, CTX on
// V4.3) — board_config.h resolves it. The other line sits on the SX1262's
// DIO2 and is driven by the radio itself via setDio2AsRfSwitch().
#ifdef BOARD_HAS_FEM
static inline void femInitPins() {
  pinMode(LORA_FEM_CTL_PIN, OUTPUT); digitalWrite(LORA_FEM_CTL_PIN, HIGH);
  delayMicroseconds(LORA_FEM_SUPPLY_SETTLE_US);
  pinMode(LORA_FEM_EN_PIN,  OUTPUT); digitalWrite(LORA_FEM_EN_PIN,  HIGH);
  pinMode(LORA_FEM_TX_PIN,  OUTPUT); digitalWrite(LORA_FEM_TX_PIN,  LOW);
}
static inline void femTX() { digitalWrite(LORA_FEM_TX_PIN, HIGH); }
static inline void femRX() { digitalWrite(LORA_FEM_TX_PIN, LOW);  }
#else
static inline void femInitPins() {}
static inline void femTX()      {}
static inline void femRX()      {}
#endif

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

static volatile bool    dio1Flag = false;
static volatile int64_t dio1Time = 0;   // esp_timer_get_time() at IRQ; ISR-safe.

IRAM_ATTR static void onDio1() { dio1Time = esp_timer_get_time(); dio1Flag = true; }

// ===================== HELPERS =====================

void bsUpdateActiveFreqBw() {
  activeFreqMHz = channelToFreqMHz(activeChannel);
  activeBwKHz   = channelToBwKHz(activeChannel);
}

// ===================== INIT =====================

bool bsRadioInit() {
  femInitPins();
  bsUpdateActiveFreqBw();

  int st = radio.begin(activeFreqMHz, activeBwKHz, activeSF,
                       LORA_CR, LORA_SYNCWORD, activePower, LORA_PREAMBLE);
  if (st != RADIOLIB_ERR_NONE) {
    Serial.print("bs radio.begin failed: "); Serial.println(st);
    bsLoraReady = false;
    bsRadioState = BS_RADIO_OFF;
    return false;
  }

  radio.setRxBoostedGainMode(true);
  radio.setDio1Action(onDio1);
  st = radio.startReceive();
  if (st != RADIOLIB_ERR_NONE) {
    Serial.print("bs radio.startReceive failed: "); Serial.println(st);
    bsRadioState = BS_RADIO_OFF;
    return false;
  }

  bsLoraReady  = true;
  bsRadioState = BS_RADIO_RX;
  femRX();
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
  if (st == RADIOLIB_ERR_NONE) { femRX(); bsRadioState = BS_RADIO_RX; ledOnRX(); }
  else                         { bsRadioState = BS_RADIO_OFF; ledOff(); }
}

// ===================== LR LISTEN MODE =====================
// Switches the radio to LR (SF12, implicit-header) RX for a fixed window,
// then auto-reverts. Implemented as a state flag + deadline checked in poll.
// While in LR listen, normal telem RX is paused.

static bool          bsLRListening      = false;
static unsigned long bsLRListenUntilMs  = 0;

bool bsRadioInLRListen() { return bsLRListening; }

static void bsApplyLRMode() {
  radio.standby();
  radio.setSpreadingFactor(LORA_LR_SF);
  radio.implicitHeader(LORA_LR_IMPLICIT_LEN);
  dio1Flag = false;
  int st = radio.startReceive();
  if (st == RADIOLIB_ERR_NONE) { femRX(); bsRadioState = BS_RADIO_RX; ledOnRX(); }
  else                         { bsRadioState = BS_RADIO_OFF; ledOff(); }
}

static void bsRevertFromLRMode() {
  radio.standby();
  radio.setSpreadingFactor(activeSF);
  radio.explicitHeader();
  dio1Flag = false;
  int st = radio.startReceive();
  if (st == RADIOLIB_ERR_NONE) { femRX(); bsRadioState = BS_RADIO_RX; ledOnRX(); }
  else                         { bsRadioState = BS_RADIO_OFF; ledOff(); }
}

void bsRadioEnterLRListen(uint32_t durationMs) {
  if (!bsLoraReady) return;
  if (durationMs == 0) {
    if (bsLRListening) { bsLRListening = false; bsRevertFromLRMode(); }
    return;
  }
  bsLRListening     = true;
  bsLRListenUntilMs = millis() + durationMs;
  bsApplyLRMode();
  Serial.print("bs radio: LR listen for "); Serial.print(durationMs); Serial.println(" ms");
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
  femTX();
  int st = radio.startTransmit((uint8_t*)pkt, len);
  if (st != RADIOLIB_ERR_NONE) {
    bsTxFailCount++;
    femRX();
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

  // LR-listen window expiry: revert to normal modulation when the deadline
  // passes. Packet receive in LR mode happens via the normal RX path below
  // (with implicitHeader having been set up by bsApplyLRMode).
  if (bsLRListening && (long)(millis() - bsLRListenUntilMs) >= 0) {
    bsLRListening = false;
    Serial.println("bs radio: LR listen window expired, reverting");
    bsRevertFromLRMode();
    return;
  }

  // TX watchdog — recover from missed TxDone IRQ (wedged radio).
  if (bsRadioState == BS_RADIO_TX && (millis() - bsTxStartedMs) > BS_TX_WATCHDOG_MS) {
    Serial.println("bs radio: TX watchdog fired — forcing standby+RX");
    bsTxFailCount++;
    radio.standby();
    dio1Flag = false;
    femRX();
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
    femRX();
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
    femRX();
    radio.startReceive();
    bsRadioState = BS_RADIO_RX;
    ledOnRX();
    return;
  }
  int st = radio.readData(buf, len);
  float rssi = radio.getRSSI();
  float snr  = radio.getSNR();

  femRX();
  radio.startReceive();
  bsRadioState = BS_RADIO_RX;
  ledOnRX();

  if (st == RADIOLIB_ERR_NONE) {
    bsRxCount++;
    bsLastTelemRxMs = millis();   // any rocket packet (telem or LR) counts as "rocket heard"
    if (bsLRListening) {
      // LR packet: 3 bytes on air with implicit header. Prepend type+devid
      // so the rest of the pipeline sees a 5-byte normal packet.
      uint8_t lrFrame[2 + LORA_LR_IMPLICIT_LEN];
      lrFrame[0] = PKT_LONGRANGE; //only lr packets go over this framing, so assume its an long-range opacket payload
      lrFrame[1] = FAVORITE_ROCKET_DEVICE_ID;//assume its from the main rocket.  we might have proper tracking later
      memcpy(lrFrame + 2, buf, (len < LORA_LR_IMPLICIT_LEN) ? len : LORA_LR_IMPLICIT_LEN);
      bsOnPacketReceived(lrFrame, 2 + LORA_LR_IMPLICIT_LEN, snr, rssi);
    } else {
      bsOnPacketReceived(buf, len, snr, rssi);
    }
  } else {
    bsRxFailCount++;
  }
}

// ===================== PING BUILDER =====================
// Defined in main.cpp (uses Preferences `bsNvs` and `highestNonce`).
