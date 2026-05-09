// radio.cpp — LoRa radio hardware, TX/RX state machine, slot-based scheduling.
// See radio.h for the public API.
//
// TEST MODE: uncomment to replace the slot state machine with bare continuous RX.
//#define ROCKET_RADIO_TEST_MODE

#include <Arduino.h>
#include "radio.h"
#include "sx126x.h"
#include "sx126x_hal.h"
#include "telemetry.h"
#include "commands.h"
#include "globals.h"
#include "gps.h"

// ===================== HARDWARE OBJECTS =====================

SPIClass loraSPI(FSPI);

sx126x_hal_context_t radioCtx = {
  .spi           = &loraSPI,
  .nss           = LORA_NSS_PIN,
  .busy          = LORA_BUSY_PIN,
  .rst           = LORA_RST_PIN,
  .initMode      = true,
  .allowBusyRead = false,
};

// ===================== ACTIVE RADIO CONFIG =====================

uint8_t activeChannel = DEFAULT_CHANNEL;
uint8_t activeSF      = DEFAULT_SF;
int8_t  activePower   = DEFAULT_POWER;
float   activeFreqMHz = 917.5f;
float   activeBwKHz   = 125.0f;

// ===================== HOP SEQUENCE =====================

uint8_t hopSeq[NUM_HOP_CHANNELS];

// ===================== RADIO STATE =====================

bool        loraReady  = false;
RadioState  radioState = RADIO_STANDBY; //Remember this is not controlling the actual state, this is just our rekoning of it, the actual state changes by itself in the sx1262 radio's own logic

// ===================== SLOT CLOCK STATE =====================

int64_t syncAnchorUs      = 0;
int64_t syncSeedSlotIndex = 0;
int64_t lastValidCmdUs    = 0;

// ===================== MODULATION SNAPSHOT =====================

sx126x_mod_params_lora_t savedModParams = {};
sx126x_pkt_params_lora_t savedPktParams = {};
bool                      savedIsLR      = false;

// ===================== STATS =====================

uint16_t delayedTxCount = 0;
uint16_t invalidRxCount = 0;

// ===================== HELPERS =====================

void updateActiveFreqBw() {
  activeFreqMHz = channelToFreqMHz(activeChannel);
  activeBwKHz   = channelToBwKHz(activeChannel);
}

static sx126x_lora_bw_t bwKHzToEnum(float bwKHz) {
  if (bwKHz >= 490.0f) return SX126X_LORA_BW_500;
  if (bwKHz >= 240.0f) return SX126X_LORA_BW_250;
  return SX126X_LORA_BW_125;
}

enum RadioCfg : uint8_t { CFG_NORMAL, CFG_LR };

static sx126x_mod_params_lora_t buildModParams(RadioCfg cfg) {
  if (cfg == CFG_LR)
    return { (sx126x_lora_sf_t)LORA_LR_SF, bwKHzToEnum(activeBwKHz), (sx126x_lora_cr_t)LORA_LR_CR, 1 };
  return { (sx126x_lora_sf_t)activeSF, bwKHzToEnum(activeBwKHz), SX126X_LORA_CR_4_5, 0 };
}

static sx126x_pkt_params_lora_t buildPktParams(RadioCfg cfg, uint8_t pldLen) {
  if (cfg == CFG_LR)
    return { 5, SX126X_LORA_PKT_IMPLICIT, (uint8_t)(pldLen ? pldLen : 3), false, false };
  return { LORA_PREAMBLE, SX126X_LORA_PKT_EXPLICIT, pldLen, true, false };
}

// Track which channel the radio is actually tuned to, so hop-position can be
// derived independently of slotIndex (user requirement: hop and slot are independent).
static uint8_t currentTunedChannel = DEFAULT_CHANNEL;

// Wait for BUSY pin to clear (the chip raises BUSY while it processes a command).
// Returns true if BUSY went low within the timeout. Used between consecutive SPI
// ops so the HAL doesn't drop the next command for "BUSY high."
// Captures the µs spent waiting in *waitedUsOut (may be null) so callers can tell
// "was already low" from "spun for ages and only just dropped".
static bool waitBusyClear(uint32_t timeoutUs, uint32_t* waitedUsOut = nullptr) {
  unsigned long t0 = micros();
  while (digitalRead(LORA_BUSY_PIN)) {
    if ((micros() - t0) >= timeoutUs) {
      if (waitedUsOut) *waitedUsOut = (uint32_t)(micros() - t0);
      return false;
    }
  }
  if (waitedUsOut) *waitedUsOut = (uint32_t)(micros() - t0);
  return true;
}

// Rate-limit identical log lines. Pass a unique slot id (0..N-1, must be a
// distinct constant per call site). Returns true if the caller should print
// AND records the time. Suppressed lines are counted; when printing resumes,
// the count is included in *suppressedOut so it can be appended to the message.
//
// Usage:
//   uint32_t supp;
//   if (logRate(LR_RX_MOD_REJECT, 1000, &supp)) {
//     Serial.print("RX: set_lora_mod_params rejected st=3 (suppressed since="); Serial.print(supp); Serial.println(")");
//   }
static constexpr uint8_t LR_NUM_SLOTS = 16;
static bool logRate(uint8_t slot, uint32_t intervalMs, uint32_t* suppressedOut = nullptr) {
  static uint32_t lastMs[LR_NUM_SLOTS]    = {0};
  static uint32_t supressed[LR_NUM_SLOTS] = {0};
  if (slot >= LR_NUM_SLOTS) return true;  // unknown slot — print
  uint32_t now = millis();
  if (lastMs[slot] == 0 || (now - lastMs[slot]) >= intervalMs) {
    if (suppressedOut) *suppressedOut = supressed[slot];
    supressed[slot] = 0;
    lastMs[slot] = now;
    return true;
  }
  supressed[slot]++;
  return false;
}

// Log-rate slot IDs for this file. Keep contiguous and < LR_NUM_SLOTS.
enum : uint8_t {
  LRSLOT_RX_ATTEMPT    = 0,
  LRSLOT_RX_MOD_FAIL   = 1,
  LRSLOT_RX_PKT_FAIL   = 2,
  LRSLOT_RX_CLR_FAIL   = 3,
  LRSLOT_RX_SETRX_FAIL = 4,
  LRSLOT_TX_ATTEMPT    = 5,
  LRSLOT_TX_MOD_FAIL   = 6,
  LRSLOT_TX_PKT_FAIL   = 7,
  LRSLOT_TX_CLR_FAIL   = 8,
  LRSLOT_TX_WB_FAIL    = 9,
  LRSLOT_TX_SETTX_FAIL = 10,
  LRSLOT_BUSY_PRE_MOD  = 11,
  LRSLOT_FREQ_FAIL     = 12,
};

// Print everything we know about a failed SPI op: caller-side BUSY samples,
// HAL drop counters/opcode, time since last drop. This is the "what just
// happened" diagnostic for status != OK.
static void logSpiFail(const char* tag, sx126x_status_t st,
                       uint32_t busyBefore, uint32_t waitedUs,
                       uint32_t busyAfter,
                       uint32_t hwReadDropsAtStart, uint32_t hwWriteDropsAtStart) {
  Serial.print(tag);
  Serial.print(" st="); Serial.print(st);
  Serial.print(" busyPre="); Serial.print(busyBefore);
  Serial.print(" waitUs="); Serial.print(waitedUs);
  Serial.print(" busyAfter="); Serial.print(busyAfter);
  Serial.print(" busyNow="); Serial.print(digitalRead(LORA_BUSY_PIN));
  Serial.print(" newWriteDrops="); Serial.print(totalBusyWriteDrops - hwWriteDropsAtStart);
  Serial.print(" newReadDrops="); Serial.print(totalBusyReadDrops - hwReadDropsAtStart);
  Serial.print(" lastDropOp=0x"); Serial.print(lastDroppedOpcode, HEX);
  Serial.print(" lastDropAgeUs="); Serial.print((uint32_t)(micros() - lastDroppedAtMicros));
  Serial.println();
}

// Returns true if the frequency was actually applied. The HAL drops SPI commands
// when BUSY is high (returns SX126X_HAL_STATUS_ERROR -> non-OK sx126x_status_t).
// Spin briefly for BUSY to clear, issue, check return, then wait for BUSY again
// because set_rf_freq itself raises BUSY while the chip retunes — the very next
// SPI op (typically set_lora_mod_params) would otherwise be dropped.
static bool applyFrequency(uint8_t ch) {
  if (!waitBusyClear(200)) {
    Serial.print("RK applyFrequency: BUSY stuck pre, freq NOT applied ch="); Serial.println(ch);
    return false;
  }
  float freqMHz = channelToFreqMHz(ch);
  uint32_t freqHz = (uint32_t)(freqMHz * 1e6f + 0.5f);
  sx126x_status_t st = sx126x_set_rf_freq(&radioCtx, freqHz);
  if (st != SX126X_STATUS_OK) {
    Serial.print("RK applyFrequency: set_rf_freq rejected st="); Serial.print(st);
    Serial.print(" ch="); Serial.println(ch);
    return false;
  }
  if (!waitBusyClear(200)) {
    Serial.print("RK applyFrequency: BUSY stuck post-write ch="); Serial.println(ch);
    currentTunedChannel = ch;
    return false;
  }
  currentTunedChannel = ch;
  return true;
}

// SX1262 errata §15.3: after any RX with implicit header + timeout, stop the RTC timer
// to prevent a spurious timeout IRQ in the next RX mode.
static void applyImplicitHeaderErrataFix() {
  uint8_t val = 0;
  sx126x_status_t st;
  st = sx126x_write_register(&radioCtx, 0x0920, &val, 1);
  if (st != SX126X_STATUS_OK) { Serial.print("RK errata: write 0x0920 fail st="); Serial.println(st); return; }
  st = sx126x_read_register(&radioCtx, 0x0944, &val, 1);
  if (st != SX126X_STATUS_OK) { Serial.print("RK errata: read 0x0944 fail st="); Serial.println(st); return; }
  val |= 0x02;
  st = sx126x_write_register(&radioCtx, 0x0944, &val, 1);
  if (st != SX126X_STATUS_OK) { Serial.print("RK errata: write 0x0944 fail st="); Serial.println(st); }
}

static void ledOnTX()  { ledcWrite(LED_PIN, 64); }
static void ledOnRX()  { ledcWrite(LED_PIN, 5); }
static void ledOff()   { ledcWrite(LED_PIN, 0); }

// ===================== INIT =====================

bool radioInit() {
#ifdef ROCKET_RADIO_TEST_MODE
  Serial.println("*** ROCKET RADIO TEST MODE ACTIVE — RX only, no telem TX ***");
#endif
  pinMode(LORA_NSS_PIN,  OUTPUT);
  digitalWrite(LORA_NSS_PIN, HIGH);
  pinMode(LORA_BUSY_PIN, INPUT);

#ifdef LORA_FEM_EN_PIN
  pinMode(LORA_FEM_EN_PIN,  OUTPUT); digitalWrite(LORA_FEM_EN_PIN,  HIGH);
  pinMode(LORA_FEM_CTL_PIN, OUTPUT); digitalWrite(LORA_FEM_CTL_PIN, HIGH);
  pinMode(LORA_FEM_PA_PIN,  OUTPUT); digitalWrite(LORA_FEM_PA_PIN,  LOW);
  Serial.println("LoRa: FEM init");
#endif

  rederiveHopSequence(activeChannel);

  Serial.println("LoRa: resetting...");
  sx126x_hal_reset(&radioCtx);
  if (!DO_NOT_CALL_WHILE_ARMED_radioWaitBusy_WARNING_LONG_BLOCKING(&radioCtx, 100)) {
    Serial.println("LoRa init FAIL: BUSY stuck after reset");
    return false;
  }

  sx126x_status_t st;
  st = sx126x_set_standby(&radioCtx, SX126X_STANDBY_CFG_RC);
  if (st != SX126X_STATUS_OK) return false;

  st = sx126x_set_dio3_as_tcxo_ctrl(&radioCtx, LORA_TCXO_VOLTAGE, 320);
  Serial.print("LoRa: set_dio3_tcxo -> "); Serial.println(st);
  if (!DO_NOT_CALL_WHILE_ARMED_radioWaitBusy_WARNING_LONG_BLOCKING(&radioCtx, 100)) {
    Serial.println("LoRa init FAIL: BUSY stuck after TCXO setup");
    return false;
  }

  st = sx126x_set_reg_mode(&radioCtx, SX126X_REG_MODE_DCDC);
  if (st != SX126X_STATUS_OK) { Serial.print("LoRa init: set_reg_mode st="); Serial.println(st); }
  st = sx126x_set_pkt_type(&radioCtx, SX126X_PKT_TYPE_LORA);
  if (st != SX126X_STATUS_OK) { Serial.print("LoRa init: set_pkt_type st="); Serial.println(st); }
  st = sx126x_set_dio2_as_rf_sw_ctrl(&radioCtx, true);
  if (st != SX126X_STATUS_OK) { Serial.print("LoRa init: set_dio2_as_rf_sw_ctrl st="); Serial.println(st); }

  const uint8_t syncWord[2] = { 0x14, 0x24 };
  st = sx126x_write_register(&radioCtx, 0x0740, syncWord, 2);
  if (st != SX126X_STATUS_OK) { Serial.print("LoRa init: syncWord write st="); Serial.println(st); }

  radioApplyConfig_BLOCKING();

  sx126x_irq_mask_t irqMask = SX126X_IRQ_TX_DONE | SX126X_IRQ_RX_DONE | SX126X_IRQ_TIMEOUT |
                               SX126X_IRQ_CRC_ERROR | SX126X_IRQ_HEADER_ERROR;
  st = sx126x_set_dio_irq_params(&radioCtx, irqMask, irqMask, SX126X_IRQ_NONE, SX126X_IRQ_NONE);
  if (st != SX126X_STATUS_OK) { Serial.print("LoRa init: set_dio_irq_params st="); Serial.println(st); }
  st = sx126x_clear_irq_status(&radioCtx, SX126X_IRQ_ALL);
  if (st != SX126X_STATUS_OK) { Serial.print("LoRa init: clear_irq_status st="); Serial.println(st); }

  st = sx126x_cfg_rx_boosted(&radioCtx, LORA_RX_BOOSTED);
  if (st != SX126X_STATUS_OK) { Serial.print("LoRa init: cfg_rx_boosted st="); Serial.println(st); }

  radioMcpwmInit(LORA_DIO1_PIN);

  radioCtx.initMode = false;
  radioState = RADIO_STANDBY;

  Serial.print("LoRa init OK: ch="); Serial.print(activeChannel);
  Serial.print(" "); Serial.print(activeFreqMHz, 1); Serial.print("MHz SF");
  Serial.print(activeSF); Serial.print(" BW"); Serial.print((int)activeBwKHz);
  Serial.print("kHz pwr="); Serial.print(activePower); Serial.println("dBm");
  Serial.print("RK HAL drops during init: write="); Serial.print(totalBusyWriteDrops);
  Serial.print(" read="); Serial.print(totalBusyReadDrops);
  Serial.print(" busyPin="); Serial.println(digitalRead(LORA_BUSY_PIN));
  return true;
}

void radioApplyConfig_BLOCKING() {
  sx126x_status_t st;
  sx126x_mod_params_lora_t mp = buildModParams(CFG_NORMAL);
  st = sx126x_set_lora_mod_params(&radioCtx, &mp);
  if (st != SX126X_STATUS_OK) { Serial.print("ApplyConfig: set_lora_mod_params st="); Serial.println(st); }
  DO_NOT_CALL_WHILE_ARMED_radioWaitBusy_WARNING_LONG_BLOCKING(&radioCtx);

  sx126x_pkt_params_lora_t pp = buildPktParams(CFG_NORMAL,255);
  st = sx126x_set_lora_pkt_params(&radioCtx, &pp);
  if (st != SX126X_STATUS_OK) { Serial.print("ApplyConfig: set_lora_pkt_params st="); Serial.println(st); }
  DO_NOT_CALL_WHILE_ARMED_radioWaitBusy_WARNING_LONG_BLOCKING(&radioCtx);

  uint32_t freqHz = (uint32_t)(activeFreqMHz * 1e6f + 0.5f);
  st = sx126x_set_rf_freq(&radioCtx, freqHz);
  if (st != SX126X_STATUS_OK) { Serial.print("ApplyConfig: set_rf_freq st="); Serial.println(st); }
  DO_NOT_CALL_WHILE_ARMED_radioWaitBusy_WARNING_LONG_BLOCKING(&radioCtx);

  sx126x_pa_cfg_params_t paCfg = { .pa_duty_cycle = 0x04, .hp_max = 0x07, .device_sel = 0x00, .pa_lut = 0x01 };
  st = sx126x_set_pa_cfg(&radioCtx, &paCfg);
  if (st != SX126X_STATUS_OK) { Serial.print("ApplyConfig: set_pa_cfg st="); Serial.println(st); }
  DO_NOT_CALL_WHILE_ARMED_radioWaitBusy_WARNING_LONG_BLOCKING(&radioCtx);

  st = sx126x_set_tx_params(&radioCtx, activePower, SX126X_RAMP_200_US);
  if (st != SX126X_STATUS_OK) { Serial.print("ApplyConfig: set_tx_params st="); Serial.println(st); }
  DO_NOT_CALL_WHILE_ARMED_radioWaitBusy_WARNING_LONG_BLOCKING(&radioCtx);

  // Re-derive hop sequence in case activeChannel changed.
  rederiveHopSequence(activeChannel);
}

// ===================== WIN_LR PACKET BUILDER =====================

size_t buildLRPacketCore(uint8_t* buf) {
  uint16_t latFrac = 0x7FF;
  uint16_t lonFrac = 0x7FF;
  if (gps.valid) {
    double latF = fmod(fabs(gps.lat), 1.0) * 2000.0;
    double lonF = fmod(fabs(gps.lon), 1.0) * 2000.0;
    latFrac = (uint16_t)latF + ((rand() / (float)RAND_MAX) < (latF - (int)latF) ? 1 : 0);
    lonFrac = (uint16_t)lonF + ((rand() / (float)RAND_MAX) < (lonF - (int)lonF) ? 1 : 0);
    if (latFrac > 1999) latFrac = 1999;
    if (lonFrac > 1999) lonFrac = 1999;
  }
  bool lowBatt = false;
  if (batteryMv <= 3400) {
    lowBatt = true;
  } else if (batteryMv < 3700) {
    lowBatt = ((rand() / (float)RAND_MAX) < (3700.0f - (float)batteryMv) / 300.0f);
  }
  uint32_t word = ((uint32_t)(latFrac & 0x7FF))
                | ((uint32_t)(lonFrac & 0x7FF) << 11)
                | ((uint32_t)lowBatt            << 22);
  buf[0] = (uint8_t)(word);
  buf[1] = (uint8_t)(word >> 8);
  buf[2] = (uint8_t)(word >> 16);
  return 3;
}

// ===================== RX / TX =====================

void radioStartRxTimeout(uint32_t timeoutRtcSteps,
                         const sx126x_mod_params_lora_t& modParams,
                         const sx126x_pkt_params_lora_t& pktParams,
                         bool isLR,
                         int64_t slotIndex, uint8_t seqIdx, WindowMode win, uint8_t ch) {
  uint32_t supp;
  if (LOG_RK_RX_ATTEMPT && logRate(LRSLOT_RX_ATTEMPT, 1000, &supp)) {
    Serial.print("RxAttempt: slot="); Serial.print((long long)slotIndex);
    Serial.print(" seq="); Serial.print(seqIdx);
    Serial.print(" win="); Serial.print(windowModeName(win));
    Serial.print(" ch="); Serial.print(ch);
    Serial.print(" tunedCh="); Serial.print(currentTunedChannel);
    Serial.print(" busy="); Serial.print(digitalRead(LORA_BUSY_PIN));
    Serial.print(" radioState="); Serial.print((int)radioState);
    Serial.print(" supp="); Serial.println(supp);
  }
  if (digitalRead(LORA_BUSY_PIN)) {
    if (LOG_RK_RX_ATTEMPT) Serial.println("RX: BUSY at start — skip");
    return;
  }

  // Capture HAL drop counters at entry — diff later to see how many drops
  // happened during *this* call.
  const uint32_t hwReadDropsAtStart  = totalBusyReadDrops;
  const uint32_t hwWriteDropsAtStart = totalBusyWriteDrops;

  // Apply modulation and packet params unconditionally — checking returns since
  // the HAL silently drops SPI commands when BUSY is high. Spin BUSY before
  // each SPI op since the prior op (set_rf_freq, mod_params, etc.) raises BUSY
  // briefly while the chip processes.
  uint32_t waited = 0;
  uint32_t busyBefore = digitalRead(LORA_BUSY_PIN);
  if (!waitBusyClear(200, &waited)) {
    if (logRate(LRSLOT_BUSY_PRE_MOD, 1000, &supp)) {
      Serial.print("RX: BUSY stuck pre mod_params waitedUs="); Serial.print(waited);
      Serial.print(" newWriteDrops="); Serial.print(totalBusyWriteDrops - hwWriteDropsAtStart);
      Serial.print(" supp="); Serial.println(supp);
    }
    return;
  }
  uint32_t busyAfter = digitalRead(LORA_BUSY_PIN);
  sx126x_status_t stMod = sx126x_set_lora_mod_params(&radioCtx, &modParams);
  if (stMod != SX126X_STATUS_OK) {
    if (logRate(LRSLOT_RX_MOD_FAIL, 1000, &supp)) {
      logSpiFail("RX: set_lora_mod_params FAIL", stMod, busyBefore, waited, busyAfter,
                 hwReadDropsAtStart, hwWriteDropsAtStart);
      Serial.print("    suppressed="); Serial.println(supp);
    }
    return;  // would RX with stale modulation -> garbage demod
  }
  if (!waitBusyClear(200)) {
    if (logRate(LRSLOT_RX_PKT_FAIL, 1000)) Serial.println("RX: BUSY stuck pre pkt_params");
    return;
  }
  sx126x_status_t stPkt = sx126x_set_lora_pkt_params(&radioCtx, &pktParams);
  if (stPkt != SX126X_STATUS_OK) {
    if (logRate(LRSLOT_RX_PKT_FAIL, 1000)) {
      Serial.print("RX: set_lora_pkt_params rejected st="); Serial.println(stPkt);
    }
    return;
  }

  if (!waitBusyClear(200)) {
    if (logRate(LRSLOT_RX_CLR_FAIL, 1000)) Serial.println("RX: BUSY stuck pre clear_irq");
    return;
  }
  sx126x_status_t stClr = sx126x_clear_irq_status(&radioCtx, SX126X_IRQ_ALL);
  if (stClr != SX126X_STATUS_OK) {
    if (logRate(LRSLOT_RX_CLR_FAIL, 1000)) {
      Serial.print("RX: clear_irq_status rejected st="); Serial.println(stClr);
    }
    return;
  }
  dio1Fired = false;

  if (!waitBusyClear(200)) {
    if (logRate(LRSLOT_RX_SETRX_FAIL, 1000)) Serial.println("RX: BUSY stuck pre set_rx");
    return;
  }
  sx126x_status_t st = sx126x_set_rx_with_timeout_in_rtc_step(&radioCtx, timeoutRtcSteps);
  if (st == SX126X_STATUS_OK) {
    radioState      = RADIO_RX_ACTIVE;
    savedModParams  = modParams;
    savedPktParams  = pktParams;
    savedIsLR       = isLR;
    ledOnRX();
    if (LOG_RK_RX_START) {
      Serial.print("RxStart: slot="); Serial.print((long long)slotIndex);
      Serial.print(" seq="); Serial.print(seqIdx);
      Serial.print(" win="); Serial.print(windowModeName(win));
      Serial.print(" ch="); Serial.print(ch);
      Serial.print(" tunedCh="); Serial.print(currentTunedChannel);
      Serial.print(" hop="); Serial.print(hopIndexOf(currentTunedChannel));
      Serial.print(" timeout="); Serial.print((uint32_t)(timeoutRtcSteps * 15.625f));
      Serial.println("us");
    }
  } else {
    // Chip rejected set_rx — most commonly because a previous RX is still
    // in progress. Reception-preserving scheduler: this is expected; do NOT
    // change radioState (the chip is still busy with its previous action).
    if (LOG_RK_RX_ATTEMPT && logRate(LRSLOT_RX_SETRX_FAIL, 1000)) {
      Serial.print("RX: set_rx rejected st="); Serial.print(st);
      Serial.print(" priorState="); Serial.println((int)radioState);
    }
  }
}

bool radioStartTx(const uint8_t* pkt, size_t len,
                  const sx126x_mod_params_lora_t& modParams,
                  const sx126x_pkt_params_lora_t& pktParams,
                  bool isLR,
                  int64_t slotIndex, uint8_t seqIdx, WindowMode win, uint8_t ch) {
  uint32_t supp;
  if (LOG_RK_TX_ATTEMPT && logRate(LRSLOT_TX_ATTEMPT, 1000, &supp)) {
    int64_t now = esp_timer_get_time();
    int64_t slotStart = syncAnchorUs + (slotIndex - syncSeedSlotIndex) * (int64_t)SLOT_DURATION_US;
    Serial.print("TxAttempt: slot="); Serial.print((long long)slotIndex);
    Serial.print(" seq="); Serial.print(seqIdx);
    Serial.print(" win="); Serial.print(windowModeName(win));
    Serial.print(" ch="); Serial.print(ch);
    Serial.print(" tunedCh="); Serial.print(currentTunedChannel);
    Serial.print(" len="); Serial.print((unsigned)len);
    Serial.print(" intoSlotUs="); Serial.print((long long)(now - slotStart));
    Serial.print(" busy="); Serial.print(digitalRead(LORA_BUSY_PIN));
    Serial.print(" radioState="); Serial.print((int)radioState);
    Serial.print(" supp="); Serial.println(supp);
  }
  if (digitalRead(LORA_BUSY_PIN)) {
    if (LOG_RK_TX_ATTEMPT) Serial.println("TX: BUSY — skip");
    return false;
  }

  const uint32_t hwReadDropsAtStart  = totalBusyReadDrops;
  const uint32_t hwWriteDropsAtStart = totalBusyWriteDrops;

  // Apply modulation and packet params unconditionally — checking returns since
  // the HAL silently drops SPI commands when BUSY is high. Spin BUSY before
  // each SPI op since the prior op raises BUSY briefly while the chip processes.
  uint32_t waited = 0;
  uint32_t busyBefore = digitalRead(LORA_BUSY_PIN);
  if (!waitBusyClear(200, &waited)) {
    if (logRate(LRSLOT_BUSY_PRE_MOD, 1000, &supp)) {
      Serial.print("TX: BUSY stuck pre mod_params waitedUs="); Serial.print(waited);
      Serial.print(" newWriteDrops="); Serial.print(totalBusyWriteDrops - hwWriteDropsAtStart);
      Serial.print(" supp="); Serial.println(supp);
    }
    return false;
  }
  uint32_t busyAfter = digitalRead(LORA_BUSY_PIN);
  sx126x_status_t stMod = sx126x_set_lora_mod_params(&radioCtx, &modParams);
  if (stMod != SX126X_STATUS_OK) {
    if (logRate(LRSLOT_TX_MOD_FAIL, 1000, &supp)) {
      logSpiFail("TX: set_lora_mod_params FAIL", stMod, busyBefore, waited, busyAfter,
                 hwReadDropsAtStart, hwWriteDropsAtStart);
      Serial.print("    suppressed="); Serial.println(supp);
    }
    return false;
  }
  if (!waitBusyClear(200)) {
    if (logRate(LRSLOT_TX_PKT_FAIL, 1000)) Serial.println("TX: BUSY stuck pre pkt_params");
    return false;
  }
  // For TX, rebuild pkt params with actual payload length.
  sx126x_pkt_params_lora_t ppTx = pktParams;
  ppTx.pld_len_in_bytes = (uint8_t)len;
  sx126x_status_t stPkt = sx126x_set_lora_pkt_params(&radioCtx, &ppTx);
  if (stPkt != SX126X_STATUS_OK) {
    if (logRate(LRSLOT_TX_PKT_FAIL, 1000)) {
      Serial.print("TX: set_lora_pkt_params rejected st="); Serial.println(stPkt);
    }
    return false;
  }

  if (!waitBusyClear(200)) {
    if (logRate(LRSLOT_TX_CLR_FAIL, 1000)) Serial.println("TX: BUSY stuck pre clear_irq");
    return false;
  }
  sx126x_status_t stClr = sx126x_clear_irq_status(&radioCtx, SX126X_IRQ_ALL);
  if (stClr != SX126X_STATUS_OK) {
    if (logRate(LRSLOT_TX_CLR_FAIL, 1000)) {
      Serial.print("TX: clear_irq_status rejected st="); Serial.println(stClr);
    }
    return false;
  }
  dio1Fired = false;

  if (!waitBusyClear(200)) {
    if (logRate(LRSLOT_TX_WB_FAIL, 1000)) Serial.println("TX: BUSY stuck pre write_buffer");
    return false;
  }
  sx126x_status_t st = sx126x_write_buffer(&radioCtx, 0, pkt, (uint8_t)len);
  if (st != SX126X_STATUS_OK) {
    if (logRate(LRSLOT_TX_WB_FAIL, 1000)) {
      Serial.print("TX: write_buffer fail st="); Serial.println(st);
    }
    return false;
  }

  if (!waitBusyClear(200)) {
    if (logRate(LRSLOT_TX_SETTX_FAIL, 1000)) Serial.println("TX: BUSY stuck pre set_tx — abort");
    return false;
  }

#ifdef LORA_FEM_PA_PIN
  digitalWrite(LORA_FEM_PA_PIN, HIGH);
#endif

  st = sx126x_set_tx(&radioCtx, 0);
  if (st == SX126X_STATUS_OK) {
    radioState     = RADIO_TX_ACTIVE;
    savedModParams = modParams;
    savedPktParams = ppTx;
    savedIsLR      = isLR;
    ledOnTX();
    if (LOG_RK_TX_START) {
      Serial.print("TxStart: slot="); Serial.print((long long)slotIndex);
      Serial.print(" seq="); Serial.print(seqIdx);
      Serial.print(" win="); Serial.print(windowModeName(win));
      Serial.print(" ch="); Serial.print(ch);
      Serial.print(" tunedCh="); Serial.print(currentTunedChannel);
      Serial.print(" hop="); Serial.print(hopIndexOf(currentTunedChannel));
      Serial.print(" len="); Serial.println((unsigned)len);
    }
    return true;
  }
  Serial.print("TX: set_tx fail st="); Serial.println(st);
  radioState = RADIO_STANDBY;
  return false;
}

void radioStandby() {
#ifdef LORA_FEM_PA_PIN
  digitalWrite(LORA_FEM_PA_PIN, LOW);
#endif
  // The HAL drops SPI commands while BUSY is high. If our standby command got
  // dropped and we set radioState=STANDBY anyway, the slot machine will think
  // the radio is idle and re-issue set_rx every loop, all of which the chip
  // rejects because it's still mid-RX. Spin briefly for BUSY, check the return,
  // and only update state on success.
  {
    unsigned long t0 = micros();
    while (digitalRead(LORA_BUSY_PIN) && (micros() - t0) < 200) {}
  }
  if (digitalRead(LORA_BUSY_PIN)) {
    Serial.println("standby: BUSY stuck — standby NOT issued, state unchanged");
    return;
  }
  sx126x_status_t st = sx126x_set_standby(&radioCtx, SX126X_STANDBY_CFG_RC);
  if (st != SX126X_STATUS_OK) {
    Serial.print("standby: set_standby rejected st="); Serial.print(st);
    Serial.println(" — state unchanged");
    return;
  }
  radioState = RADIO_STANDBY;
  ledOff();
}

// ===================== IRQ / RX PACKET HANDLER =====================

static void handleRxDone() {
  sx126x_rx_buffer_status_t bufStatus = {};
  sx126x_status_t st = sx126x_get_rx_buffer_status(&radioCtx, &bufStatus);
  if (st != SX126X_STATUS_OK) {
    Serial.print("RX: get_rx_buffer_status fail st="); Serial.println(st);
    invalidRxCount++;
    return;
  }

  uint8_t rxBuf[255];
  uint8_t rxLen = bufStatus.pld_len_in_bytes;
  if (rxLen == 0 || rxLen > sizeof(rxBuf)) {
    Serial.print("RX: bad length "); Serial.println(rxLen);
    invalidRxCount++;
    return;
  }

  st = sx126x_read_buffer(&radioCtx, bufStatus.buffer_start_pointer, rxBuf, rxLen);
  if (st != SX126X_STATUS_OK) {
    Serial.print("RX: read_buffer fail st="); Serial.println(st);
    invalidRxCount++;
    return;
  }

  sx126x_pkt_status_lora_t pktStatus = {};
  sx126x_get_lora_pkt_status(&radioCtx, &pktStatus);
  int8_t pktRssi = pktStatus.rssi_pkt_in_dbm;
  int8_t pktSnr  = pktStatus.snr_pkt_in_db;

  bool structOk = (rxLen >= 2) && (rxBuf[0] == PKT_TELEMETRY || rxBuf[0] == PKT_COMMAND ||
                                    rxBuf[0] == PKT_BACKHAUL);
  if (!structOk) {
    Serial.print("RX: unknown type 0x"); Serial.println(rxBuf[0], HEX);
    invalidRxCount++;
    return;
  }

  static unsigned long lastRxLogMs = 0;
  unsigned long nowMs = millis();
  if (nowMs - lastRxLogMs >= 1000) {
    lastRxLogMs = nowMs;
    const char* typeStr = (rxBuf[0] == PKT_COMMAND) ? "CMD" :
                          (rxBuf[0] == PKT_TELEMETRY) ? "TELEM" : "BH";
    Serial.print("RX "); Serial.print(typeStr);
    Serial.print(" "); Serial.print(rxLen); Serial.print("B rssi=");
    Serial.print(pktRssi); Serial.print(" snr="); Serial.println(pktSnr);
  }

  processReceivedPacket(rxBuf, rxLen, pktRssi, pktSnr);
}

static void radioHandleIrq() {
  uint64_t eventUs = dio1TimestampUs();
  dio1Fired = false;

  sx126x_irq_mask_t irqFlags = 0;
  sx126x_status_t st = sx126x_get_and_clear_irq_status(&radioCtx, &irqFlags);
  if (st != SX126X_STATUS_OK) {
    Serial.print("IRQ: get_and_clear fail st="); Serial.println(st);
    radioState = RADIO_STANDBY;
    ledOff();
    return;
  }

  int64_t eventSlotIdx = ((int64_t)eventUs - syncAnchorUs) / (int64_t)SLOT_DURATION_US + syncSeedSlotIndex;

  if (irqFlags & SX126X_IRQ_TX_DONE) {
#ifdef LORA_FEM_PA_PIN
    digitalWrite(LORA_FEM_PA_PIN, LOW);
#endif
    radioState = RADIO_STANDBY;
    ledOff();
    if (LOG_RK_TX_DONE) {
      Serial.print("TxDone: slot="); Serial.println((long long)eventSlotIdx);
    }
  }

  if (irqFlags & SX126X_IRQ_RX_DONE) {
    radioState = RADIO_STANDBY;
    if (savedIsLR) applyImplicitHeaderErrataFix();
    if (LOG_RK_RX_DONE) {
      Serial.print("RxDone: slot="); Serial.println((long long)eventSlotIdx);
    }
    handleRxDone();
    ledOff();
  }

  if (irqFlags & SX126X_IRQ_TIMEOUT) {
    radioState = RADIO_STANDBY;
    if (savedIsLR) applyImplicitHeaderErrataFix();
    ledOff();
    if (LOG_RK_RX_TIMEOUT) {
      Serial.print("RxTimeout: slot="); Serial.println((long long)eventSlotIdx);
    }
  }

  if (irqFlags & (SX126X_IRQ_CRC_ERROR | SX126X_IRQ_HEADER_ERROR)) {
    radioState = RADIO_STANDBY;
    ledOff();
    invalidRxCount++;
    if (irqFlags & SX126X_IRQ_CRC_ERROR)    Serial.print("RX: CRC_ERROR ");
    if (irqFlags & SX126X_IRQ_HEADER_ERROR) Serial.print("RX: HEADER_ERROR ");
    Serial.print("slot="); Serial.println((long long)eventSlotIdx);
  }

  if (irqFlags == 0) {
    Serial.print("IRQ: flags=0 radioState="); Serial.print(radioState);
    Serial.print(" BUSY="); Serial.println(digitalRead(LORA_BUSY_PIN));
  }
}

// ===================== MAIN RADIO STATE MACHINE =====================
//
// Slot-based scheduling driven by nextActionUs timestamp.
// WIN_TELEM: TX telemetry on hopped channel at each slot boundary.
// WIN_CMD:   RX for commands on fixed command channel.
// WIN_LR:    TX long-range packet on hopped channel.
// WIN_CONTINUE: no-op, extends previous slot.
// WIN_OFF:   radio standby.

#ifdef ROCKET_RADIO_TEST_MODE
void nonblockingRadio() {
  if (!loraReady) return;
  if (!dio1Fired && digitalRead(LORA_DIO1_PIN) &&
      (radioState == RADIO_TX_ACTIVE || radioState == RADIO_RX_ACTIVE)) {
    dio1CaptureVal = micros();
    dio1Fired = true;
  }
  if (dio1Fired) radioHandleIrq();
  if (radioState == RADIO_STANDBY) {
    sx126x_mod_params_lora_t mp = buildModParams(CFG_NORMAL);
    sx126x_pkt_params_lora_t pp = buildPktParams(CFG_NORMAL,255);
    uint32_t timeoutUs = 100'000UL;
    radioStartRxTimeout((uint32_t)(timeoutUs / 15.625f), mp, pp, false,
                        0, 0, WIN_TELEM, currentTunedChannel);
  }
}
#else

// nextActionUs: int64 esp_timer timestamp at which the next slot action should begin.
// All slot math uses int64 µs so we don't overflow the slotIndex*SLOT_DURATION_US
// product on long uptimes, and so signed-subtraction comparisons are unambiguous.
static int64_t nextActionUs        = 0;
static int64_t nextActionSlotIndex = 0;

// Log whenever nextActionUs is reassigned, so we can see if it ever leaps far ahead.
static inline void setNextAction(int64_t newUs, int64_t newSlot, const char* reason) {
  if (LOG_RK_SETNEXT) {
    int64_t now = esp_timer_get_time();
    Serial.print("RADIO: setNext "); Serial.print(reason);
    Serial.print(" slot="); Serial.print((long long)newSlot);
    Serial.print(" inUs="); Serial.println((long long)(newUs - now));
  }
  nextActionUs        = newUs;
  nextActionSlotIndex = newSlot;
}
// Consecutive unexpected overruns (busy radio at slot boundary, excluding WIN_CONTINUE).
static uint8_t unexpectedOverruns = 0;

// Rate-limit recurring radio diagnostics to keep them from drowning the serial log.
static int64_t lastOverrunLogUs = 0;
static int64_t lastBusyLogUs    = 0;
static uint32_t overrunSinceLog = 0;
static uint32_t busySinceLog    = 0;

// Compute the absolute end-of-slot timestamp for the given slot index.
static inline int64_t slotEndUsFor(int64_t slotIndex) {
  return syncAnchorUs + (slotIndex - syncSeedSlotIndex + 1) * (int64_t)SLOT_DURATION_US;
}

void nonblockingRadio() {
  if (!loraReady) return;

  // Always handle IRQ first if DIO1 fired.
  if (dio1Fired) {
    radioHandleIrq();
  }

  int64_t now = esp_timer_get_time();

  // Compute current slot state.
  int64_t slotIndex = radioGetSlotIndex();
  uint8_t  seqIdx   = (uint8_t)(((uint64_t)(slotIndex - syncSeedSlotIndex)) % SLOT_SEQUENCE_LEN);
  WindowMode win    = SLOT_SEQUENCE[seqIdx];
  uint8_t  ch       = hopChannel((uint32_t)(slotIndex - syncSeedSlotIndex));

  // First call after boot: align nextActionUs with current time so the first slot fires.
  if (nextActionUs == 0) {
    setNextAction(now, slotIndex, "boot");
  }

  // Periodic visibility ping — prints once per second so we can see why the
  // slot machine isn't firing actions even when no overrun/busy logs appear.
  static int64_t lastHeartbeatUs = 0;
  if (LOG_RK_HB && (now - lastHeartbeatUs) > 1'000'000) {
    lastHeartbeatUs = now;
    Serial.print("RADIO: hb slot="); Serial.print((long long)slotIndex);
    Serial.print(" seq="); Serial.print(seqIdx);
    Serial.print(" win="); Serial.print(windowModeName(win));
    Serial.print(" ch="); Serial.print(ch);
    Serial.print(" nextDelta="); Serial.print((long long)(nextActionUs - now));
    Serial.print(" busy="); Serial.print(digitalRead(LORA_BUSY_PIN));
    Serial.print(" radioState="); Serial.print((int)radioState);
    Serial.print(" dio1Fired="); Serial.print((int)dio1Fired);
    Serial.print(" overruns="); Serial.print((int)unexpectedOverruns);
    Serial.print(" txEn="); Serial.print((int)txSendingEnabled);
    Serial.print(" anchor="); Serial.print((long long)syncAnchorUs);
    Serial.print(" seed="); Serial.println((long long)syncSeedSlotIndex);
  }

  // WIN_CONTINUE: no radio action, no overrun counting. Just advance nextActionUs.
  if (win == WIN_CONTINUE) {
    if (slotIndex != nextActionSlotIndex) {
      setNextAction(slotEndUsFor(slotIndex), slotIndex, "CONTINUE");
    }
    return;
  }

  // Not yet time for the next action — int64 signed subtraction.
  if (now < nextActionUs) return;

  // Time has come (or overdue). Check overrun conditions before BUSY.
  bool isOverrun = false;

  if (win == WIN_TELEM || win == WIN_LR) {
    // TX: skip if too far behind the intended start.
    if ((now - nextActionUs) > (int64_t)TX_LATE_THRESHOLD_US) {
      isOverrun = true;
      delayedTxCount++;
    }
  } else if (win == WIN_CMD) {
    // RX: skip if remaining time in slot is too short.
    if ((slotEndUsFor(slotIndex) - now) < (int64_t)BS_RX_MIN_REMAINING_US) {
      isOverrun = true;
    }
  }

  if (isOverrun) {
    unexpectedOverruns++;
    overrunSinceLog++;
    if (LOG_RK_OVERRUN && (now - lastOverrunLogUs) > 1'000'000) {
      Serial.print("RADIO: overrun win="); Serial.print((int)win);
      Serial.print(" consec="); Serial.print(unexpectedOverruns);
      Serial.print(" totalSinceLog="); Serial.print(overrunSinceLog);
      Serial.print(" slot="); Serial.println((long long)slotIndex);
      lastOverrunLogUs = now;
      overrunSinceLog  = 0;
    }
    if (unexpectedOverruns >= 3) {
      if (LOG_RK_OVERRUN) Serial.println("RADIO: 3 overruns — forcing standby");
      radioStandby();
      unexpectedOverruns = 0;
    }
    setNextAction(slotEndUsFor(slotIndex), slotIndex + 1, "overrun");
    return;
  }

  // Check BUSY pin.
  if (digitalRead(LORA_BUSY_PIN)) {
    unexpectedOverruns++;
    busySinceLog++;
    if (LOG_RK_OVERRUN && (now - lastBusyLogUs) > 1'000'000) {
      Serial.print("RADIO: BUSY at boundary consec="); Serial.print(unexpectedOverruns);
      Serial.print(" totalSinceLog="); Serial.print(busySinceLog);
      Serial.print(" slot="); Serial.println((long long)slotIndex);
      lastBusyLogUs = now;
      busySinceLog  = 0;
    }
    if (unexpectedOverruns >= 3) {
      if (LOG_RK_OVERRUN) Serial.println("RADIO: BUSY stuck 3 slots — forcing standby");
      radioStandby();
      unexpectedOverruns = 0;
      setNextAction(slotEndUsFor(slotIndex), slotIndex + 1, "busy3");
    }
    return;
  }

  // BUSY clear and time has come. Take the slot action.
  unexpectedOverruns = 0;
  setNextAction(slotEndUsFor(slotIndex), slotIndex + 1, "fire");

  if (win == WIN_TELEM) {
    if (txSendingEnabled) {
      if (!applyFrequency(ch)) return;  // skip — would TX on wrong channel
      sx126x_mod_params_lora_t mp = buildModParams(CFG_NORMAL);
      uint8_t pkt[255];
      size_t len = buildTelemetryPacket(pkt);
      sx126x_pkt_params_lora_t pp = buildPktParams(CFG_NORMAL,(uint8_t)len);
      radioStartTx(pkt, len, mp, pp, false, slotIndex, seqIdx, win, ch);
    } else {
      // TX disabled — listen on this channel.
      if (!applyFrequency(ch)) return;  // skip — would listen on wrong channel
      sx126x_mod_params_lora_t mp = buildModParams(CFG_NORMAL);
      int64_t remainUs = slotEndUsFor(slotIndex) - now;
      if (remainUs < (int64_t)BS_RX_MIN_REMAINING_US) return;
      // Reception-preserving scheduler: leave a tail guard so the radio's
      // own timeout fires before the next slot starts. See CLAUDE.md.
      int64_t rxTimeoutUs = remainUs - (int64_t)BS_RX_TAIL_GUARD_US;
      if (rxTimeoutUs < 0) rxTimeoutUs = 0;
      sx126x_pkt_params_lora_t pp = buildPktParams(CFG_NORMAL,255);
      radioStartRxTimeout((uint32_t)(rxTimeoutUs / 15.625f), mp, pp, false,
                          slotIndex, seqIdx, win, ch);
    }

  } else if (win == WIN_LR) {
    if (txSendingEnabled) {
      if (!applyFrequency(ch)) return;  // skip — would TX on wrong channel
      sx126x_mod_params_lora_t mp = buildModParams(CFG_LR);
      uint8_t pkt[3];
      size_t len = buildLRPacketCore(pkt);
      sx126x_pkt_params_lora_t pp = buildPktParams(CFG_LR,(uint8_t)len);
      radioStartTx(pkt, len, mp, pp, true, slotIndex, seqIdx, win, ch);
    }

  } else if (win == WIN_CMD) {
    if (!applyFrequency(activeChannel)) return;  // skip — would listen on wrong channel
    sx126x_mod_params_lora_t mp = buildModParams(CFG_NORMAL);

    bool hasRecentCommand = (lastValidCmdUs != 0 &&
                             (now - lastValidCmdUs) < (int64_t)ROCKET_NO_BASE_HEARD_THRESHOLD_US);
    int64_t timeoutUs = hasRecentCommand ? (int64_t)ROCKET_RX_TIMEOUT_US
                                         : (int64_t)ROCKET_LONG_RX_TIMEOUT_US;
    int64_t remainUs = slotEndUsFor(slotIndex) - now;
    if (remainUs > timeoutUs) remainUs = timeoutUs;
    if (remainUs < (int64_t)BS_RX_MIN_REMAINING_US) return;
    // Reception-preserving scheduler: leave a tail guard so the radio's
    // own timeout fires before the next slot starts. See CLAUDE.md.
    int64_t rxTimeoutUs = remainUs - (int64_t)BS_RX_TAIL_GUARD_US;
    if (rxTimeoutUs < 0) rxTimeoutUs = 0;

    sx126x_pkt_params_lora_t pp = buildPktParams(CFG_NORMAL,255);
    radioStartRxTimeout((uint32_t)(rxTimeoutUs / 15.625f), mp, pp, false,
                        slotIndex, seqIdx, win, activeChannel);

  } else if (win == WIN_OFF) {
    radioStandby();
  }
}

#endif  // ROCKET_RADIO_TEST_MODE
