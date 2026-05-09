// base_station/radio.cpp — LoRa radio, passive sync state machine, and slot scheduling.
//
// TEST MODE: uncomment to replace the slot state machine with a bare TX every 5s + continuous RX.
//#define BS_RADIO_TEST_MODE

#include <Arduino.h>
#include "radio.h"
#include "sx126x.h"
#include "sx126x_hal.h"

// ===================== EXTERNAL STATE (from main.cpp) =====================

struct CmdTxState {
  uint8_t pkt[64];
  uint8_t pktLen;
  uint8_t sends;
  uint8_t sent;
  uint16_t waitMs;
  unsigned long lastSendMs;
  unsigned long queuedMs;
  bool active;
};
extern CmdTxState cmdTx;

// ===================== HARDWARE OBJECTS =====================

SPIClass bsLoraSPI(FSPI);

sx126x_hal_context_t bsRadioCtx = {
  .spi           = &bsLoraSPI,
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

uint8_t bhChannel = DEFAULT_BH_CHANNEL;
uint8_t bhSF      = DEFAULT_BH_SF;
int8_t  bhPower   = DEFAULT_BH_POWER;

// ===================== HOP SEQUENCE =====================

uint8_t hopSeq[NUM_HOP_CHANNELS];

// ===================== RADIO STATE =====================

BsRadioState bsRadioState = BS_RADIO_STANDBY;
bool         bsLoraReady  = false;

// ===================== PASSIVE SYNC STATE =====================

BsScanState  bsScanState            = SCAN_SEARCHING;

int64_t bsSyncAnchorUs           = 0;
int64_t bsSyncSeedSlotIndex      = 0;

int64_t bsCandidateAnchorUs      = 0;
int64_t bsCandidateSeedSlotIndex = 0;
float   bsCandidateDriftEmaUs    = 0.0f;

int64_t bsBackupAnchorUs         = 0;
int64_t bsBackupSeedSlotIndex    = 0;
float   bsBackupDriftEmaUs       = 0.0f;

float         bsDriftEmaUs       = 0.0f;

unsigned long bsScanStartMs      = 0;
unsigned long bsCandidateStartMs = 0;

int64_t  bsLastGoodTelemUs       = 0;
uint32_t bsLastTelemErrorUs      = UINT32_MAX;

// ===================== MODULATION SNAPSHOT =====================

sx126x_mod_params_lora_t bsSavedModParams = {};
sx126x_pkt_params_lora_t bsSavedPktParams = {};
bool                      bsSavedIsLR     = false;

// ===================== OTHER STATE =====================

float        bsBgRssiEma  = -128.0f;
unsigned long bsLastTelemRxMs = 0;
unsigned long bsLastCmdSentMs = 0;
bool bsWinCmdReady = false;

// Diagnostic: timestamp when set_rx was last issued, so terminating IRQs can log
// actual-RX-duration for missing-packet diagnosis.
int64_t bsRxIssuedUs = 0;

// ===================== HELPERS =====================

void bsUpdateActiveFreqBw() {
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

// Track which channel the radio is actually tuned to. Hop index is then derived
// from this (independent of slot index, per user requirement).
static uint8_t currentTunedChannel = DEFAULT_CHANNEL;

// Wait for BUSY pin to clear (the chip raises BUSY while it processes a command).
// Returns true if BUSY went low within the timeout. Used between consecutive SPI
// ops so the HAL doesn't drop the next command for "BUSY high."
static bool waitBusyClear(uint32_t timeoutUs) {
  unsigned long t0 = micros();
  while (digitalRead(LORA_BUSY_PIN)) {
    if ((micros() - t0) >= timeoutUs) return false;
  }
  return true;
}

// Returns true if the frequency was actually applied. The HAL drops SPI commands
// when BUSY is high (returns SX126X_HAL_STATUS_ERROR -> non-OK sx126x_status_t).
// Spin briefly for BUSY to clear, issue, check return, then wait for BUSY again
// because set_rf_freq itself raises BUSY while the chip retunes — the very next
// SPI op (typically set_lora_mod_params) would otherwise be dropped.
static bool applyFrequency(uint8_t ch) {
  if (!waitBusyClear(200)) {
    Serial.print("BS applyFrequency: BUSY stuck pre, freq NOT applied ch="); Serial.println(ch);
    return false;
  }
  float freqMHz = channelToFreqMHz(ch);
  uint32_t freqHz = (uint32_t)(freqMHz * 1e6f + 0.5f);
  sx126x_status_t st = sx126x_set_rf_freq(&bsRadioCtx, freqHz);
  if (st != SX126X_STATUS_OK) {
    Serial.print("BS applyFrequency: set_rf_freq rejected st="); Serial.print(st);
    Serial.print(" ch="); Serial.println(ch);
    return false;
  }
  // Wait for BUSY to drop after the freq write so the next SPI op doesn't get
  // HAL-dropped.
  if (!waitBusyClear(200)) {
    Serial.print("BS applyFrequency: BUSY stuck post-write ch="); Serial.println(ch);
    // The freq write went through, so update currentTunedChannel anyway —
    // the chip IS on the new channel, the next caller just may have to wait.
    currentTunedChannel = ch;
    return false;
  }
  currentTunedChannel = ch;
  return true;
}

// SX1262 errata §15.3: after any RX with implicit header + timeout, prevent spurious timeout.
static void applyImplicitHeaderErrataFix() {
  uint8_t val = 0;
  sx126x_status_t st;
  st = sx126x_write_register(&bsRadioCtx, 0x0920, &val, 1);
  if (st != SX126X_STATUS_OK) { Serial.print("BS errata: write 0x0920 fail st="); Serial.println(st); return; }
  st = sx126x_read_register(&bsRadioCtx, 0x0944, &val, 1);
  if (st != SX126X_STATUS_OK) { Serial.print("BS errata: read 0x0944 fail st="); Serial.println(st); return; }
  val |= 0x02;
  st = sx126x_write_register(&bsRadioCtx, 0x0944, &val, 1);
  if (st != SX126X_STATUS_OK) { Serial.print("BS errata: write 0x0944 fail st="); Serial.println(st); }
}

static void bsLedOn()  {
#if LED_MODE == LED_MODE_LOGIC
  ledcWrite(LED_PIN, 64);
#endif
}
static void bsLedOff() {
#if LED_MODE == LED_MODE_LOGIC
  ledcWrite(LED_PIN, 0);
#endif
}
static void bsLedUpdateFromBusy() {
#if LED_MODE == LED_MODE_BUSY
  ledcWrite(LED_PIN, digitalRead(LORA_BUSY_PIN) ? 64 : 0);
#endif
}

// ===================== INIT =====================

bool bsRadioInit() {
  pinMode(LORA_NSS_PIN,  OUTPUT);
  digitalWrite(LORA_NSS_PIN, HIGH);
  pinMode(LORA_BUSY_PIN, INPUT);

  rederiveHopSequence(activeChannel);

  Serial.println("BS LoRa: resetting...");
  sx126x_hal_reset(&bsRadioCtx);
  if (!DO_NOT_CALL_WHILE_ARMED_radioWaitBusy_WARNING_LONG_BLOCKING(&bsRadioCtx, 100)) {
    Serial.println("BS LoRa init FAIL: BUSY stuck after reset");
    return false;
  }

  sx126x_status_t st;
  st = sx126x_set_standby(&bsRadioCtx, SX126X_STANDBY_CFG_RC);
  if (st != SX126X_STATUS_OK) return false;

  st = sx126x_set_dio3_as_tcxo_ctrl(&bsRadioCtx, SX126X_TCXO_CTRL_1_8V, 320);
  if (!DO_NOT_CALL_WHILE_ARMED_radioWaitBusy_WARNING_LONG_BLOCKING(&bsRadioCtx, 100)) {
    Serial.println("BS LoRa init FAIL: BUSY stuck after TCXO setup");
    return false;
  }

  sx126x_set_reg_mode(&bsRadioCtx, SX126X_REG_MODE_DCDC);
  sx126x_set_pkt_type(&bsRadioCtx, SX126X_PKT_TYPE_LORA);
  sx126x_set_dio2_as_rf_sw_ctrl(&bsRadioCtx, true);

  const uint8_t syncWord[2] = { 0x14, 0x24 };
  sx126x_write_register(&bsRadioCtx, 0x0740, syncWord, 2);

  bsRadioApplyConfig_BLOCKING();

  sx126x_irq_mask_t irqMask = SX126X_IRQ_TX_DONE | SX126X_IRQ_RX_DONE | SX126X_IRQ_TIMEOUT |
                               SX126X_IRQ_CRC_ERROR | SX126X_IRQ_HEADER_ERROR;
  sx126x_set_dio_irq_params(&bsRadioCtx, irqMask, irqMask, SX126X_IRQ_NONE, SX126X_IRQ_NONE);
  sx126x_clear_irq_status(&bsRadioCtx, SX126X_IRQ_ALL);

  sx126x_cfg_rx_boosted(&bsRadioCtx, LORA_RX_BOOSTED);

  radioMcpwmInit(LORA_DIO1_PIN);

  bsRadioCtx.initMode = false;
  bsRadioState        = BS_RADIO_STANDBY;

  // Start passive scan on boot.
  bsScanState   = SCAN_SEARCHING;
  bsScanStartMs = millis();

  Serial.print("BS LoRa init OK: ch="); Serial.print(activeChannel);
  Serial.print(" "); Serial.print(activeFreqMHz, 1); Serial.print("MHz SF");
  Serial.print(activeSF); Serial.print(" BW"); Serial.print((int)activeBwKHz);
  Serial.print("kHz pwr="); Serial.print(activePower); Serial.println("dBm");
  return true;
}

void bsRadioApplyConfig_BLOCKING() {
  sx126x_mod_params_lora_t mp = buildModParams(CFG_NORMAL);
  sx126x_set_lora_mod_params(&bsRadioCtx, &mp);
  DO_NOT_CALL_WHILE_ARMED_radioWaitBusy_WARNING_LONG_BLOCKING(&bsRadioCtx);

  sx126x_pkt_params_lora_t pp = buildPktParams(CFG_NORMAL,255);
  sx126x_set_lora_pkt_params(&bsRadioCtx, &pp);
  DO_NOT_CALL_WHILE_ARMED_radioWaitBusy_WARNING_LONG_BLOCKING(&bsRadioCtx);

  uint32_t freqHz = (uint32_t)(activeFreqMHz * 1e6f + 0.5f);
  sx126x_set_rf_freq(&bsRadioCtx, freqHz);
  DO_NOT_CALL_WHILE_ARMED_radioWaitBusy_WARNING_LONG_BLOCKING(&bsRadioCtx);

  sx126x_pa_cfg_params_t paCfg = { .pa_duty_cycle = 0x04, .hp_max = 0x07, .device_sel = 0x00, .pa_lut = 0x01 };
  sx126x_set_pa_cfg(&bsRadioCtx, &paCfg);
  DO_NOT_CALL_WHILE_ARMED_radioWaitBusy_WARNING_LONG_BLOCKING(&bsRadioCtx);

  sx126x_set_tx_params(&bsRadioCtx, activePower, SX126X_RAMP_200_US);
  DO_NOT_CALL_WHILE_ARMED_radioWaitBusy_WARNING_LONG_BLOCKING(&bsRadioCtx);

  rederiveHopSequence(activeChannel);
}

// ===================== RX / TX =====================

// slotIndex/seqIdx/win/ch are the values the slot machine decided to act on —
// pass them through so logs reflect the actual decision, not whatever
// bsGetSlotIndex() happens to return at log time (which has been observed to
// disagree with the decision, especially when the action runs late).
void bsRadioStartRxTimeout(uint32_t timeoutRtcSteps,
                            const sx126x_mod_params_lora_t& modParams,
                            const sx126x_pkt_params_lora_t& pktParams,
                            bool isLR,
                            int64_t slotIndex, uint8_t seqIdx, WindowMode win, uint8_t ch) {
  if (LOG_BS_RX_ATTEMPT) {
    int64_t now = esp_timer_get_time();
    int64_t slotStart = bsSyncAnchorUs + (slotIndex - bsSyncSeedSlotIndex) * (int64_t)SLOT_DURATION_US;
    Serial.print("BS RxAttempt: slot="); Serial.print((long long)slotIndex);
    Serial.print(" seq="); Serial.print(seqIdx);
    Serial.print(" win="); Serial.print(windowModeName(win));
    Serial.print(" ch="); Serial.print(ch);
    Serial.print(" tunedCh="); Serial.print(currentTunedChannel);
    Serial.print(" intoSlotUs="); Serial.print((long long)(now - slotStart));
    Serial.print(" busy="); Serial.print(digitalRead(LORA_BUSY_PIN));
    Serial.print(" priorState="); Serial.print((int)bsRadioState);
    Serial.print(" dio1Fired="); Serial.println((int)dio1Fired);
  }
  // Reception-preserving scheduler: it is EXPECTED for set_rx to be issued
  // while the chip is still in a previous RX. If a packet is mid-flight the
  // chip will reject the new command (handled below via st != OK) and we
  // simply let the previous RX complete. Do NOT call set_standby here.
  if (digitalRead(LORA_BUSY_PIN)) {
    if (LOG_BS_RX_ATTEMPT) Serial.println("BS RX: BUSY at start — skip");
    return;
  }

  // Each SPI op raises BUSY while the chip processes it; the HAL drops the next
  // command if BUSY is still high. waitBusyClear between ops keeps the sequence
  // moving without dropped writes.
  if (!waitBusyClear(200)) { Serial.println("BS RX: BUSY stuck before mod_params"); return; }
  sx126x_status_t stMod = sx126x_set_lora_mod_params(&bsRadioCtx, &modParams);
  if (stMod != SX126X_STATUS_OK) {
    Serial.print("BS RX: set_lora_mod_params rejected st="); Serial.println(stMod);
    return;  // would RX with stale modulation -> garbage demod
  }
  if (!waitBusyClear(200)) { Serial.println("BS RX: BUSY stuck before pkt_params"); return; }
  sx126x_status_t stPkt = sx126x_set_lora_pkt_params(&bsRadioCtx, &pktParams);
  if (stPkt != SX126X_STATUS_OK) {
    Serial.print("BS RX: set_lora_pkt_params rejected st="); Serial.println(stPkt);
    return;  // would RX with stale pkt params -> wrong header mode / payload len
  }
  if (!waitBusyClear(200)) { Serial.println("BS RX: BUSY stuck before clear_irq"); return; }
  sx126x_status_t stClr = sx126x_clear_irq_status(&bsRadioCtx, SX126X_IRQ_ALL);
  if (stClr != SX126X_STATUS_OK) {
    Serial.print("BS RX: clear_irq_status rejected st="); Serial.println(stClr);
    return;
  }
  dio1Fired = false;

  if (!waitBusyClear(200)) { Serial.println("BS RX: BUSY stuck before set_rx"); return; }
  sx126x_status_t st = sx126x_set_rx_with_timeout_in_rtc_step(&bsRadioCtx, timeoutRtcSteps);
  if (st == SX126X_STATUS_OK) {
    bsRadioState    = BS_RADIO_RX_ACTIVE;
    bsSavedModParams = modParams;
    bsSavedPktParams = pktParams;
    bsSavedIsLR     = isLR;
    extern int64_t bsRxIssuedUs;
    bsRxIssuedUs = esp_timer_get_time();
    bsLedOn();
    if (LOG_BS_RX_START) {
      Serial.print("BS RxStart: slot="); Serial.print((long long)slotIndex);
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
    // in progress. This is expected under the reception-preserving scheduler;
    // do NOT change bsRadioState (the chip is still busy with whatever it
    // was doing, not in standby).
    if (LOG_BS_RX_ATTEMPT) {
      Serial.print("BS RX: set_rx rejected st="); Serial.print(st);
      Serial.print(" priorState="); Serial.println((int)bsRadioState);
    }
  }
}

bool bsRadioStartTx(const uint8_t* pkt, size_t len) {
  // Snapshot the slot context ONCE at entry so all subsequent logs in this
  // function refer to the same decision, not whatever bsGetSlotIndex() drifts
  // to during the SPI sequence. (TX is always WIN_CMD on activeChannel.)
  const int64_t  txSlot   = bsGetSlotIndex();
  const uint8_t  txSeqIdx = (uint8_t)(((uint64_t)(txSlot - bsSyncSeedSlotIndex)) % SLOT_SEQUENCE_LEN);
  const uint8_t  txCh     = activeChannel;
  if (LOG_BS_TX_ATTEMPT) {
    Serial.print("BS TxAttempt: slot="); Serial.print((long long)txSlot);
    Serial.print(" seq="); Serial.print(txSeqIdx);
    Serial.print(" win="); Serial.print(windowModeName(WIN_CMD));
    Serial.print(" ch="); Serial.print(txCh);
    Serial.print(" tunedCh="); Serial.print(currentTunedChannel);
    Serial.print(" len="); Serial.print((unsigned)len);
    Serial.print(" busy="); Serial.println(digitalRead(LORA_BUSY_PIN));
  }
  if (digitalRead(LORA_BUSY_PIN)) {
    if (LOG_BS_TX_ATTEMPT) Serial.println("BS TX: BUSY — skip");
    return false;
  }

  // Commands always use the command channel (activeChannel) with normal modulation.
  // applyFrequency checks the SPI return — if it failed (HAL drop / chip rejection)
  // we'd be TX'ing on whatever channel the radio was last tuned to, which on this
  // base station is more often than not a hop channel, not the command channel.
  if (!applyFrequency(activeChannel)) {
    Serial.println("BS TX: applyFrequency failed — abort TX");
    return false;
  }

  sx126x_mod_params_lora_t mp = buildModParams(CFG_NORMAL);
  if (!waitBusyClear(200)) { Serial.println("BS TX: BUSY stuck before mod_params"); return false; }
  sx126x_status_t stMod = sx126x_set_lora_mod_params(&bsRadioCtx, &mp);
  if (stMod != SX126X_STATUS_OK) {
    Serial.print("BS TX: set_lora_mod_params rejected st="); Serial.println(stMod);
    return false;
  }

  sx126x_pkt_params_lora_t pp = buildPktParams(CFG_NORMAL,(uint8_t)len);
  if (!waitBusyClear(200)) { Serial.println("BS TX: BUSY stuck before pkt_params"); return false; }
  sx126x_status_t stPkt = sx126x_set_lora_pkt_params(&bsRadioCtx, &pp);
  if (stPkt != SX126X_STATUS_OK) {
    Serial.print("BS TX: set_lora_pkt_params rejected st="); Serial.println(stPkt);
    return false;
  }

  if (!waitBusyClear(200)) { Serial.println("BS TX: BUSY stuck before clear_irq"); return false; }
  sx126x_status_t stClr = sx126x_clear_irq_status(&bsRadioCtx, SX126X_IRQ_ALL);
  if (stClr != SX126X_STATUS_OK) {
    Serial.print("BS TX: clear_irq_status rejected st="); Serial.println(stClr);
    return false;
  }
  dio1Fired = false;

  if (!waitBusyClear(200)) { Serial.println("BS TX: BUSY stuck before write_buffer"); return false; }
  sx126x_status_t st = sx126x_write_buffer(&bsRadioCtx, 0, pkt, (uint8_t)len);
  if (st != SX126X_STATUS_OK) {
    Serial.print("BS TX: write_buffer fail st="); Serial.println(st);
    return false;
  }

  {
    unsigned long t0 = micros();
    while (digitalRead(LORA_BUSY_PIN)) {
      if (micros() - t0 > 500) {
        Serial.println("BS TX: BUSY stuck after write_buffer — abort");
        return false;
      }
    }
  }

  st = sx126x_set_tx(&bsRadioCtx, 0);
  if (st == SX126X_STATUS_OK) {
    bsRadioState = BS_RADIO_TX_ACTIVE;
    bsLedOn();
    if (LOG_BS_TX_START) {
      Serial.print("BS TxStart: slot="); Serial.print((long long)txSlot);
      Serial.print(" seq="); Serial.print(txSeqIdx);
      Serial.print(" win="); Serial.print(windowModeName(WIN_CMD));
      Serial.print(" ch="); Serial.print(txCh);
      Serial.print(" tunedCh="); Serial.print(currentTunedChannel);
      Serial.print(" len="); Serial.println((unsigned)len);
    }
    return true;
  }
  Serial.print("BS TX: set_tx fail st="); Serial.println(st);
  bsRadioState = BS_RADIO_STANDBY;
  return false;
}

void bsRadioStandby() {
  // The HAL drops SPI commands while BUSY is high. If our standby command got
  // dropped, the chip is NOT in standby — and if we set bsRadioState=STANDBY
  // anyway, the slot machine will think the radio is idle and re-issue set_rx
  // every loop, all of which the chip will reject because it's still mid-RX.
  // That's the wedged-BUSY loop. Spin briefly for BUSY, check the return, and
  // only update state on success.
  {
    unsigned long t0 = micros();
    while (digitalRead(LORA_BUSY_PIN) && (micros() - t0) < 200) {}
  }
  if (digitalRead(LORA_BUSY_PIN)) {
    Serial.println("BS standby: BUSY stuck — standby NOT issued, state unchanged");
    return;
  }
  sx126x_status_t st = sx126x_set_standby(&bsRadioCtx, SX126X_STANDBY_CFG_RC);
  if (st != SX126X_STATUS_OK) {
    Serial.print("BS standby: set_standby rejected st="); Serial.print(st);
    Serial.println(" — state unchanged");
    return;
  }
  bsRadioState = BS_RADIO_STANDBY;
  bsLedOff();
}

void bsTriggerScan() {
  bsBackupAnchorUs        = bsSyncAnchorUs;
  bsBackupSeedSlotIndex   = bsSyncSeedSlotIndex;
  bsBackupDriftEmaUs      = bsDriftEmaUs;
  bsScanState             = SCAN_SEARCHING;
  bsScanStartMs           = millis();
  Serial.println("BS: triggered passive scan");
}

// ===================== DRIFT EMA =====================

// Called on each valid WIN_TELEM RxDone only.
// drift = rxStart_actual - expectedSlotStart. Uses saved modulation params + 5ms/packet cap.
static void applyDriftCorrection(int64_t rxDoneUs, int64_t slotIndex,
                                  int64_t* anchorUs, float* driftEma) {
  uint32_t airtimeMs = sx126x_get_lora_time_on_air_in_ms(&bsSavedPktParams, &bsSavedModParams);
  int64_t  airtimeUs = (int64_t)airtimeMs * 1000;

  int64_t expectedStartUs = *anchorUs + (slotIndex - bsSyncSeedSlotIndex) * (int64_t)SLOT_DURATION_US;
  int64_t rxStartUs       = rxDoneUs - airtimeUs;
  int32_t driftUs         = (int32_t)(rxStartUs - expectedStartUs);

  *driftEma = (*driftEma) * 0.95f + (float)driftUs * 0.05f;

  int32_t correction = (int32_t)(*driftEma * 0.05f);
  if (correction >  5000) correction =  5000;
  if (correction < -5000) correction = -5000;

  *anchorUs += correction;

  static int64_t lastDriftLogUs = 0;
  int64_t nowL = esp_timer_get_time();
  if (nowL - lastDriftLogUs > 1'000'000) {
    lastDriftLogUs = nowL;
    Serial.print("BS DRIFT: airtimeMs="); Serial.print(airtimeMs);
    Serial.print(" drift="); Serial.print(driftUs);
    Serial.print(" ema="); Serial.print(*driftEma, 1);
    Serial.print(" corr="); Serial.print(correction);
    Serial.print(" slot="); Serial.println((long long)slotIndex);
  }
}

// ===================== RX PACKET HANDLER =====================

extern void bsOnPacketReceived(const uint8_t* buf, size_t len, float snrF, float rssiF,
                               int64_t slotIndex, uint8_t seqIdx, uint8_t win,
                               uint32_t timeOnAirMs, float driftEmaUs);

static void bsHandleRxDone(uint64_t eventUs) {
  sx126x_rx_buffer_status_t bufStatus = {};
  sx126x_status_t st = sx126x_get_rx_buffer_status(&bsRadioCtx, &bufStatus);
  if (st != SX126X_STATUS_OK) {
    Serial.print("BS RX: get_rx_buffer_status fail st="); Serial.println(st);
    return;
  }

  uint8_t buf[255];
  uint8_t rxLen = bufStatus.pld_len_in_bytes;
  if (rxLen == 0 || rxLen > sizeof(buf)) {
    Serial.print("BS RX: bad length "); Serial.println(rxLen);
    return;
  }

  st = sx126x_read_buffer(&bsRadioCtx, bufStatus.buffer_start_pointer, buf, rxLen);
  if (st != SX126X_STATUS_OK) {
    Serial.print("BS RX: read_buffer fail st="); Serial.println(st);
    return;
  }

  // Update actual payload length in saved pkt params for airtime calc.
  bsSavedPktParams.pld_len_in_bytes = rxLen;

  sx126x_pkt_status_lora_t pktStatus = {};
  sx126x_get_lora_pkt_status(&bsRadioCtx, &pktStatus);
  float snrF  = (float)pktStatus.snr_pkt_in_db;
  float rssiF = (float)pktStatus.rssi_pkt_in_dbm;

  uint32_t timeOnAirMs = sx126x_get_lora_time_on_air_in_ms(&bsSavedPktParams, &bsSavedModParams);

  int64_t  slotIndex = bsGetSlotIndex();
  uint8_t  seqIdx    = (uint8_t)(((uint64_t)(slotIndex - bsSyncSeedSlotIndex)) % SLOT_SEQUENCE_LEN);
  WindowMode win     = SLOT_SEQUENCE[seqIdx];

  // WIN_LR: reconstruct the 5-byte format.
  if (bsSavedIsLR && rxLen == 3) {
    uint8_t synth[5];
    synth[0] = PKT_LONGRANGE;
    synth[1] = FAVORITE_ROCKET_DEVICE_ID;
    synth[2] = buf[0]; synth[3] = buf[1]; synth[4] = buf[2];
    bsOnPacketReceived(synth, 5, snrF, rssiF, slotIndex, seqIdx, (uint8_t)win, timeOnAirMs, bsDriftEmaUs);
    return;
  }

  bool isTelemetry = (rxLen >= 10 && buf[0] == 0xAF && buf[1] == FAVORITE_ROCKET_DEVICE_ID);

  // ---- Passive sync logic ----
  if (isTelemetry) {
    bsLastTelemRxMs = millis();

    uint16_t stateFlags = (uint16_t)buf[8] | ((uint16_t)buf[9] << 8);
    uint8_t  packetSlotSeqIdx = (stateFlags >> 12) & 0x0F;

    int64_t airtimeUs = (int64_t)timeOnAirMs * 1000;
    int64_t rxStartUs = (int64_t)eventUs - airtimeUs;

    if (bsScanState == SCAN_SEARCHING) {
      // anchor + packetSlotSeqIdx * SLOT_DURATION_US = rxStartUs
      bsCandidateAnchorUs      = rxStartUs - (int64_t)packetSlotSeqIdx * (int64_t)SLOT_DURATION_US;
      bsCandidateSeedSlotIndex = 0;
      bsCandidateDriftEmaUs    = 0.0f;

      bsScanState        = SCAN_CANDIDATE;
      bsCandidateStartMs = millis();

      bsSyncAnchorUs      = bsCandidateAnchorUs;
      bsSyncSeedSlotIndex = bsCandidateSeedSlotIndex;

      Serial.print("BS SCAN: candidate anchor="); Serial.print((long long)bsCandidateAnchorUs);
      Serial.print(" packetSlot="); Serial.println(packetSlotSeqIdx);

    } else {
      int64_t expectedSlotStartUs = bsSyncAnchorUs +
        (slotIndex - bsSyncSeedSlotIndex) * (int64_t)SLOT_DURATION_US;
      int64_t timingErrorUs = rxStartUs - expectedSlotStartUs;
      bsLastTelemErrorUs = (uint32_t)(timingErrorUs < 0 ? -timingErrorUs : timingErrorUs);

      if (bsScanState == SCAN_CANDIDATE) {
        applyDriftCorrection((int64_t)eventUs, slotIndex, &bsSyncAnchorUs, &bsCandidateDriftEmaUs);

        if (bsLastTelemErrorUs < BS_IN_SYNC_TIMING_US) {
          bsDriftEmaUs      = bsCandidateDriftEmaUs;
          bsScanState       = SCAN_LOCKED;
          bsLastGoodTelemUs = esp_timer_get_time();
          Serial.print("BS SCAN: LOCKED anchor="); Serial.print((long long)bsSyncAnchorUs);
          Serial.print(" error="); Serial.print(bsLastTelemErrorUs); Serial.println("us");
        }
      } else {
        applyDriftCorrection((int64_t)eventUs, slotIndex, &bsSyncAnchorUs, &bsDriftEmaUs);

        if (bsLastTelemErrorUs < BS_IN_SYNC_TIMING_US) {
          bsLastGoodTelemUs = esp_timer_get_time();
        }
      }
    }
  }

  bsOnPacketReceived(buf, rxLen, snrF, rssiF, slotIndex, seqIdx, (uint8_t)win, timeOnAirMs, bsDriftEmaUs);
}

// ===================== IRQ HANDLER =====================

static void bsRadioHandleIrq() {
  uint64_t eventUs = dio1TimestampUs();
  dio1Fired = false;

  sx126x_irq_mask_t irqFlags = 0;
  sx126x_status_t st = sx126x_get_and_clear_irq_status(&bsRadioCtx, &irqFlags);
  if (st != SX126X_STATUS_OK) {
    Serial.print("BS IRQ: get_and_clear fail st="); Serial.println(st);
    bsRadioState = BS_RADIO_STANDBY;
    bsLedOff();
    return;
  }

  if (irqFlags & SX126X_IRQ_TX_DONE) {
    bsRadioState = BS_RADIO_STANDBY;
    bsLedOff();
    if (LOG_BS_TX_DONE) {
      Serial.print("BS TxDone: slot="); Serial.println((long long)bsGetSlotIndex());
    }
  }

  if (irqFlags & SX126X_IRQ_RX_DONE) {
    bsRadioState = BS_RADIO_STANDBY;
    if (bsSavedIsLR) applyImplicitHeaderErrataFix();
    bsHandleRxDone(eventUs);
    bsLedOff();
    if (LOG_BS_RX_DONE) {
      int64_t durUs = (int64_t)eventUs - bsRxIssuedUs;
      Serial.print("BS RxDone: slot="); Serial.print((long long)bsGetSlotIndex());
      Serial.print(" rxDurUs="); Serial.print((long long)durUs);
      Serial.print(" irqFlags=0x"); Serial.println((unsigned)irqFlags, HEX);
    }
  }

  if (irqFlags & SX126X_IRQ_TIMEOUT) {
    bsRadioState = BS_RADIO_STANDBY;
    if (bsSavedIsLR) applyImplicitHeaderErrataFix();
    bsLedOff();
    if (LOG_BS_RX_TIMEOUT) {
      int64_t durUs = (int64_t)eventUs - bsRxIssuedUs;
      Serial.print("BS RxTimeout: slot="); Serial.print((long long)bsGetSlotIndex());
      Serial.print(" rxDurUs="); Serial.println((long long)durUs);
    }

    // If searching, restart infinite-timeout RX on same channel.
    if (bsScanState == SCAN_SEARCHING) {
      // Will restart in bsHandleRadio() below.
    }
  }

  if (irqFlags & (SX126X_IRQ_CRC_ERROR | SX126X_IRQ_HEADER_ERROR)) {
    bsRadioState = BS_RADIO_STANDBY;
    bsLedOff();
    if (irqFlags & SX126X_IRQ_CRC_ERROR)    Serial.print("BS RX: CRC_ERROR ");
    if (irqFlags & SX126X_IRQ_HEADER_ERROR) Serial.print("BS RX: HEADER_ERROR ");
    Serial.print("slot="); Serial.print((long long)bsGetSlotIndex());
    Serial.print(" irqFlags=0x"); Serial.println((unsigned)irqFlags, HEX);
  }

  if (irqFlags == 0) {
    Serial.print("BS IRQ: flags=0 state="); Serial.print(bsRadioState);
    Serial.print(" BUSY="); Serial.println(digitalRead(LORA_BUSY_PIN));
  }
}

// ===================== MAIN RADIO UPDATE =====================

#ifdef BS_RADIO_TEST_MODE

static const uint8_t bsTestPkt[] = {
  0x9A, 0x92, 0x40,
  0x01, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};
static unsigned long bsTestLastTxMs = 0;

void bsHandleRadio() {
  if (!dio1Fired && digitalRead(LORA_DIO1_PIN) &&
      (bsRadioState == BS_RADIO_TX_ACTIVE || bsRadioState == BS_RADIO_RX_ACTIVE)) {
    dio1CaptureVal = micros();
    dio1Fired = true;
  }
  if (dio1Fired) bsRadioHandleIrq();
  unsigned long nowMs = millis();
  if (bsRadioState == BS_RADIO_STANDBY) {
    sx126x_mod_params_lora_t mp = buildModParams(CFG_NORMAL);
    sx126x_pkt_params_lora_t pp = buildPktParams(CFG_NORMAL,255);
    bsRadioStartRxTimeout((uint32_t)(100'000UL / 15.625f), mp, pp, false,
                          0, 0, WIN_TELEM, currentTunedChannel);
  }
  if (nowMs - bsTestLastTxMs >= 5000) {
    bsTestLastTxMs = nowMs;
    if (bsRadioState == BS_RADIO_RX_ACTIVE) bsRadioStandby();
    bsRadioStartTx(bsTestPkt, sizeof(bsTestPkt));
  }
}

#else

// Slot scheduling state. All time arithmetic is int64 µs (esp_timer_get_time()).
//
// User architectural rule: actions span slot boundaries (e.g. RX on telem starts 20ms
// before its slot to catch the leading edge). So we don't track "did we act in THIS
// slot" — we track which slot we last STARTED an action for. If lastActionSlot ==
// targetSlot, that target's action is already underway; nothing to do.
static int64_t bsLastActionStartedSlot = -1;  // absolute slot index whose action we last fired
static uint8_t bsUnexpectedOverruns   = 0;

// Rate-limit recurring diagnostics.
static int64_t  bsLastOverrunLogUs = 0;
static int64_t  bsLastBusyLogUs    = 0;
static uint32_t bsOverrunSinceLog  = 0;
static uint32_t bsBusySinceLog     = 0;

// Background RSSI EMA
static bool  bsBgRssiInit  = false;
static bool  bsBgRssiReady = false;
#define BG_RSSI_ALPHA  0.05f

static inline int64_t bsSlotStartUsFor(int64_t slotIndex) {
  return bsSyncAnchorUs + (slotIndex - bsSyncSeedSlotIndex) * (int64_t)SLOT_DURATION_US;
}

// Compute the absolute timestamp at which the action for the given slot is supposed
// to BEGIN. Accounts for early-RX (telem/LR/CMD-listen) and late-TX (cmd dispatch)
// offsets. WIN_CONTINUE has no action; caller must skip it before calling this.
static inline int64_t bsActionStartUsFor(int64_t slotIndex, WindowMode win) {
  int64_t slotStart = bsSlotStartUsFor(slotIndex);
  switch (win) {
    case WIN_TELEM:
    case WIN_LR:    return slotStart - (int64_t)BS_RX_EARLY_US;
    case WIN_CMD:   return slotStart + (int64_t)BS_CMD_TX_OFFSET_US;
    default:        return slotStart;
  }
}

void bsHandleRadio() {
  if (dio1Fired) {
    bsRadioHandleIrq();
    bsBgRssiReady = false;
  }

  bsLedUpdateFromBusy();

  // Background RSSI sampling.
  if (bsRadioState == BS_RADIO_RX_ACTIVE) {
    if (!bsBgRssiReady) {
      bsBgRssiReady = true;
    } else {
      int16_t rssiDbm = 0;
      bsRadioCtx.allowBusyRead = true;
      if (sx126x_get_rssi_inst(&bsRadioCtx, &rssiDbm) == SX126X_STATUS_OK) {
        if (!bsBgRssiInit) { bsBgRssiEma = (float)rssiDbm; bsBgRssiInit = true; }
        else bsBgRssiEma += BG_RSSI_ALPHA * ((float)rssiDbm - bsBgRssiEma);
      }
      bsRadioCtx.allowBusyRead = false;
    }
  }

  int64_t now = esp_timer_get_time();

  // ---- SCAN_SEARCHING: continuous single-shot infinite-timeout RX ----
  if (bsScanState == SCAN_SEARCHING) {
    if ((millis() - bsScanStartMs) >= BS_SCAN_TOTAL_MS) {
      if (bsBackupAnchorUs != 0) {
        bsSyncAnchorUs      = bsBackupAnchorUs;
        bsSyncSeedSlotIndex = bsBackupSeedSlotIndex;
        bsDriftEmaUs        = bsBackupDriftEmaUs;
        bsScanState         = SCAN_LOCKED;
        Serial.println("BS SCAN: timeout — reverted to backup anchor");
      } else {
        bsScanStartMs = millis();
        Serial.println("BS SCAN: timeout — restarting search");
      }
      if (bsRadioState == BS_RADIO_RX_ACTIVE) bsRadioStandby();
      return;
    }

    if (bsRadioState == BS_RADIO_STANDBY) {
      uint8_t scanCh = hopSeq[2];  // TODO: random per-attempt channel
      if (!applyFrequency(scanCh)) return;  // skip — would listen on wrong channel
      sx126x_mod_params_lora_t mp = buildModParams(CFG_NORMAL);
      sx126x_pkt_params_lora_t pp = buildPktParams(CFG_NORMAL,255);
      // SCAN_SEARCHING — slot context is meaningless (we have no anchor yet);
      // pass 0 / WIN_TELEM / scanCh just so the log fields exist.
      bsRadioStartRxTimeout(0, mp, pp, false, 0, 0, WIN_TELEM, scanCh);
      if (LOG_BS_RX_START) {
        Serial.print("BS SCAN: searching on ch="); Serial.println(scanCh);
      }
    }
    return;
  }

  // ---- SCAN_CANDIDATE: check candidate timer ----
  if (bsScanState == SCAN_CANDIDATE) {
    if ((millis() - bsCandidateStartMs) >= BS_CANDIDATE_TIMEOUT_MS) {
      Serial.println("BS SCAN: candidate timeout — restarting");
      bsScanState = SCAN_SEARCHING;
      if (bsRadioState == BS_RADIO_RX_ACTIVE) bsRadioStandby();
    }
  }

  // ---- Slotted operation (CANDIDATE or LOCKED) ----

  int64_t  slotIndex = bsGetSlotIndex();
  uint8_t  seqIdx    = (uint8_t)(((uint64_t)(slotIndex - bsSyncSeedSlotIndex)) % SLOT_SEQUENCE_LEN);
  WindowMode win     = SLOT_SEQUENCE[seqIdx];

  // Hop position is INDEPENDENT of slot index. We pick the channel based on the slot
  // type (telem/LR -> hopSeq driven, but per-slot chosen by an independent counter):
  // for now, the rocket and base both index hopSeq[] by an independent counter that
  // advances on each hopped action. To stay synced without a separate channel field,
  // both sides use slotIndex (a shared count) but the user's plan is that this counter
  // is logically the hop counter, not the slot counter. For now this is a single-counter
  // implementation; CRT-style coupling has been removed.
  uint8_t  ch        = hopChannel((uint32_t)(slotIndex - bsSyncSeedSlotIndex));

  if (win == WIN_CONTINUE) return;

  // Compute when this slot's action is supposed to begin. Skip forward through
  // already-acted-on slots; an action might already be underway from the previous
  // slot (e.g. a long telem RX started 20ms early).
  int64_t actionStartUs = bsActionStartUsFor(slotIndex, win);

  // Already started the action for this slot — wait for next slot.
  if (bsLastActionStartedSlot == slotIndex) return;

  // Compute when the NEXT slot's action would start — this gives our action's hard
  // deadline (we must finish before the next radio action begins).
  int64_t nextSlot = slotIndex + 1;
  // skip WIN_CONTINUE when computing next-action-start (continue = no action)
  WindowMode nextWin = SLOT_SEQUENCE[(uint8_t)(((uint64_t)(nextSlot - bsSyncSeedSlotIndex)) % SLOT_SEQUENCE_LEN)];
  while (nextWin == WIN_CONTINUE) {
    nextSlot++;
    nextWin = SLOT_SEQUENCE[(uint8_t)(((uint64_t)(nextSlot - bsSyncSeedSlotIndex)) % SLOT_SEQUENCE_LEN)];
  }
  int64_t nextActionStartUs = bsActionStartUsFor(nextSlot, nextWin);

  // Not yet time to start this slot's action.
  if (now < actionStartUs) return;

  // How much time remains before the next action must begin? That's our timeout.
  int64_t remainUs = nextActionStartUs - now;
  if (remainUs < (int64_t)BS_RX_MIN_REMAINING_US) {
    bsUnexpectedOverruns++;
    if (LOG_BS_OVERRUN) {
      Serial.print("BS RADIO: overrun win="); Serial.print(windowModeName(win));
      Serial.print(" consec="); Serial.print(bsUnexpectedOverruns);
      Serial.print(" slot="); Serial.print((long long)slotIndex);
      Serial.print(" remainUs="); Serial.println((long long)remainUs);
    }
    if (bsUnexpectedOverruns >= BS_OVERRUN_MAX) {
      if (LOG_BS_OVERRUN) Serial.println("BS RADIO: 3 overruns — forcing standby");
      bsRadioStandby();
      bsUnexpectedOverruns = 0;
    }
    bsLastActionStartedSlot = slotIndex;
    return;
  }

  // BUSY check — radio mid-action from previous slot.
  if (digitalRead(LORA_BUSY_PIN)) {
    bsUnexpectedOverruns++;
    if (LOG_BS_OVERRUN) {
      Serial.print("BS RADIO: BUSY at boundary consec="); Serial.print(bsUnexpectedOverruns);
      Serial.print(" slot="); Serial.print((long long)slotIndex);
      Serial.print(" win="); Serial.println(windowModeName(win));
    }
    if (bsUnexpectedOverruns >= BS_OVERRUN_MAX) {
      if (LOG_BS_OVERRUN) Serial.println("BS RADIO: BUSY stuck — forcing standby");
      bsRadioStandby();
      bsUnexpectedOverruns = 0;
    }
    return;
  }
  bsUnexpectedOverruns = 0;

  // Time has come and BUSY is clear — fire the slot's action.
  bsLastActionStartedSlot = slotIndex;

  // Reception-preserving scheduler: RX timeout ends BEFORE the next slot's
  // action would start, by BS_RX_TAIL_GUARD_US. If the radio's own timeout
  // hasn't fired by then, a packet is arriving and we must let it complete —
  // the next slot's set_rx attempt will be rejected by the chip, which we
  // tolerate (counted as overrun). Never preempt in-flight RX.
  int64_t rxTimeoutUs = remainUs - (int64_t)BS_RX_TAIL_GUARD_US;
  if (rxTimeoutUs < 0) rxTimeoutUs = 0;
  uint32_t timeoutSteps = (uint32_t)(rxTimeoutUs / 15.625f);

  if (win == WIN_TELEM) {
    if (!applyFrequency(ch)) return;  // skip — would listen on wrong channel
    sx126x_mod_params_lora_t mp = buildModParams(CFG_NORMAL);
    sx126x_pkt_params_lora_t pp = buildPktParams(CFG_NORMAL, 255);
    bsRadioStartRxTimeout(timeoutSteps, mp, pp, false, slotIndex, seqIdx, win, ch);

  } else if (win == WIN_LR) {
    if (!applyFrequency(ch)) return;  // skip — would listen on wrong channel
    sx126x_mod_params_lora_t mp = buildModParams(CFG_LR);
    sx126x_pkt_params_lora_t pp = buildPktParams(CFG_LR, 3);
    bsRadioStartRxTimeout(timeoutSteps, mp, pp, true, slotIndex, seqIdx, win, ch);

  } else if (win == WIN_CMD) {
    // Signal the main loop's command dispatcher.
    bsWinCmdReady = true;
    // If no command actually queued, fall back to listening on the backhaul channel
    // for the remainder of this slot.
    if (!cmdTx.active && bsRadioState == BS_RADIO_STANDBY) {
      if (!applyFrequency(bhChannel)) return;  // skip — would listen on wrong channel
      sx126x_mod_params_lora_t mp = {
        (sx126x_lora_sf_t)bhSF, bwKHzToEnum(channelToBwKHz(bhChannel)),
        SX126X_LORA_CR_4_5, 0
      };
      sx126x_pkt_params_lora_t pp = buildPktParams(CFG_NORMAL, 255);
      bsRadioStartRxTimeout(timeoutSteps, mp, pp, false, slotIndex, seqIdx, win, bhChannel);
    }

  } else if (win == WIN_OFF) {
    // WIN_OFF intentionally requests low-power standby — explicit user intent,
    // not a preemption of in-flight RX.
    bsRadioStandby();
  }

  // Periodic state dump.
  if (LOG_BS_HB) {
    static unsigned long bsLastDumpMs = 0;
    unsigned long nowMs = millis();
    if (nowMs - bsLastDumpMs >= 5000) {
      bsLastDumpMs = nowMs;
      long telemAge = (bsLastTelemRxMs == 0) ? -1 : (long)(nowMs - bsLastTelemRxMs);
      const char* stateStr = (bsScanState == SCAN_SEARCHING) ? "SEARCH" :
                             (bsScanState == SCAN_CANDIDATE) ? "CAND" : "LOCK";
      Serial.print("BS STATE: scan="); Serial.print(stateStr);
      Serial.print(" inSync="); Serial.print(bsInSync() ? 'Y' : 'N');
      Serial.print(" rstate="); Serial.print((int)bsRadioState);
      Serial.print(" busy="); Serial.print(digitalRead(LORA_BUSY_PIN));
      Serial.print(" slot="); Serial.print((long long)slotIndex);
      Serial.print(" seqIdx="); Serial.print(seqIdx);
      Serial.print(" anchor="); Serial.print((long long)bsSyncAnchorUs);
      Serial.print(" telemAgeMs="); Serial.print(telemAge);
      Serial.print(" dio1ISR="); Serial.println(dio1IsrCount);
    }
  }
}

#endif  // BS_RADIO_TEST_MODE
