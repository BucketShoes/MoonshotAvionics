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
float   activeBwKHz   = 500.0f;

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

unsigned long bsSyncAnchorUs        = 0;
uint32_t      bsSyncSeedSlotIndex   = 0;

unsigned long bsCandidateAnchorUs   = 0;
uint32_t      bsCandidateSeedSlotIndex = 0;
float         bsCandidateDriftEmaUs = 0.0f;

unsigned long bsBackupAnchorUs      = 0;
uint32_t      bsBackupSeedSlotIndex = 0;
float         bsBackupDriftEmaUs    = 0.0f;

float         bsDriftEmaUs          = 0.0f;

unsigned long bsScanStartMs         = 0;
unsigned long bsCandidateStartMs    = 0;

unsigned long bsLastGoodTelemUs     = 0;
uint32_t      bsLastTelemErrorUs    = UINT32_MAX;

// ===================== MODULATION SNAPSHOT =====================

sx126x_mod_params_lora_t bsSavedModParams = {};
sx126x_pkt_params_lora_t bsSavedPktParams = {};
bool                      bsSavedIsLR     = false;

// ===================== OTHER STATE =====================

float        bsBgRssiEma  = -128.0f;
unsigned long bsLastTelemRxMs = 0;
unsigned long bsLastCmdSentMs = 0;
bool bsWinCmdReady = false;

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

static sx126x_mod_params_lora_t buildNormalModParams() {
  sx126x_mod_params_lora_t mp = {};
  mp.sf   = (sx126x_lora_sf_t)activeSF;
  mp.bw   = bwKHzToEnum(activeBwKHz);
  mp.cr   = SX126X_LORA_CR_4_5;
  mp.ldro = 0;
  return mp;
}

static sx126x_mod_params_lora_t buildLRModParams() {
  sx126x_mod_params_lora_t mp = {};
  mp.sf   = (sx126x_lora_sf_t)LORA_LR_SF;
  mp.bw   = bwKHzToEnum(activeBwKHz);
  mp.cr   = (sx126x_lora_cr_t)LORA_LR_CR;
  mp.ldro = 1;
  return mp;
}

static sx126x_pkt_params_lora_t buildNormalPktParams(uint8_t pldLen) {
  sx126x_pkt_params_lora_t pp = {};
  pp.preamble_len_in_symb = LORA_PREAMBLE;
  pp.header_type          = SX126X_LORA_PKT_EXPLICIT;
  pp.pld_len_in_bytes     = pldLen;
  pp.crc_is_on            = true;
  pp.invert_iq_is_on      = false;
  return pp;
}

static sx126x_pkt_params_lora_t buildLRPktParams(uint8_t pldLen) {
  sx126x_pkt_params_lora_t pp = {};
  pp.preamble_len_in_symb = 5;
  pp.header_type          = SX126X_LORA_PKT_IMPLICIT;
  pp.pld_len_in_bytes     = pldLen;
  pp.crc_is_on            = false;
  pp.invert_iq_is_on      = false;
  return pp;
}

static void applyFrequency(uint8_t ch) {
  float freqMHz = channelToFreqMHz(ch);
  uint32_t freqHz = (uint32_t)(freqMHz * 1e6f + 0.5f);
  sx126x_set_rf_freq(&bsRadioCtx, freqHz);
}

// SX1262 errata §15.3: after any RX with implicit header + timeout, prevent spurious timeout.
static void applyImplicitHeaderErrataFix() {
  uint8_t val = 0;
  sx126x_write_register(&bsRadioCtx, 0x0920, &val, 1);
  sx126x_read_register(&bsRadioCtx, 0x0944, &val, 1);
  val |= 0x02;
  sx126x_write_register(&bsRadioCtx, 0x0944, &val, 1);
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
  sx126x_mod_params_lora_t mp = buildNormalModParams();
  sx126x_set_lora_mod_params(&bsRadioCtx, &mp);
  DO_NOT_CALL_WHILE_ARMED_radioWaitBusy_WARNING_LONG_BLOCKING(&bsRadioCtx);

  sx126x_pkt_params_lora_t pp = buildNormalPktParams(255);
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

void bsRadioStartRxTimeout(uint32_t timeoutRtcSteps,
                            const sx126x_mod_params_lora_t& modParams,
                            const sx126x_pkt_params_lora_t& pktParams,
                            bool isLR) {
  if (digitalRead(LORA_BUSY_PIN)) {
    Serial.println("BS RX: BUSY at start — skip");
    return;
  }

  sx126x_set_lora_mod_params(&bsRadioCtx, &modParams);
  {
    unsigned long t0 = micros();
    while (digitalRead(LORA_BUSY_PIN) && (micros() - t0) < 100) {}
  }
  sx126x_set_lora_pkt_params(&bsRadioCtx, &pktParams);

  sx126x_clear_irq_status(&bsRadioCtx, SX126X_IRQ_ALL);
  dio1Fired = false;

  sx126x_status_t st = sx126x_set_rx_with_timeout_in_rtc_step(&bsRadioCtx, timeoutRtcSteps);
  if (st == SX126X_STATUS_OK) {
    bsRadioState    = BS_RADIO_RX_ACTIVE;
    bsSavedModParams = modParams;
    bsSavedPktParams = pktParams;
    bsSavedIsLR     = isLR;
    bsLedOn();
    if (LOG_RX_START) {
      Serial.print("BS RxStart: slot="); Serial.print(bsGetSlotIndex());
      Serial.print(" ch="); Serial.print(hopChannel(bsGetSlotIndex()));
      Serial.print(" timeout="); Serial.print((uint32_t)(timeoutRtcSteps * 15.625f));
      Serial.println("us");
    }
  } else {
    Serial.print("BS RX: set_rx fail st="); Serial.println(st);
    bsRadioState = BS_RADIO_STANDBY;
  }
}

bool bsRadioStartTx(const uint8_t* pkt, size_t len) {
  if (digitalRead(LORA_BUSY_PIN)) {
    Serial.println("BS TX: BUSY — drop");
    return false;
  }

  // Commands always use normal modulation on the command channel.
  sx126x_mod_params_lora_t mp = buildNormalModParams();
  sx126x_set_lora_mod_params(&bsRadioCtx, &mp);
  {
    unsigned long t0 = micros();
    while (digitalRead(LORA_BUSY_PIN) && (micros() - t0) < 100) {}
  }

  sx126x_pkt_params_lora_t pp = buildNormalPktParams((uint8_t)len);
  sx126x_set_lora_pkt_params(&bsRadioCtx, &pp);

  sx126x_clear_irq_status(&bsRadioCtx, SX126X_IRQ_ALL);
  dio1Fired = false;

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
    if (LOG_TX_START) {
      Serial.print("BS TxStart: slot="); Serial.print(bsGetSlotIndex());
      Serial.print(" len="); Serial.println((unsigned)len);
    }
    return true;
  }
  Serial.print("BS TX: set_tx fail st="); Serial.println(st);
  bsRadioState = BS_RADIO_STANDBY;
  return false;
}

void bsRadioStandby() {
  sx126x_set_standby(&bsRadioCtx, SX126X_STANDBY_CFG_RC);
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
// drift = time from expected slot start to actual packet start = rxDoneUs - airtimeUs - expectedSlotStartUs
// Uses saved modulation params (not current settings). 5ms per-packet cap.
static void applyDriftCorrection(uint64_t rxDoneUs, uint32_t slotIndex,
                                  unsigned long* anchorUs, float* driftEma) {
  // Airtime from saved params at RX start.
  uint32_t rxLen = bsSavedPktParams.pld_len_in_bytes;
  sx126x_pkt_params_lora_t pp = bsSavedPktParams;
  pp.pld_len_in_bytes = rxLen;
  uint32_t airtimeMs = sx126x_get_lora_time_on_air_in_ms(&pp, &bsSavedModParams);
  uint32_t airtimeUs = airtimeMs * 1000;

  // Expected slot start based on anchor.
  uint32_t slotsSinceAnchor = slotIndex - bsSyncSeedSlotIndex;
  int64_t expectedStartUs = (int64_t)(*anchorUs) + (int64_t)slotsSinceAnchor * SLOT_DURATION_US;
  int64_t rxStartUs = (int64_t)rxDoneUs - (int64_t)airtimeUs;
  int32_t driftUs = (int32_t)(rxStartUs - expectedStartUs);

  // Slow EMA (alpha = 0.05).
  *driftEma = (*driftEma) * 0.95f + (float)driftUs * 0.05f;

  // Per-packet cap: 5ms max correction.
  int32_t correction = (int32_t)(*driftEma * 0.05f);
  if (correction >  5000) correction =  5000;
  if (correction < -5000) correction = -5000;

  *anchorUs = (unsigned long)((int64_t)(*anchorUs) + correction);
}

// ===================== RX PACKET HANDLER =====================

extern void bsOnPacketReceived(const uint8_t* buf, size_t len, float snrF, float rssiF,
                               uint32_t slotIndex, uint8_t seqIdx, uint8_t win,
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

  uint32_t slotIndex = bsGetSlotIndex();
  uint8_t  seqIdx    = (uint8_t)(slotIndex % SLOT_SEQUENCE_LEN);
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

    // Extract slot_seq_idx from bits [12:15] of stateFlags (bytes 8–9 in 0xAF header).
    uint16_t stateFlags = (uint16_t)buf[8] | ((uint16_t)buf[9] << 8);
    uint8_t packetSlotSeqIdx = (stateFlags >> 12) & 0x0F;

    // Compute expected slot start for this packet.
    uint32_t airtimeUs = timeOnAirMs * 1000;
    uint64_t rxStartUs = (uint64_t)eventUs - (uint64_t)airtimeUs;

    if (bsScanState == SCAN_SEARCHING) {
      // Compute candidate anchor: anchor such that slot packetSlotSeqIdx starts at rxStartUs.
      // bsSyncSeedSlotIndex=0 means anchor is the start of slot 0 in the sequence.
      // We want: anchor + packetSlotSeqIdx * SLOT_DURATION_US = rxStartUs
      // So: anchor = rxStartUs - packetSlotSeqIdx * SLOT_DURATION_US
      bsCandidateAnchorUs       = (unsigned long)(rxStartUs - (uint64_t)packetSlotSeqIdx * SLOT_DURATION_US);
      bsCandidateSeedSlotIndex  = 0;
      bsCandidateDriftEmaUs     = 0.0f;

      // Transition to CANDIDATE.
      bsScanState        = SCAN_CANDIDATE;
      bsCandidateStartMs = millis();

      // Switch to slotted operation using candidate anchor.
      bsSyncAnchorUs      = bsCandidateAnchorUs;
      bsSyncSeedSlotIndex = bsCandidateSeedSlotIndex;

      Serial.print("BS SCAN: candidate anchor="); Serial.print(bsCandidateAnchorUs);
      Serial.print(" packetSlot="); Serial.println(packetSlotSeqIdx);

    } else {
      // SCAN_CANDIDATE or SCAN_LOCKED: apply drift correction.
      // Compute timing error against current anchor.
      uint32_t slotsSinceAnchor = slotIndex - bsSyncSeedSlotIndex;
      int64_t expectedSlotStartUs = (int64_t)bsSyncAnchorUs + (int64_t)slotsSinceAnchor * SLOT_DURATION_US;
      int64_t timingErrorUs = rxStartUs - expectedSlotStartUs;
      bsLastTelemErrorUs = (uint32_t)(timingErrorUs < 0 ? -timingErrorUs : timingErrorUs);

      if (bsScanState == SCAN_CANDIDATE) {
        applyDriftCorrection(eventUs, slotIndex, &bsSyncAnchorUs, &bsCandidateDriftEmaUs);

        if (bsLastTelemErrorUs < BS_IN_SYNC_TIMING_US) {
          // Timing consistent — lock in.
          bsDriftEmaUs        = bsCandidateDriftEmaUs;
          bsScanState         = SCAN_LOCKED;
          bsLastGoodTelemUs   = (unsigned long)micros();
          Serial.print("BS SCAN: LOCKED anchor="); Serial.print(bsSyncAnchorUs);
          Serial.print(" error="); Serial.print(bsLastTelemErrorUs); Serial.println("us");
        }
      } else {
        // SCAN_LOCKED: normal drift EMA.
        applyDriftCorrection(eventUs, slotIndex, &bsSyncAnchorUs, &bsDriftEmaUs);

        if (bsLastTelemErrorUs < BS_IN_SYNC_TIMING_US) {
          bsLastGoodTelemUs = (unsigned long)micros();
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
    if (LOG_TX_DONE) {
      Serial.print("BS TxDone: slot="); Serial.println(bsGetSlotIndex());
    }
  }

  if (irqFlags & SX126X_IRQ_RX_DONE) {
    bsRadioState = BS_RADIO_STANDBY;
    if (bsSavedIsLR) applyImplicitHeaderErrataFix();
    bsHandleRxDone(eventUs);
    bsLedOff();
    if (LOG_RX_DONE) {
      Serial.print("BS RxDone: slot="); Serial.println(bsGetSlotIndex());
    }
  }

  if (irqFlags & SX126X_IRQ_TIMEOUT) {
    bsRadioState = BS_RADIO_STANDBY;
    if (bsSavedIsLR) applyImplicitHeaderErrataFix();
    bsLedOff();
    static unsigned long bsRxTimeoutCountTotal = 0;
    bsRxTimeoutCountTotal++;
    if (bsRxTimeoutCountTotal % 10 == 0) {
      Serial.print("BS RX_TIMEOUT (count="); Serial.print(bsRxTimeoutCountTotal);
      Serial.println(")");
    }
    if (LOG_RX_TIMEOUT) {
      Serial.print("BS RxTimeout: slot="); Serial.println(bsGetSlotIndex());
    }

    // If searching, restart infinite-timeout RX on same channel.
    if (bsScanState == SCAN_SEARCHING) {
      // Will restart in bsHandleRadio() below.
    }
  }

  if (irqFlags & (SX126X_IRQ_CRC_ERROR | SX126X_IRQ_HEADER_ERROR)) {
    bsRadioState = BS_RADIO_STANDBY;
    bsLedOff();
    static unsigned long lastRxErrLogMs = 0;
    unsigned long nowMs = millis();
    if (nowMs - lastRxErrLogMs >= 5000) {
      lastRxErrLogMs = nowMs;
      if (irqFlags & SX126X_IRQ_CRC_ERROR)    Serial.print("BS RX: CRC_ERROR ");
      if (irqFlags & SX126X_IRQ_HEADER_ERROR) Serial.print("BS RX: HEADER_ERROR ");
      Serial.println();
    }
  }

  if (irqFlags == 0) {
    static unsigned long lastSpuriousLogMs = 0;
    unsigned long nowMs = millis();
    if (nowMs - lastSpuriousLogMs >= 1000) {
      lastSpuriousLogMs = nowMs;
      Serial.print("BS IRQ: flags=0 state="); Serial.print(bsRadioState);
      Serial.print(" BUSY="); Serial.println(digitalRead(LORA_BUSY_PIN));
    }
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
    sx126x_mod_params_lora_t mp = buildNormalModParams();
    sx126x_pkt_params_lora_t pp = buildNormalPktParams(255);
    bsRadioStartRxTimeout((uint32_t)(100'000UL / 15.625f), mp, pp, false);
  }
  if (nowMs - bsTestLastTxMs >= 5000) {
    bsTestLastTxMs = nowMs;
    if (bsRadioState == BS_RADIO_RX_ACTIVE) bsRadioStandby();
    bsRadioStartTx(bsTestPkt, sizeof(bsTestPkt));
  }
}

#else

// nextActionUs: timestamp at which the next slot action should start.
static unsigned long bsNextActionUs     = 0;
static uint32_t      bsNextActionSlot   = 0;
static uint8_t       bsUnexpectedOverruns = 0;
static bool          bsCmdSentThisSlot  = false;
static bool          bsRxStartedThisSlot = false;

// Background RSSI EMA
static bool  bsBgRssiInit  = false;
static bool  bsBgRssiReady = false;
#define BG_RSSI_ALPHA  0.05f

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

  unsigned long now = micros();

  // ---- SCAN_SEARCHING: continuous single-shot infinite-timeout RX ----
  if (bsScanState == SCAN_SEARCHING) {
    // Check scan timeout.
    if ((millis() - bsScanStartMs) >= BS_SCAN_TOTAL_MS) {
      // Scan timed out.
      if (bsBackupAnchorUs != 0) {
        // Revert to backup.
        bsSyncAnchorUs      = bsBackupAnchorUs;
        bsSyncSeedSlotIndex = bsBackupSeedSlotIndex;
        bsDriftEmaUs        = bsBackupDriftEmaUs;
        bsScanState         = SCAN_LOCKED;
        Serial.println("BS SCAN: timeout — reverted to backup anchor");
      } else {
        // No prior lock; restart.
        bsScanStartMs = millis();
        Serial.println("BS SCAN: timeout — restarting search");
      }
      if (bsRadioState == BS_RADIO_RX_ACTIVE) bsRadioStandby();
      return;
    }

    // Start infinite-timeout RX if not already running.
    if (bsRadioState == BS_RADIO_STANDBY) {
      uint8_t scanCh = hopSeq[2];  // always index 2 per design
      applyFrequency(scanCh);
      sx126x_mod_params_lora_t mp = buildNormalModParams();
      sx126x_pkt_params_lora_t pp = buildNormalPktParams(255);
      // timeout=0 = infinite single-shot RX (SX1262 stays in RX until packet or explicit standby).
      bsRadioStartRxTimeout(0, mp, pp, false);
      if (LOG_RX_START) {
        Serial.print("BS SCAN: searching on ch="); Serial.println(scanCh);
      }
    }
    return;
  }

  // ---- SCAN_CANDIDATE: check candidate timer ----
  if (bsScanState == SCAN_CANDIDATE) {
    if ((millis() - bsCandidateStartMs) >= BS_CANDIDATE_TIMEOUT_MS) {
      // Candidate failed — restart searching (within the scan timer).
      Serial.println("BS SCAN: candidate timeout — restarting");
      bsScanState = SCAN_SEARCHING;
      // Don't reset bsScanStartMs — the 60s total scan timer keeps running.
      if (bsRadioState == BS_RADIO_RX_ACTIVE) bsRadioStandby();
    }
  }

  // ---- Slotted operation (CANDIDATE or LOCKED) ----

  uint32_t slotIndex = bsGetSlotIndex();
  uint8_t  seqIdx    = (uint8_t)(slotIndex % SLOT_SEQUENCE_LEN);
  WindowMode win     = SLOT_SEQUENCE[seqIdx];
  uint8_t  ch        = hopChannel(slotIndex);

  // Initialise nextActionUs on first entry or after mode switch.
  if (bsNextActionUs == 0) {
    bsNextActionUs   = now;
    bsNextActionSlot = slotIndex;
    bsCmdSentThisSlot  = false;
    bsRxStartedThisSlot = false;
  }

  // WIN_CONTINUE: no action.
  if (win == WIN_CONTINUE) {
    if (slotIndex != bsNextActionSlot) {
      bsNextActionSlot = slotIndex;
      unsigned long slotsSince = slotIndex - bsSyncSeedSlotIndex;
      bsNextActionUs = bsSyncAnchorUs + slotsSince * SLOT_DURATION_US;
    }
    return;
  }

  // Slot transition bookkeeping.
  if (slotIndex != bsNextActionSlot) {
    bsNextActionSlot    = slotIndex;
    bsCmdSentThisSlot   = false;
    bsRxStartedThisSlot = false;
    unsigned long slotsSince = slotIndex - bsSyncSeedSlotIndex;
    // Base station: RX starts 20ms early; TX starts 5ms late.
    if (win == WIN_TELEM || win == WIN_LR) {
      bsNextActionUs = bsSyncAnchorUs + slotsSince * SLOT_DURATION_US - BS_RX_EARLY_US;
    } else if (win == WIN_CMD) {
      bsNextActionUs = bsSyncAnchorUs + slotsSince * SLOT_DURATION_US + BS_CMD_TX_OFFSET_US;
    } else {
      bsNextActionUs = bsSyncAnchorUs + slotsSince * SLOT_DURATION_US;
    }
  }

  // WIN_CMD: signal dispatch at BS_CMD_TX_OFFSET_US past boundary.
  if (win == WIN_CMD && !bsCmdSentThisSlot) {
    unsigned long slotsSince = slotIndex - bsSyncSeedSlotIndex;
    unsigned long slotBoundary = bsSyncAnchorUs + slotsSince * SLOT_DURATION_US;
    if ((long)(now - (slotBoundary + BS_CMD_TX_OFFSET_US)) >= 0) {
      bsCmdSentThisSlot = true;
      bsWinCmdReady = true;
      // If no command, listen on backhaul channel/modulation for the remainder.
      if (!cmdTx.active && !bsRxStartedThisSlot && bsRadioState == BS_RADIO_STANDBY) {
        applyFrequency(bhChannel);
        sx126x_mod_params_lora_t mp = {};
        mp.sf   = (sx126x_lora_sf_t)bhSF;
        mp.bw   = bwKHzToEnum(channelToBwKHz(bhChannel));
        mp.cr   = SX126X_LORA_CR_4_5;
        mp.ldro = 0;
        sx126x_pkt_params_lora_t pp = buildNormalPktParams(255);
        unsigned long slotEnd = slotBoundary + SLOT_DURATION_US;
        uint32_t remainUs = (uint32_t)((long)(slotEnd - now));
        if (remainUs >= BS_RX_MIN_REMAINING_US) {
          bsRadioStartRxTimeout((uint32_t)(remainUs / 15.625f), mp, pp, false);
          bsRxStartedThisSlot = true;
        }
      }
    }
    return;
  }

  // Not yet time for this slot's action.
  if ((long)(now - bsNextActionUs) < 0) return;

  // Check overrun conditions.
  if (win == WIN_TELEM || win == WIN_LR) {
    unsigned long slotsSince = slotIndex - bsSyncSeedSlotIndex;
    unsigned long slotEnd = bsSyncAnchorUs + (slotsSince + 1) * SLOT_DURATION_US;
    uint32_t remainUs = (uint32_t)((long)(slotEnd - now));
    if (remainUs < BS_RX_MIN_REMAINING_US) {
      bsUnexpectedOverruns++;
      if (bsUnexpectedOverruns >= BS_OVERRUN_MAX) {
        Serial.println("BS RADIO: 3 overruns — forcing standby");
        bsRadioStandby();
        bsUnexpectedOverruns = 0;
      }
      return;
    }
  }

  // Check BUSY.
  if (digitalRead(LORA_BUSY_PIN)) {
    bsUnexpectedOverruns++;
    if (bsUnexpectedOverruns >= BS_OVERRUN_MAX) {
      Serial.println("BS RADIO: BUSY stuck — forcing standby");
      bsRadioStandby();
      bsUnexpectedOverruns = 0;
    }
    return;
  }
  bsUnexpectedOverruns = 0;

  if (bsRxStartedThisSlot) return;
  bsRxStartedThisSlot = true;

  if (win == WIN_TELEM) {
    applyFrequency(ch);
    sx126x_mod_params_lora_t mp = buildNormalModParams();
    unsigned long slotsSince = slotIndex - bsSyncSeedSlotIndex;
    unsigned long slotEnd = bsSyncAnchorUs + (slotsSince + 1) * SLOT_DURATION_US;
    uint32_t remainUs = (uint32_t)((long)(slotEnd - now));
    sx126x_pkt_params_lora_t pp = buildNormalPktParams(255);
    bsRadioStartRxTimeout((uint32_t)(remainUs / 15.625f), mp, pp, false);

  } else if (win == WIN_LR) {
    applyFrequency(ch);
    sx126x_mod_params_lora_t mp = buildLRModParams();
    sx126x_pkt_params_lora_t pp = buildLRPktParams(3);
    unsigned long slotsSince = slotIndex - bsSyncSeedSlotIndex;
    unsigned long slotEnd = bsSyncAnchorUs + (slotsSince + 1) * SLOT_DURATION_US;
    uint32_t remainUs = (uint32_t)((long)(slotEnd - now));
    if (remainUs >= BS_RX_MIN_REMAINING_US) {
      bsRadioStartRxTimeout((uint32_t)(remainUs / 15.625f), mp, pp, true);
    }

  } else if (win == WIN_OFF) {
    bsRadioStandby();
  }

  // Periodic state dump.
  {
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
      Serial.print(" slot="); Serial.print(slotIndex);
      Serial.print(" seqIdx="); Serial.print(seqIdx);
      Serial.print(" telemAgeMs="); Serial.print(telemAge);
      Serial.print(" dio1ISR="); Serial.println(dio1IsrCount);
    }
  }
}

#endif  // BS_RADIO_TEST_MODE
