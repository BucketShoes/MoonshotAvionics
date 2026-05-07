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
float   activeBwKHz   = 500.0f;

// ===================== HOP SEQUENCE =====================

uint8_t hopSeq[NUM_HOP_CHANNELS];

// ===================== RADIO STATE =====================

bool        loraReady  = false;
RadioState  radioState = RADIO_STANDBY;

// ===================== SLOT CLOCK STATE =====================

unsigned long syncAnchorUs      = 0;
uint32_t      syncSeedSlotIndex = 0;
unsigned long lastValidCmdUs    = 0;

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

// Build modulation params for normal telem/cmd slots using current runtime config.
static sx126x_mod_params_lora_t buildNormalModParams() {
  sx126x_mod_params_lora_t mp = {};
  mp.sf   = (sx126x_lora_sf_t)activeSF;
  mp.bw   = bwKHzToEnum(activeBwKHz);
  mp.cr   = SX126X_LORA_CR_4_5;
  mp.ldro = 0;
  return mp;
}

// Build modulation params for WIN_LR slots.
static sx126x_mod_params_lora_t buildLRModParams() {
  sx126x_mod_params_lora_t mp = {};
  mp.sf   = (sx126x_lora_sf_t)LORA_LR_SF;
  mp.bw   = bwKHzToEnum(activeBwKHz);
  mp.cr   = (sx126x_lora_cr_t)LORA_LR_CR;
  mp.ldro = 1;
  return mp;
}

// Build packet params for normal (explicit header, CRC on) TX with given payload length.
static sx126x_pkt_params_lora_t buildNormalPktParams(uint8_t pldLen) {
  sx126x_pkt_params_lora_t pp = {};
  pp.preamble_len_in_symb = LORA_PREAMBLE;
  pp.header_type          = SX126X_LORA_PKT_EXPLICIT;
  pp.pld_len_in_bytes     = pldLen;
  pp.crc_is_on            = true;
  pp.invert_iq_is_on      = false;
  return pp;
}

// Build packet params for WIN_LR TX (implicit header, CRC off, short preamble).
static sx126x_pkt_params_lora_t buildLRPktParams(uint8_t pldLen) {
  sx126x_pkt_params_lora_t pp = {};
  pp.preamble_len_in_symb = 5;
  pp.header_type          = SX126X_LORA_PKT_IMPLICIT;
  pp.pld_len_in_bytes     = pldLen;
  pp.crc_is_on            = false;
  pp.invert_iq_is_on      = false;
  return pp;
}

// Set radio frequency from channel index. Radio must be in standby.
static void applyFrequency(uint8_t ch) {
  float freqMHz = channelToFreqMHz(ch);
  uint32_t freqHz = (uint32_t)(freqMHz * 1e6f + 0.5f);
  sx126x_set_rf_freq(&radioCtx, freqHz);
}

// SX1262 errata §15.3: after any RX with implicit header + timeout, stop the RTC timer
// to prevent a spurious timeout IRQ in the next RX mode.
static void applyImplicitHeaderErrataFix() {
  uint8_t val = 0;
  sx126x_write_register(&radioCtx, 0x0920, &val, 1);
  sx126x_read_register(&radioCtx, 0x0944, &val, 1);
  val |= 0x02;
  sx126x_write_register(&radioCtx, 0x0944, &val, 1);
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

  sx126x_set_reg_mode(&radioCtx, SX126X_REG_MODE_DCDC);
  sx126x_set_pkt_type(&radioCtx, SX126X_PKT_TYPE_LORA);
  sx126x_set_dio2_as_rf_sw_ctrl(&radioCtx, true);

  const uint8_t syncWord[2] = { 0x14, 0x24 };
  sx126x_write_register(&radioCtx, 0x0740, syncWord, 2);

  radioApplyConfig_BLOCKING();

  sx126x_irq_mask_t irqMask = SX126X_IRQ_TX_DONE | SX126X_IRQ_RX_DONE | SX126X_IRQ_TIMEOUT |
                               SX126X_IRQ_CRC_ERROR | SX126X_IRQ_HEADER_ERROR;
  sx126x_set_dio_irq_params(&radioCtx, irqMask, irqMask, SX126X_IRQ_NONE, SX126X_IRQ_NONE);
  sx126x_clear_irq_status(&radioCtx, SX126X_IRQ_ALL);

  sx126x_cfg_rx_boosted(&radioCtx, LORA_RX_BOOSTED);

  radioMcpwmInit(LORA_DIO1_PIN);

  radioCtx.initMode = false;
  radioState = RADIO_STANDBY;

  Serial.print("LoRa init OK: ch="); Serial.print(activeChannel);
  Serial.print(" "); Serial.print(activeFreqMHz, 1); Serial.print("MHz SF");
  Serial.print(activeSF); Serial.print(" BW"); Serial.print((int)activeBwKHz);
  Serial.print("kHz pwr="); Serial.print(activePower); Serial.println("dBm");
  return true;
}

void radioApplyConfig_BLOCKING() {
  sx126x_mod_params_lora_t mp = buildNormalModParams();
  sx126x_set_lora_mod_params(&radioCtx, &mp);
  DO_NOT_CALL_WHILE_ARMED_radioWaitBusy_WARNING_LONG_BLOCKING(&radioCtx);

  sx126x_pkt_params_lora_t pp = buildNormalPktParams(255);
  sx126x_set_lora_pkt_params(&radioCtx, &pp);
  DO_NOT_CALL_WHILE_ARMED_radioWaitBusy_WARNING_LONG_BLOCKING(&radioCtx);

  uint32_t freqHz = (uint32_t)(activeFreqMHz * 1e6f + 0.5f);
  sx126x_set_rf_freq(&radioCtx, freqHz);
  DO_NOT_CALL_WHILE_ARMED_radioWaitBusy_WARNING_LONG_BLOCKING(&radioCtx);

  sx126x_pa_cfg_params_t paCfg = { .pa_duty_cycle = 0x04, .hp_max = 0x07, .device_sel = 0x00, .pa_lut = 0x01 };
  sx126x_set_pa_cfg(&radioCtx, &paCfg);
  DO_NOT_CALL_WHILE_ARMED_radioWaitBusy_WARNING_LONG_BLOCKING(&radioCtx);

  sx126x_set_tx_params(&radioCtx, activePower, SX126X_RAMP_200_US);
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
                         bool isLR) {
  if (digitalRead(LORA_BUSY_PIN)) {
    Serial.println("RX: BUSY at start — skip");
    return;
  }

  // Apply modulation and packet params unconditionally.
  sx126x_set_lora_mod_params(&radioCtx, &modParams);
  {
    unsigned long t0 = micros();
    while (digitalRead(LORA_BUSY_PIN) && (micros() - t0) < 100) {}
  }
  sx126x_set_lora_pkt_params(&radioCtx, &pktParams);

  sx126x_clear_irq_status(&radioCtx, SX126X_IRQ_ALL);
  dio1Fired = false;

  sx126x_status_t st = sx126x_set_rx_with_timeout_in_rtc_step(&radioCtx, timeoutRtcSteps);
  if (st == SX126X_STATUS_OK) {
    radioState      = RADIO_RX_ACTIVE;
    savedModParams  = modParams;
    savedPktParams  = pktParams;
    savedIsLR       = isLR;
    ledOnRX();
    if (LOG_RX_START) {
      Serial.print("RxStart: slot="); Serial.print(radioGetSlotIndex());
      Serial.print(" ch="); Serial.print(hopChannel(radioGetSlotIndex()));
      Serial.print(" timeout="); Serial.print((uint32_t)(timeoutRtcSteps * 15.625f));
      Serial.println("us");
    }
  } else {
    Serial.print("RX: set_rx fail st="); Serial.println(st);
    radioState = RADIO_STANDBY;
  }
}

bool radioStartTx(const uint8_t* pkt, size_t len,
                  const sx126x_mod_params_lora_t& modParams,
                  const sx126x_pkt_params_lora_t& pktParams,
                  bool isLR) {
  if (digitalRead(LORA_BUSY_PIN)) {
    Serial.println("TX: BUSY — skip");
    return false;
  }

  // Apply modulation and packet params unconditionally.
  sx126x_set_lora_mod_params(&radioCtx, &modParams);
  {
    unsigned long t0 = micros();
    while (digitalRead(LORA_BUSY_PIN) && (micros() - t0) < 100) {}
  }
  // For TX, rebuild pkt params with actual payload length.
  sx126x_pkt_params_lora_t ppTx = pktParams;
  ppTx.pld_len_in_bytes = (uint8_t)len;
  sx126x_set_lora_pkt_params(&radioCtx, &ppTx);

  sx126x_clear_irq_status(&radioCtx, SX126X_IRQ_ALL);
  dio1Fired = false;

  sx126x_status_t st = sx126x_write_buffer(&radioCtx, 0, pkt, (uint8_t)len);
  if (st != SX126X_STATUS_OK) {
    Serial.print("TX: write_buffer fail st="); Serial.println(st);
    return false;
  }

  if (digitalRead(LORA_BUSY_PIN)) {
    Serial.println("TX: BUSY after write_buffer — abort");
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
    if (LOG_TX_START) {
      Serial.print("TxStart: slot="); Serial.print(radioGetSlotIndex());
      Serial.print(" ch="); Serial.print(hopChannel(radioGetSlotIndex()));
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
  sx126x_set_standby(&radioCtx, SX126X_STANDBY_CFG_RC);
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

  if (irqFlags & SX126X_IRQ_TX_DONE) {
#ifdef LORA_FEM_PA_PIN
    digitalWrite(LORA_FEM_PA_PIN, LOW);
#endif
    radioState = RADIO_STANDBY;
    ledOff();
    if (LOG_TX_DONE) {
      Serial.print("TxDone: slot="); Serial.println((eventUs - syncAnchorUs) / SLOT_DURATION_US + syncSeedSlotIndex);
    }
  }

  if (irqFlags & SX126X_IRQ_RX_DONE) {
    radioState = RADIO_STANDBY;
    if (savedIsLR) applyImplicitHeaderErrataFix();
    if (LOG_RX_DONE) {
      Serial.print("RxDone: slot="); Serial.println((eventUs - syncAnchorUs) / SLOT_DURATION_US + syncSeedSlotIndex);
    }
    handleRxDone();
    ledOff();
  }

  if (irqFlags & SX126X_IRQ_TIMEOUT) {
    radioState = RADIO_STANDBY;
    if (savedIsLR) applyImplicitHeaderErrataFix();
    ledOff();
    static unsigned long rxTimeoutCountTotal = 0;
    rxTimeoutCountTotal++;
    if (rxTimeoutCountTotal % 10 == 0) {
      Serial.print("RX_TIMEOUT (count="); Serial.print(rxTimeoutCountTotal);
      Serial.print(" slot="); Serial.print((eventUs - syncAnchorUs) / SLOT_DURATION_US + syncSeedSlotIndex);
      Serial.println(")");
    }
    if (LOG_RX_TIMEOUT) {
      Serial.print("RxTimeout: slot="); Serial.println((eventUs - syncAnchorUs) / SLOT_DURATION_US + syncSeedSlotIndex);
    }
  }

  if (irqFlags & (SX126X_IRQ_CRC_ERROR | SX126X_IRQ_HEADER_ERROR)) {
    radioState = RADIO_STANDBY;
    ledOff();
    invalidRxCount++;
    static unsigned long lastRxErrLogMs = 0;
    unsigned long nowMs = millis();
    if (nowMs - lastRxErrLogMs >= 5000) {
      lastRxErrLogMs = nowMs;
      if (irqFlags & SX126X_IRQ_CRC_ERROR)    Serial.print("RX: CRC_ERROR ");
      if (irqFlags & SX126X_IRQ_HEADER_ERROR) Serial.print("RX: HEADER_ERROR ");
      Serial.println();
    }
  }

  if (irqFlags == 0) {
    static unsigned long lastSpuriousLogMs = 0;
    unsigned long nowMs = millis();
    if (nowMs - lastSpuriousLogMs >= 1000) {
      lastSpuriousLogMs = nowMs;
      Serial.print("IRQ: flags=0 radioState="); Serial.print(radioState);
      Serial.print(" BUSY="); Serial.println(digitalRead(LORA_BUSY_PIN));
    }
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
    sx126x_mod_params_lora_t mp = buildNormalModParams();
    sx126x_pkt_params_lora_t pp = buildNormalPktParams(255);
    uint32_t timeoutUs = 100'000UL;
    radioStartRxTimeout((uint32_t)(timeoutUs / 15.625f), mp, pp, false);
  }
}
#else

// nextActionUs: micros() timestamp at which the next slot action should begin.
// Computed when starting each action; the loop polls micros() vs this stamp.
static unsigned long nextActionUs = 0;
// Slot index corresponding to the next action (used to detect when the slot changes).
static uint32_t      nextActionSlotIndex = 0;
// Consecutive unexpected overruns (busy radio at slot boundary, excluding WIN_CONTINUE).
static uint8_t       unexpectedOverruns = 0;

void nonblockingRadio() {
  if (!loraReady) return;

  // Always handle IRQ first if DIO1 fired.
  if (dio1Fired) {
    radioHandleIrq();
  }

  unsigned long now = micros();

  // Compute current slot state.
  uint32_t slotIndex = radioGetSlotIndex();
  uint8_t  seqIdx    = (uint8_t)(slotIndex % SLOT_SEQUENCE_LEN);
  WindowMode win     = SLOT_SEQUENCE[seqIdx];
  uint8_t  ch        = hopChannel(slotIndex);

  // Compute the timestamp for this slot's action.
  // nextActionUs is set when we start an action; if it hasn't been set yet (boot),
  // initialise it to now so we start the first slot immediately.
  if (nextActionUs == 0) {
    nextActionUs        = now;
    nextActionSlotIndex = slotIndex;
  }

  // WIN_CONTINUE: no radio action, no overrun counting. Just advance nextActionUs.
  if (win == WIN_CONTINUE) {
    if (slotIndex != nextActionSlotIndex) {
      nextActionSlotIndex = slotIndex;
      // Advance to after this continue slot.
      nextActionUs = syncAnchorUs + (uint32_t)((slotIndex - syncSeedSlotIndex + 1) * SLOT_DURATION_US);
    }
    return;
  }

  // Not yet time for the next action.
  if ((long)(now - nextActionUs) < 0) return;

  // Time has come (or overdue). Check overrun conditions before checking BUSY.
  bool isOverrun = false;

  if (win == WIN_TELEM || win == WIN_LR) {
    // TX: skip if more than 10ms late.
    if ((long)(now - nextActionUs) > 10'000L) {
      isOverrun = true;
      delayedTxCount++;
    }
  } else if (win == WIN_CMD) {
    // RX: skip if remaining time in slot is < BS_RX_MIN_REMAINING_US equivalent.
    unsigned long slotEndUs = syncAnchorUs + (uint32_t)((slotIndex - syncSeedSlotIndex + 1) * SLOT_DURATION_US);
    if ((long)(slotEndUs - now) < 60'000L) {
      isOverrun = true;
    }
  }

  if (isOverrun) {
    unexpectedOverruns++;
    Serial.print("RADIO: overrun win="); Serial.print((int)win);
    Serial.print(" count="); Serial.println(unexpectedOverruns);
    if (unexpectedOverruns >= 3) {
      Serial.println("RADIO: 3 overruns — forcing standby");
      radioStandby();
      unexpectedOverruns = 0;
    }
    // Advance to next slot.
    nextActionSlotIndex = slotIndex + 1;
    nextActionUs = syncAnchorUs + (uint32_t)((slotIndex - syncSeedSlotIndex + 1) * SLOT_DURATION_US);
    return;
  }

  // Check BUSY pin. If busy, the radio is still running from a previous operation.
  if (digitalRead(LORA_BUSY_PIN)) {
    unexpectedOverruns++;
    if (unexpectedOverruns >= 3) {
      Serial.println("RADIO: BUSY stuck 3 slots — forcing standby");
      radioStandby();
      unexpectedOverruns = 0;
      nextActionSlotIndex = slotIndex + 1;
      nextActionUs = syncAnchorUs + (uint32_t)((slotIndex - syncSeedSlotIndex + 1) * SLOT_DURATION_US);
    }
    return;
  }

  // BUSY is clear and time has come. Take the slot action.
  unexpectedOverruns = 0;
  nextActionSlotIndex = slotIndex + 1;
  nextActionUs = syncAnchorUs + (uint32_t)((slotIndex - syncSeedSlotIndex + 1) * SLOT_DURATION_US);

  if (win == WIN_TELEM) {
    if (txSendingEnabled) {
      applyFrequency(ch);
      sx126x_mod_params_lora_t mp = buildNormalModParams();
      uint8_t pkt[255];
      size_t len = buildTelemetryPacket(pkt);
      sx126x_pkt_params_lora_t pp = buildNormalPktParams((uint8_t)len);
      radioStartTx(pkt, len, mp, pp, false);
    } else {
      // TX disabled — listen on this channel (e.g. ground test mode).
      applyFrequency(ch);
      sx126x_mod_params_lora_t mp = buildNormalModParams();
      uint32_t slotEndUs = syncAnchorUs + (uint32_t)((slotIndex - syncSeedSlotIndex + 1) * SLOT_DURATION_US);
      uint32_t remainUs = (uint32_t)((long)(slotEndUs - now));
      if (remainUs < 60'000UL) return;
      sx126x_pkt_params_lora_t pp = buildNormalPktParams(255);
      radioStartRxTimeout((uint32_t)(remainUs / 15.625f), mp, pp, false);
    }

  } else if (win == WIN_LR) {
    if (txSendingEnabled) {
      applyFrequency(ch);
      sx126x_mod_params_lora_t mp = buildLRModParams();
      uint8_t pkt[3];
      size_t len = buildLRPacketCore(pkt);
      sx126x_pkt_params_lora_t pp = buildLRPktParams((uint8_t)len);
      radioStartTx(pkt, len, mp, pp, true);
    }
    // If TX disabled, stay standby for WIN_LR.

  } else if (win == WIN_CMD) {
    // RX for command on fixed command channel.
    applyFrequency(activeChannel);
    sx126x_mod_params_lora_t mp = buildNormalModParams();

    // Timeout: use short window if we've heard a command recently, long window if not.
    bool hasRecentCommand = (lastValidCmdUs != 0 &&
                             (now - lastValidCmdUs) < ROCKET_NO_BASE_HEARD_THRESHOLD_US);
    uint32_t timeoutUs = hasRecentCommand ? ROCKET_RX_TIMEOUT_US : ROCKET_LONG_RX_TIMEOUT_US;
    // Cap to remaining slot time.
    unsigned long slotEndUs = syncAnchorUs + (uint32_t)((slotIndex - syncSeedSlotIndex + 1) * SLOT_DURATION_US);
    uint32_t remainUs = (uint32_t)((long)(slotEndUs - now));
    if (remainUs >= timeoutUs) remainUs = timeoutUs;  // don't exceed our desired timeout
    if (remainUs < 60'000UL) return;

    sx126x_pkt_params_lora_t pp = buildNormalPktParams(255);
    radioStartRxTimeout((uint32_t)(remainUs / 15.625f), mp, pp, false);

  } else if (win == WIN_OFF) {
    radioStandby();
  }
  // Other WIN_ types: stay standby.
}

#endif  // ROCKET_RADIO_TEST_MODE
