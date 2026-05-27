// Base Station / Relay - Heltec Wireless Tracker V1.1
// Supports multiple transports: WiFi/WebSocket, BLE GATT, (future: USB Serial)
// BLE allows the phone to maintain mobile internet while connected to base station.
#include <Arduino.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <Preferences.h>
#include <mbedtls/md.h>
#include <NimBLEDevice.h>
#include <LittleFS.h>
#include <esp_ota_ops.h>
#include "log_store.h"
#include "esp_wifi.h"
#include "radio.h"
//#include "tagged_serial.h"  // Serial wrapper that prefixes boot-relative micros

#define VEXT_CTRL_PIN 3
#define VBAT_ADC_PIN      1
#define VBAT_ADC_CTRL_PIN 2
#define VBAT_MULTIPLIER   4.9f
#define USER_BTN_PIN      0   // GPIO0 / BOOT button on Tracker v1.1

#define SERIAL_BAUD   2000000
#include "secrets.h"  // gitignored — copy secrets_example.h to secrets.h
#define WIFI_CHANNEL  1

#define DEVICE_ID     157

// Verbose per-PDU fetch logs. Disable when BLE log fetch is stable.
#define BLE_FETCH_VERBOSE 1

// ===================== TAGGED SERIAL =====================
unsigned long bootMicros = 0;
//TaggedSerial taggedSerial(&Serial0);

// ===================== OTA COMMAND IDs (matching rocket_avionics/config.h) =====================
#define CMD_OTA_BEGIN     0x50
#define CMD_OTA_FINALIZE  0x51
#define CMD_OTA_CONFIRM   0x52

// ===================== TRANSPORT FLAGS (for 0x31/0x32 enable/disable) =====================
#define TRANSPORT_WIFI  0x01
#define TRANSPORT_BLE   0x02
#define TRANSPORT_USB   0x04  // reserved for future
#define TRANSPORT_ALL   (TRANSPORT_WIFI | TRANSPORT_BLE | TRANSPORT_USB)

bool wifiEnabled = false;
bool bleEnabled = true;

// ===================== HARDWARE =====================

AsyncWebServer httpServer(80);
AsyncWebSocket ws("/ws");
LogStore logStore;
bool logStoreOk = false;
uint16_t baseBattMv = 0;

void readBaseBattery() {
  // VBAT_ADC_CTRL_PIN drives an NPN base with no series resistor on the PCB.
  // Use INPUT_PULLUP (~50kΩ) to switch the transistor without burning 40mA.
  pinMode(VBAT_ADC_CTRL_PIN, INPUT_PULLUP);
  delayMicroseconds(400);
  uint32_t adcMv = analogReadMilliVolts(VBAT_ADC_PIN);
  pinMode(VBAT_ADC_CTRL_PIN, INPUT);  // tri-state: base floating, transistor off
  baseBattMv = (uint16_t)(adcMv * VBAT_MULTIPLIER);
}

static void enableWifi() {
  if (wifiEnabled) return;
  static bool serverStarted = false;
  WiFi.mode(WIFI_AP);
  WiFi.softAP(WIFI_SSID, WIFI_PASS, WIFI_CHANNEL);
  esp_wifi_set_max_tx_power(20); // 0.25dBm units
  esp_wifi_set_ps(WIFI_PS_MAX_MODEM);
  if (!serverStarted) { httpServer.begin(); serverStarted = true; }
  wifiEnabled = true;
  Serial.println("WiFi: AP on");
}

static void disableWifi() {
  if (!wifiEnabled) return;
  ws.closeAll(); // cleanly disconnect WebSocket clients before pulling WiFi
  WiFi.softAPdisconnect(true);
  WiFi.mode(WIFI_OFF);
  esp_wifi_stop();
  wifiEnabled = false;
  Serial.println("WiFi: off");
}

// Long flash = WiFi on, 5 short = WiFi off
static void ledSignalWifiOn() {
  ledcWrite(LED_PIN, 2047);
  delay(800);
  ledcWrite(LED_PIN, 0);
}

static void ledSignalWifiOff() {
  for (int i = 0; i < 5; i++) {
    ledcWrite(LED_PIN, 2047);
    delay(80);
    ledcWrite(LED_PIN, 0);
    delay(80);
  }
}

Preferences bsNvs;
uint32_t highestNonce = 0;

// ===================== BLE GATT SERVER =====================

#define BLE_DEVICE_NAME         "MoonBase"
#define BLE_SHORT_NAME          "MoonBase"   // 8 chars — fits 31-byte legacy adv payload alongside UUID128
#define BLE_SERVICE_UUID        "4d4f4f4e-5348-4f54-4253-000000000000"  // ASCII: MOONSHOTBS
#define BLE_TELEM_CHAR_UUID     "4d4f4f4e-5348-4f54-4253-000000000001"
#define BLE_CMD_CHAR_UUID       "4d4f4f4e-5348-4f54-4253-000000000002"
#define BLE_STATUS_CHAR_UUID    "4d4f4f4e-5348-4f54-4253-000000000003"
#define BLE_LOGFETCH_CHAR_UUID  "4d4f4f4e-5348-4f54-4253-000000000004"
#define BLE_OTA_CHAR_UUID       "4d4f4f4e-5348-4f54-4253-000000000006"  // WRITE|WRITE_NR|NOTIFY

NimBLEServer* bleServer = nullptr;
NimBLECharacteristic* bleTelemChar = nullptr;
NimBLECharacteristic* bleCmdChar = nullptr;
NimBLECharacteristic* bleStatusChar = nullptr;
NimBLECharacteristic* bleLogFetchChar = nullptr;
NimBLECharacteristic* bleOtaChar = nullptr;
bool bleClientConnected = false;

// ===================== OTA STATE MACHINE =====================
#define OTA_STATUS_OK            0x00
#define OTA_STATUS_NOT_ACTIVE    0x01
#define OTA_STATUS_WRITE_FAIL    0x02
#define OTA_STATUS_OFFSET_GAP    0x03
#define OTA_STATUS_HMAC_MISMATCH 0x04
#define OTA_STATUS_OTA_END_FAIL  0x05
#define OTA_STATUS_VERIFYING     0x06
#define OTA_STATUS_REFUSED       0x07
#define OTA_PROGRESS_MARKER      0xA0
#define OTA_PROGRESS_INTERVAL    1200

enum OtaState { OTA_LOCKED, OTA_ERASING, OTA_RECEIVING, OTA_VERIFYING };

struct OtaContext {
  OtaState              state;
  esp_ota_handle_t      handle;
  const esp_partition_t* partition;
  uint32_t              bytesWritten;
  uint32_t              chunkCount;
  bool                  notifyPending;
  uint8_t               notifyBuf[8];
  uint8_t               notifyLen;
} bsOta = { OTA_LOCKED, 0, nullptr, 0, 0, false, {0}, 0 };

static void bsOtaQueueNotify(uint8_t status) {
  bsOta.notifyBuf[0] = status;
  bsOta.notifyLen = 1;
  bsOta.notifyPending = true;
}

static void otaHandleChunk(uint32_t offset, const uint8_t* data, size_t len) {
  if (bsOta.state == OTA_VERIFYING) {
    bsOtaQueueNotify(OTA_STATUS_VERIFYING); return;
  }
  if (bsOta.state != OTA_RECEIVING) {
    bsOtaQueueNotify(OTA_STATUS_NOT_ACTIVE); return;
  }
  if (len == 0) { bsOtaQueueNotify(OTA_STATUS_NOT_ACTIVE); return; }
  if (offset != bsOta.bytesWritten) {
    bsOtaQueueNotify(OTA_STATUS_OFFSET_GAP); return;
  }
  if (esp_ota_write(bsOta.handle, data, len) != ESP_OK) {
    esp_ota_abort(bsOta.handle);
    bsOta.state = OTA_LOCKED;
    bsOtaQueueNotify(OTA_STATUS_WRITE_FAIL); return;
  }
  bsOta.bytesWritten += (uint32_t)len;
  bsOta.chunkCount++;
  if (bsOta.chunkCount % OTA_PROGRESS_INTERVAL == 0) {
    bsOta.notifyBuf[0] = OTA_PROGRESS_MARKER;
    bsOta.notifyBuf[1] = (uint8_t)(bsOta.bytesWritten);
    bsOta.notifyBuf[2] = (uint8_t)(bsOta.bytesWritten >> 8);
    bsOta.notifyBuf[3] = (uint8_t)(bsOta.bytesWritten >> 16);
    bsOta.notifyBuf[4] = (uint8_t)(bsOta.bytesWritten >> 24);
    bsOta.notifyLen = 5;
    bsOta.notifyPending = true;
  }
}

static uint8_t otaHandleBegin() {
  {
    esp_ota_img_states_t imgState;
    if (esp_ota_get_state_partition(esp_ota_get_running_partition(), &imgState) == ESP_OK
        && imgState == ESP_OTA_IMG_PENDING_VERIFY) {
      Serial.println("OTA: begin refused — boot pending confirmation (send 0x52 first)");
      return 0x07;
    }
  }
  if (bsOta.state != OTA_LOCKED) {
    Serial.println("OTA: begin refused — already active");
    return 0x02;
  }
  const esp_partition_t* part = esp_ota_get_next_update_partition(NULL);
  if (!part) { Serial.println("OTA: no update partition"); return 0x02; }
  bsOta.state = OTA_ERASING;
  bsOta.partition = part;
  Serial.printf("OTA: erasing partition '%s'...\n", part->label);
  for (uint32_t off = 0; off < part->size; off += 4096) {
    if (esp_partition_erase_range(part, off, 4096) != ESP_OK) {
      bsOta.state = OTA_LOCKED;
      return 0x02;
    }
    vTaskDelay(1);
  }
  Serial.println("OTA: erase done");
  esp_ota_handle_t handle;
  if (esp_ota_begin(part, part->size, &handle) != ESP_OK) {
    bsOta.state = OTA_LOCKED;
    return 0x02;
  }
  bsOta.handle = handle;
  bsOta.bytesWritten = 0;
  bsOta.chunkCount = 0;
  bsOta.state = OTA_RECEIVING;
  Serial.println("OTA: session open");
  return 0x00;
}

static uint8_t otaHandleFinalize(uint32_t expectedSize, const uint8_t* firmwareHmac) {
  if (bsOta.state != OTA_RECEIVING) return 0x02;
  bsOta.state = OTA_VERIFYING;
  if (bsOta.bytesWritten != expectedSize) {
    esp_ota_abort(bsOta.handle);
    bsOta.state = OTA_LOCKED;
    return 0x03;
  }
  uint8_t computed[32];
  mbedtls_md_context_t ctx;
  mbedtls_md_init(&ctx);
  mbedtls_md_setup(&ctx, mbedtls_md_info_from_type(MBEDTLS_MD_SHA256), 1);
  mbedtls_md_hmac_starts(&ctx, HMAC_KEY, HMAC_KEY_LEN);
  uint8_t rbuf[256]; uint32_t pos = 0, rem = expectedSize; bool ok = true;
  while (rem > 0) {
    uint32_t n = rem < sizeof(rbuf) ? rem : (uint32_t)sizeof(rbuf);
    if (esp_partition_read(bsOta.partition, pos, rbuf, n) != ESP_OK) { ok = false; break; }
    mbedtls_md_hmac_update(&ctx, rbuf, n);
    pos += n; rem -= n;
    vTaskDelay(0);
  }
  mbedtls_md_hmac_finish(&ctx, computed);
  mbedtls_md_free(&ctx);
  if (!ok) { esp_ota_abort(bsOta.handle); bsOta.state = OTA_LOCKED; return 0x02; }
  uint8_t diff = 0;
  for (int i = 0; i < 32; i++) diff |= computed[i] ^ firmwareHmac[i];
  if (diff != 0) {
    Serial.println("OTA: HMAC mismatch");
    esp_ota_abort(bsOta.handle); bsOta.state = OTA_LOCKED;
    bsOtaQueueNotify(OTA_STATUS_HMAC_MISMATCH);
    return 0x02;
  }
  if (esp_ota_end(bsOta.handle) != ESP_OK) {
    bsOta.state = OTA_LOCKED;
    bsOtaQueueNotify(OTA_STATUS_OTA_END_FAIL);
    return 0x02;
  }
  if (esp_ota_set_boot_partition(bsOta.partition) != ESP_OK) {
    bsOta.state = OTA_LOCKED; return 0x02;
  }
  Serial.println("OTA: success — rebooting");
  bsOtaQueueNotify(OTA_STATUS_OK);
  delay(500);
  esp_restart();
  return 0x00;
}

static uint8_t otaHandleConfirm() {
  if (esp_ota_mark_app_valid_cancel_rollback() != ESP_OK) return 0x02;
  Serial.println("OTA: confirmed, rollback cancelled");
  return 0x00;
}

// ===================== BLE log fetch state machine — runs in main loop
struct BleLogFetch {
  volatile bool active;  // volatile: written by NimBLE task, read by loopTask
  uint32_t startRec;
  uint32_t endRec;      // exclusive
  uint32_t currentRec;
  LogStore::SeqReader seq;
  // Flow control: hold last built chunk until notify() succeeds. If notify
  // returns false (host queue full) we retry the same bytes next loop instead
  // of advancing the cursor — prevents holes in the fetched range.
  uint8_t  pendingBuf[514];  // ATT MTU 517 - 3-byte header = 514 max notify payload
  uint16_t pendingLen;  // 0 = no chunk pending
  bool     endPending;  // 0-byte end marker not yet sent
  // Diagnostics — reset on each new request
  uint32_t notifyOk;
  uint32_t notifyDrop;
  uint32_t bytesSent;
} bleLogFetch = {false, 0, 0, 0, {}, {}, 0, false, 0, 0, 0};

// Forward declaration — defined below, used in dispatchCmdTx

// ===================== TRANSPORT-AGNOSTIC COMMAND TX =====================

struct CmdTxState {
  uint8_t pkt[64];
  uint8_t pktLen;
  uint8_t sends;
  uint8_t sent;
  uint16_t waitMs;       // max ms to wait for next WIN_CMD before sending out of turn
  unsigned long lastSendMs;
  unsigned long queuedMs; // millis() when command was queued (for wait expiry)
  bool active;
} cmdTx = {.pktLen=0,.sends=0,.sent=0,.waitMs=0,.lastSendMs=0,.queuedMs=0,.active=false};

bool verifyCommandHMAC(const uint8_t* pkt, size_t pktLen) {
  if (pktLen < HMAC_TRUNC_LEN + 7) return false;
  size_t dataLen = pktLen - HMAC_TRUNC_LEN;
  uint8_t fullHmac[32];
  mbedtls_md_context_t ctx;
  mbedtls_md_init(&ctx);
  mbedtls_md_setup(&ctx, mbedtls_md_info_from_type(MBEDTLS_MD_SHA256), 1);
  mbedtls_md_hmac_starts(&ctx, HMAC_KEY, HMAC_KEY_LEN);
  mbedtls_md_hmac_update(&ctx, pkt, dataLen);
  mbedtls_md_hmac_finish(&ctx, fullHmac);
  mbedtls_md_free(&ctx);
  uint8_t diff = 0;
  for (int i = 0; i < HMAC_TRUNC_LEN; i++) diff |= fullHmac[i] ^ pkt[dataLen + i];
  return (diff == 0);
}

// Returns true on success. On failure, errorMsg is set.
bool queueCommandTx(const uint8_t* body, size_t bodyLen, String& errorMsg) {
  if (bodyLen < 5) { errorMsg = "too short"; return false; }

  uint16_t waitMs = body[0] | ((uint16_t)body[1] << 8);
  uint8_t sends = body[2];
  uint8_t pktLen = body[3];
  if (pktLen == 0 || pktLen > 60 || (size_t)(4 + pktLen) > bodyLen) {
    errorMsg = "bad len"; return false;
  }
  if (sends == 0 || sends > 20) sends = 1;

  // Stage into a local buffer so a failed verify can't corrupt an in-flight
  // valid cmdTx that's still being retransmitted (cmdTx.active=true,
  // sent<sends). Only commit to cmdTx after HMAC + nonce checks pass.
  uint8_t staged[sizeof(cmdTx.pkt)];
  memcpy(staged, body + 4, pktLen);

  if (!verifyCommandHMAC(staged, pktLen)) {
    Serial.println("CMD TX rejected: HMAC fail");
    errorMsg = "hmac fail"; return false;
  }

  if (pktLen >= 7) {
    uint32_t pktNonce = (uint32_t)staged[3] | ((uint32_t)staged[4] << 8) |
                        ((uint32_t)staged[5] << 16) | ((uint32_t)staged[6] << 24);
    if (pktNonce <= highestNonce) {
      Serial.println("CMD TX rejected: stale nonce");
      errorMsg = "stale nonce"; return false;
    }
    highestNonce = pktNonce;
    bsNvs.putUInt("nonce", highestNonce);
  }

  // Commit. cmdTx.active is set last so dispatchCmdTx won't observe a
  // half-built state. (Single-task assumption between dispatch and queue;
  // BLE/HTTP callbacks running concurrently with main-loop dispatch get a
  // brief window between the field writes and active=true — they read
  // !active and skip, which is fine.)
  memcpy(cmdTx.pkt, staged, pktLen);
  cmdTx.pktLen = pktLen;
  cmdTx.sends = sends;
  cmdTx.sent = 0;
  cmdTx.waitMs = waitMs;
  cmdTx.lastSendMs = 0;
  cmdTx.queuedMs = millis();
  cmdTx.active = true;

  Serial.print("CMD TX queued: "); Serial.print(pktLen);
  Serial.print("B x"); Serial.print(sends);
  Serial.print(" wait="); Serial.println(waitMs);

  errorMsg = ""; return true;
}

// ===================== PUSH TELEMETRY TO ALL TRANSPORTS =====================

struct {
  uint8_t data[256]; size_t len; int8_t snr4; uint32_t timestamp; bool valid;
} latestTelem = {.len=0,.snr4=0,.timestamp=0,.valid=false};

void pushToAllTransports(const uint8_t* wsBuf, size_t wsLen) {
  if (wifiEnabled) {
    ws.binaryAll(wsBuf, wsLen);
  }
  if (bleEnabled && bleClientConnected && bleTelemChar) {
    bleTelemChar->notify((uint8_t*)wsBuf, wsLen);
  }
}

// ===================== PACKET RECEIVED CALLBACK =====================
// Invoked from radio.cpp bsHandleRxDone() for every valid received packet.

static void processBaseLoraCommand(const uint8_t* pkt, size_t pktLen);  // fwd decl

void bsOnPacketReceived(const uint8_t* buf, size_t len, float snrF, float rssiF) {
  // Wire/log format only: log_store records snr in dB*4 (spec).
  int8_t snr4 = (int8_t)constrain((int)(snrF * 4.0f), -128, 127);
  uint32_t nowMs = millis();

  int32_t recNum = -1;
  if (logStoreOk) recNum = logStore.writeRecord(buf, (uint8_t)len, snr4, nowMs);

  if (len >= 10 && buf[0] == 0xAF) {
    memcpy(latestTelem.data, buf, len);
    latestTelem.len = len; latestTelem.snr4 = snr4;
    latestTelem.timestamp = nowMs; latestTelem.valid = true;
  }

  // LoRa-received command targeted at this base station — relay-mode entry
  // point. Verifies HMAC + nonce, then runs base-local handler.
  if (len >= HMAC_TRUNC_LEN + 7 && buf[0] == PKT_COMMAND && buf[1] == DEVICE_ID) {
    processBaseLoraCommand(buf, len);
  }

  if (len >= 5 && buf[0] == PKT_LONGRANGE) {
    uint32_t word   = (uint32_t)buf[2] | ((uint32_t)buf[3] << 8) | ((uint32_t)buf[4] << 16);
    uint16_t latFrac = (uint16_t)(word & 0x7FF);
    uint16_t lonFrac = (uint16_t)((word >> 11) & 0x7FF);
    bool     lowBatt = (word >> 22) & 1;
    bool     gpsErr  = (latFrac >= 2000 || lonFrac >= 2000);
    Serial.print("LR packet: lat="); Serial.print(gpsErr ? -1 : (int)latFrac);
    Serial.print(" lon="); Serial.print(gpsErr ? -1 : (int)lonFrac);
    Serial.print(" lowbatt="); Serial.println(lowBatt ? "YES" : "NO");
  }

  uint8_t wsBuf[268];
  memcpy(wsBuf, &snrF, 4);
  memcpy(wsBuf + 4, &rssiF, 4);
  memcpy(wsBuf + 8, &recNum, 4);
  memcpy(wsBuf + 12, buf, len);
  pushToAllTransports(wsBuf, 12 + len);

  Serial.printf("BS RX: Sig:%.1f/%.0f #%d %dB: [", snrF, rssiF, recNum, (int)len);
  size_t hexLen = (len < 14) ? len : 14;
  for (size_t i = 0; i < hexLen; i++) Serial.printf("%02X", buf[i]);
  if (hexLen < len) Serial.print("...");
  Serial.println("]");
}

// ===================== PING PACKET BUILDER =====================
// Builds a CMD_PING (0x40) command packet signed with HMAC.
// Returns packet length (always 17).

size_t bsBuildPingCmdPacket(uint8_t* buf) {
  highestNonce++;
  bsNvs.putUInt("nonce", highestNonce);

  buf[0] = 0x9A;               // PKT_COMMAND
  buf[1] = FAVORITE_ROCKET_DEVICE_ID;
  buf[2] = 0x40;               // CMD_PING
  buf[3] = (uint8_t)(highestNonce);
  buf[4] = (uint8_t)(highestNonce >> 8);
  buf[5] = (uint8_t)(highestNonce >> 16);
  buf[6] = (uint8_t)(highestNonce >> 24);

  uint8_t fullHmac[32];
  mbedtls_md_context_t ctx;
  mbedtls_md_init(&ctx);
  mbedtls_md_setup(&ctx, mbedtls_md_info_from_type(MBEDTLS_MD_SHA256), 1);
  mbedtls_md_hmac_starts(&ctx, HMAC_KEY, HMAC_KEY_LEN);
  mbedtls_md_hmac_update(&ctx, buf, 7);
  mbedtls_md_hmac_finish(&ctx, fullHmac);
  mbedtls_md_free(&ctx);
  memcpy(buf + 7, fullHmac, HMAC_TRUNC_LEN);

  return 17;
}

// ===================== LOCAL COMMAND EXECUTION =====================
// Base-station-targeted commands (target byte == DEVICE_ID) run here.
// Caller is responsible for HMAC + nonce verification — this function only
// dispatches. Intended to be called from BOTH the post-TX local-apply path
// (when the operator queues a base-targeted command via web/BLE/HTTP, the
// HMAC is already verified by queueCommandTx) AND the LoRa-RX path (when
// a remote operator-station relays a base-targeted command — verified by
// processBaseLoraCommand below).
// Returns true if the command was a base-local one and was handled.
static bool tryExecuteLocalCommand(const uint8_t* pkt, size_t pktLen) {
  if (pktLen < 7) return false;
  if (pkt[1] != DEVICE_ID) return false;
  uint8_t cmdId = pkt[2];

  switch (cmdId) {
    case CMD_OTA_BEGIN:
      otaHandleBegin();
      return true;

    case CMD_OTA_FINALIZE: {
      if (pktLen < 7 + 36 + HMAC_TRUNC_LEN) return false;
      uint32_t fwSize = (uint32_t)pkt[7] | ((uint32_t)pkt[8] << 8)
                      | ((uint32_t)pkt[9] << 16) | ((uint32_t)pkt[10] << 24);
      const uint8_t* fwHmac = &pkt[11];
      otaHandleFinalize(fwSize, fwHmac);
      return true;
    }

    case CMD_OTA_CONFIRM:
      otaHandleConfirm();
      return true;

    case 0x30: {  // SET RELAY RADIO
      if (pktLen < 23) return false;
      uint8_t priCh  = pkt[7];
      uint8_t priSf  = pkt[8];
      int8_t  priPwr = (int8_t)pkt[9];
      uint8_t bhCh_  = pkt[10];
      uint8_t bhSf_  = pkt[11];
      int8_t  bhPwr_ = (int8_t)pkt[12];

      float priFreq = channelToFreqMHz(priCh);
      if (priFreq != 0.0f && priSf >= 5 && priSf <= 12 && priPwr >= -9 && priPwr <= 22) {
        activeChannel = priCh; activeSF = priSf; activePower = priPwr;
        bsUpdateActiveFreqBw();
        bsNvs.putUChar("radio_ch", activeChannel);
        bsNvs.putUChar("radio_sf", activeSF);
        bsNvs.putChar("radio_pwr", activePower);
      }
      float bhFreq_ = channelToFreqMHz(bhCh_);
      if (bhFreq_ != 0.0f && bhSf_ >= 5 && bhSf_ <= 12 && bhPwr_ >= -9 && bhPwr_ <= 22) {
        bhChannel = bhCh_; bhSF = bhSf_; bhPower = bhPwr_;
        bsNvs.putUChar("bh_ch", bhChannel);
        bsNvs.putUChar("bh_sf", bhSF);
        bsNvs.putChar("bh_pwr", bhPower);
      }
      bsRadioApplyConfig();
      return true;
    }

    case CMD_LR_LISTEN: {
      // params: uint16 durationMs, little-endian. 0 = cancel.
      if (pktLen < 7 + 2 + HMAC_TRUNC_LEN) return false;
      uint16_t durationMs = (uint16_t)pkt[7] | ((uint16_t)pkt[8] << 8);
      bsRadioEnterLRListen(durationMs);
      return true;
    }

    case 0x31: {  // ENABLE TRANSPORT — params: uint8 flags (TRANSPORT_WIFI=0x01, TRANSPORT_BLE=0x02)
      if (pktLen < 7 + 1 + HMAC_TRUNC_LEN) return false;
      uint8_t flags = pkt[7];
      if (flags & TRANSPORT_WIFI) enableWifi();
      // BLE disable/enable not supported at runtime — always on
      return true;
    }

    case 0x32: {  // DISABLE TRANSPORT — params: uint8 flags
      if (pktLen < 7 + 1 + HMAC_TRUNC_LEN) return false;
      uint8_t flags = pkt[7];
      if (flags & TRANSPORT_WIFI) disableWifi();
      return true;
    }

    default:
      return false;
  }
}

// Verify HMAC + nonce for a LoRa-received command targeting this base, then
// run tryExecuteLocalCommand. Used to let other base stations (or in future,
// any HMAC-holding device) trigger base-local actions over the air.
static void processBaseLoraCommand(const uint8_t* pkt, size_t pktLen) {
  if (pktLen < HMAC_TRUNC_LEN + 7) return;
  if (pkt[0] != PKT_COMMAND) return;
  if (pkt[1] != DEVICE_ID) return;       // not for us
  if (!verifyCommandHMAC(pkt, pktLen)) {
    Serial.println("BS LoRa CMD: HMAC fail");
    return;
  }
  uint32_t nonce = (uint32_t)pkt[3] | ((uint32_t)pkt[4] << 8)
                 | ((uint32_t)pkt[5] << 16) | ((uint32_t)pkt[6] << 24);
  if (nonce <= highestNonce) {
    Serial.println("BS LoRa CMD: stale nonce");
    return;
  }
  highestNonce = nonce;
  bsNvs.putUInt("nonce", highestNonce);
  Serial.print("BS LoRa CMD: 0x"); Serial.print(pkt[2], HEX);
  Serial.print(" nonce="); Serial.println(nonce);
  if (!tryExecuteLocalCommand(pkt, pktLen)) {
    Serial.println("BS LoRa CMD: not a base-local command");
  }
}

// ===================== CMD TX DISPATCH =====================
// First send waits for the post-RxDone window (cleanest air just opened) or
// for `waitMs` since queueing to elapse, whichever comes first. Subsequent
// sends in a burst (sends>1, used when struggling to find the rocket's RX
// window) blast back-to-back with no further waiting.

#define BS_POST_RX_WINDOW_MS  30      // open a TX window for this long after each telem RxDone
#define BS_TX_OVERRUN_MS      2000    // after this long blocked by busy RX, force through

static void dispatchCmdTx() {
  if (!cmdTx.active || cmdTx.sent >= cmdTx.sends) return;
  if (bsRadioState != BS_RADIO_RX) return;       // radio mid-readData / TX in flight

  unsigned long now = millis();
  unsigned long age = now - cmdTx.queuedMs;

  // Reception-preserving: defer if a packet is currently arriving. But after
  // BS_TX_OVERRUN_MS past queue time, force through — someone may be on a
  // long continuous transmission and we'd never get on air.
  bool rxBusy  = bsRadioRxBusy();
  bool overrun = (age > BS_TX_OVERRUN_MS);
  if (rxBusy && !overrun) return;
  if (rxBusy && overrun) {
    Serial.println("CMD TX: overrun, forcing through busy RX");
  }

  // Gate the FIRST send on post-RX window or waitMs expiry. Retries blast.
  if (cmdTx.sent == 0 && !overrun) {
    bool inPostRxWindow = (bsLastTelemRxMs != 0) &&
                          ((now - bsLastTelemRxMs) <= BS_POST_RX_WINDOW_MS);
    bool waitExpired    = (cmdTx.waitMs == 0) ||
                          (age >= cmdTx.waitMs);
    if (!inPostRxWindow && !waitExpired) return;
  }

  Serial.print("CMD TX "); Serial.print(cmdTx.sent + 1);
  Serial.print("/"); Serial.print(cmdTx.sends);
  Serial.print(" len="); Serial.println(cmdTx.pktLen);

  if (!bsRadioStartTx(cmdTx.pkt, cmdTx.pktLen, /*forceThroughBusy=*/overrun)) {
    Serial.println("CMD TX start fail — retry next loop");
    return;
  }

  cmdTx.sent++;
  cmdTx.lastSendMs = millis();

  if (cmdTx.sent >= cmdTx.sends) {
    cmdTx.active = false;
    Serial.print("CMD TX complete: "); Serial.print(cmdTx.sends); Serial.println(" sent");
    if (logStoreOk) logStore.writeRecord(cmdTx.pkt, cmdTx.pktLen, 0x7F, millis());

    // Base-targeted command (queued via web/BLE/HTTP, HMAC already verified
    // in queueCommandTx). Run any local effect after the on-air relay.
    tryExecuteLocalCommand(cmdTx.pkt, cmdTx.pktLen);
  }
}

// ===================== BUILD STATUS JSON =====================

size_t buildStatusJson(char* json, size_t maxLen) {
  readBaseBattery();
  return snprintf(json, maxLen,
    "{\"uptimeMs\":%lu,\"records\":%lu,\"oldest\":%lu,\"ringSize\":%lu,\"vpos\":%lu,"
    "\"logOk\":%s,\"nonce\":%lu,\"deviceId\":%d,\"baseBattMv\":%u,"
    "\"wifiOn\":%s,\"bleOn\":%s,\"lrListen\":%s,"
    "\"radioCh\":%u,\"radioSF\":%u,\"radioPwr\":%d}",
    (unsigned long)millis(),
    (unsigned long)logStore.getRecordCounter(),
    (unsigned long)logStore.getOldestRecord(),
    (unsigned long)logStore.getRingSize(),
    (unsigned long)logStore.getVirtualPos(),
    logStoreOk ? "true" : "false",
    (unsigned long)highestNonce,
    DEVICE_ID,
    (unsigned)baseBattMv,
    wifiEnabled ? "true" : "false",
    bleEnabled ? "true" : "false",
    bsRadioInLRListen() ? "true" : "false",
    (unsigned)activeChannel,
    (unsigned)activeSF,
    (int)activePower);
}

// ===================== HTTP HANDLERS =====================

void handleApiCommand(AsyncWebServerRequest *req, uint8_t *data, size_t len, size_t index, size_t total) {
  if (index != 0) return;
  if (len < total) { req->send(400, "text/plain", "fragmented"); return; }
  String errorMsg;
  if (queueCommandTx(data, len, errorMsg)) {
    req->send(200, "text/plain", "ok");
  } else {
    int code = (errorMsg == "hmac fail" || errorMsg == "stale nonce") ? 403 : 400;
    req->send(code, "text/plain", errorMsg.c_str());
  }
}

void handleApiOtaChunk(AsyncWebServerRequest *req, uint8_t *data, size_t len, size_t index, size_t total) {
  if (index != 0) return;
  if (len < total) { req->send(400, "text/plain", "fragmented"); return; }
  if (len < 5) { req->send(400, "text/plain", "too short"); return; }
  uint32_t offset = (uint32_t)data[0] | ((uint32_t)data[1] << 8)
                  | ((uint32_t)data[2] << 16) | ((uint32_t)data[3] << 24);
  bsOta.notifyPending = false;
  otaHandleChunk(offset, data + 4, len - 4);
  uint8_t status = bsOta.notifyPending ? bsOta.notifyBuf[0] : OTA_STATUS_OK;
  bsOta.notifyPending = false;
  req->send(200, "application/octet-stream", String((char)status));
}

void handleApiStatus(AsyncWebServerRequest *req) {
  char json[512];
  buildStatusJson(json, sizeof(json));
  req->send(200, "application/json", json);
}

void handleApiLogs(AsyncWebServerRequest *req) {
  if (!logStoreOk) { req->send(503, "text/plain", "no log"); return; }
  uint32_t startRec = 0, count = 100;
  if (req->hasParam("start")) startRec = req->getParam("start")->value().toInt();
  if (req->hasParam("count")) { count = req->getParam("count")->value().toInt(); if (count > 200) count = 200; }

  uint32_t oldest = logStore.getOldestRecord();
  uint32_t newest = logStore.getRecordCounter();
  if (startRec < oldest) startRec = oldest;
  if (startRec >= newest || newest == 0) { req->send(200, "application/octet-stream", ""); return; }
  if (startRec + count > newest) count = newest - startRec;

  size_t maxBuf = count * (10 + LOG_MAX_PAYLOAD);
  if (maxBuf > 60000) maxBuf = 60000;
  uint8_t* outBuf = (uint8_t*)malloc(maxBuf);
  if (!outBuf) { req->send(503, "text/plain", "OOM"); return; }

  size_t outPos = 0; uint8_t recBuf[LOG_MAX_PAYLOAD]; int8_t snr; uint32_t ts;
  LogStore::SeqReader seq = logStore.seqReader(startRec, startRec + count);
  while (seq.hasMore() && outPos + 10 + LOG_MAX_PAYLOAD <= maxBuf) {
    uint32_t rn = seq.currentRec();
    int pLen = seq.readNext(recBuf, sizeof(recBuf), &snr, &ts);
    if (pLen < 0) break;
    memcpy(outBuf+outPos, &rn, 4); outPos += 4;
    outBuf[outPos++] = (uint8_t)pLen;
    outBuf[outPos++] = (uint8_t)snr;
    memcpy(outBuf+outPos, &ts, 4); outPos += 4;
    memcpy(outBuf+outPos, recBuf, pLen); outPos += pLen;
  }
  AsyncWebServerResponse *resp = req->beginResponse_P(200, "application/octet-stream", outBuf, outPos);
  resp->addHeader("Cache-Control", "no-cache");
  req->send(resp);
  free(outBuf);
}

void onWsEvent(AsyncWebSocket *s, AsyncWebSocketClient *c, AwsEventType t, void *a, uint8_t *d, size_t l) {
  if (t == WS_EVT_CONNECT) { Serial.print("WS+ #"); Serial.println(c->id()); }
  else if (t == WS_EVT_DISCONNECT) { Serial.print("WS- #"); Serial.println(c->id()); }
  else { Serial.print("WS evt="); Serial.println((int)t); }
}

// ===================== BLE CALLBACKS =====================

class BleServerCallbacks : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer* server, NimBLEConnInfo& connInfo) override {
    bleClientConnected = true;
    Serial.print("BLE+ addr:"); Serial.println(connInfo.getAddress().toString().c_str());
    //TODO: @@@ force coded phy s=8
  }
  void onDisconnect(NimBLEServer* server, NimBLEConnInfo& connInfo, int reason) override {
    bleClientConnected = false;
    bleLogFetch.active = false;
    Serial.print("BLE- reason:"); Serial.println(reason);
    NimBLEDevice::startAdvertising(0);
    NimBLEDevice::startAdvertising(1);
  }
};

class BleCmdCallbacks : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic* chr, NimBLEConnInfo& connInfo) override {
    NimBLEAttValue val = chr->getValue();
    if (val.size() < 5) {
      uint8_t err = 0x01;
      chr->setValue(&err, 1);
      return;
    }
    String errorMsg;
    bool ok = queueCommandTx(val.data(), val.size(), errorMsg);
    uint8_t result = ok ? 0x00 : 0x02;
    chr->setValue(&result, 1);
    Serial.print("BLE CMD "); Serial.println(ok ? "OK" : errorMsg.c_str());
  }
};

class BleStatusCallbacks : public NimBLECharacteristicCallbacks {
  void onRead(NimBLECharacteristic* chr, NimBLEConnInfo& connInfo) override {
    char json[512];
    buildStatusJson(json, sizeof(json));
    chr->setValue((uint8_t*)json, strlen(json));
  }
};

class BleLogFetchCallbacks : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic* chr, NimBLEConnInfo& connInfo) override {
    NimBLEAttValue val = chr->getValue();
    Serial.printf("[BLEFetch] onWrite size=%u logStoreOk=%d active=%d\n",
      (unsigned)val.size(), logStoreOk?1:0, bleLogFetch.active?1:0);
    if (val.size() < 6 || !logStoreOk) {
      Serial.println("[BLEFetch] onWrite ignored (bad size or no log store)");
      return;
    }

    uint32_t startRec = val.data()[0] | ((uint32_t)val.data()[1] << 8) |
                        ((uint32_t)val.data()[2] << 16) | ((uint32_t)val.data()[3] << 24);
    uint16_t count = val.data()[4] | ((uint16_t)val.data()[5] << 8);
    if (count > 2000) count = 2000;

    uint32_t oldest = logStore.getOldestRecord();
    uint32_t newest = logStore.getRecordCounter();
    Serial.printf("[BLEFetch] req start=%lu count=%u | store oldest=%lu newest=%lu\n",
      (unsigned long)startRec, (unsigned)count, (unsigned long)oldest, (unsigned long)newest);
    if (startRec < oldest) startRec = oldest;
    if (startRec >= newest) {
      Serial.println("[BLEFetch] start >= newest — sending immediate end marker");
      bool ok = chr->notify((uint8_t*)"\xFF", 1, true);
      Serial.printf("[BLEFetch] empty notify ok=%d\n", ok?1:0);
      return;
    }
    uint32_t endRec = startRec + count;
    if (endRec > newest) endRec = newest;

    // Disable in-flight fetch so loopTask can't observe a half-built state while
    // we re-init seq below.
    bleLogFetch.active = false;
    bleLogFetch.startRec = startRec;
    bleLogFetch.endRec = endRec;
    bleLogFetch.currentRec = startRec;
    bleLogFetch.seq = logStore.seqReader(startRec, endRec);
    bleLogFetch.pendingLen = 0;
    bleLogFetch.endPending = false;
    bleLogFetch.notifyOk = 0;
    bleLogFetch.notifyDrop = 0;
    bleLogFetch.bytesSent = 0;

    Serial.printf("[BLEFetch] BLE fetch: %lu-%lu — going active\n",
      (unsigned long)startRec, (unsigned long)endRec);
    // Set active LAST so loopTask sees a fully-initialized state.
    bleLogFetch.active = true;
  }
};

class BleOtaCallbacks : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic* chr, NimBLEConnInfo& connInfo) override {
    NimBLEAttValue val = chr->getValue();
    if (val.size() < 5) return;
    uint32_t offset = (uint32_t)val.data()[0] | ((uint32_t)val.data()[1] << 8)
                    | ((uint32_t)val.data()[2] << 16) | ((uint32_t)val.data()[3] << 24);
    otaHandleChunk(offset, val.data() + 4, val.size() - 4);
  }
};

// ===================== BLE LOG FETCH STATE MACHINE =====================

void handleBleLogFetch() {
  if (!bleLogFetch.active || !bleClientConnected || !bleLogFetchChar) return;

  // Retry held chunk — flow control is the notify() return value, not a timer.
  // We yield() on a drop so the BLE host task can drain its TX queue; this is
  // cheap and doesn't gate throughput when the queue isn't full.
  if (bleLogFetch.pendingLen > 0) {
    if (!bleLogFetchChar->notify(bleLogFetch.pendingBuf, bleLogFetch.pendingLen, true)) {
      bleLogFetch.notifyDrop++;
      yield();
      return;
    }
    bleLogFetch.notifyOk++;
    bleLogFetch.bytesSent += bleLogFetch.pendingLen;
    bleLogFetch.pendingLen = 0;
  }

  // Retry the end-of-fetch marker if previously dropped.
  if (bleLogFetch.endPending) {
    if (!bleLogFetchChar->notify((uint8_t*)"\xFF", 1, true)) { yield(); return; }
    bleLogFetch.endPending = false;
    bleLogFetch.active = false;
    Serial.printf("[BLEFetch] done @%lu notifyOk=%lu drop=%lu bytes=%lu (endPending path)\n",
      (unsigned long)bleLogFetch.currentRec, (unsigned long)bleLogFetch.notifyOk,
      (unsigned long)bleLogFetch.notifyDrop, (unsigned long)bleLogFetch.bytesSent);
    return;
  }

  if (!bleLogFetch.seq.hasMore()) {
    if (!bleLogFetchChar->notify((uint8_t*)"\xFF", 1, true)) {
      bleLogFetch.endPending = true;
      Serial.println("[BLEFetch] end marker dropped — will retry");
      return;
    }
    bleLogFetch.active = false;
    Serial.printf("[BLEFetch] done @%lu notifyOk=%lu drop=%lu bytes=%lu\n",
      (unsigned long)bleLogFetch.currentRec, (unsigned long)bleLogFetch.notifyOk,
      (unsigned long)bleLogFetch.notifyDrop, (unsigned long)bleLogFetch.bytesSent);
    return;
  }

  // Build a chunk directly into pendingBuf so we can retransmit on congestion.
  uint16_t chunkPos = 0;
  uint8_t recBuf[LOG_MAX_PAYLOAD];
  int8_t snr;
  uint32_t ts;

  while (bleLogFetch.seq.hasMore() && chunkPos + 10 + LOG_MAX_PAYLOAD <= sizeof(bleLogFetch.pendingBuf)) {
    uint32_t rn = bleLogFetch.seq.currentRec();
    int pLen = bleLogFetch.seq.readNext(recBuf, sizeof(recBuf), &snr, &ts);
    if (pLen < 0) break;

    memcpy(bleLogFetch.pendingBuf + chunkPos, &rn, 4); chunkPos += 4;
    bleLogFetch.pendingBuf[chunkPos++] = (uint8_t)pLen;
    bleLogFetch.pendingBuf[chunkPos++] = (uint8_t)snr;
    memcpy(bleLogFetch.pendingBuf + chunkPos, &ts, 4); chunkPos += 4;
    memcpy(bleLogFetch.pendingBuf + chunkPos, recBuf, pLen); chunkPos += pLen;
    bleLogFetch.currentRec = bleLogFetch.seq.currentRec();
  }

  if (chunkPos > 0) {
    if (!bleLogFetchChar->notify(bleLogFetch.pendingBuf, chunkPos, true)) {
      bleLogFetch.pendingLen = chunkPos;  // hold for retry
      bleLogFetch.notifyDrop++;
      yield();
    } else {
      bleLogFetch.notifyOk++;
      bleLogFetch.bytesSent += chunkPos;
#if BLE_FETCH_VERBOSE
      Serial.printf("[BLEFetch] notify ok #%lu len=%u curRec=%lu bytesTot=%lu\n",
        (unsigned long)bleLogFetch.notifyOk, (unsigned)chunkPos,
        (unsigned long)bleLogFetch.currentRec, (unsigned long)bleLogFetch.bytesSent);
#endif
    }
  } else {
    // hasMore() said true but built 0 bytes — readNext returned <0 immediately.
    // Force the next iteration to take the seq.hasMore()==false branch.
    Serial.println("[BLEFetch] WARN built empty chunk — readNext failed");
  }
}

// ===================== BLE INIT / DEINIT =====================

void initBLE() {
  NimBLEDevice::init(BLE_DEVICE_NAME);
  NimBLEDevice::setPower(BLUETOOTH_POWER);
  NimBLEDevice::setMTU(517);

  bleServer = NimBLEDevice::createServer();
  bleServer->setCallbacks(new BleServerCallbacks());

  NimBLEService* svc = bleServer->createService(BLE_SERVICE_UUID);

  bleTelemChar = svc->createCharacteristic(BLE_TELEM_CHAR_UUID, NIMBLE_PROPERTY::NOTIFY);
  bleCmdChar = svc->createCharacteristic(BLE_CMD_CHAR_UUID, NIMBLE_PROPERTY::WRITE);
  bleCmdChar->setCallbacks(new BleCmdCallbacks());
  bleStatusChar = svc->createCharacteristic(BLE_STATUS_CHAR_UUID, NIMBLE_PROPERTY::READ);
  bleStatusChar->setCallbacks(new BleStatusCallbacks());
  bleLogFetchChar = svc->createCharacteristic(BLE_LOGFETCH_CHAR_UUID,
    NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::NOTIFY);
  bleLogFetchChar->setCallbacks(new BleLogFetchCallbacks());

  bleOtaChar = svc->createCharacteristic(BLE_OTA_CHAR_UUID,
    NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR | NIMBLE_PROPERTY::NOTIFY);
  bleOtaChar->setCallbacks(new BleOtaCallbacks());

  svc->start();

  // Instance 0: legacy PDU for Chrome/Web Bluetooth.
  // Payload hand-built to exactly 31 bytes:
  //   Flags (3) + Shortened Name "MoonBase"/8 chars (10) + UUID128 (18) = 31
  {
    NimBLEExtAdvertisement legacyAdv;
    legacyAdv.setLegacyAdvertising(true);
    legacyAdv.setConnectable(true);
    legacyAdv.setPrimaryPhy(BLE_HCI_LE_PHY_1M);
    legacyAdv.setSecondaryPhy(BLE_HCI_LE_PHY_1M);
    legacyAdv.setMinInterval(BLE_ADV_INTERVAL_MIN);
    legacyAdv.setMaxInterval(BLE_ADV_INTERVAL_MAX);

    static_assert(sizeof(BLE_SHORT_NAME) - 1 == 8, "BLE_SHORT_NAME must be exactly 8 chars to fit 31-byte legacy adv payload");
    uint8_t payload[31];
    uint8_t pos = 0;
    payload[pos++] = 2; payload[pos++] = 0x01; payload[pos++] = 0x06;  // Flags: LE discoverable, no BR/EDR
    const char* shortName = BLE_SHORT_NAME;
    uint8_t nlen = sizeof(BLE_SHORT_NAME) - 1;  // compile-time constant, no strlen needed
    payload[pos++] = nlen + 1; payload[pos++] = 0x08;                  // Shortened Local Name
    memcpy(&payload[pos], shortName, nlen); pos += nlen;
    NimBLEUUID svcUuid(BLE_SERVICE_UUID);
    const uint8_t* uuidBytes = svcUuid.getValue();
    payload[pos++] = 17; payload[pos++] = 0x07;                        // Complete 128-bit UUIDs
    memcpy(&payload[pos], uuidBytes, 16); pos += 16;
    legacyAdv.setData(payload, pos);

    bool ok0 = NimBLEDevice::getAdvertising()->setInstanceData(0, legacyAdv);
    Serial.printf("BLE adv inst0 setData=%d\n", ok0 ? 1 : 0);
  }
  bool start0 = NimBLEDevice::getAdvertising()->start(0);
  Serial.printf("BLE adv inst0 start=%d\n", start0 ? 1 : 0);

  // Instance 1: extended, Coded PHY — for Android / long-range.
  // secondary_phy_opt left at 0 (no preference); controller defaults to S=8 for Coded PHY.
  // Future: add relay telem data here as manufacturer-specific AD records.
  {
    NimBLEExtAdvertisement codedAdv;
    codedAdv.setLegacyAdvertising(false);
    codedAdv.setConnectable(true);
    codedAdv.setPrimaryPhy(BLE_HCI_LE_PHY_CODED);
    codedAdv.setSecondaryPhy(BLE_HCI_LE_PHY_CODED);
    codedAdv.setMinInterval(BLE_ADV_INTERVAL_MIN);
    codedAdv.setMaxInterval(BLE_ADV_INTERVAL_MAX);
    codedAdv.addServiceUUID(BLE_SERVICE_UUID);
    codedAdv.setName(BLE_DEVICE_NAME);

    bool ok1 = NimBLEDevice::getAdvertising()->setInstanceData(1, codedAdv);
    Serial.printf("BLE adv inst1 setData=%d\n", ok1 ? 1 : 0);
  }
  bool start1 = NimBLEDevice::getAdvertising()->start(1);
  Serial.printf("BLE adv inst1 start=%d\n", start1 ? 1 : 0);

  Serial.println("BLE GATT started");
}

void deinitBLE() {
  if (!bleServer) return;
  NimBLEDevice::deinit(true);
  bleServer = nullptr;
  bleTelemChar = nullptr;
  bleCmdChar = nullptr;
  bleStatusChar = nullptr;
  bleLogFetchChar = nullptr;
  bleOtaChar = nullptr;
  bleClientConnected = false;
  bleLogFetch.active = false;
  Serial.println("BLE deinited");
}

// ===================== SETUP =====================

// ===================== POWER TEST MODE =====================
// Select ONE test mode, then optionally add KEEP_* flags for that mode.
//
// POWER_TEST_DEEP   — deep sleep (chip halted, lowest possible floor)
// POWER_TEST_LIGHT  — light sleep (CPU halted, peripherals can stay alive,
//                     BLE controller can wake on connection events)
// POWER_TEST_CPU    — CPU spinning at chosen frequency, no subsystems
//
// KEEP flags (apply to all modes except deep sleep, which kills everything):
//   POWER_TEST_KEEP_LORA   — leave SX1262 in RX
//   POWER_TEST_KEEP_BLE    — init BLE advertising
//   POWER_TEST_KEEP_WIFI   — start WiFi AP
//
// CPU frequency for POWER_TEST_CPU and POWER_TEST_LIGHT (pick one):
//   POWER_TEST_CPU_240, POWER_TEST_CPU_160, POWER_TEST_CPU_80, POWER_TEST_CPU_40
//   Default if none defined: 80MHz

// -- Uncomment one: --
// #define POWER_TEST_DEEP
//#define POWER_TEST_LIGHT
//#define POWER_TEST_CPU//20ma

// -- Optionally add (not meaningful for DEEP): --
#define POWER_TEST_KEEP_LORA
//#define POWER_TEST_KEEP_BLE
//#define POWER_TEST_KEEP_WIFI//80ma

// -- CPU frequency for LIGHT and CPU modes: --
 //#define POWER_TEST_CPU_240
// #define POWER_TEST_CPU_160
//#define POWER_TEST_CPU_80   // default
// #define POWER_TEST_CPU_40

#if defined(POWER_TEST_DEEP) || defined(POWER_TEST_LIGHT) || defined(POWER_TEST_CPU)
#define POWER_TEST_ACTIVE

static void powerTestInit() {
  Serial.begin(115200);
  delay(500);


#if defined(POWER_TEST_DEEP)
  Serial.println("=== POWER TEST: deep sleep ===");
#elif defined(POWER_TEST_LIGHT)
  Serial.println("=== POWER TEST: light sleep ===");
#else
  Serial.println("=== POWER TEST: CPU spinning ===");
#endif

  ledcAttach(LED_PIN, 1000, 11);
  ledcWrite(LED_PIN, 0);
  pinMode(VEXT_CTRL_PIN, OUTPUT);     digitalWrite(VEXT_CTRL_PIN, LOW);
  pinMode(VBAT_ADC_CTRL_PIN, INPUT);  // tri-state; readBaseBattery() switches to INPUT_PULLUP briefly

  // SX1262 is already in standby after reset; deep sleep mode costs ~0.6µA
  // and prevents it responding to SPI glitches. Send SetSleep explicitly.
  // (SX1262 comes out of HW reset in standby, not sleep — datasheet 8.1)
#ifdef POWER_TEST_KEEP_LORA
  bsLoraSPI.begin(LORA_SCK_PIN, LORA_MISO_PIN, LORA_MOSI_PIN, LORA_NSS_PIN);
  bsRadioInit();
  Serial.println("SX1262: RX");
#else
  // pinMode(LORA_NSS_PIN, OUTPUT); digitalWrite(LORA_NSS_PIN, HIGH);
  // bsLoraSPI.begin(LORA_SCK_PIN, LORA_MISO_PIN, LORA_MOSI_PIN, LORA_NSS_PIN);
  // bsLoraSPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  // digitalWrite(LORA_NSS_PIN, LOW);
  // bsLoraSPI.transfer(0x84); bsLoraSPI.transfer(0x00); // SetSleep, cold start
  // digitalWrite(LORA_NSS_PIN, HIGH);
  // bsLoraSPI.endTransaction();
  Serial.println("SX1262: sleep");
#endif

#ifdef POWER_TEST_KEEP_BLE
  initBLE();
  Serial.println("BLE: advertising");
#else
  Serial.println("BLE: off (never inited)");
#endif

#ifdef POWER_TEST_KEEP_WIFI
  WiFi.mode(WIFI_AP);
  WiFi.softAP(WIFI_SSID, WIFI_PASS, WIFI_CHANNEL);
  Serial.println("WiFi: AP up");
#else
  WiFi.mode(WIFI_OFF);
  esp_wifi_stop();
  Serial.println("WiFi: off");
#endif

#if defined(POWER_TEST_CPU_240)
  setCpuFrequencyMhz(240);
#elif defined(POWER_TEST_CPU_160)
  setCpuFrequencyMhz(160);
#elif defined(POWER_TEST_CPU_40)
  setCpuFrequencyMhz(40);
#else
  setCpuFrequencyMhz(80);
#endif
  Serial.print("CPU: "); Serial.print(getCpuFrequencyMhz()); Serial.println("MHz");

  // ADC ctrl pin characterisation: sample voltage in each pin state once per second.
  // Measures: INPUT (tri-state), INPUT_PULLUP, then OUTPUT HIGH — in each case
  // immediately after switching and again after 400µs settle, to check ramp time.
  // Helps verify pullup approach gives comparable reading to direct drive.
  analogSetAttenuation(ADC_11db);
  auto adcMv = []() -> uint32_t { return (uint32_t)(analogReadMilliVolts(VBAT_ADC_PIN) * VBAT_MULTIPLIER); };

  Serial.println("=== ADC ctrl characterisation (mV, once/sec) ===");
  Serial.println("State           instant  +400us");
  while (true) {
    uint32_t v0, v1;

    pinMode(VBAT_ADC_CTRL_PIN, INPUT);
    v0 = adcMv(); delayMicroseconds(400); v1 = adcMv();
    Serial.printf("INPUT (float)   %5u    %5u\n", v0, v1);

    pinMode(VBAT_ADC_CTRL_PIN, INPUT_PULLUP);
    v0 = adcMv(); delayMicroseconds(400); v1 = adcMv();
    Serial.printf("INPUT_PULLUP    %5u    %5u\n", v0, v1);
    pinMode(VBAT_ADC_CTRL_PIN, INPUT); // back off

    pinMode(VBAT_ADC_CTRL_PIN, OUTPUT); digitalWrite(VBAT_ADC_CTRL_PIN, HIGH);
    v0 = adcMv(); delayMicroseconds(400); v1 = adcMv();
    Serial.printf("OUTPUT HIGH     %5u    %5u\n", v0, v1);
    pinMode(VBAT_ADC_CTRL_PIN, INPUT);

    Serial.println("---");
    delay(1000);
  }
  // unreachable — loop above runs forever
}
#endif // defined(POWER_TEST_DEEP) || defined(POWER_TEST_LIGHT) || defined(POWER_TEST_CPU)

void setup() {
#ifdef POWER_TEST_ACTIVE
  powerTestInit();

#if defined(POWER_TEST_DEEP)
  esp_deep_sleep_start(); // halts here — no wakeup source = sleep forever
#elif defined(POWER_TEST_LIGHT)
  // Timer wakeup every 10s so it doesn't sleep forever, but ratio is
  // ~10s sleep / <1ms wake so duty cycle is effectively 100% sleep.
  esp_sleep_enable_timer_wakeup(10ULL * 1000000ULL);
  while (true) { esp_light_sleep_start(); }
#else // POWER_TEST_CPU
  while (true) { vTaskDelay(1); } // yield keeps TWDT happy
#endif
#endif // POWER_TEST_ACTIVE

  bootMicros = micros();  // Capture boot time for tagged serial timestamps
  //taggedSerial.begin(SERIAL_BAUD);
  Serial.begin(SERIAL_BAUD);
  delay(500);
  Serial.println("\n=== Rocket Base Station ===");

  // Must downscale CPU before NimBLE init — BLE stack calibrates its timers at init time.
  // Downclocking after init causes connection instability.
  setCpuFrequencyMhz(160);
  Serial.print("CPU: "); Serial.print(getCpuFrequencyMhz()); Serial.println("MHz");

  initBLE();

  ledcAttach(LED_PIN, 1000, 11);
  ledcWrite(LED_PIN, 50);

  pinMode(VEXT_CTRL_PIN, OUTPUT); digitalWrite(VEXT_CTRL_PIN, LOW); //dont need gps, etc
  pinMode(VBAT_ADC_CTRL_PIN, INPUT);  // tri-state; readBaseBattery() switches to INPUT_PULLUP briefly
  analogSetAttenuation(ADC_11db);
  readBaseBattery();

  if (!LittleFS.begin(false, "/littlefs", 10, "fs")) Serial.println("LittleFS mount failed — run LittleFS Data Upload");
  else Serial.println("LittleFS mounted ok");

  logStoreOk = logStore.begin("log_data", "log_index", "bs_log");
  Serial.print("logStore: "); Serial.println(logStoreOk ? "OK" : "FAIL");
  if (logStoreOk) {
    Serial.print("  rec="); Serial.print(logStore.getRecordCounter());
    Serial.print(" old="); Serial.print(logStore.getOldestRecord());
    Serial.print(" vpos=0x"); Serial.println(logStore.getVirtualPos(), HEX);
  }

  bsNvs.begin("basestation", false);
  highestNonce = bsNvs.getUInt("nonce", 0);

  // Load radio settings from NVS
  if (bsNvs.isKey("radio_ch")) { uint8_t c = bsNvs.getUChar("radio_ch", DEFAULT_CHANNEL); activeChannel = (channelToFreqMHz(c) != 0.0f) ? c : DEFAULT_CHANNEL; }
  if (bsNvs.isKey("radio_sf")) { uint8_t s = bsNvs.getUChar("radio_sf", DEFAULT_SF); activeSF = (s >= 5 && s <= 12) ? s : DEFAULT_SF; }
  if (bsNvs.isKey("radio_pwr")) { int8_t p = bsNvs.getChar("radio_pwr", DEFAULT_POWER); activePower = (p >= -9 && p <= 22) ? p : DEFAULT_POWER; }
  bsUpdateActiveFreqBw();

  if (bsNvs.isKey("bh_ch")) { uint8_t c = bsNvs.getUChar("bh_ch", DEFAULT_BH_CHANNEL); bhChannel = (channelToFreqMHz(c) != 0.0f) ? c : DEFAULT_BH_CHANNEL; }
  if (bsNvs.isKey("bh_sf")) { uint8_t s = bsNvs.getUChar("bh_sf", DEFAULT_BH_SF); bhSF = (s >= 5 && s <= 12) ? s : DEFAULT_BH_SF; }
  if (bsNvs.isKey("bh_pwr")) { int8_t p = bsNvs.getChar("bh_pwr", DEFAULT_BH_POWER); bhPower = (p >= -9 && p <= 22) ? p : DEFAULT_BH_POWER; }

  Serial.print("Radio pri: ch"); Serial.print(activeChannel);
  Serial.print(" "); Serial.print(activeFreqMHz, 1);
  Serial.print("MHz SF"); Serial.print(activeSF);
  Serial.print(" BW"); Serial.print((int)activeBwKHz);
  Serial.print(" pwr="); Serial.println(activePower);
  Serial.print("Device ID: "); Serial.println(DEVICE_ID);

  // WiFi off by default — button toggles it on/off at runtime
  WiFi.mode(WIFI_OFF);
  esp_wifi_stop();
  Serial.println("WiFi: off (press USER_BTN to enable)");

  // Register HTTP routes — server not started until WiFi is enabled
  httpServer.on("/", HTTP_GET, [](AsyncWebServerRequest *r){
    Serial.println("HTTP GET /");
    AsyncWebServerResponse *resp = r->beginResponse(LittleFS, "/index.html.gz", "text/html");
    if (!resp) { r->send(503, "text/plain", "LittleFS not mounted"); return; }
    resp->addHeader("Content-Encoding", "gzip");
    r->send(resp);
  });
  httpServer.on("/style.css", HTTP_GET, [](AsyncWebServerRequest *r){
    AsyncWebServerResponse *resp = r->beginResponse(LittleFS, "/style.css.gz", "text/css");
    if (!resp) { r->send(503, "text/plain", "LittleFS not mounted"); return; }
    resp->addHeader("Content-Encoding", "gzip");
    r->send(resp);
  });
  httpServer.on("/app.js", HTTP_GET, [](AsyncWebServerRequest *r){
    AsyncWebServerResponse *resp = r->beginResponse(LittleFS, "/app.js.gz", "application/javascript");
    if (!resp) { r->send(503, "text/plain", "LittleFS not mounted"); return; }
    resp->addHeader("Content-Encoding", "gzip");
    r->send(resp);
  });
  httpServer.on("/api/status", HTTP_GET, handleApiStatus);
  httpServer.on("/api/logs", HTTP_GET, handleApiLogs);
  httpServer.on("/api/command", HTTP_POST,
    [](AsyncWebServerRequest *req){ }, NULL, handleApiCommand);
  httpServer.on("/api/ota/chunk", HTTP_PUT,
    [](AsyncWebServerRequest *req){ }, NULL, handleApiOtaChunk);
  ws.onEvent(onWsEvent);
  httpServer.addHandler(&ws);
  // httpServer.begin() called by enableWifi() when button pressed

  pinMode(USER_BTN_PIN, INPUT_PULLUP);

  // LoRa
  bsLoraSPI.begin(LORA_SCK_PIN, LORA_MISO_PIN, LORA_MOSI_PIN, LORA_NSS_PIN);
  Serial.print("LoRa init... ");
  if (bsRadioInit()) {
    Serial.println("ok");
    bsLoraReady = true;
    // Slot machine starts listening on first slot boundary; no need to start RX here.
  } else {
    Serial.println("FAIL");
  }

  Serial.println("=== Ready ===");
}

// ===================== MAIN LOOP =====================

// ---- USER BUTTON: toggle WiFi ----
static void handleUserButton() {
  static bool lastState = HIGH;
  static unsigned long pressedAt = 0;
  bool state = digitalRead(USER_BTN_PIN);
  if (state == LOW && lastState == HIGH) {
    pressedAt = millis();
  }
  if (state == HIGH && lastState == LOW && (millis() - pressedAt) > 30) {
    // Rising edge after debounce — toggle WiFi
    if (wifiEnabled) {
      disableWifi();
      ledSignalWifiOff();
    } else {
      enableWifi();
      ledSignalWifiOn();
    }
  }
  lastState = state;
}

void loop() {
  //TODO: should restructure this to have more done by main loop - currently more stuff than there should be has ended up in callbacks on the wrong task, e.g. notify pushes ws which delays it a while.

  handleUserButton();

  // ---- Radio ----
  if (bsLoraReady) {
    bsRadioPoll();
    if (cmdTx.active && cmdTx.sent < cmdTx.sends && bsRadioState == BS_RADIO_RX) {
      dispatchCmdTx();
    }
  }

  handleBleLogFetch();

  // OTA notify drain
  if (bsOta.notifyPending && bleOtaChar && bleClientConnected) {
    bleOtaChar->notify(bsOta.notifyBuf, bsOta.notifyLen);
    bsOta.notifyPending = false;
  }

  if (wifiEnabled) ws.cleanupClients();

}


