// Two-simultaneous-connection BLE test.
// Client: connects to the rocket, receives telem notifies, copies into a ring buffer.
// Server: advertises the rocket UUID, sends the latest buffered packet every 1s to any subscriber.
// The two sides are fully decoupled — no direct call from receive callback to notify.
#include <Arduino.h>
#include <NimBLEDevice.h>
#include <WiFi.h>

#define SERIAL_BAUD 2000000

#define RKT_SVC_UUID   "524f434b-4554-5354-424c-000000000000"
#define RKT_TELEM_UUID "524f434b-4554-5354-424c-000000000001"

// ===================== SHARED BUFFER =====================
// Written by client notify callback, read by server send in loop.
// Single-slot: newest packet wins, no queue needed for this test.
static uint8_t  rxBuf[520];
static uint16_t rxLen    = 0;
static bool     rxFresh  = false;
static uint32_t rxCount  = 0;

// ===================== SERVER SIDE =====================
static NimBLECharacteristic* serverTelemChar = nullptr;
static uint32_t txCount = 0;

class SrvCB : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer*, NimBLEConnInfo& info) override {
    Serial.printf("[SRV] phone connected handle=%u\n", info.getConnHandle());
    NimBLEDevice::startAdvertising();
  }
  void onDisconnect(NimBLEServer*, NimBLEConnInfo& info, int reason) override {
    Serial.printf("[SRV] phone disconnected handle=%u reason=%d\n", info.getConnHandle(), reason);
    NimBLEDevice::startAdvertising();
  }
};

// ===================== CLIENT SIDE =====================
static NimBLEClient*               rktClient   = nullptr;
static NimBLERemoteCharacteristic* rktTelemChar = nullptr;
static volatile bool rocketConnected = false;
static volatile bool connectPending  = false;
static NimBLEAddress pendingAddr;
static bool scanActive = false;
static unsigned long lastScanMs = 0;

static void onRocketTelem(NimBLERemoteCharacteristic*, uint8_t* data, size_t len, bool) {
  // Called on NimBLE stack task — only copy, never notify from here.
  size_t capped = len > sizeof(rxBuf) ? sizeof(rxBuf) : len;
  memcpy(rxBuf, data, capped);
  rxLen   = (uint16_t)capped;
  rxFresh = true;
  rxCount++;
}

class CliCB : public NimBLEClientCallbacks {
  void onConnect(NimBLEClient*) override {
    Serial.println("[CLI] rocket connected");
  }
  void onDisconnect(NimBLEClient*, int reason) override {
    rocketConnected = false;
    rktTelemChar    = nullptr;
    Serial.printf("[CLI] rocket disconnected reason=%d\n", reason);
  }
};

class ScanCB : public NimBLEScanCallbacks {
  void onResult(const NimBLEAdvertisedDevice* adv) override {
    std::string advAddr = adv->getAddress().toString();
    std::string ownAddr = NimBLEDevice::getAddress().toString();
    if (advAddr == ownAddr) return;
    if (!adv->isAdvertisingService(NimBLEUUID(RKT_SVC_UUID))) return;
    Serial.printf("[SCAN] found rocket: %s rssi=%d\n", advAddr.c_str(), adv->getRSSI());
    NimBLEDevice::getScan()->stop();
    scanActive     = false;
    pendingAddr    = adv->getAddress();
    connectPending = true;
  }
  void onScanEnd(const NimBLEScanResults&, int reason) override {
    scanActive = false;
    Serial.printf("[SCAN] ended reason=%d\n", reason);
  }
};

static void doConnect() {
  connectPending = false;
  if (!rktClient) {
    rktClient = NimBLEDevice::createClient();
    rktClient->setClientCallbacks(new CliCB(), true);
  }
  Serial.printf("[CLI] connecting to %s...\n", pendingAddr.toString().c_str());
  if (!rktClient->connect(pendingAddr)) {
    Serial.println("[CLI] connect failed");
    lastScanMs = millis();
    return;
  }
  NimBLERemoteService* svc = rktClient->getService(RKT_SVC_UUID);
  if (!svc) { Serial.println("[CLI] service not found"); rktClient->disconnect(); return; }
  rktTelemChar = svc->getCharacteristic(RKT_TELEM_UUID);
  if (!rktTelemChar) { Serial.println("[CLI] telem char not found"); rktClient->disconnect(); return; }
  if (!rktTelemChar->subscribe(true, onRocketTelem)) {
    Serial.println("[CLI] subscribe failed"); rktClient->disconnect(); return;
  }
  rocketConnected = true;
  Serial.println("[CLI] subscribed to rocket telem");
}

// ===================== SETUP / LOOP =====================

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(500);
  Serial.println("\n=== Two-connection decoupled BLE test ===");

  WiFi.mode(WIFI_OFF);

  NimBLEDevice::init("Moonshot-Rocket");
  NimBLEDevice::setPower(ESP_PWR_LVL_P9);
  NimBLEDevice::setMTU(517);

  // Server
  NimBLEServer* server = NimBLEDevice::createServer();
  server->setCallbacks(new SrvCB());
  NimBLEService* svc = server->createService(RKT_SVC_UUID);
  serverTelemChar = svc->createCharacteristic(RKT_TELEM_UUID, NIMBLE_PROPERTY::NOTIFY);
  svc->start();

  NimBLEAdvertising* adv = NimBLEDevice::getAdvertising();
  adv->addServiceUUID(RKT_SVC_UUID);
  adv->setName("Moonshot-Rocket");
  adv->enableScanResponse(true);
  adv->start(0);

  // Client scanner
  NimBLEScan* scan = NimBLEDevice::getScan();
  scan->setScanCallbacks(new ScanCB(), true);
  scan->setActiveScan(true);
  scan->setInterval(160);
  scan->setWindow(80);

  Serial.println("=== Ready ===");
}

void loop() {
  if (!NimBLEDevice::getAdvertising()->isAdvertising()) {
    NimBLEDevice::getAdvertising()->start(0);
  }

  // Client: connect if pending
  if (connectPending) {
    doConnect();
    return;
  }

  // Client: scan if not connected
  if (!rocketConnected && !scanActive) {
    unsigned long now = millis();
    if (now - lastScanMs >= 2000) {
      lastScanMs = now;
      scanActive = true;
      Serial.println("[SCAN] scanning for rocket...");
      NimBLEDevice::getScan()->start(2000, false);
    }
  }

  // Server: send latest buffered packet every 1s (decoupled from receive)
  static unsigned long lastTxMs = 0;
  unsigned long now = millis();
  if (now - lastTxMs >= 1000) {
    lastTxMs = now;
    if (rxLen > 0) {
      bool ok = serverTelemChar->notify(rxBuf, rxLen);
      txCount++;
      Serial.printf("[SRV] tx #%lu %uB ok=%d  rx_total=%lu rocket=%d\n",
                    txCount, rxLen, ok ? 1 : 0, rxCount, (int)rocketConnected);
    } else {
      Serial.printf("[SRV] no data yet  rocket=%d\n", (int)rocketConnected);
    }
  }
}
