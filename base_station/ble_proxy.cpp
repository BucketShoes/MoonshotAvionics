// ble_proxy.cpp — BLE range-extension proxy.
// See ble_proxy.h for design notes.

#include <Arduino.h>
#include <NimBLEDevice.h>
#include "ble_proxy.h"

// Mirror of rocket_avionics/config.h — must stay in sync.
#define RKT_SVC_UUID       "524f434b-4554-5354-424c-000000000000"
#define RKT_TELEM_UUID     "524f434b-4554-5354-424c-000000000001"  // NOTIFY
#define RKT_CMD_UUID       "524f434b-4554-5354-424c-000000000002"  // WRITE | WRITE_NR
#define RKT_STATUS_UUID    "524f434b-4554-5354-424c-000000000003"  // READ
#define RKT_CONNSET_UUID   "524f434b-4554-5354-424c-000000000004"  // WRITE | WRITE_NR
#define RKT_LOGFETCH_UUID  "524f434b-4554-5354-424c-000000000005"  // WRITE | NOTIFY
#define RKT_OTA_UUID       "524f434b-4554-5354-424c-000000000006"  // WRITE | WRITE_NR | NOTIFY

#define ROCKET_DEVICE_NAME "Moonshot-Rocket"
#define PX_INVALID_CONN    0xFFFF

// ===================== STATE =====================

// --- Server side (phone connects here) ---
static NimBLEServer*         pxServer      = nullptr;
static NimBLECharacteristic* pxTelemChar   = nullptr;  // NOTIFY → phone
static NimBLECharacteristic* pxCmdChar     = nullptr;  // WRITE ← phone
static NimBLECharacteristic* pxStatusChar  = nullptr;  // READ  ← phone
static NimBLECharacteristic* pxConnSetChar = nullptr;  // WRITE ← phone
static NimBLECharacteristic* pxFetchChar   = nullptr;  // WRITE | NOTIFY ↔ phone
static NimBLECharacteristic* pxOtaChar     = nullptr;  // WRITE | NOTIFY ↔ phone

static volatile bool phoneConnected = false;
static uint16_t      phoneConnHandle = PX_INVALID_CONN;

// --- Client side (connects to rocket) ---
static NimBLEClient*               pxClient      = nullptr;
static NimBLERemoteCharacteristic* rxTelemChar   = nullptr;
static NimBLERemoteCharacteristic* rxCmdChar     = nullptr;
static NimBLERemoteCharacteristic* rxStatusChar  = nullptr;
static NimBLERemoteCharacteristic* rxConnSetChar = nullptr;
static NimBLERemoteCharacteristic* rxFetchChar   = nullptr;
static NimBLERemoteCharacteristic* rxOtaChar     = nullptr;

static volatile bool rocketConnected = false;
static volatile bool scanActive      = false;
static volatile bool connectPending  = false;  // onResult found rocket, connecting

// Scan retry back-off
static unsigned long lastScanMs    = 0;
static unsigned long scanIntervalMs = 5000;  // starts at 5 s, backs off to 30 s

// ===================== BACKPRESSURE =====================
// If a phone-side notify() drops, hold payload and retry next loop before
// consuming the next rocket notification.

struct PendingNotify {
    uint8_t  buf[514];
    uint16_t len;
    bool     pending;
};
static PendingNotify pxTelemPending = {};
static PendingNotify pxFetchPending = {};
static PendingNotify pxOtaPending   = {};

static void notifyPhone(NimBLECharacteristic* chr, PendingNotify& pn,
                        const uint8_t* data, size_t len) {
    if (!phoneConnected || !chr) return;
    size_t capped = len > sizeof(pn.buf) ? sizeof(pn.buf) : len;
    if (!chr->notify(data, capped)) {
        memcpy(pn.buf, data, capped);
        pn.len = (uint16_t)capped;
        pn.pending = true;
    }
}

static bool retryPending(NimBLECharacteristic* chr, PendingNotify& pn) {
    if (!pn.pending) return true;
    if (!phoneConnected || !chr) { pn.pending = false; return true; }
    if (!chr->notify(pn.buf, pn.len)) return false;
    pn.pending = false;
    return true;
}

// ===================== ROCKET NOTIFY CALLBACKS =====================
// Fire on the NimBLE task when the rocket sends a notification.

static void onRocketTelem(NimBLERemoteCharacteristic*, uint8_t* data, size_t len, bool) {
    notifyPhone(pxTelemChar, pxTelemPending, data, len);
}

static void onRocketFetch(NimBLERemoteCharacteristic*, uint8_t* data, size_t len, bool) {
    notifyPhone(pxFetchChar, pxFetchPending, data, len);
}

static void onRocketOta(NimBLERemoteCharacteristic*, uint8_t* data, size_t len, bool) {
    notifyPhone(pxOtaChar, pxOtaPending, data, len);
}

// ===================== PHONE → ROCKET WRITE FORWARDING =====================

static void forwardWrite(NimBLERemoteCharacteristic* rxChr,
                         const uint8_t* data, size_t len, bool withResponse) {
    if (!rocketConnected || !rxChr) return;
    rxChr->writeValue(data, len, withResponse);
}

// ===================== SERVER CONNECT/DISCONNECT =====================
// Called from the unified BleServerCallbacks in main.cpp.

void bleProxyOnServerConnect(uint16_t connHandle) {
    phoneConnected  = true;
    phoneConnHandle = connHandle;
    Serial.printf("[PROXY] Phone connected handle=%u\n", connHandle);
}

void bleProxyOnServerDisconnect(uint16_t connHandle) {
    if (connHandle != phoneConnHandle) return;
    phoneConnected  = false;
    phoneConnHandle = PX_INVALID_CONN;
    pxTelemPending.pending = false;
    pxFetchPending.pending = false;
    pxOtaPending.pending   = false;
    Serial.println("[PROXY] Phone disconnected");
}

// ===================== SERVER CHARACTERISTIC CALLBACKS =====================

class PxCmdCallbacks : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic* chr, NimBLEConnInfo&) override {
        NimBLEAttValue v = chr->getValue();
        if (!rocketConnected || !rxCmdChar) { uint8_t nack = 0xFF; chr->setValue(&nack, 1); return; }
        bool ok = rxCmdChar->writeValue(v.data(), v.size(), true);
        if (ok) {
            NimBLEAttValue resp = rxCmdChar->getValue();
            chr->setValue(resp.data(), resp.size());
        } else {
            uint8_t nack = 0xFF;
            chr->setValue(&nack, 1);
        }
    }
};

class PxStatusCallbacks : public NimBLECharacteristicCallbacks {
    void onRead(NimBLECharacteristic* chr, NimBLEConnInfo&) override {
        if (!rocketConnected || !rxStatusChar) {
            const char* err = "{\"proxy\":\"no_rocket\"}";
            chr->setValue((uint8_t*)err, strlen(err));
            return;
        }
        NimBLEAttValue v = rxStatusChar->readValue();
        chr->setValue(v.data(), v.size());
    }
};

class PxConnSetCallbacks : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic* chr, NimBLEConnInfo&) override {
        NimBLEAttValue v = chr->getValue();
        forwardWrite(rxConnSetChar, v.data(), v.size(), false);
        // Rocket receives this and calls updatePhy() on its connection with us.
        // We accept the incoming PHY update automatically as the central role.
        // The base↔phone link is unaffected.
    }
};

class PxFetchCallbacks : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic* chr, NimBLEConnInfo&) override {
        NimBLEAttValue v = chr->getValue();
        pxFetchPending.pending = false;
        forwardWrite(rxFetchChar, v.data(), v.size(), false);
    }
};

class PxOtaCallbacks : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic* chr, NimBLEConnInfo&) override {
        NimBLEAttValue v = chr->getValue();
        forwardWrite(rxOtaChar, v.data(), v.size(), false);
    }
};

// ===================== CLIENT CALLBACKS =====================

class PxClientCallbacks : public NimBLEClientCallbacks {
    void onConnect(NimBLEClient* client) override {
        Serial.println("[PROXY] Rocket BLE connected");
        scanIntervalMs = 5000;
        connectPending = false;
    }
    void onDisconnect(NimBLEClient* client, int reason) override {
        rocketConnected = false;
        connectPending  = false;
        rxTelemChar = rxCmdChar = rxStatusChar = nullptr;
        rxConnSetChar = rxFetchChar = rxOtaChar = nullptr;
        Serial.printf("[PROXY] Rocket BLE disconnected reason=%d\n", reason);
        scanIntervalMs = min(scanIntervalMs * 2, (unsigned long)30000);
        lastScanMs = millis();
    }
};

// ===================== SCAN CALLBACKS =====================

class PxScanCallbacks : public NimBLEScanCallbacks {
    void onResult(const NimBLEAdvertisedDevice* adv) override {
        // Match by name only — prevents base stations connecting to each other
        // even if they advertise the same service UUID.
        if (adv->getName() != std::string(ROCKET_DEVICE_NAME)) return;

        Serial.printf("[PROXY] Found rocket: %s  RSSI=%d\n",
                      adv->getAddress().toString().c_str(), adv->getRSSI());
        NimBLEDevice::getScan()->stop();
        scanActive     = false;
        connectPending = true;

        if (!pxClient) {
            pxClient = NimBLEDevice::createClient();
            pxClient->setClientCallbacks(new PxClientCallbacks(), true);
        }

        // EXT_ADV not enabled, so setConnectPhy() isn't available.
        // Connect on 1M (default); coded PHY negotiated post-connect via ConnSet.
        if (!pxClient->connect(adv)) {
            Serial.println("[PROXY] connect() failed");
            connectPending = false;
            scanIntervalMs = min(scanIntervalMs * 2, (unsigned long)30000);
            lastScanMs = millis();
            return;
        }

        NimBLERemoteService* svc = pxClient->getService(RKT_SVC_UUID);
        if (!svc) {
            Serial.println("[PROXY] rocket service not found");
            pxClient->disconnect();
            return;
        }

        rxTelemChar   = svc->getCharacteristic(RKT_TELEM_UUID);
        rxCmdChar     = svc->getCharacteristic(RKT_CMD_UUID);
        rxStatusChar  = svc->getCharacteristic(RKT_STATUS_UUID);
        rxConnSetChar = svc->getCharacteristic(RKT_CONNSET_UUID);
        rxFetchChar   = svc->getCharacteristic(RKT_LOGFETCH_UUID);
        rxOtaChar     = svc->getCharacteristic(RKT_OTA_UUID);

        if (!rxTelemChar || !rxCmdChar || !rxStatusChar) {
            Serial.println("[PROXY] missing required rocket chars");
            pxClient->disconnect();
            return;
        }

        if (!rxTelemChar->subscribe(true, onRocketTelem)) {
            Serial.println("[PROXY] telem subscribe failed");
            pxClient->disconnect();
            return;
        }
        if (rxFetchChar) rxFetchChar->subscribe(true, onRocketFetch);
        if (rxOtaChar)   rxOtaChar->subscribe(true, onRocketOta);

        rocketConnected = true;
        Serial.println("[PROXY] Rocket proxy active");
    }

    void onScanEnd(const NimBLEScanResults&, int reason) override {
        scanActive = false;
        Serial.printf("[PROXY] Scan ended reason=%d\n", reason);
    }
};

// ===================== PUBLIC API =====================

void bleProxyInit() {
    // Add proxy service to the existing shared NimBLE server (created by initBLE()).
    pxServer = NimBLEDevice::getServer();
    if (!pxServer) pxServer = NimBLEDevice::createServer();

    NimBLEService* svc = pxServer->createService(RKT_SVC_UUID);
    pxTelemChar   = svc->createCharacteristic(RKT_TELEM_UUID,    NIMBLE_PROPERTY::NOTIFY);
    pxCmdChar     = svc->createCharacteristic(RKT_CMD_UUID,      NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR);
    pxStatusChar  = svc->createCharacteristic(RKT_STATUS_UUID,   NIMBLE_PROPERTY::READ);
    pxConnSetChar = svc->createCharacteristic(RKT_CONNSET_UUID,  NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR);
    pxFetchChar   = svc->createCharacteristic(RKT_LOGFETCH_UUID, NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::NOTIFY);
    pxOtaChar     = svc->createCharacteristic(RKT_OTA_UUID,      NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR | NIMBLE_PROPERTY::NOTIFY);

    pxCmdChar->setCallbacks(new PxCmdCallbacks());
    pxStatusChar->setCallbacks(new PxStatusCallbacks());
    pxConnSetChar->setCallbacks(new PxConnSetCallbacks());
    pxFetchChar->setCallbacks(new PxFetchCallbacks());
    pxOtaChar->setCallbacks(new PxOtaCallbacks());

    // Advertising: both service UUIDs + rocket device name.
    // NimBLEDevice::init() sets the GAP device name from the string passed to it
    // (which is WIFI_SSID in initBLE()).  Override it here so scan responses show
    // the correct name.  This is the name the phone and nRF Connect see.
    NimBLEDevice::setDeviceName(ROCKET_DEVICE_NAME);

    NimBLEAdvertising* advert = NimBLEDevice::getAdvertising();
    advert->addServiceUUID(RKT_SVC_UUID);
    // Base station service UUID is already added by initBLE() — both appear in advert.
    advert->setName(ROCKET_DEVICE_NAME);
    advert->enableScanResponse(true);
    advert->setMinInterval(0x20);
    advert->setMaxInterval(0x40);
    advert->start();

    // Scan config — 1M PHY only (CONFIG_BT_NIMBLE_EXT_ADV not set).
    NimBLEScan* scan = NimBLEDevice::getScan();
    scan->setScanCallbacks(new PxScanCallbacks(), false);
    scan->setActiveScan(true);  // active so we get the scan response with the device name
    scan->setInterval(200);
    scan->setWindow(50);

    Serial.println("[PROXY] BLE proxy init done");
}

void bleProxyLoop() {
    retryPending(pxTelemChar, pxTelemPending);
    retryPending(pxFetchChar, pxFetchPending);
    retryPending(pxOtaChar,   pxOtaPending);

    if (!rocketConnected && !scanActive && !connectPending) {
        unsigned long now = millis();
        if (now - lastScanMs >= scanIntervalMs) {
            lastScanMs = now;
            scanActive = true;
            Serial.println("[PROXY] Scanning for rocket...");
            NimBLEDevice::getScan()->start(10, false);  // 10 s scan, non-blocking
        }
    }
}

bool bleProxyRocketConnected() { return rocketConnected; }
bool bleProxyPhoneConnected()  { return phoneConnected; }
