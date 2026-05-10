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
static NimBLECharacteristic* pxTelemChar   = nullptr;
static NimBLECharacteristic* pxCmdChar     = nullptr;
static NimBLECharacteristic* pxStatusChar  = nullptr;
static NimBLECharacteristic* pxConnSetChar = nullptr;
static NimBLECharacteristic* pxFetchChar   = nullptr;
static NimBLECharacteristic* pxOtaChar     = nullptr;

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

// onResult sets this; bleProxyLoop() does the actual connect outside the callback.
static NimBLEAddress pendingAddr;
static volatile bool connectPending  = false;

// Scan retry back-off
static unsigned long lastScanMs     = 0;
static unsigned long scanIntervalMs = 2000;  // 2s gap between scan attempts

// ===================== BACKPRESSURE =====================

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
// Called from BleServerCallbacks in main.cpp.

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
            uint8_t nack = 0xFF; chr->setValue(&nack, 1);
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
    void onConnect(NimBLEClient*) override {
        Serial.println("[PROXY] Rocket BLE transport up");
        scanIntervalMs = 2000;
    }
    void onDisconnect(NimBLEClient*, int reason) override {
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
        std::string advAddr = adv->getAddress().toString();
        std::string ownAddr = NimBLEDevice::getAddress().toString();

        // Always log what we see so we can diagnose what's on air.
        Serial.printf("[PROXY] adv: %s rssi=%d name='%s' svc_match=%d self=%d\n",
                      advAddr.c_str(), adv->getRSSI(), adv->getName().c_str(),
                      (int)adv->isAdvertisingService(NimBLEUUID(RKT_SVC_UUID)),
                      (int)(advAddr == ownAddr));

        // Skip our own advertisement (base station advertises the rocket UUID too).
        // Compare as strings — address type differences can break operator==.
        if (advAddr == ownAddr) return;

        if (!adv->isAdvertisingService(NimBLEUUID(RKT_SVC_UUID))) return;

        Serial.printf("[PROXY] Found rocket candidate: %s  RSSI=%d\n",
                      advAddr.c_str(), adv->getRSSI());

        // Record address for doConnect() — do NOT call connect() here (BLE stack task).
        NimBLEDevice::getScan()->stop();
        scanActive     = false;
        pendingAddr    = adv->getAddress();
        connectPending = true;
    }

    void onScanEnd(const NimBLEScanResults&, int reason) override {
        scanActive = false;
        Serial.printf("[PROXY] Scan ended reason=%d\n", reason);
    }
};

// ===================== CONNECT + SERVICE DISCOVERY =====================
// Called from bleProxyLoop() on the Arduino main task, not from a BLE callback.

static void doConnect() {
    connectPending = false;

    if (!pxClient) {
        pxClient = NimBLEDevice::createClient();
        pxClient->setClientCallbacks(new PxClientCallbacks(), true);
    }

    Serial.printf("[PROXY] Connecting to %s...\n", pendingAddr.toString().c_str());
    if (!pxClient->connect(pendingAddr)) {
        Serial.println("[PROXY] connect() failed");
        scanIntervalMs = min(scanIntervalMs * 2, (unsigned long)30000);
        lastScanMs = millis();
        return;
    }

    NimBLERemoteService* svc = pxClient->getService(RKT_SVC_UUID);
    if (!svc) {
        Serial.println("[PROXY] rocket service not found — wrong device?");
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

// ===================== PUBLIC API =====================

void bleProxyInit() {
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

    // Override the GAP device name set by NimBLEDevice::init(WIFI_SSID) in initBLE().
    NimBLEDevice::setDeviceName(ROCKET_DEVICE_NAME);

    NimBLEAdvertising* advert = NimBLEDevice::getAdvertising();
    advert->addServiceUUID(RKT_SVC_UUID);
    advert->setName(ROCKET_DEVICE_NAME);
    advert->enableScanResponse(true);
    advert->setMinInterval(0x20);
    advert->setMaxInterval(0x40);
    advert->start();

    NimBLEScan* scan = NimBLEDevice::getScan();
    // wantDuplicates=true: keep firing onResult throughout the scan duration,
    // not just on the first seen device.  Needed so we see the rocket even if
    // we see something else (our own advert) first.
    scan->setScanCallbacks(new PxScanCallbacks(), true);
    scan->setActiveScan(true);
    // interval=160ms, window=80ms (50% duty) — leaves gaps for WiFi beacon coex.
    // 100% duty actually performs worse: the controller constantly interrupts the
    // scan window for WiFi beacons instead of fitting them in the gaps cleanly.
    scan->setInterval(160);
    scan->setWindow(80);

    Serial.println("[PROXY] BLE proxy init done");
}

void bleProxyLoop() {
    retryPending(pxTelemChar, pxTelemPending);
    retryPending(pxFetchChar, pxFetchPending);
    retryPending(pxOtaChar,   pxOtaPending);

    if (connectPending) {
        doConnect();
        return;
    }

    if (!rocketConnected && !scanActive) {
        unsigned long now = millis();
        if (now - lastScanMs >= scanIntervalMs) {
            lastScanMs = now;
            scanActive = true;
            Serial.println("[PROXY] Scanning for rocket...");
            NimBLEDevice::getScan()->start(10, false);
        }
    }
}

bool bleProxyRocketConnected() { return rocketConnected; }
bool bleProxyPhoneConnected()  { return phoneConnected; }
