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

static volatile bool phoneConnected  = false;
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

static NimBLEAddress pendingAddr;
static volatile bool connectPending  = false;

static unsigned long lastScanMs     = 0;
static unsigned long scanIntervalMs = 2000;

// ===================== PENDING NOTIFY BUFFERS =====================
// All BLE stack callbacks (both server-side onWrite and client-side notify)
// run on the NimBLE task. They must never call notify() or writeValue() —
// those allocate mbufs while the stack already holds some for the current
// packet, exhausting the pool. Instead every direction goes through a
// pending buffer that bleProxyLoop() drains on the Arduino task.

struct PendingNotify {
    uint8_t       buf[514];
    uint16_t      len;
    bool          pending;
    unsigned long retryAfterMs;
};
static PendingNotify pxTelemPending   = {};
static PendingNotify pxFetchPending   = {};
static PendingNotify pxOtaPending     = {};

// Cached rocket status — refreshed by bleProxyLoop(), served synchronously
// from PxStatusCallbacks::onRead() without touching the client stack.
static uint8_t  statusCache[256]   = {};
static uint16_t statusCacheLen     = 0;
static bool     statusCacheValid   = false;
static unsigned long statusLastFetchMs = 0;
#define STATUS_CACHE_INTERVAL_MS 2000

// Phone→rocket write forwarding also goes through pending buffers so the
// server onWrite callback never touches the client stack.
struct PendingWrite {
    uint8_t  buf[514];
    uint16_t len;
    bool     withResponse;
    bool     pending;
};
static PendingWrite  fwdCmdPending     = {};
static PendingWrite  fwdConnSetPending = {};
static PendingWrite  fwdFetchPending   = {};
static PendingWrite  fwdOtaPending     = {};

// ===================== DRAIN HELPERS =====================

static bool drainNotify(NimBLECharacteristic* chr, PendingNotify& pn) {
    if (!pn.pending) return true;
    if (!phoneConnected || !chr) { pn.pending = false; return true; }
    if (millis() < pn.retryAfterMs) return false;
    if (!chr->notify(pn.buf, pn.len)) {
        pn.retryAfterMs = millis() + 20;
        return false;
    }
    pn.pending = false;
    return true;
}

static void drainWrite(NimBLERemoteCharacteristic* rxChr, PendingWrite& pw, const char* name) {
    if (!pw.pending) return;
    if (!rocketConnected || !rxChr) {
        Serial.printf("[PROXY] drainWrite %s dropped (no rocket)\n", name);
        pw.pending = false;
        return;
    }
    bool ok = rxChr->writeValue(pw.buf, pw.len, pw.withResponse);
    Serial.printf("[PROXY] drainWrite %s %uB ok=%d\n", name, pw.len, ok ? 1 : 0);
    pw.pending = false;
}

// ===================== ROCKET → PHONE NOTIFY CALLBACKS =====================
// Called on NimBLE stack task — copy only, never notify().

static void onRocketTelem(NimBLERemoteCharacteristic*, uint8_t* data, size_t len, bool) {
    int rssi = pxClient ? pxClient->getRssi() : 0;
    Serial.printf("[PROXY] telem rx %uB phone=%d rssi=%d\n", (unsigned)len, (int)phoneConnected, rssi);
    size_t capped = len > sizeof(pxTelemPending.buf) ? sizeof(pxTelemPending.buf) : len;
    memcpy(pxTelemPending.buf, data, capped);
    pxTelemPending.len     = (uint16_t)capped;
    pxTelemPending.pending = true;
}

static void onRocketFetch(NimBLERemoteCharacteristic*, uint8_t* data, size_t len, bool) {
    size_t capped = len > sizeof(pxFetchPending.buf) ? sizeof(pxFetchPending.buf) : len;
    memcpy(pxFetchPending.buf, data, capped);
    pxFetchPending.len     = (uint16_t)capped;
    pxFetchPending.pending = true;
}

static void onRocketOta(NimBLERemoteCharacteristic*, uint8_t* data, size_t len, bool) {
    size_t capped = len > sizeof(pxOtaPending.buf) ? sizeof(pxOtaPending.buf) : len;
    memcpy(pxOtaPending.buf, data, capped);
    pxOtaPending.len     = (uint16_t)capped;
    pxOtaPending.pending = true;
}

// ===================== PHONE → ROCKET WRITE CALLBACKS =====================
// Called on NimBLE stack task — copy into pending buffer, never touch client stack.

class PxCmdCallbacks : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic* chr, NimBLEConnInfo&) override {
        NimBLEAttValue v = chr->getValue();
        Serial.printf("[PROXY] CMD write %uB: ", (unsigned)v.size());
        for (size_t i = 0; i < v.size() && i < 16; i++) Serial.printf("%02X ", v.data()[i]);
        Serial.println();
        size_t capped = v.size() > sizeof(fwdCmdPending.buf) ? sizeof(fwdCmdPending.buf) : v.size();
        memcpy(fwdCmdPending.buf, v.data(), capped);
        fwdCmdPending.len          = (uint16_t)capped;
        fwdCmdPending.withResponse = true;
        fwdCmdPending.pending      = true;
    }
};

class PxStatusCallbacks : public NimBLECharacteristicCallbacks {
    void onRead(NimBLECharacteristic* chr, NimBLEConnInfo&) override {
        Serial.printf("[PROXY] STATUS read rocketConn=%d cacheValid=%d cacheLen=%u\n",
                      (int)rocketConnected, (int)statusCacheValid, statusCacheLen);
        // Serve from cache — never call readValue() from a stack callback.
        // bleProxyLoop() refreshes the cache every STATUS_CACHE_INTERVAL_MS.
        if (!statusCacheValid) {
            const char* err = rocketConnected ? "{\"proxy\":\"fetching\"}" : "{\"proxy\":\"no_rocket\"}";
            chr->setValue((uint8_t*)err, strlen(err));
            return;
        }
        chr->setValue(statusCache, statusCacheLen);
    }
};

class PxConnSetCallbacks : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic* chr, NimBLEConnInfo&) override {
        NimBLEAttValue v = chr->getValue();
        Serial.printf("[PROXY] CONNSET write %uB\n", (unsigned)v.size());
        size_t capped = v.size() > sizeof(fwdConnSetPending.buf) ? sizeof(fwdConnSetPending.buf) : v.size();
        memcpy(fwdConnSetPending.buf, v.data(), capped);
        fwdConnSetPending.len          = (uint16_t)capped;
        fwdConnSetPending.withResponse = false;
        fwdConnSetPending.pending      = true;
    }
};

class PxFetchCallbacks : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic* chr, NimBLEConnInfo&) override {
        NimBLEAttValue v = chr->getValue();
        Serial.printf("[PROXY] FETCH write %uB\n", (unsigned)v.size());
        pxFetchPending.pending = false;  // discard any in-flight fetch response
        size_t capped = v.size() > sizeof(fwdFetchPending.buf) ? sizeof(fwdFetchPending.buf) : v.size();
        memcpy(fwdFetchPending.buf, v.data(), capped);
        fwdFetchPending.len          = (uint16_t)capped;
        fwdFetchPending.withResponse = false;
        fwdFetchPending.pending      = true;
    }
};

class PxOtaCallbacks : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic* chr, NimBLEConnInfo&) override {
        NimBLEAttValue v = chr->getValue();
        Serial.printf("[PROXY] OTA write %uB\n", (unsigned)v.size());
        size_t capped = v.size() > sizeof(fwdOtaPending.buf) ? sizeof(fwdOtaPending.buf) : v.size();
        memcpy(fwdOtaPending.buf, v.data(), capped);
        fwdOtaPending.len          = (uint16_t)capped;
        fwdOtaPending.withResponse = false;
        fwdOtaPending.pending      = true;
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
        fwdCmdPending.pending = fwdConnSetPending.pending = false;
        fwdFetchPending.pending = fwdOtaPending.pending   = false;
        statusCacheValid = false;
        statusCacheLen   = 0;
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

        Serial.printf("[PROXY] adv: %s rssi=%d name='%s' svc_match=%d self=%d\n",
                      advAddr.c_str(), adv->getRSSI(), adv->getName().c_str(),
                      (int)adv->isAdvertisingService(NimBLEUUID(RKT_SVC_UUID)),
                      (int)(advAddr == ownAddr));

        if (advAddr == ownAddr) return;
        if (!adv->isAdvertisingService(NimBLEUUID(RKT_SVC_UUID))) return;

        Serial.printf("[PROXY] Found rocket candidate: %s  RSSI=%d\n",
                      advAddr.c_str(), adv->getRSSI());

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

    NimBLEDevice::setDeviceName(ROCKET_DEVICE_NAME);

    NimBLEAdvertising* advert = NimBLEDevice::getAdvertising();
    advert->addServiceUUID(RKT_SVC_UUID);
    advert->setName(ROCKET_DEVICE_NAME);
    advert->enableScanResponse(true);
    advert->setMinInterval(0x20);
    advert->setMaxInterval(0x40);
    advert->start(0);

    NimBLEScan* scan = NimBLEDevice::getScan();
    scan->setScanCallbacks(new PxScanCallbacks(), true);
    scan->setActiveScan(true);
    scan->setInterval(160);
    scan->setWindow(80);

    Serial.println("[PROXY] BLE proxy init done");
}

void bleProxyLoop() {
    // Drain rocket→phone notifies
    drainNotify(pxTelemChar, pxTelemPending);
    drainNotify(pxFetchChar, pxFetchPending);
    drainNotify(pxOtaChar,   pxOtaPending);

    // Drain phone→rocket writes (deferred out of server callbacks)
    drainWrite(rxCmdChar,     fwdCmdPending,     "CMD");
    drainWrite(rxConnSetChar, fwdConnSetPending, "CONNSET");
    drainWrite(rxFetchChar,   fwdFetchPending,   "FETCH");
    drainWrite(rxOtaChar,     fwdOtaPending,     "OTA");

    if (connectPending) {
        doConnect();
        return;
    }

    // Periodically refresh status cache from rocket — on Arduino task, never from a callback.
    if (rocketConnected && rxStatusChar) {
        unsigned long now = millis();
        if (now - statusLastFetchMs >= STATUS_CACHE_INTERVAL_MS) {
            statusLastFetchMs = now;
            NimBLEAttValue v = rxStatusChar->readValue();
            if (v.size() > 0) {
                size_t capped = v.size() > sizeof(statusCache) ? sizeof(statusCache) : v.size();
                memcpy(statusCache, v.data(), capped);
                statusCacheLen   = (uint16_t)capped;
                statusCacheValid = true;
            }
        }
    }

    if (!rocketConnected && !scanActive) {
        unsigned long now = millis();
        if (now - lastScanMs >= scanIntervalMs) {
            lastScanMs = now;
            scanActive = true;
            Serial.println("[PROXY] Scanning for rocket...");
            NimBLEDevice::getScan()->start(2000, false);
        }
    }
}

bool bleProxyRocketConnected() { return rocketConnected; }
bool bleProxyPhoneConnected()  { return phoneConnected; }
