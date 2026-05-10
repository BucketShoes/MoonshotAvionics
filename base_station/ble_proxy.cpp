// ble_proxy.cpp — BLE range-extension proxy.
// See ble_proxy.h for design notes.

#include <Arduino.h>
#include <NimBLEDevice.h>
#include "ble_proxy.h"

// ===================== ROCKET SERVICE / CHARACTERISTIC UUIDS =====================
// Mirror of rocket_avionics/config.h — must stay in sync.

#define RKT_SVC_UUID       "524f434b-4554-5354-424c-000000000000"
#define RKT_TELEM_UUID     "524f434b-4554-5354-424c-000000000001"  // NOTIFY
#define RKT_CMD_UUID       "524f434b-4554-5354-424c-000000000002"  // WRITE | WRITE_NR
#define RKT_STATUS_UUID    "524f434b-4554-5354-424c-000000000003"  // READ
#define RKT_CONNSET_UUID   "524f434b-4554-5354-424c-000000000004"  // WRITE | WRITE_NR
#define RKT_LOGFETCH_UUID  "524f434b-4554-5354-424c-000000000005"  // WRITE | NOTIFY
#define RKT_OTA_UUID       "524f434b-4554-5354-424c-000000000006"  // WRITE | WRITE_NR | NOTIFY

#define ROCKET_DEVICE_NAME "Moonshot-Rocket"

// ===================== STATE =====================

// --- Server side (phone connects here) ---
static NimBLEServer*         pxServer      = nullptr;
static NimBLECharacteristic* pxTelemChar   = nullptr;  // NOTIFY → phone
static NimBLECharacteristic* pxCmdChar     = nullptr;  // WRITE ← phone
static NimBLECharacteristic* pxStatusChar  = nullptr;  // READ  ← phone
static NimBLECharacteristic* pxConnSetChar = nullptr;  // WRITE ← phone
static NimBLECharacteristic* pxFetchChar   = nullptr;  // WRITE | NOTIFY ↔ phone
static NimBLECharacteristic* pxOtaChar     = nullptr;  // WRITE | NOTIFY ↔ phone
static NimBLEAdvertising*    pxAdvert      = nullptr;

static volatile bool phoneConnected = false;
static uint16_t      phoneConnHandle = BLE_HS_CONN_HANDLE_NONE;

// --- Client side (connects to rocket) ---
static NimBLEClient*              pxClient      = nullptr;
static NimBLERemoteCharacteristic* rxTelemChar   = nullptr;
static NimBLERemoteCharacteristic* rxCmdChar     = nullptr;
static NimBLERemoteCharacteristic* rxStatusChar  = nullptr;
static NimBLERemoteCharacteristic* rxConnSetChar = nullptr;
static NimBLERemoteCharacteristic* rxFetchChar   = nullptr;
static NimBLERemoteCharacteristic* rxOtaChar     = nullptr;

static volatile bool rocketConnected = false;
static volatile bool scanActive      = false;

// Scan retry back-off
static unsigned long lastScanMs = 0;
static unsigned long scanIntervalMs = 5000;  // starts at 5 s, backs off to 30 s

// --- Backpressure state per notify channel ---
// If a phone-side notify() drops we hold the payload and retry before consuming
// the next rocket notification.  One buffer per channel (telem, logfetch, ota).

struct PendingNotify {
    uint8_t  buf[514];
    uint16_t len;
    bool     pending;
};
static PendingNotify pxTelemPending   = {};
static PendingNotify pxFetchPending   = {};
static PendingNotify pxOtaPending     = {};

// ===================== HELPERS =====================

static bool notifyPhone(NimBLECharacteristic* chr, PendingNotify& pn,
                        const uint8_t* data, size_t len) {
    if (!phoneConnected || !chr) return false;
    size_t capped = len > sizeof(pn.buf) ? sizeof(pn.buf) : len;
    if (!chr->notify(data, capped)) {
        // Queue for retry next loop.
        memcpy(pn.buf, data, capped);
        pn.len = (uint16_t)capped;
        pn.pending = true;
        return false;
    }
    return true;
}

static bool retryPending(NimBLECharacteristic* chr, PendingNotify& pn) {
    if (!pn.pending) return true;
    if (!phoneConnected || !chr) { pn.pending = false; return true; }
    if (!chr->notify(pn.buf, pn.len)) return false;
    pn.pending = false;
    return true;
}

// ===================== ROCKET NOTIFY CALLBACKS =====================
// These fire on the NimBLE task when the rocket sends a notification.
// We just forward to the phone — backpressure is handled in bleProxyLoop().

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

// Generic: forward write payload to the corresponding rocket characteristic.
static void forwardWrite(NimBLERemoteCharacteristic* rxChr, const uint8_t* data, size_t len, bool withResponse) {
    if (!rocketConnected || !rxChr) return;
    if (withResponse) rxChr->writeValue(data, len, true);
    else              rxChr->writeValue(data, len, false);
}

// ===================== SERVER CONNECT/DISCONNECT =====================
// Called from the unified server callbacks in main.cpp.

void bleProxyOnServerConnect(uint16_t connHandle) {
    phoneConnected  = true;
    phoneConnHandle = connHandle;
    Serial.printf("[PROXY] Phone connected handle=%u\n", connHandle);
}

void bleProxyOnServerDisconnect(uint16_t connHandle) {
    if (connHandle != phoneConnHandle) return;
    phoneConnected  = false;
    phoneConnHandle = BLE_HS_CONN_HANDLE_NONE;
    pxTelemPending.pending = false;
    pxFetchPending.pending = false;
    pxOtaPending.pending   = false;
    Serial.println("[PROXY] Phone disconnected");
}

// CMD: phone → rocket (WRITE with response — rocket reads result byte)
class PxCmdCallbacks : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic* chr, NimBLEConnInfo&) override {
        NimBLEAttValue v = chr->getValue();
        if (!rocketConnected || !rxCmdChar) return;
        // Forward with response so the rocket's result byte comes back.
        bool ok = rxCmdChar->writeValue(v.data(), v.size(), true);
        // Echo rocket's response value back to phone (or a nack if disconnected).
        if (ok) {
            NimBLEAttValue resp = rxCmdChar->getValue();
            chr->setValue(resp.data(), resp.size());
        } else {
            uint8_t nack = 0xFF;
            chr->setValue(&nack, 1);
        }
    }
};

// STATUS: phone reads → we read from rocket and return it.
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

// CONNSET: phone writes → forward to rocket (WRITE_NR, no response expected).
class PxConnSetCallbacks : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic* chr, NimBLEConnInfo&) override {
        NimBLEAttValue v = chr->getValue();
        forwardWrite(rxConnSetChar, v.data(), v.size(), false);
        // Note: if phone sends PHY change via connset, rocket will call
        // updatePhy() on the rocket↔base link.  The base station (central role)
        // accepts incoming PHY update requests automatically.  The base↔phone
        // link is not affected.
    }
};

// LOGFETCH: phone writes request → rocket; rocket streams notifications back
// via onRocketFetch() → phone.
class PxFetchCallbacks : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic* chr, NimBLEConnInfo&) override {
        NimBLEAttValue v = chr->getValue();
        pxFetchPending.pending = false;  // cancel any in-flight retry
        forwardWrite(rxFetchChar, v.data(), v.size(), false);
    }
};

// OTA: phone writes chunks → rocket; rocket sends progress notifications back.
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
    }
    void onDisconnect(NimBLEClient* client, int reason) override {
        rocketConnected = false;
        rxTelemChar = rxCmdChar = rxStatusChar = nullptr;
        rxConnSetChar = rxFetchChar = rxOtaChar = nullptr;
        Serial.printf("[PROXY] Rocket BLE disconnected reason=%d\n", reason);
        // Back off scan retry exponentially up to 30 s.
        scanIntervalMs = min(scanIntervalMs * 2, (unsigned long)30000);
        lastScanMs = millis();
    }
};

// ===================== SCAN CALLBACKS =====================

class PxScanCallbacks : public NimBLEScanCallbacks {
    void onResult(const NimBLEAdvertisedDevice* adv) override {
        if (adv->getName() != ROCKET_DEVICE_NAME) return;
        Serial.printf("[PROXY] Found rocket: %s  RSSI=%d\n",
                      adv->getAddress().toString().c_str(), adv->getRSSI());
        NimBLEDevice::getScan()->stop();
        scanActive = false;

        if (!pxClient) {
            pxClient = NimBLEDevice::createClient();
            pxClient->setCallbacks(new PxClientCallbacks());
        }

        // Connect.  Request coded PHY if supported; ESP32-S3/C3 support it.
        // NimBLE will negotiate down to 1M if the peer can't do coded.
        // Prefer coded PHY; fall back to 1M if rocket doesn't support it yet.
        pxClient->setConnectPhy(BLE_GAP_LE_PHY_CODED_MASK | BLE_GAP_LE_PHY_1M_MASK);

        if (!pxClient->connect(adv)) {
            Serial.println("[PROXY] connect() failed");
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

        // Subscribe to the three notification characteristics.
        if (!rxTelemChar->subscribe(true, onRocketTelem)) {
            Serial.println("[PROXY] telem subscribe failed");
            pxClient->disconnect();
            return;
        }
        if (rxFetchChar)  rxFetchChar->subscribe(true, onRocketFetch);
        if (rxOtaChar)    rxOtaChar->subscribe(true, onRocketOta);

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
    // --- Server: add rocket proxy service to the existing shared NimBLE server.
    // NimBLE has one server instance; initBLE() already created it with
    // BleServerCallbacks.  We replace server callbacks with a unified one
    // (installed from main.cpp via bleProxyInstallServerCallbacks()) that handles
    // both base-station and proxy connect/disconnect state.
    pxServer = NimBLEDevice::getServer();
    if (!pxServer) pxServer = NimBLEDevice::createServer();  // shouldn't happen

    NimBLEService* svc = pxServer->createService(RKT_SVC_UUID);

    pxTelemChar   = svc->createCharacteristic(RKT_TELEM_UUID,   NIMBLE_PROPERTY::NOTIFY);
    pxCmdChar     = svc->createCharacteristic(RKT_CMD_UUID,     NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR);
    pxStatusChar  = svc->createCharacteristic(RKT_STATUS_UUID,  NIMBLE_PROPERTY::READ);
    pxConnSetChar = svc->createCharacteristic(RKT_CONNSET_UUID, NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR);
    pxFetchChar   = svc->createCharacteristic(RKT_LOGFETCH_UUID, NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::NOTIFY);
    pxOtaChar     = svc->createCharacteristic(RKT_OTA_UUID,     NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR | NIMBLE_PROPERTY::NOTIFY);

    pxCmdChar->setCallbacks(new PxCmdCallbacks());
    pxStatusChar->setCallbacks(new PxStatusCallbacks());
    pxConnSetChar->setCallbacks(new PxConnSetCallbacks());
    pxFetchChar->setCallbacks(new PxFetchCallbacks());
    pxOtaChar->setCallbacks(new PxOtaCallbacks());

    svc->start();

    // --- Advertise rocket UUID + device name ---
    pxAdvert = NimBLEDevice::getAdvertising();
    pxAdvert->addServiceUUID(RKT_SVC_UUID);
    pxAdvert->setName(ROCKET_DEVICE_NAME);
    pxAdvert->enableScanResponse(true);
    pxAdvert->setMinInterval(0x20);
    pxAdvert->setMaxInterval(0x40);

    // --- Scan config ---
    NimBLEScan* scan = NimBLEDevice::getScan();
    scan->setScanCallbacks(new PxScanCallbacks(), false);
    // Scan on both 1M and coded PHY — finds the rocket regardless of which it advertises on.
    scan->setPhy(NimBLEScan::Phy::SCAN_ALL);
    scan->setActiveScan(false);  // passive — we know what we're looking for
    scan->setInterval(200);
    scan->setWindow(50);

    Serial.println("[PROXY] BLE proxy init done");
}

void bleProxyLoop() {
    // Retry any dropped phone-side notifications first.
    retryPending(pxTelemChar,  pxTelemPending);
    retryPending(pxFetchChar,  pxFetchPending);
    retryPending(pxOtaChar,    pxOtaPending);

    // Trigger a scan if not connected and not already scanning.
    if (!rocketConnected && !scanActive) {
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
