// Two-connection BLE test — no proxy, no rocket.
// Advertises one NOTIFY characteristic. Sends 3 hardcoded bytes every 500ms
// to every subscribed connection. Connect two phones (or phone + nRF Connect)
// and verify both receive without crashing.
#include <Arduino.h>
#include <NimBLEDevice.h>
#include <WiFi.h>

#define SERIAL_BAUD 2000000

#define TEST_SVC_UUID  "12345678-1234-1234-1234-123456789abc"
#define TEST_CHAR_UUID "12345678-1234-1234-1234-123456789abd"

static NimBLECharacteristic* testChar = nullptr;
static uint8_t connCount = 0;

class SrvCB : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer*, NimBLEConnInfo& info) override {
    connCount++;
    Serial.printf("BLE+ handle=%u addr=%s  total=%u\n",
                  info.getConnHandle(), info.getAddress().toString().c_str(), connCount);
    // Keep advertising so a second device can still connect.
    NimBLEDevice::startAdvertising();
  }
  void onDisconnect(NimBLEServer*, NimBLEConnInfo& info, int reason) override {
    if (connCount) connCount--;
    Serial.printf("BLE- handle=%u reason=%d  total=%u\n",
                  info.getConnHandle(), reason, connCount);
    NimBLEDevice::startAdvertising();
  }
};

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(500);
  Serial.println("\n=== Two-connection BLE test ===");

  WiFi.mode(WIFI_OFF);

  NimBLEDevice::init("BLE-2conn-test");
  NimBLEDevice::setPower(ESP_PWR_LVL_P9);
  NimBLEDevice::setMTU(517);

  NimBLEServer* server = NimBLEDevice::createServer();
  server->setCallbacks(new SrvCB());

  NimBLEService* svc = server->createService(TEST_SVC_UUID);
  testChar = svc->createCharacteristic(TEST_CHAR_UUID, NIMBLE_PROPERTY::NOTIFY);
  svc->start();

  NimBLEAdvertising* adv = NimBLEDevice::getAdvertising();
  adv->addServiceUUID(TEST_SVC_UUID);
  adv->start();

  Serial.println("Advertising. Connect and subscribe to the NOTIFY char.");
}

void loop() {
  
  static unsigned long lastMs = 0;
  unsigned long now = millis();
  if (now - lastMs >= 500) {
    lastMs = now;
      if (!NimBLEDevice::getAdvertising()->isAdvertising()) {
    NimBLEDevice::getAdvertising()->start(0);
  }
    static uint8_t seq = 0;
    uint8_t payload[3] = { 0xAA, 0xBB, seq++ };
    bool ok = testChar->notify(payload, sizeof(payload));
    Serial.printf("[%lus] notify seq=%u ok=%d conns=%u\n",
                  now / 1000, (unsigned)(seq - 1), ok ? 1 : 0, connCount);
  }
}
