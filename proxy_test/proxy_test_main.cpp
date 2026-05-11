// Minimal BLE proxy test — no radio, no WiFi, no log store.
// Verifies that the ESP32-S3 hardware can sustain a BLE dual-role proxy
// (client to rocket + server to phone) in isolation.
#include <Arduino.h>
#include <NimBLEDevice.h>
#include <WiFi.h>
#include "ble_proxy.h"

#define SERIAL_BAUD 2000000

// BLE server callbacks — required by ble_proxy.h contract.
class BleServerCallbacks : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer*, NimBLEConnInfo& connInfo) override {
    bleProxyOnServerConnect(connInfo.getConnHandle());
    Serial.printf("BLE+ handle=%u addr=%s\n", connInfo.getConnHandle(),
                  connInfo.getAddress().toString().c_str());
  }
  void onDisconnect(NimBLEServer*, NimBLEConnInfo& connInfo, int reason) override {
    bleProxyOnServerDisconnect(connInfo.getConnHandle());
    Serial.printf("BLE- handle=%u reason=%d\n", connInfo.getConnHandle(), reason);
    NimBLEDevice::startAdvertising();
  }
};

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(500);
  Serial.println("\n=== BLE Proxy Test ===");

  WiFi.mode(WIFI_OFF);

  NimBLEDevice::init("Moonshot-Rocket");
  NimBLEDevice::setPower(ESP_PWR_LVL_P9);
  NimBLEDevice::setMTU(517);

  NimBLEServer* server = NimBLEDevice::createServer();
  server->setCallbacks(new BleServerCallbacks());

  bleProxyInit();

  Serial.println("=== Ready ===");
}

void loop() {
  bleProxyLoop();

  static unsigned long lastStatusMs = 0;
  unsigned long now = millis();
  if (now - lastStatusMs >= 5000) {
    lastStatusMs = now;
    Serial.printf("[STATUS] rocket=%d phone=%d uptime=%lus\n",
                  (int)bleProxyRocketConnected(),
                  (int)bleProxyPhoneConnected(),
                  now / 1000);
  }
}
