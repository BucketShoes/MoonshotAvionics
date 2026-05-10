// ble_proxy.h — BLE range-extension proxy for base station.
//
// Scans for "Moonshot-Rocket" by device name, connects as a GATT client on
// coded PHY, and mirrors the rocket's entire service (UUIDs 0001-0006) as a
// GATT server so a phone can connect as if it were talking directly to the
// rocket.  All characteristic data is forwarded verbatim in both directions —
// the phone sees the rocket's UUIDs and the rocket sees a client that behaves
// like a phone.
//
// Backpressure: if the phone-side notify() drops (host TX queue full), we
// stop consuming rocket notifications until the retry succeeds.  The rocket's
// own flow control then stalls naturally because its notify() will also start
// dropping (phone is the indirect consumer of that queue depth too).
//
// Loop-avoidance: we scan by DEVICE NAME "Moonshot-Rocket", not by service
// UUID.  Two base stations both advertise the rocket UUID but neither
// advertises that name, so they will never connect to each other.

#pragma once
#include <Arduino.h>

// Call once during setup, after NimBLEDevice::init().
void bleProxyInit();

// Call every loop iteration.
void bleProxyLoop();

// True if currently connected to the rocket as a BLE client.
bool bleProxyRocketConnected();

// True if a phone is connected to the proxy server.
bool bleProxyPhoneConnected();

// Called by the unified BLE server callbacks in main.cpp so the proxy can
// track phone connect/disconnect without owning the server callbacks slot.
void bleProxyOnServerConnect(uint16_t connHandle);
void bleProxyOnServerDisconnect(uint16_t connHandle);
