/**
 * ble_adapter.js — multi-connection BLE abstraction
 *
 * Usage:
 *   const conn = NativeBLE.getConnection('base');   // or 'rocket', any string
 *   conn.scan(serviceUUID, onFound);                // onFound(addr, name, rssi, advBytes) per result
 *   const svc = await conn.connect(addr, svcUUID);  // stops scan, returns FakeGattService
 *   const char = await svc.getCharacteristic(uuid); // FakeChar
 *   await char.startNotifications();
 *   char.addEventListener('characteristicvaluechanged', ev => { ev.target.value });  // DataView
 *   char.readValue()                                // Promise<DataView>
 *   char.writeValueWithResponse(buf)               // Promise<void>
 *   char.writeValueWithoutResponse(buf)            // Promise<void>
 *   conn.disconnect();
 *   conn.readRssi()                                // Promise<number|null>
 *
 * NativeBLE.isNative — true when window.AndroidBLE is present, false in Chrome
 *
 * On Chrome: conn.scan() opens the browser picker (onFound called once on selection).
 *            rssi and advBytes are null on Chrome (not available via Web Bluetooth).
 * On Android: scan fires onFound for every device found; continues until connect() or stopScan().
 *             All packets delivered per-characteristic with no batching.
 */

const NativeBLE = (() => {
  const _native   = window.AndroidBLE;
  const _isNative = !!_native;

  // Map<connId, ConnectionObject>
  const _connections = new Map();

  // ── Utility helpers ───────────────────────────────────────────────────────

  function _b64ToBuffer(b64) {
    const bin = atob(b64);
    const buf = new ArrayBuffer(bin.length);
    const u8  = new Uint8Array(buf);
    for (let i = 0; i < bin.length; i++) u8[i] = bin.charCodeAt(i);
    return buf;
  }

  function _bufToHex(bufferSource) {
    const u8 = bufferSource instanceof ArrayBuffer
      ? new Uint8Array(bufferSource)
      : new Uint8Array(bufferSource.buffer, bufferSource.byteOffset, bufferSource.byteLength);
    return Array.from(u8).map(b => b.toString(16).padStart(2, '0')).join('');
  }

  // ── Global native callbacks — dispatched by connId ────────────────────────

  window.onBLEDeviceFound = (connId, addr, name, rssi, advB64) => {
    const conn = _connections.get(connId);
    if (!conn) return;
    let advBytes = null;
    if (advB64) {
      const bin = atob(advB64);
      advBytes = new Uint8Array(bin.length);
      for (let i = 0; i < bin.length; i++) advBytes[i] = bin.charCodeAt(i);
    }
    conn._onDeviceFound?.(addr, name, rssi, advBytes);
  };

  window.onBLEConnected = (connId) => {
    _connections.get(connId)?._resolveConnect?.();
  };

  window.onBLEDisconnected = (connId) => {
    const conn = _connections.get(connId);
    if (!conn) return;
    // Clear stale GATT state so the same conn object can be reused for a reconnect
    conn._fakeChars.clear();
    conn._resolveConnect = null;
    conn._rejectConnect  = null;
    conn._resolveRssi    = null;
    conn._onDisconnected?.();
  };

  window.onBLEError = (connId, msg) => {
    const conn = _connections.get(connId);
    if (!conn) return;
    const reject = conn._rejectConnect;
    conn._resolveConnect = null;
    conn._rejectConnect  = null;
    conn._fakeChars.clear();
    if (reject) reject(new Error(msg));
    else conn._onError?.(msg);
  };

  // One packet per char — route to correct FakeChar and fire synthetic event
  window.onBLECharacteristic = (connId, charUUID, b64) => {
    const conn = _connections.get(connId);
    if (!conn) return;
    const char = conn._fakeChars.get(charUUID.toLowerCase());
    if (!char) return;
    char._dispatchValue(_b64ToBuffer(b64));
  };

  // Single-shot read result
  window.onBLEReadResult = (connId, charUUID, b64) => {
    const conn = _connections.get(connId);
    if (!conn) return;
    const char = conn._fakeChars.get(charUUID.toLowerCase());
    if (!char) return;
    char._resolveRead(b64 ? _b64ToBuffer(b64) : null);
  };

  window.onBLERssi = (connId, rssi) => {
    const conn = _connections.get(connId);
    if (!conn) return;
    conn._resolveRssi?.(rssi);
    conn._resolveRssi = null;
  };

  // ── FakeChar ──────────────────────────────────────────────────────────────
  //
  // Native path: startNotifications() triggers subscribe to Kotlin.
  //              readValue() issues readChar to Kotlin, waits for onBLEReadResult.
  // Web path:    thin wrapper around the real BluetoothRemoteGATTCharacteristic.

  function _makeFakeCharNative(connId, svcUUID, charUUID) {
    const _listeners = new Map(); // type → Set<handler>
    const _readQueue = [];        // {resolve, reject}[]

    const char = {
      uuid: charUUID,

      startNotifications() {
        _native.subscribe(connId, svcUUID, charUUID);
        return Promise.resolve(char);
      },

      addEventListener(type, handler) {
        if (!_listeners.has(type)) _listeners.set(type, new Set());
        _listeners.get(type).add(handler);
      },

      removeEventListener(type, handler) {
        _listeners.get(type)?.delete(handler);
      },

      readValue() {
        return new Promise((resolve, reject) => {
          _readQueue.push({ resolve, reject });
          _native.readChar(connId, svcUUID, charUUID);
        });
      },

      writeValueWithResponse(bufferSource) {
        _native.write(connId, svcUUID, charUUID, _bufToHex(bufferSource));
        return Promise.resolve();
      },

      writeValueWithoutResponse(bufferSource) {
        _native.write(connId, svcUUID, charUUID, _bufToHex(bufferSource));
        return Promise.resolve();
      },

      // Called by onBLECharacteristic dispatcher
      _dispatchValue(arrayBuffer) {
        const syntheticEv = { target: { value: new DataView(arrayBuffer) } };
        for (const h of (_listeners.get('characteristicvaluechanged') ?? [])) {
          try { h(syntheticEv); } catch(e) { console.error('[BLE adapter] handler error', e); }
        }
      },

      // Called by onBLEReadResult dispatcher
      _resolveRead(arrayBuffer) {
        const entry = _readQueue.shift();
        if (!entry) return;
        if (arrayBuffer === null) entry.reject(new Error('BLE read failed'));
        else entry.resolve(new DataView(arrayBuffer));
      }
    };

    return char;
  }

  function _makeFakeCharWeb(realChar) {
    return {
      uuid: realChar.uuid,
      startNotifications:        ()    => realChar.startNotifications(),
      addEventListener:          (t,h) => realChar.addEventListener(t, h),
      removeEventListener:       (t,h) => realChar.removeEventListener(t, h),
      readValue:                 ()    => realChar.readValue(),
      writeValueWithResponse:    (b)   => realChar.writeValueWithResponse(b),
      writeValueWithoutResponse: (b)   => realChar.writeValueWithoutResponse(b),
    };
  }

  // ── FakeGattService ───────────────────────────────────────────────────────

  function _makeFakeServiceNative(connId, svcUUID, conn) {
    return {
      getCharacteristic(charUUID) {
        const uuid = charUUID.toLowerCase();
        if (conn._fakeChars.has(uuid)) return Promise.resolve(conn._fakeChars.get(uuid));
        const char = _makeFakeCharNative(connId, svcUUID, uuid);
        conn._fakeChars.set(uuid, char);
        return Promise.resolve(char);
      }
    };
  }

  function _makeFakeServiceWeb(realSvc) {
    return {
      async getCharacteristic(charUUID) {
        return _makeFakeCharWeb(await realSvc.getCharacteristic(charUUID));
      }
    };
  }

  // ── ConnectionObject ──────────────────────────────────────────────────────

  function _makeConnection(connId) {
    const conn = {
      isNative: _isNative,

      // Callbacks set by scan/connect callers
      _onDeviceFound:  null,
      _onDisconnected: null,
      _onError:        null,
      _resolveConnect: null,
      _rejectConnect:  null,
      _resolveRssi:    null,

      // FakeChar registry — charUUID(lowercase) → FakeChar
      _fakeChars: new Map(),

      // Web Bluetooth state
      _webDevice: null,

      /**
       * Start scanning. onFound(addr, name, rssi, advBytes) fires per result.
       * On native: continuous until connect() or stopScan() is called.
       * On Chrome: browser picker fires once, rssi/advBytes are null.
       * Returns a stopScan function.
       */
      scan(svcUUID, onFound) {
        conn._onDeviceFound = onFound;
        if (_isNative) {
          _native.scan(connId, svcUUID);
          return () => _native.stopScan(connId);
        } else {
          if (!navigator.bluetooth) {
            onFound && setTimeout(() => conn._onError?.('Web Bluetooth not available'), 0);
            return () => {};
          }
          navigator.bluetooth.requestDevice({
            filters: [{ services: [svcUUID] }]
          }).then(device => {
            conn._webDevice = device;
            device.addEventListener('gattserverdisconnected', () => conn._onDisconnected?.());
            onFound?.(device.id, device.name ?? 'Unknown', null, null);
          }).catch(e => conn._onError?.(e.message));
          return () => {};
        }
      },

      /**
       * Connect to a scanned device. Also stops the scan.
       * Returns Promise<FakeGattService>.
       */
      connect(address, svcUUID) {
        if (_isNative) {
          return new Promise((resolve, reject) => {
            conn._resolveConnect = () => {
              conn._resolveConnect = null;
              conn._rejectConnect  = null;
              resolve(_makeFakeServiceNative(connId, svcUUID, conn));
            };
            conn._rejectConnect = (e) => {
              conn._resolveConnect = null;
              conn._rejectConnect  = null;
              reject(e);
            };
            _native.connect(connId, address); // Kotlin calls stopScan internally
          });
        } else {
          return (async () => {
            const server = await conn._webDevice.gatt.connect();
            const realSvc = await server.getPrimaryService(svcUUID);
            return _makeFakeServiceWeb(realSvc);
          })();
        }
      },

      disconnect() {
        if (_isNative) {
          _native.disconnect(connId);
        } else {
          if (conn._webDevice?.gatt?.connected) conn._webDevice.gatt.disconnect();
        }
        conn._onDeviceFound = null;
        conn._onDisconnected = null;
        conn._onError = null;
        conn._resolveConnect = null;
        conn._rejectConnect = null;
        conn._resolveRssi = null;
        conn._fakeChars.clear();
        conn._webDevice = null;
        _connections.delete(connId);
      },

      readRssi() {
        if (_isNative) {
          return new Promise(resolve => {
            conn._resolveRssi = resolve;
            _native.readRssi(connId);
          });
        }
        return Promise.resolve(null);
      }
    };

    return conn;
  }

  // ── Public API ────────────────────────────────────────────────────────────

  return {
    isNative: _isNative,

    getConnection(connId) {
      if (!_connections.has(connId)) _connections.set(connId, _makeConnection(connId));
      return _connections.get(connId);
    }
  };
})();
