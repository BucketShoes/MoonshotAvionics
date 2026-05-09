# Moonshot Avionics — Project Context for Claude

## What this is

A model rocket avionics and telemetry system built around ESP32-S3 (rocket) and ESP32 (base station). Communicates via SX1262 LoRa radio (AU915 channel plan). A web dashboard (JavaScript/HTML) runs on the base station and in-browser via BLE. This is a personal/hobby project, currently unflown but bench-tested before each change.

## Safety context

Most of this codebase is cosmetic or tracking-only (telemetry display, logging, voice callouts). However:

- **Pyro channels exist** (parachute ejection charges) and are triggered automatically by flight phase logic. Bugs here have real consequences. (Setting off the charge near someone, or failing to deploy and landing on someone)
- **The pyro decision path is**: `sensors.h/cpp` → `flight.h/cpp` (phase state machine) → pyro GPIO. Flight phase data also feeds into `telemetry.h/cpp` (logged and transmitted).
- **Project scope: parachute deployment only.** Active aerodynamic control / TVC is no longer a goal. The tight timing determinism that target required is no longer needed. Pyro for parachute deployment tolerates ms-scale loop variation. (Older comments and memories that assert active-control constraints are obsolete.)
- **Flight warnings are noted in `flight.h` and `flight.cpp`** but the sensors feeding those decisions are upstream.
- **Arming is required** before any automatic pyro action. Ground test firing is command-only (FIRE PYRO command, while armed).
- **When touching flight logic or sensor data used in flight decisions**, apply extra care and flag any tradeoffs or edge cases explicitly, right at the end of the message so they dont get missed (in addition to inline discussion)

## Loop timing rule

Pyro for parachute deployment at apogee/descent is the only safety-critical loop work. It tolerates ms-scale variation, and sensors must be read with regular cadence to decide to deploy. Budget:

- **Typical loop iteration: ~1ms is fine.** No need to chase µs.
- **Worst-case iteration: up to ~10ms is acceptable.**
- **Anything that can exceed 10ms** must be gated behind `!isArmed()` or implemented as a state machine.
- **Anything over ~1ms** still goes in the timing budget list in `main.cpp` so the worst-case sum stays auditable. Sub-ms work doesn't need to be listed.

This is a deliberate relaxation from the older "1ms hard ceiling / 1000Hz loop" rule. That rule existed for active aero control, which is no longer a project goal. Reliability of telemetry and recovery (don't lose the rocket) now outweighs µs-precision.

Implications:
- Libraries with internal blocking waits up to ~10ms (e.g. RadioLib) are acceptable. Don't reject them on timing grounds.
- BLE runs permanently during flight — no longer needs the "disable if needed" caveat.
- Don't propose elaborate non-blocking state machines to shave µs off a sub-10ms operation.
- Total per-loop worst-case still must stay under 10ms. If multiple ≥1ms operations could line up in a single iteration, that needs attention.
- `executeLogDownload()` is the only intentionally fully-blocking call and is still refused while armed.
- Sector erases in `nonblockingLogging()` (~30-50ms) are still over budget — still a known defect, lower priority than before.

## Reception-preserving (non-preemptive) slot scheduler

The radio slot machine is **non-preemptive**: once RX is issued, only the radio's own timeout or an RxDone/CRC IRQ ends it — software never aborts in-flight reception. Slots are deliberately small relative to airtime; long packets routinely span 2–4 slots.

Do **not** call `radioStandby()` / `bsRadioStandby()` from the slot-machine path "to be safe" before a new RX/TX — that aborts whatever the radio was doing and produces garbage payloads. Standby is reserved for init, wedged-recovery (N consecutive overruns), `WIN_OFF`, and explicit policy transitions. Issuing `SetRx`/`SetTx` while the chip is still in a previous action is expected; the chip rejects with non-OK status and we just count + retry next slot.

The `BUSY` GPIO is "command-in-flight," not "radio-in-use" — it goes low once TX/RX starts. RX timeouts must subtract `BS_RX_TAIL_GUARD_US` so the chip's own timeout fires before the next slot's action.

## System overview

```
rocket_avionics/         ESP32-C3 firmware (primary focus)
base_station/            ESP32 base station firmware
docs/                    Web dashboard (JS/HTML/CSS)
design documents/        Specifications — see index below
```

### Rocket firmware modules (`rocket_avionics/`)

| File | Owns |
|---|---|
| `config.h` | All compile-time constants, pin numbers, protocol codes |
| `globals.h` | Cross-module extern declarations |
| `sensors.h/.cpp` | IMU drivers: ADXL345, ITG3200, VCM5883L, BMP280 |
| `flight.h/.cpp` | Flight phase state machine, arming, pyro decisions |
| `telemetry.h/.cpp` | Packet builders (0x01–0x0D pages), flash log writer |
| `commands.h/.cpp` | HMAC auth, nonce replay protection, command dispatch |
| `radio.h/.cpp` | SX1262 TX/RX state machine, CSMA, scheduling |
| `gps.h/.cpp` | NMEA parsing, GPS state |
| `log_store.h` | Flash ring buffer, index partition |
| `rocket_avionics.ino` | `setup()`, `loop()`, init state machine, globals |

Dependency order (no cycles): `config.h` → `globals.h` → sensor/gps modules → `flight.h` → `telemetry.h`, `commands.h`, `radio.h`

## Design documents index

Pull the relevant doc into the conversation when working in that area.

| Doc | When to consult |
|---|---|
| `Code architecture - Avionics.md` | Adding/moving files, understanding module boundaries, dependency questions |
| `flight phases.txt` | Any change to arming, launch detect, apogee, pyro triggers, landing detection |
| `Packet, log, and data formats.txt` | Wire protocol, data page formats, BLE GATT services, log record layout |
| `Lora command listing.txt` | Adding/modifying commands, HMAC/nonce handling, download protocol |
| `Ring buffer layout and log storage format.txt` | Flash storage, index partition, boot recovery, log erase |
| `Channel mapping.txt` | LoRa frequencies, future hopping system, GPS time sync |
| `avionics structure.txt` | Simple vs fancy flight model distinction (pyro uses simple model only) |
| `JS notes.txt` | Web dashboard: session rebuilding, log history loading, packet decoding |
| `voice callouts.txt` | Voice announcement logic and abbreviated number formatting |

## Key design decisions to preserve

- **Simple model only for flight decisions**: barometric EMA + accelerometer threshold. The Kalman/fusion data is for post-flight analysis and telemetry display only. Never route fancy-model data into pyro or phase decisions.
- **Two baro EMAs**: slow (30s period, arming/calibration), fast (1s period, flight decisions and telemetry header). Both always running.
- **Disarm semantics**: clears armed flag and sets phase to IDLE only. Does NOT reset EMAs, ground level, peaks, orientation, or launch time. Re-arming resets everything.
- **Arming stability**: 10-second continuous window. One bad reading resets that sensor's timer. Any sensor error in the last 10s blocks arming (except gyro/mag/GPS which are tracked but not required).
- **B+C launch detect window**: 5 seconds (not 10 — the spec doc has an uncorrected old value).
- **ARM command**: 0 bytes = force arm with defaults; 9 bytes = `uint16 boostAccelMg, uint16 boostAltM, uint16 coastAltM, uint16 mainDeployAltM, uint8 flags` (bit 0 = force).
- **State flags [9:11]**: `baro_ok`, `accel_ok`, `arm_ready` (not reserved — the packet format doc has an older version).
- **Fusion altitude in telemetry header**: 1-second baro EMA in metres MSL, not raw baro or GPS. `-32768` = invalid.
- **Log protection point**: set on ARM at 60 seconds before arm time. Ring buffer stops logging when full (does not overwrite protected region).
- **executeLogDownload() refused while armed** — it is fully blocking and bandwidth-heavy.
- **Relays**: All base stations are also relays (not yet implemented). Reference to a base station is the one the user is connected to. References to relays are other identical base stations placed far away acting in their role as a relay, sending traffic on backhaul.

## Known TODOs in the codebase

- Rocket BLE char 0002 result reporting is noted as wrong in the packet format doc (TODO `@@@@`).
- Rocket BLE char 0005 log fetch is missing timestamp and SNR per record (TODO `@@@`).
- SET RELAY RADIO command (0x30) BW param is unresolved (TODO in command doc).

## What's not yet documented (gaps)

- **Base station firmware** has no architecture doc equivalent to `Code architecture - Avionics.md`.
- **BLE implementation** (`ble.h/.cpp`) has only `rocket ble.md` — no equivalent base station BLE doc.
- **Hardware**: no doc covering ESP32-S3 memory limits, I2C bus layout, pin assignments (partially in `config.h`), or SEN0140 v2 IMU board specifics.
- **Test/validation approach**: no doc on how bench testing is done, what a pre-flight check looks like, or what regression testing exists.
- **Relay devices**: mentioned in commands and backhaul packet format but no architecture doc.

## Keep updated
- When changing functionality, consider if it should also require an update to the design docs but do not make frivolous changes.
- If this CLAUDE.md file falls out of sync with a designs, update it.