# Moonshot V1 — flight analysis, 2026-09-10 (session 36, log #474182, 2263 records)

One-off analysis scripts for the last V1 flight. Not a viewer, not maintained —
V2 logs at 8 kHz to NAND with a different record format, so this is throwaway.

## Flight summary

AeroTech H135W-14A, 432 g liftoff / 350 g burnout, launched from
-31.1577173, 149.9282093 (303 m MSL per map; barometer's ISA pressure
altitude read 263 m, a QNH offset of about 40 m).

| | |
|---|---|
| Apogee | 2019 m AGL, 17.15 s after first motion |
| Max speed | ~380 m/s (Mach ~1.11) for a nominal burn — see caveat below |
| Peak acceleration | ~38 g inferred (accelerometer railed at 16.4 g; not measured) |
| Total impulse | ~226 Ns, within about ±20% of nominal. Burn *duration* is not recoverable |
| Subsonic drag area | 4.5e-4 m² climbing, 6.4e-4 m² descending (Cd 0.50 / 0.70 on the 34 mm body) |
| Terminal velocity | ~100 m/s, nose-down, ballistic |
| Drogue fired | T+17.4 s (0.9 s after apogee) — apogee detect worked |
| Main | never fired; would have triggered at ground level (see below) |
| Landing | -31.1631022, 149.9317184; 686 m from pad, bearing 151 deg |
| Log ends | T+43.51 s, 86 m AGL indicated = ~21 m AGL true, ~0.2 s before impact |

## Method

The barometer is unusable on the way up — a static-port airspeed error of
roughly 0.10-0.15 of dynamic pressure, peaking above 1100 m of false altitude
near Mach 0.9. The accelerometer rails at 16.4 g through the whole boost.
So the ascent is reconstructed by:

1. Anchoring on apogee altitude, which *is* trustworthy — the airspeed error
   vanishes when the rocket stops. Confirmed independently: a ballistic fall
   from 2019 m at 100 m/s terminal takes 26.4 s; measured 26.36 s.
2. Back-integrating the clean (unsaturated, post-vibration) coast accelerometer
   data from apogee to get a model-free velocity and altitude at T+3.5 s.
3. Fitting a drag area to a published H135W thrust curve so the forward
   simulation reproduces both. The fitted CdA (4.54e-4 m^2) matches the value
   derived independently from the coast deceleration (4.6e-4 m^2).

### What this does and does not pin down

Apogee altitude, apogee time and the subsonic drag areas are solid. Impulse is
good to about +/-20%, because the required value trades against the assumed
transonic drag rise (`tradeoff.py`). **Burn duration is not recoverable at all** --
the accelerometer is railed or vibration-aliased from ignition to T+2.9 s, which
covers every candidate burnout. Max speed depends on it: 377 m/s for a nominal
2.07 s burn, 517 m/s if the motor dumped the same impulse in 0.85 s
(`fastburn.py`). Apogee time mildly favours the longer burn but does not settle it.
Logging page 0x0E would have settled it outright.

The ascent/descent drag asymmetry is *required*, not fitted: using the descent
drag area for the ascent puts apogee 1.6 s early, far outside error. The gyro
gives the reason -- 3-50 deg/s of pitch/yaw coasting up, 100-340 deg/s coming
down.

## Scripts

| File | Does |
|---|---|
| `parse.py` | Frames the raw `.bin` (`[rec u32][len u8][snr i8][ts u32][payload]`), reports gaps and a page histogram |
| `series.py` | Decodes the flight window into `flight.csv`, one row per log record |
| `motor.py` | Integrates the H135W thrust curve (229 Ns over 2.072 s) |
| `anchor.py` | Model-free back-integration from apogee — the core result |
| `axial.py` | Compares vector-magnitude vs axial-only integration (roll centripetal inflates the vector) |
| `check.py` | Descent ballistics sanity check and GPS geometry |
| `final2.py` | Full reconstruction, Mach profile, static-port error vs Mach |
| `roll.py` | Attempt to recover the railed roll rate from transverse accel — **inconclusive, kept as a negative result** |
| `tradeoff.py` | Impulse vs transonic-drag trade; shows apogee time rules out a symmetric-drag solution |
| `fastburn.py` | Burn-duration sensitivity, and the airspeed correction on the final logged altitude |
| `charge.py` | Searches the apogee window for an ejection-charge pressure or shock transient (none found) |
| `gpsalt.py` | GPS altitude against airspeed-corrected barometric altitude |

Usage: `MOONSHOT_BIN=path/to/moonshot-fetch.bin python3 parse.py`, then
`python3 series.py` to produce `flight.csv`, then the rest read that CSV.

## Firmware issues this flight exposed

1. **No drogue-failure path.** `PHASE_DROGUE` only calls `checkMainDeploy()`,
   which is a pure altitude test. If the drogue fails there is nothing that
   deploys the main early.
2. **Main-deploy threshold is unreachable at ballistic descent rates.** The
   test uses the 1-second baro EMA, which lags a steady descent by
   (time constant x rate) = 100 m at 100 m/s. A 100 m AGL threshold therefore
   triggers at ground level.
3. **`peaks.maxAltCmMSL` tracks GPS altitude** (`main.cpp:332`, existing TODO),
   not fusion or baro. With no GPS fix it logged garbage (1474 m from a
   descent glitch, vs the real 2019 m).
4. **`peaks.maxAccel100g` and `maxVvel10` are also meaningless here** — the
   first is a vector magnitude of railed axes, the second is the baro EMA
   during the transonic excursion (1754 m/s).
5. **Gyro and transverse accel are aliased.** Roll railed the ITG3200
   (>=2280 deg/s) and the 1 Hz gyro / 10 Hz accel log rates cannot recover it.
6. **Page 0x0E is silently dropped.** `flight.cpp:257` sets `thrustLogForce` on
   coast entry and `nonblockingLogging()` honours it, but `dispatchBuildPage()`
   has **no `case 0x0E`**, so `logPage()` builds zero bytes and writes nothing --
   and the force flag is consumed either way, so it never retries. Zero 0x0E
   records exist in 340,000. Note that simply adding the case would smash the
   stack: `logPage()` uses `uint8_t buf[32]` and a 230-sample page needs ~237
   bytes.
7. **Nothing forces a log record on a pyro event** — `freshMask` is set but
   `nonblockingLogging()` is purely interval-driven (existing TODO). Fires are
   only caught by the 10 Hz telemetry header's fired bits.
