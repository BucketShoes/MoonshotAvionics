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
| Max speed | ~380 m/s (Mach ~1.11), 1.57 s after first motion |
| Peak acceleration | ~38 g (accelerometer railed at 16.4 g) |
| Burnout | 2.072 s, ~318 m/s, ~540 m AGL — **motor delivered full impulse** |
| Terminal velocity | ~100 m/s, nose-down, ballistic |
| Drogue fired | T+17.4 s (0.9 s after apogee) — apogee detect worked |
| Main | never fired; would have triggered at ground level (see below) |
| Landing | -31.1631022, 149.9317184; 686 m from pad, bearing 151 deg |
| Log ends | T+43.51 s at 86 m AGL (battery disconnect, not impact) |

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
3. Fitting a drag area to the published H135W thrust curve so the forward
   simulation reproduces both. The fitted CdA (4.54e-4 m^2) matches the value
   derived independently from the coast deceleration (4.6e-4 m^2).

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
6. **Nothing forces a log record on a pyro event** — `freshMask` is set but
   `nonblockingLogging()` is purely interval-driven (existing TODO). Fires are
   only caught by the 10 Hz telemetry header's fired bits.
