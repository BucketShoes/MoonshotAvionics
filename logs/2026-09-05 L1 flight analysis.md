# 2026-09-05 L1 flight — what the flash dumps say

Dumps: `dump-moonshot1-…full_8MB.bin` (rocket) and `dump-moonbase-…bin` (base station).
Decoded with `tools/moonlog.py`; browse them with `tools/logviewer.html` (open from disk, drop both files on it).

Acronyms: AGL/MSL = above ground level / mean sea level; EMA = exponential moving average (the "1 s fast baro EMA" flight logic uses); SNR = signal-to-noise ratio; SF = LoRa spreading factor; LR = the long-range SF12 beacon.

## Where the flight is

| | rocket | base station |
|---|---|---|
| boot session | records 445073–476444 (uptime 0.6–840.3 s) | records 89370–89928 (shares a row with an earlier June boot — the base only logs what it hears) |
| SET_RADIO ch3 SF9 **+22** | rec 472446, t=733.35, acked OK | rec 89781 |
| ARM (force, boost 3000 mg, boost alt 100 m, coast alt 200 m, main 100 m, 3000 ms pulses) | rec 472969, t=747.38 | rec 89788 |
| flight packets received | — | 89811–89827 |

Clock: UTC pages put rocket uptime 0 at **01:33:38 UTC** (consistent from t=100 s to t=770 s). GPS time is whole seconds, so ±1 s.

## Timeline (rocket uptime, seconds after liftoff in brackets)

| t | event | evidence |
|---|---|---|
| 796.85 (0) | liftoff, 01:46:54.9 UTC | accel jumps 1.1 g → 16.4 g between the 796.81 and 796.91 records |
| 796.9–797.6 | X axis pinned at +16376 mg | ADXL345 clips at about 16 g; real thrust acceleration was higher |
| 797.38 (+0.5) | **BOOST** detected | FLIGHT page tLaunch=69 ms at 797.445 |
| 797.7 (+0.85) | X axis flips to −15.4 g — burnout, drag deceleration | |
| 798.9–799.7 | X axis positive again (+2…+6.6 g) for ~0.8 s, then negative | unexplained: motor chuff, or the rocket briefly flying sideways/backwards. Gyro X is pinned at its limit (−2275 = ±2000 °/s clipped) through boost, so it was spinning hard |
| 798.97 (+2.1) | the "1023 m" packet goes out | fusion alt passes 1014 → 1032 between the 798.92 and 799.02 records |
| 800.4 (+3.6) | GPS fix lost (43 → 28 → 0 sats) | |
| 803.52 (+6.7) | **COAST** detected | needs total accel < 2 g for 500 ms; drag deceleration stayed over 2 g until here |
| 805.2 | thrust-curve packet received by base | 50 samples, 7145 ms window, −11736…+16376 mg |
| 814.03 (+17.2) | raw baro peak 2281.95 m MSL = **~2019 m AGL** | |
| 814.83 (+18.0) | **APOGEE → DROGUE, pyro 1 fired** | 0.8 s after the raw-baro peak, 0.3 s after the EMA peak, vertical speed −3.9 m/s. PYRO page shows ch1 energised 3000 ms (815.4–817.4 samples), no continuity, HV ~330 mV (nothing connected) |
| 830–838 | GPS blips back: 830.45, 833.5–837.5 | positions trend to −31.1626, 149.9316 (~640 m SSE of the pad) |
| ~835 → | descending at a steady 100–110 m/s, accel magnitude ≈ 1 g | terminal velocity, no drag device |
| 840.27 (+43.4) | **last record**, 01:47:38 UTC | raw baro 349 m MSL = **86 m AGL**; EMA 452 m MSL = **189 m AGL**; still DROGUE |

Not in the flash: 759.02–766.43 s (pad, armed). That 4 KB sector holds two record streams bit-ANDed together — it was written over without being erased. Same thing happened twice before in this dump (records 351371 and 363896–364059). Nothing flight-relevant was in it.

## Questions answered

**Did it deploy at apogee?** Yes. Drogue (ch1) fired 0.8 s after the true peak at ~2019 m AGL. If a charge had been connected and lit, the drogue would have come out within a second of apogee.

**Fallback for a high descent rate?** None exists. In code, MAIN_DEPLOY is only entered from DROGUE when the altitude condition below is met. Nothing looks at descent rate.

**Would it have fired mains near the ground?** Effectively no. The main-deploy check uses the **1 s baro EMA**, not raw altitude. At 105 m/s the EMA lags the real altitude by about 105 m. At the last record the EMA was still 189 m AGL (89 m above the 100 m threshold). Projecting forward, the EMA hits 100 m AGL at about 841.1 s, and the raw baro hits ground at about 841.1 s too. The main would have fired at the moment of impact. Under a working drogue (~20–30 m/s) the same lag would make it fire 20–30 m lower than set.

**Does the missing MAIN_DEPLOY mean it never tried?** Yes, for everything the log covers. Headers are written every 100 ms, the last full header/baro/accel set is at 840.27, and the next boot's first record sits exactly where the next record would have gone. So nothing half-written was left, and power was gone before 840.37. At 840.27 the main condition was 89 m from being true, which at 105 m/s takes about 0.85 s, longer than the ≤0.1 s left. It had not tried, and could not have tried before power was lost.

**Impact time.** The UTC page reads 01:47:36 at 840.02, but that value is stale (no GPS time update since about 837.5). Mapping uptime instead gives the last record at **01:47:38.3 UTC**. Baro says it was still 86 m up then. Comparing GPS altitude with baro in descent suggests the baro read high at speed (pad offset GPS−baro +66 m, descent offset about 0), so it was probably lower than 86 m. The end of the log is about the moment of impact.

**Why did the radio go quiet?** It didn't. The base station logged **every telemetry packet through the whole flight**: 16 after the 1023 m one, the last at rocket t≈839.0 (EMA 584 m MSL, DROGUE, P1 fired). The next was due at about 841.0, after impact. The screenshot froze at the 1023 m packet because the page's WebSocket had dropped ("WS: disconnected"). The base kept receiving and logging; the phone just stopped getting updates (screen lock or backgrounding during the launch is the likely cause).
- The rocket **was on +22 dBm**: SET_RADIO ch3/SF9/+22 was accepted (ack result 0) 64 s before launch.
- Cadence: one packet every 2 s. Every 20 s the LR beacon takes a slot, and the base wasn't in LR-listen mode, so that shows as a 4 s gap.
- Lost packets: 3 in the whole flight. One at liftoff (rocket t≈797.0), one near apogee (t≈811), one in descent (t≈825).
- SNR stayed 6–10 dB, except 1.25–1.5 dB around apogee, which looks like antenna orientation rather than range. SF9 decodes down to about −12.5 dB, so there was lots of margin.

**Thrust curve over LoRa.** Enabled, and it worked. On COAST entry the next packet carried page 0x0E with 50 samples (the SF9 setting): 67 bytes, about 410 ms airtime at SF9/125 kHz, against about 205 ms for the largest normal packet (22 bytes). Received cleanly at SNR 9.5. At SF8 it would be 100 samples (117 bytes); at SF7, 200 samples (217 bytes, ~345 ms). The base only logs packets that pass CRC; CRC failures are counted but not logged, so a corrupted long packet would leave no trace. That didn't happen here.

**Is 1023 a power-of-2 artefact?** No. The rocket's 10 Hz header log steps 969, 979, …, 1014, 1032, 1054, smoothly, up to 2277. The packet just happened to be built between those two records. The number isn't a real altitude either: during boost the raw baro read up to 1587 m MSL one second after liftoff (physically impossible), presumably pressure disturbance around the static port at high speed. The EMA smooths that but still inherits it. The base did log the 1023 packet (rec 89811). The rocket doesn't log what it transmits, but its own header records bracket it.

**Did it fail to detect burnout?** It detected COAST at +6.7 s, about 5.9 s after burnout. That's late because the test is total acceleration < 2 g and drag deceleration exceeded 2 g until then. It didn't matter here: apogee came 11 s later.

## Other findings

- **baro_ok / accel_ok / arm_ready are never set**: zero of 350k header records. `updateBaroStability()` / `updateAccelStability()` call `isStale()` before any reading has been recorded, and `recordError()` never sets `lastReadingUs`, so they stay stale forever. Arming only works with force, which is what was used (armFlags=1). The arming safety checks are effectively dead.
- **Baro is wrong at speed**: about +1300 m during boost, and possibly +60…80 m in the 100 m/s descent (by comparison with GPS). It helped launch detection here (the 100 m AGL trigger was reached by the error) but it feeds main-deploy altitude.
- **+22 dBm transmit disturbs the baro**: from 735 s on there is a +1 m bump every 2 s, in step with TX. 55 bumps in 60 s, against 9 random ones in the 133 s at −9 dBm.
- **Sensor limits hit**: accel clipped at 16 g for the whole burn. Gyro clipped (±2000 °/s) for most of boost and descent. Gyro X has a +36 °/s offset at rest. The magnetometer reads 0,0,0 throughout.
- **PEAKS page**: max altitude comes from GPS (1474 m was a bad first fix after reacquire), and max vertical speed comes from baro vvel (1754.8 m/s spike). Cosmetic.
- **After the flight**: boots 476445–579280 read all-zero accel and baro (sensor board disconnected in the impact?), working again from 579281.
- **Flash corruption**: the log writer's erase-ahead skips the erase when the next sector's first byte is 0xFF, and doesn't check the read result. The corrupted sector's old first byte was not 0xFF, so this doesn't fully explain it; root cause undetermined. For the NAND logger: per-record CRC, and verify after erase.

## Would it have saved the rocket?

If a drogue charge had been wired, the drogue would have deployed within 1 s of apogee at ~2019 m, and the main would then have fired 20–30 m below the 100 m setting. As a backup for a failed motor ejection, the apogee half works. The main half has no protection against a failed drogue, because of the EMA lag and because there is no descent-rate fallback. For the next flight:

1. Base main deploy on a lag-free altitude: raw baro, or EMA plus vertical speed × 1 s. Or make the threshold speed-aware.
2. Add a fast-descent fallback: if the rocket is still falling faster than about 40 m/s some seconds after drogue, fire main early, high enough to matter.
3. Add a timer fallback out of BOOST/COAST. Today, with no COAST there is no apogee and no pyro at all. That path depends on the accelerometer reading < 2 g, and an invalid accel blocks it.
4. Vent or shield the baro, and expect large errors at speed.
