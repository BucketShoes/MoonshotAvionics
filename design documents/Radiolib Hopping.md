# Radio Architecture — RadioLib Hopping Rewrite

## Overview

Rewrite radio handling for both rocket and base station using RadioLib, replacing the current low-level lora-net/sx1262 driver implementation. The primary goals are reliability, hopping support from day one, and an architecture that can accommodate relays, multiple modulations, and power management without later structural changes.

This document covers the target architecture. Implementation is phased (see bottom). It supersedes the previous `Radiolib Hopping.md` and `Hopping radio slot structure.md`.

---

## Core Timing Model

**Slot-based, drift-followed, no GPS discipline.**

- Slot duration: 200ms (configurable constant in shared `radio_config.h`)
- Slot index: monotonically incrementing counter on each device
- Rocket sets its own clock. Base station follows drift via EMA.
- No handshake for initial sync — base station acquires by listening (see Cold Start Acquisition).
- GPS time is intentionally **not** used for slot discipline. The system must work without GPS lock, indoors, after a hard landing and reboot, and with GPS powered down. GPS timing may be added later as an optional enhancement but must not be assumed by this architecture — doing so would bake in bad assumptions about GPS availability, prevent sleep modes, and create additional failure modes.

**Rocket timing rule:**  
At the start of each new slot, act if and only if the current time is within ±10ms of the intended slot boundary. If not within that window, skip and wait for the next slot. Never send late — a late transmission corrupts drift calculations on the base station and reaches nobody.

**Base station timing rule:**  
Start listening 20ms early (before the expected slot start, on the next hop channel, next modulation). Start sending a command 5ms after the slot start. When a packet is received, compute `rxdone_time − calculated_airtime = estimated_tx_start`. Feed this into a slow EMA to track the rocket's clock. Apply separately per packet type (different airtimes for different modulations).

**Shared constants (`radio_config.h`):**  
`SLOT_DURATION_US`, `RX_LISTEN_TIMEOUT_MS` (150ms), `DRIFT_LEAD_US` (20ms), `CMD_SEND_DELAY_US` (5ms), `SLOT_OVERRUN_MAX` (2), `TIMING_GATE_MS` (10ms), relay random re-send interval bounds, and all channel/modulation parameters.

---

## Frequency Hopping

- Default: 23 channels, selected from AU915 upper band (channels 64–86, ~902–914 MHz, 200kHz spacing) (EDIT: no, 915-928, as per existing code 0-63)
- Hop sequence: Fisher-Yates shuffle of channel indices, hardcoded seed, deterministic pure function of slot index
- All rocket TX hops. **Command channel is fixed** — see Command Path.
- Hop function: `channel = HOP_SEQUENCE[slot_index % NUM_CHANNELS]`
- `NUM_CHANNELS` (23) and the full hop sequence are defined in `radio_config.h`
- FCC extension: increase `NUM_CHANNELS` to 50+ with a longer Fisher-Yates seed; same function, longer period

The hop sequence length and slot sequence length must be **coprime** so all (slot_type, channel) combinations are eventually visited. (EDIT: all combinations equal, but the goal is actually just all channels get equal usage, given that slots have different airtime, etc, i.e. we cant always do LR on a specific channel like if they werent coprime)

(EDIT: the hopping sequence excludes the current command channel. do the fisher yates of the full 64, then if the command channel was selected in the 23, then swap it with the 23+1th - this way most of the channels are the same regardless of the current command channel, so the whole thing doesnt shuffle when command channel changes, just swap command channel with the 24th item in the shuffled list. when changing command channel, rederive the list and swap again, since the n+1th item in the list might have been swapped)

---

## Slot Sequence

A fixed repeating sequence of slot types, defined in `radio_config.h`. Length must be coprime with `NUM_CHANNELS`.
(EDIT: selectable sequences for different modes)

**Slot types:**
- `WIN_TELEM` — rocket TX telemetry, base RX. Hopped channel, telemetry modulation.
- `WIN_CMD` — base TX command OR relay TX, rocket RX. Fixed command channel, command modulation. (EDIT: a command base to rocket is fixed command modulation. but the free timeslot is base with nothing to send listens on backhaul modulation or random tx on backhaul modulation. if multiple cmd slots, then rocket does single rx with longer timeout)
- `WIN_LR` — rocket TX long-range location. Hopped channel, LR modulation. Always overruns into the following WIN_CONTINUE.
- `WIN_CONTINUE` — explicit no-op. Allocated immediately after any slot that always overruns (e.g. WIN_LR). The radio is left in whatever state the previous slot put it in; no slot action is taken and the overrun counter is not incremented.
- `WIN_OFF` — radio in standby. Used in low-power cycle variants. (EDIT: This is often equivalent to continue, because the radio is off after the previous anyway. might combine these)

`WIN_CONTINUE` is the mechanism for handling always-overrunning slots. Variable-length slot arithmetic (`if slot 3 is 2x long, what slot are we in at time T?`) is avoided entirely — slot index is always `floor(time / SLOT_DURATION) % SLOT_SEQUENCE_LEN`. (EDIT: note that things can overrun, this is just to avoid putting a slot that would be otherwise unused and confusing, e.g. lr,cmd would make the base think theres a usable command window, but the rocket would always be busy overrunning from the lr)

**Current sequence (subject to tuning):**
```
TELEM, TELEM, TELEM, LR, CONTINUE, TELEM, TELEM, CMD, TELEM, TELEM, TELEM
```
Length 12 (coprime with 23 channels → full coverage every 276 slots ≈ 55 seconds). (EDIT: no, changed to 11)

**Slot index derivation:**  
`slot_type = SLOT_SEQUENCE[slot_index % SLOT_SEQUENCE_LEN]`  
`hop_channel = HOP_SEQUENCE[slot_index % NUM_CHANNELS]`  
Both are pure functions of `slot_index`. Nothing else is needed.

**Overrun tracking:**  
Record what operation was started in the current slot. When an operation finishes (RxDone IRQ, TxDone IRQ, or timeout), compute airtime from that record. Do not assume the radio is idle at a slot boundary — check IRQ flags.

---

## Slot Overrun Handling

Some operations legitimately take longer than one slot (e.g. a 400ms command receive in a 200ms slot, or WIN_LR always overruns into WIN_CONTINUE).

Rules:
1. At each slot boundary, check if radio is still active: TxDone not fired, or `PreambleDetected`/`HeaderValid` set without `RxDone`.
2. If active, skip the slot action. Increment an overrun counter.
3. After `SLOT_OVERRUN_MAX` (default 2) consecutive unexpected overruns, call `setStandby()` and resume normal slot actions. This is the safety cutoff for a stuck radio.
4. `WIN_CONTINUE` is an explicit no-op — the overrun counter is **not** incremented for it. It is expected to be mid-packet.
5. Commands usually produce dead air (no command pending → RX with 150ms timeout). If `PreambleDetected` fires before timeout expiry, the radio keeps listening past 150ms. Let it. If it overruns into the next slot, apply rule 2 for that next slot.

(EDIT: that might not be the right logic. i think it should be more about rxdone vs timeout, or the busy status, but preamble/header interupts are also informative. the timeout is generated by the sx1262 itself - and it intenally cancels the timeout when a packet is detected, at sync word, which isnt something we can hook into, but is a better hook for this)

**IRQ usage:**  
SX1262 exposes `PreambleDetected` and `HeaderValid` as first-class IRQs on DIO1. Use `setDio1Action()` with a combined IRQ mask including these plus `RxDone`, `TxDone`, `Timeout`, and `CrcErr`. Poll `getIrqFlags()` at slot boundaries — non-blocking, ~10µs.
(EDIT: no, the getirqstatus must be done immediately in every loop, you cant wait for the next slot because multiple different interupts will happen by then. in the interupt handler, save the dio1 time (for unknown irq reason), then in next loop, if dio1, getirqstatus and put the time into the appropriate timer for preamble/header/rxdone/etc, then use those for drift calc afer we have the full packet so we can use the exact timers for drift calc, without jitter from loop - cant getirqstatus in the isr because its spi, so just save the dio1 time in each interupt, then put that time into the right timer in next loop when getirq, then apply those timers after rxdone for drift calc)



```cpp
// Setup
radio.setDio1Action(onDio1);
radio.setDioIrqParams(RADIOLIB_SX126X_IRQ_ALL, RADIOLIB_SX126X_IRQ_ALL);

// At each slot boundary (non-blocking)(EDIT: nope - dont wait for slot)
uint16_t flags = radio.getIrqFlags();
bool preamble  = flags & RADIOLIB_SX126X_IRQ_PREAMBLE_DETECTED;
bool header    = flags & RADIOLIB_SX126X_IRQ_HEADER_VALID;
bool rxDone    = flags & RADIOLIB_SX126X_IRQ_RX_DONE;
bool txDone    = flags & RADIOLIB_SX126X_IRQ_TX_DONE;
bool timeout_  = flags & RADIOLIB_SX126X_IRQ_TIMEOUT;
```
(EDIT: nope - that code is wrong)
The DIO1 ISR only sets a volatile flag. All logic runs in `loop()`. (EDIT: ISR also sets the timer from micros() - important for accurate time, usable when we know which interupt it was for)

---

## Modulations

| Slot type | Modulation | Channel |
|---|---|---|
| WIN_TELEM | Configurable SF (default SF9), BW 125kHz | Hopped |
| WIN_CMD | LoRa SF9, BW 125kHz | Fixed |
| WIN_LR | LoRa SF12, BW 125kHz | Hopped |
| WIN_CONTINUE | — | — |
| WIN_OFF | standby | — |

GFSK is a future option for WIN_TELEM. Narrowband GFSK may outperform LoRa at range; wideband GFSK may allow multi-page telemetry per slot. Neither is defined yet — bench testing needed. The architecture must not assume telemetry will always be LoRa.
(EDIT: note gfsk needs a bigger restart of the radios than just changing between lora settings. gfsk might be better at high error rates, vs lora getting no packets. lr/cmd might also be configurable at runtime)

Exact modulation parameters are **configurable via `SET_RADIO_PARAMS` command** and should not be hardcoded in logic. Use named constant structs from `radio_config.h`. Each slot type has its own modulation config; they may differ and will change.

(EDIT: because each slot type has its own modulation settings, but they can change at runtime, this means the "current" settings might not match the slot config - so save full settings when starting a radio operation, not just save which slot type enum, e.g. sf9 rx, then change to sf5, then calc airtime would be wrong because it was actually recieved with sf9 but the settings say sf5 when the calc was done after the packet finished, or change hopping seed would think we're on the wrong channel, etc)

---

## Telemetry Header — Slot Sequence Byte

Add 1 byte to the 0xAF telemetry header: `uint8_t slot_seq_index` = `slot_index % SLOT_SEQUENCE_LEN` at time of transmission.
(EDIT: might be able to fit this ino the existing reserved bits)

This allows the base station to:
- Determine which slot in the sequence the rocket believes it's in
- Derive which hop channel it should be on (cross-check against received channel)
- Anchor slot tracking after cold-start acquisition

The received channel is known from context (the frequency we were listening on); it does not need to be encoded in the packet.

---

## Long-Range Packets

WIN_LR packets are extremely payload-limited. At SF12/BW125, airtime for a few bytes exceeds 400ms — always overruns the 200ms slot, hence always followed by WIN_CONTINUE.
(EDIT: others might overrun, e.g. command will overruyn if there is an actual command, but lr always overruns, so the following slot is meaningless. continue avoids confusion)

- Payload: ~3 bytes — packed GPS coordinates + flags. Exact format TBD. (EDIT: format in packet formats doc)
- Only 1 LR slot in the sequence. The receiver knows it's an LR packet by its modulation; the channel is confirmed by hop function + slot index. (EDIT: i.e. it doesnt need to say its slot index, because theres only one lr, so the index can be inferred)
- Acquisition via LR packets is architecturally possible but not implemented in phase 2. Telemetry-based acquisition is the primary path. (EDIT: also because the percentage of telem is so much higher, easier to catch)

---

## Command Path

**Fixed channel, not hopped.** The command channel is the same for all base stations and relays. This is the only time the rocket is guaranteed not to be transmitting.

**Rocket during WIN_CMD:**  
Switch to fixed command channel and command modulation. Start RX with 150ms timeout. If `PreambleDetected` fires: keep listening, do not enforce slot boundary until packet completes or safety cutout triggers. If `RxDone` fires: process command, resume slot tracking. If timeout fires with no preamble: resume next slot normally. (EDIT: the :keep listening" logic is internal to the radio.. it just wont fire a timeout, so listen for the rxdone/etc as normal - the point here is dont interupt it if busy when the next slot starts)

**Base station during WIN_CMD:**  
- If command queued: wait 5ms from slot start, TX. No CSMA — timing must be exact. Accept that collisions with relay traffic are possible; rely on retry. (EDIT: there is existing logic in command sending for optional wai for slot, and optional multi-send. currently, multisend is jammed together with no slot timing delays. later might have it wait again and resend on next cmd window, using the same nonce, so it would be ignored if rocket did get the first one. but for now the wait for slot then send x times is fine. )
- If no command queued and not in relay-send window: start RX with 150ms timeout (same overrun logic). Log any received relay packet; do not re-relay.
- If in relay-send window: TX relay packet (see Relay section).

---

## Relay Architecture (Phase 3)

All base stations try to hear the rocket. Relay allows a base station that heard the rocket to share that data with others that didn't. (EDIT: every base station is a relay, we just refer to a relay in terms of its from someone else, and treat it as second-hand information)

**Relay mechanism:**  
Each base station tracks "last heard telemetry." After a random delay of 120–300 seconds (re-randomised after each send), transmit that record in the next WIN_CMD slot. (EDIT: transmit regularly; not just after you hear one - its a continuous repeat of the last known good for those that missed it)

Receiving bases: log relay packet for client serving. Do not re-relay. Do not feed into drift EMA.

**Relay packet format (not 0xAF):**  
Distinct packet type prefix. Contains:
- Original telemetry payload
- RSSI/SNR as heard by the relaying base station
- Timestamp of original reception
- Relay base station ID
- Slot index and channel it was originally heard on
(EDIT: the timestamp is in the relay's time, which wont necesarily match anything else. also include relays current timestamp in packet... for battery, relays might not have gps running so might not know the real time, maybe  include rekoning of current utc time? should we also include enough to let a base that heard a relay sync to the same rockets timings? dont need to send slot index, thats in the original payload. might also want to make this relay format able to hold lr and other packets; maybe top several - payload limits less of an issue for relays because they are in a known location so we dont need to push for max range modulations)

(EDIT: we might end up making the base station/relay connection be via ble coded phy instead of lora - simplifies some of the handling because we only have one sx1262 which is busy, and coded phy gets 300m+ confirmed working on esp to phone. esp to esp are both class 1 so will have an extra 16db link budget so likely more than 300m)


**Collision handling:**  
No CSMA — we can't wait without missing the timing. Send blind. Random intervals reduce collision probability. This is eventual-delivery, not time-critical. (EDIT: random interval... but wait for the next win_cmd slot - thats the only time the other base stations will be listening, otherwise they are busy listening to the rocket)

**Rocket ignores relay packets:**  
Relay packets have a distinct magic byte / packet type. Rocket's command handler only acts on packets addressed to it.
(EDIT: also, usually on the wrong modulation, so it wouldnt hear it anyway.. this isnt for the rocket, its just spare time when the rocket isnt allowed to talk, so therefore the base station radios arent otherwise busy listening to the rocket, so is a good time to send backhaul)
---

## Cold Start Acquisition

Base station starts with no timing information.

1. Pick a random channel from the hop sequence (any of the 23).
2. Set telemetry modulation.
3. Start RX with very long timeout (SX1262 supports ~262 seconds via max timer value; alternatively, loop 150ms listens without changing channel). (EDIT: no: its CONTINUOUS rx - not long timeout, and certainly not 150ms, that would leave so many holes to miss packets. the scan is explicitly NOT following the slot timings at all. slots are ignored while scanning)
4. Wait for a telemetry packet.
5. On receipt: read `slot_seq_index` from header. Derive expected hop channel for that slot index and confirm it matches the received channel. If consistent: lock to this timing. Feed `rxdone_time − airtime` into drift EMA as first sample. (EDIT: nope, you cant derive hop channel and you cant cross-check it - look at the channel you heard it on - thats the hop channel. find that in the hopping list thats your hop index. the slot index in the header tells you the slot index - which is completely separate sequence for which direction/which modulation will be on the next channel - e.g. where are the cmd slots relative to this telem slot - but either way you know the channel regardless of the slot index in the header. also, the first one, just sets it direectly. only all future ones follow drift. bear in mind this same mechanism can also be used when we're not sure if we've lost it or not, so the scan logic needs to return a new setup, not jkust override the settings directly - so it can be used on startup, set settings, and on command scan for a while, then return to old settings - which needs to also be able to include if we heard one packet, but nothing else matched. we dont have 100% certainty that a telem packet is real like we do with hmac'd commands, it could have the right magic nuymbers by chance, and we dont want to throw away sync just because we heard junk - thats why the drift ema is so slow)
6. Begin normal slotted operation with lead/follow timing.

**Expected acquisition time:**  
(SLOT_SEQUENCE_LEN / num_telem_slots) × NUM_CHANNELS × SLOT_DURATION  
≈ (12 / 8) × 23 × 200ms ≈ 7 seconds typical.

Because the hop sequence and slot sequence lengths are coprime, listening on any single channel will eventually catch a telemetry slot. No dedicated beacon or lighthouse channel needed.

---

## Shared Configuration File

`radio_config.h` at the repository root, included by both firmwares as `../radio_config.h`.

Contains:
- All timing constants
- Hop sequence array and `NUM_CHANNELS`
- Slot sequence array and `SLOT_SEQUENCE_LEN`
- Channel frequency table (AU915 upper band) (EDIT: no table, just a formula, pre-existing code)
- Modulation parameter structs for TELEM, CMD, LR (EDIT: not sure these make sense as defines - these are configurable at runtime. makes sense for some ofthe params, or maybe the defaults as defines/const)
- Magic bytes / packet type identifiers (EDIT: magic numbers arent configurable, they are not subject to change, put those somewhere else like constants or something - it does also make sense that be shared)
- Relay timing bounds
- Any other constant shared between firmwares

Per-device configuration (current TX power, relay enable, etc.) stays in the respective `config.h`. (EDIT: these are runtime editable examples. thefixed  defines for radio still make sense shared in one file even if only one side uses them, e.g. which kinds of debug logging is enabled, has bs and rocket specific flags))



---

## Blocking Budget

Complies with the non-blocking rules in CLAUDE.md. Phase 1 relaxes the main loop limit to 10ms worst case; radio operations should consume ≤1ms of that.

- SPI transactions (RadioLib calls): ≤1ms typical, ≤3ms worst case. Acceptable. (EDIT: Claude has a habit of declaring unacceptable things as fine or acceptable specifically when they arent. any statement of this form where it ends with saying it is acceptable or ok or fine should be dealt with extreme skepticism. 3ms is not ok, it needs to be reviewed and should definitely be avoided. its just no longer automatically kicked out and immediate cause to remove the entire feature. anything which could contribute along with other things to add to more than 10ms is absolutely not ok)
- IRQ flag reads (`getIrqFlags()`): ~10µs. Always acceptable.
- `setStandby()`: ≤500µs. Acceptable. (EDIT: if reported in budget)
- Starting TX/RX (`startTransmit()`, `startReceive()`): ≤1ms. Acceptable. (EDIT: this is borderline and not acceptable)
- **Never call `*_BLOCKING` or `waitBusy` from `loop()`**, even from a non-armed path. These remain init-only in `setup()` paths.
- Safety cutout `setStandby()` at 2-slot overrun: bounded, acceptable. (EDIT: this is not a flight safety or injury risk, threshold like the other safety mechanisms above. this just risks the rocket never being found. its an entirely different meaning of the word safety. BLOCKING THE MAIN LOOP MORE THAN 10 MS COULD CAUSE HUMAN INJURY. failing to reset a radio is just lost property or minor inconvenience)

(EDIT: anything with a worst-case over 1ms blocking main loop still needs to be accounted in the budget; it just no longer needs many complex state machines, and blocking is allowed if worst case total is under 10ms for all possible opeations combined, e.g. if multiple spi transxactions could happen in a single loop, their worst cases need to add to <10ms, and if total of worst cases is >1ms needs to be in budget. this is to be used for planning another unrelated controller architecure which will handle the TVC/active aero control surfaces, etc, since this controller with ble/lora has unreconcilable delays. THERE IS STILL A HARD LIMIT FOR 10MS TOTAL PER LOOP - this controller still controls pyros for parachutes, which are safety critical and must fire on time every time, based on continuous sensor readings and must turn off reliably. However, avoiding the waitbusy and *_BLOCKING methods adds substantial complexity - they are allowed if the max timeout and total is constrained and accounted - including accounting for everything that calls them, and everything that calls any of those, etc - the total of multiple if all possible in a single loop must still be under 10ms per loop )

---

## Implementation Phases

### Phase 1 — RadioLib port, shared config, command queue
Port rocket and base station radio to RadioLib. Introduce `radio_config.h`. Replace boolean command flag with queue. Retain existing single fixed modulation, no hopping. Validate bench timing and IRQ handling via `PreambleDetected`/`HeaderValid`. Reduce code duplication between rocket and base station. (EDIT: true fifo queue is much lower priority.)


### Phase 2 — Hopping, passive sync, multiple modulations
Add hop function. All rocket TX hops. Add `slot_seq_index` byte to 0xAF header. Add WIN_LR + WIN_CONTINUE to slot sequence. Add cold-start acquisition on base station. Add drift EMA on base station. Establish fixed command channel. Define LR modulation parameters. (EDIT: win_lr is already implemented. existing implemented drift ema is built, but tracks incorrectly, buggy, needs rewrite, 0xaf might not need a whole byte, if under 16 telem slots in the cycle, the existing reserved bits are enough - we only need to track which telem slot it is, because if this packet was sent, it was sent in a telem slot, which gives us more total slots in 4 bits)

### Phase 3 — Relays, HMAC, power saving
Relay packet format and base station relay TX/RX. Replace per-packet CRC with HMAC-32 (commands already have full HMAC + nonce). Sleep/low-power cycle variants (alternate slot sequences with WIN_OFF). "Quiet" command for post-landing battery saving. (EDIT: power saving for post landing or prelaunch will probably be done by changing slot sequence; or maybe by suppressing a fraction of telems - rx uses some power, but tx uses much more. also tx with lower power can save power. possible option for pings to keep rocket in low tx power and skipping most windows.)

### Phase 4 — Multiple rockets, FCC band, GPS timing option
Slot-phase offset per rocket. Extend to 50 channels for FCC (same hop function, longer period). GPS-disciplined timing as optional enhancement (does not replace drift following; adds a bit flag "I have GPS time" to header; receivers use it as a hint only). (EDIT: no, 53 channels, gotta be prime. fcc requires at least 50 so 53 prime; but au only requires 20, so 23 prime - GPS time discipline can be used for better sleep, and better tracking when lost due to range, still in sync when we get closer again - although bases dont normally run gps, so teven if the rocket is perfect, the bases drift, too - drift calc is the mutual offset between both rocket and base station drift. after landing, rockets might turn off gps for power saving, although reacquire intermittently to hold perfect long term)

---

## Open Questions

- **GFSK vs LoRa for telemetry.** Narrowband GFSK may outperform wideband LoRa at range. Needs bench testing with SX1262 at various BW settings. Architecture supports swapping without structural change. (EDIT: not bench testing, field testing. My bench isn't multiple km long. also, a real launch is the only way to test airborne vs near-obstruction/fresnel)
- **LR packet payload.** 3-byte packed GPS format not yet defined. (EDIT: yes, format defined and pre-existing and implemented, working)
- **Multi-page GFSK telem.** If wideband GFSK fits multiple pages per slot, the one-page-per-slot assumption needs revisiting. Defer to Phase 2 evaluation.
- **WIN_CONTINUE and overrun counter.** WIN_CONTINUE slots do not increment the overrun counter; only unexpected overruns do. Confirm the safety cutout does not count WIN_CONTINUE as an overrun.
- **`Hopping radio slot structure.md`** — superseded by this document. Can be deleted or archived.






priority fixes:

minimum slot type support: telem, cmd, off, lr, continue
lr packets have implicit headers, no crc. command have explicit headers, different cr,sf,bw,etc. telem is currently planned to be gfsk but may be lora in future based on testing. future intention to replace crc with hmac in payload; and to change fec settings, e.g. 4/6 or 4/8 - differently for each slot type.
remember, calculation of radio settings cant be done based on which slot you're currently in, you might have started recieving from a previous slot - you must store what the radio is actually using. radio settings can be changed at runtime by config, so you must store the actual settings, not just the slot type that started the tx/rx operation (currently the command for change radio settings is outdated; to be fixed )
slots are a duration, but the event happens at the start of each slot. given that many packets can exceed one slot, it may be more useful to conceptualise it as merely the events, not the entireity of the duration of each slot
for now, the loop jitter before starting and the time between beginning the command sequence to do it vs the actual tx/rx start time is ignored and assumed to be zero or consistent.
fix docs - it assumes telem is lora, incorrect. its gfsk. this means it needs to reset the modems between slots. we can go with lora for now, but remember when we switch to gfsk, you might need to fully reset the modem between every slot (except continue)
remember, base station needs to start listening early, which may overlap the previous slot - depending on modulation settings and payload size, the previous might be done, or might be still running - but if the previous is still running, its ok to startlistening late. same with rocket rx, its ok to start listening late; but dont tx late.
multiple slot sequences - 
e.g.standard: [telem,telem,telem,lr,continue,telem,telem,cmd]
e.g.power save: [telem,off,off,LR,continue,continue,continue,cmd,continue,off,off,off,off,]
e.g.bench testing [telem,cmd]
e.g.long listen[telem,lr,continue,continue,cmd,cmd,cmd,cmd]

try to make cmd always 4 slots after lr so unknown sequence still knows when the rocket will listen. (or if we ever allow variable/configurable at runtime for the slot timing for the same thing but slower, then make command listen windows always a specific time after lr)

TODO: is off and continue the same? the previous should end and turn off back to standby - or do we want to force it to off, not just continue if something went wrong and it didnt stop? should the rx time also increase? or is scanning a pure non-slotted setup?
TODO: should base station be allowed to delay tx of command until after rx? a long telem could consistently cut off command window starting timesa - but the rocket will start listening eventually, and should be able to catch it. maybe a min turnaround time for rx to tx on base?
TODO: can we get under 16 slots, fit in existing flags? can we squeeze some more out of what we have? should we change to a new format from 0zaf if we break compatibility to make it easier to spot outdated code? should we add the multi-record format at the same time, since we're changing the header anyway (add length byte to data page, rather than inferring it from the total length; so we can allow multiple pages - or should that just be a page type, saving 2 bytes on single page packets?)
future maybe gps timing discipline. not for now
low priotity for short rx windows when confirmed good recently - minor battery saving but big reliability impact. also less important now a higher percentage are planned as telem, theres less time where the rocket even could be long vs short listen windows. no need to listen during non-cmd windows on rocket. base station is either listen to the slots, or listen 100% for scanning/sync
sync command on base station now just puts it back into scanning mode for 60 sec. if nothing found, reverts to its last best lock (if it had one). "found" means multiple parsable 0xAF packets in a row, reaching <50ms deviation in timing. remember old anchor while locking in on new anchor/drift. possible false positivesshouldnt corrupt known timings. in future possibly add params to sync command for how long to scan, and whether to scan telem or lr modulation (or maybe on findme, etc, but those are intended for a differnet kind of scan)
sync command will also in future have the option to sync from lr packets - much less common, so much longer wait
during scan, quickly take new timing (remember old timing), then drift calc should track how close each one is, to see if found consistently within 50ms of expected time
TODO: should we add  a WhereAreYou command? - rocket queues an immediate packet on the specified channel out of normal timing, then returns to normal hopping - base station can use this known response to hear it and know what sync pattern to follow. message includes hopping data page, with timestamp of when in the slot timing this was sent, and what the current slot number is. base calculates when this was sent, and should be able to get ahead of the rocket, continuous listen there, and fall into sync. the cmdack page stays queued for nomal hopping, the hop reply page is in the special out-of-time out-of-sequence. this interupts any in-progress radio action (i.e. setstandby, set to specified channel, tx - these can be in the main loop, by flags that override the normal logic; command handler just sets those flags) - main issue is the rocket spends such a small percentage of time listening. maybe 
for passive sync from lr packets -= these dont have hopping info - they are a different modulation to normal for airtime calc, and they cant be properly verified (any random3 bytes on sf12 is accepted, which might not be from us) but assuming the lr packet is from us, they do imply timing and channel. base station can then get ahead of the sequence and listen ahead, following telem, etc, and guessing where command slot is. we might be out of range to get consistent telem, but it gives a way to probably be on the right channel at the right time to try to get in normal sync, and also to align when cmd windows might beto send a whereareyou command

commentary/notes:-
propagation delay is not necesarily sub microsecond. it will be sub millisecond. this is a supersonic model rocket, to 30k alt. different modulations are more tollerant to doppler


15.3 Implicit Header Mode Timeout Behavior
15.3.1 Description
When receiving LoRa packets in Rx mode with Timeout active, and no header (Implicit Mode), the timer responsible for generating the
Timeout (based on the RTC timer) is not stopped on RxDone event. Therefore, it may trigger an unexpected timeout in any subsequent mode
where the RTC isn’t re-invoked, and therefore reset and re-programmed.
15.3.2 Workaround
It is advised to add the following commands after ANY Rx with Timeout active sequence, which stop the RTC and clear the timeout event, if
any. The register at address 0x0902 is used to stop the counter, while the register at address 0x0944 clears the potential event.
The following pseudo-code can be used as a reference to implement the fix:
WriteRegister(0x00)@0x0920;
value=ReadRegister@0x0944;
value = value | 0x02;
WriteRegister(value)@0x0944
^^ datasheet says that, but does it mean 0902 or 0920? maybe we just setstandby, and all other things will set their own timeout anyway?


Main: 
I want to address a number of bugs, false assumptions and architectural issues that have crept into the radio design. It was an architecture I originally wanted to have more features, but many got pulled out for debugging, or a simplified version for testing, or temporarily for initial setup, and now it can't really support what i wanted to do with it anymore, e.g. the active sync is far too much a fundamental structural concept, and once it sends the message it assumes it's in sync immediately, that it was heard sucessfully (even though the other side only listens about half the time), and that its locked in forever, all incorrect.
I think the main source of bugs is our use of the raw low-level lora-net/sx-1262 driver, rather than the abstractions provided by radiolib.
Also, there were very tight timing constraints for the rocket loop, which made it basically impossible to do multiple spi transactions without an overly complicated state machine that caused its own issues. I've decided to allow a longer time limit, since this version of the tracker is moving away from active flight control/tvc, and will just do pyro - it still needs 10ms limit because it still controls pyro charges, and i'd still like to account for anything that could ever be over 100usec in the budget, but its no longer a hard limit.
I want frequency hopping, with passive sync, so multiple base stations can all sync to the same rocket.
I want fast regular telemetry, commands, and super long-range packets, each in a different modulation, gfsk/lora, and speed/bitrate/etc settings.
I want to be able to establish and maintain tight timing sync, which is needed for frequency hopping, but also allows for optional low power (avoid rx other than exact timing window).
The low power can be later. for now, listen whenever not busy doing something else. the main limitation is the fact there's only one radio, so it can only do one thing at a time, and some packets are longer than how often i want telemetry.
I'd like to have most of it configurable in a single place, especially any settings which must be common to rx and tx should have one files used by both, not changed in two places.
For sync, i want the rocket to transmit telemetry packets on a regular basis, with occasional times which are allocated for the rocket to listen and the base station to send commands. I also want long-range packets, on a different modulation, with very limited payload (win_lr).
I'm interested in trying GFSK, especially for the main telemetry, it seems to have better performance at the edge of range. I also want to be able to change the settings to tweak and find what works best.

Concretely, I'm thinking to have 4 telemetry slots, then a command slot, then 4 more telem, then a long-range slot with extra length.
i.e. telemetry should fit within the 250ms window; but commands will be at a slower modulation, and wont fit - but 99% of the time, there is no command. the base station can queue up a command and send it at the right time, and the rocket can detect that a packet has started, and keep listening, even though the time slot should have moved to the next telem packet, keep listening.
The current approach tries to do the listening using the sx1262 built in timer cancellation - if it heads the preamble/headervalid, it will cancel the timeout, and keep running - but we need to respect that and allow it to keep going - but we need to have limits - if too many slots overrun, then we should force it back to standby and go back to normal (i think this is currently a flag, but we want 4 slots allowed) - but this approach has been a little buggy - i think some modes, (i think its from implicit headers) dont cancel the timeout, so it ends up permanently busy, etc
Commands will typically overun the slot. telem will usually fit within the slot.
Long-range packets will typically overrun one or more slots. (the slot type after lr is meaningless because it always overruns)
if we've not heard telem for 2 minutes, it can go into continuous listen mode on a random channel for 1 second, then return to normal, unless it finds something with valid 0xaf packets


I'd like to explicitly have in scope, for phase 2, the relay functionality. base stations randomly resend their last known good telemetry, for other bases to hear.
Base stations with no command to send should listen on backhaul channel/modulation during the command window. Base stations should pick a random time 60-120 sec to resend their last good telem on backhaul for other base stations. This packet should differ from the existing 0zaf format, so it doesnt confuse timelines, since it could be old data, and because it also has extra metadata, etc.


Actions:
when starting tradio, record what settings actually in use - we can't calculate it from the slot - and this needs to be more than just an enum, since radio settings can be configured, leaving it thinking its on the new settings when its acually on the old
on the rocket, simple timing - on the first loop of a new slot, if there's no incoming packet, switch to the new mode and do it. if there is, allow up to the limit.
on the base station, slightly before a telemetry slot, a base station should listen on the telem modulation, on the upcoming hop channel.



Priorities:
1 reliable passive sync (continuous listen on base station startup)
2 hopping (23 channels)
3 remove assumptions on slot timings being always longer than packet times., change to 210ms, with all slots able to continue if in progress, and with dedicated win_continue after the lr slots (packets longer than a slot)
4 relay backhaul
fix implicit header not clearing timeout -  bug in implicit headers not canelling the rxdone timer automatically - call 
remove synced boolean(use timestamp of  latest recieved good ping/good telem for confidence; although for now, just always in unconfident mode. now with passive sync, knowing we're in sync just allows saving battery)
maybe switch to radiolib







Minimal:
passive sync, faster slots, higher precentage telem, add slot index in reserved bits of 0xaf header
combine radio configs into one file for rocket and base to both use