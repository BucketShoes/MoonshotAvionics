// common/radio_config.h — Shared radio protocol constants for rocket and base station.
// Include from both rocket_avionics and base_station. Anything that must match exactly
// between the two firmwares belongs here. No implementations — see common/radio_helpers.h
// for inline functions, and common/protocol.h for command/ack IDs.

#ifndef COMMON_RADIO_CONFIG_H
#define COMMON_RADIO_CONFIG_H

#include <stdint.h>
#include "sx126x.h"  // sx126x_lora_cr_t for LORA_LR_CR

// ===================== SLOT TIMING =====================

#define SLOT_DURATION_US  250'000UL

// ===================== SLOT TYPES =====================
// Values are fixed wire constants — do not renumber.

enum WindowMode : uint8_t {
  WIN_TELEM         = 0,   // rocket TX telemetry / base RX — hopped channel, telem modulation
  WIN_CMD           = 1,   // base TX commands / rocket RX — fixed command channel, cmd modulation
  WIN_OFF           = 2,   // radio standby — low-power cycle
  WIN_LR            = 3,   // rocket TX long-range — hopped channel, LR modulation (implicit header, always overruns)
  WIN_FINDME        = 4,   // long-preamble beacon for passive scan (not yet implemented)
  WIN_BACKHAUL      = 5,   // base relay backhaul (not yet implemented)
  WIN_MULTIPURPOSE  = 6,
  WIN_RDF           = 7,
  WIN_GFSK          = 8,
  WIN_CONTINUE      = 9,   // explicit no-op: extends previous slot, no radio action taken
  WIN_NARROWBAND_TELEM = 10,
  WIN_WIDEBAND_TELEM   = 11,
};

// ===================== SLOT SEQUENCE =====================
// Length must be ≤16 (4-bit wire field in 0xAF stateFlags holds slot_index % SLOT_SEQUENCE_LEN).
// Length must be coprime with NUM_HOP_CHANNELS (23) for full channel coverage.

static const WindowMode SLOT_SEQUENCE[] = {
  WIN_LR, WIN_CONTINUE, WIN_TELEM, WIN_TELEM,
  WIN_CMD, WIN_TELEM, WIN_TELEM, WIN_TELEM,
  WIN_TELEM, WIN_TELEM, WIN_TELEM, WIN_TELEM
};
#define SLOT_SEQUENCE_LEN  (sizeof(SLOT_SEQUENCE) / sizeof(SLOT_SEQUENCE[0]))

// ===================== FREQUENCY HOPPING =====================

#define NUM_HOP_CHANNELS  23

static const uint8_t HOP_SEED[] = {
  0x69, 0x42, 0x06, 0x96, 0x6c, 0x04, 0xed, 0xbc,
  0x0e, 0x98, 0xb2, 0x50, 0x18, 0x33, 0xc9, 0xd3,
  0xdc, 0x2d, 0xda
};
#define HOP_SEED_LEN  (sizeof(HOP_SEED) / sizeof(HOP_SEED[0]))

// hopSeq[] is filled by rederiveHopSequence() (in common/radio_helpers.h).
// Definition lives in each firmware's radio.cpp.
extern uint8_t hopSeq[NUM_HOP_CHANNELS];

// ===================== CHANNEL TABLE (AU915-aligned) =====================
// Channel 0-63:  BW125, 915.2 + ch*0.2 MHz
// Channel 64-71: BW500, 915.9 + (ch-64)*1.6 MHz
#define CHANNEL_COUNT 72

// ===================== LORA MODULATION (must match) =====================

#define LORA_CR         5       // 4/5 coding rate (fixed, not command-configurable)
#define LORA_PREAMBLE   8       // preamble symbols
#define LORA_SYNCWORD   0x12    // private network

// WIN_LR slot config. BW follows activeChannel. SF is separate so it can be reduced
// for bench testing.
#define LORA_LR_SF    11
// 0x05 = 4/5 long-interleave (LI). Use SX126X_LORA_CR_4_5 for standard 4/5.
// TODO: @@@ confirm whether LI is causing the stuck-BUSY seen in testing.
#define LORA_LR_CR    SX126X_LORA_CR_4_5

#define LORA_RX_BOOSTED true

// ===================== RADIO DEFAULTS (NVS fallback) =====================

#define DEFAULT_CHANNEL     3
#define DEFAULT_SF          9
#define DEFAULT_POWER       -9    // dBm

#define DEFAULT_BH_CHANNEL  67
#define DEFAULT_BH_SF       5
#define DEFAULT_BH_POWER    -9

// ===================== SLOT TIMING / OVERRUN =====================

// Base station starts RX this many µs before the slot boundary (and CMD TX this many
// µs after). Both sides apply the same RX-min-remaining and overrun cap.
#define BS_RX_EARLY_US           20'000UL
#define BS_CMD_TX_OFFSET_US       5'000UL
#define BS_RX_MIN_REMAINING_US   60'000UL
#define BS_RX_TIMEOUT_US         (SLOT_DURATION_US - 20'000UL)
#define BS_OVERRUN_MAX           3

// Rocket telem RX timeout choices (synced vs lost-rocket fallback).
#define ROCKET_RX_TIMEOUT_US      100'000UL
#define ROCKET_LONG_RX_TIMEOUT_US 750'000UL

// Force standby if RX has been active longer than this many slots (stuck DIO1).
#define RX_STUCK_MAX_SLOTS        10UL

// "Lost rocket" widening threshold — open long RX windows so a fresh CMD_SET_SYNC can land.
#define ROCKET_NO_BASE_HEARD_THRESHOLD_US  130'000'000UL

// ===================== PASSIVE SYNC TIMING =====================

#define BS_SCAN_TOTAL_MS         60'000UL
#define BS_CANDIDATE_TIMEOUT_MS  30'000UL
#define BS_IN_SYNC_TIMING_US     50'000UL
#define BS_IN_SYNC_TIMEOUT_US    125'000'000UL

// ===================== HMAC =====================

#define HMAC_KEY_LEN    32
#define HMAC_TRUNC_LEN  10

// ===================== PACKET TYPES (wire) =====================

#define PKT_TELEMETRY  0xAF
#define PKT_COMMAND    0x9A
#define PKT_BACKHAUL   0xE2
#define PKT_LOG_CHUNK  0xCA
#define PKT_LONGRANGE  0xBB

// ===================== DEVICE IDs =====================

#define ROCKET_DEVICE_ID  0x92

// ===================== COMMAND IDs =====================

#define CMD_ARM           0x01
#define CMD_DISARM        0x02
#define CMD_FIRE_PYRO     0x03
#define CMD_SET_TX_RATE   0x10
#define CMD_SET_RADIO     0x12
#define CMD_OTA_BEGIN     0x50
#define CMD_OTA_FINALIZE  0x51
#define CMD_OTA_CONFIRM   0x52
#define CMD_PING          0x40
#define CMD_SET_SYNC      0x41
#define CMD_REBOOT        0xF0
#define CMD_LOG_ERASE     0xF1

// ===================== COMMAND ACK CODES =====================

#define CMD_OK              0x00
#define CMD_ERR_UNKNOWN     0x01
#define CMD_ERR_REFUSED     0x02
#define CMD_ERR_BAD_PARAMS  0x03

// ===================== GPS FRACTIONAL ENCODING =====================

#define GPS_FRAC_VALID_MAX    49999
#define GPS_FRAC_NO_FIX       65533
#define GPS_FRAC_INITIALISING 65534
#define GPS_FRAC_NOT_POWERED  65535

#endif // COMMON_RADIO_CONFIG_H
