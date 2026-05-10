// common/radio_config.h — Shared radio protocol constants for rocket and base station.
// Include from both rocket_avionics and base_station. Anything that must match exactly
// between the two firmwares belongs here. No implementations — see common/radio_helpers.h.
//
// The slot/hop/scan machinery has been removed — radio is now a plain RadioLib link
// (rocket TX-on-cadence + RX-when-idle, base RX-when-idle + TX-on-command).

#ifndef COMMON_RADIO_CONFIG_H
#define COMMON_RADIO_CONFIG_H

#include <stdint.h>

// ===================== CHANNEL TABLE (AU915-aligned) =====================
// Channel 0-63:  BW125, 915.2 + ch*0.2 MHz
// Channel 64-71: BW500, 915.9 + (ch-64)*1.6 MHz
#define CHANNEL_COUNT 72

// ===================== LORA MODULATION =====================
// Coding rate, preamble length, sync word are fixed (not command-configurable).
// SF / channel / power are runtime-configurable via CMD_SET_RADIO.

#define LORA_CR         5       // 4/5 coding rate
#define LORA_PREAMBLE   8       // preamble symbols
#define LORA_SYNCWORD   0x12    // private network

// ===================== RADIO DEFAULTS (NVS fallback) =====================

#define DEFAULT_CHANNEL     3
#define DEFAULT_SF          9
#define DEFAULT_POWER       -9    // dBm

#define DEFAULT_BH_CHANNEL  67
#define DEFAULT_BH_SF       5
#define DEFAULT_BH_POWER    -9

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

// ===================== "BASE HEARD" THRESHOLD =====================
// Used by rocket to decide whether the link is live (last command within window).
#define ROCKET_NO_BASE_HEARD_THRESHOLD_US  130'000'000UL

#endif // COMMON_RADIO_CONFIG_H
