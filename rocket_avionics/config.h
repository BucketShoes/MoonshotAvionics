// config.h — Rocket-specific compile-time constants.
// Anything that must match base_station lives in ../common/radio_config.h.
// Inline helpers (channel table, hop sequence, txRate) live in ../common/radio_helpers.h.

#ifndef CONFIG_H
#define CONFIG_H

#include "board_config.h"
#include "../common/radio_config.h"
#include "../common/radio_helpers.h"

// ===================== PIN DEFINITIONS =====================

#define LORA_NSS_PIN  8
#define LORA_SCK_PIN  9
#define LORA_MOSI_PIN 10
#define LORA_MISO_PIN 11
#define LORA_RST_PIN  12
#define LORA_BUSY_PIN 13
#define LORA_DIO1_PIN 14

#define USER_BTN_PIN  0

#define VBAT_MULTIPLIER   4.9f

// ===================== GENERAL =====================

#define SERIAL_BAUD   115200
#define DEVICE_ID     ROCKET_DEVICE_ID

// ===================== ROCKET-LOCAL HARDWARE =====================

// HV sense threshold: 10k:100k divider, 250mV ADC ≈ 2.75V high-side.
#define PYRO_HV_PRESENT_MV    2750
#define PYRO_HV_DIVIDER_RATIO 11

// ===================== DEBUG LOG FLAGS =====================
// These are local debug switches; the two firmwares can choose independently.

#define LOG_RX_DONE     true
#define LOG_RX_TIMEOUT  true
#define LOG_RX_START    true
#define LOG_TX_START    true
#define LOG_TX_DONE     true
#define LOG_APPLYCFG    true

// ===================== SLOT CONFIG STRUCT =====================
// Per-slot radio parameters. Currently all slots use the same config.

#include "sx126x.h"

struct SlotConfig {
  uint32_t              freqHz;
  sx126x_lora_sf_t      sf;
  sx126x_lora_bw_t      bw;
  sx126x_lora_cr_t      cr;
  uint16_t              preambleSymbols;
  sx126x_lora_pkt_len_modes_t headerType;
  uint8_t               payloadLen;
};

// ===================== TIMING =====================

#define BTN_DEBOUNCE_US        50000UL
#define VBAT_READ_INTERVAL_MS  2000
#define VBAT_SETTLE_US         400
#define MIN_LOOP_US            10UL
#define SLOW_LOOP_THRESHOLD_US 10'000UL

// ===================== NVS =====================

#define NVS_NAMESPACE   "rocket"

// ===================== DATA PAGE TYPE CONSTANTS =====================
// TODO: these are wire constants, not config — move out of config.h.

#define PAGE_THRUST_CURVE      0x0E
#define PAGE_PYRO_STATUS       0x0F

// ===================== THRUST CURVE =====================

#define THRUST_BUF_SIZE        2048
#define THRUST_SAMPLE_RATE_HZ  200

// ===================== LOG =====================

#define LOG_SNR_LOCAL  0x7F   // SNR value meaning "locally generated, not received"

// ===================== BLE =====================

#define BLE_DEVICE_NAME        "Moonshot-Rocket"
#define BLE_SERVICE_UUID       "524f434b-4554-5354-424c-000000000000"
#define BLE_TELEM_CHAR_UUID    "524f434b-4554-5354-424c-000000000001"
#define BLE_CMD_CHAR_UUID      "524f434b-4554-5354-424c-000000000002"
#define BLE_STATUS_CHAR_UUID   "524f434b-4554-5354-424c-000000000003"
#define BLE_CONNSET_CHAR_UUID  "524f434b-4554-5354-424c-000000000004"
#define BLE_LOGFETCH_CHAR_UUID "524f434b-4554-5354-424c-000000000005"
#define BLE_OTA_CHAR_UUID      "524f434b-4554-5354-424c-000000000006"

#define BLE_CONNSET_INTERVAL  0x01
#define BLE_CONNSET_PAGEMASK  0x02
#define BLE_CONNSET_PHY       0x03

#define BLE_LOGFETCH_MAX_PDU   514
#define BLE_FETCH_VERBOSE      1

#define BLE_ADV_INTERVAL       1600

#define BLE_CI_FAST_MIN        6
#define BLE_CI_FAST_MAX        6
#define BLE_CI_FAST_LATENCY    0
#define BLE_CI_FAST_TIMEOUT    800

#define BLE_CI_SLOW_MIN        24
#define BLE_CI_SLOW_MAX        40
#define BLE_CI_SLOW_LATENCY    4
#define BLE_CI_SLOW_TIMEOUT    3200

#define BLE_DEFAULT_INTERVAL_US  1000000UL
#define BLE_DEFAULT_PAGE_MASK    0x0000000000003FFEULL
#define BLE_OVF_BUF_SIZE         4096

#endif // CONFIG_H
