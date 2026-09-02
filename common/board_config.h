// board_config.h — Hardware pin assignments, switchable per board variant.
// Single source of truth for all board-specific GPIO mappings.
// Applies to both rocket_avionics and base_station firmware.
// Include via config.h (rocket) or directly (base station).
//
// ===================== NAMING =====================
// Two board FAMILIES, two hardware GENERATIONS. They are independent axes:
//
//              gen 1 (no FEM)        gen 2 (external FEM)
//   Tracker    Tracker V1.1          Tracker V2.x
//   LoRa32     WiFi LoRa 32 V3       WiFi LoRa 32 V4.x
//
// Select exactly one board with a -DBOARD_* flag in platformio.ini build_flags.
// Board macros name the SPECIFIC revision, because revisions differ in ways
// that matter (see BOARD_LORA32_V4_1 vs BOARD_LORA32_V4_3 below).
//
// ===================== CAPABILITY FLAGS =====================
// Firmware code should test the CAPABILITY flags derived at the bottom of this
// file, never the board macro itself. Adding a new board should mean adding a
// pin block here — not touching radio.cpp/main.cpp.
//
//   BOARD_HAS_FEM   — external PA/LNA front-end; needs supply + TX/RX steering.

#ifndef BOARD_CONFIG_H
#define BOARD_CONFIG_H

// ===================== HELTEC WIRELESS TRACKER V1.1 =====================
// ESP32-S3FN8, 8MB internal flash. Gen 1: no FEM, SX1262 drives the antenna
// directly through its own internal RF switch on DIO2. UC6580 GNSS onboard.

#if defined(BOARD_TRACKER_V1)
  #define VEXT_CTRL_PIN         3
  #define VBAT_ADC_CTRL_PIN     2
  #define VBAT_ADC_PIN          1
  #define LED_PIN               18
  #define USER_BTN_PIN          0    // GPIO0 / BOOT button
  #define GPS_RX_PIN            33
  #define GPS_TX_PIN            34
  #define GPS_RST_PIN           35
  #define GPS_BAUD              115200
  #define SENSOR_SDA_PIN        4
  #define SENSOR_SCL_PIN        5
  #define PYRO_CH1_PIN          45   // drogue ejection charge
  #define PYRO_CH2_PIN          46   // main ejection charge
  #define PYRO_CH3_PIN          42   // chute nichrome cut
  #define PYRO_SENSE_CH1_PIN    39   // continuity detect
  #define PYRO_SENSE_CH2_PIN    40
  #define PYRO_SENSE_CH3_PIN    41
  #define PYRO_HV_SENSE_PIN     6    // high-side voltage sense
  // No external FEM — DIO2 drives the SX1262's internal RF switch.
  // Safe SX1262 output range: the chip's full -9..+22 dBm.

// ===================== HELTEC WIFI LORA 32 V4.1 / V4.2 =====================
// ESP32-S3R2, 16MB external flash (W25Q128) + 2MB quad PSRAM (unused).
// Gen 2. FEM = GC1109. No GNSS onboard — 8-pin plug for an external L76K.
//
// FEM control (schematic WiFi_LoRa_32_V4.2, GC1109 truth table Table 4):
//   CSD  (shutdown)      GPIO2   — HIGH = awake
//   CTX  (TX select)     DIO2    — driven by the SX1262 itself
//   CPS  (PA vs bypass)  GPIO46  — HIGH = full PA path, LOW = bypass
//   VFEM_Ctrl (VCC en)   GPIO7   — HIGH = FEM supply up
// So the one pin firmware must steer per-packet is CPS on GPIO46.

#elif defined(BOARD_LORA32_V4_1)
  #define VEXT_CTRL_PIN         36
  #define VGNSS_CTRL_PIN        34   // separate GNSS module power rail (L76K)
  #define VBAT_ADC_CTRL_PIN     37
  #define VBAT_ADC_PIN          1
  #define LED_PIN               35
  #define USER_BTN_PIN          0    // GPIO0 / BOOT button
  #define GPS_RX_PIN            38   // L76K GNSS module UART (plug-in)
  #define GPS_TX_PIN            39
  #define GPS_RST_PIN           42
  #define GPS_BAUD              9600   // L76K default (vs UC6580 at 115200)
  #define SENSOR_SDA_PIN        17   // shared with onboard OLED (SSD1315 @ 0x3C)
  #define SENSOR_SCL_PIN        18
  #define PYRO_CH1_PIN          45   // UNVERIFIED — no carrier board exists yet
  #define PYRO_CH2_PIN          47
  #define PYRO_CH3_PIN          48
  #define PYRO_SENSE_CH1_PIN    4
  #define PYRO_SENSE_CH2_PIN    5
  #define PYRO_SENSE_CH3_PIN    6
  #define PYRO_HV_SENSE_PIN     3
  #define BOARD_HAS_FEM         1
  #define LORA_FEM_EN_PIN       2    // CSD  — held HIGH after init
  #define LORA_FEM_CTL_PIN      7    // VFEM_Ctrl — FEM supply, HIGH after init
  #define LORA_FEM_TX_PIN       46   // CPS  — HIGH during TX, LOW during RX
  // GC1109 absolute maximum input (datasheet Table 1): +5 dBm at the TX port.
  // The SX1262 RFO feeds the FEM through a matching network with NO pad, so
  // SX1262 output == FEM input. Keep SX1262 output at or below 0 dBm.
  #define BOARD_FEM_MAX_DRIVE_DBM  0

// ===================== HELTEC WIFI LORA 32 V4.3 =====================
// Same MCU/flash as V4.1. Gen 2. FEM = KCT8103L (upgraded from GC1109).
//
// FEM control (schematic HTIT-WB32LAF_V4.3):
//   CSD  (shutdown)      GPIO2   — HIGH = awake            (unchanged from V4.1)
//   CTX  (TX select)     GPIO5   — MOVED off DIO2 onto a GPIO
//   CPS  (LNA bypass)    DIO2    — now the SX1262-driven line
//   VFEM_Ctrl (VCC en)   GPIO7   — HIGH = FEM supply up    (unchanged from V4.1)
//   GPIO46               NC      — freed, available for user application
//
// The V4.1 -> V4.3 change swaps which FEM line sits on DIO2. Firmware must
// drive CTX on GPIO5 per-packet; leaving it low pins the FEM in receive mode
// and no transmit power reaches the antenna.

#elif defined(BOARD_LORA32_V4_3)
  #define VEXT_CTRL_PIN         36
  #define VGNSS_CTRL_PIN        34
  #define VBAT_ADC_CTRL_PIN     37
  #define VBAT_ADC_PIN          1
  #define LED_PIN               35
  #define USER_BTN_PIN          0
  #define GPS_RX_PIN            38
  #define GPS_TX_PIN            39
  #define GPS_RST_PIN           42
  #define GPS_BAUD              9600
  #define SENSOR_SDA_PIN        17
  #define SENSOR_SCL_PIN        18
  #define PYRO_CH1_PIN          45   // UNVERIFIED — no carrier board exists yet
  #define PYRO_CH2_PIN          47
  #define PYRO_CH3_PIN          48
  #define PYRO_SENSE_CH1_PIN    4
  // NOTE: GPIO5 is FEM CTX on this revision — NOT available for pyro sense.
  // GPIO46 is free here (NC), so it takes the channel 2 sense line instead.
  #define PYRO_SENSE_CH2_PIN    46
  #define PYRO_SENSE_CH3_PIN    6
  #define PYRO_HV_SENSE_PIN     3
  #define BOARD_HAS_FEM         1
  #define LORA_FEM_EN_PIN       2    // CSD  — held HIGH after init
  #define LORA_FEM_CTL_PIN      7    // VFEM_Ctrl — FEM supply, HIGH after init
  #define LORA_FEM_TX_PIN       5    // CTX  — HIGH during TX, LOW during RX
  // KCT8103L datasheet not on hand. Same topology and same ~30 dB gain to reach
  // Heltec's quoted 28 dBm, so assume the GC1109's +5 dBm input maximum until
  // proven otherwise. Keep SX1262 output at or below 0 dBm.
  #define BOARD_FEM_MAX_DRIVE_DBM  0

//TODO: add tracker v2 — gen 2, same era as lora32 v4.
// Has the external FEM, but keeps the onboard UC6580 GNSS and puts everything
// else on different GPIOs. Uses a TFT rather than an OLED, so no shared I2C bus.
// Reported FEM control: pa_ctx on gpio4, pa_csd on gpio4, vfem_ctrl on gpio7 —
// needs checking against References/Heltec Wireless Tracker v2/ before use.

#else
  #error "No board defined. Add one of -DBOARD_TRACKER_V1 / -DBOARD_LORA32_V4_1 / -DBOARD_LORA32_V4_3 to platformio.ini build_flags."
#endif

// ===================== FEM TIMING =====================
// GC1109 datasheet note 2: VBAT must come up before the CSD/CPS/CTX control
// pins. Settling time is sub-microsecond; 10us is a generous guard that stays
// well inside the loop timing budget and is only ever paid once, during init.
#ifdef BOARD_HAS_FEM
  #define LORA_FEM_SUPPLY_SETTLE_US  10
#endif

#endif // BOARD_CONFIG_H
