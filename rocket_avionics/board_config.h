// board_config.h — Hardware pin assignments, switchable per board variant.
// Single source of truth for all board-specific GPIO mappings.
// Include this in config.h and sensors.h; use the resulting defines throughout.

#ifndef BOARD_CONFIG_H
#define BOARD_CONFIG_H

// ===================== BOARD SELECTION (via -DBOARD_WIRELESS_TRACKER or -DBOARD_LORA32_V4) =====================

#if defined(BOARD_WIRELESS_TRACKER)
  // --- Heltec Wireless Tracker V1.1 (ESP32-S3FN8, 8MB internal flash) ---
  #define VEXT_CTRL_PIN         3
  #define VBAT_ADC_CTRL_PIN     2
  #define VBAT_ADC_PIN          1
  #define LED_PIN               18
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
  #define LORA_TCXO_VOLTAGE     SX126X_TCXO_CTRL_1_8V
  // No external FEM on Tracker — DIO2 RF switch internal to SX1262

#elif defined(BOARD_LORA32_V4)
  // --- Heltec WiFi LoRa 32 V4 (ESP32-S3R2, 16MB external flash + 2MB PSRAM) ---
  #define VEXT_CTRL_PIN         36
  #define VGNSS_CTRL_PIN        34   // V4-only: separate GNSS module power rail (L76K)
  #define VBAT_ADC_CTRL_PIN     37
  #define VBAT_ADC_PIN          1    // same ADC pin, different control
  #define LED_PIN               35
  #define GPS_RX_PIN            38   // L76K GNSS module UART
  #define GPS_TX_PIN            39
  #define GPS_RST_PIN           42
  #define GPS_BAUD              9600   // L76K default (vs UC6580 at 115200)
  #define SENSOR_SDA_PIN        17   // OLED shared I2C bus
  #define SENSOR_SCL_PIN        18
  #define PYRO_CH1_PIN          45   // drogue ejection charge (same as Tracker)
  #define PYRO_CH2_PIN          47   // main ejection charge (different GPIO)
  #define PYRO_CH3_PIN          48   // chute nichrome cut (different GPIO)
  #define PYRO_SENSE_CH1_PIN    4    // continuity detect (reuse freed-up ADC pins)
  #define PYRO_SENSE_CH2_PIN    5
  #define PYRO_SENSE_CH3_PIN    6
  #define PYRO_HV_SENSE_PIN     3    // high-side voltage sense (different GPIO)
  #define LORA_TCXO_VOLTAGE     SX126X_TCXO_CTRL_1_8V  // same as Tracker
  // External FEM on V4 — needs PA control and init
  #define LORA_FEM_EN_PIN       2    // always HIGH after init (amplifier enable)
  #define LORA_FEM_CTL_PIN      7    // always HIGH after init (antenna control)
  #define LORA_FEM_PA_PIN       46   // HIGH during TX, LOW during RX (PA bypass)
/*
//TODO: need to fix this - i think we might have been using the wrong fem settings. todo; figure out what these do in arduino or the heltec code. probable the fem - pa_ctx / pa_csd / vfem ctrl?
//figure ou twhat the bolow stuff causes in the heltec/arduino stack and make sure we apply it, too
For the WiFi LoRa 32 V4 development board, different hardware revisions also require selecting the correct LoRa FEM option in the Tools menu.

V4.2, select USE_GC1109_PA.
V4.3, select USE_KCT8103L_PA.

2026-2-25 update

The FEM chip has been upgraded to KCT8103L, allowing software control over whether the RX signal path passes through the LNA.

GPIO Adjustments

GPIO5 has been reassigned as the FEM control pin.

GPIO46 is now reserved as an available GPIO for user applications.

*/

//TODO: add tracker v2 - its same generation as lora32 v4 
//so it has the external fem/pa, but it has the built in uc6580 and other pins on different gpios
//uses tft not oled, so not a shared i2c bus, etc
 //pa_ctx on gpio4/ pa_csd on gpio4 and vfem ctrl on gpio7

#else
  #error "No board defined. Add -DBOARD_WIRELESS_TRACKER or -DBOARD_LORA32_V4 to platformio.ini build_flags."
#endif

#endif // BOARD_CONFIG_H
