/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2023 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 */
#pragma once

#include "env_validate.h"

#define BOARD_INFO_NAME      "GT OCTOPUS PRO V1.0"

#define USES_DIAG_JUMPERS

// Onboard I2C EEPROM
#if EITHER(NO_EEPROM_SELECTED, I2C_EEPROM)
  #undef NO_EEPROM_SELECTED
  #define I2C_EEPROM
  #define SOFT_I2C_EEPROM                         // Force the use of Software I2C
  #define I2C_SCL_PIN                       PB8
  #define I2C_SDA_PIN                       PB9
  #define MARLIN_EEPROM_SIZE              0x1000  // 4KB
#endif

// Avoid conflict with TIMER_TONE
#define STEP_TIMER 8

//
// Servos
//
#define SERVO0_PIN                          PB7

//
// Misc. Functions
//
#define LED_PIN                             PA13    //WORK led

//
// Trinamic Stallguard pins
//
#define X_DIAG_PIN                          PG6   // M1-STOP
#define Y_DIAG_PIN                          PG9   // M2-STOP
#define Z_DIAG_PIN                          PG10   // M3-STOP
#define Z2_DIAG_PIN                         PG11   // M4-STOP
#define E0_DIAG_PIN                         PG12   // M5-STOP
#define E1_DIAG_PIN                         PG13  // M6-STOP
#define E2_DIAG_PIN                         PG14  // PWRDET
#define E3_DIAG_PIN                         PG15  // PWRDET
//
// Z Probe (when not Z_MIN_PIN)
//
#ifndef Z_MIN_PROBE_PIN
  #define Z_MIN_PROBE_PIN                   PB6
#endif

//
// Check for additional used endstop pins
//
#if HAS_EXTRA_ENDSTOPS
  #define _ENDSTOP_IS_ANY(ES) X2_USE_ENDSTOP == ES || Y2_USE_ENDSTOP == ES || Z2_USE_ENDSTOP == ES || Z3_USE_ENDSTOP == ES || Z4_USE_ENDSTOP == ES
  #if _ENDSTOP_IS_ANY(_XMIN_) || _ENDSTOP_IS_ANY(_XMAX_)
    #define NEEDS_X_MINMAX
  #endif
  #if _ENDSTOP_IS_ANY(_YMIN_) || _ENDSTOP_IS_ANY(_YMAX_)
    #define NEEDS_Y_MINMAX
  #endif
  #if _ENDSTOP_IS_ANY(_ZMIN_) || _ENDSTOP_IS_ANY(_ZMAX_)
    #define NEEDS_Z_MINMAX
  #endif
  #undef _ENDSTOP_IS_ANY
#endif

//
// Limit Switches
//
#ifdef X_STALL_SENSITIVITY
  #define X_STOP_PIN                  X_DIAG_PIN
  #if X_HOME_TO_MIN
    #define X_MAX_PIN                E0_DIAG_PIN  // E0DET
  #else
    #define X_MIN_PIN                E0_DIAG_PIN  // E0DET
  #endif
#elif EITHER(DUAL_X_CARRIAGE, NEEDS_X_MINMAX)
  #ifndef X_MIN_PIN
    #define X_MIN_PIN                 X_DIAG_PIN  // X-STOP
  #endif
  #ifndef X_MAX_PIN
    #define X_MAX_PIN                E0_DIAG_PIN  // E0DET
  #endif
#else
  #define X_STOP_PIN                  X_DIAG_PIN  // X-STOP
#endif

#ifdef Y_STALL_SENSITIVITY
  #define Y_STOP_PIN                  Y_DIAG_PIN
  #if Y_HOME_TO_MIN
    #define Y_MAX_PIN                E1_DIAG_PIN  // E1DET
  #else
    #define Y_MIN_PIN                E1_DIAG_PIN  // E1DET
  #endif
#elif ENABLED(NEEDS_Y_MINMAX)
  #ifndef Y_MIN_PIN
    #define Y_MIN_PIN                 Y_DIAG_PIN  // Y-STOP
  #endif
  #ifndef Y_MAX_PIN
    #define Y_MAX_PIN                E1_DIAG_PIN  // E1DET
  #endif
#else
  #define Y_STOP_PIN                  Y_DIAG_PIN  // Y-STOP
#endif

#ifdef Z_STALL_SENSITIVITY
  #define Z_STOP_PIN                  Z_DIAG_PIN
  #if Z_HOME_TO_MIN
    #define Z_MAX_PIN                E2_DIAG_PIN  // PWRDET
  #else
    #define Z_MIN_PIN                E2_DIAG_PIN  // PWRDET
  #endif
#elif ENABLED(NEEDS_Z_MINMAX)
  #ifndef Z_MIN_PIN
    #define Z_MIN_PIN                 Z_DIAG_PIN  // Z-STOP
  #endif
  #ifndef Z_MAX_PIN
    #define Z_MAX_PIN                E2_DIAG_PIN  // PWRDET
  #endif
#else
  #define Z_STOP_PIN                  Z_DIAG_PIN  // Z-STOP
#endif

//
// Filament Runout Sensor
//
#define FIL_RUNOUT_PIN                      PF1   // E0DET
#define FIL_RUNOUT2_PIN                     PF15  // E1DET

//
// Power Supply Control
//
#ifndef PS_ON_PIN
  #define PS_ON_PIN                         PE11  // PS-ON
#endif

//
// Power Loss Detection
//
#ifndef POWER_LOSS_PIN
  #define POWER_LOSS_PIN                    PC0  // PWRDET
#endif

//
// Steppers
//

#define X_STEP_PIN                          PF13  // MOTOR 1
#define X_DIR_PIN                           PF12
#define X_ENABLE_PIN                        PF14
#ifndef X_CS_PIN
  #define X_CS_PIN                          PC4
#endif

#define Y_STEP_PIN                          PG0   // MOTOR 2
#define Y_DIR_PIN                           PG1
#define Y_ENABLE_PIN                        PF15
#ifndef Y_CS_PIN
  #define Y_CS_PIN                          PD11
#endif

#define Z_STEP_PIN                          PF11   // MOTOR 3
#define Z_DIR_PIN                           PG3
#define Z_ENABLE_PIN                        PG5
#ifndef Z_CS_PIN
  #define Z_CS_PIN                          PC6
#endif

#define Z2_STEP_PIN                         PG4   // MOTOR 4
#define Z2_DIR_PIN                          PC1
#define Z2_ENABLE_PIN                       PA0
#ifndef Z2_CS_PIN
  #define Z2_CS_PIN                         PC7
#endif

#define E0_STEP_PIN                         PF9   // MOTOR 5
#define E0_DIR_PIN                          PF10
#define E0_ENABLE_PIN                       PG2
#ifndef E0_CS_PIN
  #define E0_CS_PIN                         PF2
#endif

#define E1_STEP_PIN                         PC13  // MOTOR 6
#define E1_DIR_PIN                          PF0
#define E1_ENABLE_PIN                       PF1
#ifndef E1_CS_PIN
  #define E1_CS_PIN                         PE4
#endif

#define E2_STEP_PIN                         PE2   // MOTOR 7
#define E2_DIR_PIN                          PE3
#define E2_ENABLE_PIN                       PD4
#ifndef E2_CS_PIN
  #define E2_CS_PIN                         PE1
#endif

#define E3_STEP_PIN                         PE6  // MOTOR 8
#define E3_DIR_PIN                          PA14
#define E3_ENABLE_PIN                       PE0
#ifndef E3_CS_PIN
  #define E3_CS_PIN                         PD3
#endif

#if 0
#define E4_STEP_PIN                         PA8   // MOTOR 9
#define E4_DIR_PIN                          PC7
#define E4_ENABLE_PIN                       PC9
#ifndef E4_CS_PIN
  #define E4_CS_PIN                         PG8
#endif

#define E5_STEP_PIN                         PG6   // MOTOR 10
#define E5_DIR_PIN                          PC6
#define E5_ENABLE_PIN                       PC8
#ifndef E5_CS_PIN
  #define E5_CS_PIN                         PG7
#endif

#endif
//
// Temperature Sensors
//
#define TEMP_BED_PIN                        PF3   // TB
#define TEMP_0_PIN                          PF4   // TH0
#define TEMP_1_PIN                          PF5   // TH1
#define TEMP_2_PIN                          PF6   // TH2
#define TEMP_3_PIN                          PF7   // TH3

//
// Heaters / Fans
//
#define HEATER_BED_PIN                      PA1   // Hotbed
#define HEATER_0_PIN                        PA2   // Heater0
#define HEATER_1_PIN                        PA3   // Heater1
#define HEATER_2_PIN                        PB10   // Heater2
#define HEATER_3_PIN                        PB11   // Heater3

#define FAN_PIN                             PA8   // Fan0
#define FAN1_PIN                            PE5   // Fan1
#define FAN2_PIN                            PD12   // Fan2
#define FAN3_PIN                            PD13   // Fan3
#define FAN4_PIN                            PD14   // Fan4 
#define FAN5_PIN                            PD15   //  Fan5
//#define FAN6_PIN                            PA2   // Fan6

//
// SD Support
//
#ifndef SDCARD_CONNECTION
  #if HAS_WIRED_LCD
    #define SDCARD_CONNECTION                LCD
  #else
    #define SDCARD_CONNECTION            ONBOARD
  #endif
#endif

//
// Software SPI pins for TMC2130 stepper drivers
//
#if ENABLED(TMC_USE_SW_SPI)
  #ifndef TMC_SW_MOSI
    #define TMC_SW_MOSI                     PE14
  #endif
  #ifndef TMC_SW_MISO
    #define TMC_SW_MISO                     PE13
  #endif
  #ifndef TMC_SW_SCK
    #define TMC_SW_SCK                      PE12
  #endif
#endif

#if HAS_TMC_UART
  /**
   * TMC2208/TMC2209 stepper drivers
   *
   * Hardware serial communication ports.
   * If undefined software serial is used according to the pins below
   */
  //#define X_HARDWARE_SERIAL  Serial1
  //#define X2_HARDWARE_SERIAL Serial1
  //#define Y_HARDWARE_SERIAL  Serial1
  //#define Y2_HARDWARE_SERIAL Serial1
  //#define Z_HARDWARE_SERIAL  Serial1
  //#define Z2_HARDWARE_SERIAL Serial1
  //#define E0_HARDWARE_SERIAL Serial1
  //#define E1_HARDWARE_SERIAL Serial1
  //#define E2_HARDWARE_SERIAL Serial1
  //#define E3_HARDWARE_SERIAL Serial1
  //#define E4_HARDWARE_SERIAL Serial1

  #define X_SERIAL_TX_PIN                   PG14
  #define X_SERIAL_RX_PIN        X_SERIAL_TX_PIN

  #define Y_SERIAL_TX_PIN                   PG13
  #define Y_SERIAL_RX_PIN        Y_SERIAL_TX_PIN

  #define Z_SERIAL_TX_PIN                   PG12
  #define Z_SERIAL_RX_PIN        Z_SERIAL_TX_PIN

  #define Z2_SERIAL_TX_PIN                  PG11
  #define Z2_SERIAL_RX_PIN      Z2_SERIAL_TX_PIN

  #define E0_SERIAL_TX_PIN                  PG10
  #define E0_SERIAL_RX_PIN      E0_SERIAL_TX_PIN

  #define E1_SERIAL_TX_PIN                  PG9
  #define E1_SERIAL_RX_PIN      E1_SERIAL_TX_PIN

  #define E2_SERIAL_TX_PIN                  PD7
  #define E2_SERIAL_RX_PIN      E2_SERIAL_TX_PIN

  #define E3_SERIAL_TX_PIN                  PD6
  #define E3_SERIAL_RX_PIN      E3_SERIAL_TX_PIN

  #define E4_SERIAL_TX_PIN                  PG8
  #define E4_SERIAL_RX_PIN      E3_SERIAL_TX_PIN

  #define E5_SERIAL_TX_PIN                  PG7
  #define E5_SERIAL_RX_PIN      E3_SERIAL_TX_PIN

  // Reduce baud rate to improve software serial reliability
  #define TMC_BAUD_RATE                    19200
#endif

/**
 *                ------                                   ------
 *  (BEEPER) PE8  | 1  2 | PE7 (BTN_ENC)       (MISO)  PA6  | 1  2 | PA5 (SCK)
 *  (LCD_EN) PE9  | 3  4 | PE10 (LCD_RS)     (BTN_EN2)  PB1 | 3  4 | PA4  (SD_SS)
 *  (LCD_D4) PE12 | 5  6 |  PE13 (LCD_D5)     (BTN_EN1) PB2 | 5  6 |  PA7 (MOSI)
 *  (LCD_D6) PE14 | 7  8 | PE15 (LCD_D7)   (SD_DETECT) PC15 | 7  8 | PF2   RST
 *           GND  | 9 10 | 5V                          GND | 9 10 | --
 *                ------                                   ------
 *                 EXP1                                     EXP2
 */

#define EXP1_01_PIN                         PE8
#define EXP1_02_PIN                         PE7
#define EXP1_03_PIN                         PE9
#define EXP1_04_PIN                         PE10
#define EXP1_05_PIN                         PE12
#define EXP1_06_PIN                         PE13
#define EXP1_07_PIN                         PE14
#define EXP1_08_PIN                         PE15

#define EXP2_01_PIN                         PA6
#define EXP2_02_PIN                         PA5
#define EXP2_03_PIN                         PB1
#define EXP2_04_PIN                         PA4
#define EXP2_05_PIN                         PB2
#define EXP2_06_PIN                         PA7
#define EXP2_07_PIN                         PC15
#define EXP2_08_PIN                         -1    //RST

//
// Onboard SD card
// Must use soft SPI because Marlin's default hardware SPI is tied to LCD's EXP2
//
#if SD_CONNECTION_IS(ONBOARD)
  #ifndef SD_DETECT_STATE
    #define SD_DETECT_STATE HIGH
  #elif SD_DETECT_STATE == LOW
    #error "BOARD_BTT_OCTOPUS_MAX_EZ onboard SD requires SD_DETECT_STATE set to HIGH."
  #endif
  #define SDSS                              PB12
  #define SD_SS_PIN                         SDSS
  #define SD_SCK_PIN                        PE12
  #define SD_MISO_PIN                       PE13
  #define SD_MOSI_PIN                       PE14
  #define SD_DETECT_PIN                     PB13
  #define SOFTWARE_SPI
#elif SD_CONNECTION_IS(LCD)
  #define SDSS                       EXP2_04_PIN
  #define SD_SS_PIN                         SDSS
  #define SD_SCK_PIN                 EXP2_02_PIN
  #define SD_MISO_PIN                EXP2_01_PIN
  #define SD_MOSI_PIN                EXP2_06_PIN
  #define SD_DETECT_PIN              EXP2_07_PIN
  #define SOFTWARE_SPI
#elif SD_CONNECTION_IS(CUSTOM_CABLE)
  #error "CUSTOM_CABLE is not a supported SDCARD_CONNECTION for this board"
#endif

//
// LCDs and Controllers
//
#if IS_TFTGLCD_PANEL

  #if ENABLED(TFTGLCD_PANEL_SPI)
    #define TFTGLCD_CS               EXP2_03_PIN
  #endif

#elif HAS_WIRED_LCD

  #define BEEPER_PIN                 EXP1_01_PIN
  #define BTN_ENC                    EXP1_02_PIN

  #if ENABLED(CR10_STOCKDISPLAY)

    #define LCD_PINS_RS              EXP1_07_PIN

    #define BTN_EN1                  EXP1_03_PIN
    #define BTN_EN2                  EXP1_05_PIN

    #define LCD_PINS_ENABLE          EXP1_08_PIN
    #define LCD_PINS_D4              EXP1_06_PIN

  #else

    #define LCD_PINS_RS              EXP1_04_PIN

    #define BTN_EN1                  EXP2_03_PIN
    #define BTN_EN2                  EXP2_05_PIN

    #define LCD_PINS_ENABLE          EXP1_03_PIN
    #define LCD_PINS_D4              EXP1_05_PIN

    #if ENABLED(FYSETC_MINI_12864)
      #define DOGLCD_CS              EXP1_03_PIN
      #define DOGLCD_A0              EXP1_04_PIN
      #define DOGLCD_SCK             EXP2_02_PIN
      #define DOGLCD_MOSI            EXP2_06_PIN

      #define SOFTWARE_SPI
      #define FORCE_SOFT_SPI                      // Use this if default of hardware SPI causes display problems
                                                  //   results in LCD soft SPI mode 3, SD soft SPI mode 0
      //#define LCD_BACKLIGHT_PIN           -1
      #define LCD_RESET_PIN          EXP1_05_PIN  // Must be high or open for LCD to operate normally.
      #if EITHER(FYSETC_MINI_12864_1_2, FYSETC_MINI_12864_2_0)
        #ifndef RGB_LED_R_PIN
          #define RGB_LED_R_PIN      EXP1_06_PIN
        #endif
        #ifndef RGB_LED_G_PIN
          #define RGB_LED_G_PIN      EXP1_07_PIN
        #endif
        #ifndef RGB_LED_B_PIN
          #define RGB_LED_B_PIN      EXP1_08_PIN
        #endif
      #elif ENABLED(FYSETC_MINI_12864_2_1)
        #define NEOPIXEL_PIN         EXP1_06_PIN
      #endif
    #endif // !FYSETC_MINI_12864

    #if IS_ULTIPANEL
      #define LCD_PINS_D5            EXP1_06_PIN
      #define LCD_PINS_D6            EXP1_07_PIN
      #define LCD_PINS_D7            EXP1_08_PIN

      #if ENABLED(REPRAP_DISCOUNT_FULL_GRAPHIC_SMART_CONTROLLER)
        #define BTN_ENC_EN           LCD_PINS_D7  // Detect the presence of the encoder
      #endif

    #endif

  #endif
#endif  // HAS_WIRED_LCD

// Alter timing for graphical display
#if IS_U8GLIB_ST7920
  #define BOARD_ST7920_DELAY_1               120
  #define BOARD_ST7920_DELAY_2                80
  #define BOARD_ST7920_DELAY_3               580
#endif

#if HAS_SPI_TFT
  #define TFT_CS_PIN                 EXP2_04_PIN
  #define TFT_A0_PIN                 EXP2_07_PIN
  #define TFT_SCK_PIN                EXP2_02_PIN
  #define TFT_MISO_PIN               EXP2_01_PIN
  #define TFT_MOSI_PIN               EXP2_06_PIN

  #define TOUCH_INT_PIN              EXP1_07_PIN
  #define TOUCH_MISO_PIN             EXP1_06_PIN
  #define TOUCH_MOSI_PIN             EXP1_03_PIN
  #define TOUCH_SCK_PIN              EXP1_05_PIN
  #define TOUCH_CS_PIN               EXP1_04_PIN

  #define BTN_EN1                    EXP2_03_PIN
  #define BTN_EN2                    EXP2_05_PIN
  #define BTN_ENC                    EXP1_02_PIN
#endif

//
// NeoPixel LED
//
#ifndef NEOPIXEL_PIN
  #define NEOPIXEL_PIN                      EXP1_06_PIN
#endif

#ifndef NEOPIXEL2_PIN
  #define NEOPIXEL2_PIN                     PB0
#endif

#if ENABLED(WIFISUPPORT)
  //
  // WIFI
  //

  /**
   *                      -------
   *            GND | 9  |       | 8 | 3.3V
   *  (ESP-CS) PG1  | 10 |       | 7 | PB15 (ESP-MOSI)
   *           3.3V | 11 |       | 6 | PB14 (ESP-MISO)
   * (ESP-IO0)  PG0 | 12 |       | 5 | PB13 (ESP-CLK)
   * (ESP-IO4) PF15 | 13 |       | 4 | --
   *             -- | 14 |       | 3 | 3.3V (ESP-EN)
   *  (ESP-RX)  PE7 | 15 |       | 2 | --
   *  (ESP-TX)  PE8 | 16 |       | 1 | PB2 (ESP-RST)
   *                      -------
   *                       WIFI
   */
  #define ESP_WIFI_MODULE_COM                  7  // Must also set either SERIAL_PORT or SERIAL_PORT_2 to this
  #define ESP_WIFI_MODULE_BAUDRATE      BAUDRATE  // Must use same BAUDRATE as SERIAL_PORT & SERIAL_PORT_2
  #define ESP_WIFI_MODULE_RESET_PIN         PB2
  #define ESP_WIFI_MODULE_GPIO0_PIN         PG0
  #define ESP_WIFI_MODULE_GPIO4_PIN         PF15
#endif
