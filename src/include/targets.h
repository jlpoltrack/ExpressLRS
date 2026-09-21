#pragma once
#if !defined TARGET_NATIVE
#include <Arduino.h>
#endif

// Used to XOR with OtaCrcInitializer and macSeed to reduce compatibility with previous versions.
// It should be incremented when the OTA packet structure is modified.
#define OTA_VERSION_ID      4

#define UNDEF_PIN (-1)

/// General Features ///
#define LED_MAX_BRIGHTNESS 50 //0..255 for max led brightness

/////////////////////////

#define WORD_ALIGNED_ATTR __attribute__((aligned(4)))
#define WORD_PADDED(size) (((size)+3) & ~3)

#undef ICACHE_RAM_ATTR //fix to allow both esp32 and esp8266 to use ICACHE_RAM_ATTR for mapping to IRAM
#define ICACHE_RAM_ATTR IRAM_ATTR

#if defined(TARGET_NATIVE)
#define IRAM_ATTR
#include "native.h"
#endif

#if defined(PLATFORM_RP2)
// .time_critical* lands in .data, which the core's linker script copies into RAM at boot,
// worth -25% alarm latency and -19% ISR average for 8.7KB on a Pico 2W + LR2021 RX. Each
// site needs a unique section name: ordinary functions collide otherwise, the same reason
// the SDK's __not_in_flash_func appends the function name.
#define _IRAM_STR2(x) #x
#define _IRAM_STR(x) _IRAM_STR2(x)
#define IRAM_ATTR __attribute__((section(".time_critical." _IRAM_STR(__COUNTER__))))
#endif

// Every ESP has WiFi; on RP2 it is the board's CYW43 module, so follow the flag
// the core itself keys off rather than naming each "W" board
#if defined(PLATFORM_ESP32) || defined(PLATFORM_ESP8266)
#define HAS_WIFI
#elif defined(PLATFORM_RP2) && defined(PICO_CYW43_SUPPORTED)
#define HAS_WIFI
#endif

#if defined(PLATFORM_ESP32) || defined(PLATFORM_RP2)
// A second hardware UART is free for the Serial1 output protocols
#define HAS_SERIAL1
#endif

/*
 * Features
 * define features based on pins before defining pins as UNDEF_PIN
 */

// Using these DEBUG_* imply that no SerialIO will be used so the output is readable
#if !defined(DEBUG_CRSF_NO_OUTPUT) && defined(TARGET_RX) && (defined(DEBUG_RCVR_LINKSTATS) || defined(DEBUG_RX_SCOREBOARD) || defined(DEBUG_RCVR_SIGNAL_STATS))
#define DEBUG_CRSF_NO_OUTPUT
#endif

#if defined(DEBUG_CRSF_NO_OUTPUT)
#define OPT_CRSF_RCVR_NO_SERIAL true
#elif defined(TARGET_RX)
extern bool pwmSerialDefined;

#define OPT_CRSF_RCVR_NO_SERIAL (GPIO_PIN_RCSIGNAL_RX == UNDEF_PIN && GPIO_PIN_RCSIGNAL_TX == UNDEF_PIN && !pwmSerialDefined)
#else
#define OPT_CRSF_RCVR_NO_SERIAL false
#endif

#if defined(RADIO_SX128X)
#define Regulatory_Domain_ISM_2400 1
// ISM 2400 band is in use => undefine other requlatory domain defines
#undef Regulatory_Domain_AU_915
#undef Regulatory_Domain_EU_868
#undef Regulatory_Domain_IN_866
#undef Regulatory_Domain_FCC_915
#undef Regulatory_Domain_AU_433
#undef Regulatory_Domain_EU_433
#undef Regulatory_Domain_US_433
#undef Regulatory_Domain_US_433_WIDE
#undef Regulatory_Domain_TH_920

#elif defined(RADIO_SX127X) || defined(RADIO_LR1121) || defined(RADIO_LR2021)
#if !(defined(Regulatory_Domain_AU_915) || defined(Regulatory_Domain_FCC_915) || \
        defined(Regulatory_Domain_EU_868) || defined(Regulatory_Domain_IN_866) || \
        defined(Regulatory_Domain_AU_433) || defined(Regulatory_Domain_EU_433) || \
        defined(Regulatory_Domain_US_433) || defined(Regulatory_Domain_US_433_WIDE) || \
        defined(Regulatory_Domain_TH_920) || \
        defined(UNIT_TEST))
#error "Regulatory_Domain is not defined for 900MHz device. Check user_defines.txt!"
#endif
#else
#error "Either RADIO_SX127X, RADIO_SX128X, RADIO_LR1121 or RADIO_LR2021 must be defined!"
#endif

#if defined(PLATFORM_ESP32)
#include <soc/uart_pins.h>
#elif defined(PLATFORM_RP2)
#define U0RXD_GPIO_NUM (1)
#define U0TXD_GPIO_NUM (0)
#endif
#if !defined(U0RXD_GPIO_NUM)
#define U0RXD_GPIO_NUM (3)
#endif
#if !defined(U0TXD_GPIO_NUM)
#define U0TXD_GPIO_NUM (1)
#endif

#include "hardware.h"
