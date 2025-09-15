// LMIC Project Configuration for T-Weigh LoRaWAN
#ifndef _lmic_project_config_h_
#define _lmic_project_config_h_

// Enable AU915 region
#define CFG_au915 1

// Disable other regions to save space
#undef CFG_eu868
#undef CFG_us915

// Select radio chip - SX1262
#define CFG_sx1262_radio 1
#undef CFG_sx1276_radio
#undef CFG_sx1272_radio
//#define CFG_LMIC_REGION_MASK LMIC_REGION_au915

// Disable unused features
#define DISABLE_PING
#define DISABLE_BEACONS

// Enable verbose debugging for SX1262 issues
#define LMIC_DEBUG_LEVEL 2
#define LMIC_PRINTF_TO Serial.printf

// Important for ESP32 timing
#define LMIC_ENABLE_arbitrary_clock_error 1

// ESP32 specific
#define OSTICKS_PER_SEC 32768

// Disable LMIC's built-in failure handling
#define LMIC_FAILURE_TO Serial.printf

// Disable LMIC assertions for debugging
#define DISABLE_LMIC_FAILURE_TO

// Allow clock error
#define LMIC_CLOCKERROR_PPM 10

// Use interrupts for better timing
#define LMIC_USE_INTERRUPTS

// SX1262 specific settings
#define LMIC_SX1262_TXCO_STARTUP_DELAY 5

// Allow relaxed timing for debugging
#define LMIC_ENABLE_long_messages 1

#endif