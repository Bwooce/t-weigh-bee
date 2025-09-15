/*
 * T-Weigh LoRaWAN Sensor Node - RadioLib Implementation
 * Version: 1.0.0
 *
 * UPLINK DATA FORMAT (Port 1, 12 bytes):
 * ----------------------------------------
 * Bytes 0-2:  Channel 0 raw ADC value (int24, big-endian, signed)
 * Bytes 3-5:  Channel 1 raw ADC value (int24, big-endian, signed)
 * Bytes 6-8:  Channel 2 raw ADC value (int24, big-endian, signed)
 * Bytes 9-11: Channel 3 raw ADC value (int24, big-endian, signed)
 *
 * Raw ADC range: -8,388,608 to 8,388,607 (24-bit signed)
 * Note: Application layer should handle tare/calibration
 *
 * CONFIG STATUS UPLINK (Port 2, 12 bytes):
 * -----------------------------------------
 * Byte 0:     Config version (0x01)
 * Bytes 1-2:  TX interval (seconds, big-endian)
 * Bytes 3-4:  Stabilization time (ms, big-endian)
 * Byte 5:     LoRa plan (0=AU915, 1=US915, 2=EU868, 3=AS923)
 * Byte 6:     Sub-band (0-8, used for US915/AU915)
 * Byte 7:     Flags byte:
 *             - Bit 0: Dwell time enforcement (0=off, 1=on)
 *             - Bit 1: HX711 power control (0=off, 1=on)
 *             - Bit 2: Debug mode (0=off, 1=on)
 *             - Bits 3-7: Reserved
 * Bytes 8-11: Firmware version (4 ASCII chars, e.g. "1.0.0")
 *
 * Config uplinks sent: After join + every 12 hours
 *
 * DOWNLINK COMMANDS (Port 1):
 * ----------------------------
 * 0x20 + 2 bytes: Set TX interval (seconds, big-endian, 10-65535)
 * 0x21 + 2 bytes: Set stabilization time (ms, big-endian, 100-10000)
 * 0x22 + 1 byte:  Set LoRa plan (0=AU915, 1=US915, 2=EU868, 3=AS923)
 * 0x23 + 1 byte:  Set sub-band (0-8, for US915/AU915 only)
 * 0x24 + 1 byte:  Set dwell time enforcement (0=off, 1=on)
 * 0x25 + 1 byte:  Set HX711 power control (0=off, 1=on)
 * 0x26 + 1 byte:  Set debug mode (0=off, 1=on)
 * 0x30:           Request config uplink immediately
 * 0xFF:           Reset device
 *
 * HARDWARE REQUIREMENTS:
 * ----------------------
 * - Board: LilyGO T-Weigh with T-Micro32 (ESP32-PICO-D4)
 * - LoRa: SX1262 transceiver module
 * - Load Cells: 4x channels via HX711 24-bit ADC with CD4051 multiplexer
 * - USB: Requires special USB-TTL adapter from LilyGO (NOT standard USB-C data!)
 *
 * POWER OPTIMIZATION:
 * -------------------
 * - Deep sleep between transmissions (default 60s)
 * - HX711 power-down during sleep (configurable)
 * - 2-second stabilization after wake (configurable)
 * - Serial port disabled when debug=0 (saves ~20mA)
 */

// Enable RadioLib debug output to see what's being received
#define RADIOLIB_DEBUG_PORT Serial
#define RADIOLIB_DEBUG_PROTOCOL 1
#define RADIOLIB_DEBUG_BASIC 1

#include <RadioLib.h>
#include <Preferences.h>
#include <HX711.h>
#include "driver/rtc_io.h"
#include "esp_sleep.h"

// ================================
// Configuration
// ================================

// Debug mode
#define DEBUG 1

#if DEBUG
  #define DEBUG_PRINT(...) Serial.print(__VA_ARGS__)
  #define DEBUG_PRINTLN(...) Serial.println(__VA_ARGS__)
  #define DEBUG_PRINTF(...) Serial.printf(__VA_ARGS__)
#else
  #define DEBUG_PRINT(...)
  #define DEBUG_PRINTLN(...)
  #define DEBUG_PRINTF(...)
#endif

// LoRaWAN Configuration
#include "lorawan_credentials.h"

// Board Configuration - T-Weigh with SX1262
#define LORA_CS_PIN    5
#define LORA_DIO1_PIN  15
#define LORA_RST_PIN   4
#define LORA_BUSY_PIN  2

// SPI Configuration (using default ESP32 SPI pins)
#define SPI_SCK        18
#define SPI_MISO       19
#define SPI_MOSI       23

// Load Cell Configuration
#define LOADCELL_DOUT_PIN  21
#define LOADCELL_SCK_PIN   22

// Channel Multiplexer Control Pins
#define CDA_PIN  27
#define CDB_PIN  14
#define CDC_PIN  26
#define CDD_PIN  25

// Timing Configuration
#define TX_INTERVAL_MS      60000   // Send every 60 seconds
#define CALIBRATION_SAMPLES 10       // Number of samples for calibration
#define CHANNEL_SETTLE_MS   100      // Time to wait after switching channels
#define WAKE_STABILIZE_MS   2000     // Time to wait after wake for HX711 to stabilize (2s recommended)
#define CONFIG_UPLINK_INTERVAL_MIN 720  // Send config every 12 hours (720 minutes)

// LoRaWAN Configuration  
#define LORAWAN_FPORT       1        // Application port
#define LORAWAN_ADR         true     // Adaptive Data Rate

// Downlink Commands
#define CMD_SET_INTERVAL    0x20     // Set TX interval (followed by 2 bytes, seconds)
#define CMD_SET_STABILIZE   0x21     // Set stabilization time (followed by 2 bytes, milliseconds)
#define CMD_SET_LORA_PLAN   0x22     // Set LoRa plan (followed by 1 byte: 0=AU915, 1=US915, 2=EU868, 3=AS923)
#define CMD_SET_SUBBAND     0x23     // Set sub-band (followed by 1 byte: 0-8)
#define CMD_SET_DWELL       0x24     // Set dwell time enforcement (followed by 1 byte: 0=off, 1=on)
#define CMD_SET_HX711_PWR   0x25     // Set HX711 power control (followed by 1 byte: 0=off, 1=on)
#define CMD_SET_DEBUG       0x26     // Set debug mode (followed by 1 byte: 0=off, 1=on)
#define CMD_REQUEST_CONFIG  0x30     // Request config uplink
#define CMD_RESET_DEVICE    0xFF     // Reset device

// LoRa Frequency Plans - using RadioLib's enum values
#define LORA_PLAN_EU868     BandEU868  // Europe 868MHz (0)
#define LORA_PLAN_US915     BandUS915  // US 915MHz (1)
#define LORA_PLAN_EU433     BandEU433  // Europe 433MHz (2)
#define LORA_PLAN_AU915     BandAU915  // Australia 915MHz (3)
#define LORA_PLAN_CN470     BandCN470  // China 470MHz (4)
#define LORA_PLAN_AS923     BandAS923  // Asia 923MHz (5)

// ================================
// Global Objects
// ================================

// RadioLib objects
SX1262 radio = new Module(LORA_CS_PIN, LORA_DIO1_PIN, LORA_RST_PIN, LORA_BUSY_PIN);
// LoRaWAN node - configured dynamically in setup
LoRaWANNode* node = nullptr;

// Load cell and storage
HX711 scale;
Preferences preferences;

// Channel multiplexer control values
const uint8_t channelSelect[4] = {0x0F, 0x0B, 0x09, 0x0D};

// ================================
// RTC Memory (survives deep sleep)
// ================================

RTC_DATA_ATTR uint32_t bootCount = 0;
RTC_DATA_ATTR bool sessionSaved = false;  // True if we have a saved session in RTC
RTC_DATA_ATTR uint8_t sessionBuffer[RADIOLIB_LORAWAN_SESSION_BUF_SIZE];  // Session state
RTC_DATA_ATTR uint8_t noncesBuffer[RADIOLIB_LORAWAN_NONCES_BUF_SIZE];    // Nonces
RTC_DATA_ATTR uint32_t transmitCount = 0;  // Count transmissions for periodic nonce saves
RTC_DATA_ATTR uint32_t lastConfigUplink = 0;  // Last time config was sent (minutes since boot)

// ================================
// Global Variables
// ================================

uint32_t txInterval = TX_INTERVAL_MS;
bool joinedNetwork = false;
uint8_t loraPlan = LORA_PLAN_AU915;  // Default to AU915
uint8_t loraSubBand = 2;             // Default to sub-band 2 for TTN
uint16_t wakeStabilizeMs = WAKE_STABILIZE_MS;  // HX711 stabilization time
bool enforceDwellTime = true;        // Default to enforce 400ms dwell time for AU915
bool hx711PowerControl = true;       // Default to power down HX711 during sleep
bool debugMode = DEBUG;               // Runtime debug mode
const char* firmwareVersion = "1.0.0";

// ================================
// Function Declarations
// ================================

void selectChannel(uint8_t channel);
long readLoadCellRaw(uint8_t channel);
void loadPreferences();
void savePreferences();
void processDownlink(uint8_t* data, size_t len);
void processSerialCommand();
void sendLoRaWANData();
void sendConfigUplink();
void enterDeepSleep();
void printHelp();

// ================================
// Channel Multiplexer Control
// ================================

void selectChannel(uint8_t channel) {
    if (channel > 3) return;
    
    pinMode(CDA_PIN, OUTPUT);
    pinMode(CDB_PIN, OUTPUT);
    pinMode(CDC_PIN, OUTPUT);
    pinMode(CDD_PIN, OUTPUT);
    
    digitalWrite(CDA_PIN, (channelSelect[channel] >> 3) & 0x01);
    digitalWrite(CDB_PIN, (channelSelect[channel] >> 2) & 0x01);
    digitalWrite(CDC_PIN, (channelSelect[channel] >> 1) & 0x01);
    digitalWrite(CDD_PIN, channelSelect[channel] & 0x01);
    
    delay(CHANNEL_SETTLE_MS);
}

// ================================
// Load Cell Functions
// ================================

long readLoadCellRaw(uint8_t channel) {
    selectChannel(channel);

    // Take multiple readings and average - always try to get data
    // Increased retries and timeout for better reliability
    long sum = 0;
    int validReadings = 0;
    const int MAX_ATTEMPTS = 5;
    const int TIMEOUT_MS = 200;

    for (int i = 0; i < MAX_ATTEMPTS; i++) {
        if (scale.wait_ready_timeout(TIMEOUT_MS)) {
            sum += scale.read();
            validReadings++;
        } else {
            // Small delay before retry
            delay(10);
        }
    }

    long rawValue = 0;

    if (validReadings > 0) {
        rawValue = sum / validReadings;
        DEBUG_PRINTF("[HX711] Ch%d: Raw ADC value=%ld (readings: %d/%d)\n",
                     channel, rawValue, validReadings, MAX_ATTEMPTS);
    } else {
        // No valid readings after 5 attempts - likely disconnected
        DEBUG_PRINTF("[HX711] Ch%d: No readings after %d attempts, sending 0\n", channel, MAX_ATTEMPTS);
    }

    return rawValue;
}


// ================================
// Preferences Management
// ================================

void loadPreferences() {
    preferences.begin("t-weigh", false);

    // Load TX interval
    txInterval = preferences.getUInt("interval", TX_INTERVAL_MS);

    // Load LoRa configuration
    loraPlan = preferences.getUChar("loraPlan", LORA_PLAN_AU915);
    loraSubBand = preferences.getUChar("subBand", 2);

    // Load HX711 stabilization time
    wakeStabilizeMs = preferences.getUShort("stabilizeMs", WAKE_STABILIZE_MS);

    // Load dwell time enforcement setting
    enforceDwellTime = preferences.getBool("dwellTime", true);

    // Load HX711 power control setting
    hx711PowerControl = preferences.getBool("hx711Power", true);

    // Load debug mode setting
    debugMode = preferences.getBool("debugMode", DEBUG);

    preferences.end();

    DEBUG_PRINTLN("[PREFS] Loaded preferences:");
    DEBUG_PRINTF("  TX Interval: %lu ms\n", txInterval);
    DEBUG_PRINTF("  Stabilization: %u ms\n", wakeStabilizeMs);
    DEBUG_PRINTF("  LoRa Plan: %d\n", loraPlan);
    DEBUG_PRINTF("  Sub-band: %d\n", loraSubBand);
    DEBUG_PRINTF("  Dwell time: %s\n", enforceDwellTime ? "enforced" : "disabled");
    DEBUG_PRINTF("  HX711 Power: %s\n", hx711PowerControl ? "enabled" : "disabled");
    DEBUG_PRINTF("  Debug Mode: %s\n", debugMode ? "enabled" : "disabled");
}

void savePreferences() {
    preferences.begin("t-weigh", false);

    // Save TX interval
    preferences.putUInt("interval", txInterval);

    // Save LoRa configuration
    preferences.putUChar("loraPlan", loraPlan);
    preferences.putUChar("subBand", loraSubBand);

    // Save HX711 stabilization time
    preferences.putUShort("stabilizeMs", wakeStabilizeMs);

    // Save dwell time enforcement setting
    preferences.putBool("dwellTime", enforceDwellTime);

    // Save HX711 power control setting
    preferences.putBool("hx711Power", hx711PowerControl);

    // Save debug mode setting
    preferences.putBool("debugMode", debugMode);

    preferences.end();

    DEBUG_PRINTLN("[PREFS] Preferences saved");
}

void saveNoncesToNVS() {
    // Save nonces to NVS for power-loss recovery
    preferences.begin("lorawan", false);
    preferences.putBytes("nonces", noncesBuffer, RADIOLIB_LORAWAN_NONCES_BUF_SIZE);
    preferences.end();
    DEBUG_PRINTLN("[NVS] LoRaWAN nonces saved to flash");
}

void loadNoncesFromNVS() {
    // Load nonces from NVS if available
    preferences.begin("lorawan", false);
    size_t len = preferences.getBytes("nonces", noncesBuffer, RADIOLIB_LORAWAN_NONCES_BUF_SIZE);
    preferences.end();
    if (len == RADIOLIB_LORAWAN_NONCES_BUF_SIZE) {
        DEBUG_PRINTLN("[NVS] LoRaWAN nonces loaded from flash");
        // Debug: Show what was loaded
        uint16_t loadedDevNonce = (noncesBuffer[0] << 8) | noncesBuffer[1];
        DEBUG_PRINTF("[NVS] Loaded DevNonce from NVS: %u\n", loadedDevNonce);
        DEBUG_PRINT("[NVS] Nonce buffer (first 8 bytes): ");
        for (int i = 0; i < 8 && i < RADIOLIB_LORAWAN_NONCES_BUF_SIZE; i++) {
            DEBUG_PRINTF("%02X ", noncesBuffer[i]);
        }
        DEBUG_PRINTLN();
    } else {
        DEBUG_PRINTF("[NVS] No saved nonces found (got %d bytes, expected %d)\n", len, RADIOLIB_LORAWAN_NONCES_BUF_SIZE);
    }
}

// ================================
// LoRaWAN Functions
// ================================

void sendLoRaWANData() {
    // Prepare payload (3 bytes per channel for 24-bit raw ADC values)
    // HX711 provides 24-bit signed values (-8,388,608 to 8,388,607)
    // Reducing from 16 to 12 bytes helps fit within 400ms dwell time at DR2
    uint8_t payload[12];
    uint8_t payloadSize = 0;

    for (int i = 0; i < 4; i++) {
        long rawValue = readLoadCellRaw(i);
        DEBUG_PRINTF("[HX711] Ch%d: Raw ADC value=%ld\n", i, rawValue);

        // Pack as big-endian 24-bit signed integer (3 bytes)
        // Sign extension handled by casting to 24-bit before packing
        payload[payloadSize++] = (rawValue >> 16) & 0xFF;
        payload[payloadSize++] = (rawValue >> 8) & 0xFF;
        payload[payloadSize++] = rawValue & 0xFF;
    }

    // Ensure all debug output is sent before continuing
    if (debugMode) {
        Serial.flush();
    }
    
    // Send uplink with request for downlink
    DEBUG_PRINTLN("[LoRa] Sending uplink...");

    uint8_t downlinkPayload[256];
    size_t downlinkSize = 0;

    int16_t state = node->sendReceive(
        payload, payloadSize,
        LORAWAN_FPORT,
        downlinkPayload, &downlinkSize
    );

    // For sendReceive, positive values mean success with downlink in that window
    // 0 means success without downlink, negative values are errors
    if (state >= RADIOLIB_ERR_NONE) {
        if (state > 0) {
            DEBUG_PRINTF("[LoRa] Uplink sent successfully, downlink received in window %d\n", state);
        } else {
            DEBUG_PRINTLN("[LoRa] Uplink sent successfully, no downlink");
        }
        transmitCount++;

        // Save nonces to NVS every 100 transmissions to minimize flash wear
        if (transmitCount % 100 == 0) {
            uint8_t* nonces = node->getBufferNonces();
            memcpy(noncesBuffer, nonces, RADIOLIB_LORAWAN_NONCES_BUF_SIZE);
            saveNoncesToNVS();
        }

        // Send config uplink after join or every 12 hours
        // Removed redundant state check - we're already inside RADIOLIB_ERR_NONE
        uint32_t minutesSinceBoot = (bootCount * txInterval) / 60000;
        if (lastConfigUplink == 0 || (minutesSinceBoot - lastConfigUplink) >= CONFIG_UPLINK_INTERVAL_MIN) {
            DEBUG_PRINTLN("[Config] Sending periodic configuration uplink...");
            sendConfigUplink();
            lastConfigUplink = minutesSinceBoot;
        }

        // Check for downlink
        if (downlinkSize > 0) {
            DEBUG_PRINTF("[LoRa] Received downlink (%d bytes)\n", downlinkSize);
            processDownlink(downlinkPayload, downlinkSize);
        }
    } else {
        DEBUG_PRINTF("[LoRa] Send failed, code %d\n", state);

        // If network not joined, clear session and rejoin on next boot
        if (state == -1101) {  // RADIOLIB_ERR_NETWORK_NOT_JOINED
            DEBUG_PRINTLN("[LoRa] Network not joined - will rejoin on next boot");
            joinedNetwork = false;
            sessionSaved = false;
        }
    }
}

void processDownlink(uint8_t* data, size_t len) {
    if (len < 1) return;
    
    uint8_t command = data[0];

    switch (command) {
        case CMD_SET_INTERVAL:
            if (len >= 3) {
                txInterval = (data[1] << 8) | data[2];
                txInterval *= 1000;  // Convert seconds to milliseconds
                DEBUG_PRINTF("[CMD] Set TX interval to %lu ms\n", txInterval);
                savePreferences();
            }
            break;

        case CMD_SET_STABILIZE:
            if (len >= 3) {
                wakeStabilizeMs = (data[1] << 8) | data[2];
                DEBUG_PRINTF("[CMD] Set stabilization time to %u ms\n", wakeStabilizeMs);
                savePreferences();
            }
            break;

        case CMD_SET_LORA_PLAN:
            if (len >= 2) {
                uint8_t plan = data[1];
                if (plan <= 3) {
                    loraPlan = plan;
                    DEBUG_PRINTF("[CMD] Set LoRa plan to %d\n", loraPlan);
                    savePreferences();
                    // Note: Requires restart to take effect
                }
            }
            break;

        case CMD_SET_SUBBAND:
            if (len >= 2) {
                uint8_t sb = data[1];
                if (sb <= 8) {
                    loraSubBand = sb;
                    DEBUG_PRINTF("[CMD] Set sub-band to %d\n", loraSubBand);
                    savePreferences();
                    // Note: Requires restart to take effect
                }
            }
            break;

        case CMD_SET_DWELL:
            if (len >= 2) {
                enforceDwellTime = (data[1] != 0);
                DEBUG_PRINTF("[CMD] Dwell time enforcement: %s\n", enforceDwellTime ? "enabled" : "disabled");
                savePreferences();
                // Note: Requires restart to take effect
            }
            break;

        case CMD_SET_HX711_PWR:
            if (len >= 2) {
                hx711PowerControl = (data[1] != 0);
                DEBUG_PRINTF("[CMD] HX711 power control: %s\n", hx711PowerControl ? "enabled" : "disabled");
                savePreferences();
            }
            break;

        case CMD_SET_DEBUG:
            if (len >= 2) {
                bool newDebugMode = (data[1] != 0);
                if (newDebugMode && !debugMode) {
                    // Turning debug ON - initialize serial if not already done
                    if (!Serial) {
                        Serial.begin(115200);
                        delay(100);  // Small delay for serial init
                    }
                    debugMode = true;
                    DEBUG_PRINTLN("[CMD] Debug mode enabled - serial output activated");
                } else if (!newDebugMode && debugMode) {
                    // Turning debug OFF
                    DEBUG_PRINTLN("[CMD] Debug mode disabled - serial output will stop");
                    debugMode = false;
                    Serial.flush();
                }
                savePreferences();
            }
            break;

        case CMD_REQUEST_CONFIG:
            DEBUG_PRINTLN("[CMD] Config uplink requested");
            sendConfigUplink();
            break;

        case CMD_RESET_DEVICE:
            DEBUG_PRINTLN("[CMD] Reset device");
            ESP.restart();
            break;
            
        default:
            DEBUG_PRINTF("[CMD] Unknown command: 0x%02X\n", command);
            break;
    }
}

// ================================
// Serial Command Interface
// ================================

void processSerialCommand() {
    if (!Serial.available()) return;
    
    String command = Serial.readStringUntil('\n');
    command.trim();
    
    if (command.length() == 0) return;
    
    DEBUG_PRINTF("[SERIAL] Command: %s\n", command.c_str());
    
    if (command == "help" || command == "?") {
        printHelp();
    }
    else if (command == "read") {
        for (int i = 0; i < 4; i++) {
            long rawValue = readLoadCellRaw(i);
            Serial.printf("Channel %d: %ld (raw ADC)\n", i, rawValue);
        }
    }
    else if (command == "status") {
        Serial.println("\n=== T-Weigh Status ===");
        Serial.printf("Boot count: %lu\n", bootCount);
        Serial.printf("Network joined: %s\n", joinedNetwork ? "Yes" : "No");
        Serial.printf("TX interval: %lu ms\n", txInterval);
        Serial.printf("Transmissions: %lu\n", transmitCount);
        const char* planNames[] = {"AU915", "US915", "EU868", "AS923"};
        Serial.printf("LoRa Plan: %s\n", planNames[loraPlan]);
        Serial.printf("Sub-band: %d\n", loraSubBand);
        Serial.printf("Dwell time: %s\n", enforceDwellTime ? "Enforced" : "Disabled");
    }
    else if (command.startsWith("plan ")) {
        int plan = command.substring(5).toInt();
        if (plan >= 0 && plan <= 3) {
            loraPlan = plan;
            savePreferences();
            Serial.println("LoRa plan updated - restart to apply");
        } else {
            Serial.println("Invalid plan (0=AU915, 1=US915, 2=EU868, 3=AS923)");
        }
    }
    else if (command.startsWith("subband ")) {
        int sb = command.substring(8).toInt();
        if (sb >= 0 && sb <= 8) {
            loraSubBand = sb;
            savePreferences();
            Serial.println("Sub-band updated - restart to apply");
        } else {
            Serial.println("Invalid sub-band (0-8)");
        }
    }
    else if (command == "dwell on") {
        enforceDwellTime = true;
        savePreferences();
        Serial.println("Dwell time enforcement enabled - restart to apply");
    }
    else if (command == "dwell off") {
        enforceDwellTime = false;
        savePreferences();
        Serial.println("Dwell time enforcement disabled - restart to apply");
        Serial.println("WARNING: Ensure local regulations permit this!");
    }
    else if (command == "reset") {
        DEBUG_PRINTLN("Resetting device...");
        ESP.restart();
    }
    else if (command == "send") {
        if (joinedNetwork) {
            sendLoRaWANData();
        } else {
            DEBUG_PRINTLN("Not joined to network");
        }
    }
    else if (command == "save") {
        // Manual command to save nonces (for testing)
        if (joinedNetwork) {
            uint8_t* nonces = node->getBufferNonces();
            memcpy(noncesBuffer, nonces, RADIOLIB_LORAWAN_NONCES_BUF_SIZE);
            saveNoncesToNVS();
            DEBUG_PRINTLN("Nonces saved to NVS");
        } else {
            DEBUG_PRINTLN("No active session to save");
        }
    }
    else {
        DEBUG_PRINTLN("Unknown command. Type 'help' for available commands.");
    }
}

void printHelp() {
    Serial.println("\n=== T-Weigh Commands ===");
    Serial.println("help        - Show this help");
    Serial.println("read        - Read all channels");
    Serial.println("status      - Show device status");
    Serial.println("send        - Send data immediately");
    Serial.println("save        - Save LoRaWAN nonces to flash");
    Serial.println("plan [0-5]  - Set LoRa plan (0=EU868, 1=US915, 2=EU433, 3=AU915, 4=CN470, 5=AS923)");
    Serial.println("subband [n] - Set sub-band (0-8)");
    Serial.println("reset       - Reset device");
    Serial.println("========================\n");
}

// ================================
// Configuration Uplink
// ================================

void sendConfigUplink() {
    if (!joinedNetwork) {
        DEBUG_PRINTLN("[Config] Not joined, skipping config uplink");
        return;
    }

    DEBUG_PRINTLN("[Config] Sending configuration uplink...");

    uint8_t configPayload[12];
    configPayload[0] = 0x01;  // Config version

    // TX interval (seconds)
    uint16_t txIntervalSec = txInterval / 1000;
    configPayload[1] = (txIntervalSec >> 8) & 0xFF;
    configPayload[2] = txIntervalSec & 0xFF;

    // Stabilization time (ms)
    configPayload[3] = (wakeStabilizeMs >> 8) & 0xFF;
    configPayload[4] = wakeStabilizeMs & 0xFF;

    // LoRa plan and sub-band
    configPayload[5] = loraPlan;
    configPayload[6] = loraSubBand;

    // Flags
    configPayload[7] = 0;
    if (enforceDwellTime) configPayload[7] |= 0x01;
    if (hx711PowerControl) configPayload[7] |= 0x02;
    if (debugMode) configPayload[7] |= 0x04;

    // Firmware version (4 bytes ASCII)
    memcpy(&configPayload[8], firmwareVersion, 4);

    // Send on port 2
    uint8_t downlinkPayload[256];
    size_t downlinkSize = 0;

    int16_t state = node->sendReceive(
        configPayload, sizeof(configPayload),
        2,  // Port 2 for config
        downlinkPayload, &downlinkSize
    );

    // For sendReceive, positive values mean success with downlink in that window
    // 0 means success without downlink, negative values are errors
    if (state >= RADIOLIB_ERR_NONE) {
        if (state > 0) {
            DEBUG_PRINTF("[Config] Configuration uplink sent successfully, downlink received in window %d\n", state);
        } else {
            DEBUG_PRINTLN("[Config] Configuration uplink sent successfully, no downlink");
        }

        // Check for downlink in response to config
        if (downlinkSize > 0) {
            DEBUG_PRINTF("[Config] Received downlink (%d bytes) in response to config\n", downlinkSize);
            processDownlink(downlinkPayload, downlinkSize);
        }
    } else {
        DEBUG_PRINTF("[Config] Send failed, code %d\n", state);
    }
}

// ================================
// Deep Sleep Management
// ================================

void enterDeepSleep() {
    // Save LoRaWAN state to RTC memory (nonces first, then session)
    uint8_t* nonces = node->getBufferNonces();
    memcpy(noncesBuffer, nonces, RADIOLIB_LORAWAN_NONCES_BUF_SIZE);

    uint8_t* session = node->getBufferSession();
    memcpy(sessionBuffer, session, RADIOLIB_LORAWAN_SESSION_BUF_SIZE);

    sessionSaved = true;

    // Power down HX711 to save power during sleep (if enabled)
    if (hx711PowerControl) {
        scale.power_down();
    }

    DEBUG_PRINTF("[SLEEP] Entering deep sleep for %lu ms\n", txInterval);
    DEBUG_PRINTLN("[SLEEP] Session saved to RTC memory");

    // Flush serial before sleep to ensure all output is sent
    if (debugMode) {
        Serial.flush();
        delay(10);  // Small delay to ensure UART completes
    }
    
    // Configure wake-up timer
    esp_sleep_enable_timer_wakeup(txInterval * 1000ULL);
    
    // Enter deep sleep
    esp_deep_sleep_start();
}

// ================================
// Setup Function
// ================================

void setup() {
    // Increment boot counter
    bootCount++;

    // Initialize serial (only if debug mode is enabled)
    if (debugMode) {
        Serial.begin(115200);
        delay(2000);  // Give time to open serial monitor

        Serial.println("\n=====================================");
        Serial.println("T-Weigh LoRaWAN Sensor - RadioLib");
        Serial.println("=====================================");
        Serial.printf("Boot count: %lu\n", bootCount);
        Serial.printf("Wake reason: %d\n", esp_sleep_get_wakeup_cause());
    }

    // Check for button press early to clear session if needed
    pinMode(0, INPUT_PULLUP);
    delay(50);  // Debounce
    if (digitalRead(0) == LOW) {
        DEBUG_PRINTLN("[TEST] Button pressed - clearing session to force rejoin...");
        sessionSaved = false;
        joinedNetwork = false;
        // Don't clear the nonces though - we want to keep incrementing!
    }
    
    // Initialize HX711
    DEBUG_PRINTLN("[INIT] Initializing HX711...");
    scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
    scale.set_gain(128);  // Set gain for ±20mV range

    // Power up HX711 if coming from deep sleep (if power control enabled)
    if (esp_sleep_get_wakeup_cause() != ESP_SLEEP_WAKEUP_UNDEFINED) {
        if (hx711PowerControl) {
            scale.power_up();
        }
        DEBUG_PRINTF("[INIT] Waiting %d ms for HX711 to stabilize after wake...\n", wakeStabilizeMs);
        delay(wakeStabilizeMs);
    }
    
    // Load preferences
    loadPreferences();

    // ALWAYS load nonces from NVS to preserve DevNonce across all reboots
    // This is critical for preventing DevNonce reuse
    loadNoncesFromNVS();
    
    // Initialize SPI
    SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI, LORA_CS_PIN);
    
    // Initialize radio for AU915 frequencies
    DEBUG_PRINTLN("[INIT] Initializing SX1262...");

    // Begin with AU915 frequency
    float freq = 915.2;  // AU915 base frequency
    int16_t state = radio.begin(freq);

    if (state != RADIOLIB_ERR_NONE) {
        DEBUG_PRINTF("[INIT] Radio init failed: %d\n", state);
        enterDeepSleep();
        return;
    }

    // IMPORTANT: For SX1262, configure DIO2 as RF switch
    // This is required for the T-Weigh board's SX1262 module
    radio.setDio2AsRfSwitch();

    // Set the SX1262 to use TCXO for better frequency stability
    // The T-Weigh board uses a 32MHz TCXO
    radio.setTCXO(2.4);

    DEBUG_PRINTLN("[INIT] Radio initialized");
    
    // Initialize LoRaWAN
    DEBUG_PRINTLN("[LoRa] Initializing LoRaWAN...");

    // Create LoRaWAN node dynamically based on plan and sub-band
    if (loraPlan == LORA_PLAN_AU915) {
        node = new LoRaWANNode(&radio, &AU915, loraSubBand);
    } else if (loraPlan == LORA_PLAN_US915) {
        node = new LoRaWANNode(&radio, &US915, loraSubBand);
    } else if (loraPlan == LORA_PLAN_EU868) {
        node = new LoRaWANNode(&radio, &EU868);
    } else {
        // Default to AU915
        node = new LoRaWANNode(&radio, &AU915, loraSubBand);
    }
    
    // Prepare credentials from header file
    uint64_t joinEUI = 0x0000000000000000;  // All zeros for TTN

    // Convert DevEUI from byte array to uint64_t
    // The DEVEUI array is in LSB format: { 0x1D, 0x2B, 0x07, 0xD0, 0x7E, 0xD5, 0xB3, 0x70 }
    // We need to read it in reverse to get MSB value for RadioLib
    uint64_t devEUI = 0;
    for (int i = 0; i < 8; i++) {
        devEUI = (devEUI << 8) | pgm_read_byte(&DEVEUI[7-i]);
    }

    // Debug: Print the credentials being used
    DEBUG_PRINTF("[LoRa] DevEUI: %016llX\n", devEUI);
    DEBUG_PRINTF("[LoRa] JoinEUI: %016llX\n", joinEUI);

    uint8_t appKey[16];
    uint8_t nwkKey[16];

    // Copy AppKey from credentials
    memcpy_P(appKey, APPKEY, 16);
    memcpy_P(nwkKey, APPKEY, 16);  // For LoRaWAN 1.0.x, nwkKey = appKey

    // Debug: Print AppKey
    DEBUG_PRINT("[LoRa] AppKey: ");
    for (int i = 0; i < 16; i++) {
        DEBUG_PRINTF("%02X", appKey[i]);
    }
    DEBUG_PRINTLN();

    
    // Check if we have a saved session from previous boot
    if (sessionSaved && bootCount > 1) {
        DEBUG_PRINTLN("[LoRa] Restoring session from RTC memory...");
        
        // First initialize the node with credentials
        // For LoRaWAN 1.0.x, pass NULL for nwkKey
        int16_t state = node->beginOTAA(joinEUI, devEUI, NULL, appKey);
        if (state != RADIOLIB_ERR_NONE) {
            DEBUG_PRINTF("[LoRa] BeginOTAA failed: %d\n", state);
            sessionSaved = false;
            joinedNetwork = false;
        } else {
            // Restore nonces FIRST, then session (RadioLib requirement)
            state = node->setBufferNonces(noncesBuffer);
            if (state == RADIOLIB_ERR_NONE) {
                DEBUG_PRINTLN("[LoRa] Nonces restored from RTC");
                state = node->setBufferSession(sessionBuffer);
                if (state == RADIOLIB_ERR_NONE) {
                    DEBUG_PRINTLN("[LoRa] Session restored, activating...");

                    // Activate the restored session
                    state = node->activateOTAA();
                    if (state == RADIOLIB_LORAWAN_SESSION_RESTORED || state == RADIOLIB_ERR_NONE) {
                        DEBUG_PRINTLN("[LoRa] Session activated successfully!");
                        joinedNetwork = true;

                        // For AU915 with dwell time, use DR3 for 12-byte payload
                        if (enforceDwellTime && loraPlan == LORA_PLAN_AU915) {
                            DEBUG_PRINTLN("[LoRa] Setting DR3 for AU915 dwell time compliance with 12-byte payload");
                            node->setDatarate(3);  // DR3 = SF9BW125
                        }
                    } else {
                        DEBUG_PRINTF("[LoRa] Failed to activate session: %d\n", state);
                        sessionSaved = false;
                        joinedNetwork = false;
                    }
                } else {
                    DEBUG_PRINTF("[LoRa] Failed to restore session: %d\n", state);
                    sessionSaved = false;
                    joinedNetwork = false;
                }
            } else {
                DEBUG_PRINTF("[LoRa] Failed to restore nonces: %d\n", state);
                sessionSaved = false;
                joinedNetwork = false;
            }
        }
    }
    
    // If no valid session, perform OTAA join
    if (!joinedNetwork) {
        DEBUG_PRINTLN("[LoRa] Starting OTAA join...");

        // Begin OTAA first to initialize the node
        // For LoRaWAN 1.0.x, pass NULL for nwkKey to avoid RadioLib thinking it's 1.1
        int16_t state = node->beginOTAA(joinEUI, devEUI, NULL, appKey);

        if (state != RADIOLIB_ERR_NONE) {
            DEBUG_PRINTF("[LoRa] BeginOTAA failed: %d\n", state);
            enterDeepSleep();
            return;
        }

        // CRITICAL: Restore saved nonces from NVS
        // This MUST happen after beginOTAA but before activateOTAA
        // Check if we have valid nonces saved (not all zeros)
        bool hasValidNonces = false;
        for (int i = 0; i < RADIOLIB_LORAWAN_NONCES_BUF_SIZE; i++) {
            if (noncesBuffer[i] != 0) {
                hasValidNonces = true;
                break;
            }
        }

        if (hasValidNonces) {
            DEBUG_PRINTLN("[LoRa] Restoring nonces from NVS...");
            state = node->setBufferNonces(noncesBuffer);
            if (state == RADIOLIB_ERR_NONE) {
                // Debug: Print restored DevNonce
                uint8_t* nonces = node->getBufferNonces();
                uint16_t restoredDevNonce = (nonces[0] << 8) | nonces[1];
                DEBUG_PRINTF("[LoRa] Successfully restored DevNonce: %u\n", restoredDevNonce);
            } else {
                DEBUG_PRINTF("[LoRa] Failed to restore nonces: %d\n", state);
            }
        } else {
            DEBUG_PRINTLN("[LoRa] No saved nonces found, starting fresh");
        }

        // Debug: Print current DevNonce before join attempt
        uint8_t* nonces = node->getBufferNonces();
        uint16_t currentDevNonce = (nonces[0] << 8) | nonces[1];
        DEBUG_PRINTF("[LoRa] DevNonce before join attempt: %u\n", currentDevNonce);

        // Configure for AU915 FSB2 (The Things Network)
        // For AU915, we need to select the correct sub-band
        // FSB2 uses channels 8-15 (125kHz) + 65 (500kHz)

        // Set ADR
        node->setADR(LORAWAN_ADR);

        // Configure dwell time enforcement BEFORE join
        if (!enforceDwellTime) {
            DEBUG_PRINTLN("[LoRa] Disabling dwell time enforcement");
            node->setDwellTime(false);
        }

        // Perform join with retry logic
        const int MAX_JOIN_ATTEMPTS = 3;
        for (int attempt = 1; attempt <= MAX_JOIN_ATTEMPTS; attempt++) {
            DEBUG_PRINTF("[LoRa] Join attempt %d of %d...\n", attempt, MAX_JOIN_ATTEMPTS);
            state = node->activateOTAA();

            if (state == RADIOLIB_LORAWAN_NEW_SESSION) {
                DEBUG_PRINTLN("[LoRa] Join successful!");
                joinedNetwork = true;

                // For AU915 with dwell time, use DR3 for 12-byte payload
                if (enforceDwellTime && loraPlan == LORA_PLAN_AU915) {
                    DEBUG_PRINTLN("[LoRa] Setting DR3 for AU915 dwell time compliance with 12-byte payload");
                    node->setDatarate(3);  // DR3 = SF9BW125
                }

                // Get nonces first, then session (order matters for RadioLib)
                uint8_t* nonces = node->getBufferNonces();
                memcpy(noncesBuffer, nonces, RADIOLIB_LORAWAN_NONCES_BUF_SIZE);

                uint8_t* session = node->getBufferSession();
                memcpy(sessionBuffer, session, RADIOLIB_LORAWAN_SESSION_BUF_SIZE);

                // Save nonces to NVS for persistence across power loss
                saveNoncesToNVS();

                // Mark session as valid only after both are saved
                sessionSaved = true;
                DEBUG_PRINTLN("[LoRa] Session and nonces saved after join");

                // Send config uplink after successful join
                DEBUG_PRINTLN("[Config] Sending config uplink after join...");
                lastConfigUplink = 0;  // Reset to force send
                break;
            } else if (state == RADIOLIB_LORAWAN_SESSION_RESTORED) {
                DEBUG_PRINTLN("[LoRa] Session restored!");
                joinedNetwork = true;
                sessionSaved = true;
                break;
            } else if (state == RADIOLIB_ERR_NONE) {
                DEBUG_PRINTLN("[LoRa] Already joined!");
                joinedNetwork = true;
                sessionSaved = true;
                break;
            } else {
                DEBUG_PRINTF("[LoRa] Join failed with code: %d\n", state);
                if (state == -1116) {
                    DEBUG_PRINTLN("[LoRa] Error -1116: RADIOLIB_ERR_DOWNLINK_MALFORMED");
                    DEBUG_PRINTLN("[LoRa] This means join accept was received but couldn't be parsed");
                    DEBUG_PRINTLN("[LoRa] Possible causes: Wrong AppKey, LoRaWAN version mismatch");
                }

                // IMPORTANT: Save the incremented DevNonce after EACH failure
                // RadioLib has already incremented it, and we must not reuse it
                uint8_t* nonces = node->getBufferNonces();
                uint16_t newDevNonce = (nonces[0] << 8) | nonces[1];
                DEBUG_PRINTF("[LoRa] DevNonce after attempt %d: %u\n", attempt, newDevNonce);

                memcpy(noncesBuffer, nonces, RADIOLIB_LORAWAN_NONCES_BUF_SIZE);
                saveNoncesToNVS();

                if (attempt < MAX_JOIN_ATTEMPTS) {
                    DEBUG_PRINTF("[LoRa] Waiting 10 seconds before retry...\n");
                    delay(10000);  // Wait 10 seconds before retry
                } else {
                    DEBUG_PRINTLN("[LoRa] Max join attempts reached, going to sleep");
                    // Clear flags for next boot attempt
                    joinedNetwork = false;
                    sessionSaved = false;
                    enterDeepSleep();
                    return;
                }
            }
        }
    }
    
    // Check for button press on boot (GPIO0)
    pinMode(0, INPUT_PULLUP);
    if (digitalRead(0) == LOW) {
        DEBUG_PRINTLN("[INIT] Button pressed - entering interactive mode");
        
        Serial.println("\n=== Interactive Mode ===");
        Serial.println("Type 'help' for commands");
        Serial.println("Device will not sleep");
        Serial.println("========================\n");
        
        // Interactive mode - don't sleep
        while (true) {
            processSerialCommand();
            
            // Send data periodically even in interactive mode
            static unsigned long lastTx = 0;
            if (millis() - lastTx > txInterval) {
                sendLoRaWANData();
                lastTx = millis();
            }
            
            delay(10);
        }
    }
    
    // Normal operation - send data and sleep
    if (joinedNetwork) {
        // For AU915 with dwell time, use DR3 for 12-byte payload
        if (enforceDwellTime && loraPlan == LORA_PLAN_AU915) {
            DEBUG_PRINTLN("[LoRa] Ensuring DR3 for AU915 dwell time compliance with 12-byte payload");
            node->setDatarate(3);  // DR3 = SF9BW125
        }

        // Give the stack time to settle after join
        // Wait longer after a fresh join to respect duty cycle
        if (bootCount == 1 || !sessionSaved) {
            DEBUG_PRINTLN("[MAIN] Waiting 10s after join for duty cycle...");
            delay(10000);  // 10 seconds after join
        } else {
            delay(2000);   // 2 seconds for normal wake
        }

        // Send config uplink after fresh join or if it's time
        if (lastConfigUplink == 0) {
            DEBUG_PRINTLN("[Config] Sending initial config uplink after join...");
            sendConfigUplink();
            uint32_t minutesSinceBoot = (bootCount * txInterval) / 60000;
            lastConfigUplink = minutesSinceBoot;
            delay(2000);  // Small delay between config and data uplinks
        }

        DEBUG_PRINTLN("[MAIN] Sending sensor data...");
        sendLoRaWANData();
    } else {
        DEBUG_PRINTLN("[MAIN] Not joined to network, skipping data send");
    }

    // Enter deep sleep
    enterDeepSleep();
}

// ================================
// Loop Function (not used)
// ================================

void loop() {
    // Should never reach here due to deep sleep
}