/*
 * T-Weigh LoRaWAN Sensor Node for The Things Network (AU915)
 * 
 * Reads 4 load cells via HX711 with multiplexed channels
 * Sends data via LoRaWAN every minute
 * Optimized for low power with deep sleep
 */

#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <HX711.h>
// #include <Wire.h> // Not needed - no I2C devices on this board
#include <driver/rtc_io.h>
#include "esp_sleep.h"
#include "lorawan_credentials.h"

// Pin definitions from T-Weigh hardware
#define CDA 27
#define CDB 14
#define CDC 26
#define CDD 25

// HX711 load cell pins
#define LOADCELL_DOUT_PIN 21
#define LOADCELL_SCK_PIN 22

// I2C pins (unused - no display or RTC on this board)
// #define IIC_SCL 32
// #define IIC_SDA 33

// Sleep configuration
#define TX_INTERVAL 60  // Send every 60 seconds

// HX711 calibration value (adjust based on your calibration)
#define CALIBRATION_VALUE 2208

// LoRaWAN credential callbacks - credentials defined in lorawan_credentials.h
void os_getArtEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8); }
void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8); }
void os_getDevKey (u1_t* buf) { memcpy_P(buf, APPKEY, 16); }

// Pin mapping for SX1262
const lmic_pinmap lmic_pins = {
    .nss = 5,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 4,
    .dio = {15, 13, LMIC_UNUSED_PIN},  // DIO1, DIO3
    .rxtx_rx_active = 0,
    .rssi_cal = 0,
    .spi_freq = 8000000,
};

// RTC memory for preserving data across deep sleep
RTC_DATA_ATTR long weight_tare[4] = {0, 0, 0, 0};
RTC_DATA_ATTR int boot_count = 0;
RTC_DATA_ATTR u4_t seqnoUp = 0;
RTC_DATA_ATTR u4_t seqnoDn = 0;
RTC_DATA_ATTR devaddr_t devaddr = 0;
RTC_DATA_ATTR u1_t nwkKey[16];
RTC_DATA_ATTR u1_t artKey[16];

// Load cell
HX711 scale;

// Channel multiplexer values
const char channel[4] = {0x0F, 0x0B, 0x09, 0x0D};

// Global variables
static uint8_t payload[10];
static osjob_t sendjob;
bool joined = false;
bool sleeping = false;

void SelectChannel(uint8_t ch) {
    pinMode(CDA, OUTPUT);
    pinMode(CDB, OUTPUT);
    pinMode(CDC, OUTPUT);
    pinMode(CDD, OUTPUT);
    
    digitalWrite(CDA, channel[ch] & 0x08);
    digitalWrite(CDB, channel[ch] & 0x04);
    digitalWrite(CDC, channel[ch] & 0x02);
    digitalWrite(CDD, channel[ch] & 0x01);
}

void CalibrateTare() {
    Serial.println(F("Calibrating load cells (remove all weight)..."));
    for (uint8_t i = 0; i < 4; i++) {
        SelectChannel(i);
        delay(100);
        scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
        while (!scale.is_ready()) {
            delay(10);
        }
        weight_tare[i] = scale.read();
        Serial.print(F("Channel "));
        Serial.print(i);
        Serial.print(F(" tare: "));
        Serial.println(weight_tare[i]);
    }
}

float ReadLoadCell(uint8_t channel_num) {
    SelectChannel(channel_num);
    delay(50);
    
    if (scale.is_ready()) {
        long reading = scale.read();
        reading = reading - weight_tare[channel_num];
        float weight_g = (float)reading / CALIBRATION_VALUE;
        return weight_g;
    }
    return 0.0;
}

void do_sleep() {
    Serial.println(F("Entering deep sleep..."));
    Serial.flush();
    
    // Save LMIC state
    seqnoUp = LMIC.seqnoUp;
    seqnoDn = LMIC.seqnoDn;
    devaddr = LMIC.devaddr;
    memcpy(nwkKey, LMIC.nwkKey, 16);
    memcpy(artKey, LMIC.artKey, 16);
    
    // Configure wake up timer
    esp_sleep_enable_timer_wakeup(TX_INTERVAL * 1000000ULL);
    
    // Enter deep sleep
    esp_deep_sleep_start();
}

void do_send(osjob_t* j) {
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending"));
    } else {
        // Read all load cells
        float weights[4];
        float total_weight = 0;
        
        Serial.println(F("\nReading load cells:"));
        for (uint8_t i = 0; i < 4; i++) {
            weights[i] = ReadLoadCell(i);
            total_weight += weights[i];
            Serial.print(F("Ch"));
            Serial.print(i);
            Serial.print(F(": "));
            Serial.print(weights[i] / 1000.0, 3);
            Serial.println(F(" kg"));
        }
        Serial.print(F("Total: "));
        Serial.print(total_weight / 1000.0, 3);
        Serial.println(F(" kg"));
        
        // Read battery voltage (placeholder - implement actual ADC reading if available)
        uint16_t battery_mv = 3300;
        
        // Prepare payload (compact binary format)
        int idx = 0;
        for (uint8_t i = 0; i < 4; i++) {
            int16_t weight_int = (int16_t)weights[i];
            payload[idx++] = (weight_int >> 8) & 0xFF;
            payload[idx++] = weight_int & 0xFF;
        }
        payload[idx++] = (battery_mv >> 8) & 0xFF;
        payload[idx++] = battery_mv & 0xFF;
        
        // Send data
        LMIC_setTxData2(1, payload, sizeof(payload), 0);
        Serial.println(F("Packet queued"));
    }
}

void onEvent (ev_t ev) {
    Serial.print(os_getTime());
    Serial.print(": ");
    switch(ev) {
        case EV_SCAN_TIMEOUT:
            Serial.println(F("EV_SCAN_TIMEOUT"));
            break;
        case EV_BEACON_FOUND:
            Serial.println(F("EV_BEACON_FOUND"));
            break;
        case EV_BEACON_MISSED:
            Serial.println(F("EV_BEACON_MISSED"));
            break;
        case EV_BEACON_TRACKED:
            Serial.println(F("EV_BEACON_TRACKED"));
            break;
        case EV_JOINING:
            Serial.println(F("EV_JOINING"));
            break;
        case EV_JOINED:
            Serial.println(F("EV_JOINED"));
            joined = true;
            
            // Disable link check validation
            LMIC_setLinkCheckMode(0);
            
            // Set data rate and transmit power for AU915
            LMIC_setDrTxpow(DR_SF7, 20);
            
            // Start sending
            do_send(&sendjob);
            break;
        case EV_JOIN_FAILED:
            Serial.println(F("EV_JOIN_FAILED"));
            // Try again after sleep
            do_sleep();
            break;
        case EV_REJOIN_FAILED:
            Serial.println(F("EV_REJOIN_FAILED"));
            break;
        case EV_TXCOMPLETE:
            Serial.println(F("EV_TXCOMPLETE"));
            if (LMIC.txrxFlags & TXRX_ACK)
                Serial.println(F("Received ack"));
            if (LMIC.dataLen) {
                Serial.print(F("Received "));
                Serial.print(LMIC.dataLen);
                Serial.println(F(" bytes"));
                
                // Process downlink
                if (LMIC.dataLen > 0 && LMIC.frame[LMIC.dataBeg] == 0x01) {
                    // Command 0x01 = recalibrate
                    CalibrateTare();
                }
            }
            // Schedule sleep
            sleeping = true;
            break;
        case EV_LOST_TSYNC:
            Serial.println(F("EV_LOST_TSYNC"));
            break;
        case EV_RESET:
            Serial.println(F("EV_RESET"));
            break;
        case EV_RXCOMPLETE:
            Serial.println(F("EV_RXCOMPLETE"));
            break;
        case EV_LINK_DEAD:
            Serial.println(F("EV_LINK_DEAD"));
            break;
        case EV_LINK_ALIVE:
            Serial.println(F("EV_LINK_ALIVE"));
            break;
        case EV_TXSTART:
            Serial.println(F("EV_TXSTART"));
            break;
        case EV_TXCANCELED:
            Serial.println(F("EV_TXCANCELED"));
            break;
        case EV_RXSTART:
            break;
        case EV_JOIN_TXCOMPLETE:
            Serial.println(F("EV_JOIN_TXCOMPLETE: no JoinAccept"));
            break;
        default:
            Serial.print(F("Unknown event: "));
            Serial.println((unsigned) ev);
            break;
    }
}

void setup() {
    Serial.begin(115200);
    delay(1000);
    
    boot_count++;
    Serial.println(F("\n=== T-Weigh LoRaWAN Node ==="));
    Serial.print(F("Boot count: "));
    Serial.println(boot_count);
    
    
    // Initialize load cell
    scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
    
    // Calibrate on first boot
    if (boot_count == 1) {
        CalibrateTare();
    }
    
    // Initialize LMIC
    os_init();
    LMIC_reset();
    
    // Configure for AU915
    // Disable all channels except 8-15 (used by TTN)
    for (int i = 0; i < 72; i++) {
        if (i < 8 || i > 15) {
            LMIC_disableChannel(i);
        }
    }
    
    // Set data rate and transmit power
    LMIC_setDrTxpow(DR_SF7, 20);
    
    // Set adaptive data rate
    LMIC_setAdrMode(1);
    
    // Check if we have a saved session
    if (boot_count > 1 && devaddr != 0) {
        Serial.println(F("Restoring saved session"));
        LMIC_setSession(0x13, devaddr, nwkKey, artKey);
        LMIC.seqnoUp = seqnoUp;
        LMIC.seqnoDn = seqnoDn;
        joined = true;
        
        // Start sending immediately
        do_send(&sendjob);
    } else {
        // Start joining
        Serial.println(F("Starting OTAA join"));
        LMIC_startJoining();
    }
}

void loop() {
    os_runloop_once();
    
    // Check if we should sleep
    if (sleeping) {
        delay(100);  // Small delay to ensure all operations complete
        do_sleep();
    }
}