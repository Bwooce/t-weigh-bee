/*
 * T-Weigh LoRaWAN Sensor Node - RadioLib Implementation
 * 
 * WARNING: This RadioLib implementation is UNTESTED!
 * For production use, please use the LMIC implementation in the parent directory.
 * 
 * Known Issues:
 * - LoRaWAN session may not persist across deep sleep
 * - May require OTAA join after each wake (high power consumption)
 * - AU915 frequency configuration may need manual adjustment
 */

#include <Arduino.h>
#include <RadioLib.h>
#include <HX711.h>
#include <Wire.h>
#include <driver/rtc_io.h>

// Pin definitions from T-Weigh hardware
#define CDA 27
#define CDB 14
#define CDC 26
#define CDD 25

// LoRa SX1262 pins
#define csPin 5
#define resetPin 4
#define DIO1 15
#define DIO3 13
#define NRSET 4
#define BUSY 2

// SPI pins
#define SPI_SCLK 18
#define SPI_MOSI 23
#define SPI_MISO 19

// HX711 load cell pins
#define LOADCELL_DOUT_PIN 21
#define LOADCELL_SCK_PIN 22

// I2C pins for RTC/sensors
#define IIC_SCL 32
#define IIC_SDA 33

// Sleep configuration
#define SLEEP_INTERVAL_MS 60000  // 60 seconds
#define CALIBRATION_VALUE 2208   // HX711 calibration value

// LoRaWAN credentials - CHANGE THESE to your TTN device credentials!
// For OTAA (preferred for production)
uint64_t joinEui = 0x0000000000000000;  // TTN Application EUI (MSB)
uint64_t devEui = 0x0000000000000000;   // Device EUI (MSB)
uint8_t appKey[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
uint8_t nwkKey[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

// LoRaWAN node
SPIClass spi;
SX1262 radio = nullptr;
LoRaWANNode node(&radio, &AU915, 2);  // AU915 for Australia, subband 2 for TTN

// Load cell
HX711 scale;
RTC_DATA_ATTR long weight_tare[4] = {0, 0, 0, 0};
RTC_DATA_ATTR int boot_count = 0;

// Channel multiplexer values
const char channel[4] = {0x0F, 0x0B, 0x09, 0x0D};

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
  Serial.println("Calibrating load cells (remove all weight)...");
  for (uint8_t i = 0; i < 4; i++) {
    SelectChannel(i);
    delay(100);
    scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
    while (!scale.is_ready()) {
      delay(10);
    }
    weight_tare[i] = scale.read();
    Serial.printf("Channel %d tare: %ld\n", i, weight_tare[i]);
  }
}

float ReadLoadCell(uint8_t channel_num) {
  SelectChannel(channel_num);
  delay(50);  // Allow channel to settle
  
  if (scale.is_ready()) {
    long reading = scale.read();
    reading = reading - weight_tare[channel_num];
    float weight_g = (float)reading / CALIBRATION_VALUE;
    return weight_g;
  }
  return 0.0;
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  boot_count++;
  Serial.printf("\n=== T-Weigh LoRaWAN Sensor Node ===\n");
  Serial.printf("Boot count: %d\n", boot_count);
  
  // Initialize SPI
  spi.begin(SPI_SCLK, SPI_MISO, SPI_MOSI, csPin);
  
  // Initialize LoRa radio
  pinMode(NRSET, OUTPUT);
  digitalWrite(NRSET, HIGH);
  delay(100);
  
  radio = new Module(csPin, DIO1, NRSET, BUSY, spi);
  
  Serial.print("Initializing SX1262... ");
  int state = radio.begin();
  if (state == RADIOLIB_ERR_NONE) {
    Serial.println("success!");
  } else {
    Serial.printf("failed, code %d\n", state);
    ESP.deepSleep(SLEEP_INTERVAL_MS * 1000);  // Sleep and retry
  }
  
  // Configure for AU915
  radio.setFrequency(915.2);  // AU915 first channel
  radio.setBandwidth(125.0);
  radio.setSpreadingFactor(7);
  radio.setOutputPower(14);  // 14 dBm for AU915
  
  // Initialize LoRaWAN
  Serial.println("Joining LoRaWAN network...");
  node.beginOTAA(joinEui, devEui, nwkKey, appKey);
  
  state = node.activateOTAA();
  if (state == RADIOLIB_LORAWAN_NEW_SESSION) {
    Serial.println("Join successful!");
  } else {
    Serial.printf("Join failed, code %d\n", state);
    ESP.deepSleep(SLEEP_INTERVAL_MS * 1000);  // Sleep and retry
  }
  
  // Initialize I2C for any additional sensors
  Wire.begin(IIC_SDA, IIC_SCL);
  
  // Calibrate on first boot or if requested
  if (boot_count == 1) {
    CalibrateTare();
  }
  
  // Initialize load cell
  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
  
  // Read all load cells
  float weights[4];
  float total_weight = 0;
  
  Serial.println("\nReading load cells:");
  for (uint8_t i = 0; i < 4; i++) {
    weights[i] = ReadLoadCell(i);
    total_weight += weights[i];
    Serial.printf("Channel %d: %.3f kg\n", i, weights[i] / 1000.0);
  }
  Serial.printf("Total weight: %.3f kg\n", total_weight / 1000.0);
  
  // Read battery voltage (if available on ADC)
  uint16_t battery_mv = 3300;  // Default 3.3V if no battery monitoring
  
  // Prepare LoRaWAN payload (compact binary format)
  // Format: [weights as int16 in grams][battery as uint16 in mV]
  uint8_t payload[10];
  int idx = 0;
  
  for (uint8_t i = 0; i < 4; i++) {
    int16_t weight_int = (int16_t)weights[i];
    payload[idx++] = (weight_int >> 8) & 0xFF;
    payload[idx++] = weight_int & 0xFF;
  }
  
  payload[idx++] = (battery_mv >> 8) & 0xFF;
  payload[idx++] = battery_mv & 0xFF;
  
  // Send uplink
  Serial.println("Sending uplink...");
  state = node.sendUplink(payload, sizeof(payload), 1);  // Port 1
  
  if (state == RADIOLIB_ERR_NONE) {
    Serial.println("Uplink sent successfully!");
  } else {
    Serial.printf("Failed to send uplink, code %d\n", state);
  }
  
  // Allow time for any downlinks
  delay(5000);
  
  // Check for downlink
  uint8_t downlink[256];
  size_t downlinkLen = 0;
  state = node.readDownlink(downlink, &downlinkLen);
  if (state == RADIOLIB_ERR_NONE && downlinkLen > 0) {
    Serial.printf("Received downlink (%d bytes): ", downlinkLen);
    for (size_t i = 0; i < downlinkLen; i++) {
      Serial.printf("%02X ", downlink[i]);
    }
    Serial.println();
    
    // Process downlink commands if needed
    if (downlinkLen > 0 && downlink[0] == 0x01) {
      // Command 0x01 = recalibrate
      CalibrateTare();
    }
  }
  
  // Prepare for deep sleep
  Serial.printf("Going to deep sleep for %d seconds...\n", SLEEP_INTERVAL_MS / 1000);
  Serial.flush();
  
  // Configure wake up timer
  esp_sleep_enable_timer_wakeup(SLEEP_INTERVAL_MS * 1000);
  
  // Save LoRaWAN session (if supported)
  // Note: RadioLib may not fully support session persistence yet
  
  // Enter deep sleep
  ESP.deepSleep(SLEEP_INTERVAL_MS * 1000);
}

void loop() {
  // Never reached due to deep sleep
}