# T-Weigh LoRaWAN Setup Guide for The Things Network (AU915)

## Overview
This project implements a LoRaWAN sensor node for the T-Weigh board that:
- Reads 4 load cells via HX711 with multiplexed channels
- Sends weight data every 60 seconds
- Uses deep sleep between transmissions for power efficiency
- Operates on AU915 frequency band for Australia

## Hardware
- **Board**: T-Weigh with T-Micro32 module (ESP32-PICO-D4 based)
- **MCU**: ESP32-PICO-D4 (System-in-Package with integrated 4MB flash)
- **LoRa**: SX1262 transceiver module
- **Load Cells**: 4 channels via HX711 ADC with CD4051 multiplexer
- **Button**: GPIO 0 for calibration trigger

## Required Libraries

Install these libraries via Arduino IDE Library Manager:
1. **MCCI LoRaWAN LMIC library** by MCCI Catena
2. **HX711** by Bogdan Necula
3. **Wire** (included with Arduino)

## The Things Network Setup

### 1. Create TTN Account
1. Go to https://console.thethingsnetwork.org/
2. Sign up for a free account
3. Select Australia region

### 2. Create Application
1. Click "Applications" → "Add Application"
2. Choose an Application ID (unique name)
3. Add description
4. Select Handler: ttn-handler-au (for Australia)

### 3. Register Device
1. In your application, click "Devices" → "Register Device"
2. Enter Device ID (unique name)
3. Generate Device EUI (click generate button)
4. Click "Register"

### 4. Get Credentials
After device registration, go to device settings and copy:
- **Device EUI** (LSB format)
- **Application EUI** (LSB format) 
- **App Key** (MSB format)

### 5. Configure Code
Update the credentials in `T-Weigh_LoRaWAN_LMIC.ino`:

```cpp
// Example values - REPLACE with your actual credentials!
// Device EUI (LSB) - reverse the byte order from TTN console
static const u1_t PROGMEM DEVEUI[8] = { 0x01, 0x23, 0x45, 0x67, 0x89, 0xAB, 0xCD, 0xEF };

// Application EUI (LSB) - reverse the byte order from TTN console  
static const u1_t PROGMEM APPEUI[8] = { 0xFE, 0xDC, 0xBA, 0x98, 0x76, 0x54, 0x32, 0x10 };

// App Key (MSB) - copy exactly as shown in TTN console
static const u1_t PROGMEM APPKEY[16] = { 0x2B, 0x7E, 0x15, 0x16, 0x28, 0xAE, 0xD2, 0xA6,
                                          0xAB, 0xF7, 0x15, 0x88, 0x09, 0xCF, 0x4F, 0x3C };
```

## Arduino IDE Setup

### 1. Board Configuration
1. Add ESP32 board support:
   - File → Preferences → Additional Board Manager URLs:
   - `https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json`
2. Tools → Board → ESP32 Arduino → **"ESP32 PICO-D4"**
   - Board profile: `esp32:esp32:pico32`
   - The T-Weigh uses a T-Micro32 module (LilyGO's name for their ESP32-PICO-D4 based module)
   - ESP32-PICO-D4 is Espressif's System-in-Package with integrated 4MB flash

### 2. LMIC Configuration
Create/edit `project_config/lmic_project_config.h`:
```cpp
#define CFG_au915 1
#define CFG_sx1262_radio 1
#define LMIC_USE_INTERRUPTS
```

### 3. Upload Settings
- Upload Speed: 115200
- Flash Frequency: 80MHz
- Flash Size: 4MB
- Partition Scheme: Default

## Payload Decoder (TTN Console)

Add this decoder in TTN Application → Payload Formatters:

```javascript
function Decoder(bytes, port) {
  var decoded = {};
  
  // Decode 4 weight values (int16, grams)
  decoded.weight1_kg = ((bytes[0] << 8) | bytes[1]) / 1000.0;
  decoded.weight2_kg = ((bytes[2] << 8) | bytes[3]) / 1000.0;
  decoded.weight3_kg = ((bytes[4] << 8) | bytes[5]) / 1000.0;
  decoded.weight4_kg = ((bytes[6] << 8) | bytes[7]) / 1000.0;
  
  // Total weight
  decoded.total_kg = decoded.weight1_kg + decoded.weight2_kg + 
                     decoded.weight3_kg + decoded.weight4_kg;
  
  // Battery voltage (uint16, mV)
  decoded.battery_v = ((bytes[8] << 8) | bytes[9]) / 1000.0;
  
  return decoded;
}
```

## Power Optimization

The code implements several power-saving features:
1. **Deep Sleep**: ESP32 enters deep sleep between transmissions
2. **RTC Memory**: Preserves calibration and session data across sleep cycles
3. **Adaptive Data Rate**: Automatically adjusts spreading factor for optimal power/range
4. **60-second interval**: Balances data freshness with battery life

## Calibration

### Load Cell Calibration
1. Remove all weight from load cells
2. Power on device (first boot triggers auto-calibration)
3. Or send downlink command `0x01` to recalibrate

### Adjusting Calibration Value
The `CALIBRATION_VALUE` constant (default 2208) may need adjustment:
```cpp
#define CALIBRATION_VALUE 2208  // Adjust based on your load cells
```

To find correct value:
1. Place known weight on cell
2. Read raw value
3. Calculate: `CALIBRATION_VALUE = (raw_reading - tare) / actual_weight_grams`

## Troubleshooting

### Device Not Joining
- Verify credentials are correct (especially byte order for EUI)
- Check antenna connection
- Ensure gateway coverage
- Verify AU915 frequency plan

### No Data Received
- Check serial monitor for error messages
- Verify pin connections
- Ensure load cells are powered properly

### Incorrect Weights
- Recalibrate with no load
- Adjust CALIBRATION_VALUE
- Check wiring of multiplexer channels

## Battery Operation
For battery operation:
1. Connect battery to appropriate power input
2. Consider adding voltage divider for battery monitoring
3. Adjust sleep interval for desired battery life

## Expected Battery Life
With 60-second interval:
- 2000mAh battery: ~2-3 weeks
- 5000mAh battery: ~5-8 weeks
- Solar panel addition recommended for permanent installation

## Downlink Commands
- `0x01`: Trigger recalibration
- Additional commands can be added in `onEvent()` function

## Data Visualization
TTN integrations available:
- Cayenne
- Datacake  
- InfluxDB
- Custom webhook to your server

## Support
- TTN Forum: https://www.thethingsnetwork.org/forum/
- RadioLib Documentation: https://github.com/jgromes/RadioLib
- LMIC Documentation: https://github.com/mcci-catena/arduino-lmic