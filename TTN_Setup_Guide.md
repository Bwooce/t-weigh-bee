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

Install these libraries via Arduino CLI or Arduino IDE Library Manager:
1. **RadioLib** by Jan Gromes - Universal wireless library with native SX1262 support
2. **HX711** by Bogdan Necula - Load cell interface library

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
1. Copy `lorawan_credentials.h.example` to `lorawan_credentials.h`
2. Update with your actual TTN credentials:

```cpp
// Device EUI (LSB format - reverse byte order from TTN console)
uint64_t devEUI = 0x0123456789ABCDEF;

// Join/Application EUI (LSB format - usually all zeros for TTN)
uint64_t joinEUI = 0x0000000000000000;

// Application Key (MSB format - copy exactly from TTN console)
uint8_t appKey[] = { 0x2B, 0x7E, 0x15, 0x16, 0x28, 0xAE, 0xD2, 0xA6,
                      0xAB, 0xF7, 0x15, 0x88, 0x09, 0xCF, 0x4F, 0x3C };

// Network Key (not used for LoRaWAN 1.0.x, same as appKey)
uint8_t nwkKey[] = { 0x2B, 0x7E, 0x15, 0x16, 0x28, 0xAE, 0xD2, 0xA6,
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

### 2. Compile and Upload
Using Arduino CLI:
```bash
export PATH=$PATH:/home/bruce/Arduino/bin
arduino-cli compile --fqbn esp32:esp32:pico32 t-weigh-bee.ino
arduino-cli upload --fqbn esp32:esp32:pico32 --port /dev/ttyUSB0 t-weigh-bee.ino
```

### 3. Upload Settings
- Upload Speed: 115200
- Flash Frequency: 80MHz
- Flash Size: 4MB
- Partition Scheme: Default

## Payload Decoder (TTN Console)

Add this decoder in TTN Application → Payload Formatters:

```javascript
function decodeUplink(input) {
  var decoded = {};
  var bytes = input.bytes;
  var port = input.fPort;

  if (port === 1) {
    // Data uplink - 8 bytes with 16-bit signed values
    // Convert from signed 16-bit to actual values
    function toInt16(b1, b2) {
      var val = (b1 << 8) | b2;
      if (val > 32767) val -= 65536;
      return val;
    }

    decoded.channel0_raw = toInt16(bytes[0], bytes[1]);
    decoded.channel1_raw = toInt16(bytes[2], bytes[3]);
    decoded.channel2_raw = toInt16(bytes[4], bytes[5]);
    decoded.channel3_raw = toInt16(bytes[6], bytes[7]);

    // Note: These are raw ADC values, apply calibration as needed
    // Example: weight_grams = (raw_value - tare_value) / calibration_factor
  } else if (port === 2) {
    // Configuration uplink - 12 bytes
    decoded.config_version = bytes[0];
    decoded.tx_interval_sec = (bytes[1] << 8) | bytes[2];
    decoded.stabilization_ms = (bytes[3] << 8) | bytes[4];
    decoded.lora_plan = bytes[5];
    decoded.sub_band = bytes[6];

    // Flags byte (byte 7)
    var flags = bytes[7];
    decoded.data_rate = (flags >> 4) & 0x0F;
    decoded.dwell_time = (flags & 0x01) ? true : false;
    decoded.hx711_power = (flags & 0x02) ? true : false;
    decoded.debug_mode = (flags & 0x04) ? true : false;

    // Firmware version (4 ASCII chars)
    decoded.firmware = String.fromCharCode(bytes[8], bytes[9], bytes[10], bytes[11]);
  }

  return {
    data: decoded
  };
}
```

## Power Optimization

The code implements several power-saving features:
1. **Deep Sleep**: ESP32 enters deep sleep between transmissions
2. **RTC Memory**: Preserves LoRaWAN session data across sleep cycles
3. **Manual Data Rate**: ADR disabled for full control over SF (default SF12 for max range)
4. **HX711 Power Control**: Can power down load cell ADC during sleep
5. **60-second interval**: Default interval, adjustable via downlink

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

## Downlink Commands (Port 1)

| Command | Payload | Description |
|---------|---------|-------------|
| 0x01 | None | Tare all channels |
| 0x10-0x13 | None | Tare specific channel (0-3) |
| 0x20 | 2 bytes | Set TX interval (10-65535 seconds) |
| 0x21 | 2 bytes | Set stabilization time (100-10000 ms) |
| 0x22 | 1 byte | Set LoRa plan (0=EU868, 1=US915, 3=AU915, etc.) |
| 0x23 | 1 byte | Set sub-band (0-8) |
| 0x24 | 1 byte | Set dwell time (0=off, 1=on) |
| 0x25 | 1 byte | Set HX711 power control (0=off, 1=on) |
| 0x26 | 1 byte | Set debug mode (0=off, 1=on) |
| 0x27 | 1 byte | Set data rate (0=SF12, 1=SF11, 2=SF10, 3=SF9, 4=SF8, 5=SF7) |
| 0x30 | None | Request config uplink |
| 0xFF | None | Reset device |

## Data Visualization
TTN integrations available:
- Cayenne
- Datacake  
- InfluxDB
- Custom webhook to your server

## Support
- TTN Forum: https://www.thethingsnetwork.org/forum/
- RadioLib Documentation: https://github.com/jgromes/RadioLib
- RadioLib LoRaWAN Examples: https://github.com/jgromes/RadioLib/tree/master/examples/LoRaWAN