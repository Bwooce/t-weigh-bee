# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

T-Weigh LoRaWAN Sensor Node for reading 4 load cells via HX711 and sending data over LoRaWAN to The Things Network (AU915 frequency band). The project is designed for the LilyGO T-Weigh board with ESP32-PICO-D4 and SX1262 LoRa transceiver.

## Hardware Configuration

- **Board**: T-Weigh with T-Micro32 module (ESP32-PICO-D4 based)
- **MCU**: ESP32-PICO-D4 (System-in-Package with integrated 4MB flash)
- **LoRa**: SX1262 transceiver module
- **Load Cells**: 4 channels via HX711 ADC with CD4051 multiplexer
- **Button**: GPIO 0 for calibration trigger

## Development Commands

### Arduino CLI Setup
- Arduino CLI is located at: `/home/bruce/Arduino/bin/arduino-cli`
- Always add to PATH: `export PATH=$PATH:/home/bruce/Arduino/bin`

### Board Configuration
- **Board Type**: ESP32 PICO-D4
- **FQBN**: `esp32:esp32:pico32`

### Compilation and Upload

#### For RadioLib version (RECOMMENDED):
```bash
export PATH=$PATH:/home/bruce/Arduino/bin
arduino-cli compile --fqbn esp32:esp32:pico32 t-weigh-bee-radiolib.ino
arduino-cli upload --fqbn esp32:esp32:pico32 --port /dev/ttyUSB0 t-weigh-bee-radiolib.ino
```

#### For LMIC version (legacy):
```bash
export PATH=$PATH:/home/bruce/Arduino/bin
arduino-cli compile --fqbn esp32:esp32:pico32 t-weigh-bee.ino
arduino-cli upload --fqbn esp32:esp32:pico32 --port /dev/ttyUSB0 t-weigh-bee.ino
```

### Monitor Serial Output
```bash
arduino-cli monitor --port /dev/ttyUSB0 --config baudrate=115200
```

## Required Libraries

### For RadioLib version:
- **RadioLib** (latest) - Universal wireless library with native SX1262 support
- **HX711** (v0.6.2) - Load cell interface

### For LMIC version:
- **MCCI LoRaWAN LMIC library** (v5.0.1) - LoRaWAN stack (experimental SX1262 support)
- **HX711** (v0.6.2) - Load cell interface

### Installing RadioLib:
```bash
arduino-cli lib install RadioLib
```

## Pin Assignments

### Load Cell Multiplexer Control
- CDA (GPIO 27) - Channel select bit A
- CDB (GPIO 14) - Channel select bit B  
- CDC (GPIO 26) - Channel select bit C
- CDD (GPIO 25) - Channel select bit D

### HX711 Load Cell Interface
- LOADCELL_DOUT_PIN (GPIO 21) - Data output from HX711
- LOADCELL_SCK_PIN (GPIO 22) - Serial clock to HX711

## Code Architecture

### Main Components
1. **Load Cell Reading**: HX711 interface with channel multiplexing via CD4051
2. **LoRaWAN Communication**: LMIC library for TTN connectivity on AU915 band
3. **Power Management**: Deep sleep between transmissions (60-second intervals)
4. **Calibration**: Button-triggered calibration mode on GPIO 0

### Key Configuration
- **Transmission Interval**: 60 seconds (TX_INTERVAL)
- **Calibration Value**: 2208 (adjust based on load cell calibration)
- **Debug Mode**: Controlled via DEBUG define (set to 0 for production)

## Credentials Setup

1. Copy `lorawan_credentials.h.example` to `lorawan_credentials.h`
2. Update with actual TTN credentials:
   - APPEUI (JoinEUI) - Usually all zeros for TTN, LSB format
   - DEVEUI - Device EUI in LSB format (reverse byte order from TTN console)
   - APPKEY - Application Key in MSB format (exact copy from TTN console)

## Power Management

The code uses ESP32 deep sleep between LoRaWAN transmissions:
- Wakes every 60 seconds to read sensors and transmit
- Uses `esp_sleep.h` and `driver/rtc_io.h` for sleep management
- GPIO states preserved during sleep for multiplexer control

## Debugging

Set `#define DEBUG 1` to enable serial output:
- DEBUG_PRINT(x) - Print without newline
- DEBUG_PRINTLN(x) - Print with newline  
- DEBUG_PRINTF(x, ...) - Printf-style formatted output

## File Structure

- `t-weigh_bee.ino` - Main Arduino sketch
- `lorawan_credentials.h` - TTN credentials (gitignored)
- `lorawan_credentials.h.example` - Template for credentials
- `TTN_Setup_Guide.md` - Detailed setup instructions
- `direct/` - Contains alternative implementation files

## Notes

- The project is optimized for low power operation with deep sleep
- Load cell calibration value may need adjustment based on physical setup
- LoRaWAN operates on AU915 frequency band for Australia
- Uses OTAA (Over-The-Air Activation) for TTN connection

## RadioLib Version Features

The new RadioLib implementation (`t-weigh-bee-radiolib.ino`) includes:

### Serial Commands
- `help` - Show available commands
- `tare` - Tare all channels
- `tare [0-3]` - Tare specific channel
- `read` - Read all channels
- `calib [value]` - Set calibration factor
- `status` - Show device status
- `send` - Send data immediately
- `reset` - Reset device

### LoRaWAN Downlink Commands
The device accepts the following downlink commands on port 1:
- `0x01` - Tare all channels
- `0x10` - Tare channel 0
- `0x11` - Tare channel 1
- `0x12` - Tare channel 2
- `0x13` - Tare channel 3
- `0x20` + 2 bytes - Set TX interval (in seconds)
- `0xFF` - Reset device

### Persistent Storage
Tare values and calibration settings are stored in ESP32 preferences and survive power cycles.

### Interactive Mode
Hold GPIO 0 button during boot to enter interactive mode where the device stays awake for serial commands.

### Payload Format
Uplink payload (8 bytes total):
- Bytes 0-1: Channel 0 weight (int16, big-endian, grams)
- Bytes 2-3: Channel 1 weight (int16, big-endian, grams)
- Bytes 4-5: Channel 2 weight (int16, big-endian, grams)
- Bytes 6-7: Channel 3 weight (int16, big-endian, grams)

Special value -32768 indicates sensor error/not connected.
- The t-weigh device will appear on lsusb as a "QinHeng Electronics USB Single Serial" as it needs a usb to serial device, so look for that to identify which device to upload to.