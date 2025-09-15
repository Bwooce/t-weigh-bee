# T-Weigh LoRaWAN Sensor Node

## Overview

T-Weigh is a LoRaWAN-enabled weight monitoring system designed for the LilyGO T-Weigh board. It reads four load cells via HX711 ADCs and transmits raw ADC values over LoRaWAN to The Things Network (TTN).

## Hardware Requirements

### Board
- **LilyGO T-Weigh** with T-Micro32 module (ESP32-PICO-D4 System-in-Package)
- **MCU**: ESP32-PICO-D4 with integrated 4MB flash
- **LoRa**: SX1262 transceiver module
- **Load Cells**: 4 channels via HX711 24-bit ADC with CD4051 multiplexer

### Important: USB Connection
⚠️ **The T-Weigh board's USB-C port is NOT a standard USB data port!**

You need a special **USB-to-TTL adapter** from LilyGO to program this board:
- The USB-C port on the T-Weigh board only provides power
- Programming requires the LilyGO USB-TTL adapter (CH340 or similar)
- Connect the adapter to the programming pins on the board
- The device will appear as "QinHeng Electronics USB Single Serial" in lsusb

## Features

- Reads 4 load cells with 24-bit resolution
- Sends raw ADC values (no tare/calibration on device)
- Deep sleep between transmissions for power efficiency
- Configurable via LoRaWAN downlinks
- Runtime configurable parameters stored in NVS
- Automatic session restoration after deep sleep
- 12-hour periodic configuration status reports
- HX711 power management with configurable stabilization time

## Quick Start

### 1. Install Dependencies

```bash
arduino-cli lib install RadioLib
arduino-cli lib install HX711
```

### 2. Configure Credentials

```bash
cp lorawan_credentials.h.example lorawan_credentials.h
# Edit lorawan_credentials.h with your TTN credentials
```

### 3. Compile and Upload

```bash
export PATH=$PATH:/home/bruce/Arduino/bin
arduino-cli compile --fqbn esp32:esp32:pico32 t-weigh-bee.ino
arduino-cli upload --fqbn esp32:esp32:pico32 --port /dev/ttyUSB0 t-weigh-bee.ino
```

### 4. Monitor Output

```bash
arduino-cli monitor --port /dev/ttyUSB0 --config baudrate=115200
```

## Data Format

### Data Uplink (Port 1, 12 bytes)
| Bytes | Content | Format | Range |
|-------|---------|--------|-------|
| 0-2   | Channel 0 raw ADC | int24, big-endian, signed | -8,388,608 to 8,388,607 |
| 3-5   | Channel 1 raw ADC | int24, big-endian, signed | -8,388,608 to 8,388,607 |
| 6-8   | Channel 2 raw ADC | int24, big-endian, signed | -8,388,608 to 8,388,607 |
| 9-11  | Channel 3 raw ADC | int24, big-endian, signed | -8,388,608 to 8,388,607 |

**Note:** Raw ADC values are sent without tare or calibration. Application layer should handle conversion to actual weight.

### Configuration Status Uplink (Port 2, 12 bytes)
Sent automatically after join and every 12 hours:

| Byte | Content | Description |
|------|---------|-------------|
| 0    | Config version | Always 0x01 for this version |
| 1-2  | TX interval | Seconds between transmissions (big-endian) |
| 3-4  | Stabilization time | Milliseconds to wait after wake (big-endian) |
| 5    | LoRa plan | 0=AU915, 1=US915, 2=EU868, 3=AS923 |
| 6    | Sub-band | 0-8 (only used for US915/AU915) |
| 7    | Flags | Bit field (see below) |
| 8-11 | Firmware | 4 ASCII characters (e.g., "1.0.0") |

**Flags byte (byte 7):**
- Bit 0: Dwell time enforcement (0=disabled, 1=enabled)
- Bit 1: HX711 power control (0=always on, 1=power down during sleep)
- Bit 2: Debug mode (0=disabled, 1=enabled with serial output)
- Bits 3-7: Reserved for future use

## Downlink Commands (Port 1)

| Command | Payload | Description | Valid Range |
|---------|---------|-------------|-------------|
| 0x20    | 2 bytes | Set TX interval | 10-65535 seconds |
| 0x21    | 2 bytes | Set stabilization time | 100-10000 ms |
| 0x22    | 1 byte  | Set LoRa plan | 0-3 (see above) |
| 0x23    | 1 byte  | Set sub-band | 0-8 |
| 0x24    | 1 byte  | Set dwell time enforcement | 0=off, 1=on |
| 0x25    | 1 byte  | Set HX711 power control | 0=off, 1=on |
| 0x26    | 1 byte  | Set debug mode | 0=off, 1=on |
| 0x30    | None    | Request immediate config uplink | - |
| 0xFF    | None    | Reset device | - |

**Note:** All multi-byte values are big-endian. Settings are saved to non-volatile storage and persist across reboots.

## Serial Commands

Hold GPIO 0 button during boot to enter interactive mode:

- `help` - Show available commands
- `read` - Read all channels
- `status` - Show device status
- `send` - Send data immediately
- `save` - Save LoRaWAN nonces to flash
- `plan [0-3]` - Set LoRa plan
- `subband [n]` - Set sub-band (0-8)
- `dwell on/off` - Control dwell time enforcement
- `reset` - Reset device

## Power Optimization

The device implements several power-saving features:

1. **Deep Sleep**: Between transmissions (default 60 seconds)
2. **HX711 Power Control**: Configurable power-down during sleep
3. **Serial Port**: Disabled when debug mode is off
4. **Stabilization Time**: 2 seconds recommended after wake

## TTN Setup

1. Create an application in The Things Network Console
2. Register a new device with:
   - LoRaWAN version: 1.0.3
   - Regional Parameters: AU915 (or your region)
   - Frequency plan: Australia 915-928 MHz, FSB 2
3. Copy the credentials to `lorawan_credentials.h`:
   - DevEUI in LSB format (reverse byte order)
   - AppKey in MSB format (exact copy)
   - JoinEUI: typically all zeros for TTN

## Troubleshooting

### Device Not Joining
- Check credentials in `lorawan_credentials.h`
- Verify frequency plan matches your gateway
- Ensure DevNonce is incrementing (check serial output)

### No Data After Join
- Device may be in deep sleep (60s default)
- Check if session is being restored properly
- Verify HX711 connections

### Serial Port Issues
- Ensure you're using the USB-TTL adapter, not the USB-C port
- Check device appears as `/dev/ttyUSB0` or `/dev/ttyACM1`
- Debug mode must be enabled for serial output

## License

MIT License - See LICENSE file for details

## Support

For issues or questions:
- GitHub Issues: [Create an issue](https://github.com/your-repo/t-weigh-bee/issues)
- TTN Forum: [The Things Network Community](https://www.thethingsnetwork.org/forum/)