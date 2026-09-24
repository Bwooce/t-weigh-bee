# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

T-Weigh LoRaWAN sensor node: reads 4 load cells via an HX711 (multiplexed through a CD4051) and sends raw readings over LoRaWAN to The Things Network (AU915). Target board is the LilyGO T-Weigh (ESP32-PICO-D4 + SX1262). The firmware is a single RadioLib-based sketch, `t-weigh-bee.ino`; the old LMIC version has been removed.

## Hardware

- **Board**: T-Weigh with T-Micro32 module (ESP32-PICO-D4, integrated 4MB flash)
- **LoRa**: SX1262, 32MHz TCXO (`radio.setTCXO(2.4)`)
- **Load cells**: 4 channels, one HX711 (gain 128) behind a CD4051 multiplexer
- **Button**: GPIO 0 (held at boot → forces rejoin and enters interactive mode)
- **USB**: the board's USB-C port is power only. Programming/serial needs LilyGO's USB-TTL adapter, which shows up in `lsusb` as "QinHeng Electronics USB Single Serial" — use that to find the right `/dev/ttyUSB*`.

### Pins
- Mux select: CDA GPIO 27, CDB GPIO 14, CDC GPIO 26, CDD GPIO 25 (channel codes in `channelSelect[]`)
- HX711: DOUT GPIO 21, SCK GPIO 22

## Development Commands

Arduino CLI lives at `/home/bruce/Arduino/bin/arduino-cli` on the dev machine (not present in cloud sessions — download it from downloads.arduino.cc if needed).

```bash
export PATH=$PATH:/home/bruce/Arduino/bin
arduino-cli compile --fqbn esp32:esp32:pico32 t-weigh-bee.ino
arduino-cli upload  --fqbn esp32:esp32:pico32 --port /dev/ttyUSB0 t-weigh-bee.ino
arduino-cli monitor --port /dev/ttyUSB0 --config baudrate=115200
```

Libraries: `arduino-cli lib install RadioLib HX711` (last verified with RadioLib 7.7.1, HX711 0.6.5, esp32 core 3.3.11).

To compile without real credentials, copy `lorawan_credentials.h.example` to `lorawan_credentials.h` — the example compiles as-is.

## Credentials

`lorawan_credentials.h` is gitignored. It defines `uint8_t` arrays (not LMIC `u1_t`):
- `DEVEUI[8]` — LSB (reverse of the TTN console)
- `APPKEY[16]` — MSB (exact copy from TTN console); also used as the network key (LoRaWAN 1.0.x)
- `APPEUI[8]` — present but currently ignored; the sketch always joins with an all-zero JoinEUI

## Code Architecture

Everything runs in `setup()`; `loop()` is unused because the device deep-sleeps after each cycle.

1. **Boot**: load settings from NVS (`Preferences`, namespace `t-weigh`), sanitising out-of-range values
2. **LoRaWAN**: build a `LoRaWANNode` for the configured plan; restore session from RTC memory if present, otherwise OTAA join. Nonces are kept in RTC memory and saved to NVS every 100 uplinks.
3. **Send**: read all 4 channels (averaged raw HX711 counts), send an 8-byte uplink on port 1, handle any downlink
4. **Config uplink**: 12-byte status on port 2 after join and every 12 hours
5. **Sleep**: save session to RTC, optionally power down the HX711, deep sleep for `txInterval`

Defaults: 60 s interval, AU915 sub-band 2, DR0/SF12, ADR off, dwell time off, HX711 power control on, 2000 ms HX711 stabilisation after wake.

### Uplink payload (port 1, 8 bytes)
Four int16 big-endian **raw ADC counts** (channel 0–3). There is no tare or calibration on the device; the application/TTN decoder is expected to convert to weight. Raw values are clamped to ±32767 and a channel with no reading sends 0.

Full payload, config-uplink and decoder details: `README.md` and `TTN_Setup_Guide.md`.

### Downlink commands (port 1)
| Cmd | Payload | Action |
|-----|---------|--------|
| 0x20 | 2 bytes | TX interval, seconds (10–65535; smaller ignored) |
| 0x21 | 2 bytes | HX711 stabilisation, ms (100–10000) |
| 0x22 | 1 byte | LoRa plan: 0=EU868, 1=US915, 3=AU915 (restart to apply) |
| 0x23 | 1 byte | Sub-band 0–8 |
| 0x24 | 1 byte | Dwell time 0/1 |
| 0x25 | 1 byte | HX711 power-down during sleep 0/1 |
| 0x26 | 1 byte | Debug/serial output 0/1 |
| 0x27 | 1 byte | Data rate 0–5 (DR0/SF12 … DR5/SF7) |
| 0x30 | — | Send config uplink now |
| 0xFF | — | Reset |

Settings persist in NVS. Plan values are RadioLib `LoRaWANBandNum_t` values; only plans that `setup()` builds a node for are accepted (see `isSupportedLoraPlan()`).

### Serial commands (interactive mode)
Hold GPIO 0 at boot: the device stays awake, accepts serial commands and still uplinks every `txInterval`.
`help`, `read`, `status`, `send`, `save` (nonces to NVS), `plan [n]`, `subband [n]`, `dwell on|off`, `reset`.

## Debugging

`#define DEBUG 1` sets the default for the runtime `debugMode` (also toggled by downlink 0x26). Serial is only initialised when debug mode is on. Macros: `DEBUG_PRINT`, `DEBUG_PRINTLN`, `DEBUG_PRINTF`.

## File Structure

- `t-weigh-bee.ino` — firmware
- `lorawan_credentials.h.example` — credentials template (copy to `lorawan_credentials.h`)
- `README.md` — user-facing overview, payload formats, commands
- `TTN_Setup_Guide.md` — TTN setup, payload decoder
