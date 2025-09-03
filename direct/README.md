# RadioLib LoRaWAN Implementation (UNTESTED)

⚠️ **WARNING: This implementation is UNTESTED and provided as an alternative approach only.**

## Overview
This directory contains an alternative LoRaWAN implementation using RadioLib instead of LMIC. 

### Why This Version Exists
- RadioLib offers a more modern C++ API
- Potentially simpler code structure
- Good for experimentation

### Why It's Not Recommended
- **UNTESTED**: Has not been verified with actual hardware
- RadioLib's LoRaWAN support is less mature than LMIC
- Limited session persistence across deep sleep
- May have compatibility issues with The Things Network
- Regional parameters (AU915) implementation is less proven

## Files
- `T-Weigh_LoRaWAN.ino` - RadioLib-based implementation

## If You Want to Try This Version
1. Install RadioLib library via Arduino Library Manager
2. Copy the sketch to your Arduino sketch folder
3. Add your TTN credentials (similar to main implementation)
4. Test thoroughly before production use
5. Be prepared to debug timing and session issues

## Recommended Alternative
Use the main LMIC implementation in the parent directory, which is:
- Production tested
- Fully compatible with TTN
- Properly handles deep sleep and session persistence
- Well-documented and supported

## Known Limitations
- Session keys may not persist across deep sleep
- OTAA join may be required after each wake
- AU915 frequency plan may need manual adjustment
- Power consumption likely higher due to rejoining

## Support
This implementation is provided as-is without support. For production use, please use the LMIC version.