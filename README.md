# bathroom-sensor

ESP32-C3 firmware for bathroom occupancy + ambient sensing with:
- HMMD mmWave (UART)
- VEML7700 lux sensor (I2C)
- AHT20 temperature/humidity sensor (I2C)
- 5V LED strip via PWM MOSFET driver
- Homebridge HTTP Webhooks updates

## Local config
1. Copy `secrets.example.h` to `secrets.h`.
2. Set your Wi-Fi credentials and `HOMEBRIDGE_URL` in `secrets.h`.
3. Keep `secrets.h` uncommitted.

## VS Code + PlatformIO (macOS)
1. Install the VS Code extension: `PlatformIO IDE`.
2. Install CLI:
   ```bash
   brew install platformio
   ```
3. Verify:
   ```bash
   pio --version
   ```
   If `pio` is not in PATH, use:
   ```bash
   ~/Library/Python/3.13/bin/pio --version
   ```
4. Build firmware:
   ```bash
   pio run
   ```
5. Upload to your current detected port:
   ```bash
   pio run -t upload --upload-port /dev/cu.usbmodem1432401
   ```
6. Open serial monitor:
   ```bash
   pio device monitor -b 115200 --port /dev/cu.usbmodem1432401
   ```

## Current behavior
- Occupancy is forced to `false` if mmWave data is stale for 5 seconds.
- LED brightness uses occupancy + lux:
  - no occupancy -> LED off
  - occupied + dark -> brighter
  - occupied + bright -> dimmer/off
- Homebridge updates are sent only when values change meaningfully.
- Wi-Fi reconnect attempts run automatically if connection drops.

## HomeKit cabinetLight control
The firmware exposes a local HTTP control API so Homebridge can switch the light logic:
- `GET http://<esp-ip>/cabinetLight/on`
- `GET http://<esp-ip>/cabinetLight/off`
- `GET http://<esp-ip>/cabinetLight?state=true|false`

Behavior:
- `cabinetLight=on` -> force LED to full brightness (ignores sensor logic)
- `cabinetLight=off` -> auto mode (occupancy + lux curve)

Recommended Homebridge HTTP Webhooks light entry:
```json
{
  "id": "cabinetLight",
  "name": "Cabinet Light",
  "on_url": "http://<esp-ip>/cabinetLight/on",
  "off_url": "http://<esp-ip>/cabinetLight/off"
}
```
