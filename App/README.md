# HeatStrokeWearable Mobile App

Flutter BLE app for your ESP32-C3 heat-stroke wearable firmware protocol.

## Implemented BLE Protocol
- Device name: `HeatStrokeWearable`
- Service UUID: `12345678-1234-5678-1234-56789abc0000`
- Characteristics:
  - Status `...0001` read/notify
  - Live `...0002` read/notify
  - Risk `...0003` read/notify
  - Settings `...0004` read/write
  - Command `...0005` write/write-no-response

## Features
- Scan and connect to BLE wearable
- Auto-subscribe to status/live/risk notifications
- Decode little-endian binary packets from firmware structs
- Apply `_x100` / `_x10` scaling
- Handle sentinels (`0x8000`, `0xFFFF`) as unavailable values
- Use `validity_flags` before showing key metrics
- Dashboard with live telemetry and risk card
- Settings read/write
- Command actions: `NOP`, `FORCE_NOTIFY`, `RESET_COUNTERS`
- Manual read of cached GATT values if notifications are missed

## Project Structure
- `lib/ble/` BLE constants, repository, device manager
- `lib/models/` packet models, enums, command/settings codecs
- `lib/services/` packet decoder + telemetry service state
- `lib/screens/` scan, dashboard, settings screens
- `lib/widgets/` connection banner, telemetry card, risk card

## Run in VS Code
1. Open this folder: `heat_stroke_predictor_app`
2. Run:
   - `flutter pub get`
   - `flutter run`
3. Power on your wearable and keep BLE advertising enabled.
4. In app:
   - Go to `Scan`
   - Connect to `HeatStrokeWearable`
   - Open `Dashboard` for live values
   - Open `Settings` for read/write and commands

## Platform BLE Setup
- Android BLE permissions are in `android/app/src/main/AndroidManifest.xml`
- iOS Bluetooth usage strings are in `ios/Runner/Info.plist`

## Notes
- This app is for prototype BLE mode (no pairing/auth).
- Production should use secure BLE pairing/bonding and authenticated writes.
"# heat_stroke_predictor_app" 
