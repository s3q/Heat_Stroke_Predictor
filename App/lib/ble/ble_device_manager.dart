import 'package:flutter_reactive_ble/flutter_reactive_ble.dart';

enum DeviceConnectionPhase { disconnected, connecting, connected }

class BleDeviceManager {
  String? deviceId;
  String? deviceName;
  DeviceConnectionPhase phase = DeviceConnectionPhase.disconnected;
  int? mtu;

  void beginConnect({required String id, required String name}) {
    deviceId = id;
    deviceName = name;
    phase = DeviceConnectionPhase.connecting;
  }

  void applyConnectionUpdate(ConnectionStateUpdate update) {
    switch (update.connectionState) {
      case DeviceConnectionState.connecting:
        phase = DeviceConnectionPhase.connecting;
        break;
      case DeviceConnectionState.connected:
        phase = DeviceConnectionPhase.connected;
        break;
      case DeviceConnectionState.disconnecting:
      case DeviceConnectionState.disconnected:
        phase = DeviceConnectionPhase.disconnected;
        break;
    }
  }

  void disconnect() {
    phase = DeviceConnectionPhase.disconnected;
    mtu = null;
  }

  bool get isConnected => phase == DeviceConnectionPhase.connected;
}
