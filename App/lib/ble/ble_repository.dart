import 'dart:typed_data';

import 'package:flutter_reactive_ble/flutter_reactive_ble.dart';

import 'ble_constants.dart';

class BleRepository {
  BleRepository({FlutterReactiveBle? ble}) : _ble = ble ?? FlutterReactiveBle();

  final FlutterReactiveBle _ble;

  Stream<DiscoveredDevice> scanDevices() {
    return _ble.scanForDevices(
      withServices: [BleConstants.serviceUuid],
      scanMode: ScanMode.lowLatency,
    );
  }

  Stream<ConnectionStateUpdate> connectToDevice(String deviceId) {
    return _ble.connectToDevice(
      id: deviceId,
      connectionTimeout: const Duration(seconds: 12),
    );
  }

  Stream<List<int>> subscribe(QualifiedCharacteristic characteristic) {
    return _ble.subscribeToCharacteristic(characteristic);
  }

  Future<Uint8List> read(QualifiedCharacteristic characteristic) async {
    final bytes = await _ble.readCharacteristic(characteristic);
    return Uint8List.fromList(bytes);
  }

  Future<void> writeWithResponse(
    QualifiedCharacteristic characteristic,
    Uint8List payload,
  ) async {
    await _ble.writeCharacteristicWithResponse(characteristic, value: payload);
  }

  Future<void> writeWithoutResponse(
    QualifiedCharacteristic characteristic,
    Uint8List payload,
  ) async {
    await _ble.writeCharacteristicWithoutResponse(
      characteristic,
      value: payload,
    );
  }
}
