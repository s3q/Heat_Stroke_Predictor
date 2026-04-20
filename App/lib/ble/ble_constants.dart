import 'package:flutter_reactive_ble/flutter_reactive_ble.dart';

class BleConstants {
  static const String deviceName = 'HeatStrokeWearable';

  static final Uuid serviceUuid = Uuid.parse(
    '12345678-1234-5678-1234-56789abc0000',
  );
  static final Uuid statusUuid = Uuid.parse(
    '12345678-1234-5678-1234-56789abc0001',
  );
  static final Uuid liveUuid = Uuid.parse(
    '12345678-1234-5678-1234-56789abc0002',
  );
  static final Uuid riskUuid = Uuid.parse(
    '12345678-1234-5678-1234-56789abc0003',
  );
  static final Uuid settingsUuid = Uuid.parse(
    '12345678-1234-5678-1234-56789abc0004',
  );
  static final Uuid commandUuid = Uuid.parse(
    '12345678-1234-5678-1234-56789abc0005',
  );
}

class WearableCharacteristics {
  WearableCharacteristics(this.deviceId);

  final String deviceId;

  QualifiedCharacteristic get status => QualifiedCharacteristic(
    serviceId: BleConstants.serviceUuid,
    characteristicId: BleConstants.statusUuid,
    deviceId: deviceId,
  );

  QualifiedCharacteristic get live => QualifiedCharacteristic(
    serviceId: BleConstants.serviceUuid,
    characteristicId: BleConstants.liveUuid,
    deviceId: deviceId,
  );

  QualifiedCharacteristic get risk => QualifiedCharacteristic(
    serviceId: BleConstants.serviceUuid,
    characteristicId: BleConstants.riskUuid,
    deviceId: deviceId,
  );

  QualifiedCharacteristic get settings => QualifiedCharacteristic(
    serviceId: BleConstants.serviceUuid,
    characteristicId: BleConstants.settingsUuid,
    deviceId: deviceId,
  );

  QualifiedCharacteristic get command => QualifiedCharacteristic(
    serviceId: BleConstants.serviceUuid,
    characteristicId: BleConstants.commandUuid,
    deviceId: deviceId,
  );
}
