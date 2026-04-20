import 'dart:typed_data';

import 'enums.dart';

class RiskPacket {
  static const int byteLength = 16;

  const RiskPacket({
    required this.version,
    required this.ppgContactState,
    required this.hrInvalidReason,
    required this.spo2InvalidReason,
    required this.bodyContactQuality,
    required this.riskCategory,
    required this.buzzerState,
    required this.ppgSaturation,
    required this.riskHeartRate,
    required this.riskBodyTemp,
    required this.riskHeatIndex,
    required this.totalRisk,
    required this.validityFlags,
  });

  final int version;
  final PpgContactState ppgContactState;
  final InvalidReason hrInvalidReason;
  final InvalidReason spo2InvalidReason;
  final int bodyContactQuality;
  final RiskCategory riskCategory;
  final BuzzerState buzzerState;
  final bool ppgSaturation;
  final int riskHeartRate;
  final int riskBodyTemp;
  final int riskHeatIndex;
  final int totalRisk;
  final int validityFlags;

  static RiskPacket? fromBytes(Uint8List bytes) {
    if (bytes.lengthInBytes != byteLength) {
      return null;
    }
    final data = ByteData.sublistView(bytes);
    return RiskPacket(
      version: data.getUint8(0),
      ppgContactState: PpgContactState.fromValue(data.getUint8(1)),
      hrInvalidReason: InvalidReason.fromValue(data.getUint8(2)),
      spo2InvalidReason: InvalidReason.fromValue(data.getUint8(3)),
      bodyContactQuality: data.getUint8(4),
      riskCategory: RiskCategory.fromValue(data.getUint8(5)),
      buzzerState: BuzzerState.fromValue(data.getUint8(6)),
      ppgSaturation: data.getUint8(7) != 0,
      riskHeartRate: data.getInt8(8),
      riskBodyTemp: data.getInt8(9),
      riskHeatIndex: data.getInt8(10),
      totalRisk: data.getInt8(11),
      validityFlags: data.getUint32(12, Endian.little),
    );
  }
}
