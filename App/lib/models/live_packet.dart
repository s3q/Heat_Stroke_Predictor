import 'dart:typed_data';

import 'enums.dart';

class LivePacket {
  static const int byteLength = 48;
  static const int invalidI16 = -32768;
  static const int invalidU16 = 0xFFFF;

  const LivePacket({
    required this.version,
    required this.reserved0,
    required this.reserved1,
    required this.sampleCounter,
    required this.timestampMs,
    required this.rawBodyTempC,
    required this.filteredBodyTempC,
    required this.estimatedBodyTempC,
    required this.ambientTempC,
    required this.humidityPercent,
    required this.pressureHpa,
    required this.heatIndexC,
    required this.heartRateBpm,
    required this.spo2Percent,
    required this.heartRateConfPctRaw,
    required this.spo2ConfPctRaw,
    required this.ppgContactState,
    required this.hrInvalidReason,
    required this.spo2InvalidReason,
    required this.bodyContactQuality,
    required this.riskHeartRate,
    required this.riskBodyTemp,
    required this.riskHeatIndex,
    required this.totalRisk,
    required this.riskCategory,
    required this.buzzerState,
    required this.ppgSaturation,
    required this.reserved2,
    required this.validityFlags,
  });

  final int version;
  final int reserved0;
  final int reserved1;
  final int sampleCounter;
  final int timestampMs;
  final double? rawBodyTempC;
  final double? filteredBodyTempC;
  final double? estimatedBodyTempC;
  final double? ambientTempC;
  final double? humidityPercent;
  final double? pressureHpa;
  final double? heatIndexC;
  final double? heartRateBpm;
  final double? spo2Percent;
  final int heartRateConfPctRaw;
  final int spo2ConfPctRaw;
  final PpgContactState ppgContactState;
  final InvalidReason hrInvalidReason;
  final InvalidReason spo2InvalidReason;
  final int bodyContactQuality;
  final int riskHeartRate;
  final int riskBodyTemp;
  final int riskHeatIndex;
  final int totalRisk;
  final RiskCategory riskCategory;
  final BuzzerState buzzerState;
  final bool ppgSaturation;
  final int reserved2;
  final int validityFlags;

  int? get heartRateConfidencePct =>
      validityFlags.hasFlag(ValidityFlags.hrConfidence)
      ? heartRateConfPctRaw
      : null;

  int? get spo2ConfidencePct =>
      validityFlags.hasFlag(ValidityFlags.spo2Confidence)
      ? spo2ConfPctRaw
      : null;

  static LivePacket? fromBytes(Uint8List bytes) {
    if (bytes.lengthInBytes != byteLength) {
      return null;
    }
    final data = ByteData.sublistView(bytes);
    return LivePacket(
      version: data.getUint8(0),
      reserved0: data.getUint8(1),
      reserved1: data.getUint16(2, Endian.little),
      sampleCounter: data.getUint32(4, Endian.little),
      timestampMs: data.getUint32(8, Endian.little),
      rawBodyTempC: _decodeI16Scaled(data.getInt16(12, Endian.little), 100.0),
      filteredBodyTempC: _decodeI16Scaled(
        data.getInt16(14, Endian.little),
        100.0,
      ),
      estimatedBodyTempC: _decodeI16Scaled(
        data.getInt16(16, Endian.little),
        100.0,
      ),
      ambientTempC: _decodeI16Scaled(data.getInt16(18, Endian.little), 100.0),
      humidityPercent: _decodeU16Scaled(
        data.getUint16(20, Endian.little),
        100.0,
      ),
      pressureHpa: _decodeU16Scaled(data.getUint16(22, Endian.little), 10.0),
      heatIndexC: _decodeI16Scaled(data.getInt16(24, Endian.little), 100.0),
      heartRateBpm: _decodeU16Scaled(data.getUint16(26, Endian.little), 10.0),
      spo2Percent: _decodeU16Scaled(data.getUint16(28, Endian.little), 10.0),
      heartRateConfPctRaw: data.getUint8(30),
      spo2ConfPctRaw: data.getUint8(31),
      ppgContactState: PpgContactState.fromValue(data.getUint8(32)),
      hrInvalidReason: InvalidReason.fromValue(data.getUint8(33)),
      spo2InvalidReason: InvalidReason.fromValue(data.getUint8(34)),
      bodyContactQuality: data.getUint8(35),
      riskHeartRate: data.getUint8(36),
      riskBodyTemp: data.getUint8(37),
      riskHeatIndex: data.getUint8(38),
      totalRisk: data.getUint8(39),
      riskCategory: RiskCategory.fromValue(data.getUint8(40)),
      buzzerState: BuzzerState.fromValue(data.getUint8(41)),
      ppgSaturation: data.getUint8(42) != 0,
      reserved2: data.getUint8(43),
      validityFlags: data.getUint32(44, Endian.little),
    );
  }

  static double? _decodeI16Scaled(int value, double scale) {
    if (value == invalidI16) {
      return null;
    }
    return value / scale;
  }

  static double? _decodeU16Scaled(int value, double scale) {
    if (value == invalidU16) {
      return null;
    }
    return value / scale;
  }
}
