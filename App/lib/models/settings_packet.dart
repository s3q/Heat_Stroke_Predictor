import 'dart:typed_data';

class SettingsPacket {
  static const int versionCurrent = 1;
  static const int byteLength = 8;
  static const int minNotifyIntervalMs = 200;
  static const int maxNotifyIntervalMs = 10000;

  const SettingsPacket({
    required this.version,
    required this.buzzerEnabled,
    required this.debugLogLevel,
    required this.reserved,
    required this.notifyIntervalMs,
    required this.bodyTempOffsetCX100,
  });

  final int version;
  final bool buzzerEnabled;
  final int debugLogLevel;
  final int reserved;
  final int notifyIntervalMs;
  final int bodyTempOffsetCX100;

  double get bodyTempOffsetC => bodyTempOffsetCX100 / 100.0;

  SettingsPacket copyWith({
    bool? buzzerEnabled,
    int? debugLogLevel,
    int? notifyIntervalMs,
    int? bodyTempOffsetCX100,
  }) {
    final nextNotify = notifyIntervalMs ?? this.notifyIntervalMs;
    return SettingsPacket(
      version: versionCurrent,
      buzzerEnabled: buzzerEnabled ?? this.buzzerEnabled,
      debugLogLevel: debugLogLevel ?? this.debugLogLevel,
      reserved: 0,
      notifyIntervalMs: nextNotify.clamp(
        minNotifyIntervalMs,
        maxNotifyIntervalMs,
      ),
      bodyTempOffsetCX100: bodyTempOffsetCX100 ?? this.bodyTempOffsetCX100,
    );
  }

  Uint8List toBytes() {
    final bytes = Uint8List(byteLength);
    final data = ByteData.sublistView(bytes);
    data.setUint8(0, version);
    data.setUint8(1, buzzerEnabled ? 1 : 0);
    data.setUint8(2, debugLogLevel);
    data.setUint8(3, reserved);
    data.setUint16(4, notifyIntervalMs, Endian.little);
    data.setInt16(6, bodyTempOffsetCX100, Endian.little);
    return bytes;
  }

  static SettingsPacket? fromBytes(Uint8List bytes) {
    if (bytes.lengthInBytes != byteLength) {
      return null;
    }
    final data = ByteData.sublistView(bytes);
    final notifyInterval = data.getUint16(4, Endian.little);
    return SettingsPacket(
      version: data.getUint8(0),
      buzzerEnabled: data.getUint8(1) != 0,
      debugLogLevel: data.getUint8(2),
      reserved: data.getUint8(3),
      notifyIntervalMs: notifyInterval.clamp(
        minNotifyIntervalMs,
        maxNotifyIntervalMs,
      ),
      bodyTempOffsetCX100: data.getInt16(6, Endian.little),
    );
  }
}
