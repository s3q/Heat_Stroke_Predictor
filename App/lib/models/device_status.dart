import 'dart:typed_data';

class DeviceStatusPacket {
  static const int byteLength = 16;

  const DeviceStatusPacket({
    required this.version,
    required this.connected,
    required this.mtu,
    required this.uptimeMs,
    required this.sampleCounter,
    required this.notifyIntervalMs,
    required this.advActive,
    required this.reserved,
  });

  final int version;
  final bool connected;
  final int mtu;
  final int uptimeMs;
  final int sampleCounter;
  final int notifyIntervalMs;
  final bool advActive;
  final int reserved;

  static DeviceStatusPacket? fromBytes(Uint8List bytes) {
    if (bytes.lengthInBytes != byteLength) {
      return null;
    }
    final data = ByteData.sublistView(bytes);
    return DeviceStatusPacket(
      version: data.getUint8(0),
      connected: data.getUint8(1) != 0,
      mtu: data.getUint16(2, Endian.little),
      uptimeMs: data.getUint32(4, Endian.little),
      sampleCounter: data.getUint32(8, Endian.little),
      notifyIntervalMs: data.getUint16(12, Endian.little),
      advActive: data.getUint8(14) != 0,
      reserved: data.getUint8(15),
    );
  }
}
