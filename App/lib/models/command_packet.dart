import 'dart:typed_data';

enum WearableCommandId {
  nop(0),
  forceNotify(1),
  resetCounters(2);

  const WearableCommandId(this.value);
  final int value;
}

class CommandPacket {
  static const int versionCurrent = 1;
  static const int byteLength = 4;

  const CommandPacket({
    required this.version,
    required this.commandId,
    required this.valueI16,
  });

  final int version;
  final WearableCommandId commandId;
  final int valueI16;

  Uint8List toBytes() {
    final bytes = Uint8List(byteLength);
    final data = ByteData.sublistView(bytes);
    data.setUint8(0, version);
    data.setUint8(1, commandId.value);
    data.setInt16(2, valueI16, Endian.little);
    return bytes;
  }

  factory CommandPacket.nop() => const CommandPacket(
    version: versionCurrent,
    commandId: WearableCommandId.nop,
    valueI16: 0,
  );

  factory CommandPacket.forceNotify() => const CommandPacket(
    version: versionCurrent,
    commandId: WearableCommandId.forceNotify,
    valueI16: 0,
  );

  factory CommandPacket.resetCounters() => const CommandPacket(
    version: versionCurrent,
    commandId: WearableCommandId.resetCounters,
    valueI16: 0,
  );
}
