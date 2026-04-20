import 'dart:typed_data';

import '../models/device_status.dart';
import '../models/live_packet.dart';
import '../models/risk_packet.dart';
import '../models/settings_packet.dart';

class PacketDecoder {
  const PacketDecoder();

  DeviceStatusPacket? decodeStatus(Uint8List bytes) =>
      DeviceStatusPacket.fromBytes(bytes);

  LivePacket? decodeLive(Uint8List bytes) => LivePacket.fromBytes(bytes);

  RiskPacket? decodeRisk(Uint8List bytes) => RiskPacket.fromBytes(bytes);

  SettingsPacket? decodeSettings(Uint8List bytes) =>
      SettingsPacket.fromBytes(bytes);
}
