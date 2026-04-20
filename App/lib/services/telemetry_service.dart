import 'dart:async';
import 'dart:io';

import 'package:flutter/foundation.dart';
import 'package:flutter_reactive_ble/flutter_reactive_ble.dart';
import 'package:permission_handler/permission_handler.dart';

import '../ble/ble_constants.dart';
import '../ble/ble_device_manager.dart';
import '../ble/ble_repository.dart';
import '../models/command_packet.dart';
import '../models/device_status.dart';
import '../models/live_packet.dart';
import '../models/risk_packet.dart';
import '../models/settings_packet.dart';
import 'packet_decoder.dart';

class TelemetryService extends ChangeNotifier {
  TelemetryService({BleRepository? bleRepository, PacketDecoder? decoder})
    : _bleRepository = bleRepository ?? BleRepository(),
      _decoder = decoder ?? const PacketDecoder();

  final BleRepository _bleRepository;
  final PacketDecoder _decoder;
  final BleDeviceManager deviceManager = BleDeviceManager();

  final Map<String, DiscoveredDevice> _scanCache = {};

  StreamSubscription<DiscoveredDevice>? _scanSub;
  StreamSubscription<ConnectionStateUpdate>? _connectionSub;
  StreamSubscription<List<int>>? _statusSub;
  StreamSubscription<List<int>>? _liveSub;
  StreamSubscription<List<int>>? _riskSub;

  WearableCharacteristics? _characteristics;
  String? _lastDeviceId;
  String? _lastDeviceName;

  bool isScanning = false;
  String? errorMessage;
  DeviceStatusPacket? statusPacket;
  LivePacket? livePacket;
  RiskPacket? riskPacket;
  SettingsPacket? settingsPacket;
  DateTime? lastStatusPacketAt;
  DateTime? lastLivePacketAt;
  DateTime? lastRiskPacketAt;

  List<DiscoveredDevice> get scannedDevices {
    final devices = _scanCache.values.toList();
    devices.sort((a, b) => b.rssi.compareTo(a.rssi));
    return devices;
  }

  bool get isConnected => deviceManager.isConnected;
  bool get canReconnect =>
      _lastDeviceId != null &&
      !isConnected &&
      deviceManager.phase != DeviceConnectionPhase.connecting;
  String? get lastDeviceName => _lastDeviceName;

  String get connectionLabel {
    switch (deviceManager.phase) {
      case DeviceConnectionPhase.connected:
        return 'Connected';
      case DeviceConnectionPhase.connecting:
        return 'Connecting';
      case DeviceConnectionPhase.disconnected:
        return 'Disconnected';
    }
  }

  Future<bool> _ensurePermissions() async {
    if (kIsWeb) {
      return true;
    }
    if (Platform.isAndroid) {
      final statuses = await <Permission>[
        Permission.bluetoothScan,
        Permission.bluetoothConnect,
        Permission.locationWhenInUse,
      ].request();
      return statuses.values.every(
        (status) => status.isGranted || status.isLimited,
      );
    }
    if (Platform.isIOS) {
      final status = await Permission.bluetooth.request();
      return status.isGranted || status.isLimited;
    }
    return true;
  }

  Future<void> startScan() async {
    errorMessage = null;
    final granted = await _ensurePermissions();
    if (!granted) {
      errorMessage =
          'Bluetooth permissions denied. Enable permissions in system settings.';
      notifyListeners();
      return;
    }

    await _scanSub?.cancel();
    _scanCache.clear();
    isScanning = true;
    notifyListeners();

    _scanSub = _bleRepository.scanDevices().listen(
      (device) {
        if (_matchesDevice(device)) {
          _scanCache[device.id] = device;
          notifyListeners();
        }
      },
      onError: (Object error) {
        isScanning = false;
        errorMessage = 'Scan error: $error';
        notifyListeners();
      },
    );
  }

  bool _matchesDevice(DiscoveredDevice device) {
    if (device.name == BleConstants.deviceName) {
      return true;
    }
    return device.name.contains(BleConstants.deviceName);
  }

  Future<void> stopScan() async {
    await _scanSub?.cancel();
    _scanSub = null;
    isScanning = false;
    notifyListeners();
  }

  Future<void> connect(DiscoveredDevice device) async {
    await stopScan();
    await disconnect(clearLastDevice: false);
    errorMessage = null;

    _lastDeviceId = device.id;
    _lastDeviceName = device.name;
    deviceManager.beginConnect(id: device.id, name: device.name);
    notifyListeners();

    _connectionSub = _bleRepository
        .connectToDevice(device.id)
        .listen(
          (update) async {
            deviceManager.applyConnectionUpdate(update);
            notifyListeners();

            if (update.connectionState == DeviceConnectionState.connected) {
              await _onConnected(device.id);
            } else if (update.connectionState ==
                DeviceConnectionState.disconnected) {
              await _clearPacketStreams();
              _characteristics = null;
              notifyListeners();
            }
          },
          onError: (Object error) async {
            errorMessage = 'Connection error: $error';
            deviceManager.disconnect();
            await _clearPacketStreams();
            _characteristics = null;
            notifyListeners();
          },
        );
  }

  Future<void> reconnectLastDevice() async {
    if (_lastDeviceId == null) {
      return;
    }
    await stopScan();
    await disconnect(clearLastDevice: false);
    errorMessage = null;
    deviceManager.beginConnect(
      id: _lastDeviceId!,
      name: _lastDeviceName ?? BleConstants.deviceName,
    );
    notifyListeners();

    _connectionSub = _bleRepository
        .connectToDevice(_lastDeviceId!)
        .listen(
          (update) async {
            deviceManager.applyConnectionUpdate(update);
            notifyListeners();

            if (update.connectionState == DeviceConnectionState.connected) {
              await _onConnected(_lastDeviceId!);
            } else if (update.connectionState ==
                DeviceConnectionState.disconnected) {
              await _clearPacketStreams();
              _characteristics = null;
              notifyListeners();
            }
          },
          onError: (Object error) async {
            errorMessage = 'Reconnect error: $error';
            deviceManager.disconnect();
            await _clearPacketStreams();
            _characteristics = null;
            notifyListeners();
          },
        );
  }

  Future<void> _onConnected(String deviceId) async {
    _characteristics = WearableCharacteristics(deviceId);
    await _subscribePackets();
    await refreshCharacteristicReads();
    await readSettings();
  }

  Future<void> _subscribePackets() async {
    if (_characteristics == null) {
      return;
    }

    await _statusSub?.cancel();
    _statusSub = _bleRepository
        .subscribe(_characteristics!.status)
        .listen(
          (payload) {
            final decoded = _decoder.decodeStatus(Uint8List.fromList(payload));
            if (decoded != null) {
              statusPacket = decoded;
              lastStatusPacketAt = DateTime.now();
              notifyListeners();
            }
          },
          onError: (Object error) {
            errorMessage = 'Status notify error: $error';
            notifyListeners();
          },
        );

    await _liveSub?.cancel();
    _liveSub = _bleRepository
        .subscribe(_characteristics!.live)
        .listen(
          (payload) {
            final decoded = _decoder.decodeLive(Uint8List.fromList(payload));
            if (decoded != null) {
              livePacket = decoded;
              lastLivePacketAt = DateTime.now();
              notifyListeners();
            }
          },
          onError: (Object error) {
            errorMessage = 'Live notify error: $error';
            notifyListeners();
          },
        );

    await _riskSub?.cancel();
    _riskSub = _bleRepository
        .subscribe(_characteristics!.risk)
        .listen(
          (payload) {
            final decoded = _decoder.decodeRisk(Uint8List.fromList(payload));
            if (decoded != null) {
              riskPacket = decoded;
              lastRiskPacketAt = DateTime.now();
              notifyListeners();
            }
          },
          onError: (Object error) {
            errorMessage = 'Risk notify error: $error';
            notifyListeners();
          },
        );
  }

  Future<void> refreshCharacteristicReads() async {
    if (_characteristics == null || !isConnected) {
      return;
    }
    try {
      final statusBytes = await _bleRepository.read(_characteristics!.status);
      final liveBytes = await _bleRepository.read(_characteristics!.live);
      final riskBytes = await _bleRepository.read(_characteristics!.risk);

      final decodedStatus = _decoder.decodeStatus(statusBytes);
      final decodedLive = _decoder.decodeLive(liveBytes);
      final decodedRisk = _decoder.decodeRisk(riskBytes);

      if (decodedStatus != null) {
        statusPacket = decodedStatus;
        lastStatusPacketAt = DateTime.now();
      }
      if (decodedLive != null) {
        livePacket = decodedLive;
        lastLivePacketAt = DateTime.now();
      }
      if (decodedRisk != null) {
        riskPacket = decodedRisk;
        lastRiskPacketAt = DateTime.now();
      }
      notifyListeners();
    } catch (error) {
      errorMessage = 'Read error: $error';
      notifyListeners();
    }
  }

  Future<void> readSettings() async {
    if (_characteristics == null || !isConnected) {
      return;
    }
    try {
      final bytes = await _bleRepository.read(_characteristics!.settings);
      final decoded = _decoder.decodeSettings(bytes);
      if (decoded != null) {
        settingsPacket = decoded;
        notifyListeners();
      }
    } catch (error) {
      errorMessage = 'Settings read error: $error';
      notifyListeners();
    }
  }

  Future<void> writeSettings(SettingsPacket settings) async {
    if (_characteristics == null || !isConnected) {
      return;
    }
    try {
      await _bleRepository.writeWithResponse(
        _characteristics!.settings,
        settings.toBytes(),
      );
      await readSettings();
    } catch (error) {
      errorMessage = 'Settings write error: $error';
      notifyListeners();
    }
  }

  Future<void> sendForceNotify() => _sendCommand(CommandPacket.forceNotify());

  Future<void> sendResetCounters() =>
      _sendCommand(CommandPacket.resetCounters());

  Future<void> sendNop() => _sendCommand(CommandPacket.nop());

  Future<void> _sendCommand(CommandPacket command) async {
    if (_characteristics == null || !isConnected) {
      return;
    }
    try {
      await _bleRepository.writeWithResponse(
        _characteristics!.command,
        command.toBytes(),
      );
    } catch (error) {
      errorMessage = 'Command write error: $error';
      notifyListeners();
    }
  }

  Future<void> disconnect({bool clearLastDevice = true}) async {
    await _connectionSub?.cancel();
    _connectionSub = null;
    await _clearPacketStreams();
    _characteristics = null;
    deviceManager.disconnect();
    if (clearLastDevice) {
      _lastDeviceId = null;
      _lastDeviceName = null;
    }
    notifyListeners();
  }

  Future<void> _clearPacketStreams() async {
    await _statusSub?.cancel();
    await _liveSub?.cancel();
    await _riskSub?.cancel();
    _statusSub = null;
    _liveSub = null;
    _riskSub = null;
  }

  @override
  Future<void> dispose() async {
    await stopScan();
    await disconnect();
    super.dispose();
  }
}
