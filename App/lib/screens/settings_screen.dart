import 'package:flutter/material.dart';
import 'package:provider/provider.dart';

import '../models/settings_packet.dart';
import '../services/telemetry_service.dart';
import '../widgets/connection_banner.dart';

class SettingsScreen extends StatefulWidget {
  const SettingsScreen({super.key});

  @override
  State<SettingsScreen> createState() => _SettingsScreenState();
}

class _SettingsScreenState extends State<SettingsScreen> {
  final TextEditingController _notifyController = TextEditingController();
  final TextEditingController _offsetController = TextEditingController();

  bool _buzzerEnabled = true;
  int _debugLevel = 3;
  String? _settingsSnapshotKey;

  @override
  void dispose() {
    _notifyController.dispose();
    _offsetController.dispose();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    final service = context.watch<TelemetryService>();
    final settings = service.settingsPacket;

    if (settings != null) {
      final key =
          '${settings.notifyIntervalMs}|${settings.debugLogLevel}|'
          '${settings.buzzerEnabled}|${settings.bodyTempOffsetCX100}';
      if (_settingsSnapshotKey != key) {
        _settingsSnapshotKey = key;
        _buzzerEnabled = settings.buzzerEnabled;
        _debugLevel = settings.debugLogLevel;
        _notifyController.text = settings.notifyIntervalMs.toString();
        _offsetController.text = settings.bodyTempOffsetC.toStringAsFixed(2);
      }
    }

    return ListView(
      padding: const EdgeInsets.all(12),
      children: [
        const ConnectionBanner(),
        Row(
          children: [
            ElevatedButton.icon(
              onPressed: service.readSettings,
              icon: const Icon(Icons.download),
              label: const Text('Read Settings'),
            ),
            const SizedBox(width: 8),
            ElevatedButton.icon(
              onPressed: _buildWriteAction(service),
              icon: const Icon(Icons.save),
              label: const Text('Write Settings'),
            ),
          ],
        ),
        const SizedBox(height: 12),
        Card(
          child: Padding(
            padding: const EdgeInsets.all(12),
            child: Column(
              crossAxisAlignment: CrossAxisAlignment.start,
              children: [
                Text(
                  'Device Settings',
                  style: Theme.of(context).textTheme.titleMedium,
                ),
                const SizedBox(height: 8),
                SwitchListTile(
                  contentPadding: EdgeInsets.zero,
                  title: const Text('Buzzer Enabled'),
                  value: _buzzerEnabled,
                  onChanged: (value) => setState(() => _buzzerEnabled = value),
                ),
                DropdownButtonFormField<int>(
                  value: _debugLevel,
                  decoration: const InputDecoration(
                    labelText: 'Debug Log Level',
                  ),
                  items: List.generate(
                    6,
                    (i) => DropdownMenuItem<int>(value: i, child: Text('$i')),
                  ),
                  onChanged: (value) {
                    if (value != null) {
                      setState(() => _debugLevel = value);
                    }
                  },
                ),
                const SizedBox(height: 10),
                TextField(
                  controller: _notifyController,
                  keyboardType: TextInputType.number,
                  decoration: const InputDecoration(
                    labelText: 'notify_interval_ms (200..10000)',
                  ),
                ),
                const SizedBox(height: 10),
                TextField(
                  controller: _offsetController,
                  keyboardType: const TextInputType.numberWithOptions(
                    decimal: true,
                  ),
                  decoration: const InputDecoration(
                    labelText: 'body_temp_offset_c',
                  ),
                ),
              ],
            ),
          ),
        ),
        Card(
          child: Padding(
            padding: const EdgeInsets.all(12),
            child: Column(
              crossAxisAlignment: CrossAxisAlignment.start,
              children: [
                Text(
                  'Commands',
                  style: Theme.of(context).textTheme.titleMedium,
                ),
                const SizedBox(height: 8),
                Wrap(
                  spacing: 8,
                  runSpacing: 8,
                  children: [
                    ElevatedButton(
                      onPressed: service.sendForceNotify,
                      child: const Text('FORCE_NOTIFY'),
                    ),
                    ElevatedButton(
                      onPressed: service.sendResetCounters,
                      child: const Text('RESET_COUNTERS'),
                    ),
                    OutlinedButton(
                      onPressed: service.sendNop,
                      child: const Text('NOP'),
                    ),
                  ],
                ),
              ],
            ),
          ),
        ),
      ],
    );
  }

  VoidCallback? _buildWriteAction(TelemetryService service) {
    if (!service.isConnected) {
      return null;
    }
    return () async {
      final notify = int.tryParse(_notifyController.text.trim());
      final offsetC = double.tryParse(_offsetController.text.trim());

      if (notify == null || offsetC == null) {
        if (!mounted) {
          return;
        }
        ScaffoldMessenger.of(context).showSnackBar(
          const SnackBar(content: Text('Invalid settings values.')),
        );
        return;
      }

      final packet = SettingsPacket(
        version: SettingsPacket.versionCurrent,
        buzzerEnabled: _buzzerEnabled,
        debugLogLevel: _debugLevel,
        reserved: 0,
        notifyIntervalMs: notify.clamp(
          SettingsPacket.minNotifyIntervalMs,
          SettingsPacket.maxNotifyIntervalMs,
        ),
        bodyTempOffsetCX100: (offsetC * 100).round(),
      );
      await service.writeSettings(packet);
      if (!mounted) {
        return;
      }
      ScaffoldMessenger.of(
        context,
      ).showSnackBar(const SnackBar(content: Text('Settings written')));
    };
  }
}
