import 'package:flutter/material.dart';
import 'package:provider/provider.dart';

import '../services/telemetry_service.dart';
import '../widgets/connection_banner.dart';

class ScanScreen extends StatefulWidget {
  const ScanScreen({super.key});

  @override
  State<ScanScreen> createState() => _ScanScreenState();
}

class _ScanScreenState extends State<ScanScreen> {
  @override
  void initState() {
    super.initState();
    WidgetsBinding.instance.addPostFrameCallback((_) {
      context.read<TelemetryService>().startScan();
    });
  }

  @override
  Widget build(BuildContext context) {
    final service = context.watch<TelemetryService>();
    final devices = service.scannedDevices;

    return ListView(
      padding: const EdgeInsets.all(12),
      children: [
        const ConnectionBanner(),
        Row(
          children: [
            ElevatedButton.icon(
              onPressed: service.isScanning
                  ? service.stopScan
                  : service.startScan,
              icon: Icon(service.isScanning ? Icons.stop : Icons.search),
              label: Text(service.isScanning ? 'Stop Scan' : 'Start Scan'),
            ),
            const SizedBox(width: 8),
            OutlinedButton.icon(
              onPressed: service.refreshCharacteristicReads,
              icon: const Icon(Icons.refresh),
              label: const Text('Read Cached'),
            ),
          ],
        ),
        if (service.errorMessage != null) ...[
          const SizedBox(height: 8),
          Text(
            service.errorMessage!,
            style: TextStyle(color: Theme.of(context).colorScheme.error),
          ),
        ],
        const SizedBox(height: 12),
        Text(
          'Discovered Devices',
          style: Theme.of(context).textTheme.titleMedium,
        ),
        const SizedBox(height: 8),
        if (devices.isEmpty)
          const Card(
            child: Padding(
              padding: EdgeInsets.all(12),
              child: Text('No device found yet. Scan for HeatStrokeWearable.'),
            ),
          )
        else
          ...devices.map(
            (device) => Card(
              child: ListTile(
                title: Text(device.name.isEmpty ? '(Unnamed)' : device.name),
                subtitle: Text('ID: ${device.id}\nRSSI: ${device.rssi} dBm'),
                isThreeLine: true,
                trailing: ElevatedButton(
                  onPressed: service.deviceManager.phase.name == 'connecting'
                      ? null
                      : () => service.connect(device),
                  child: const Text('Connect'),
                ),
              ),
            ),
          ),
      ],
    );
  }
}
