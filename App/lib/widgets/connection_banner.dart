import 'package:flutter/material.dart';
import 'package:provider/provider.dart';

import '../ble/ble_device_manager.dart';
import '../services/telemetry_service.dart';

class ConnectionBanner extends StatelessWidget {
  const ConnectionBanner({super.key});

  @override
  Widget build(BuildContext context) {
    final service = context.watch<TelemetryService>();
    final phase = service.deviceManager.phase;

    Color color;
    switch (phase) {
      case DeviceConnectionPhase.connected:
        color = Colors.green.shade700;
        break;
      case DeviceConnectionPhase.connecting:
        color = Colors.orange.shade700;
        break;
      case DeviceConnectionPhase.disconnected:
        color = Colors.grey.shade700;
        break;
    }

    final title = service.deviceManager.deviceName == null
        ? service.connectionLabel
        : '${service.connectionLabel}: ${service.deviceManager.deviceName}';

    return Card(
      color: color.withAlpha(36),
      child: Padding(
        padding: const EdgeInsets.all(12),
        child: Row(
          children: [
            Icon(Icons.bluetooth, color: color),
            const SizedBox(width: 10),
            Expanded(child: Text(title)),
            if (service.isConnected)
              TextButton(
                onPressed: () => service.disconnect(clearLastDevice: false),
                child: const Text('Disconnect'),
              ),
            if (!service.isConnected && service.canReconnect)
              TextButton(
                onPressed: service.reconnectLastDevice,
                child: const Text('Reconnect'),
              ),
          ],
        ),
      ),
    );
  }
}
