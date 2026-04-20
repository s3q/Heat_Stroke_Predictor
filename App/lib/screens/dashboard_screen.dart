import 'package:flutter/material.dart';
import 'package:provider/provider.dart';

import '../models/enums.dart';
import '../services/telemetry_service.dart';
import '../widgets/connection_banner.dart';
import '../widgets/risk_card.dart';
import '../widgets/telemetry_card.dart';

class DashboardScreen extends StatelessWidget {
  const DashboardScreen({super.key});

  @override
  Widget build(BuildContext context) {
    final service = context.watch<TelemetryService>();
    final live = service.livePacket;
    final status = service.statusPacket;
    final risk = service.riskPacket;

    return ListView(
      padding: const EdgeInsets.all(12),
      children: [
        const ConnectionBanner(),
        Row(
          children: [
            ElevatedButton.icon(
              onPressed: service.refreshCharacteristicReads,
              icon: const Icon(Icons.refresh),
              label: const Text('Manual Read'),
            ),
            const SizedBox(width: 8),
            ElevatedButton.icon(
              onPressed: service.sendForceNotify,
              icon: const Icon(Icons.notifications_active),
              label: const Text('FORCE_NOTIFY'),
            ),
          ],
        ),
        const SizedBox(height: 10),
        TelemetryCard(
          title: 'Body Temperature',
          fields: [
            TelemetryField(
              label: 'Estimated Body Temp',
              value: _flaggedDouble(
                live?.estimatedBodyTempC,
                live?.validityFlags,
                ValidityFlags.estimatedBodyTemp,
                '°C',
              ),
            ),
            TelemetryField(
              label: 'Raw Body Temp',
              value: _flaggedDouble(
                live?.rawBodyTempC,
                live?.validityFlags,
                ValidityFlags.rawBodyTemp,
                '°C',
              ),
            ),
            TelemetryField(
              label: 'Filtered Body Temp',
              value: _flaggedDouble(
                live?.filteredBodyTempC,
                live?.validityFlags,
                ValidityFlags.filteredBodyTemp,
                '°C',
              ),
            ),
          ],
        ),
        TelemetryCard(
          title: 'Environment',
          fields: [
            TelemetryField(
              label: 'Ambient Temp',
              value: _flaggedDouble(
                live?.ambientTempC,
                live?.validityFlags,
                ValidityFlags.ambientTemp,
                '°C',
              ),
            ),
            TelemetryField(
              label: 'Humidity',
              value: _flaggedDouble(
                live?.humidityPercent,
                live?.validityFlags,
                ValidityFlags.humidity,
                '%',
              ),
            ),
            TelemetryField(
              label: 'Pressure',
              value: _flaggedDouble(
                live?.pressureHpa,
                live?.validityFlags,
                ValidityFlags.pressure,
                'hPa',
              ),
            ),
            TelemetryField(
              label: 'Heat Index',
              value: _flaggedDouble(
                live?.heatIndexC,
                live?.validityFlags,
                ValidityFlags.heatIndex,
                '°C',
              ),
            ),
          ],
        ),
        TelemetryCard(
          title: 'Cardio / PPG',
          fields: [
            TelemetryField(
              label: 'Heart Rate',
              value: _flaggedDouble(
                live?.heartRateBpm,
                live?.validityFlags,
                ValidityFlags.heartRate,
                'bpm',
              ),
            ),
            TelemetryField(
              label: 'SpO2',
              value: _flaggedDouble(
                live?.spo2Percent,
                live?.validityFlags,
                ValidityFlags.spo2,
                '%',
              ),
            ),
            TelemetryField(
              label: 'HR Confidence',
              value: live?.heartRateConfidencePct == null
                  ? 'NA'
                  : '${live!.heartRateConfidencePct}%',
            ),
            TelemetryField(
              label: 'SpO2 Confidence',
              value: live?.spo2ConfidencePct == null
                  ? 'NA'
                  : '${live!.spo2ConfidencePct}%',
            ),
            TelemetryField(
              label: 'PPG Contact',
              value: live?.ppgContactState.label ?? 'NA',
            ),
            TelemetryField(
              label: 'HR Invalid Reason',
              value: live?.hrInvalidReason.label ?? 'NA',
            ),
            TelemetryField(
              label: 'SpO2 Invalid Reason',
              value: live?.spo2InvalidReason.label ?? 'NA',
            ),
            TelemetryField(
              label: 'Body Contact Quality',
              value: live == null ? 'NA' : live.bodyContactQuality.toString(),
            ),
          ],
        ),
        RiskCard(packet: risk),
        TelemetryCard(
          title: 'Debug / Device Status',
          fields: [
            TelemetryField(
              label: 'Connected',
              value: status == null ? 'NA' : status.connected.toString(),
            ),
            TelemetryField(
              label: 'MTU',
              value: status == null ? 'NA' : status.mtu.toString(),
            ),
            TelemetryField(
              label: 'Advertising Active',
              value: status == null ? 'NA' : status.advActive.toString(),
            ),
            TelemetryField(
              label: 'Notify Interval',
              value: status == null ? 'NA' : '${status.notifyIntervalMs} ms',
            ),
            TelemetryField(
              label: 'Sample Counter',
              value: live == null ? 'NA' : live.sampleCounter.toString(),
            ),
            TelemetryField(
              label: 'Uptime',
              value: status == null ? 'NA' : '${status.uptimeMs} ms',
            ),
            TelemetryField(
              label: 'Last Live Packet',
              value: _formatTime(service.lastLivePacketAt),
            ),
            TelemetryField(
              label: 'Last Risk Packet',
              value: _formatTime(service.lastRiskPacketAt),
            ),
            TelemetryField(
              label: 'Validity Flags',
              value: live == null
                  ? 'NA'
                  : '0x${live.validityFlags.toRadixString(16)}',
            ),
          ],
        ),
      ],
    );
  }

  String _flaggedDouble(double? value, int? flags, int flag, String unit) {
    if (value == null || flags == null || !flags.hasFlag(flag)) {
      return 'NA';
    }
    return '${value.toStringAsFixed(2)} $unit';
  }

  String _formatTime(DateTime? time) {
    if (time == null) {
      return 'NA';
    }
    final h = time.hour.toString().padLeft(2, '0');
    final m = time.minute.toString().padLeft(2, '0');
    final s = time.second.toString().padLeft(2, '0');
    return '$h:$m:$s';
  }
}
