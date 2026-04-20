import 'package:flutter/material.dart';

import '../models/enums.dart';
import '../models/risk_packet.dart';

class RiskCard extends StatelessWidget {
  const RiskCard({super.key, required this.packet});

  final RiskPacket? packet;

  @override
  Widget build(BuildContext context) {
    if (packet == null) {
      return const Card(
        child: Padding(
          padding: EdgeInsets.all(12),
          child: Text('Risk packet: NA'),
        ),
      );
    }

    final color = _riskColor(packet!.riskCategory);

    return Card(
      color: color.withAlpha(30),
      child: Padding(
        padding: const EdgeInsets.all(12),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            Row(
              children: [
                Icon(Icons.warning_amber_rounded, color: color),
                const SizedBox(width: 8),
                Text(
                  'Risk: ${packet!.riskCategory.label}',
                  style: Theme.of(context).textTheme.titleMedium,
                ),
              ],
            ),
            const SizedBox(height: 8),
            _row('risk_heart_rate', packet!.riskHeartRate.toString()),
            _row('risk_body_temp', packet!.riskBodyTemp.toString()),
            _row('risk_heat_index', packet!.riskHeatIndex.toString()),
            _row('total_risk', packet!.totalRisk.toString()),
            _row('buzzer_state', packet!.buzzerState.label),
          ],
        ),
      ),
    );
  }

  Widget _row(String label, String value) {
    return Padding(
      padding: const EdgeInsets.symmetric(vertical: 2),
      child: Row(
        children: [
          Expanded(child: Text(label)),
          Text(value, style: const TextStyle(fontWeight: FontWeight.w600)),
        ],
      ),
    );
  }

  Color _riskColor(RiskCategory category) {
    switch (category) {
      case RiskCategory.normal:
        return Colors.green;
      case RiskCategory.moderate:
        return Colors.orange;
      case RiskCategory.high:
        return Colors.deepOrange;
      case RiskCategory.veryHigh:
        return Colors.red;
      case RiskCategory.unknown:
        return Colors.grey;
    }
  }
}
