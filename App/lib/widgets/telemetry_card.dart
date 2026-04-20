import 'package:flutter/material.dart';

class TelemetryField {
  const TelemetryField({required this.label, required this.value});

  final String label;
  final String value;
}

class TelemetryCard extends StatelessWidget {
  const TelemetryCard({super.key, required this.title, required this.fields});

  final String title;
  final List<TelemetryField> fields;

  @override
  Widget build(BuildContext context) {
    return Card(
      child: Padding(
        padding: const EdgeInsets.all(12),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            Text(title, style: Theme.of(context).textTheme.titleMedium),
            const SizedBox(height: 8),
            ...fields.map(
              (field) => Padding(
                padding: const EdgeInsets.symmetric(vertical: 2),
                child: Row(
                  children: [
                    Expanded(child: Text(field.label)),
                    Text(
                      field.value,
                      style: const TextStyle(fontWeight: FontWeight.w600),
                    ),
                  ],
                ),
              ),
            ),
          ],
        ),
      ),
    );
  }
}
