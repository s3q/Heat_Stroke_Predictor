import 'package:flutter_test/flutter_test.dart';
import 'package:heat_stroke_predictor_app/app.dart';

void main() {
  testWidgets(
    'App boots and shows BLE title',
    (WidgetTester tester) async {
      await tester.pumpWidget(const HeatStrokeApp());
      await tester.pumpAndSettle();

      expect(find.text('HeatStrokeWearable BLE'), findsOneWidget);
    },
    skip: true,
  );
}
