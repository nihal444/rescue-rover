import 'package:flutter/material.dart';
import 'package:flutter_test/flutter_test.dart';
import 'package:gyro_car/main.dart';  // Update to match project name

void main() {
  testWidgets('Smoke test - app builds and runs', (WidgetTester tester) async {
    // Build our app and trigger a frame.
    await tester.pumpWidget(const MyApp());

    // Verify app starts properly
    expect(find.byType(MaterialApp), findsOneWidget);
  });
}
