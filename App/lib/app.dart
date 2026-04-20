import 'package:flutter/material.dart';
import 'package:provider/provider.dart';

import 'screens/dashboard_screen.dart';
import 'screens/scan_screen.dart';
import 'screens/settings_screen.dart';
import 'services/telemetry_service.dart';

class HeatStrokeApp extends StatelessWidget {
  const HeatStrokeApp({super.key});

  @override
  Widget build(BuildContext context) {
    return ChangeNotifierProvider<TelemetryService>(
      create: (_) => TelemetryService(),
      child: MaterialApp(
        title: 'Heat Stroke Wearable',
        debugShowCheckedModeBanner: false,
        theme: ThemeData(
          colorScheme: ColorScheme.fromSeed(seedColor: Colors.redAccent),
          useMaterial3: true,
        ),
        darkTheme: ThemeData(
          colorScheme: ColorScheme.fromSeed(
            brightness: Brightness.dark,
            seedColor: Colors.redAccent,
          ),
          useMaterial3: true,
        ),
        home: const HomeShell(),
      ),
    );
  }
}

class HomeShell extends StatefulWidget {
  const HomeShell({super.key});

  @override
  State<HomeShell> createState() => _HomeShellState();
}

class _HomeShellState extends State<HomeShell> {
  int _index = 0;

  static const _screens = [ScanScreen(), DashboardScreen(), SettingsScreen()];

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(title: const Text('HeatStrokeWearable BLE')),
      body: IndexedStack(index: _index, children: _screens),
      bottomNavigationBar: NavigationBar(
        selectedIndex: _index,
        onDestinationSelected: (value) => setState(() => _index = value),
        destinations: const [
          NavigationDestination(
            icon: Icon(Icons.bluetooth_searching),
            label: 'Scan',
          ),
          NavigationDestination(
            icon: Icon(Icons.monitor_heart_outlined),
            label: 'Dashboard',
          ),
          NavigationDestination(icon: Icon(Icons.tune), label: 'Settings'),
        ],
      ),
    );
  }
}
