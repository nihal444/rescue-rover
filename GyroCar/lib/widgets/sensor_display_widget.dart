import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import '../providers/car_control_provider.dart';
import '../services/connection_service.dart';

class SensorDisplayWidget extends StatelessWidget {
  const SensorDisplayWidget({super.key});

  @override
  Widget build(BuildContext context) {
    return Consumer<CarControlProvider>(
      builder: (context, provider, child) {
        final sensorData = provider.lastSensorData;
        final isConnected = provider.connectionStatus == ConnectionStatus.connected;
        
        return Row(
          mainAxisAlignment: MainAxisAlignment.spaceEvenly,
          children: [
            _buildParameter(
              value: sensorData?.distance.toStringAsFixed(1) ?? '--',
              label: 'cm',
              isConnected: isConnected,
            ),
            _buildParameter(
              value: sensorData?.temperature.toStringAsFixed(1) ?? '--',
              label: '°C',
              isConnected: isConnected,
            ),
            _buildParameter(
              value: sensorData?.pressure.toStringAsFixed(0) ?? '--',
              label: 'hPa',
              isConnected: isConnected,
            ),
          ],
        );
      },
    );
  }

  Widget _buildParameter({
    required String value, 
    required String label, 
    required bool isConnected
  }) {
    return Column(
      children: [
        Text(
          value,
          style: TextStyle(
            fontSize: 32,
            fontWeight: FontWeight.w700,
            color: isConnected ? Colors.white : const Color(0xFF8E8E93),
            fontFeatures: const [FontFeature.tabularFigures()],
          ),
        ),
        const SizedBox(height: 4),
        Text(
          label,
          style: const TextStyle(
            fontSize: 14,
            color: Color(0xFF8E8E93),
            fontWeight: FontWeight.w500,
          ),
        ),
      ],
    );
  }
}
