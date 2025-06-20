class SensorData {
  final double distance;      // Distance in cm from HC-SR04
  final double temperature;   // Temperature in °C from BMP280
  final double pressure;      // Pressure in hPa from BMP280
  final DateTime timestamp;

  SensorData({
    required this.distance,
    required this.temperature,
    required this.pressure,
    DateTime? timestamp,
  }) : timestamp = timestamp ?? DateTime.now();

  factory SensorData.fromJson(Map<String, dynamic> json) {
    return SensorData(
      distance: (json['distance'] as num?)?.toDouble() ?? 0.0,
      temperature: (json['temperature'] as num?)?.toDouble() ?? 0.0,
      pressure: (json['pressure'] as num?)?.toDouble() ?? 0.0,
    );
  }

  Map<String, dynamic> toJson() {
    return {
      'distance': distance,
      'temperature': temperature,
      'pressure': pressure,
      'timestamp': timestamp.toIso8601String(),
    };
  }

  @override
  String toString() {
    return 'SensorData{distance: ${distance.toStringAsFixed(1)}cm, '
           'temperature: ${temperature.toStringAsFixed(1)}°C, '
           'pressure: ${pressure.toStringAsFixed(1)}hPa}';
  }
}
