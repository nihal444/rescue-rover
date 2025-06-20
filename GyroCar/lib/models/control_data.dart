class ControlData {
  final double x;
  final double y;
  final double z;
  final double sensitivity;

  ControlData({
    required this.x,
    required this.y,
    required this.z,
    required this.sensitivity,
  });

  Map<String, dynamic> toJson() {
    return {
      'x': x * sensitivity,
      'y': y * sensitivity,
      'z': z * sensitivity,
    };
  }

  @override
  String toString() {
    return 'ControlData(x: ${x * sensitivity}, y: ${y * sensitivity}, z: ${z * sensitivity})';
  }
}
