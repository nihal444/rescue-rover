import 'dart:async';
import '../models/control_data.dart';

class JoystickService {
  Function(ControlData)? _onDataReceived;
  bool _isActive = false;
  double _sensitivity = 1.0;
  Timer? _sendTimer;
  
  // Current joystick position
  double _currentX = 0.0;
  double _currentY = 0.0;

  bool get isActive => _isActive;
  double get sensitivity => _sensitivity;

  void setSensitivity(double value) {
    _sensitivity = value;
  }

  void start({required Function(ControlData) onDataReceived}) {
    if (_isActive) return;

    _onDataReceived = onDataReceived;
    _isActive = true;
    
    // Start sending data at regular intervals (similar to gyroscope frequency)
    _sendTimer = Timer.periodic(const Duration(milliseconds: 50), (timer) {
      _sendCurrentPosition();
    });
  }

  void stop() {
    _sendTimer?.cancel();
    _sendTimer = null;
    _onDataReceived = null;
    _isActive = false;
    
    // Reset position to center when stopped
    _currentX = 0.0;
    _currentY = 0.0;
  }

  void updateJoystickPosition(double x, double y) {
    _currentX = x;
    _currentY = y;
  }

  void _sendCurrentPosition() {
    if (_onDataReceived != null && _isActive) {
      final controlData = ControlData(
        x: _currentX,
        y: _currentY,
        z: 0.0, // Not needed for joystick control
        sensitivity: _sensitivity,
      );
      _onDataReceived?.call(controlData);
    }
  }

  void dispose() {
    stop();
  }
}
