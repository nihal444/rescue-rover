import 'package:flutter/foundation.dart';
import 'dart:convert';
import '../services/joystick_service.dart';
import '../services/connection_service.dart';
import '../models/control_data.dart';
import '../models/sensor_data.dart';
import '../models/bluetooth_device.dart';

class CarControlProvider with ChangeNotifier {
  final JoystickService _joystickService = JoystickService();
  final ConnectionService _connectionService = ConnectionService();
  bool get isControlActive => _joystickService.isActive;
  double get sensitivity => _joystickService.sensitivity;
  ConnectionType get connectionType => _connectionService.connectionType;
  ConnectionStatus get connectionStatus => _connectionService.connectionStatus;
  String get errorMessage => _connectionService.errorMessage;
  String get targetAddress => _connectionService.targetAddress;
  
  ControlData? _lastControlData;
  ControlData? get lastControlData => _lastControlData;

  // Sensor data properties
  SensorData? _lastSensorData;
  SensorData? get lastSensorData => _lastSensorData;

  void toggleControl() {
    if (isControlActive) {
      stopControl();
    } else {
      startControl();
    }
  }
  void startControl() {
    if (_connectionService.connectionStatus != ConnectionStatus.connected) {
      // Can't start if not connected
      return;
    }

    _joystickService.start(onDataReceived: _handleJoystickData);
    notifyListeners();
  }

  void stopControl() {
    _joystickService.stop();
    notifyListeners();
  }

  void setSensitivity(double value) {
    _joystickService.setSensitivity(value);
    notifyListeners();
  }
  void updateJoystickPosition(double x, double y) {
    _joystickService.updateJoystickPosition(x, y);
  }

  Future<bool> connectBluetooth(String address) async {
    _connectionService.setDataCallback(handleSensorData);
    final result = await _connectionService.connectBluetooth(address);
    notifyListeners();
    return result;
  }

  Future<void> disconnect() async {
    await _connectionService.disconnect();
    notifyListeners();
  }

  Future<List<BluetoothDevice>> scanBluetoothDevices() {
    return _connectionService.scanBluetoothDevices();
  }

  // Handle sensor data received from ESP32
  void handleSensorData(String jsonData) {
    try {
      final Map<String, dynamic> data = json.decode(jsonData);
      
      // Check if this is sensor data (contains distance, temperature, pressure)
      if (data.containsKey('distance') && 
          data.containsKey('temperature') && 
          data.containsKey('pressure')) {
        _lastSensorData = SensorData.fromJson(data);
        notifyListeners();
        
        if (kDebugMode) {
          print('Sensor data received: $_lastSensorData');
        }
      }
    } catch (e) {
      if (kDebugMode) {
        print('Error parsing sensor data: $e');
      }
    }
  }
  void _handleJoystickData(ControlData data) {
    _lastControlData = data;
    _connectionService.sendControlData(data);
    notifyListeners();
  }

  @override
  void dispose() {
    _joystickService.dispose();
    _connectionService.disconnect();
    super.dispose();
  }
}
