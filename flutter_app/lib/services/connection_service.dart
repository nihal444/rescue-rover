import 'dart:async';
import 'dart:convert';
import 'dart:typed_data';
import 'package:flutter_bluetooth_serial/flutter_bluetooth_serial.dart' as fbs;
import '../models/control_data.dart';
import '../models/bluetooth_device.dart';
import '../utils/logger.dart';

enum ConnectionType { bluetooth, none }
enum ConnectionStatus { connected, disconnected, connecting, error }

class ConnectionService {
  ConnectionType _connectionType = ConnectionType.none;
  ConnectionStatus _connectionStatus = ConnectionStatus.disconnected;
  fbs.BluetoothConnection? _bluetoothConnection;
  String _errorMessage = '';
  String _targetAddress = '';
  
  // Data listening
  StreamSubscription<Uint8List>? _dataSubscription;
  Function(String)? _onDataReceived;
  
  // Buffer for incoming data
  String _dataBuffer = '';

  // Getters
  ConnectionType get connectionType => _connectionType;
  ConnectionStatus get connectionStatus => _connectionStatus;
  String get errorMessage => _errorMessage;
  String get targetAddress => _targetAddress;

  // Set data callback
  void setDataCallback(Function(String) callback) {
    _onDataReceived = callback;
  }
  // Connect via Bluetooth to ESP32 running GyroCar firmware
  Future<bool> connectBluetooth(String address) async {
    _connectionType = ConnectionType.bluetooth;
    _connectionStatus = ConnectionStatus.connecting;
    _targetAddress = address;
    _errorMessage = '';
    
    try {
      Logger.log('Connecting to Bluetooth device: $address');
      
      // Connect to the Bluetooth device
      _bluetoothConnection = await fbs.BluetoothConnection.toAddress(address)
          .timeout(const Duration(seconds: 10));
      
      if (_bluetoothConnection != null && _bluetoothConnection!.isConnected) {
        Logger.log('Bluetooth connection established');
        
        // Listen to incoming data from ESP32
        _dataSubscription = _bluetoothConnection!.input?.listen(
          (data) {
            _handleIncomingData(data);
          },
          onError: (error) {
            Logger.log('Bluetooth data error: $error');
            _errorMessage = "Bluetooth data error: ${error.toString()}";
            _connectionStatus = ConnectionStatus.error;
          },
          onDone: () {
            Logger.log('Bluetooth connection closed');
            _connectionStatus = ConnectionStatus.disconnected;
          },
        );
        
        _connectionStatus = ConnectionStatus.connected;
        Logger.log('Successfully connected to GyroCar ESP32');
        return true;
      } else {
        _errorMessage = "Failed to establish Bluetooth connection";
        _connectionStatus = ConnectionStatus.error;
        return false;
      }
    } catch (e) {
      Logger.log('Bluetooth connection error: $e');
      _errorMessage = e.toString();
      _connectionStatus = ConnectionStatus.error;
      return false;
    }
  }

  // Handle incoming data from ESP32
  void _handleIncomingData(Uint8List data) {
    try {
      // Convert bytes to string and add to buffer
      final String incoming = utf8.decode(data);
      _dataBuffer += incoming;
      
      // Process complete JSON messages (separated by newlines)
      while (_dataBuffer.contains('\n')) {
        int newlineIndex = _dataBuffer.indexOf('\n');
        String jsonLine = _dataBuffer.substring(0, newlineIndex).trim();
        _dataBuffer = _dataBuffer.substring(newlineIndex + 1);
        
        if (jsonLine.isNotEmpty) {
          Logger.log('Received from ESP32: $jsonLine');
          
          // Try to parse as JSON to validate
          try {
            final jsonData = json.decode(jsonLine);
            if (jsonData is Map<String, dynamic>) {
              // Valid JSON sensor data, forward to app
              _onDataReceived?.call(jsonLine);
            }
          } catch (parseError) {
            Logger.log('Invalid JSON received: $jsonLine');
          }
        }
      }
    } catch (e) {
      Logger.log('Error processing incoming data: $e');
    }
  }
  // Disconnect from ESP32
  Future<void> disconnect() async {
    Logger.log('Disconnecting from ESP32...');
    
    // Cancel data subscription
    await _dataSubscription?.cancel();
    _dataSubscription = null;
    
    // Close Bluetooth connection
    if (_bluetoothConnection != null) {
      await _bluetoothConnection!.close();
      _bluetoothConnection = null;
    }
    
    // Clear buffer
    _dataBuffer = '';
    
    _connectionStatus = ConnectionStatus.disconnected;
    _connectionType = ConnectionType.none;
    _targetAddress = '';
    
    Logger.log('Disconnected from ESP32');
  }

  // Send control data to ESP32
  Future<bool> sendControlData(ControlData data) async {
    if (_connectionStatus != ConnectionStatus.connected || _bluetoothConnection == null) {
      return false;
    }

    try {
      // Create JSON string matching ESP32 expected format
      final jsonData = jsonEncode({
        'x': data.x * data.sensitivity,
        'y': data.y * data.sensitivity,
      });
      
      // Send data to ESP32 with newline terminator
      _bluetoothConnection!.output.add(Uint8List.fromList(utf8.encode("$jsonData\n")));
      await _bluetoothConnection!.output.allSent.timeout(
        const Duration(seconds: 2),
        onTimeout: () {
          throw TimeoutException('Bluetooth send timeout');
        },
      );
      
      Logger.log('Sent to ESP32: $jsonData');
      return true;
    } catch (e) {
      Logger.log('Error sending data to ESP32: $e');
      _errorMessage = "Data sending error: ${e.toString()}";
      _connectionStatus = ConnectionStatus.error;
      return false;
    }
  }

  // Scan for available Bluetooth devices
  Future<List<BluetoothDevice>> scanBluetoothDevices() async {
    try {
      Logger.log('Scanning for Bluetooth devices...');
      
      // Get bonded (paired) devices
      final bondedDevices = await fbs.FlutterBluetoothSerial.instance
          .getBondedDevices()
          .timeout(const Duration(seconds: 10));
      
      Logger.log('Found ${bondedDevices.length} bonded devices');
      
      // Convert to our model and filter for ESP32 devices
      final devices = bondedDevices
          .map((device) => BluetoothDevice.fromFlutterBluetoothSerial(device))
          .toList();      // Prioritize devices named "GyroCar" or containing "ESP32"
      devices.sort((a, b) {
        if (a.name == 'GyroCar') return -1;
        if (b.name == 'GyroCar') return 1;
        if (a.name != null && a.name!.contains('ESP32')) return -1;
        if (b.name != null && b.name!.contains('ESP32')) return 1;
        return (a.name ?? '').compareTo(b.name ?? '');
      });
      
      return devices;
    } catch (e) {
      Logger.log('Bluetooth scan error: $e');
      _errorMessage = "Bluetooth scan failed: ${e.toString()}";
      return [];
    }
  }
}
