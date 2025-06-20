class BluetoothDevice {
  final String name;
  final String address;

  BluetoothDevice({
    required this.name,
    required this.address,
  });

  factory BluetoothDevice.fromFlutterBluetoothSerial(dynamic device) {
    return BluetoothDevice(
      name: device.name ?? 'Unknown Device',
      address: device.address,
    );
  }
}
