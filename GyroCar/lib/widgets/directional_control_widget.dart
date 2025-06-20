import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import '../providers/car_control_provider.dart';

class DirectionalControlWidget extends StatefulWidget {
  const DirectionalControlWidget({super.key});

  @override
  State<DirectionalControlWidget> createState() => _DirectionalControlWidgetState();
}

class _DirectionalControlWidgetState extends State<DirectionalControlWidget> {
  String? _pressedButton;

  @override
  Widget build(BuildContext context) {
    final provider = Provider.of<CarControlProvider>(context, listen: false);

    return Center(
      child: SizedBox(
        width: 200,
        height: 200,
        child: GridView.count(
          crossAxisCount: 3,
          mainAxisSpacing: 8,
          crossAxisSpacing: 8,
          shrinkWrap: true,
          physics: const NeverScrollableScrollPhysics(),
          children: [
            // Empty space
            const SizedBox(),
            // Forward button
            _buildDirectionButton(
              symbol: '↑',
              onPressed: () => provider.updateJoystickPosition(0, -1),
              onReleased: () => provider.updateJoystickPosition(0, 0),
              buttonId: 'forward',
            ),
            // Empty space
            const SizedBox(),
            // Left button
            _buildDirectionButton(
              symbol: '←',
              onPressed: () => provider.updateJoystickPosition(-1, 0),
              onReleased: () => provider.updateJoystickPosition(0, 0),
              buttonId: 'left',
            ),
            // Empty space
            const SizedBox(),
            // Right button
            _buildDirectionButton(
              symbol: '→',
              onPressed: () => provider.updateJoystickPosition(1, 0),
              onReleased: () => provider.updateJoystickPosition(0, 0),
              buttonId: 'right',
            ),
            // Empty space
            const SizedBox(),
            // Backward button
            _buildDirectionButton(
              symbol: '↓',
              onPressed: () => provider.updateJoystickPosition(0, 1),
              onReleased: () => provider.updateJoystickPosition(0, 0),
              buttonId: 'backward',
            ),
            // Empty space
            const SizedBox(),
          ],
        ),
      ),
    );
  }

  Widget _buildDirectionButton({
    required String symbol,
    required VoidCallback onPressed,
    required VoidCallback onReleased,
    required String buttonId,
  }) {
    final isPressed = _pressedButton == buttonId;
    
    return GestureDetector(
      onTapDown: (_) {
        setState(() {
          _pressedButton = buttonId;
        });
        onPressed();
      },
      onTapUp: (_) {
        setState(() {
          _pressedButton = null;
        });
        onReleased();
      },
      onTapCancel: () {
        setState(() {
          _pressedButton = null;
        });
        onReleased();
      },
      child: Container(
        width: 60,
        height: 60,
        decoration: BoxDecoration(
          color: isPressed ? const Color(0xFF007AFF) : const Color(0xFF1C1C1E),
          borderRadius: BorderRadius.circular(12),
          border: Border.all(
            color: isPressed ? const Color(0xFF007AFF) : const Color(0xFF38383A),
            width: 1,
          ),
        ),
        child: Center(
          child: Text(
            symbol,
            style: const TextStyle(
              fontSize: 20,
              fontWeight: FontWeight.w600,
              color: Colors.white,
            ),
          ),
        ),
      ),
    );
  }
}
