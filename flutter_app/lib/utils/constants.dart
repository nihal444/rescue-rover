import 'package:flutter/material.dart';

class AppColors {
  static const Color primaryColor = Color(0xFF333333);  // Dark gray
  static const Color secondaryColor = Color(0xFFEEEEEE);  // Light gray
  static const Color accentColor = Color(0xFF008080);  // Teal
  static const Color errorColor = Color(0xFFD32F2F);  // Red for errors
  static const Color surfaceColor = Color(0xFFFFFFFF);  // White surface
  static const Color textColor = Color(0xFF212121);  // Dark text
}

class AppTheme {
  static ThemeData getTheme() {
    return ThemeData(
      primaryColor: AppColors.primaryColor,
      scaffoldBackgroundColor: AppColors.secondaryColor,
      colorScheme: const ColorScheme.light( // Add const
        primary: AppColors.primaryColor,
        secondary: AppColors.accentColor,
        error: AppColors.errorColor,
      ),
      appBarTheme: const AppBarTheme(
        backgroundColor: AppColors.primaryColor,
        foregroundColor: AppColors.secondaryColor,
      ),
      sliderTheme: SliderThemeData(
        activeTrackColor: AppColors.accentColor,
        thumbColor: AppColors.accentColor,
        inactiveTrackColor: AppColors.primaryColor.withAlpha((0.3 * 255).round()), // Replace withOpacity
      ),
      elevatedButtonTheme: ElevatedButtonThemeData(
        style: ElevatedButton.styleFrom(
          backgroundColor: AppColors.accentColor,
          foregroundColor: AppColors.secondaryColor,
        ),
      ),
    );
  }
}
