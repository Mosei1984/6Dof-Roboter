#pragma once
#include <Arduino.h>
#include <AccelStepper.h>
#include "RobotKinematics.h"

extern const unsigned char PROGMEM robotArmBitmap[];

// Struktur für Pin-Konfiguration
struct PinConfig {
  int leftXPin;
  int leftYPin;
  int leftBtnPin;
  int rightXPin;
  int rightYPin;
  int rightBtnPin;
  int stepperPins[6][4];  // 6 Stepper, jeweils [STEP, DIR, ENABLE, LIMIT] Pins
  int errorLedPin;
  int oledSdaPin;
  int oledSclPin;
};

// Joystick-Konfiguration
struct JoystickConfig {
  int deadband;       // Totbereich für Joystick (0-512)
  float sensitivity;  // Empfindlichkeit für Joystick-Bewegungen
};

// Definition der StepperConfig-Struktur
struct StepperConfig {
  float stepsPerDegree;  // Schritte pro Grad
  float maxSpeed;        // Maximale Geschwindigkeit
  float acceleration;    // Beschleunigung
  float homingSpeed;     // Geschwindigkeit beim Homing
  long minPosition;      // Minimale Position in Schritten
  long maxPosition;      // Maximale Position in Schritten
};

// Globale Instanzen
extern PinConfig _pinConfig;
extern JoystickConfig _joystickConfig;
extern StepperConfig _stepperConfig[6];
extern AccelStepper* _steppers[6];

// Globale Roboterkonfiguration
extern RobotConfig g_robotConfig;

// Funktionen für Konfigurationsspeicherung
bool saveConfigToSD();
bool loadConfigFromSD();
bool resetConfigToDefaults();

// Konfigurationsfunktionen
void loadDefaultPinConfig();
void loadDefaultJoystickConfig();
void loadDefaultStepperConfig();
void initializeRobotArm();
