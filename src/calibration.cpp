#include "calibration.h"
#include "joystick.h"
#include "JoystickSystem.h"  // Add this include

bool Calibration::_isCalibrating = false;

void Calibration::calibrateJoysticks() {
    // Get joystick instances from JoystickSystem instead of using globals
    JoystickSystem::leftJoystick->calibrate();
    JoystickSystem::rightJoystick->calibrate();
}

// Implementierung für den Setter
void Calibration::setCalibrating(bool calibrating) {
    _isCalibrating = calibrating;
}

// Die startFullCalibration Methode kann nun vereinfacht werden
void Calibration::startFullCalibration() {
    if (_isCalibrating) return; // Verhindere doppelten Start
    
    _isCalibrating = true;
    
    // Starte die erweiterte Kalibrierung für beide Joysticks
    JoystickSystem::leftJoystick->startCalibration();
    JoystickSystem::rightJoystick->startCalibration();
    
    _isCalibrating = false;
}

// Original-Methode, die jetzt nicht mehr automatisch geprüft wird
void Calibration::checkCalibrationTrigger() {
    // Diese Methode macht jetzt nichts mehr 
    // Sie wird nur aus Kompatibilitätsgründen beibehalten
}

bool Calibration::isCalibrating() {
    return _isCalibrating;
}