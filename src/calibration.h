#pragma once

class Calibration {
public:
    static void calibrateJoysticks();
    static void checkCalibrationTrigger();
    static bool isCalibrating();
    static void setCalibrating(bool calibrating); // Neuer Setter
    static void startFullCalibration();
    static bool _isCalibrating;
};
