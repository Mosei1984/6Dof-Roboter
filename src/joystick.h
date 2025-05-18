#pragma once
#include <Arduino.h>
#include "kalmanfilter.h"

class Joystick {
public:
    Joystick(int xPin, int yPin, int btnPin);
    void begin();
    void calibrate();
    void read();
    int getX();
    int getY();
    bool isPressed();
    float getNormalizedX();
    float getNormalizedY();
    // Neue Funktionen für erweiterte Kalibrierung
    void startCalibration();
    void saveMinMax(int xMin, int xMax, int yMin, int yMax);

    // Füge diese Methode zur Joystick-Klasse hinzu oder aktualisiere sie
    float getXWithDeadband(float deadbandPercentage) {
        float value = getX();
        if (abs(value) < deadbandPercentage) {
            return 0.0f;
        }
        return value;
    }

    float getYWithDeadband(float deadbandPercentage) {
        float value = getY();
        if (abs(value) < deadbandPercentage) {
            return 0.0f;
        }
        return value;
    }
    
private:
    int _xPin, _yPin, _btnPin;
    int _xCenter, _yCenter;
    int _xValue, _yValue;
    int _xMin, _xMax, _yMin, _yMax; // Min/Max-Werte für Mapping
    KalmanFilter _xFilter, _yFilter;

    // Hilfsfunktion zum Mapping auf 0-1000
    int mapJoystickValue(int value, int minVal, int center, int maxVal);
};
