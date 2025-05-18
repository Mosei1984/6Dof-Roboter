#include "joystick.h"
#include "config.h"    // definiert JoystickConfig und _joystickConfig

extern JoystickConfig _joystickConfig;
Joystick::Joystick(int xPin, int yPin, int btnPin)
    : _xPin(xPin), _yPin(yPin), _btnPin(btnPin), 
      _xCenter(0), _yCenter(0), _xValue(0), _yValue(0),
      _xMin(0), _xMax(0), _yMin(0), _yMax(0),
      _xFilter(0.1, 0.1, 1.0), _yFilter(0.1, 0.1, 1.0) {}

void Joystick::begin() {
    pinMode(_xPin, INPUT);
    pinMode(_yPin, INPUT);
    pinMode(_btnPin, INPUT_PULLUP);
}

void Joystick::calibrate() {
    // Mittelwert von mehreren Messungen als Nullpunkt
    long xSum = 0, ySum = 0;
    for (int i = 0; i < 100; ++i) {
        xSum += analogRead(_xPin);
        ySum += analogRead(_yPin);
        delay(2);
    }
    _xCenter = xSum / 100;
    _yCenter = ySum / 100;
    
    // Wenn keine Min/Max-Werte gesetzt sind, verwende Standardwerte
    if (_xMin == 0 && _xMax == 0) {
        _xMin = _xCenter - 100;
        _xMax = _xCenter + 100;
    }
    if (_yMin == 0 && _yMax == 0) {
        _yMin = _yCenter - 100;
        _yMax = _yCenter + 100;
    }
}

void Joystick::startCalibration() {
    // Diese Methode sammelt Min/Max-Werte während einer Kalibrierungsphase
    int xMin = 1023, xMax = 0, yMin = 1023, yMax = 0;
    int xReading, yReading;
    
    // Mittelwert zuerst bestimmen
    calibrate();
    
    // Visuelle Anzeige für den Kalibrierungsstart
    Serial.println("Bewege Joystick in alle Richtungen!");
    Serial.println("5 Sekunden zum Kalibrieren...");
    
    // Min/Max-Werte sammeln
    unsigned long startTime = millis();
    while (millis() - startTime < 5000) {
        xReading = analogRead(_xPin);
        yReading = analogRead(_yPin);
        
        // Min/Max aktualisieren
        if (xReading < xMin) xMin = xReading;
        if (xReading > xMax) xMax = xReading;
        if (yReading < yMin) yMin = yReading;
        if (yReading > yMax) yMax = yReading;
        
        delay(10);
    }
    
    // Min/Max-Werte speichern
    saveMinMax(xMin, xMax, yMin, yMax);
    Serial.println("Kalibrierung abgeschlossen!");
}

void Joystick::saveMinMax(int xMin, int xMax, int yMin, int yMax) {
    _xMin = xMin;
    _xMax = xMax;
    _yMin = yMin;
    _yMax = yMax;
    
    Serial.println("Kalibrierungswerte:");
    Serial.print("X: Min="); Serial.print(_xMin);
    Serial.print(" Center="); Serial.print(_xCenter);
    Serial.print(" Max="); Serial.println(_xMax);
    Serial.print("Y: Min="); Serial.print(_yMin);
    Serial.print(" Center="); Serial.print(_yCenter);
    Serial.print(" Max="); Serial.println(_yMax);
}

void Joystick::read() {
    int rawX = analogRead(_xPin);
    int rawY = analogRead(_yPin);
    
    // Gefilterte Werte
    int filteredX = _xFilter.update(rawX);
    int filteredY = _yFilter.update(rawY);
    
    // Mapping auf 0-1000 mit 500 als Mitte
    _xValue = mapJoystickValue(filteredX, _xMin, _xCenter, _xMax);
    _yValue = mapJoystickValue(filteredY, _yMin, _yCenter, _yMax);
}

int Joystick::mapJoystickValue(int value, int minVal, int center, int maxVal) {
    // 0) Clamp aufs Kalibrierungs-Intervall
    value = constrain(value, minVal, maxVal);

    // 1) Mapping für Werte unter dem Zentrum (0-500)
    int m;
    if (value <= center) {
        m = map(value, minVal, center, 0, 500);
    } 
    // 2) Mapping für Werte über dem Zentrum (500-1000)
    else {
        m = map(value, center, maxVal, 500, 1000);
    }

    // 3) (Sicherheit) Clamp nochmal auf 0…1000
    return constrain(m, 0, 1000);
}


int Joystick::getX() { return _xValue; }
int Joystick::getY() { return _yValue; }
bool Joystick::isPressed() { return digitalRead(_btnPin) == LOW; }
// Normalisierte X-Achse:  0…1000 → –1…+1
    float Joystick::getNormalizedX() {
        float raw = (float)getX();                     // z.B. in [0…1000]
  float v   = raw - 500.0f;                      // Deviaton um die Mitte
  if (fabs(v) < _joystickConfig.deadband) return 0;
  // Normiere auf [-1…+1]
  return v > 0
    ? (v - _joystickConfig.deadband) / (500.0f - _joystickConfig.deadband)
    : (v + _joystickConfig.deadband) / (500.0f - _joystickConfig.deadband);
}
    

    float Joystick::getNormalizedY() {
        float raw = (float)getY();
  float v   = raw - 500.0f;
  if (fabs(v) < _joystickConfig.deadband) return 0;
  return v > 0
    ? (v - _joystickConfig.deadband) / (500.0f - _joystickConfig.deadband)
    : (v + _joystickConfig.deadband) / (500.0f - _joystickConfig.deadband);
}