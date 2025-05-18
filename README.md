# 6Dof-Roboter

## Projektziel

Dieses Projekt steuert einen selbstgebauten 6-Achsen-Roboterarm (6DOF) mit Schrittmotoren, Endschaltern, OLED-Display und zwei Joysticks. Ziel ist es, den Roboterarm sowohl im **Joint Mode** (direkte Achsensteuerung) als auch im **Kinematic Mode** (Positionssteuerung im Raum) präzise und benutzerfreundlich zu bedienen.  
Das System unterstützt Homing, Kalibrierung, Speichern/Laden von Positionen und ist modular für Erweiterungen (z.B. Sensorfeedback) ausgelegt.

---

## Hauptfunktionen

- **6 Schrittmotoren** für die Achsen J1–J6 (Basis, Schulter, Ellbogen, Handgelenk Pitch/Roll, Greifer)
- **Endschalter** für sicheres Referenzieren (Homing)
- **2 Joysticks** für intuitive Steuerung (Gelenk- oder Raumsteuerung)
- **OLED-Display** zur Anzeige von Status, Positionen und Menüs
- **EEPROM-Speicherung** von Home-Positionen
- **Kinematik-Bibliothek** für Vorwärts- und Inverse Kinematik
- **Kalibrierung** der Joysticks
- **Modulares Menüsystem** für Homing, Kalibrierung, etc.
- **(Optional) Sensorintegration** (z.B. ADXL345 am Greifer für Feedback)

---

## Systemarchitektur & Zusammenarbeit der Module

### 1. **Pin-Konfiguration (`config.h` / `config.cpp`)**
- Alle verwendeten Pins für Motoren, Endschalter, Joysticks und LEDs sind zentral definiert.
- Beispiel:
  - `stepperPins[i][0]` = STEP-Pin für Motor i
  - `stepperPins[i][1]` = DIR-Pin für Motor i
  - `stepperPins[i][2]` = ENABLE-Pin für Motor i
  - `stepperPins[i][3]` = Endschalter-Pin für Motor i

### 2. **DH-Parameter & Robotergeometrie**
- Die Kinematik basiert auf den **Denavit-Hartenberg-Parametern** (`dhParams`), die für jede Achse die Geometrie des Arms beschreiben.
- Beispiel:
  ```cpp
  robotConfig.dhParams[0] = {35.5,  -M_PI/2, 135.0, 0.0};      // Basis
  robotConfig.dhParams[1] = {160.0,  0.0,     0.0,  -M_PI/2};  // Schulter
  // usw.
  ```
- Diese Parameter werden für die Vorwärts- und Inverse Kinematik verwendet.

### 3. **Kinematik-Berechnungen (`RobotKinematics.h/cpp`)**
- **Vorwärtskinematik:** Berechnet die Position und Orientierung des Endeffektors aus den aktuellen Gelenkwinkeln.
- **Inverse Kinematik:** Berechnet die benötigten Gelenkwinkel, um eine gewünschte Position im Raum zu erreichen.
- Die Kinematik wird bei jeder Bewegung aktualisiert und synchronisiert.

### 4. **Stepper-Steuerung**
- Jeder Motor wird über die [AccelStepper](https://www.airspayce.com/mikem/arduino/AccelStepper/) Bibliothek angesteuert.
- Die Parameter (maxSpeed, acceleration, stepsPerDegree, homingSpeed) werden für jede Achse individuell gesetzt.
- Die Motoren werden im Loop kontinuierlich mit `run()` oder `runSpeed()` angesteuert.
- Nach Bewegungen werden die Stepper-Positionen mit der Kinematik synchronisiert.

### 5. **Joystick-Steuerung**
- Zwei Joysticks (links/rechts) steuern entweder die einzelnen Gelenke (Joint Mode) oder die Position des Endeffektors (Kinematic Mode).
- Die Joystick-Klassen bieten Deadzone, Kalibrierung und Normalisierung der Werte.

### 6. **OLED-Display**
- Zeigt aktuelle Betriebsart, Gelenkwinkel, Endeffektorposition, Menüs und Statusmeldungen an.
- Fortschrittsbalken bei Bewegungen (z.B. Homing, "Fahre zur Mitte").

### 7. **Menüsystem & Betriebsmodi**
- **Joint Mode:** Direkte Steuerung einzelner Gelenke.
- **Kinematic Mode:** Steuerung des Endeffektors im Raum (XYZ + Yaw).
- **Homing Mode:** Referenzfahrt aller Achsen mit Endschaltern.
- **Calibration Mode:** Kalibrierung der Joysticks.
- **Startup:** Initialisierung und Startbildschirm.

### 8. **EEPROM-Speicherung**
- Home-Positionen können gespeichert, geladen oder gelöscht werden.
- Speicherung erfolgt im EEPROM ab Adresse 100.

### 9. **Kalman-Filter**
- Für die Joystick-Werte werden Kalman-Filter verwendet, um Bewegungen zu glätten.

---

## Bedienung

- **Moduswechsel:** Rechter Joystick-Button
- **Gelenk auswählen:** Rechter Joystick X-Achse
- **Gelenk bewegen:** Linker Joystick X-Achse
- **Kinematiksteuerung:** Beide Joysticks (XYZ + Yaw)
- **Homing starten:** Linker Joystick-Button im Homing-Menü
- **Kalibrierung:** Linker Joystick-Button im Calibration-Menü
- **Menüauswahl:** Rechter Joystick Y-Achse
- **Aktion bestätigen:** Linker Joystick-Button

---

## Pin-Konfiguration (Beispiel)

| Funktion         | Pin (Beispiel) |
|------------------|---------------|
| Stepper 1 STEP   | 2             |
| Stepper 1 DIR    | 3             |
| Stepper 1 ENABLE | 4             |
| Stepper 1 Limit  | 5             |
| ...              | ...           |
| Joystick L X     | 40            |
| Joystick L Y     | 41            |
| Joystick L Btn   | 27            |
| Joystick R X     | 42            |
| Joystick R Y     | 43            |
| Joystick R Btn   | 28            |
| OLED SDA/SCL     | 18/19         |

> **Hinweis:** Die tatsächlichen Pins findest du in `src/config.cpp`.

---

## DH-Parameter (Beispiel)

```cpp
robotConfig.dhParams[0] = {35.5,  -M_PI/2, 135.0, 0.0};      // Basis
robotConfig.dhParams[1] = {160.0,  0.0,     0.0,  -M_PI/2};  // Schulter
robotConfig.dhParams[2] = {15.0,  -M_PI/2,  0.0,  0.0};      // Ellbogen
robotConfig.dhParams[3] = {0.0,    M_PI/2, 138.4, 0.0};      // Handgelenk Pitch
robotConfig.dhParams[4] = {0.0,   -M_PI/2,  0.0,  0.0};      // Handgelenk Roll
robotConfig.dhParams[5] = {0.0,    0.0,    26.06, 0.0};      // Greifer
```

---

## Erweiterungen

- **Sensorintegration:** Das System ist vorbereitet, um z.B. einen ADXL345-Beschleunigungssensor am Greifer für Feedback zu integrieren.
- **Weitere Sensoren oder Aktoren** können über die zentrale Pin- und Konfigurationsstruktur einfach ergänzt werden.

---

## Voraussetzungen

- Teensy (z.B. 3.2, 4.0)
- AccelStepper-Bibliothek
- Adafruit SSD1306 & GFX Bibliothek
- Adafruit ADXL345 (optional)
- Zwei analoge Joysticks
- 6 Schrittmotoren mit Treibern
- Endschalter für jede Achse
- OLED-Display (128x64 I2C)

---

## Aufbau & Verdrahtung

- Die genaue Verdrahtung ist in der Pin-Konfiguration dokumentiert.
- Jeder Motor benötigt STEP/DIR/ENABLE und einen Endschalter.
- Joysticks werden an analoge Pins angeschlossen.
- OLED-Display an I2C (SDA/SCL).
- Optional: Sensoren wie ADXL345 an I2C.

---

## Lizenz

MIT License

---

## Kontakt

Fragen, Anregungen oder Verbesserungen?  
Erstelle ein Issue oder einen Pull Request!

---
