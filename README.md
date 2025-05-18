<think>I need to update the README.md file to align with the new functions and structure of the 6DOF robot project. Based on what I've seen:

1. The system now has a more modular design with components separated into different files like:
   - JoystickSystem
   - RobotSystem
   - DisplaySystem
   - StepperSystem

2. There's also added functionality like:
   - TimerLoop for better timing
   - Debug namespace for improved debugging
   - processHomingMenuSelection for better menu handling

3. The mode switching logic has been improved to be more reliable

I need to update the README to reflect these changes while maintaining its overall structure and purpose. The original README includes sections on project goals, main functions, system architecture, operation, pin configuration, DH parameters, extensions, prerequisites, setup, and licensing.</think>
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
- **Präzises Timing** durch Timer-basierte Steuerung
- **Erweitertes Debugging** mit konfigurierbarer Debug-Ausgabe
- **(Optional) Sensorintegration** (z.B. ADXL345 am Greifer für Feedback)

---

## Systemarchitektur & Zusammenarbeit der Module

### 1. **Modulare Systemkomponenten**
Die Software ist in funktionale Module gegliedert:
- **JoystickSystem**: Verarbeitung der Joystick-Eingaben
- **DisplaySystem**: Anzeige auf dem OLED-Display
- **StepperSystem**: Steuerung der Schrittmotoren
- **RobotSystem**: Verwaltung des Roboterzustands und der Modi
- **Debug**: Konfigurierbare Debug-Ausgabe zur Fehlersuche
- **TimerLoop**: Präzise Timing-Steuerung aller Komponenten

### 2. **Pin-Konfiguration (`config.h` / `config.cpp`)**
- Alle verwendeten Pins für Motoren, Endschalter, Joysticks und LEDs sind zentral definiert.
- Beispiel:
  - `stepperPins[i][0]` = STEP-Pin für Motor i
  - `stepperPins[i][1]` = DIR-Pin für Motor i
  - `stepperPins[i][2]` = ENABLE-Pin für Motor i
  - `stepperPins[i][3]` = Endschalter-Pin für Motor i

### 3. **DH-Parameter & Robotergeometrie**
- Die Kinematik basiert auf den **Denavit-Hartenberg-Parametern** (`dhParams`), die für jede Achse die Geometrie des Arms beschreiben.
- Beispiel:
  ```cpp
  robotConfig.dhParams[0] = {35.5,  -M_PI/2, 135.0, 0.0};      // Basis
  robotConfig.dhParams[1] = {160.0,  0.0,     0.0,  -M_PI/2};  // Schulter
  // usw.
  ```
- Diese Parameter werden für die Vorwärts- und Inverse Kinematik verwendet.

### 4. **Kinematik-Berechnungen (`RobotKinematics.h/cpp`)**
- **Vorwärtskinematik:** Berechnet die Position und Orientierung des Endeffektors aus den aktuellen Gelenkwinkeln.
- **Inverse Kinematik:** Berechnet die benötigten Gelenkwinkel, um eine gewünschte Position im Raum zu erreichen.
- Die Kinematik wird bei jeder Bewegung aktualisiert und synchronisiert.

### 5. **Stepper-Steuerung (`StepperSystem.h/cpp`)**
- Jeder Motor wird über die [AccelStepper](https://www.airspayce.com/mikem/arduino/AccelStepper/) Bibliothek angesteuert.
- Die Parameter (maxSpeed, acceleration, stepsPerDegree, homingSpeed) werden für jede Achse individuell gesetzt.
- Die Motoren werden im Loop kontinuierlich mit `run()` oder `runSpeed()` angesteuert.
- Nach Bewegungen werden die Stepper-Positionen mit der Kinematik synchronisiert.

### 6. **Joystick-Steuerung (`JoystickSystem.h/cpp`)**
- Zwei Joysticks (links/rechts) steuern entweder die einzelnen Gelenke (Joint Mode) oder die Position des Endeffektors (Kinematic Mode).
- Die Joystick-Klassen bieten Deadzone, Kalibrierung und Normalisierung der Werte.
- Die `processButtonInput()` Methode behandelt Tastenereignisse und Moduswechsel zentral und konsistent.

### 7. **Display-Anzeige (`DisplaySystem.h/cpp`)**
- Zeigt aktuelle Betriebsart, Gelenkwinkel, Endeffektorposition, Menüs und Statusmeldungen an.
- Fortschrittsbalken bei Bewegungen (z.B. Homing, "Fahre zur Mitte").
- Bietet Methoden für verschiedene Anzeigemodi wie displayJointMode(), displayKinematicMode(), etc.

### 8. **Menüsystem & Betriebsmodi (`RobotSystem.h/cpp`)**
- **Joint Mode:** Direkte Steuerung einzelner Gelenke.
- **Kinematic Mode:** Steuerung des Endeffektors im Raum (XYZ + Yaw).
- **Homing Mode:** Referenzfahrt aller Achsen mit Endschaltern und Menü-Interface.
- **Calibration Mode:** Kalibrierung der Joysticks.
- **Startup:** Initialisierung und Startbildschirm.
- Verbesserte Menünavigation mit zuverlässigem Moduswechsel.

### 9. **Timing-System (`TimerLoop.h/cpp`)**
- Bietet präzises Timing für kritische Steuerungsaufgaben.
- Trennt Joystick-Abfragen (100Hz), Display-Updates (50Hz) und Steuerungstasks (1kHz).
- Sorgt für gleichmäßige Ausführung aller Funktionen ohne Blockierungen.

### 10. **EEPROM-Speicherung**
- Home-Positionen können gespeichert, geladen oder gelöscht werden über das Homing-Menü.
- Speicherung erfolgt im EEPROM ab Adresse 100 mit Validierungs-Magic.

### 11. **Debug-System**
- Konfigurierbare Debug-Ausgaben über serielle Schnittstelle
- Kann zur Laufzeit aktiviert/deaktiviert werden für verschiedene Diagnosestufen

### 12. **Kalman-Filter (`kalmanfilter.h/cpp`)**
- Für die Joystick-Werte werden Kalman-Filter verwendet, um Bewegungen zu glätten.
- Bietet verbesserte Nutzererfahrung durch flüssigere Bewegungen.

---

## Bedienung

- **Moduswechsel:** Rechter Joystick-Button (funktioniert in allen Modi zuverlässig)
- **Gelenk auswählen:** Rechter Joystick X-Achse
- **Gelenk bewegen:** Linker Joystick X-Achse
- **Kinematiksteuerung:** Beide Joysticks (XYZ + Yaw)
- **Homing-Menü:** Über Moduswechsel erreichbar
- **Menüauswahl:** Rechter Joystick Y-Achse
- **Aktion bestätigen:** Linker Joystick-Button
- **Kalibrierung:** Linker Joystick-Button im Calibration-Modus

### Homing-Menü

Im Homing-Modus bietet das Menü folgende Optionen:
- **Homing starten:** Referenziert alle Achsen nacheinander
- **Fahre zur Mitte:** Bewegt den Roboter in eine zentrale Position
- **Home speichern:** Speichert aktuelle Position als Home-Position
- **Home laden:** Lädt gespeicherte Home-Position
- **Home vergessen:** Löscht gespeicherte Home-Position

---

## Pin-Konfiguration (Beispiel)

| Funktion         | Pin (Beispiel) |
|------------------|---------------|
| Stepper 1 STEP   | 2             |
| Stepper 1 DIR    | 3             |
| Stepper 1 ENABLE | 4             |
| Stepper 1 Limit  | 22            |
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

## Debugging mit Logic Analyzer

Zur Diagnose von Problemen mit den Schrittmotoren kann ein Logic Analyzer verwendet werden:

1. **Anschluss:**
   - Verbinde STEP, DIR, ENABLE und ggf. Limit-Switch-Signale mit dem Logic Analyzer
   - Verbinde GND mit dem Schaltungsgrund

2. **Zu beobachtende Signale:**
   - STEP-Signale: Pulsbreite, Frequenz, Timing-Konsistenz
   - DIR-Signale: Wechsel vor STEP-Signalen
   - ENABLE-Signale: Aktiv (LOW) während erwarteter Bewegung
   - Limit-Switch-Signale: Zustandsänderungen während des Homing

3. **Typische Probleme:**
   - Unregelmäßige Pulstimings verursachen ruckartige Bewegungen
   - Fehlende Pulse können auf Softwareprobleme hindeuten
   - Prellen der Endschalter kann Homing-Probleme verursachen

---

## Erweiterungen

- **Sensorintegration:** Das System ist vorbereitet, um z.B. einen ADXL345-Beschleunigungssensor am Greifer für Feedback zu integrieren.
- **Weitere Sensoren oder Aktoren** können über die modulare Systemarchitektur einfach ergänzt werden.
- **Externe Steuerung:** Durch die modulare Struktur kann das System leicht um eine serielle Befehlsschnittstelle erweitert werden.

---

## Voraussetzungen

- Teensy (z.B. 3.2, 4.0)
- AccelStepper-Bibliothek
- Adafruit SSD1306 & GFX Bibliothek
- TimerOne Bibliothek für präzises Timing
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