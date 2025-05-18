# 6Dof-Roboter

## Projektziel

Dieser selbstgebaute 6-Achsen-Roboterarm vereint Präzision, Flexibilität und Benutzerfreundlichkeit. Ausgestattet mit Schrittmotoren, Endschaltern, einem OLED-Display und zwei Joysticks, ermöglicht er sowohl eine direkte Achsensteuerung (Joint Mode) als auch eine positionsbasierte Steuerung im Raum (Kinematic Mode). Unser Ziel ist es, dir ein verlässliches System zu bieten, das Homing, Kalibrierung und das Speichern von Positionen intuitiv handhabbar macht – und gleichzeitig modular genug ist, um zukünftige Erweiterungen, zum Beispiel mit Sensorfeedback, zu integrieren.

---

## Hauptfunktionen

Dank seiner modularen Architektur kannst du auf einen Blick erkennen, welche Kernfunktionen der Roboterarm bietet:

* **Sechs Schrittmotoren (J1–J6)** steuern Basis, Schulter, Ellbogen, Handgelenk (Pitch/Roll) und Greifer mit hoher Genauigkeit.
* **Endschalter** sorgen für sicheres Homing und präzises Referenzieren.
* **Zwei Joysticks** ermöglichen eine intuitive Bedienung im Gelenk- oder Raumsteuerungsmodus.
* **OLED-Display** zeigt Menüs, Statusmeldungen und Positionsdaten übersichtlich an.
* **TimerLoop** garantiert gleichmäßige, blockierungsfreie Abläufe und trennt Aufgaben wie Joystick-Abfragen (100 Hz), Display-Updates (50 Hz) und Motorsteuerung (1 kHz).
* **EEPROM-Unterstützung** zum Speichern, Laden und Löschen von Home-Positionen.
* **Kinematikbibliothek** (vorwärts und invers) für präzise Bewegungsberechnungen.
* **Erweitertes Debugging** über einen konfigurierbaren Debug-Namespace für detaillierte Fehlersuche.

---

## Systemarchitektur

Die Software ist bewusst in übersichtliche Module aufgeteilt, die jeweils eine klar umrissene Aufgabe übernehmen:

1. **JoystickSystem**

   * Liest und kalibriert Joystick-Signale, verwaltet Deadzones und Tasten-Events.
2. **DisplaySystem**

   * Steuert das OLED-Display, zeigt Menüs, Balkendiagramme und Status an.
3. **StepperSystem**

   * Regelt die Schrittmotoren mit der AccelStepper-Bibliothek (Geschwindigkeit, Beschleunigung, Schritte pro Grad).
4. **RobotSystem**

   * Koordiniert Betriebsmodi (Joint, Kinematic, Homing, Calibration) und verarbeitet Menünavigation.
5. **TimerLoop**

   * Bietet ein fein abgestuftes Timing für alle Systemkomponenten, um Verzögerungen zu vermeiden.
6. **Debug**

   * Schaltet serielle Debug-Ausgaben flexibel ein oder aus, um während der Entwicklung gezielt Informationen zu erhalten.

Alle Hardware-Pins und Parameter sind zentral in `config.h`/`config.cpp` definiert – so behältst du jederzeit den Überblick.

---

## Kinematik & DH-Parameter

Die Vorwärts- und Inverse-Kinematik basieren auf Denavit–Hartenberg-Parametern. Jede Achse hat folgende beispielhafte Definition:

```cpp
robotConfig.dhParams[0] = {35.5,  -M_PI/2, 135.0, 0.0};      // Basis
robotConfig.dhParams[1] = {160.0,  0.0,     0.0,  -M_PI/2};  // Schulter
robotConfig.dhParams[2] = {15.0,  -M_PI/2,  0.0,  0.0};      // Ellbogen
robotConfig.dhParams[3] = {0.0,    M_PI/2, 138.4, 0.0};      // Handgelenk Pitch
robotConfig.dhParams[4] = {0.0,   -M_PI/2,  0.0,  0.0};      // Handgelenk Roll
robotConfig.dhParams[5] = {0.0,    0.0,    26.06, 0.0};      // Greifer
```

Diese Werte fließen in alle Bewegungsberechnungen ein und sorgen für konsistente Steuerung.

---

## Bedienung im Überblick

* **Modus wechseln:** Rechter Joystick-Button (funktioniert in jedem Modus).
* **Gelenk auswählen:** Rechter Joystick X-Achse.
* **Bewegen:** Linker Joystick X-Achse (Joint Mode) bzw. beide Joysticks für XYZ+Yaw (Kinematic Mode).
* **Menüs bedienen:** Rechter Joystick Y-Achse zum Navigieren, linker Joystick-Button zum Bestätigen.
* **Homing-Menü:** Starte Homing, fahre in Mittelstellung oder speichere/lade/vergesse Positionen.
* **Kalibrierung:** Linker Joystick-Button im Calibration-Modus.

---

## Pin-Konfiguration (Beispiel)

| Funktion         | Pin   |
| ---------------- | ----- |
| Stepper 1 STEP   | 2     |
| Stepper 1 DIR    | 3     |
| Stepper 1 ENABLE | 4     |
| Stepper 1 Limit  | 22    |
| Joystick L X     | 40    |
| Joystick L Y     | 41    |
| Joystick L Btn   | 27    |
| Joystick R X     | 42    |
| Joystick R Y     | 43    |
| Joystick R Btn   | 28    |
| OLED SDA/SCL     | 18/19 |

> **Hinweis:** Die exakten Pins findest du in `src/config.cpp`.

---

## Debugging mit Logic Analyzer

Falls du ungewöhnliche Motorbewegungen beobachtest, kannst du folgende Signale überprüfen:

1. **STEP:** Pulsbreite und -frequenz.
2. **DIR:** Umschaltzeitpunkte vor den STEP-Pulsen.
3. **ENABLE:** Low = aktiv, High = inaktiv.
4. **Limit-Switches:** Schaltsignale während Homing.

So entdeckst du Prellen, Aussetzer oder Timing-Probleme schnell.

---

## Erweiterungen & Zukunft

Das System ist offen für neue Ideen:

* **Sensorintegration** (z. B. ADXL345) für Kraft- oder Lageregulierung.
* **Serielle oder drahtlose Steuerung** über zusätzliche Schnittstellen.
* **Weitere Aktoren**, um den Roboterarm zu erweitern.

---

## Voraussetzungen & Installation

1. Teensy (z. B. 3.2 oder 4.0)
2. AccelStepper, Adafruit SSD1306/GFX, TimerOne Bibliotheken
3. Zwei analoge Joysticks, 6 Schrittmotoren mit Treibern, Endschalter, OLED-Display (128×64 I2C)
4. (Optional) ADXL345-Beschleunigungssensor

Klonen, Bibliotheken installieren und direkt loslegen:

```bash
git clone https://github.com/dein-repo/6Dof-Roboter.git
cd 6Dof-Roboter
arduino-cli compile --fqbn teensy:avr:teensy40\arduino-cli upload --fqbn teensy:avr:teensy40
```

---

## Lizenz

Dieses Projekt steht unter der MIT License. Sieh die `LICENSE`-Datei für Details.

---

## Kontakt

Feedback, Fragen oder Pull Requests? Öffne gerne ein Issue oder erstelle einen Pull Request auf GitHub!
