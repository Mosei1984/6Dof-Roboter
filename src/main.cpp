#include <Arduino.h>
#include "config.h"
#include "joystick.h"
#include "calibration.h"
#include "RobotKinematics.h"
#include "kalmanfilter.h"
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <AccelStepper.h>
#include "workspace_limits.h"
#include "axis_config.h"
#include <EEPROM.h>

#define ROBOT_HOME_MAGIC 0x42ABCDEF

void processHomingMenu();
void exitKinematicMode();
void synchronizeKinematicsWithSteppers();
struct RobotHomeData {
    uint32_t magic;
    float jointAngles[6];
};
void saveRobotHome(const JointAngles& angles) {
    RobotHomeData data;
    data.magic = ROBOT_HOME_MAGIC;
    for (int i = 0; i < 6; ++i) data.jointAngles[i] = angles.angles[i];
    EEPROM.put(100, data);
}
bool loadRobotHome(JointAngles& angles) {
    RobotHomeData data;
    EEPROM.get(100, data);
    if (data.magic != ROBOT_HOME_MAGIC) return false;
    for (int i = 0; i < 6; ++i) angles.angles[i] = data.jointAngles[i];
    return true;
}
void clearRobotHome() {
    RobotHomeData data = {0};
    EEPROM.put(100, data);
}
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1

// Hier wird die display-Variable definiert
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Joystick-Objekte als Pointer deklarieren
Joystick* leftJoystick = nullptr;
Joystick* rightJoystick = nullptr;
// Extern-Deklarationen für globale Konfigurationsvariablen
extern JoystickConfig _joystickConfig;
extern StepperConfig _stepperConfig[6];
extern AccelStepper* _steppers[6];
// Roboter-Kinematik Objekt und ausgewähltes Gelenk
RobotConfig robotConfig;
RobotKinematics* robotKin = nullptr;
int selectedJoint = 0;

// Joystick-Zustände für die Erkennung von Richtungsänderungen
enum JoystickState {
  JOYSTICK_CENTERED,
  JOYSTICK_LEFT,
  JOYSTICK_RIGHT,
  JOYSTICK_UP,
  JOYSTICK_DOWN
};
enum HomingMenuOption {
    HOMING_MENU_START_HOMING = 0,
    HOMING_MENU_TO_CENTER,
    HOMING_MENU_SAVE_HOME,
    HOMING_MENU_LOAD_HOME,   // <--- NEU
    HOMING_MENU_CLEAR_HOME,
    HOMING_MENU_COUNT
};
int homingMenuSelection = 0;
bool homingMenuActive = false;
bool homingMenuConfirmed = false;
unsigned long lastMenuMove = 0;
unsigned long lastClearHomePress = 0;
int clearHomePressCount = 0;
// Aktueller Joystick-Zustand für die Gelenkauswahl
JoystickState joystickXState = JOYSTICK_CENTERED;
JoystickState joystickYState = JOYSTICK_CENTERED;

// Zustandsmaschine - Zustände
enum SystemState {
  STATE_STARTUP,
  STATE_JOINT_MODE,
  STATE_KINEMATIC_MODE,
  STATE_HOMING_MODE,
  STATE_CALIBRATION_MODE  // Kalibrierung als separater Modus
};

// Funktion zum Homen eines Gelenks (Deklaration)
bool homeJoint(int jointIndex) {
    static bool homingJointStarted = false;
    static bool coarseHomingDone = false;
    static int lastSwitchState = HIGH;

    // Erste Initialisierung für dieses Gelenk
    if (!homingJointStarted) {
        homingJointStarted = true;
        coarseHomingDone = false;

        Serial.print("Starte Homing für Gelenk ");
        Serial.println(jointIndex + 1);

        // Motor aktivieren
        digitalWrite(_pinConfig.stepperPins[jointIndex][2], LOW); // Enable aktivieren

        // Setze schnelle Homing-Geschwindigkeit für die Grobsuche
        _steppers[jointIndex]->setSpeed(-_stepperConfig[jointIndex].homingSpeed);

        // Merke aktuelle Position
        _steppers[jointIndex]->setCurrentPosition(0);

        // Initialen Zustand des Endschalters lesen
        lastSwitchState = digitalRead(_pinConfig.stepperPins[jointIndex][3]);

        // Debug-Ausgabe
        Serial.print("Endschalter-Pin: ");
        Serial.print(_pinConfig.stepperPins[jointIndex][3]);
        Serial.print(", Anfangszustand: ");
        Serial.println(lastSwitchState == HIGH ? "HIGH" : "LOW");
    }

    // Endschalter überprüfen
    int limitSwitchPin = _pinConfig.stepperPins[jointIndex][3];
    int currentSwitchState = digitalRead(limitSwitchPin);

    // Phase 1: Grobe Suche nach dem Endschalter
    if (!coarseHomingDone) {
        if (currentSwitchState != lastSwitchState && currentSwitchState == LOW) {
            Serial.print("Endschalter für Gelenk ");
            Serial.print(jointIndex + 1);
            Serial.println(" beim groben Homing erreicht");

            // Motor anhalten
            _steppers[jointIndex]->setSpeed(0);
            _steppers[jointIndex]->stop();

            delay(100);

            // Vom Endschalter zurückfahren (positive Richtung, weg vom Endschalter)
            _steppers[jointIndex]->move(100); // Etwa 5 Grad zurück bei 20 steps/degree
            _steppers[jointIndex]->setSpeed(_stepperConfig[jointIndex].homingSpeed * 0.5);

            while (_steppers[jointIndex]->distanceToGo() != 0) {
                _steppers[jointIndex]->runSpeed();
            }

            delay(100);

            Serial.print("Rückzug abgeschlossen, starte präzises Homing für Gelenk ");
            Serial.println(jointIndex + 1);

            // Langsames Homing einleiten (1/4 der normalen Geschwindigkeit)
            _steppers[jointIndex]->setSpeed(-_stepperConfig[jointIndex].homingSpeed * 0.25);

            coarseHomingDone = true;
            lastSwitchState = HIGH; // Zurücksetzen für die Feinsuche
        }

        lastSwitchState = currentSwitchState;
        _steppers[jointIndex]->runSpeed();
    }
    // Phase 2: Präzise langsame Suche
    else {
        if (currentSwitchState != lastSwitchState && currentSwitchState == LOW) {
            Serial.print("Endschalter für Gelenk ");
            Serial.print(jointIndex + 1);
            Serial.print(" beim präzisen Homing erreicht (Pin ");
            Serial.print(limitSwitchPin);
            Serial.println(")");

            _steppers[jointIndex]->setSpeed(0);
            _steppers[jointIndex]->stop();

            _steppers[jointIndex]->setCurrentPosition(0);

            digitalWrite(_pinConfig.stepperPins[jointIndex][2], HIGH);

            JointAngles currentAngles = robotKin->getCurrentJointAngles();
            currentAngles.angles[jointIndex] = 0.0f;
            robotKin->setCurrentJointAngles(currentAngles);

            homingJointStarted = false;
            coarseHomingDone = false;
            return true; // Homing für dieses Gelenk abgeschlossen
        }

        lastSwitchState = currentSwitchState;
        _steppers[jointIndex]->runSpeed();
    }

    return false; // Homing für dieses Gelenk läuft noch
}

// Globale Variablen für Homing
int homingJointIndex = 0;
bool homingComplete = false;
bool homingStarted = false;

// Globaler Zustand
SystemState currentState = STATE_STARTUP;
bool calibrationLock = false;
unsigned long stateChangeTime = 0;
unsigned long lastButtonCheckTime = 0;
unsigned long lastJointChangeTime = 0;
bool rightButtonPressed = false;

// Debug-Variablen
bool debugOutputEnabled = false;  // Debug-Ausgabe standardmäßig deaktiviert
unsigned long lastDebugTime = 0;  // Zeitpunkt der letzten Debug-Ausgabe



// Kalmanfilter für geglättete Bewegungen
KalmanFilter positionFilterX(0.01, 0.1);
KalmanFilter positionFilterY(0.01, 0.1);
KalmanFilter positionFilterZ(0.01, 0.1);
KalmanFilter rotationFilter(0.01, 0.1);

// Hinzufügen einer Debug-Anzeige, die den aktuellen Zustand zeigt
void displayDebugStatus() {
  Serial.print("Aktueller Zustand: ");
  switch (currentState) {
    case STATE_STARTUP: Serial.println("STARTUP"); break;
    case STATE_CALIBRATION_MODE: Serial.println("CALIBRATION_MODE"); break;
    case STATE_JOINT_MODE: Serial.println("JOINT_MODE"); break;
    case STATE_KINEMATIC_MODE: Serial.println("KINEMATIC_MODE"); break;
    case STATE_HOMING_MODE: Serial.println("HOMING_MODE"); break;
    default: Serial.println("UNBEKANNT"); break;
  }
}

// Roboterkonfiguration mit Standardwerten initialisieren
void initRobotConfig() {
  // Gelenkwinkelgrenzen (in Grad)
  for (int i = 0; i < 6; i++) {
    robotConfig.jointMin[i] = -180.0 * DEG_TO_RAD;
    robotConfig.jointMax[i] = 180.0 * DEG_TO_RAD;
  }
  
  // DH-Parameter - einfacher 6DOF-Arm
  // Format: {a, alpha, d, theta}
  robotConfig.dhParams[0] = {0.0, M_PI/2, 50.0, 0.0};     // Basis
  robotConfig.dhParams[1] = {80.0, 0.0, 0.0, M_PI/2};     // Schulter
  robotConfig.dhParams[2] = {80.0, 0.0, 0.0, 0.0};        // Ellbogen
  robotConfig.dhParams[3] = {0.0, M_PI/2, 80.0, 0.0};     // Handgelenk Pitch
  robotConfig.dhParams[4] = {0.0, -M_PI/2, 0.0, 0.0};     // Handgelenk Roll
  robotConfig.dhParams[5] = {0.0, 0.0, 40.0, 0.0};        // Greifer
  
  // Werkzeug-Offset
  robotConfig.toolOffsetX = 0.0;
  robotConfig.toolOffsetY = 0.0;
  robotConfig.toolOffsetZ = 30.0;
}

// Anzeigeoptionen für jeden Zustand
void displayStartupScreen() {
  display.clearDisplay();
  display.setCursor(0,0);
  display.println("6DOF Robot Start");
  display.println("Druecke L-Joystick");
  display.println("fuer Kalibrierung");
  display.display();
}

void displayJointMode() {
  display.clearDisplay();
  display.setCursor(0,0);
  display.println("JOINT MODE");
  
  // Joint-Winkel in Grad anzeigen
  JointAngles angles = robotKin->getCurrentJointAngles();
  for (int i = 0; i < 6; i++) {
    display.print("J");
    display.print(i+1);
    display.print(":");
    display.print(angles.angles[i] * 180.0 / M_PI, 0);
    display.print(i == selectedJoint ? "* " : "  ");
    if (i == 2) display.println();
  }
  display.println();
  
  // Endeffektorposition anzeigen
  CartesianPose pose = robotKin->getCurrentPose();
  display.print("X:");
  display.print(pose.x, 1);
  display.print(" Y:");
  display.println(pose.y, 1);
  display.print("Z:");
  display.print(pose.z, 1);
  display.println();
  
  // Bedienungshinweise
  display.println("R Joy X: Gelenk waehlen");
  display.println("L Joy X: +/- Bewegung");
  display.println("R Btn: Moduswechsel");
  
  display.display();
}

void displayKinematicMode() {
  display.clearDisplay();
  display.setCursor(0,0);
  display.println("KINEMATIC MODE");
  
  // Joint-Winkel in Grad anzeigen
  JointAngles angles = robotKin->getCurrentJointAngles();
  for (int i = 0; i < 3; i++) {
    display.print("J");
    display.print(i+1);
    display.print(":");
    display.print(angles.angles[i] * 180.0 / M_PI, 0);
    display.print(" ");
  }
  display.println();
  for (int i = 3; i < 6; i++) {
    display.print("J");
    display.print(i+1);
    display.print(":");
    display.print(angles.angles[i] * 180.0 / M_PI, 0);
    display.print(" ");
  }
  display.println();
  
  // Endeffektorposition anzeigen
  CartesianPose pose = robotKin->getCurrentPose();
  display.print("X:");
  display.print(pose.x, 1);
  display.print(" Y:");
  display.println(pose.y, 1);
  display.print("Z:");
  display.print(pose.z, 1);
  display.print(" Yaw:");
  display.println(pose.yaw * 180.0 / M_PI, 1);
  
  // Reichweitenanzeige
  float maxReach = robotConfig.dhParams[1].a + robotConfig.dhParams[2].a;
  float currentDistance = sqrt(pose.x*pose.x + pose.y*pose.y + 
                             (pose.z-robotConfig.dhParams[0].d)*(pose.z-robotConfig.dhParams[0].d));
  display.print("Dist:");
  display.print(currentDistance, 1);
  display.print("/");
  display.print(maxReach, 1);
  display.println("mm");
  
  display.display();
}
void displayHomingMode() {
    display.clearDisplay();
    display.setCursor(0,0);
    display.println("HOMING MODUS");
    display.println("----------------");
  
    if (!homingStarted) {
        display.println("Druecke den linken");
        display.println("Button zum Starten");
        display.println("des Homings.");
        display.println("");
        display.println("Rechter Button: Modi wechseln");
    } else {
        display.print("Referenzieren Achse ");
        display.println(homingJointIndex + 1);
        display.drawRect(0, 30, 128, 10, SSD1306_WHITE);
        int progress = (homingJointIndex * 128) / 6;
        display.fillRect(0, 30, progress, 10, SSD1306_WHITE);
        display.setCursor(0, 45);
        display.println("Bitte warten...");
    }
    display.display();
}

void displayHomingMenu() {
    display.clearDisplay();
    display.setCursor(0,0);
    display.println("Homing-Menue");
    display.println("--------------------");
    display.println("Wähle mit R-Joystick:");

    const int visibleLines = 3;
    int startIdx = homingMenuSelection - 1;
    if (startIdx < 0) startIdx = 0;
    if (startIdx > HOMING_MENU_COUNT - visibleLines) startIdx = HOMING_MENU_COUNT - visibleLines;
    if (startIdx < 0) startIdx = 0;

    display.println("");
    for (int i = startIdx; i < startIdx + visibleLines && i < HOMING_MENU_COUNT; ++i) {
        if (i == homingMenuSelection) display.print("> ");
        else display.print("  ");
        switch (i) {
            case HOMING_MENU_START_HOMING: display.println("Homing starten"); break;
            case HOMING_MENU_TO_CENTER:    display.println("Fahre zur Mitte"); break;
            case HOMING_MENU_SAVE_HOME:    display.println("Home speichern"); break;
            case HOMING_MENU_LOAD_HOME:    display.println("Home laden"); break;
            case HOMING_MENU_CLEAR_HOME:   display.println("Home vergessen"); break;
        }
    }
    display.println("");
    display.println("Bestätige mit L-Button");
    display.display();
}
void processHomingMenu() {
    // --- Moduswechsel per rechtem Button (entprellt) ---
    static bool rightButtonPressed = false;
    if (rightJoystick->isPressed() && !rightButtonPressed) {
        if (millis() - stateChangeTime > 500) {
            rightButtonPressed = true;
            stateChangeTime = millis();

            // Modi durchschalten: Homing -> Calibration -> Joint -> Kinematic -> Homing ...
            if (currentState == STATE_HOMING_MODE) {
                currentState = STATE_CALIBRATION_MODE;
            } else if (currentState == STATE_CALIBRATION_MODE) {
                currentState = STATE_JOINT_MODE;
            } else if (currentState == STATE_JOINT_MODE) {
                currentState = STATE_KINEMATIC_MODE;
            } else if (currentState == STATE_KINEMATIC_MODE) {
                exitKinematicMode();
                // 3. Dann Modus wechseln
                currentState = STATE_HOMING_MODE;
                homingStarted = false;
                homingJointIndex = 0;
            }
            Serial.print("Modus gewechselt zu: ");
            Serial.println(currentState);
            return;
        }
    } else if (!rightJoystick->isPressed()) {
        rightButtonPressed = false;
    }

    displayHomingMenu();

    // Menüauswahl mit rechtem Joystick Y
    float rightY = rightJoystick->getNormalizedY();
    static unsigned long lastMenuMove = 0;
    if (millis() - lastMenuMove > 200) {
        if (rightY > 0.7f && homingMenuSelection > 0) {
            homingMenuSelection--;
            lastMenuMove = millis();
        } else if (rightY < -0.7f && homingMenuSelection < HOMING_MENU_COUNT-1) {
            homingMenuSelection++;
            lastMenuMove = millis();
        }
    }

    // Linker Button: Bestätigen (nur auf steigende Flanke)
    static bool lastLeftPressed = false;
    bool leftPressed = leftJoystick->isPressed();

    if (leftPressed && !lastLeftPressed) {
        switch (homingMenuSelection) {
            case HOMING_MENU_START_HOMING:
                homingStarted = true;
                homingJointIndex = 0;
                break;
            case HOMING_MENU_TO_CENTER: {
                Serial.println("Menü: Fahre zur Mitte gewählt");

                // Zielpunkt im Arbeitsraum definieren 
                float a1 = robotConfig.dhParams[1].a;
                float a2 = robotConfig.dhParams[2].a;
                float minZ = robotConfig.dhParams[0].d;
                CartesianPose centerPose;
                centerPose.x = 0;
                centerPose.y = 0;
                centerPose.z = minZ + (a1 + a2) / 2.0f;
                centerPose.yaw = 0;
                centerPose.pitch = 0;
                centerPose.roll = 0;

                Serial.print("Ziel-Pose: X="); Serial.print(centerPose.x);
                Serial.print(" Y="); Serial.print(centerPose.y);
                Serial.print(" Z="); Serial.println(centerPose.z);

                // 2. IK berechnen
                JointAngles centerAngles;
                if (robotKin->inverseKinematics(centerPose, centerAngles)) {

                    // Zielpositionen berechnen
                    long targetSteps[6];
                    long startSteps[6];
                    for (int i = 0; i < 6; ++i) {
                        float deg = centerAngles.angles[i] * 180.0 / M_PI;
                        targetSteps[i] = deg * _stepperConfig[i].stepsPerDegree;
                        startSteps[i] = _steppers[i]->currentPosition();
                        _steppers[i]->moveTo(targetSteps[i]);
                    }

                    // Bewegung mit Fortschrittsbalken
                    bool allDone = false;
                    while (!allDone) {
                        allDone = true;
                        long maxDist = 0, maxDistToGo = 0;
                        for (int i = 0; i < 6; ++i) {
                            long dist = abs(targetSteps[i] - startSteps[i]);
                            long distToGo = abs(_steppers[i]->distanceToGo());
                            if (dist > maxDist) maxDist = dist;
                            if (distToGo > maxDistToGo) maxDistToGo = distToGo;
                            if (_steppers[i]->distanceToGo() != 0) {
                                _steppers[i]->run();
                                allDone = false;
                            }
                        }

                        // Fortschritt berechnen (0.0 ... 1.0)
                        float progress = 1.0f;
                        if (maxDist > 0) {
                            progress = 1.0f - (float)maxDistToGo / (float)maxDist;
                            if (progress < 0) progress = 0;
                            if (progress > 1) progress = 1;
                        }

                        // Display aktualisieren
                        display.clearDisplay();
                        display.setCursor(0,0);
                        display.println("Homing-Menue");
                        display.println("--------------------");
                        display.println("Waehle mit RJoy");
                        display.println("");
                        display.println("Fahre zur Mitte...");
                        // Fortschrittsbalken zeichnen (z.B. 100px breit, 8px hoch)
                        int barWidth = 100;
                        int barHeight = 8;
                        int barX = 14;
                        int barY = 40;
                        display.drawRect(barX, barY, barWidth, barHeight, SSD1306_WHITE);
                        display.fillRect(barX, barY, (int)(barWidth * progress), barHeight, SSD1306_WHITE);
                        display.display();

                        delay(10);
                    }

                    for (int i = 0; i < 6; ++i) {
                        _steppers[i]->setCurrentPosition(_steppers[i]->targetPosition());
                    }
                    robotKin->setCurrentJointAngles(centerAngles);

                    // FK zur Kontrolle
                    CartesianPose pose = robotKin->getCurrentPose();
                    Serial.print("FK Endpose: X="); Serial.print(pose.x,2);
                    Serial.print(" Y="); Serial.print(pose.y,2);
                    Serial.print(" Z="); Serial.print(pose.z,2);
                    Serial.print(" | Yaw="); Serial.print(pose.yaw*180.0/M_PI,2);
                    Serial.print(" Pitch="); Serial.print(pose.pitch*180.0/M_PI,2);
                    Serial.print(" Roll="); Serial.println(pose.roll*180.0/M_PI,2);

                    display.clearDisplay();
                    display.setCursor(0,0);
                    display.println("Roboter steht jetzt");
                    display.println("in der Mitte!");
                    display.display();
                    delay(1500);
                } else {
                    Serial.println("IK Fehler! Ziel nicht erreichbar.");
                    display.clearDisplay();
                    display.setCursor(0,0);
                    display.println("IK Fehler!");
                    display.display();
                    delay(1500);
                }
                break;
            }
            case HOMING_MENU_SAVE_HOME: {
                JointAngles current = robotKin->getCurrentJointAngles();
                saveRobotHome(current);
                display.clearDisplay();
                display.setCursor(0,0);
                display.println("Home gespeichert!");
                display.display();
                delay(1200);
                break;
            }
            case HOMING_MENU_LOAD_HOME: {
                JointAngles homeAngles;
                if (loadRobotHome(homeAngles)) {
                    // KEINE Bewegung! Nur Kinematik und Stepper-Positionen setzen
                    robotKin->setCurrentJointAngles(homeAngles);
                    for (int i = 0; i < 6; ++i) {
                        _steppers[i]->setCurrentPosition(
                            homeAngles.angles[i] * 180.0 / M_PI * _stepperConfig[i].stepsPerDegree
                        );
                    }
                    Serial.println("Home geladen! Kinematik synchronisiert, keine Bewegung.");
                    display.clearDisplay();
                    display.setCursor(0,0);
                    display.println("Home geladen!");
                    display.println("Keine Bewegung.");
                    display.display();
                    delay(1200);
                } else {
                    display.clearDisplay();
                    display.setCursor(0,0);
                    display.println("Kein Home");
                    display.println("gespeichert!");
                    display.display();
                    delay(1200);
                }
                break;
            }
            case HOMING_MENU_CLEAR_HOME: {
                static unsigned long lastClearHomePress = 0;
                static int clearHomePressCount = 0;
                if (clearHomePressCount == 0 || millis() - lastClearHomePress > 1500) {
                    clearHomePressCount = 1;
                    lastClearHomePress = millis();
                    display.clearDisplay();
                    display.setCursor(0,0);
                    display.println("Nochmal druecken");
                    display.println("um Home zu loeschen!");
                    display.display();
                } else if (clearHomePressCount == 1) {
                    clearRobotHome();
                    display.clearDisplay();
                    display.setCursor(0,0);
                    display.println("Home geloescht!");
                    display.display();
                    delay(1200);
                    clearHomePressCount = 0;
                }
                break;
            }
        }
    }
    lastLeftPressed = leftPressed;
}

void processJointControl() {
    // --- Debug-Ausgabe für Fehlerfindung ---
    Serial.print("Joystick-Werte: Left X: ");
    Serial.print(leftJoystick->getX());
    Serial.print(", Left Y: ");
    Serial.println(leftJoystick->getY());

    // ---- 1) Gelenk­auswahl mit rechtem Joystick X ----
    float rightX = (rightJoystick->getX() - 500.0f) / 500.0f;  // in [-1..+1]
    if (rightX >  0.7f && joystickXState == JOYSTICK_CENTERED) {
        joystickXState = JOYSTICK_RIGHT;
        if (selectedJoint < 5) {
            selectedJoint++;
            Serial.print("Gelenk erhöht auf: ");
            Serial.println(selectedJoint + 1);
        }
    }
    else if (rightX < -0.7f && joystickXState == JOYSTICK_CENTERED) {
        joystickXState = JOYSTICK_LEFT;
        if (selectedJoint > 0) {
            selectedJoint--;
            Serial.print("Gelenk verringert auf: ");
            Serial.println(selectedJoint + 1);
        }
    }
    else if (fabs(rightX) < 0.4f) {
        joystickXState = JOYSTICK_CENTERED;
    }

    // --- 2) Gelenk­bewegung mit linkem Joystick X (Deadzone auslagern) ---
    // getXWithDeadband() liefert 0, wenn |getX()| < deadband :contentReference[oaicite:4]{index=4}:contentReference[oaicite:5]{index=5}
    float rawX = leftJoystick->getXWithDeadband(_joystickConfig.deadband);
    // Normalisieren auf [-1..+1]
    float norm = (rawX - 500.0f) / 500.0f;
    Serial.print("Joint-Modus: rawX w/ deadzone = ");
    Serial.print(rawX);
    Serial.print(" → norm = ");
    Serial.println(norm);

    // --- 3) Geschwindigkeit setzen ---
    if (rawX == 0.0f) {
        // im Deadzone-Bereich = Stillstand
        _steppers[selectedJoint]->setSpeed(0);
    } else {
        // volle Empfindlichkeit: ±maxSpeed bei |norm|==1
        float spd = -norm * _stepperConfig[selectedJoint].maxSpeed;
        Serial.print(" -> Setze Speed für Joint ");
        Serial.print(selectedJoint);
        Serial.print(": ");
        Serial.println(spd);
        _steppers[selectedJoint]->setSpeed(spd);
    }

    // --- 4) Motor kontinuierlich laufen lassen ---
    _steppers[selectedJoint]->runSpeed();
}

// Kinematik-Modus-Steuerung verbessert
void processKinematicControl() {
    Serial.println(F("=== processKinematicControl() start ==="));

    // 1) Joystick-Rohwerte
    float leftX  =  leftJoystick->getNormalizedX();
    float leftY  =  leftJoystick->getNormalizedY();
    float rightX = rightJoystick->getNormalizedX();
    float rightY = rightJoystick->getNormalizedY();
    // --- Filter-Reset bei Stillstand ---
    static bool wasMoving = false;
    bool isMoving = (fabs(leftJoystick->getNormalizedX()) > 0.01f) || (fabs(leftJoystick->getNormalizedY()) > 0.01f) ||
                    (fabs(rightJoystick->getNormalizedX()) > 0.01f) || (fabs(rightJoystick->getNormalizedY()) > 0.01f);

    if (!isMoving && wasMoving) {
        // Joystick wurde losgelassen → Filter zurücksetzen
        positionFilterX.reset();
        positionFilterY.reset();
        positionFilterZ.reset();
        rotationFilter.reset();
        Serial.println("Joystick losgelassen, Filter zurückgesetzt!");
    }
    wasMoving = isMoving;
    Serial.print("  Joystick: Lx="); Serial.print(leftX, 2);
    Serial.print("  Ly=");           Serial.print(leftY, 2);
    Serial.print("  Rx=");           Serial.print(rightX, 2);
    Serial.print("  Ry=");           Serial.println(rightY, 2);

    // 2) Geschwindigkeit dynamisch setzen
    float speedFactor = max( max(fabs(leftX), fabs(leftY)),
                             max(fabs(rightX), fabs(rightY)) );
    float dynSpd = speedFactor * 100.0f;
    float dynAcc = dynSpd * 2.0f;
    for (int i = 0; i < 6; ++i) {
        _steppers[i]->setMaxSpeed(dynSpd);
        _steppers[i]->setAcceleration(dynAcc);
    }
    Serial.print("  dynSpd="); Serial.print(dynSpd,1);
    Serial.print("  dynAcc="); Serial.println(dynAcc,1);

    // 3) Delta-Berechnung via Filter (jetzt float!)
    CartesianPose current = robotKin->getCurrentPose();
    float xChg   = positionFilterX.update(  leftX   * 1.0f );
    float yChg   = positionFilterY.update( -leftY   * 1.0f );
    float zChg   = positionFilterZ.update( -rightY  * 1.0f );
    float yawChg = rotationFilter.update(  rightX  * 0.03f );

    // 4) Debug: immer die Deltas ausgeben
    Serial.print("  Deltas: xChg=");   Serial.print(xChg, 3);
    Serial.print("  yChg=");            Serial.print(yChg, 3);
    Serial.print("  zChg=");            Serial.print(zChg, 3);
    Serial.print("  yawChg=");          Serial.println(yawChg, 4);

    // 5) Schwellen-IF wieder aktivieren
    if (fabs(xChg) > 0.01f || fabs(yChg) > 0.01f ||
        fabs(zChg) > 0.01f || fabs(yawChg) > 0.001f) {

        // 6) Zielpose berechnen und clampen
        CartesianPose target = current;
        target.x   += xChg;
        target.y   += yChg;
        target.z   += zChg;
        target.yaw += yawChg;

        const float a1 = robotConfig.dhParams[1].a,
                    a2 = robotConfig.dhParams[2].a;
        const float maxRad = a1 + a2,
                    minZ   = robotConfig.dhParams[0].d,
                    maxZ   = minZ + maxRad;
        target.x = constrain(target.x, -maxRad, +maxRad);
        target.y = constrain(target.y, -maxRad, +maxRad);
        target.z = constrain(target.z,  minZ,     maxZ);

        Serial.print("  Kinematik-Ziel: X=");   Serial.print(target.x, 2);
        Serial.print("  Y=");                    Serial.print(target.y, 2);
        Serial.print("  Z=");                    Serial.println(target.z, 2);
        Serial.print("               Yaw=");      Serial.println(target.yaw, 3);

        // 7) IK aufrufen
        JointAngles newA;
        if (robotKin->inverseKinematics(target, newA)) {
            for (int i = 0; i < 6; ++i) {
                float deg = newA.angles[i] * 180.0 / M_PI;
                long steps = deg * _stepperConfig[i].stepsPerDegree;
                _steppers[i]->moveTo(steps);
            }
        }
        else {
            Serial.println("  IK fehlerhaf Bewegung übersprungen");
            // Stepper-Targets auf aktuelle Position setzen, damit sie nicht „verbleibend“ anzeigen
            for (int i = 0; i < 6; ++i) {
                _steppers[i]->moveTo(_steppers[i]->currentPosition());
            }
        }
      
                for (int i = 0; i < 6; ++i) {
        float deg = newA.angles[i] * 180.0 / M_PI;
        long steps = deg * _stepperConfig[i].stepsPerDegree;
        _steppers[i]->moveTo(steps);
    }
    robotKin->setCurrentJointAngles(newA);

        }
        else {
            Serial.println("  IK fehlerhaf Bewegung übersprungen");
            // Stepper-Targets auf aktuelle Position setzen, damit sie nicht „verbleibend“ anzeigen
            for (int i = 0; i < 6; ++i) {
                _steppers[i]->moveTo(_steppers[i]->currentPosition());
            }
        }
      // Ende Schwellen-IF

    Serial.println(F("=== processKinematicControl() end ==="));
}
 void exitKinematicMode() {
    for (int i = 0; i < 6; ++i) {
        _steppers[i]->moveTo(_steppers[i]->currentPosition());
    }
    synchronizeKinematicsWithSteppers();
} 
// Homing-Modus-Steuerung
void processHomingMode() {
    // Menü immer anzeigen, wenn kein Homing läuft
    if (!homingStarted) {
        processHomingMenu();
        return;
    }
    // Wenn Homing läuft, Fortschritt anzeigen und homeJoint() benutzen
    displayHomingMode();
    if (homingJointIndex < 6) {
        if (homeJoint(homingJointIndex)) {
            homingJointIndex++;
            Serial.print("Gelenk ");
            Serial.print(homingJointIndex);
            Serial.println(" referenziert");
            delay(500);
        }
    } else {
        // Nach Homing zurück ins Menü
        Serial.println("Homing abgeschlossen");
        homingStarted = false;
        homingJointIndex = 0;
        homingMenuSelection = 0;
        // Kinematik synchronisieren
        JointAngles homeAngles;
        for (int i = 0; i < 6; i++) {
            float posDegrees = _steppers[i]->currentPosition() / _stepperConfig[i].stepsPerDegree;
            homeAngles.angles[i] = posDegrees * M_PI / 180.0;
        }
        robotKin->setCurrentJointAngles(homeAngles);
    }
}

void setup() {
  Serial.begin(115200);
  
  // ZUERST die Konfiguration laden
  loadDefaultPinConfig();
  loadDefaultJoystickConfig();
  loadDefaultStepperConfig();
  pinMode(_pinConfig.errorLedPin, OUTPUT);
  
  // DANACH die Joysticks erstellen
  leftJoystick = new Joystick(_pinConfig.leftXPin, _pinConfig.leftYPin, _pinConfig.leftBtnPin);
  rightJoystick = new Joystick(_pinConfig.rightXPin, _pinConfig.rightYPin, _pinConfig.rightBtnPin);

  // I2C initialisieren
  Wire.begin();

  // OLED initialisieren
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
      digitalWrite(_pinConfig.errorLedPin, HIGH);
      Serial.println("Display initialization failed");
      while(1);
  }
    
  display.clearDisplay();
  display.display();
  display.setTextColor(SSD1306_WHITE);
  
  // Roboterkonfiguration und Kinematik initialisieren
  initRobotConfig();
  robotKin = new RobotKinematics(robotConfig);
  JointAngles init = {{0,0,0,0,0,0}};    // alle Gelenke auf 0 rad
  robotKin->setCurrentJointAngles(init);  // internal currentPose wird einmal berechnet
  // Stepper und Endschalter initialisieren
  for (int i = 0; i < 6; i++) {
    // Step/Dir Pins für jeden Motor einstellen
    _steppers[i] = new AccelStepper(AccelStepper::DRIVER, 
                                   _pinConfig.stepperPins[i][0],   // STEP Pin
                                   _pinConfig.stepperPins[i][1]);  // DIR Pin
    
    // Debugging: Ausgabe der Stepper-Pins
    Serial.print("Stepper ");
    Serial.print(i);
    Serial.print(" Pins - STEP: ");
    Serial.print(_pinConfig.stepperPins[i][0]);
    Serial.print(", DIR: ");
    Serial.print(_pinConfig.stepperPins[i][1]);
    Serial.print(", ENABLE: ");
    Serial.println(_pinConfig.stepperPins[i][2]);
    
    // Enable Pin als OUTPUT setzen
    pinMode(_pinConfig.stepperPins[i][2], OUTPUT);
    digitalWrite(_pinConfig.stepperPins[i][2], HIGH);  // Deaktiviert zu Beginn (aktiv LOW)
    
    // Limit/Endschalter Pin als INPUT mit Pull-up
    pinMode(_pinConfig.stepperPins[i][3], INPUT_PULLUP);
    
    // Stepper-Parameter einstellen
    _steppers[i]->setMaxSpeed(_stepperConfig[i].maxSpeed);
    _steppers[i]->setAcceleration(_stepperConfig[i].acceleration);
    
    // Debug: Stepper-Konfiguration
    Serial.print("Stepper ");
    Serial.print(i);
    Serial.print(" Config - MaxSpeed: ");
    Serial.print(_stepperConfig[i].maxSpeed);
    Serial.print(", Accel: ");
    Serial.print(_stepperConfig[i].acceleration);
    Serial.print(", StepsPerDegree: ");
    Serial.println(_stepperConfig[i].stepsPerDegree);
  }
  
  // Test der Stepper-Motoren
  Serial.println("Teste Stepper-Kommunikation...");
  for (int i = 0; i < 6; i++) {
    // Motor aktivieren
    digitalWrite(_pinConfig.stepperPins[i][2], LOW);
    
    // Kurze Bewegung ausführen
    _steppers[i]->setCurrentPosition(0);
    _steppers[i]->moveTo(100);  // 100 Schritte vorwärts
    
    // Warte auf Bewegung
    while (_steppers[i]->distanceToGo() != 0) {
        _steppers[i]->run();
        delay(1);
    }
    
    delay(200);  // Kurze Pause
    
    // Zurück zur Startposition
    _steppers[i]->moveTo(0);
    
    while (_steppers[i]->distanceToGo() != 0) {
        _steppers[i]->run();
        delay(1);
    }
    
    // Motor deaktivieren
    digitalWrite(_pinConfig.stepperPins[i][2], HIGH);
    
    Serial.print("Stepper ");
    Serial.print(i);
    Serial.println(" getestet");
    
    delay(500);  // Pause zwischen Tests
  }
  
  leftJoystick->begin();
  rightJoystick->begin();
  
  // Anfangszustand setzen
  currentState = STATE_STARTUP;
  stateChangeTime = millis();
  
  // Startmeldung anzeigen
  displayStartupScreen();
  delay(1500);
  
  // Roboter-Bitmap beim Start anzeigen
  display.clearDisplay();
  display.drawBitmap(0, 0, robotArmBitmap, SCREEN_WIDTH, SCREEN_HEIGHT, SSD1306_WHITE);
  display.display();
  delay(2000);

  // Grundkalibrierung (nur Zentrum)
  Calibration::calibrateJoysticks();
  
  // Im Joint-Modus starten
  currentState = STATE_HOMING_MODE;
  stateChangeTime = millis();
  
  Serial.println("System initialisiert, starte im Homing-Modus");
}

// Neue Funktion für Kalibrierungsbildschirm
void displayCalibrationMode() {
  display.clearDisplay();
  display.setCursor(0,0);
  display.println("KALIBRIERUNG");
  display.println("----------------");
  display.println("Druecke den linken");
  display.println("Button zum Starten");
  display.println("der Kalibrierung.");
  display.println("");
  display.println("Rechter Button: Modi wechseln");
  display.display();
}

// Funktion zum Starten der Kalibrierung
void startCalibration() {
  Serial.println("Starte Kalibrierung manuell");
  
  // Wir nutzen hier die bestehende Calibration-Klasse, aber steuern sie direkt
  Calibration::_isCalibrating = true;
  
  // Benutzer informieren
  display.clearDisplay();
  display.setCursor(0,0);
  display.println("KALIBRIERE...");
  display.println("Bewege beide Joysticks");
  display.println("zu den Extrempositionen");
  display.println("innerhalb von 5 Sek.");
  display.display();
  
  // Starte die erweiterte Kalibrierung für beide Joysticks
  leftJoystick->startCalibration();
  rightJoystick->startCalibration();
  
  // Kalibrierung beenden
  Calibration::_isCalibrating = false;
  
  // Bestätigungsbildschirm
  display.clearDisplay();
  display.setCursor(0,0);
  display.println("KALIBRIERUNG FERTIG");
  display.println("----------------");
  display.println("Werte gespeichert.");
  display.println("");
  display.println("Druecke rechten Button");
  display.println("zum Fortfahren.");
  display.display();
  
  delay(2000);
}

// Neue Funktion zur Synchronisierung von Stepper-Positionen und Kinematik
void synchronizeKinematicsWithSteppers() {
    JointAngles currentAngles = robotKin->getCurrentJointAngles();
    bool updated = false;
    
    for (int i = 0; i < 6; i++) {
        // Aktuelle Stepper-Position in Grad umrechnen
        long currentSteps = _steppers[i]->currentPosition();
        float currentDegrees = currentSteps / _stepperConfig[i].stepsPerDegree;
        float currentRadians = currentDegrees * M_PI / 180.0;
        
        // Wenn es einen Unterschied gibt, aktualisiere die Kinematik
        if (fabs(currentRadians - currentAngles.angles[i]) > 0.01) {
            currentAngles.angles[i] = currentRadians;
            updated = true;
            
            if (debugOutputEnabled) {
                Serial.print("Synchronisiere Gelenk ");
                Serial.print(i + 1);
                Serial.print(": ");
                Serial.print(currentDegrees);
                Serial.print("° (");
                Serial.print(currentSteps);
                Serial.println(" Schritte)");
            }
        }
    }
    
    if (updated) {
        robotKin->setCurrentJointAngles(currentAngles);
    }
}
void processButtonInput() {
    bool leftPressed = leftJoystick->isPressed();
    bool rightPressed = rightJoystick->isPressed();

    // Entprellen
    if (millis() - lastButtonCheckTime < 200) return;

    // Rechter Button: Moduswechsel
    if (rightPressed && !rightButtonPressed) {
        if (millis() - stateChangeTime > 500) {
            rightButtonPressed = true;
            lastButtonCheckTime = millis();

            // Modi durchschalten
            if (currentState == STATE_JOINT_MODE) {
                currentState = STATE_KINEMATIC_MODE;
            } else if (currentState == STATE_KINEMATIC_MODE) {
                // --- Kinematic Mode sauber verlassen ---
                for (int i = 0; i < 6; ++i) {
                    _steppers[i]->moveTo(_steppers[i]->currentPosition());
                }
                synchronizeKinematicsWithSteppers();
                currentState = STATE_HOMING_MODE;
                homingStarted = false;
                homingJointIndex = 0;
            } else if (currentState == STATE_HOMING_MODE) {
                currentState = STATE_CALIBRATION_MODE;
            } else if (currentState == STATE_CALIBRATION_MODE) {
                currentState = STATE_JOINT_MODE;
            }

            Serial.print("Zustand geändert zu: ");
            Serial.println(currentState);
            stateChangeTime = millis();
        }
    } else if (!rightPressed) {
        rightButtonPressed = false;
    }

    // Linker Button: Aktion im Modus
    if (leftPressed && !calibrationLock) {
        if (millis() - stateChangeTime > 500) {
            if (currentState == STATE_HOMING_MODE && !homingStarted) {
                homingStarted = true;
                Serial.println("Homing gestartet");
            } else if (currentState == STATE_CALIBRATION_MODE) {
                Serial.println("Starte Kalibrierung manuell");
                display.clearDisplay();
                display.setCursor(0,0);
                display.println("KALIBRIERE...");
                display.println("Bewege beide Joysticks");
                display.println("zu den Extrempositionen");
                display.println("innerhalb von 5 Sek.");
                display.display();

                Calibration::startFullCalibration();

                display.clearDisplay();
                display.setCursor(0,0);
                display.println("KALIBRIERUNG FERTIG");
                display.println("----------------");
                display.println("Werte gespeichert.");
                display.println("");
                display.println("Druecke rechten Button");
                display.println("zum Fortfahren.");
                display.display();

                delay(2000);
            }
            stateChangeTime = millis();
            lastButtonCheckTime = millis();
        }
    } else if (!leftPressed) {
        calibrationLock = false;
    }
}
 
void loop() {
    // Joystick-Werte auslesen
    leftJoystick->read();
    rightJoystick->read();
    
    // Stepper-Motoren bewegen - mit mehr Debugging
    bool anyMotorRunning = false;
    for (int i = 0; i < 6; i++) {
        if (_steppers[i]->distanceToGo() != 0) {
            _steppers[i]->run();
            anyMotorRunning = true;
            
            // Wenn sich der Motor bewegt, geben wir das aus
            if (i == selectedJoint) {  // Nur für das ausgewählte Gelenk, um Spam zu vermeiden
                Serial.print("Stepper ");
                Serial.print(i);
                Serial.print(" bewegt sich zu Position ");
                Serial.print(_steppers[i]->targetPosition());
                Serial.print(", aktuell: ");
                Serial.print(_steppers[i]->currentPosition());
                Serial.print(", verbleibend: ");
                Serial.println(_steppers[i]->distanceToGo());
            }
        } else {
            // Motor deaktivieren, wenn er das Ziel erreicht hat
            digitalWrite(_pinConfig.stepperPins[i][2], HIGH);
        }
    }
    
    // Wenn Motoren bewegt wurden, synchronisiere das Kinematik-Modell
    if (anyMotorRunning) {
        synchronizeKinematicsWithSteppers();
    }
    
    // Zustandsmaschine
    switch (currentState) {
      case STATE_STARTUP:
        // Startzustand ist nur für anfängliche Anzeige, dann Übergang zu Joint-Modus
        if (millis() - stateChangeTime > 2000) {
          currentState = STATE_JOINT_MODE;
          stateChangeTime = millis();
        }
        break;
        
      case STATE_JOINT_MODE:
        // Button-Eingabe für Zustandsänderungen verarbeiten
        processButtonInput();
        
        // Joint-Steuerung verarbeiten
        processJointControl();
        
        // //zeige aktualisieren
        displayJointMode();
        break;
        
      case STATE_KINEMATIC_MODE:
        // Button-Eingabe für Zustandsänderungen verarbeiten
        processButtonInput();
        
        // Kinematik-Steuerung verarbeiten
        processKinematicControl();
        
        // Anzeige aktualisieren
        displayKinematicMode();
        break;
        
      case STATE_HOMING_MODE:
        if (homingStarted) {
            processHomingMode(); // klassisches Homing, setzt homingStarted=false wenn fertig
            // Wenn Homing fertig:
            if (!homingStarted) {
                // Nach Homing zurück ins Menü
                homingMenuSelection = 0;
                clearHomePressCount = 0;
            }
        } else {
            processHomingMenu(); // Menü immer anzeigen, solange kein Homing läuft
        }
        break;
        
      case STATE_CALIBRATION_MODE:
        // Button-Eingabe für Zustandsänderungen verarbeiten
        processButtonInput();
        
        // Anzeige aktualisieren
        displayCalibrationMode();
        break;
    }
  
    delay(50);
}
