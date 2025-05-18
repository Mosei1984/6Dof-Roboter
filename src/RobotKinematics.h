#pragma once

#include <Arduino.h>
#include <math.h>

// DH-Parameter für ein Gelenk
struct DHParams {
    float a;      // Link-Length
    float alpha;  // Link-Twist
    float d;      // Link-Offset
    float theta;  // Joint-Angle-Offset
};

// Pose im kartesischen Raum
struct CartesianPose {
    float x, y, z;    // Position
    float yaw, pitch, roll;  // Orientierung (ZYX-Euler-Winkel)
};

// Gelenkwinkel-Container
struct JointAngles {
    float angles[6];
};

// Roboter-Konfiguration (DH-Tabellen, Joint-Grenzen, Tool-Offset)
struct RobotConfig {
    DHParams   dhParams[6];       // DH-Parameter für Gelenk 0…5
    float      jointMin[6];       // minimale Gelenkwinkel (rad)
    float      jointMax[6];       // maximale Gelenkwinkel (rad)
    float      toolOffsetX;       // Werkzeug-Offset in X
    float      toolOffsetY;       // Werkzeug-Offset in Y
    float      toolOffsetZ;       // Werkzeug-Offset in Z
    
    // Neue Konfigurationsfelder
    float maxRobotSpeed;          // Maximale Robotergeschwindigkeit in %
    float homingSpeed;            // Geschwindigkeit beim Referenzieren
    float centerX;                // X-Koordinate des Zentrums
    float centerY;                // Y-Koordinate des Zentrums
    float centerZ;                // Z-Koordinate des Zentrums
    bool invertJoysticks;         // Joysticks invertieren?
    int uiTheme;                  // UI-Theme (0=Standard, 1=Hell, 2=Dunkel)
    float joystickDeadband;       // Joystick-Totzone
};

// Forward declaration of StepperConfig
struct StepperConfig;
extern StepperConfig _stepperConfig[6];

class RobotKinematics {
public:
    // Konstruktor
    RobotKinematics(const RobotConfig& config);

    // Setter/Getter für aktuelle Gelenkwinkel & Pose
    void          setCurrentJointAngles(const JointAngles& angles);
    JointAngles   getCurrentJointAngles() const;
    CartesianPose getCurrentPose() const;

    // Werkzeug-Offset nachträglich ändern
    void setToolOffset(float x, float y, float z);

    // Vorwärts-Kinematik: Gelenkwinkel → Pose
    CartesianPose forwardKinematics(const JointAngles& angles);

    // Invers-Kinematik: Zielpose → Gelenkwinkel
    bool inverseKinematics(const CartesianPose& targetPose, JointAngles& outAngles);
    
    // Analytische inverse Kinematik
    bool analyticalInverseKinematics(const CartesianPose& targetPose, JointAngles& outAngles);
    
    // Methode zur Auswahl der IK-Methode
    bool inverseKinematicsSwitch(const CartesianPose& targetPose, JointAngles& outAngles, bool useAnalytical = true);

    // NEUE METHODEN für die Motor-Integration
    // Konvertiert Gelenkwinkel zu Motorschritten
    void jointAnglesToMotorSteps(const JointAngles& angles, long steps[6]);
    
    // Konvertiert Motorschritte zu Gelenkwinkeln
    void motorStepsToJointAngles(const long steps[6], JointAngles& angles);

    // Winkel-Normalisierung auf [−π, +π]
    static float normalizeAngle(float angle);

    // Neue Getter-Methode für die Konfiguration
    const RobotConfig& getConfig() const;

    // Helper methods for workspace calculations
    float getMaxReach() const;
    float getBaseHeight() const;

private:
    // Berechnet die 4×4-DH-Transformation für Gelenk i
    void computeDHMatrix(int i, float theta, float T[4][4]);

    RobotConfig  _config;
    JointAngles  _currentAngles;
    CartesianPose _currentPose;
};
