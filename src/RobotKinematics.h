#pragma once

#include <Arduino.h>
#include <math.h>

// Pose im kartesischen Raum
struct CartesianPose {
    float x, y, z;    // Position
    float yaw, pitch, roll;  // Orientierung (ZYX-Euler-Winkel)
};

// Gelenkwinkel-Container
struct JointAngles {
    float angles[6];
};

// DH-Parameter für ein Gelenk
struct DHParams {
    float a;      // Link-Length
    float alpha;  // Link-Twist
    float d;      // Link-Offset
    float theta;  // Joint-Angle-Offset
};

// Roboter-Konfiguration (DH-Tabellen, Joint-Grenzen, Tool-Offset)
struct RobotConfig {
    DHParams   dhParams[6];       // DH-Parameter für Gelenk 0…5
    float      jointMin[6];       // minimale Gelenkwinkel (rad)
    float      jointMax[6];       // maximale Gelenkwinkel (rad)
    float      toolOffsetX;       // Werkzeug-Offset in X
    float      toolOffsetY;       // Werkzeug-Offset in Y
    float      toolOffsetZ;       // Werkzeug-Offset in Z
};

class RobotKinematics {
public:
    // NEUER Konstruktor
    RobotKinematics(const RobotConfig& config);

    // Setter/Get­ter für aktuelle Gelenkwinkel & Pose
    void          setCurrentJointAngles(const JointAngles& angles);
    JointAngles   getCurrentJointAngles() const;
    CartesianPose getCurrentPose() const;
    
    // NEW: Getter for robot configuration
    const RobotConfig& getConfig() const { return _config; }

    // Werkzeug-Offset nachträglich ändern
    void setToolOffset(float x, float y, float z);

    // Vorwärts-Kinematik: Gelenkwinkel → Pose
    CartesianPose forwardKinematics(const JointAngles& angles);

    // Invers-Kinematik: Zielpose → Gelenkwinkel
    bool inverseKinematics(const CartesianPose& targetPose, JointAngles& outAngles);

    // Reichweiten-Check (für den Wrist-Point)
    bool isPoseReachable(const CartesianPose& pose);

    // Winkel-Normalisierung auf [−π, +π]
    static float normalizeAngle(float angle);

private:
    // Berechnet die 4×4-DH-Transformation für Gelenk i
    void computeDHMatrix(int i, float theta, float T[4][4]);

    RobotConfig  _config;
    JointAngles  _currentAngles;
    CartesianPose _currentPose;
};
