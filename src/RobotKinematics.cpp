#include "RobotKinematics.h"
#include <Arduino.h>
#include <math.h>
#include "Debug.h"
#include "config.h"  // Für Zugriff auf _stepperConfig
#include "mechanical_config.h"  // Für invertMotorDirection

// Hilfs-Makros
#define RAD2DEG(x) ((x)*180.0/M_PI)
#define DEG2RAD(x) ((x)*M_PI/180.0)

RobotKinematics::RobotKinematics(const RobotConfig& config)
: _config(config)
{
// Initialisiere aktuelles Pose & Gelenkwinkel
memset(&_currentPose, 0, sizeof(_currentPose));
for (int i = 0; i < 6; ++i) _currentAngles.angles[i] = 0.0f;
}

void RobotKinematics::setCurrentJointAngles(const JointAngles& angles) {
_currentAngles = angles;
_currentPose = forwardKinematics(angles);
}
JointAngles RobotKinematics::getCurrentJointAngles() const {
return _currentAngles;
}
CartesianPose RobotKinematics::getCurrentPose() const {
return _currentPose;
}

void RobotKinematics::setToolOffset(float x, float y, float z) {
_config.toolOffsetX = x;
_config.toolOffsetY = y;
_config.toolOffsetZ = z;
}

CartesianPose RobotKinematics::forwardKinematics(const JointAngles& angles) {
float T[6][4][4];
for (int i = 0; i < 6; ++i) {
  float theta = angles.angles[i] + _config.dhParams[i].theta;
  computeDHMatrix(i, theta, T[i]);
  if (i > 0) {
    float tmp[4][4] = {0};
    for (int r = 0; r < 4; ++r)
      for (int c = 0; c < 4; ++c)
        for (int k = 0; k < 4; ++k)
          tmp[r][c] += T[i-1][r][k] * T[i][k][c];
    memcpy(T[i], tmp, sizeof(tmp));
  }
}

float (*R06)[4] = (float(*)[4])T[5];
float wx = R06[0][3], wy = R06[1][3], wz = R06[2][3];
float ex = wx
         + R06[0][0]*_config.toolOffsetX
         + R06[0][1]*_config.toolOffsetY
         + R06[0][2]*_config.toolOffsetZ;
float ey = wy
         + R06[1][0]*_config.toolOffsetX
         + R06[1][1]*_config.toolOffsetY
         + R06[1][2]*_config.toolOffsetZ;
float ez = wz
         + R06[2][0]*_config.toolOffsetX
         + R06[2][1]*_config.toolOffsetY
         + R06[2][2]*_config.toolOffsetZ;

float yaw   = atan2(R06[1][0], R06[0][0]);
float pitch = atan2(-R06[2][0], sqrt(R06[2][1]*R06[2][1] + R06[2][2]*R06[2][2]));
float roll  = atan2(R06[2][1], R06[2][2]);

CartesianPose result;
result.x     = ex;
result.y     = ey;
result.z     = ez;
result.yaw   = yaw;
result.pitch = pitch;
result.roll  = roll;

Serial.print("FK Endpose: X=");   Serial.print(ex);
Serial.print(" Y=");             Serial.print(ey);
Serial.print(" Z=");             Serial.print(ez);
Serial.print(" | Yaw=");         Serial.print(RAD2DEG(yaw));
Serial.print(" Pitch=");         Serial.print(RAD2DEG(pitch));
Serial.print(" Roll=");          Debug::println(RAD2DEG(roll));

return result;
}

bool RobotKinematics::analyticalInverseKinematics(const CartesianPose& targetPose, JointAngles& outAngles) {
  Debug::println(F("Starting analytical IK calculation"));
    
  // === 1. Positionsteil ===
  float x = targetPose.x - _config.toolOffsetX;
  float y = targetPose.y - _config.toolOffsetY;
  float z = targetPose.z - _config.toolOffsetZ;
    
  // DH-Parameter holen
  float d1 = _config.dhParams[0].d;
  float a2 = _config.dhParams[1].a;
  float a3 = _config.dhParams[2].a;
  float d6 = _config.dhParams[5].d;
    
  // Gelenk 1 (Basis)
  float theta1 = atan2(y, x);
    
  // Handgelenksposition (ohne Werkzeug)
  float wristX = x - d6 * cos(targetPose.yaw) * cos(targetPose.pitch);
  float wristY = y - d6 * sin(targetPose.yaw) * cos(targetPose.pitch);
  float wristZ = z - d6 * sin(targetPose.pitch) - d1;
    
  // Debug-Ausgabe
  Debug::print(F("Wrist position: X=")); Debug::print(wristX);
  Debug::print(F(" Y=")); Debug::print(wristY);
  Debug::print(F(" Z=")); Debug::println(wristZ);
    
  // Distanzberechnungen für Gelenk 2 und 3
  float r = sqrt(wristX*wristX + wristY*wristY);
  float s = wristZ;
  float D = (r*r + s*s - a2*a2 - a3*a3) / (2.0f * a2 * a3);
    
  // Reichweitenprüfung
  if (D < -1.0f || D > 1.0f) {
      Debug::println(F("Analytical IK: Target out of reach"));
      return false;
  }
    
  // Gelenk 3 (Ellbogen)
  float theta3 = atan2(-sqrt(1.0f - D*D), D);  // Negative Lösung (Ellbogen oben)
    
  // Gelenk 2 (Schulter)
  float theta2 = atan2(s, r) - atan2(a3 * sin(theta3), a2 + a3 * cos(theta3));
    
  // === 2. Orientierungsteil: Berechne R06 aus Roll/Pitch/Yaw ===
  float cr = cos(targetPose.roll), sr = sin(targetPose.roll);
  float cp = cos(targetPose.pitch), sp = sin(targetPose.pitch);
  float cy = cos(targetPose.yaw), sy = sin(targetPose.yaw);
    
  float R06[3][3] = {
      {cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr},
      {sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr},
      {-sp,   cp*sr,            cp*cr}
  };
    
  // === 3. R03 berechnen (FK bis Handgelenk) ===
  float ct1 = cos(theta1), st1 = sin(theta1);
  float ct23 = cos(theta2 + theta3), st23 = sin(theta2 + theta3);
    
  float R03[3][3] = {
      {ct1*ct23, -ct1*st23, st1},
      {st1*ct23, -st1*st23, -ct1},
      {st23,     ct23,      0}
  };
    
  // === 4. R06 = R03^T * R06 ===
  float R36[3][3] = {{0}};
  for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++) {
          R36[i][j] = 0;
          for (int k = 0; k < 3; k++) {
              R36[i][j] += R03[k][i] * R06[k][j]; // Transponiertes R03
          }
      }
  }
    
  // === 5. Extrahiere Theta4–6 aus R36 (Z-Y-Z Konvention) ===
  float theta5 = atan2(sqrt(R36[0][2]*R36[0][2] + R36[1][2]*R36[1][2]), R36[2][2]);
    
  // Behandle Singularitäten
  float theta4, theta6;
  if (fabs(theta5) < 0.001f) {
      // Singularität: Theta4 + Theta6 sind nicht eindeutig bestimmbar
      // Wir setzen Theta4 = 0 und berechnen Theta6
      Debug::println(F("Analytical IK: Wrist singularity detected"));
      theta4 = 0.0f;
      theta6 = atan2(R36[1][0], R36[0][0]);
  } else {
      theta4 = atan2(R36[1][2], R36[0][2]);
      theta6 = atan2(R36[2][1], -R36[2][0]);
  }
    
  // Gelenkwinkel-Offsets anwenden
  theta1 -= _config.dhParams[0].theta;
  theta2 -= _config.dhParams[1].theta;
  theta3 -= _config.dhParams[2].theta;
  theta4 -= _config.dhParams[3].theta;
  theta5 -= _config.dhParams[4].theta;
  theta6 -= _config.dhParams[5].theta;
    
  // Auf NaN prüfen
  if (isnan(theta1) || isnan(theta2) || isnan(theta3) ||
      isnan(theta4) || isnan(theta5) || isnan(theta6)) {
      Debug::println(F("Analytical IK: NaN detected in joint angles"));
      return false;
  }
    
  // Gelenkwinkel normalisieren und in Ergebnis speichern
  outAngles.angles[0] = normalizeAngle(theta1);
  outAngles.angles[1] = normalizeAngle(theta2);
  outAngles.angles[2] = normalizeAngle(theta3);
  outAngles.angles[3] = normalizeAngle(theta4);
  outAngles.angles[4] = normalizeAngle(theta5);
  outAngles.angles[5] = normalizeAngle(theta6);
    
  // Gelenkgrenzen prüfen
  for (int i = 0; i < 6; i++) {
      if (outAngles.angles[i] < _config.jointMin[i] || 
          outAngles.angles[i] > _config.jointMax[i]) {
          Debug::print(F("Analytical IK: Joint "));
          Debug::print(i+1);
          Debug::println(F(" exceeds limit"));
          return false;
      }
  }
    
  Debug::println(F("Analytical IK successful"));
  return true;
}

// Neue Methode zur Auswahl zwischen analytischer und numerischer IK
bool RobotKinematics::inverseKinematicsSwitch(const CartesianPose& targetPose, 
                                           JointAngles& outAngles, 
                                           bool useAnalytical) {
  if (useAnalytical) {
      // Versuche zuerst analytische IK
      if (analyticalInverseKinematics(targetPose, outAngles)) {
          return true;
      }
      // Wenn analytische IK fehlschlägt, verwende numerische als Fallback
      Debug::println(F("Analytical IK failed, trying numerical IK"));
      return inverseKinematics(targetPose, outAngles);
  } else {
      // Verwende nur numerische IK
      return inverseKinematics(targetPose, outAngles);
  }
}

float RobotKinematics::normalizeAngle(float angle) {
while (angle >  M_PI) angle -= 2*M_PI;
while (angle < -M_PI) angle += 2*M_PI;
return angle;
}

void RobotKinematics::computeDHMatrix(int i, float theta, float T[4][4]) {
float a     = _config.dhParams[i].a;
float alpha = _config.dhParams[i].alpha;
float d     = _config.dhParams[i].d;
T[0][0] = cos(theta);             T[0][1] = -sin(theta)*cos(alpha); T[0][2] = sin(theta)*sin(alpha);  T[0][3] = a*cos(theta);
T[1][0] = sin(theta);             T[1][1] = cos(theta)*cos(alpha);  T[1][2] = -cos(theta)*sin(alpha); T[1][3] = a*sin(theta);
T[2][0] = 0;                      T[2][1] = sin(alpha);             T[2][2] = cos(alpha);             T[2][3] = d;
T[3][0] = 0;                      T[3][1] = 0;                      T[3][2] = 0;                      T[3][3] = 1;
}
bool RobotKinematics::inverseKinematics(const CartesianPose& targetPose, JointAngles& outAngles) {
  // 1) Tool-Offset abziehen
  float px = targetPose.x - _config.toolOffsetX;
  float py = targetPose.y - _config.toolOffsetY;
  float pz = targetPose.z - _config.toolOffsetZ;

  // 2) Rotationsmatrix aus Yaw/Pitch/Roll
  float cy = cos(targetPose.yaw),   sy = sin(targetPose.yaw);
  float cp = cos(targetPose.pitch), sp = sin(targetPose.pitch);
  float cr = cos(targetPose.roll),  sr = sin(targetPose.roll);
  float R06[3][3] = {
    { cy*cp,    cy*sp*sr - sy*cr,    cy*sp*cr + sy*sr },
    { sy*cp,    sy*sp*sr + cy*cr,    sy*sp*cr - cy*sr },
    { -sp,      cp*sr,               cp*cr           }
  };

  // 3) Handgelenkposition
  float a4 = _config.dhParams[5].d;
  float wx = px - a4 * R06[0][2];
  float wy = py - a4 * R06[1][2];
  float wz = pz - a4 * R06[2][2];

  // 4) Reichweiten-Check
  float a2 = _config.dhParams[1].a;
  float a3 = _config.dhParams[2].a;
  float d  = sqrt(wx*wx + wy*wy + wz*wz);
  if (d > (a2 + a3) || d < fabs(a2 - a3)) {
    Debug::println("IK fehlerhaft: außerhalb Reichweite");
    return false;
  }

  // 5) q1–q3 berechnen
  float q1 = atan2(wy, wx);
  float d1 = _config.dhParams[0].d;
  float r  = sqrt(wx*wx + wy*wy);
  float s  = wz - d1;
  float D  = (r*r + s*s - a2*a2 - a3*a3) / (2.0f * a2 * a3);
  D = constrain(D, -1.0f, 1.0f);
  float q3 = atan2(sqrt(1.0f - D*D), D);
  float q2 = atan2(s, r)
           - atan2(a3 * sin(q3), a2 + a3 * cos(q3));

  // 6) R03 bauen
  float T01[4][4], T12[4][4], T23[4][4];
  computeDHMatrix(0, q1 + _config.dhParams[0].theta, T01);
  computeDHMatrix(1, q2 + _config.dhParams[1].theta, T12);
  computeDHMatrix(2, q3 + _config.dhParams[2].theta, T23);
  float R01[3][3], R12[3][3], R23[3][3], R02[3][3]={0}, R03[3][3]={0};
  for(int i=0;i<3;i++) for(int j=0;j<3;j++){
    R01[i][j] = T01[i][j];
    R12[i][j] = T12[i][j];
    R23[i][j] = T23[i][j];
    for(int k=0;k<3;k++){
      R02[i][j] += R01[i][k]*R12[k][j];
      R03[i][j] += R02[i][k]*R23[k][j];
    }
  }

  // 7) R36 = R03^T * R06
  float R03T[3][3], R36[3][3]={0};
  for(int i=0;i<3;i++) for(int j=0;j<3;j++){
    R03T[i][j] = R03[j][i];
    for(int k=0;k<3;k++){
      R36[i][j] += R03T[i][k] * R06[k][j];
    }
  }

  // 8) q4–q6 aus R36 mit Singulärfall-Fallback
  float q4, q5, q6;
  float sin_q5 = sqrt(R36[0][2]*R36[0][2] + R36[1][2]*R36[1][2]);
  q5 = atan2(sin_q5, R36[2][2]);
  if (fabs(sin_q5) < 1e-6) {
    // Singulärfall: q4 und q6 nicht eindeutig → setze q4=0, q6 aus Gesamtrotation um Z
    q4 = 0.0f;
    q6 = atan2(R36[0][1], R36[1][1]);
  } else {
    q4 = atan2(R36[1][2], R36[0][2]);
    q6 = atan2(R36[2][1], -R36[2][0]);
  }

  // 9) Auf NaN prüfen
  if (isnan(q1) || isnan(q2) || isnan(q3) ||
      isnan(q4) || isnan(q5) || isnan(q6)) {
    return false;
  }

  // 10) Winkel normalisieren und ausgeben
  outAngles.angles[0] = normalizeAngle(q1);
  outAngles.angles[1] = normalizeAngle(q2);
  outAngles.angles[2] = normalizeAngle(q3);
  outAngles.angles[3] = normalizeAngle(q4);
  outAngles.angles[4] = normalizeAngle(q5);
  outAngles.angles[5] = normalizeAngle(q6);

  return true;
}

// Achte auf die genaue Signatur mit RobotKinematics::
void RobotKinematics::jointAnglesToMotorSteps(const JointAngles& angles, long steps[6]) {
  for (int i = 0; i < 6; i++) {
    float degrees = angles.angles[i] * 180.0f / M_PI; // rad zu deg
    
    // Invertierung der Drehrichtung berücksichtigen
    if (invertMotorDirection[i]) {
      degrees = -degrees;
    }
    
    // Schritte berechnen
    steps[i] = (long)(degrees * _stepperConfig[i].stepsPerDegree);
  }
}

// Exakt die gleiche Signatur wie in der Header-Datei
void RobotKinematics::motorStepsToJointAngles(const long steps[6], JointAngles& angles) {
  for (int i = 0; i < 6; i++) {
    float degrees = (float)steps[i] / _stepperConfig[i].stepsPerDegree;
    
    // Invertierung rückgängig machen
    if (invertMotorDirection[i]) {
      degrees = -degrees;
    }
    
    angles.angles[i] = degrees * M_PI / 180.0f; // deg zu rad
  }
}

// Getter für die Konfiguration
const RobotConfig& RobotKinematics::getConfig() const {
    return _config;
}

float RobotKinematics::getMaxReach() const {
    // Maximum reach is the sum of the two main arm segments
    return _config.dhParams[1].a + _config.dhParams[2].a;
}

float RobotKinematics::getBaseHeight() const {
    // Base height is the first joint's d parameter (z offset)
    return _config.dhParams[0].d;
}
