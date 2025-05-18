#include "JoystickSystem.h"
#include "Debug.h"
#include "config.h"
#include "RobotSystem.h"
#include "StepperSystem.h"
#include "DisplaySystem.h"

namespace RobotSystem {
    void processHomingMenuSelection();
}

namespace JoystickSystem {
    // Joystick objects
    Joystick* leftJoystick = nullptr;
    Joystick* rightJoystick = nullptr;
    
    // Selected joint for joint mode
    static int selectedJoint = 0;
    
    // Current joystick state for joint selection
    static JoystickState joystickXState = JOYSTICK_CENTERED;
    
    // Button state tracking
    static bool rightButtonPressed = false;
    static bool leftButtonPressed = false;  // Track left button too
    static unsigned long lastButtonCheckTime = 0;
    
    // Kalman filters for smooth movement
    static KalmanFilter positionFilterX(0.01, 0.1);
    static KalmanFilter positionFilterY(0.01, 0.1);
    static KalmanFilter positionFilterZ(0.01, 0.1);
    static KalmanFilter rotationFilter(0.01, 0.1);
    
    void init() {
        Debug::println(F("Initializing joystick system..."));
        
        // Create joystick objects
        leftJoystick = new Joystick(_pinConfig.leftXPin, _pinConfig.leftYPin, _pinConfig.leftBtnPin);
        rightJoystick = new Joystick(_pinConfig.rightXPin, _pinConfig.rightYPin, _pinConfig.rightBtnPin);
        
        // Initialize joysticks
        leftJoystick->begin();
        rightJoystick->begin();
        
        // Perform basic calibration
        calibrateJoysticks();
        
        Debug::println(F("Joystick system initialized"));
    }
    
    void update() {
        // Read joystick inputs
        if (leftJoystick) leftJoystick->read();
        if (rightJoystick) rightJoystick->read();
        
        // ALWAYS check button inputs in every update cycle
        processButtonInput();
    }
    
    // Basic joystick calibration
    void calibrateJoysticks() {
        if (leftJoystick && rightJoystick) {
            Debug::println(F("Performing basic joystick calibration..."));
            leftJoystick->calibrate();
            rightJoystick->calibrate();
            Debug::println(F("Basic calibration complete"));
        } else {
            Debug::println(F("ERROR: Cannot calibrate - joysticks not initialized!"));
        }
    }
    
    // Full joystick calibration
    void startFullCalibration() {
        if (leftJoystick && rightJoystick) {
            Debug::println(F("Starting full joystick calibration..."));
            
            // Start the extended calibration for both joysticks
            leftJoystick->startCalibration();
            rightJoystick->startCalibration();
            
            Debug::println(F("Full calibration complete"));
        } else {
            Debug::println(F("ERROR: Cannot calibrate - joysticks not initialized!"));
        }
    }
    
    // Get selected joint
    int getSelectedJoint() {
        return selectedJoint;
    }
    
    // Set selected joint
    void setSelectedJoint(int joint) {
        selectedJoint = joint;
    }

    // Process button input for mode changes
    void processButtonInput() {
        if (!leftJoystick || !rightJoystick) return;
        
        // Aktuelle Tastendrücke auslesen
        bool leftCurrentlyPressed = leftJoystick->isPressed();
        bool rightCurrentlyPressed = rightJoystick->isPressed();
        
        // Aktuellen Systemzustand holen
        RobotSystem::SystemState currentState = RobotSystem::getState();
        unsigned long stateChangeTime = RobotSystem::getStateChangeTime();
        
        // Entprellen: Verzögerung zwischen Button-Aktionen erzwingen
        unsigned long now = millis();
        if (now - lastButtonCheckTime < 200) return;
        
        // ===== RECHTER BUTTON: MODUS-WECHSEL =====
        // Steigende Flanke erkennen (nicht gedrückt -> gedrückt)
        if (rightCurrentlyPressed && !rightButtonPressed) {
            rightButtonPressed = true;  // Status aktualisieren
            
            // Nur wenn genug Zeit seit dem letzten Statuswechsel vergangen ist
            if (now - stateChangeTime > 500) {
                lastButtonCheckTime = now;  // Zeitstempel aktualisieren
                
                // Debug Ausgabe VOR der Änderung
                Debug::print(F("Mode changing from: "));
                Debug::println(currentState);
                
                // Modiwechsel durchführen
                switch (currentState) {
                    case RobotSystem::STATE_JOINT_MODE:
                        RobotSystem::setState(RobotSystem::STATE_KINEMATIC_MODE);
                        break;
                        
                    case RobotSystem::STATE_KINEMATIC_MODE:
                        // Kinematik-Modus sauber verlassen
                        exitKinematicMode();
                        RobotSystem::setState(RobotSystem::STATE_HOMING_MODE);
                        RobotSystem::setHomingStarted(false);
                        RobotSystem::setHomingJointIndex(0);
                        break;
                        
                    case RobotSystem::STATE_HOMING_MODE:
                        RobotSystem::setState(RobotSystem::STATE_CALIBRATION_MODE);
                        break;
                        
                    case RobotSystem::STATE_CALIBRATION_MODE:
                        RobotSystem::setState(RobotSystem::STATE_JOINT_MODE);
                        break;
                        
                    default:
                        // Bei unbekanntem Zustand auf JOINT_MODE wechseln
                        RobotSystem::setState(RobotSystem::STATE_JOINT_MODE);
                        break;
                }
                
                // Debug Ausgabe NACH der Änderung
                Debug::print(F("State changed to: "));
                Debug::println(RobotSystem::getState());
                RobotSystem::setStateChangeTime(now);
            }
        }
        // Fallende Flanke des rechten Buttons erkennen
        else if (!rightCurrentlyPressed && rightButtonPressed) {
            rightButtonPressed = false;
        }
        
        // ===== LINKER BUTTON: AKTIONEN IM MODUS =====
        // Steigende Flanke erkennen
        if (leftCurrentlyPressed && !leftButtonPressed) {
            leftButtonPressed = true;  // Status aktualisieren
            
            // Nur wenn genug Zeit seit dem letzten Statuswechsel vergangen ist
            if (now - stateChangeTime > 500) {
                lastButtonCheckTime = now;  // Zeitstempel aktualisieren
                
                // Aktionen je nach Modus
                switch (currentState) {
                    case RobotSystem::STATE_HOMING_MODE:
                        if (!RobotSystem::isHomingStarted()) {
                            Debug::println(F("Homing Menu: Processing selection"));
                            RobotSystem::processHomingMenuSelection();
                        }
                        break;
                        
                    case RobotSystem::STATE_CALIBRATION_MODE:
                        Debug::println(F("Starting manual calibration"));
                        DisplaySystem::showMessage("CALIBRATING...", "Move joysticks to all positions");
                        startFullCalibration();
                        DisplaySystem::showMessage("CALIBRATION COMPLETE", "Press right button to continue");
                        delay(2000);
                        break;
                        
                    default:
                        // Keine spezielle Aktion für andere Modi
                        break;
                }
                
                RobotSystem::setStateChangeTime(now);
            }
        }
        // Fallende Flanke des linken Buttons erkennen
        else if (!leftCurrentlyPressed && leftButtonPressed) {
            leftButtonPressed = false;
            RobotSystem::setCalibrationLocked(false);
        }
    }
    
    // Process joint control mode
    void processJointControl() {
        if (!leftJoystick || !rightJoystick) return;
        
        // Debug output
        Debug::print(F("Joystick values: Left X: "));
        Debug::print(leftJoystick->getX());
        Debug::print(F(", Left Y: "));
        Debug::println(leftJoystick->getY());
        
        // Joint selection with right joystick X
        float rightX = (rightJoystick->getX() - 500.0f) / 500.0f;  // to [-1..+1]
        if (rightX > 0.7f && joystickXState == JOYSTICK_CENTERED) {
            joystickXState = JOYSTICK_RIGHT;
            if (selectedJoint < 5) {
                selectedJoint++;
                Debug::print(F("Joint increased to: "));
                Debug::println(selectedJoint + 1);
            }
        }
        else if (rightX < -0.7f && joystickXState == JOYSTICK_CENTERED) {
            joystickXState = JOYSTICK_LEFT;
            if (selectedJoint > 0) {
                selectedJoint--;
                Debug::print(F("Joint decreased to: "));
                Debug::println(selectedJoint + 1);
            }
        }
        else if (fabs(rightX) < 0.4f) {
            joystickXState = JOYSTICK_CENTERED;
        }
        
        // Joint movement with left joystick X (with deadband)
        float rawX = leftJoystick->getX();
        float deadband = _joystickConfig.deadband;
        
        // Apply deadband
        if (fabs(rawX - 500.0f) < deadband) {
            rawX = 500.0f;  // center value
        }
        
        // Normalize to [-1..+1]
        float norm = (rawX - 500.0f) / 500.0f;
        Debug::print(F("Joint mode: rawX w/ deadzone = "));
        Debug::print(rawX);
        Debug::print(F(" → norm = "));
        Debug::println(norm);
        
        // Set speed
        if (fabs(norm) < 0.01f) {  // effectively zero
            StepperSystem::steppers[selectedJoint]->setSpeed(0);
        } else {
            // Full sensitivity: ±maxSpeed at |norm|==1
            float spd = -norm * _stepperConfig[selectedJoint].maxSpeed;
            Debug::print(F(" -> Setting speed for Joint "));
            Debug::print(selectedJoint);
            Debug::print(F(": "));
            Debug::println(spd);
            StepperSystem::steppers[selectedJoint]->setSpeed(spd);
        }
        
        // Run motor continuously
        StepperSystem::steppers[selectedJoint]->runSpeed();
    }
    
    // Process kinematic control mode
    void processKinematicControl() {
        if (!leftJoystick || !rightJoystick) return;
        
        Debug::println(F("=== processKinematicControl() start ==="));
        
        // Get joystick values
        float leftX = leftJoystick->getNormalizedX();
        float leftY = leftJoystick->getNormalizedY();
        float rightX = rightJoystick->getNormalizedX();
        float rightY = rightJoystick->getNormalizedY();
        
        // Reset filter when joysticks are released
        static bool wasMoving = false;
        bool isMoving = (fabs(leftX) > 0.01f) || (fabs(leftY) > 0.01f) ||
                        (fabs(rightX) > 0.01f) || (fabs(rightY) > 0.01f);
        
        static KalmanFilter positionFilterX(0.01, 0.1);
        static KalmanFilter positionFilterY(0.01, 0.1);
        static KalmanFilter positionFilterZ(0.01, 0.1);
        static KalmanFilter rotationFilter(0.01, 0.1);
        
        if (!isMoving && wasMoving) {
            // Joystick was released - reset filters
            positionFilterX.reset();
            positionFilterY.reset();
            positionFilterZ.reset();
            rotationFilter.reset();
            Debug::println(F("Joystick released, filters reset!"));
        }
        wasMoving = isMoving;
        
        Debug::print(F("  Joystick: Lx=")); Debug::print(leftX, 2);
        Debug::print(F("  Ly=")); Debug::print(leftY, 2);
        Debug::print(F("  Rx=")); Debug::print(rightX, 2);
        Debug::print(F("  Ry=")); Debug::println(rightY, 2);
        
        // Dynamically set speed based on joystick movement
        float speedFactor = max(max(fabs(leftX), fabs(leftY)),
                               max(fabs(rightX), fabs(rightY)));
        float dynSpd = speedFactor * 100.0f;
        float dynAcc = dynSpd * 2.0f;
        for (int i = 0; i < 6; ++i) {
            StepperSystem::steppers[i]->setMaxSpeed(dynSpd);
            StepperSystem::steppers[i]->setAcceleration(dynAcc);
        }
        
        Debug::print(F("  dynSpd=")); Debug::print(dynSpd, 1);
        Debug::print(F("  dynAcc=")); Debug::println(dynAcc, 1);
        
        // Calculate changes via filters
        CartesianPose current = RobotSystem::getKinematics()->getCurrentPose();
        float xChg = positionFilterX.update(leftX * 1.0f);
        float yChg = positionFilterY.update(-leftY * 1.0f);
        float zChg = positionFilterZ.update(-rightY * 1.0f);
        float yawChg = rotationFilter.update(rightX * 0.03f);
        
        Debug::print(F("  Deltas: xChg=")); Debug::print(xChg, 3);
        Debug::print(F("  yChg=")); Debug::print(yChg, 3);
        Debug::print(F("  zChg=")); Debug::print(zChg, 3);
        Debug::print(F("  yawChg=")); Debug::println(yawChg, 4);
        
        // Only move if there's significant change
        if (fabs(xChg) > 0.01f || fabs(yChg) > 0.01f ||
            fabs(zChg) > 0.01f || fabs(yawChg) > 0.001f) {
            
            // Calculate target pose
            CartesianPose target = current;
            target.x += xChg;
            target.y += yChg;
            target.z += zChg;
            target.yaw += yawChg;
            
            // Apply workspace limits (simplified)
            RobotConfig robotConfig = RobotSystem::getKinematics()->getConfig();
            const float a1 = robotConfig.dhParams[1].a;
            const float a2 = robotConfig.dhParams[2].a;
            const float maxRad = a1 + a2;
            const float minZ = robotConfig.dhParams[0].d;
            const float maxZ = minZ + maxRad;
            
            target.x = constrain(target.x, -maxRad, +maxRad);
            target.y = constrain(target.y, -maxRad, +maxRad);
            target.z = constrain(target.z, minZ, maxZ);
            
            Debug::print(F("  Kinematic target: X=")); Debug::print(target.x, 2);
            Debug::print(F("  Y=")); Debug::print(target.y, 2);
            Debug::print(F("  Z=")); Debug::println(target.z, 2);
            Debug::print(F("               Yaw=")); Debug::println(target.yaw, 3);
            
            // Call inverse kinematics
            JointAngles newAngles;
            if (RobotSystem::getKinematics()->inverseKinematics(target, newAngles)) {
                // Set stepper targets based on calculated angles
                for (int i = 0; i < 6; ++i) {
                    float deg = newAngles.angles[i] * 180.0 / M_PI;
                    long steps = deg * _stepperConfig[i].stepsPerDegree;
                    StepperSystem::steppers[i]->moveTo(steps);
                }
                
                // Update kinematics with new angles
                RobotSystem::getKinematics()->setCurrentJointAngles(newAngles);
            } else {
                Debug::println(F("  IK error: Movement skipped"));
                
                // Reset stepper targets to current position
                for (int i = 0; i < 6; ++i) {
                    StepperSystem::steppers[i]->moveTo(StepperSystem::steppers[i]->currentPosition());
                }
            }
        } // End threshold-IF
        
        Debug::println(F("=== processKinematicControl() end ==="));
    }
    
    // Additional utility function to exit kinematic mode
    void exitKinematicMode() {
        // Stop all motors at current position
        for (int i = 0; i < 6; ++i) {
            StepperSystem::steppers[i]->moveTo(StepperSystem::steppers[i]->currentPosition());
        }
        
        // Synchronize kinematics model
        StepperSystem::synchronizeKinematicsWithSteppers();
        
        Debug::println(F("Exited kinematic mode"));
    }
}