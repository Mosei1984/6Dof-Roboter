#include "JoystickSystem.h"
#include "Debug.h"
#include "config.h"
#include "RobotSystem.h"
#include "StepperSystem.h"
#include "DisplaySystem.h"
#include "kalmanfilter.h"

extern JoystickConfig _joystickConfig;
extern PinConfig _pinConfig;

namespace JoystickSystem {
    // Joystick instances
    Joystick* leftJoystick = nullptr;
    Joystick* rightJoystick = nullptr;
    
    // Joystick states for detecting direction changes
    static JoystickState joystickXState = JOYSTICK_CENTERED;
    static JoystickState joystickYState = JOYSTICK_CENTERED;
    
    // Button state tracking
    static bool rightButtonPressed = false;
    static bool leftButtonPressed = false;
    
    // Timing
    static unsigned long lastButtonCheckTime = 0;
    
    // Kalman filters for smooth movements
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
        leftJoystick->calibrate();
        rightJoystick->calibrate();
        Debug::println(F("Basic joystick calibration complete"));
    }
    
    // Full calibration process
    void startFullCalibration() {
        Debug::println(F("Starting full joystick calibration"));
        DisplaySystem::showCalibrationInProgress();
        
        // Start the full calibration for both joysticks
        leftJoystick->startCalibration();
        rightJoystick->startCalibration();
        
        DisplaySystem::showCalibrationComplete();
        Debug::println(F("Full joystick calibration complete"));
    }
    
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
            if (StepperSystem::getSelectedJoint() < 5) {
                StepperSystem::setSelectedJoint(StepperSystem::getSelectedJoint() + 1);
                Debug::print(F("Joint increased to: "));
                Debug::println(StepperSystem::getSelectedJoint() + 1);
            }
        }
        else if (rightX < -0.7f && joystickXState == JOYSTICK_CENTERED) {
            joystickXState = JOYSTICK_LEFT;
            if (StepperSystem::getSelectedJoint() > 0) {
                StepperSystem::setSelectedJoint(StepperSystem::getSelectedJoint() - 1);
                Debug::print(F("Joint decreased to: "));
                Debug::println(StepperSystem::getSelectedJoint() + 1);
            }
        }
        else if (fabs(rightX) < 0.4f) {
            joystickXState = JOYSTICK_CENTERED;
        }
        
        // Joint movement with left joystick X (with deadband)
        float rawX = leftJoystick->getXWithDeadband(_joystickConfig.deadband);
        // Normalize to [-1..+1]
        float norm = (rawX - 500.0f) / 500.0f;
        
        Debug::print(F("Joint mode: rawX w/ deadzone = "));
        Debug::print(rawX);
        Debug::print(F(" → norm = "));
        Debug::println(norm);
        
        int selectedJoint = StepperSystem::getSelectedJoint();
        
        // Set speed
        if (rawX == 0.0f) {
            // In deadzone = stop
            StepperSystem::steppers[selectedJoint]->setSpeed(0);
        } else {
            // Full sensitivity: ±maxSpeed at |norm|==1
            float spd = -norm * _stepperConfig[selectedJoint].maxSpeed;
            Debug::print(F(" -> Setting speed for joint "));
            Debug::print(selectedJoint);
            Debug::print(F(": "));
            Debug::println(spd);
            StepperSystem::steppers[selectedJoint]->setSpeed(spd);
        }
        
        // Run motor continuously
        StepperSystem::steppers[selectedJoint]->runSpeed();
    }
    
    void processKinematicControl() {
        if (!leftJoystick || !rightJoystick) return;
        Debug::println(F("=== processKinematicControl() start ==="));
        
        // 1) Joystick raw values
        float leftX = leftJoystick->getNormalizedX();
        float leftY = leftJoystick->getNormalizedY();
        float rightX = rightJoystick->getNormalizedX();
        float rightY = rightJoystick->getNormalizedY();
        
        // Filter reset when stopping
        static bool wasMoving = false;
        bool isMoving = (fabs(leftX) > 0.01f) || (fabs(leftY) > 0.01f) ||
                        (fabs(rightX) > 0.01f) || (fabs(rightY) > 0.01f);
        
        if (!isMoving && wasMoving) {
            // Joystick released -> reset filters
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
        
        // 2) Dynamically set speed based on joystick movement
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
        
        // 3) Calculate changes via filters
        CartesianPose current = RobotSystem::getKinematics()->getCurrentPose();
        float xChg = positionFilterX.update(leftX * 1.0f);
        float yChg = positionFilterY.update(-leftY * 1.0f);
        float zChg = positionFilterZ.update(-rightY * 1.0f);
        float yawChg = rotationFilter.update(rightX * 0.03f);
        
        Debug::print(F("  Deltas: xChg=")); Debug::print(xChg, 3);
        Debug::print(F("  yChg=")); Debug::print(yChg, 3);
        Debug::print(F("  zChg=")); Debug::print(zChg, 3);
        Debug::print(F("  yawChg=")); Debug::println(yawChg, 4);
        
        // 5) Only move if there's significant change
        if (fabs(xChg) > 0.01f || fabs(yChg) > 0.01f ||
            fabs(zChg) > 0.01f || fabs(yawChg) > 0.001f) {
            
            // 6) Calculate target pose and clamp
            CartesianPose target = current;
            target.x += xChg;
            target.y += yChg;
            target.z += zChg;
            target.yaw += yawChg;
            
            // Get workspace limits from kinematics
            float maxReach = RobotSystem::getKinematics()->getMaxReach();
            float minZ = RobotSystem::getKinematics()->getBaseHeight();
            float maxZ = minZ + maxReach;
            
            target.x = constrain(target.x, -maxReach, +maxReach);
            target.y = constrain(target.y, -maxReach, +maxReach);
            target.z = constrain(target.z, minZ, maxZ);
            
            Debug::print(F("  Kinematic target: X=")); Debug::print(target.x, 2);
            Debug::print(F("  Y=")); Debug::print(target.y, 2);
            Debug::print(F("  Z=")); Debug::println(target.z, 2);
            Debug::print(F("               Yaw=")); Debug::println(target.yaw, 3);
            
            // 7) Call IK
            JointAngles newAngles;
            if (RobotSystem::getKinematics()->inverseKinematics(target, newAngles)) {
                for (int i = 0; i < 6; ++i) {
                    float deg = newAngles.angles[i] * 180.0 / M_PI;
                    long steps = deg * _stepperConfig[i].stepsPerDegree;
                    StepperSystem::steppers[i]->moveTo(steps);
                }
                
                // Update kinematics
                RobotSystem::getKinematics()->setCurrentJointAngles(newAngles);
            } else {
                Debug::println(F("  IK error: Movement skipped"));
                // Set stepper targets to current position to prevent continued movement
                for (int i = 0; i < 6; ++i) {
                    StepperSystem::steppers[i]->moveTo(StepperSystem::steppers[i]->currentPosition());
                }
            }
        }
        
        Debug::println(F("=== processKinematicControl() end ==="));
    }
    
    void processButtonInput() {
        if (!leftJoystick || !rightJoystick) return;
        
        bool leftPressed = leftJoystick->isPressed();
        bool rightPressed = rightJoystick->isPressed();
        
        // Debounce
        if (millis() - lastButtonCheckTime < 200) return;
        
        // Right button: Mode switch
        if (rightPressed && !rightButtonPressed) {
            if (millis() - RobotSystem::getStateChangeTime() > 500) {
                rightButtonPressed = true;
                lastButtonCheckTime = millis();
                
                // Cycle through modes
                RobotSystem::SystemState currentState = RobotSystem::getState();
                
                if (currentState == RobotSystem::STATE_JOINT_MODE) {
                    RobotSystem::setState(RobotSystem::STATE_KINEMATIC_MODE);
                } else if (currentState == RobotSystem::STATE_KINEMATIC_MODE) {
                    // Clean exit from kinematic mode
                    for (int i = 0; i < 6; ++i) {
                        StepperSystem::steppers[i]->moveTo(StepperSystem::steppers[i]->currentPosition());
                    }
                    StepperSystem::synchronizeWithKinematics();
                    RobotSystem::setState(RobotSystem::STATE_HOMING_MODE);
                    RobotSystem::setHomingStarted(false);
                    RobotSystem::setHomingJointIndex(0);
                } else if (currentState == RobotSystem::STATE_HOMING_MODE) {
                    RobotSystem::setState(RobotSystem::STATE_GEAR_CONFIG_MODE);
                    DisplaySystem::setGearMenuActive(true);
                } else if (currentState == RobotSystem::STATE_GEAR_CONFIG_MODE) {
                    DisplaySystem::setGearMenuActive(false);
                    RobotSystem::setState(RobotSystem::STATE_CALIBRATION_MODE);
                } else if (currentState == RobotSystem::STATE_CALIBRATION_MODE) {
                    RobotSystem::setState(RobotSystem::STATE_JOINT_MODE);
                }
                
                Debug::print(F("Mode changed to: "));
                Debug::println(RobotSystem::getState());
                RobotSystem::setStateChangeTime(millis());
            }
        } else if (!rightPressed) {
            rightButtonPressed = false;
        }
        
        // Left button: Action in current mode
        if (leftPressed && !leftButtonPressed) {
            if (millis() - RobotSystem::getStateChangeTime() > 500) {
                leftButtonPressed = true;
                lastButtonCheckTime = millis();
                
                RobotSystem::SystemState currentState = RobotSystem::getState();
                
                if (currentState == RobotSystem::STATE_HOMING_MODE && !RobotSystem::isHomingStarted()) {
                    RobotSystem::setHomingStarted(true);
                    Debug::println(F("Homing started"));
                } else if (currentState == RobotSystem::STATE_CALIBRATION_MODE) {
                    Debug::println(F("Starting manual calibration"));
                    DisplaySystem::showCalibrationInProgress();
                    startFullCalibration();
                }
                
                RobotSystem::setStateChangeTime(millis());
            }
        } else if (!leftPressed) {
            leftButtonPressed = false;
            RobotSystem::setCalibrationLocked(false);
        }
    }
    
    JoystickState getJoystickXState() {
        return joystickXState;
    }
    
    JoystickState getJoystickYState() {
        return joystickYState;
    }
    
    bool isLeftButtonPressed() {
        return leftJoystick && leftJoystick->isPressed();
    }
    
    bool isRightButtonPressed() {
        return rightJoystick && rightJoystick->isPressed();
    }
}

