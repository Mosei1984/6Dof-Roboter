#include "RobotSystem.h"
#include "Debug.h"
#include "config.h"
#include "JoystickSystem.h"
#include "StepperSystem.h"
#include "DisplaySystem.h"
#include <EEPROM.h>

namespace RobotSystem {
    // Global state
    static SystemState currentState = STATE_STARTUP;
    static bool calibrationLock = false;
    static unsigned long stateChangeTime = 0;
    
    // Homing state
    static int homingJointIndex = 0;
    static bool homingStarted = false;
    static int homingMenuSelection = 0;
    
    // Menu navigation variables - ADD THESE
    static unsigned long lastMenuMove = 0;
    static unsigned long lastClearHomePress = 0;
    static int clearHomePressCount = 0;
    
    // Kinematics object
    static RobotConfig robotConfig;
    static RobotKinematics* robotKin = nullptr;
    
    void init() {
        Debug::println(F("Initializing robot system..."));
        
        // Initialize robot configuration
        initRobotConfig();
        
        // Create kinematics object
        robotKin = new RobotKinematics(robotConfig);
        
        // Initialize joint angles to zeros
        JointAngles init = {{0, 0, 0, 0, 0, 0}};
        robotKin->setCurrentJointAngles(init);
        
        // Set initial state
        currentState = STATE_STARTUP;
        stateChangeTime = millis();
        
        Debug::println(F("Robot system initialized"));
    }
    
    void initRobotConfig() {
        // Joint angle limits (in degrees)
        for (int i = 0; i < 6; i++) {
            robotConfig.jointMin[i] = -180.0 * DEG_TO_RAD;
            robotConfig.jointMax[i] = 180.0 * DEG_TO_RAD;
        }
        
        // DH parameters - simple 6DOF arm
        // Format: {a, alpha, d, theta}
        robotConfig.dhParams[0] = {0.0, M_PI/2, 50.0, 0.0};     // Base
        robotConfig.dhParams[1] = {80.0, 0.0, 0.0, M_PI/2};     // Shoulder
        robotConfig.dhParams[2] = {80.0, 0.0, 0.0, 0.0};        // Elbow
        robotConfig.dhParams[3] = {0.0, M_PI/2, 80.0, 0.0};     // Wrist Pitch
        robotConfig.dhParams[4] = {0.0, -M_PI/2, 0.0, 0.0};     // Wrist Roll
        robotConfig.dhParams[5] = {0.0, 0.0, 40.0, 0.0};        // Gripper
        
        // Tool offset
        robotConfig.toolOffsetX = 0.0;
        robotConfig.toolOffsetY = 0.0;
        robotConfig.toolOffsetZ = 30.0;
    }
    
    void update() {
        // Process current system state
        processCurrentState();
    }
    
    void processCurrentState() {
        // State machine
        switch (currentState) {
            case STATE_STARTUP:
                // Startup state is just for initial display, then transition to Joint mode
                if (millis() - stateChangeTime > 2000) {
                    currentState = STATE_JOINT_MODE;
                    stateChangeTime = millis();
                    Debug::println(F("Switching to JOINT_MODE after startup"));
                }
                break;
                
            case STATE_JOINT_MODE:
                // Process button input for state changes
                JoystickSystem::processButtonInput();
                
                // Process joint control
                JoystickSystem::processJointControl();
                
                // Update display
                DisplaySystem::displayJointMode();
                break;
                
            case STATE_KINEMATIC_MODE:
                // Process button input for state changes
                JoystickSystem::processButtonInput();
                
                // Process kinematic control
                JoystickSystem::processKinematicControl();
                
                // Update display
                DisplaySystem::displayKinematicMode();
                break;
                
            case STATE_HOMING_MODE:
                // Process homing mode (implemented in RobotSystem::processHomingMode())
                processHomingMode();
                break;
                
            case STATE_CALIBRATION_MODE:
                // Process button input for state changes
                JoystickSystem::processButtonInput();
                
                // Update display
                DisplaySystem::displayCalibrationMode();
                break;
        }
    }
    
    // Getter and setter functions
    SystemState getState() {
        return currentState;
    }
    
    void setState(SystemState state) {
        currentState = state;
    }
    
    unsigned long getStateChangeTime() {
        return stateChangeTime;
    }
    
    void setStateChangeTime(unsigned long time) {
        stateChangeTime = time;
    }
    
    RobotKinematics* getKinematics() {
        return robotKin;
    }
    
    bool isCalibrationLocked() {
        return calibrationLock;
    }
    
    void setCalibrationLocked(bool locked) {
        calibrationLock = locked;
    }
    
    bool isHomingStarted() {
        return homingStarted;
    }
    
    void setHomingStarted(bool started) {
        homingStarted = started;
    }
    
    int getHomingJointIndex() {
        return homingJointIndex;
    }
    
    void setHomingJointIndex(int index) {
        homingJointIndex = index;
    }
    
    int getHomingMenuSelection() {
        return homingMenuSelection;
    }
    
    void setHomingMenuSelection(int selection) {
        homingMenuSelection = selection;
    }
    
    int getHomingMenuOptionCount() {
        return HOMING_MENU_COUNT;
    }
    
    const char* getHomingMenuOptionName(int option) {
        switch (option) {
            case HOMING_MENU_START_HOMING: return "Start Homing";
            case HOMING_MENU_TO_CENTER:    return "Go to Center";
            case HOMING_MENU_SAVE_HOME:    return "Save Home";
            case HOMING_MENU_LOAD_HOME:    return "Load Home";
            case HOMING_MENU_CLEAR_HOME:   return "Clear Home";
            default:                       return "Unknown Option";
        }
    }
    
    void saveRobotHome(const JointAngles& angles) {
        struct RobotHomeData {
            uint32_t magic;
            float jointAngles[6];
        };
        
        RobotHomeData data;
        data.magic = ROBOT_HOME_MAGIC;
        for (int i = 0; i < 6; ++i) {
            data.jointAngles[i] = angles.angles[i];
        }
        
        EEPROM.put(100, data);
        Debug::println(F("Home position saved to EEPROM"));
    }
    
    bool loadRobotHome(JointAngles& angles) {
        struct RobotHomeData {
            uint32_t magic;
            float jointAngles[6];
        };
        
        RobotHomeData data;
        EEPROM.get(100, data);
        
        if (data.magic != ROBOT_HOME_MAGIC) {
            Debug::println(F("No valid home position found in EEPROM"));
            return false;
        }
        
        for (int i = 0; i < 6; ++i) {
            angles.angles[i] = data.jointAngles[i];
        }
        
        Debug::println(F("Home position loaded from EEPROM"));
        return true;
    }
    
    void clearRobotHome() {
        struct RobotHomeData {
            uint32_t magic;
            float jointAngles[6];
        };
        
        RobotHomeData data = {0};
        EEPROM.put(100, data);
        Debug::println(F("Home position cleared from EEPROM"));
    }
    
    void processHomingMode() {
        // Check if we're in the menu or already homing
        if (!homingStarted) {
            processHomingMenu();
        } else {
            // Display progress screen
            DisplaySystem::displayHomingMode();
            
            // Process actual homing operations
            // This would typically call StepperSystem functions to home each joint
            // ...
            
            // For demonstration, we just count up the joint index
            if (homingJointIndex < 6) {
                // Call StepperSystem to home the current joint
                bool jointHomed = StepperSystem::homeJoint(homingJointIndex);
                
                if (jointHomed) {
                    homingJointIndex++;
                    Debug::print(F("Joint "));
                    Debug::print(homingJointIndex);
                    Debug::println(F(" homed"));
                    delay(500);
                }
            } else {
                // Homing completed for all joints
                Debug::println(F("Homing completed"));
                homingStarted = false;
                homingJointIndex = 0;
                homingMenuSelection = 0;
                
                // Synchronize kinematics
                JointAngles homeAngles;
                for (int i = 0; i < 6; i++) {
                    float posDegrees = StepperSystem::steppers[i]->currentPosition() / 
                                     _stepperConfig[i].stepsPerDegree;
                    homeAngles.angles[i] = posDegrees * M_PI / 180.0;
                }
                robotKin->setCurrentJointAngles(homeAngles);
            }
        }
    }
    
    void processHomingMenu() {
        // Nur Menüanzeige und Navigation, KEIN Button-Handling für Modiwechsel!
        
        // Menü anzeigen
        DisplaySystem::displayHomingMenu();

        // Menüauswahl mit rechtem Joystick Y
        float rightY = JoystickSystem::rightJoystick->getNormalizedY();
        if (millis() - lastMenuMove > 200) {
            if (rightY > 0.7f && homingMenuSelection > 0) {
                homingMenuSelection--;
                lastMenuMove = millis();
                Debug::print(F("Menu selection: "));
                Debug::println(homingMenuSelection);
            } else if (rightY < -0.7f && homingMenuSelection < HOMING_MENU_COUNT-1) {
                homingMenuSelection++;
                lastMenuMove = millis();
                Debug::print(F("Menu selection: "));
                Debug::println(homingMenuSelection);
            }
        }
        
        // Debug-Ausgabe für Diagnosezwecke
        static unsigned long lastDebugTime = 0;
        if (millis() - lastDebugTime > 1000) {  // Einmal pro Sekunde
            lastDebugTime = millis();
            Debug::print(F("HOMING MENU - Current selection: "));
            Debug::print(homingMenuSelection);
            Debug::print(F(" - System State: "));
            Debug::println(currentState);
        }
    }

    // New function to handle menu actions
    void processHomingMenuSelection() {
        Debug::print(F("Processing homing menu selection: "));
        Debug::println(homingMenuSelection);
        
        switch (homingMenuSelection) {
            case HOMING_MENU_START_HOMING:
                homingStarted = true;
                homingJointIndex = 0;
                Debug::println(F("Homing started"));
                break;
                
            case HOMING_MENU_TO_CENTER: {
                Debug::println(F("Menu: Go to center selected"));

                // Define target point in workspace
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

                Debug::print(F("Target pose: X=")); Debug::print(centerPose.x);
                Debug::print(F(" Y=")); Debug::print(centerPose.y);
                Debug::print(F(" Z=")); Debug::println(centerPose.z);

                // Calculate IK
                JointAngles centerAngles;
                if (robotKin->inverseKinematics(centerPose, centerAngles)) {
                    // Calculate target positions
                    long targetSteps[6];
                    long startSteps[6];
                    for (int i = 0; i < 6; ++i) {
                        float deg = centerAngles.angles[i] * 180.0 / M_PI;
                        targetSteps[i] = deg * _stepperConfig[i].stepsPerDegree;
                        startSteps[i] = StepperSystem::steppers[i]->currentPosition();
                        StepperSystem::steppers[i]->moveTo(targetSteps[i]);
                    }

                    // Move with progress bar
                    bool allDone = false;
                    while (!allDone) {
                        allDone = true;
                        long maxDist = 0, maxDistToGo = 0;
                        for (int i = 0; i < 6; ++i) {
                            long dist = abs(targetSteps[i] - startSteps[i]);
                            long distToGo = abs(StepperSystem::steppers[i]->distanceToGo());
                            if (dist > maxDist) maxDist = dist;
                            if (distToGo > maxDistToGo) maxDistToGo = distToGo;
                            if (StepperSystem::steppers[i]->distanceToGo() != 0) {
                                StepperSystem::steppers[i]->run();
                                allDone = false;
                            }
                        }

                        // Calculate progress (0.0 ... 1.0)
                        float progress = 1.0f;
                        if (maxDist > 0) {
                            progress = 1.0f - (float)maxDistToGo / (float)maxDist;
                            if (progress < 0) progress = 0;
                            if (progress > 1) progress = 1;
                        }

                        // Update display with progress
                        DisplaySystem::displayHomingCenterProgress(progress);
                        delay(10);
                    }

                    for (int i = 0; i < 6; ++i) {
                        StepperSystem::steppers[i]->setCurrentPosition(StepperSystem::steppers[i]->targetPosition());
                    }
                    robotKin->setCurrentJointAngles(centerAngles);

                    // FK for verification
                    CartesianPose pose = robotKin->getCurrentPose();
                    Debug::print(F("FK End pose: X=")); Debug::print(pose.x,2);
                    Debug::print(F(" Y=")); Debug::print(pose.y,2);
                    Debug::print(F(" Z=")); Debug::print(pose.z,2);
                    Debug::print(F(" | Yaw=")); Debug::print(pose.yaw*180.0/M_PI,2);
                    Debug::print(F(" Pitch=")); Debug::print(pose.pitch*180.0/M_PI,2);
                    Debug::print(F(" Roll=")); Debug::println(pose.roll*180.0/M_PI,2);

                    DisplaySystem::showMessage("Robot now at", "center position!", 1500);
                } else {
                    Debug::println(F("IK error! Target not reachable."));
                    DisplaySystem::showMessage("IK Error!", nullptr, 1500);
                }
                break;
            }
            
            case HOMING_MENU_SAVE_HOME: {
                JointAngles current = robotKin->getCurrentJointAngles();
                saveRobotHome(current);
                DisplaySystem::showMessage("Home saved!", nullptr, 1200);
                break;
            }
            
            case HOMING_MENU_LOAD_HOME: {
                JointAngles homeAngles;
                if (loadRobotHome(homeAngles)) {
                    // NO movement! Only set kinematics and stepper positions
                    robotKin->setCurrentJointAngles(homeAngles);
                    for (int i = 0; i < 6; ++i) {
                        StepperSystem::steppers[i]->setCurrentPosition(
                            homeAngles.angles[i] * 180.0 / M_PI * _stepperConfig[i].stepsPerDegree
                        );
                    }
                    Debug::println(F("Home loaded! Kinematics synchronized, no movement."));
                    DisplaySystem::showMessage("Home loaded!", "No movement.", 1200);
                } else {
                    DisplaySystem::showMessage("No home", "saved!", 1200);
                }
                break;
            }
            
            case HOMING_MENU_CLEAR_HOME: {
                if (clearHomePressCount == 0 || millis() - lastClearHomePress > 1500) {
                    clearHomePressCount = 1;
                    lastClearHomePress = millis();
                    DisplaySystem::showMessage("Press again", "to clear home!");
                } else if (clearHomePressCount == 1) {
                    clearRobotHome();
                    DisplaySystem::showMessage("Home cleared!", nullptr, 1200);
                    delay(1200);
                    clearHomePressCount = 0;
                }
                break;
            }
        }
    }
}