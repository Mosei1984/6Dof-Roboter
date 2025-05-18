#include "RobotSystem.h"
#include "Debug.h"
#include "DisplaySystem.h"
#include "JoystickSystem.h"
#include "StepperSystem.h"
#include "ConfigSystem.h"
#include <EEPROM.h>
#include <SD.h>

// External configuration references
extern PinConfig _pinConfig;
extern StepperConfig _stepperConfig[6]; 
extern AccelStepper* _steppers[6];

// Global robot configuration
RobotConfig g_robotConfig;

namespace RobotSystem {
    // Global state
    static SystemState currentState = STATE_STARTUP;
    static bool calibrationLock = false;
    static unsigned long stateChangeTime = 0;
    
    // Homing state
    static int homingJointIndex = 0;
    static bool homingStarted = false;
    static int homingMenuSelection = 0;
    
    // Menu navigation variables
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
        SystemState currState = getState();
        
        switch (currState) {
            case STATE_STARTUP:
                // Startup state is just for initial display, then transition to Joint mode
                if (millis() - stateChangeTime > 2000) {
                    setState(STATE_JOINT_MODE);
                    setStateChangeTime(millis());
                    Debug::println(F("Switching to JOINT_MODE after startup"));
                }
                break;
                
            case STATE_JOINT_MODE:
                // Process button input for state changes
                JoystickSystem::processButtonInput();
                
                // Process joint control
                JoystickSystem::processJointControl();
                break;
                
            case STATE_KINEMATIC_MODE:
                // Process button input for state changes
                JoystickSystem::processButtonInput();
                
                // Process kinematic control
                JoystickSystem::processKinematicControl();
                break;
                
            case STATE_HOMING_MODE:
                // Process homing mode
                processHomingMode();
                
                // Process button input for state changes
                JoystickSystem::processButtonInput();
                break;
                
            case STATE_CALIBRATION_MODE:
                // Process button input for state changes
                JoystickSystem::processButtonInput();
                break;
            
            case STATE_GEAR_CONFIG_MODE:
                Debug::println(F("RobotSystem: Processing GEAR_CONFIG_MODE"));
            
                // Process gear menu actions
                DisplaySystem::processGearMenu();
            
                // Process button input for state changes
                JoystickSystem::processButtonInput();
                break;
            
            case STATE_CONFIG_MODE:
                ConfigSystem::displayConfigMenu();
                ConfigSystem::processConfigMenu();
                break;
        }
    }
    
    // Get the kinematics object
    RobotKinematics* getKinematics() {
        return robotKin;
    }

    // Get current state
    SystemState getState() {
        return currentState;
    }

    // Set current state
    void setState(SystemState state) {
        if (state != currentState) {
            // If leaving kinematic mode, clean up
            if (currentState == STATE_KINEMATIC_MODE && state != STATE_KINEMATIC_MODE) {
                exitKinematicMode();
            }
            
            currentState = state;
            setStateChangeTime(millis());
            Debug::print(F("State changed to: "));
            Debug::println(static_cast<int>(state));
        }
    }

    // Get state change time
    unsigned long getStateChangeTime() {
        return stateChangeTime;
    }

    // Set state change time
    void setStateChangeTime(unsigned long time) {
        stateChangeTime = time;
    }

    // Check if homing is started
    bool isHomingStarted() {
        return homingStarted;
    }

    // Set homing started state
    void setHomingStarted(bool started) {
        homingStarted = started;
    }

    // Get homing joint index
    int getHomingJointIndex() {
        return homingJointIndex;
    }

    // Set homing joint index
    void setHomingJointIndex(int index) {
        homingJointIndex = index;
    }

    // Get homing menu selection
    int getHomingMenuSelection() {
        return homingMenuSelection;
    }

    // Set homing menu selection
    void setHomingMenuSelection(int selection) {
        homingMenuSelection = selection;
    }

    // Get homing menu option count
    int getHomingMenuOptionCount() {
        return HOMING_MENU_COUNT;
    }

    // Get name for a homing menu option
    const char* getHomingMenuOptionName(int option) {
        switch (option) {
            case HOMING_MENU_START_HOMING: return "Homing starten";
            case HOMING_MENU_TO_CENTER:    return "Fahre zur Mitte";
            case HOMING_MENU_SAVE_HOME:    return "Home speichern";
            case HOMING_MENU_LOAD_HOME:    return "Home laden";
            case HOMING_MENU_CLEAR_HOME:   return "Home vergessen";
            default:                       return "Unbekannt";
        }
    }

    // Check if calibration is locked
    bool isCalibrationLocked() {
        return calibrationLock;
    }

    // Set calibration locked state
    void setCalibrationLocked(bool locked) {
        calibrationLock = locked;
    }
    
    // Home position save/load (EEPROM version - for compatibility)
    bool saveRobotHome(const JointAngles& angles) {
        // Magic data structure for EEPROM
        struct RobotHomeData {
            uint32_t magic;
            float jointAngles[6];
        };
        
        // Create data structure
        RobotHomeData data;
        data.magic = ROBOT_HOME_MAGIC;
        for (int i = 0; i < 6; ++i) {
            data.jointAngles[i] = angles.angles[i];
        }
        
        // Write to EEPROM
        EEPROM.put(100, data);
        Debug::println(F("Home position saved to EEPROM"));
        return true;
    }
    
    bool loadRobotHome(JointAngles& angles) {
        // Magic data structure for EEPROM
        struct RobotHomeData {
            uint32_t magic;
            float jointAngles[6];
        };
        
        // Read from EEPROM
        RobotHomeData data;
        EEPROM.get(100, data);
        
        // Validate magic number
        if (data.magic != ROBOT_HOME_MAGIC) {
            Debug::println(F("Invalid magic value in EEPROM"));
            return false;
        }
        
        // Copy data to output
        for (int i = 0; i < 6; ++i) {
            angles.angles[i] = data.jointAngles[i];
        }
        
        Debug::println(F("Home position loaded from EEPROM"));
        return true;
    }
    
    bool clearRobotHome() {
        // Magic data structure for EEPROM
        struct RobotHomeData {
            uint32_t magic;
            float jointAngles[6];
        };
        
        // Create invalid data
        RobotHomeData data = {0};
        
        // Write to EEPROM
        EEPROM.put(100, data);
        Debug::println(F("Home position cleared from EEPROM"));
        return true;
    }

    // Constant for home file path
    const char* DEFAULT_HOME_FILE = "/config/default_home.json";

    // Save home position to SD card
    bool saveRobotHomeToSD(const JointAngles& angles) {
        Debug::println(F("Saving home position to SD..."));
        
        // Initialize SD card
        if (!SD.begin(BUILTIN_SDCARD)) {
            Debug::println(F("SD card initialization failed"));
            return false;
        }
        
        // Make sure directory exists
        if (!SD.exists("/config")) {
            SD.mkdir("/config");
        }
        
        // Delete existing file if it exists
        if (SD.exists(DEFAULT_HOME_FILE)) {
            SD.remove(DEFAULT_HOME_FILE);
        }
        
        // Open file
        File homeFile = SD.open(DEFAULT_HOME_FILE, FILE_WRITE);
        if (!homeFile) {
            Debug::println(F("Error opening home file"));
            return false;
        }
        
        // Create JSON document
        JsonDocument doc;
        
        // Save magic value and joint angles
        doc["magic"] = ROBOT_HOME_MAGIC;
        JsonArray jointArray = doc["joints"].to<JsonArray>();
        for (int i = 0; i < 6; ++i) {
            jointArray.add(angles.angles[i]);
        }
        
        // Write to file
        if (serializeJson(doc, homeFile) == 0) {
            Debug::println(F("Error writing home position"));
            homeFile.close();
            return false;
        }
        
        homeFile.close();
        Debug::println(F("Home position successfully saved to SD card"));
        return true;
    }

    // Load home position from SD card
    bool loadRobotHomeFromSD(JointAngles& angles) {
        Debug::println(F("Loading home position from SD..."));
        
        // Initialize SD card
        if (!SD.begin(BUILTIN_SDCARD)) {
            Debug::println(F("SD card initialization failed"));
            return false;
        }
        
        // Check if file exists
        if (!SD.exists(DEFAULT_HOME_FILE)) {
            Debug::println(F("Home file not found"));
            return false;
        }
        
        // Open file
        File homeFile = SD.open(DEFAULT_HOME_FILE, FILE_READ);
        if (!homeFile) {
            Debug::println(F("Error opening home file"));
            return false;
        }
        
        // Create JSON document and parse file
        JsonDocument doc;
        DeserializationError error = deserializeJson(doc, homeFile);
        homeFile.close();
        
        if (error) {
            Debug::print(F("JSON parsing error: "));
            Debug::println(error.c_str());
            return false;
        }
        
        // Check magic value
        if (!doc["magic"].is<uint32_t>() || doc["magic"].as<uint32_t>() != ROBOT_HOME_MAGIC) {
            Debug::println(F("Invalid magic value in home file"));
            return false;
        }
        
        // Load joint angles
        if (doc["joints"].is<JsonArray>()) {
            JsonArray jointArray = doc["joints"];
            for (int i = 0; i < min(6, (int)jointArray.size()); ++i) {
                angles.angles[i] = jointArray[i];
            }
            Debug::println(F("Home position successfully loaded from SD card"));
            return true;
        }
        
        Debug::println(F("Error: No valid joint data in home file"));
        return false;
    }

    // Clear home position from SD card
    bool clearRobotHomeFromSD() {
        Debug::println(F("Deleting home position from SD..."));
        
        // Initialize SD card
        if (!SD.begin(BUILTIN_SDCARD)) {
            Debug::println(F("SD card initialization failed"));
            return false;
        }
        
        // Check if file exists and delete it
        if (SD.exists(DEFAULT_HOME_FILE)) {
            if (SD.remove(DEFAULT_HOME_FILE)) {
                Debug::println(F("Home position successfully deleted from SD card"));
                return true;
            } else {
                Debug::println(F("Error deleting home file"));
                return false;
            }
        }
        
        Debug::println(F("Home file not found"));
        return false;
    }
    
    void processHomingMode() {
        // Display the homing menu if not in active homing
        if (!homingStarted) {
            DisplaySystem::displayHomingMenu();
            processHomingMenu();
            return;
        }
        
        // Display homing progress
        DisplaySystem::displayHomingMode();
        
                // Process homing for current joint
        if (homingJointIndex < 6) {
            // Call StepperSystem::homeJoint instead of implementing it here
            if (StepperSystem::homeJoint(homingJointIndex)) {
                homingJointIndex++;
                Debug::print(F("Joint "));
                Debug::print(homingJointIndex);
                Debug::println(F(" referenced"));
                delay(500);
            }
        } else {
            // After homing back to menu
            Debug::println(F("Homing completed"));
            homingStarted = false;
            homingJointIndex = 0;
            homingMenuSelection = 0;
            // Synchronize kinematics
            JointAngles homeAngles;
            for (int i = 0; i < 6; i++) {
                float posDegrees = _steppers[i]->currentPosition() / _stepperConfig[i].stepsPerDegree;
                homeAngles.angles[i] = posDegrees * M_PI / 180.0;
            }
            robotKin->setCurrentJointAngles(homeAngles);
        }
    }
    
    void processHomingMenu() {
        // Menu navigation with right joystick Y
        float rightY = JoystickSystem::rightJoystick->getNormalizedY();
        if (millis() - lastMenuMove > 200) {
            if (rightY > 0.7f && homingMenuSelection > 0) {
                homingMenuSelection--;
                lastMenuMove = millis();
            } else if (rightY < -0.7f && homingMenuSelection < HOMING_MENU_COUNT-1) {
                homingMenuSelection++;
                lastMenuMove = millis();
            }
        }
        
        // Left button: Confirm (on rising edge only)
        static bool lastLeftPressed = false;
        bool leftPressed = JoystickSystem::leftJoystick->isPressed();
        
        if (leftPressed && !lastLeftPressed) {
            switch (homingMenuSelection) {
                case HOMING_MENU_START_HOMING:
                    homingStarted = true;
                    homingJointIndex = 0;
                    break;
                case HOMING_MENU_TO_CENTER: {
                    Debug::println(F("Menu: Moving to center selected"));
                    
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
                    
                    Debug::print(F("Target Pose: X=")); Debug::print(centerPose.x);
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
                            startSteps[i] = _steppers[i]->currentPosition();
                            _steppers[i]->moveTo(targetSteps[i]);
                        }
                        
                        // Movement with progress bar
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
                            _steppers[i]->setCurrentPosition(_steppers[i]->targetPosition());
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
                        
                        DisplaySystem::showMessage("Robot now", "at center!", 1500);
                    } else {
                        Debug::println(F("IK Error! Target not reachable."));
                        DisplaySystem::showMessage("IK Error!", nullptr, 1500);
                    }
                    break;
                }
                case HOMING_MENU_SAVE_HOME: {
                    JointAngles current = robotKin->getCurrentJointAngles();
                    saveRobotHome(current);
                    // Also save to SD card if available
                    saveRobotHomeToSD(current);
                    DisplaySystem::showMessage("Home saved!", nullptr, 1200);
                    break;
                }
                case HOMING_MENU_LOAD_HOME: {
                    JointAngles homeAngles;
                    bool loaded = false;
                    
                    // Try SD card first, then fall back to EEPROM
                    if (SD.begin(BUILTIN_SDCARD) && SD.exists(DEFAULT_HOME_FILE)) {
                        loaded = loadRobotHomeFromSD(homeAngles);
                    }
                    
                    // Fall back to EEPROM if SD load failed
                    if (!loaded) {
                        loaded = loadRobotHome(homeAngles);
                    }
                    
                    if (loaded) {
                        // NO movement! Just set kinematics and stepper positions
                        robotKin->setCurrentJointAngles(homeAngles);
                        for (int i = 0; i < 6; ++i) {
                            _steppers[i]->setCurrentPosition(
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
                        clearRobotHomeFromSD();
                        DisplaySystem::showMessage("Home cleared!", nullptr, 1200);
                        delay(1200);
                        clearHomePressCount = 0;
                    }
                    break;
                }
            }
        }
        lastLeftPressed = leftPressed;
    }
    
    void exitKinematicMode() {
        // Stop all motors at their current positions
        for (int i = 0; i < 6; ++i) {
            _steppers[i]->moveTo(_steppers[i]->currentPosition());
        }
        // Ensure kinematic model is synchronized with physical robot
        synchronizeKinematicsWithSteppers();
    }
    
    void synchronizeKinematicsWithSteppers() {
        JointAngles currentAngles = robotKin->getCurrentJointAngles();
        bool updated = false;
        
        for (int i = 0; i < 6; i++) {
            // Convert current stepper position to degrees
            long currentSteps = _steppers[i]->currentPosition();
            float currentDegrees = currentSteps / _stepperConfig[i].stepsPerDegree;
            float currentRadians = currentDegrees * M_PI / 180.0;
            
            // If there's a difference, update the kinematics
            if (fabs(currentRadians - currentAngles.angles[i]) > 0.01) {
                currentAngles.angles[i] = currentRadians;
                updated = true;
                
                Debug::print(F("Synchronizing Joint "));
                Debug::print(i + 1);
                Debug::print(F(": "));
                Debug::print(currentDegrees);
                Debug::print(F("° ("));
                Debug::print(currentSteps);
                Debug::println(F(" steps)"));
            }
        }
        
        if (updated) {
            robotKin->setCurrentJointAngles(currentAngles);
            Debug::println(F("Kinematics synchronized with steppers"));
        }
    }
}
