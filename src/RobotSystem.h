#pragma once

#include <Arduino.h>
#include "RobotKinematics.h"

#define ROBOT_HOME_MAGIC 0x42ABCDEF

namespace RobotSystem {
    // System states
    enum SystemState {
        STATE_STARTUP,
        STATE_JOINT_MODE,
        STATE_KINEMATIC_MODE,
        STATE_HOMING_MODE,
        STATE_CALIBRATION_MODE,
        STATE_GEAR_CONFIG_MODE,  // Add this enum
        STATE_CONFIG_MODE        // Add this enum
    };
    
    // Homing menu options
    enum HomingMenuOption {
        HOMING_MENU_START_HOMING = 0,
        HOMING_MENU_TO_CENTER,
        HOMING_MENU_SAVE_HOME,
        HOMING_MENU_LOAD_HOME,
        HOMING_MENU_CLEAR_HOME,
        HOMING_MENU_COUNT
    };
    
    // String names for menu options (defined in cpp)
    extern const char* homingMenuOptionNames[HOMING_MENU_COUNT];
    
    // Basic initialization
    void init();
    void initRobotConfig();
    
    // Main update function
    void update();
    
    // State machine processing
    void processCurrentState();
    
    // State getters and setters
    SystemState getState();
    void setState(SystemState state);
    
    // Homing functions
    void processHomingMode();
    void processHomingMenu();
    void processMoveToCenter();
    
    // Home position storage
    bool saveRobotHome(const JointAngles& angles);
    bool loadRobotHome(JointAngles& angles);
    bool clearRobotHome();
    
    // Timing
    unsigned long getStateChangeTime();
    void setStateChangeTime(unsigned long time);
    
    // Homing status
    bool isHomingStarted();
    void setHomingStarted(bool started);
    int getHomingJointIndex();
    void setHomingJointIndex(int index);
    
    // Menu selection
    int getHomingMenuSelection();
    void setHomingMenuSelection(int selection);
    int getHomingMenuOptionCount();
    const char* getHomingMenuOptionName(int option);
    
    // Button state
    bool isCalibrationLocked();
    void setCalibrationLocked(bool locked);
    
    // Kinematics access
    RobotKinematics* getKinematics();

    // Add exit kinematic mode function
    void exitKinematicMode();
    void synchronizeKinematicsWithSteppers();
    // SD card home functions
    bool saveRobotHomeToSD(const JointAngles& angles);
    bool loadRobotHomeFromSD(JointAngles& angles);
    bool clearRobotHomeFromSD();
}
