#pragma once

#include "RobotKinematics.h"

namespace RobotSystem {
    // System states
    enum SystemState {
        STATE_STARTUP,
        STATE_JOINT_MODE,
        STATE_KINEMATIC_MODE,
        STATE_HOMING_MODE,
        STATE_CALIBRATION_MODE
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
    
    // Initialize robot system
    void init();
    
    // Initialize robot configuration with default values
    void initRobotConfig();
    
    // Robot magic number for EEPROM validation
    #define ROBOT_HOME_MAGIC 0x42ABCDEF
    
    // State management
    SystemState getState();
    void setState(SystemState newState);
    unsigned long getStateChangeTime();
    void setStateChangeTime(unsigned long time);
    bool isCalibrationLocked();
    void setCalibrationLocked(bool locked);
    
    // Homing operations
    void processHomingMenu();
    void processHomingMode();
    bool isHomingStarted();
    void setHomingStarted(bool started);
    int getHomingJointIndex();
    void setHomingJointIndex(int index);
    int getHomingMenuSelection();
    void setHomingMenuSelection(int selection);
    int getHomingMenuOptionCount();
    const char* getHomingMenuOptionName(int option);
    
    // Home position management
    void saveRobotHome(const JointAngles& angles);
    bool loadRobotHome(JointAngles& angles);
    void clearRobotHome();
    
    // Access to robot kinematics
    RobotKinematics* getKinematics();
    
    // Process modes
    void processCurrentState();
    // Update robot system state
    void update();
    
    // New method for processing homing menu selection
    void processHomingMenuSelection();
}