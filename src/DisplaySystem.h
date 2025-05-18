#pragma once

#include <Arduino.h>
#include <Adafruit_SSD1306.h>
#include "RobotKinematics.h"

namespace DisplaySystem {
    // Initialize display
    void init();
    
    // Update display based on current state (called periodically)
    void update();
    
    // Display screens for different states
    void displayStartupScreen();
    void displayJointMode();
    void displayKinematicMode();
    void displayHomingMode();
    void displayHomingMenu();
    void displayCalibrationMode();
    
    // Special display screens
    void showCalibrationInProgress();
    void showCalibrationComplete();
    void showMessage(const char* line1, const char* line2 = nullptr, int delayMs = 0);
    
    // Get display instance
    Adafruit_SSD1306* getDisplay();
    
    // Progress display
    void displayHomingCenterProgress(float progress);
    
    // Gear menu functions
    void displayGearMenu();
    void processGearMenu();
    void saveGearConfigToSD();
    bool loadGearConfigFromSD();
    
    // Gear menu active status
    bool isGearMenuActive();
    void setGearMenuActive(bool active);
    
    // Get selected gear axis
    int getSelectedGearAxis();
    void setSelectedGearAxis(int axis);
    
    // Home position storage functions
    void saveHomePositionToSD(const JointAngles& angles);
    bool loadHomePositionFromSD(JointAngles& angles);
    void listHomePositionsOnSD();
    void deleteHomePositionFromSD(const char* filename);
}
