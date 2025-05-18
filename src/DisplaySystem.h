#pragma once

#include <Adafruit_SSD1306.h>

namespace DisplaySystem {
    // Initialize display
    void init();
    
    // Update display based on current state (called at 60Hz)
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
    
    // Display progress during center movement
    void displayHomingCenterProgress(float progress);
}