#include "DisplaySystem.h"
#include "Debug.h"
#include "config.h"
#include "RobotSystem.h"
#include "StepperSystem.h"
#include <Adafruit_SSD1306.h>
#include <ArduinoJson.h>
#include <SD.h>

// Define display object
static Adafruit_SSD1306 display(128, 64, &Wire, -1);

// Magic number for gear configuration files
#define GEAR_CONFIG_MAGIC 0x67454152 // "GEAR" in hex

namespace DisplaySystem {
    // Gear menu state
    static bool gearMenuActive = false;
    static int selectedGearAxis = 0;
    
    void init() {
        Debug::println(F("Initializing display system..."));
        
        // Initialize I2C
        Wire.begin();
        
        // Initialize OLED display
        if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
            Debug::println(F("Display initialization failed"));
            digitalWrite(_pinConfig.errorLedPin, HIGH);
            while (1);
        }
        
        display.clearDisplay();
        display.display();
        display.setTextSize(1);
        display.setTextColor(SSD1306_WHITE);
        
        Debug::println(F("Display system initialized"));
    }
    
    void update() {
        // This function would be called by the timer loop
        // Display updates are handled by the state machine
    }
    
    void displayStartupScreen() {
        display.clearDisplay();
        display.setCursor(0, 0);
        display.println(F("6DOF Robot Start"));
        display.println(F("Press L-Joystick"));
        display.println(F("for calibration"));
        display.display();
    }
    
    void displayJointMode() {
        display.clearDisplay();
        display.setCursor(0, 0);
        display.println(F("JOINT MODE"));
        
        // Show joint angles in degrees
        JointAngles angles = RobotSystem::getKinematics()->getCurrentJointAngles();
        int selectedJoint = StepperSystem::getSelectedJoint();
        
        for (int i = 0; i < 6; i++) {
            display.print(F("J"));
            display.print(i + 1);
            display.print(F(":"));
            display.print(angles.angles[i] * 180.0 / M_PI, 0);
            display.print(i == selectedJoint ? F("* ") : F("  "));
            if (i == 2) display.println();
        }
        display.println();
        
        // Show end effector position
        CartesianPose pose = RobotSystem::getKinematics()->getCurrentPose();
        display.print(F("X:"));
        display.print(pose.x, 1);
        display.print(F(" Y:"));
        display.println(pose.y, 1);
        display.print(F("Z:"));
        display.print(pose.z, 1);
        display.println();
        
        // Operating instructions
        display.println(F("R Joy X: Select joint"));
        display.println(F("L Joy X: +/- Movement"));
        display.println(F("R Btn: Change mode"));
        
        display.display();
    }
    
    void displayKinematicMode() {
        display.clearDisplay();
        display.setCursor(0, 0);
        display.println(F("KINEMATIC MODE"));
        
        // Show joint angles in degrees
        JointAngles angles = RobotSystem::getKinematics()->getCurrentJointAngles();
        for (int i = 0; i < 3; i++) {
            display.print(F("J"));
            display.print(i + 1);
            display.print(F(":"));
            display.print(angles.angles[i] * 180.0 / M_PI, 0);
            display.print(F(" "));
        }
        display.println();
        for (int i = 3; i < 6; i++) {
            display.print(F("J"));
            display.print(i + 1);
            display.print(F(":"));
            display.print(angles.angles[i] * 180.0 / M_PI, 0);
            display.print(F(" "));
        }
        display.println();
        
        // Show end effector position
        CartesianPose pose = RobotSystem::getKinematics()->getCurrentPose();
        display.print(F("X:"));
        display.print(pose.x, 1);
        display.print(F(" Y:"));
        display.println(pose.y, 1);
        display.print(F("Z:"));
        display.print(pose.z, 1);
        display.print(F(" Yaw:"));
        display.println(pose.yaw * 180.0 / M_PI, 1);
        
        // Show reach information
        float maxReach = RobotSystem::getKinematics()->getMaxReach();
        float currentDistance = sqrt(
            pose.x * pose.x + 
            pose.y * pose.y + 
            (pose.z - RobotSystem::getKinematics()->getBaseHeight()) * 
            (pose.z - RobotSystem::getKinematics()->getBaseHeight())
        );
        
        display.print(F("Dist:"));
        display.print(currentDistance, 1);
        display.print(F("/"));
        display.print(maxReach, 1);
        display.println(F("mm"));
        
        display.display();
    }
    
    void displayHomingMode() {
        display.clearDisplay();
        display.setCursor(0, 0);
        display.println(F("HOMING MODE"));
        display.println(F("----------------"));
        
        if (!RobotSystem::isHomingStarted()) {
            display.println(F("Press the left"));
            display.println(F("button to start"));
            display.println(F("homing."));
            display.println();
            display.println(F("Right button: Change mode"));
        } else {
            display.print(F("Homing axis "));
            display.println(RobotSystem::getHomingJointIndex() + 1);
            display.drawRect(0, 30, 128, 10, SSD1306_WHITE);
            int progress = (RobotSystem::getHomingJointIndex() * 128) / 6;
            display.fillRect(0, 30, progress, 10, SSD1306_WHITE);
            display.setCursor(0, 45);
            display.println(F("Please wait..."));
        }
        
        display.display();
    }
    
    void displayHomingMenu() {
        display.clearDisplay();
        display.setCursor(0, 0);
        display.println(F("Homing Menu"));
        display.println(F("--------------------"));
        display.println(F("Select with R-Joystick:"));
        
        const int visibleLines = 3;
        int selection = RobotSystem::getHomingMenuSelection();
        int startIdx = selection - 1;
        
        if (startIdx < 0) startIdx = 0;
        if (startIdx > RobotSystem::getHomingMenuOptionCount() - visibleLines) {
            startIdx = RobotSystem::getHomingMenuOptionCount() - visibleLines;
        }
        if (startIdx < 0) startIdx = 0;
        
        display.println();
        for (int i = startIdx; i < startIdx + visibleLines && 
                               i < RobotSystem::getHomingMenuOptionCount(); ++i) {
            if (i == selection) display.print(F("> "));
            else display.print(F("  "));
            display.println(RobotSystem::getHomingMenuOptionName(i));
        }
        
        display.println();
        display.println(F("Confirm with L-Button"));
        
        display.display();
    }
    
    void displayCalibrationMode() {
        display.clearDisplay();
        display.setCursor(0, 0);
        display.println(F("CALIBRATION"));
        display.println(F("----------------"));
        display.println(F("Press the left"));
        display.println(F("button to start"));
        display.println(F("calibration."));
        display.println();
        display.println(F("Right button: Change mode"));
        
        display.display();
    }
    
    void displayHomingCenterProgress(float progress) {
        display.clearDisplay();
        display.setCursor(0, 0);
        display.println(F("Homing Menu"));
        display.println(F("--------------------"));
        display.println(F("Select with R-Joystick"));
        display.println();
        display.println(F("Moving to center..."));
        
        // Draw progress bar
        int barWidth = 100;
        int barHeight = 8;
        int barX = 14;
        int barY = 40;
        display.drawRect(barX, barY, barWidth, barHeight, SSD1306_WHITE);
        display.fillRect(barX, barY, (int)(barWidth * progress), barHeight, SSD1306_WHITE);
        
        display.display();
    }
    
    void showCalibrationInProgress() {
        display.clearDisplay();
        display.setCursor(0, 0);
        display.println(F("CALIBRATING..."));
        display.println(F("Move both joysticks"));
        display.println(F("to their extreme"));
        display.println(F("positions within"));
        display.println(F("5 seconds."));
        display.display();
    }
    
    void showCalibrationComplete() {
        display.clearDisplay();
        display.setCursor(0, 0);
        display.println(F("CALIBRATION COMPLETE"));
        display.println(F("----------------"));
        display.println(F("Values saved."));
        display.println();
        display.println(F("Press right button"));
        display.println(F("to continue."));
        display.display();
        
        delay(2000);
    }
    
    void displayGearMenu() {
        display.clearDisplay();
        display.setCursor(0, 0);
        display.println(F("GEAR CONFIGURATION"));
        display.println(F("----------------"));
        
        // Show selected joint
        display.print(F("Joint "));
        display.print(selectedGearAxis + 1);
        display.print(F(": "));
        display.print(_stepperConfig[selectedGearAxis].stepsPerDegree);
        display.println(F(" steps/deg"));
        
        display.println();
        display.println(F("R-Joy Y: Change joint"));
        display.println(F("L-Joy X: Adjust value"));
        display.println(F("L-Btn: Save, R-Btn: Exit"));
        
        display.display();
    }
    
    Adafruit_SSD1306* getDisplay() {
        return &display;
    }
    
    bool isGearMenuActive() {
        return gearMenuActive;
    }
    
    void setGearMenuActive(bool active) {
        gearMenuActive = active;
    }
    
    int getSelectedGearAxis() {
        return selectedGearAxis;
    }
    
    void setSelectedGearAxis(int axis) {
        selectedGearAxis = axis; 
    }
    
    void processGearMenu() {
        // Implementation for gear menu processing
        // This is a placeholder that would need to be implemented
        displayGearMenu();
    }
    
    void showMessage(const char* line1, const char* line2, int delayMs) {
        display.clearDisplay();
        display.setCursor(0, 0);
        display.println(line1 ? line1 : "");
        
        if (line2) {
            display.println();
            display.println(line2);
        }
        
        display.display();
        
        if (delayMs > 0) {
            delay(delayMs);
        }
    }
    
    void saveHomePositionToSD(const JointAngles& angles) {
        // Implementation for saving home position to SD
        // This is a placeholder
    }
    
    bool loadHomePositionFromSD(JointAngles& angles) {
        // Implementation for loading home position from SD
        // This is a placeholder
        return false;
    }
}
