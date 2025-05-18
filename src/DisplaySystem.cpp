#include "DisplaySystem.h"
#include "Debug.h"
#include <Wire.h>
#include "RobotSystem.h"
#include "StepperSystem.h"
#include "JoystickSystem.h"
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1

// Display instance
static Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

namespace DisplaySystem {
    void init() {
        Debug::println(F("Initializing display..."));
        
        // Initialize I2C
        Wire.begin();
        
        // Initialize display
        if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
            Debug::println(F("Display initialization failed"));
            return;
        }
        
        display.clearDisplay();
        display.display();
        display.setTextColor(SSD1306_WHITE);
        
        Debug::println(F("Display initialized"));
    }
    
    void update() {
        // Display based on current system state
        switch (RobotSystem::getState()) {
            case RobotSystem::STATE_STARTUP:
                displayStartupScreen();
                break;
                
            case RobotSystem::STATE_JOINT_MODE:
                displayJointMode();
                break;
                
            case RobotSystem::STATE_KINEMATIC_MODE:
                displayKinematicMode();
                break;
                
            case RobotSystem::STATE_HOMING_MODE:
                if (RobotSystem::isHomingStarted()) {
                    displayHomingMode();
                } else {
                    displayHomingMenu();
                }
                break;
                
            case RobotSystem::STATE_CALIBRATION_MODE:
                displayCalibrationMode();
                break;
        }
    }
    
    void displayStartupScreen() {
        display.clearDisplay();
        display.setCursor(0,0);
        display.println(F("6DOF Robot Start"));
        display.println(F("Press L-Joystick"));
        display.println(F("for calibration"));
        display.display();
    }
    
    void displayJointMode() {
        display.clearDisplay();
        display.setCursor(0,0);
        display.println(F("JOINT MODE"));
        
        // Display joint angles in degrees
        RobotKinematics* robotKin = RobotSystem::getKinematics();
        JointAngles angles = robotKin->getCurrentJointAngles();
        int selectedJoint = JoystickSystem::getSelectedJoint();
        
        for (int i = 0; i < 6; i++) {
            display.print(F("J"));
            display.print(i+1);
            display.print(F(":"));
            display.print(angles.angles[i] * 180.0 / M_PI, 0);
            display.print(i == selectedJoint ? F("* ") : F("  "));
            if (i == 2) display.println();
        }
        display.println();
        
        // Display end effector position
        CartesianPose pose = robotKin->getCurrentPose();
        display.print(F("X:"));
        display.print(pose.x, 1);
        display.print(F(" Y:"));
        display.println(pose.y, 1);
        display.print(F("Z:"));
        display.print(pose.z, 1);
        display.println();
        
        // Operating instructions
        display.println(F("R Joy X: Select joint"));
        display.println(F("L Joy X: +/- movement"));
        display.println(F("R Btn: Change mode"));
        
        display.display();
    }
    
    void displayKinematicMode() {
        display.clearDisplay();
        display.setCursor(0,0);
        display.println(F("KINEMATIC MODE"));
        
        // Display joint angles in degrees
        RobotKinematics* robotKin = RobotSystem::getKinematics();
        JointAngles angles = robotKin->getCurrentJointAngles();
        for (int i = 0; i < 3; i++) {
            display.print(F("J"));
            display.print(i+1);
            display.print(F(":"));
            display.print(angles.angles[i] * 180.0 / M_PI, 0);
            display.print(F(" "));
        }
        display.println();
        for (int i = 3; i < 6; i++) {
            display.print(F("J"));
            display.print(i+1);
            display.print(F(":"));
            display.print(angles.angles[i] * 180.0 / M_PI, 0);
            display.print(F(" "));
        }
        display.println();
        
        // Display end effector position
        CartesianPose pose = robotKin->getCurrentPose();
        display.print(F("X:"));
        display.print(pose.x, 1);
        display.print(F(" Y:"));
        display.println(pose.y, 1);
        display.print(F("Z:"));
        display.print(pose.z, 1);
        display.print(F(" Yaw:"));
        display.println(pose.yaw * 180.0 / M_PI, 1);
        
        // Range display
        RobotConfig config = robotKin->getConfig();
        float maxReach = config.dhParams[1].a + config.dhParams[2].a;
        float currentDistance = sqrt(pose.x*pose.x + pose.y*pose.y + 
                             (pose.z-config.dhParams[0].d)*(pose.z-config.dhParams[0].d));
        display.print(F("Dist:"));
        display.print(currentDistance, 1);
        display.print(F("/"));
        display.print(maxReach, 1);
        display.println(F("mm"));
        
        display.display();
    }
    
    void displayHomingMode() {
        display.clearDisplay();
        display.setCursor(0,0);
        display.println(F("HOMING MODE"));
        display.println(F("----------------"));
        
        display.print(F("Referencing axis "));
        display.println(RobotSystem::getHomingJointIndex() + 1);
        display.drawRect(0, 30, 128, 10, SSD1306_WHITE);
        int progress = (RobotSystem::getHomingJointIndex() * 128) / 6;
        display.fillRect(0, 30, progress, 10, SSD1306_WHITE);
        display.setCursor(0, 45);
        display.println(F("Please wait..."));
        
        display.display();
    }
    
    void displayHomingMenu() {
        display.clearDisplay();
        display.setCursor(0,0);
        display.println(F("Homing Menu"));
        display.println(F("--------------------"));
        display.println(F("Select with R-Joystick:"));

        const int visibleLines = 3;
        int selection = RobotSystem::getHomingMenuSelection();
        int startIdx = selection - 1;
        if (startIdx < 0) startIdx = 0;
        
        int menuCount = RobotSystem::getHomingMenuOptionCount();
        if (startIdx > menuCount - visibleLines) startIdx = menuCount - visibleLines;
        if (startIdx < 0) startIdx = 0;

        display.println(F(""));
        for (int i = startIdx; i < startIdx + visibleLines && i < menuCount; ++i) {
            if (i == selection) display.print(F("> "));
            else display.print(F("  "));
            
            display.println(RobotSystem::getHomingMenuOptionName(i));
        }
        display.println(F(""));
        display.println(F("Confirm with L-Button"));
        display.display();
    }
    
    void displayCalibrationMode() {
        display.clearDisplay();
        display.setCursor(0,0);
        display.println(F("CALIBRATION"));
        display.println(F("----------------"));
        display.println(F("Press the left"));
        display.println(F("button to start"));
        display.println(F("calibration."));
        display.println(F(""));
        display.println(F("Right button: Change mode"));
        display.display();
    }
    
    void showCalibrationInProgress() {
        display.clearDisplay();
        display.setCursor(0,0);
        display.println(F("CALIBRATING..."));
        display.println(F("Move both joysticks"));
        display.println(F("to extreme positions"));
        display.println(F("within 5 seconds."));
        display.display();
    }
    
    void showCalibrationComplete() {
        display.clearDisplay();
        display.setCursor(0,0);
        display.println(F("CALIBRATION COMPLETE"));
        display.println(F("----------------"));
        display.println(F("Values saved."));
        display.println(F(""));
        display.println(F("Press right button"));
        display.println(F("to continue."));
        display.display();
    }
    
    void showMessage(const char* line1, const char* line2, int delayMs) {
        display.clearDisplay();
        display.setCursor(0,0);
        display.println(line1);
        if (line2) {
            display.println(line2);
        }
        display.display();
        
        if (delayMs > 0) {
            delay(delayMs);
        }
    }
    
    Adafruit_SSD1306* getDisplay() {
        return &display;
    }
}

void DisplaySystem::displayHomingCenterProgress(float progress) {
    display.clearDisplay();
    display.setCursor(0,0);
    display.println("Moving to Center");
    display.println("----------------");
    
    // Draw progress bar (100px wide, 10px high)
    int barWidth = 100;
    int barHeight = 10;
    int barX = 14;
    int barY = 30;
    display.drawRect(barX, barY, barWidth, barHeight, SSD1306_WHITE);
    display.fillRect(barX, barY, (int)(barWidth * progress), barHeight, SSD1306_WHITE);
    
    // Show percentage
    display.setCursor(45, 45);
    display.print((int)(progress * 100));
    display.print("%");
    
    display.display();
}