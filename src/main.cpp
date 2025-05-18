#include <Arduino.h>
#include "config.h"
#include "Debug.h"
#include "TimerLoop.h"
#include "JoystickSystem.h"
#include "StepperSystem.h"
#include "RobotSystem.h"
#include "DisplaySystem.h"
#include <Wire.h>
#include <EEPROM.h>
#include <SD.h>

void setup() {
    // Initialize serial communication
    Serial.begin(115200);
    
    // Initialize debug output
    Debug::enabled = true;
    Debug::println(F("6DOF Robot Controller Starting..."));
    
    // Load default configurations
    loadDefaultPinConfig();
    loadDefaultJoystickConfig();
    loadDefaultStepperConfig();
    
    // Initialize error LED
    pinMode(_pinConfig.errorLedPin, OUTPUT);
    
    // Initialize SD card if available
    if (SD.begin(BUILTIN_SDCARD)) {
        Debug::println(F("SD card initialized"));
        
        // Create config directory if it doesn't exist
        if (!SD.exists("/config")) {
            SD.mkdir("/config");
        }
        
        // Create home directory if it doesn't exist
        if (!SD.exists("/home")) {
            SD.mkdir("/home");
        }
    } else {
        Debug::println(F("SD card initialization failed"));
    }
    
    // Initialize display system
    DisplaySystem::init();
    
    // Show startup screen
    DisplaySystem::displayStartupScreen();
    delay(1500);
    
    // Show robot bitmap
    DisplaySystem::getDisplay()->clearDisplay();
    DisplaySystem::getDisplay()->drawBitmap(0, 0, robotArmBitmap, 128, 64, SSD1306_WHITE);
    DisplaySystem::getDisplay()->display();
    delay(2000);
    
    // Initialize joystick system
    JoystickSystem::init();
    
    // Initialize stepper system
    StepperSystem::init();
    
    // Initialize robot system (kinematics, etc.)
    RobotSystem::init();
    
    // Initialize timer-based task system
    TimerLoop::begin(nullptr, JoystickSystem::update, DisplaySystem::update);
    
    Debug::println(F("System initialized, starting in Homing mode"));
}

void loop() {
    // Update timer-based tasks
    TimerLoop::loop();
    
    // Update stepper motors
    StepperSystem::update();
    
    // Update robot system (processes state machine)
    RobotSystem::update();
    
    // Small delay to reduce CPU usage
    delay(5);
}
