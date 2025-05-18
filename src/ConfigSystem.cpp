#include "ConfigSystem.h"
#include "DisplaySystem.h"
#include "RobotSystem.h"
#include "JoystickSystem.h"
#include "StepperSystem.h"
#include <SD.h>
#include <ArduinoJson.h>

// Configuration magic number for validation
#define CONFIG_MAGIC 0x6D0F7231

// External configuration references
extern PinConfig _pinConfig;
extern JoystickConfig _joystickConfig;
extern StepperConfig _stepperConfig[6];
extern AccelStepper* _steppers[6];

namespace ConfigSystem {
    // Current menu selection
    static int configMenuSelection = 0;
    static unsigned long lastMenuMove = 0;
    
    // Define menu items here, not in RobotSystem
    const int menuItemsCount = 14;
     const char* menuItems[] = {
        "Max Speed",
        "Homing Speed",
        "Center X",
        "Center Y", 
        "Center Z",
        "Invert Joysticks",
        "UI Theme",
        "Joystick Deadband",
        "DH Parameter",
        "Joint Limits",
        "Tool Offset",
        "Save Config",
        "Load Config",
        "Reset Config"
    };
   
    
    void init() {
        Debug::println(F("Config system initialized"));
    }
    
    void displayConfigMenu() {
        // Display the configuration menu on the OLED
        Adafruit_SSD1306* display = DisplaySystem::getDisplay();
        
        display->clearDisplay();
        display->setCursor(0, 0);
        display->println(F("Configuration Menu"));
        display->println(F("------------------"));
        
        // Menu options
        const char* menuOptions[] = {
            "Save Config",
            "Load Config",
            "List Configs",
            "Reset Defaults",
            "Back to Home"
        };
        
        const int optionCount = sizeof(menuOptions) / sizeof(menuOptions[0]);
        
        // Display menu options with selection
        for (int i = 0; i < optionCount; i++) {
            if (i == configMenuSelection) {
                display->print(F("> "));
            } else {
                display->print(F("  "));
            }
            display->println(menuOptions[i]);
        }
        
        display->println(F(""));
        display->println(F("R Joy: Select"));
        display->println(F("L Btn: Confirm"));
        
        display->display();
    }
    
    void processConfigMenu() {
        // Process input for menu navigation
        float rightY = JoystickSystem::rightJoystick->getNormalizedY();
        
        // Menu navigation with rate limiting
        if (millis() - lastMenuMove > 200) {
            if (rightY > 0.7f && configMenuSelection > 0) {
                configMenuSelection--;
                lastMenuMove = millis();
                Debug::print(F("Config menu selection: "));
                Debug::println(configMenuSelection);
            } else if (rightY < -0.7f && configMenuSelection < 4) {
                configMenuSelection++;
                lastMenuMove = millis();
                Debug::print(F("Config menu selection: "));
                Debug::println(configMenuSelection);
            }
        }
        
        // Check for selection confirmation
        if (JoystickSystem::isLeftButtonPressed()) {
            switch (configMenuSelection) {
                case 0: // Save Config
                    if (saveConfig("/config/user_config.json")) {
                        DisplaySystem::showMessage("Configuration", "saved successfully", 1500);
                    } else {
                        DisplaySystem::showMessage("Failed to save", "configuration", 1500);
                    }
                    break;
                    
                case 1: // Load Config
                    if (loadConfig("/config/user_config.json")) {
                        DisplaySystem::showMessage("Configuration", "loaded successfully", 1500);
                    } else {
                        DisplaySystem::showMessage("Failed to load", "configuration", 1500);
                    }
                    break;
                    
                case 2: // List Configs
                    listConfigurations();
                    break;
                    
                case 3: // Reset Defaults
                    loadDefaultPinConfig();
                    loadDefaultJoystickConfig();
                    loadDefaultStepperConfig();
                    DisplaySystem::showMessage("Default settings", "restored", 1500);
                    break;
                    
                case 4: // Back to Home
                    RobotSystem::setState(RobotSystem::STATE_HOMING_MODE);
                    break;
            }
            
            // Debounce
            delay(300);
        }
    }
    
    bool saveConfig(const char* filename) {
        // Make sure SD card is available
        if (!SD.begin(BUILTIN_SDCARD)) {
            Debug::println(F("SD card initialization failed"));
            return false;
        }
        
        // Create directory if it doesn't exist
        if (!SD.exists("/config")) {
            SD.mkdir("/config");
        }
        
        // Remove existing file if it exists
        if (SD.exists(filename)) {
            SD.remove(filename);
        }
        
        // Open file for writing
        File configFile = SD.open(filename, FILE_WRITE);
        if (!configFile) {
            Debug::println(F("Failed to open config file for writing"));
            return false;
        }
        
        // Create JSON document
        JsonDocument doc;
        
        // Add magic number for validation
        doc["magic"] = CONFIG_MAGIC;
        
        // Save pin configuration - UPDATED SYNTAX
        JsonObject pins = doc["pins"].to<JsonObject>();
        pins["leftX"] = _pinConfig.leftXPin;
        pins["leftY"] = _pinConfig.leftYPin;
        pins["leftBtn"] = _pinConfig.leftBtnPin;
        pins["rightX"] = _pinConfig.rightXPin;
        pins["rightY"] = _pinConfig.rightYPin;
        pins["rightBtn"] = _pinConfig.rightBtnPin;
        pins["errorLed"] = _pinConfig.errorLedPin;
        pins["oledSda"] = _pinConfig.oledSdaPin;
        pins["oledScl"] = _pinConfig.oledSclPin;
        
        // Save stepper pins - UPDATED SYNTAX
        JsonArray stepperPins = doc["stepperPins"].to<JsonArray>();
        for (int i = 0; i < 6; i++) {
            JsonArray joint = stepperPins.add<JsonArray>();
            for (int j = 0; j < 4; j++) {
                joint.add(_pinConfig.stepperPins[i][j]);
            }
        }
        
        // Save joystick configuration - UPDATED SYNTAX
        JsonObject joystick = doc["joystick"].to<JsonObject>();
        joystick["deadband"] = _joystickConfig.deadband;
        joystick["sensitivity"] = _joystickConfig.sensitivity;
        
        // Save stepper configurations - UPDATED SYNTAX
        JsonArray steppers = doc["steppers"].to<JsonArray>();
        for (int i = 0; i < 6; i++) {
            JsonObject stepper = steppers.add<JsonObject>();
            stepper["stepsPerDegree"] = _stepperConfig[i].stepsPerDegree;
            stepper["maxSpeed"] = _stepperConfig[i].maxSpeed;
            stepper["acceleration"] = _stepperConfig[i].acceleration;
            stepper["homingSpeed"] = _stepperConfig[i].homingSpeed;
            stepper["minPosition"] = _stepperConfig[i].minPosition;
            stepper["maxPosition"] = _stepperConfig[i].maxPosition;
        }
        
        // Serialize and write to file
        if (serializeJson(doc, configFile) == 0) {
            Debug::println(F("Failed to write config to file"));
            configFile.close();
            return false;
        }
        
        configFile.close();
        Debug::println(F("Configuration saved successfully"));
        return true;
    }
    
    bool loadConfig(const char* filename) {
        // Make sure SD card is available
        if (!SD.begin(BUILTIN_SDCARD)) {
            Debug::println(F("SD card initialization failed"));
            return false;
        }
        
        // Check if file exists
        if (!SD.exists(filename)) {
            Debug::println(F("Config file not found"));
            return false;
        }
        
        // Open file for reading
        File configFile = SD.open(filename, FILE_READ);
        if (!configFile) {
            Debug::println(F("Failed to open config file for reading"));
            return false;
        }
        
        // Parse JSON document
        JsonDocument doc;
        DeserializationError error = deserializeJson(doc, configFile);
        configFile.close();
        
        if (error) {
            Debug::print(F("JSON parsing error: "));
            Debug::println(error.c_str());
            return false;
        }
        
        // Verify magic number - UPDATED SYNTAX
        if (!doc["magic"].is<uint32_t>() || doc["magic"].as<uint32_t>() != CONFIG_MAGIC) {
            Debug::println(F("Invalid config file magic number"));
            return false;
        }
        
        // Load pin configuration - UPDATED SYNTAX
        if (doc["pins"].is<JsonObject>()) {
            JsonObject pins = doc["pins"];
            _pinConfig.leftXPin = pins["leftX"] | _pinConfig.leftXPin;
            _pinConfig.leftYPin = pins["leftY"] | _pinConfig.leftYPin;
            _pinConfig.leftBtnPin = pins["leftBtn"] | _pinConfig.leftBtnPin;
            _pinConfig.rightXPin = pins["rightX"] | _pinConfig.rightXPin;
            _pinConfig.rightYPin = pins["rightY"] | _pinConfig.rightYPin;
            _pinConfig.rightBtnPin = pins["rightBtn"] | _pinConfig.rightBtnPin;
            _pinConfig.errorLedPin = pins["errorLed"] | _pinConfig.errorLedPin;
            _pinConfig.oledSdaPin = pins["oledSda"] | _pinConfig.oledSdaPin;
            _pinConfig.oledSclPin = pins["oledScl"] | _pinConfig.oledSclPin;
        }
        
        // Load stepper pins - UPDATED SYNTAX
        if (doc["stepperPins"].is<JsonArray>()) {
            JsonArray stepperPins = doc["stepperPins"];
            for (int i = 0; i < min(6, (int)stepperPins.size()); i++) {
                if (stepperPins[i].is<JsonArray>()) {
                    JsonArray joint = stepperPins[i];
                    for (int j = 0; j < min(4, (int)joint.size()); j++) {
                        _pinConfig.stepperPins[i][j] = joint[j] | _pinConfig.stepperPins[i][j];
                    }
                }
            }
        }
        
        // Load joystick configuration - UPDATED SYNTAX
        if (doc["joystick"].is<JsonObject>()) {
            JsonObject joystick = doc["joystick"];
            _joystickConfig.deadband = joystick["deadband"] | _joystickConfig.deadband;
            _joystickConfig.sensitivity = joystick["sensitivity"] | _joystickConfig.sensitivity;
        }
        
        // Load stepper configurations - UPDATED SYNTAX
        if (doc["steppers"].is<JsonArray>()) {
            JsonArray steppers = doc["steppers"];
            for (int i = 0; i < min(6, (int)steppers.size()); i++) {
                if (steppers[i].is<JsonObject>()) {
                    JsonObject stepper = steppers[i];
                    _stepperConfig[i].stepsPerDegree = stepper["stepsPerDegree"] | _stepperConfig[i].stepsPerDegree;
                    _stepperConfig[i].maxSpeed = stepper["maxSpeed"] | _stepperConfig[i].maxSpeed;
                    _stepperConfig[i].acceleration = stepper["acceleration"] | _stepperConfig[i].acceleration;
                    _stepperConfig[i].homingSpeed = stepper["homingSpeed"] | _stepperConfig[i].homingSpeed;
                    _stepperConfig[i].minPosition = stepper["minPosition"] | _stepperConfig[i].minPosition;
                    _stepperConfig[i].maxPosition = stepper["maxPosition"] | _stepperConfig[i].maxPosition;
                    
                    // Update stepper motor parameters
                    if (_steppers[i]) {
                        _steppers[i]->setMaxSpeed(_stepperConfig[i].maxSpeed);
                        _steppers[i]->setAcceleration(_stepperConfig[i].acceleration);
                    }
                }
            }
        }
        
        Debug::println(F("Configuration loaded successfully"));
        return true;
    }
    
    void listConfigurations() {
        // Make sure SD card is available
        if (!SD.begin(BUILTIN_SDCARD)) {
            DisplaySystem::showMessage("SD card", "initialization failed", 1500);
            return;
        }
        
        // Open configuration directory
        File configDir = SD.open("/config");
        if (!configDir) {
            DisplaySystem::showMessage("Config directory", "not found", 1500);
            return;
        }
        
        // Check if it's a directory
        if (!configDir.isDirectory()) {
            configDir.close();
            DisplaySystem::showMessage("/config is not", "a directory", 1500);
            return;
        }
        
        // Display config files
        Adafruit_SSD1306* display = DisplaySystem::getDisplay();
        display->clearDisplay();
        display->setCursor(0, 0);
        display->println(F("Configuration Files:"));
        display->println(F("------------------"));
        
        int fileCount = 0;
        int maxFilesToShow = 5;
        
        // Read each file in the directory
        File entry = configDir.openNextFile();
        while (entry && fileCount < maxFilesToShow) {
            if (!entry.isDirectory()) {
                // Get the filename
                const char* filename = entry.name();
                
                // Display only .json files
                if (strstr(filename, ".json")) {
                    display->println(filename);
                    fileCount++;
                }
            }
            entry.close();
            entry = configDir.openNextFile();
        }
        
        if (fileCount == 0) {
            display->println(F("No config files found"));
        }
        
        display->println(F(""));
        display->println(F("Press any button to"));
        display->println(F("return to menu"));
        
        display->display();
        configDir.close();
        
        // Wait for button press to return to menu
        while (!JoystickSystem::isLeftButtonPressed() && !JoystickSystem::isRightButtonPressed()) {
            delay(50);
        }
        
        // Debounce
        delay(300);
    }
    
    void loadDefaultPinConfig() {
        _pinConfig.leftXPin = 40;
        _pinConfig.leftYPin = 41;
        _pinConfig.leftBtnPin = 27;
        _pinConfig.rightXPin = 38;
        _pinConfig.rightYPin = 39;
        _pinConfig.rightBtnPin = 26;
        
        // Stepper pins
        _pinConfig.stepperPins[0][0] = 2;   // Base - STEP
        _pinConfig.stepperPins[0][1] = 3;   // Base - DIR
        _pinConfig.stepperPins[0][2] = 4;   // Base - ENABLE
        _pinConfig.stepperPins[0][3] = 22;  // Base - LIMIT
        
        _pinConfig.stepperPins[1][0] = 5;   // Shoulder - STEP
        _pinConfig.stepperPins[1][1] = 6;   // Shoulder - DIR
        _pinConfig.stepperPins[1][2] = 7;   // Shoulder - ENABLE
        _pinConfig.stepperPins[1][3] = 23;  // Shoulder - LIMIT
        
        _pinConfig.stepperPins[2][0] = 8;   // Elbow - STEP
        _pinConfig.stepperPins[2][1] = 9;   // Elbow - DIR
        _pinConfig.stepperPins[2][2] = 10;  // Elbow - ENABLE
        _pinConfig.stepperPins[2][3] = 24;  // Elbow - LIMIT
        
        _pinConfig.stepperPins[3][0] = 11;  // Wrist Pitch - STEP
        _pinConfig.stepperPins[3][1] = 12;  // Wrist Pitch - DIR
        _pinConfig.stepperPins[3][2] = 41;  // Wrist Pitch - ENABLE
        _pinConfig.stepperPins[3][3] = 25;  // Wrist Pitch - LIMIT
        
        _pinConfig.stepperPins[4][0] = 14;  // Wrist Roll - STEP
        _pinConfig.stepperPins[4][1] = 15;  // Wrist Roll - DIR
        _pinConfig.stepperPins[4][2] = 16;  // Wrist Roll - ENABLE
        _pinConfig.stepperPins[4][3] = 29;  // Wrist Roll - LIMIT
        
        _pinConfig.stepperPins[5][0] = 17;  // Gripper - STEP
        _pinConfig.stepperPins[5][1] = 20;  // Gripper - DIR
        _pinConfig.stepperPins[5][2] = 21;  // Gripper - ENABLE
        _pinConfig.stepperPins[5][3] = 28;  // Gripper - LIMIT
        
        // Error LED pin
        _pinConfig.errorLedPin = 13;
        
        // OLED Pins
        _pinConfig.oledSdaPin = 18;
        _pinConfig.oledSclPin = 19;
        
        Debug::println(F("Default pin configuration loaded"));
    }
    
    void loadDefaultJoystickConfig() {
        _joystickConfig.deadband = 50;
        _joystickConfig.sensitivity = 1.0f;
        
        Debug::println(F("Default joystick configuration loaded"));
    }
    
    void loadDefaultStepperConfig() {
        for (int i = 0; i < 6; i++) {
            _stepperConfig[i].stepsPerDegree = 20.0f;
            _stepperConfig[i].maxSpeed = 1000.0f;
            _stepperConfig[i].acceleration = 500.0f;
            _stepperConfig[i].homingSpeed = 500.0f;
            _stepperConfig[i].minPosition = -180.0f;
            _stepperConfig[i].maxPosition = 180.0f;
        }
        
        // Apply settings to stepper motors if they exist
        for (int i = 0; i < 6; i++) {
            if (_steppers[i]) {
                _steppers[i]->setMaxSpeed(_stepperConfig[i].maxSpeed);
                _steppers[i]->setAcceleration(_stepperConfig[i].acceleration);
            }
        }
        
        Debug::println(F("Default stepper configuration loaded"));
    }
}