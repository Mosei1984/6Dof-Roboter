#pragma once

#include <Arduino.h>
#include <SD.h>
#include <ArduinoJson.h>
#include "RobotSystem.h"
#include "config.h"
#include "Debug.h"
#include "RobotKinematics.h"
#include "globals.h"

// Forward declaration
extern RobotConfig g_robotConfig;
#define CONFIG_MAGIC 0x12345678
namespace ConfigSystem {
    // Menu items
    extern const char* menuItems[];      // Nur Deklaration!
    extern const int menuItemCount;      // Nur Deklaration!

    // Initialize the configuration system
    void init();
    
    // Display configuration menu
    void displayConfigMenu();
    
    // Process configuration menu selections
    void processConfigMenu();
    
    // Save and load configurations
    bool saveConfig(const char* filename);
    bool loadConfig(const char* filename);
    
    // List available configurations
    void listConfigurations();
}