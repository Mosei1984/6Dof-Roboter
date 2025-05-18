#pragma once

#include <Arduino.h>

// Gear ratios for each joint (steps per degree multiplier)
// These are likely used in the stepper configuration
const float gearRatios[6] = {
  6.25f,   // Achse 1: GT2 16T → 100T
  25.0f,   // Achse 2: 5:1 Planetary + 16T → 80T (HTD)
  5.0f,    // Achse 3: 16T → 80T (GT2)
  3.75f,   // Achse 4: 16T → 60T
  2.0f,    // Achse 5: 16T → 32T
  1.0f     // Achse 6: Direktantrieb
};

// Motor direction inversion flags
const bool invertMotorDirection[6] = {
  false,  // J1: Base rotation
  false,  // J2: Shoulder
  false,  // J3: Elbow
  false,  // J4: Wrist pitch
  false,  // J5: Wrist roll
  false   // J6: Gripper
};

// Homing menu constants for DisplaySystem
enum HomingMenuOptions {
  HOMING_MENU_START_HOMING = 0,
  HOMING_MENU_TO_CENTER,
  HOMING_MENU_SAVE_HOME,
  HOMING_MENU_LOAD_HOME,
  HOMING_MENU_SAVE_HOME_SD,
  HOMING_MENU_LOAD_HOME_SD,
  HOMING_MENU_LIST_HOME_SD,
  HOMING_MENU_CLEAR_HOME,
  HOMING_MENU_CONFIG,
  HOMING_MENU_COUNT
};