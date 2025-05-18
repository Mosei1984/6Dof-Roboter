#pragma once

#include <Arduino.h>
#include "RobotKinematics.h"
#include "config.h"
#include <AccelStepper.h>

// SD Card pins for Teensy
#define BUILTIN_SDCARD 10  // Adjust according to your board

// Global pointers - declare as extern here so all files can see them
extern AccelStepper* _steppers[6];
extern RobotConfig g_robotConfig;

// Declare external global variables
extern PinConfig _pinConfig;
extern struct StepperConfig _stepperConfig[6];