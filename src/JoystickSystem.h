#pragma once

#include "joystick.h"
#include "kalmanfilter.h"

namespace JoystickSystem {
    // Joystick state enum (moved from cpp file to header)
    enum JoystickState {
        JOYSTICK_CENTERED,
        JOYSTICK_LEFT,
        JOYSTICK_RIGHT,
        JOYSTICK_UP,
        JOYSTICK_DOWN
    };
    
    // Initialize joystick system
    void init();
    
    // Update joystick readings
    void update();
    
    // Process button inputs
    void processButtonInput();
    
    // Process joint control mode
    void processJointControl();
    
    // Process kinematic control mode
    void processKinematicControl();
    
    // Exit kinematic mode (helper function)
    void exitKinematicMode();
    
    // Selected joint management
    int getSelectedJoint();
    void setSelectedJoint(int joint);
    
    // Basic joystick calibration (center position)
    void calibrateJoysticks();
    
    // Full joystick calibration (extremes and center)
    void startFullCalibration();
    
    // Joystick objects access
    extern Joystick* leftJoystick;
    extern Joystick* rightJoystick;
}