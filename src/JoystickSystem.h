#pragma once

#include <Arduino.h>
#include "joystick.h"

namespace JoystickSystem {
    // Joystick states
    enum JoystickState {
        JOYSTICK_CENTERED,
        JOYSTICK_LEFT,
        JOYSTICK_RIGHT,
        JOYSTICK_UP,
        JOYSTICK_DOWN
    };
    
    // Global joystick instances
    extern Joystick* leftJoystick;
    extern Joystick* rightJoystick;
    
    // Initialization
    void init();
    void update();
    
    // Calibration
    void calibrateJoysticks();
    void startFullCalibration();
    
    // Input processing
    void processJointControl();
    void processKinematicControl();
    void processButtonInput();
    
    // Joystick state getters
    JoystickState getJoystickXState();
    JoystickState getJoystickYState();
    
    // Button state getters (new)
    bool isLeftButtonPressed();
    bool isRightButtonPressed();
}
