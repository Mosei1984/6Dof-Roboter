#pragma once

#include <AccelStepper.h>
#include "config.h"

namespace StepperSystem {
    // Public stepper motor array
    extern AccelStepper* steppers[6];
    
    // Initialize stepper motors and limit switches
    void init();
    
    // Update stepper motors (called at high frequency)
    void update();
    
    // Test stepper motors with basic movements
    void testSteppers();
    
    // Home a specific joint (returns true when done)
    bool homeJoint(int jointIndex);
    
    // Synchronize kinematic model with stepper positions
    void synchronizeKinematicsWithSteppers();
}