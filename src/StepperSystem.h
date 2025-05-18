#pragma once

#include <Arduino.h>
#include <AccelStepper.h>
#include "RobotKinematics.h"

namespace StepperSystem {
    // Initialize the stepper system
    void init();
    
    // Update function - call in main loop
    void update();
    
    // Homing function for a specific joint
    bool homeJoint(int jointIndex);
    
    // Get current angles from stepper positions
    JointAngles getCurrentAnglesFromSteppers();
    
    // Synchronize kinematics with stepper positions
    void synchronizeWithKinematics();
    
    // Selected joint for joint mode
    int getSelectedJoint();
    void setSelectedJoint(int joint);
    
    // Steps per degree for a specific joint
    float getStepsPerDegree(int jointIndex);
    void setStepsPerDegree(int jointIndex, float stepsPerDegree);
    
    // Access to stepper instances
    extern AccelStepper* steppers[6];
}
