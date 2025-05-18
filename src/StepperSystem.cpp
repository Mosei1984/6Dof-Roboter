#include "StepperSystem.h"
#include "Debug.h"
#include "config.h"
#include "RobotSystem.h"
#include "JoystickSystem.h"
#include "DisplaySystem.h"

namespace StepperSystem {
    // Array of stepper motor pointers
    AccelStepper* steppers[6] = {nullptr};
    
    void init() {
        Debug::println(F("Initializing stepper system..."));
        
        // Initialize steppers and limit switches
        for (int i = 0; i < 6; i++) {
            // Set Step/Dir pins for each motor
            steppers[i] = new AccelStepper(AccelStepper::DRIVER, 
                                      _pinConfig.stepperPins[i][0],   // STEP Pin
                                      _pinConfig.stepperPins[i][1]);  // DIR Pin
            
            // Debug: Output stepper pins
            Debug::print(F("Stepper "));
            Debug::print(i);
            Debug::print(F(" Pins - STEP: "));
            Debug::print(_pinConfig.stepperPins[i][0]);
            Debug::print(F(", DIR: "));
            Debug::print(_pinConfig.stepperPins[i][1]);
            Debug::print(F(", ENABLE: "));
            Debug::println(_pinConfig.stepperPins[i][2]);
            
            // Set ENABLE pin as OUTPUT
            pinMode(_pinConfig.stepperPins[i][2], OUTPUT);
            digitalWrite(_pinConfig.stepperPins[i][2], HIGH);  // Disable at start (active LOW)
            
            // Set Limit/Endstop pin as INPUT with Pull-up
            pinMode(_pinConfig.stepperPins[i][3], INPUT_PULLUP);
            
            // Set stepper parameters
            steppers[i]->setMaxSpeed(_stepperConfig[i].maxSpeed);
            steppers[i]->setAcceleration(_stepperConfig[i].acceleration);
            
            // Debug: Output stepper configuration
            Debug::print(F("Stepper "));
            Debug::print(i);
            Debug::print(F(" Config - MaxSpeed: "));
            Debug::print(_stepperConfig[i].maxSpeed);
            Debug::print(F(", Accel: "));
            Debug::print(_stepperConfig[i].acceleration);
            Debug::print(F(", StepsPerDegree: "));
            Debug::println(_stepperConfig[i].stepsPerDegree);
        }
        
        // Test stepper motors (basic movement)
        testSteppers();
        
        Debug::println(F("Stepper system initialized"));
    }
    
    void update() {
        // Update all steppers (high frequency)
        bool anyMotorRunning = false;
        for (int i = 0; i < 6; i++) {
            if (steppers[i]->distanceToGo() != 0) {
                steppers[i]->run();
                anyMotorRunning = true;
                
                // If the motor is moving, we output it
                if (i == JoystickSystem::getSelectedJoint()) {  // Use JoystickSystem function here
                    Debug::print(F("Stepper "));
                    Debug::print(i);
                    Debug::print(F(" moving to position "));
                    Debug::print(steppers[i]->targetPosition());
                    Debug::print(F(", current: "));
                    Debug::print(steppers[i]->currentPosition());
                    Debug::print(F(", remaining: "));
                    Debug::println(steppers[i]->distanceToGo());
                }
            } else {
                // Disable motor when target is reached
                digitalWrite(_pinConfig.stepperPins[i][2], HIGH);
            }
        }
        
        // If motors were moved, synchronize the kinematics model
        if (anyMotorRunning) {
            synchronizeKinematicsWithSteppers();
        }
    }
    
    void testSteppers() {
        Debug::println(F("Testing stepper communication..."));
        for (int i = 0; i < 6; i++) {
            // Enable motor
            digitalWrite(_pinConfig.stepperPins[i][2], LOW);
            
            // Execute short movement
            steppers[i]->setCurrentPosition(0);
            steppers[i]->moveTo(100);  // 100 steps forward
            
            // Wait for movement
            while (steppers[i]->distanceToGo() != 0) {
                steppers[i]->run();
                delay(1);
            }
            
            delay(200);  // Short pause
            
            // Back to start position
            steppers[i]->moveTo(0);
            
            while (steppers[i]->distanceToGo() != 0) {
                steppers[i]->run();
                delay(1);
            }
            
            // Disable motor
            digitalWrite(_pinConfig.stepperPins[i][2], HIGH);
            
            Debug::print(F("Stepper "));
            Debug::print(i);
            Debug::println(F(" tested"));
            
            delay(500);  // Pause between tests
        }
    }
    
    bool homeJoint(int jointIndex) {
        static bool homingJointStarted = false;
        static bool coarseHomingDone = false;
        static int lastSwitchState = HIGH;
    
        // First initialization for this joint
        if (!homingJointStarted) {
            homingJointStarted = true;
            coarseHomingDone = false;
    
            Debug::print(F("Starting homing for joint "));
            Debug::println(jointIndex + 1);
    
            // Enable motor
            digitalWrite(_pinConfig.stepperPins[jointIndex][2], LOW);
    
            // Set fast homing speed for coarse search
            steppers[jointIndex]->setSpeed(-_stepperConfig[jointIndex].homingSpeed);
    
            // Remember current position
            steppers[jointIndex]->setCurrentPosition(0);
    
            // Read initial state of limit switch
            lastSwitchState = digitalRead(_pinConfig.stepperPins[jointIndex][3]);
    
            // Debug output
            Debug::print(F("Limit switch pin: "));
            Debug::print(_pinConfig.stepperPins[jointIndex][3]);
            Debug::print(F(", Initial state: "));
            Debug::println(lastSwitchState == HIGH ? "HIGH" : "LOW");
        }
    
        // Check limit switch
        int limitSwitchPin = _pinConfig.stepperPins[jointIndex][3];
        int currentSwitchState = digitalRead(limitSwitchPin);
    
        // Phase 1: Coarse search for the limit switch
        if (!coarseHomingDone) {
            if (currentSwitchState != lastSwitchState && currentSwitchState == LOW) {
                Debug::print(F("Limit switch for joint "));
                Debug::print(jointIndex + 1);
                Debug::println(F(" reached during coarse homing"));
    
                // Stop motor
                steppers[jointIndex]->setSpeed(0);
                steppers[jointIndex]->stop();
    
                delay(100);
    
                // Back away from the limit switch (positive direction, away from the switch)
                steppers[jointIndex]->move(100); // About 5 degrees back at 20 steps/degree
                steppers[jointIndex]->setSpeed(_stepperConfig[jointIndex].homingSpeed * 0.5);
    
                while (steppers[jointIndex]->distanceToGo() != 0) {
                    steppers[jointIndex]->runSpeed();
                }
    
                delay(100);
    
                Debug::print(F("Retraction complete, starting precise homing for joint "));
                Debug::println(jointIndex + 1);
    
                // Start slow homing (1/4 of normal speed)
                steppers[jointIndex]->setSpeed(-_stepperConfig[jointIndex].homingSpeed * 0.25);
    
                coarseHomingDone = true;
                lastSwitchState = HIGH; // Reset for fine search
            }
    
            lastSwitchState = currentSwitchState;
            steppers[jointIndex]->runSpeed();
        }
        // Phase 2: Precise slow search
        else {
            if (currentSwitchState != lastSwitchState && currentSwitchState == LOW) {
                Debug::print(F("Limit switch for joint "));
                Debug::print(jointIndex + 1);
                Debug::print(F(" reached during precise homing (Pin "));
                Debug::print(limitSwitchPin);
                Debug::println(F(")"));
    
                steppers[jointIndex]->setSpeed(0);
                steppers[jointIndex]->stop();
    
                steppers[jointIndex]->setCurrentPosition(0);
    
                digitalWrite(_pinConfig.stepperPins[jointIndex][2], HIGH);
    
                // FIX: Use RobotSystem::getKinematics() instead of robotKin
                JointAngles currentAngles = RobotSystem::getKinematics()->getCurrentJointAngles();
                currentAngles.angles[jointIndex] = 0.0f;
                RobotSystem::getKinematics()->setCurrentJointAngles(currentAngles);
    
                homingJointStarted = false;
                coarseHomingDone = false;
                return true; // Homing complete for this joint
            }
    
            lastSwitchState = currentSwitchState;
            steppers[jointIndex]->runSpeed();
        }
    
        return false; // Homing still in progress for this joint
    }
    
    void synchronizeKinematicsWithSteppers() {
        // FIX: Use RobotSystem::getKinematics() instead of robotKin
        JointAngles currentAngles = RobotSystem::getKinematics()->getCurrentJointAngles();
        bool updated = false;
        
        for (int i = 0; i < 6; i++) {
            // Convert current stepper position to degrees
            long currentSteps = steppers[i]->currentPosition();
            float currentDegrees = currentSteps / _stepperConfig[i].stepsPerDegree;
            float currentRadians = currentDegrees * M_PI / 180.0;
            
            // If there's a difference, update the kinematics
            if (fabs(currentRadians - currentAngles.angles[i]) > 0.01) {
                currentAngles.angles[i] = currentRadians;
                updated = true;
                
                Debug::print(F("Synchronizing joint "));
                Debug::print(i + 1);
                Debug::print(F(": "));
                Debug::print(currentDegrees);
                Debug::print(F("° ("));
                Debug::print(currentSteps);
                Debug::println(F(" steps)"));
            }
        }
        
        if (updated) {
            RobotSystem::getKinematics()->setCurrentJointAngles(currentAngles);
        }
    }
    
    void exitKinematicMode() {
        for (int i = 0; i < 6; ++i) {
            steppers[i]->moveTo(steppers[i]->currentPosition());
        }
        synchronizeKinematicsWithSteppers();
    }
    
    void enableMotor(int index, bool enable) {
        digitalWrite(_pinConfig.stepperPins[index][2], enable ? LOW : HIGH);
    }
    
    // Selected joint
    int selectedJoint = 0;
    
    int getSelectedJoint() {
        return selectedJoint;
    }
    
    void setSelectedJoint(int joint) {
        if (joint >= 0 && joint < 6) {
            selectedJoint = joint;
        }
    }
}