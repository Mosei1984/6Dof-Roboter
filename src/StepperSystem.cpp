#include "StepperSystem.h"
#include "Debug.h"
#include "RobotSystem.h"
#include "ConfigSystem.h"

// External configuration references
extern PinConfig _pinConfig;
extern StepperConfig _stepperConfig[6];

namespace StepperSystem {
    // Array of stepper motor instances
    AccelStepper* steppers[6] = {nullptr};
    void synchronizeWithKinematics() {
        JointAngles ja;
        for (int i = 0; i < 6; i++) {
            long steps = steppers[i]->currentPosition();
            float angle = steps / _stepperConfig[i].stepsPerDegree;
            ja.angles[i] = angle;
        }
        RobotSystem::getKinematics()->setCurrentJointAngles(ja);
    }
    
    // Selected joint for joint mode
    int selectedJoint = 0;
    
    void init() {
        Debug::println(F("Initializing stepper system..."));
        
        // Initialize each stepper motor
        for (int i = 0; i < 6; i++) {
            // Create stepper with Step/Dir pins
            steppers[i] = new AccelStepper(
                AccelStepper::DRIVER, 
                _pinConfig.stepperPins[i][0],   // STEP Pin
                _pinConfig.stepperPins[i][1]    // DIR Pin
            );
            
            // Debug output of stepper pins
            Debug::print(F("Stepper "));
            Debug::print(i);
            Debug::print(F(" Pins - STEP: "));
            Debug::print(_pinConfig.stepperPins[i][0]);
            Debug::print(F(", DIR: "));
            Debug::print(_pinConfig.stepperPins[i][1]);
            Debug::print(F(", ENABLE: "));
            Debug::println(_pinConfig.stepperPins[i][2]);
            
            // Set enable pin as OUTPUT
            pinMode(_pinConfig.stepperPins[i][2], OUTPUT);
            digitalWrite(_pinConfig.stepperPins[i][2], HIGH);  // Initially disabled (active LOW)
            
            // Set limit/endstop pin as INPUT with pull-up
            pinMode(_pinConfig.stepperPins[i][3], INPUT_PULLUP);
            
            // Configure stepper parameters
            steppers[i]->setMaxSpeed(_stepperConfig[i].maxSpeed);
            steppers[i]->setAcceleration(_stepperConfig[i].acceleration);
            
            // Debug stepper configuration
            Debug::print(F("Stepper "));
            Debug::print(i);
            Debug::print(F(" Config - MaxSpeed: "));
            Debug::print(_stepperConfig[i].maxSpeed);
            Debug::print(F(", Accel: "));
            Debug::print(_stepperConfig[i].acceleration);
            Debug::print(F(", StepsPerDegree: "));
            Debug::println(_stepperConfig[i].stepsPerDegree);
        }
        
        Debug::println(F("Stepper system initialized"));
    }
    
    void testSteppers() {
        Debug::println(F("Testing stepper communication..."));
        
        for (int i = 0; i < 6; i++) {
            // Enable motor
            digitalWrite(_pinConfig.stepperPins[i][2], LOW);
            
            // Perform short movement
            steppers[i]->setCurrentPosition(0);
            steppers[i]->moveTo(100);  // 100 steps forward
            
            // Wait for movement to complete
            while (steppers[i]->distanceToGo() != 0) {
                steppers[i]->run();
                delay(1);
            }
            
            delay(200);  // Short pause
            
            // Back to starting position
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
        
        Debug::println(F("Stepper test completed"));
    }
    
    void update() {
        // Motor movement logic for all axes
        bool anyMotorRunning = false;
        
        for (int i = 0; i < 6; i++) {
            // If this motor has distance to travel
            if (steppers[i]->distanceToGo() != 0) {
                // Run the motor
                steppers[i]->run();
                anyMotorRunning = true;
                
                                // Activate the motor if it's moving
                digitalWrite(_pinConfig.stepperPins[i][2], LOW);
                
                // Debug output (only for selected joint to avoid spam)
                if (i == selectedJoint) {
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
                // Disable motor when target reached (power saving)
                digitalWrite(_pinConfig.stepperPins[i][2], HIGH);
            }
        }
        
        // If any motors moved, synchronize the kinematic model
        if (anyMotorRunning) {
            RobotSystem::synchronizeKinematicsWithSteppers();
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
            
            // Read initial state of endstop
            lastSwitchState = digitalRead(_pinConfig.stepperPins[jointIndex][3]);
            
            // Debug output
            Debug::print(F("Endstop pin: "));
            Debug::print(_pinConfig.stepperPins[jointIndex][3]);
            Debug::print(F(", initial state: "));
            Debug::println(lastSwitchState == HIGH ? "HIGH" : "LOW");
        }
        
        // Check endstop
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
                
                // Back away from limit switch (positive direction, away from limit switch)
                steppers[jointIndex]->move(100); // About 5 degrees back with 20 steps/degree
                steppers[jointIndex]->setSpeed(_stepperConfig[jointIndex].homingSpeed * 0.5);
                
                while (steppers[jointIndex]->distanceToGo() != 0) {
                    steppers[jointIndex]->runSpeed();
                }
                
                delay(100);
                
                Debug::print(F("Retreat completed, starting precise homing for joint "));
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
                
                JointAngles currentAngles = RobotSystem::getKinematics()->getCurrentJointAngles();
                currentAngles.angles[jointIndex] = 0.0f;
                RobotSystem::getKinematics()->setCurrentJointAngles(currentAngles);
                
                homingJointStarted = false;
                coarseHomingDone = false;
                return true; // Homing for this joint completed
            }
            
            lastSwitchState = currentSwitchState;
            steppers[jointIndex]->runSpeed();
        }
        
        return false; // Homing for this joint still in progress
    }
    
    int getSelectedJoint() {
        return selectedJoint;
    }
    
    void setSelectedJoint(int joint) {
        if (joint >= 0 && joint < 6) {
            selectedJoint = joint;
        }
    }
    
    // Enable all motors
    void enableAllMotors() {
        for (int i = 0; i < 6; i++) {
            digitalWrite(_pinConfig.stepperPins[i][2], LOW);
        }
    }
    
    // Disable all motors
    void disableAllMotors() {
        for (int i = 0; i < 6; i++) {
            digitalWrite(_pinConfig.stepperPins[i][2], HIGH);
        }
    }
}
