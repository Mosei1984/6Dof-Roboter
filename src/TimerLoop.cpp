#include "TimerLoop.h"

namespace {
    volatile bool doControl = false;
    void (*controlCallback)() = nullptr;
    void (*joystickCallback)() = nullptr;
    void (*displayCallback)() = nullptr;
    unsigned long lastJoystickMs = 0, lastDisplayMs = 0;
    
    void controlISR() {
        doControl = true;
    }
}

namespace TimerLoop {
    void begin(void (*controlFunc)(), void (*joystickFunc)(), void (*displayFunc)()) {
        controlCallback = controlFunc;
        joystickCallback = joystickFunc;
        displayCallback = displayFunc;
        
        Timer1.initialize(1000); // 1ms = 1kHz
        Timer1.attachInterrupt(controlISR);
    }
    
    void loop() {
        unsigned long now = millis();
        
        // 1kHz control tasks
        if (doControl) {
            doControl = false;
            if (controlCallback) controlCallback();
        }
        
        // 100Hz joystick sampling
        if (now - lastJoystickMs >= 10) {
            lastJoystickMs = now;
            if (joystickCallback) joystickCallback();
        }
        
        // 50Hz display refresh
        if (now - lastDisplayMs >= 20) {
            lastDisplayMs = now;
            if (displayCallback) displayCallback();
        }
    }
}
