#pragma once

#include <Arduino.h>
#include <TimerOne.h>

namespace TimerLoop {
    // Add the correct signature with all three callbacks
    void begin(void (*controlFunc)() = nullptr, 
               void (*joystickFunc)() = nullptr, 
               void (*displayFunc)() = nullptr);
    
    // Fix the loop signature to take no arguments
    void loop();
}
