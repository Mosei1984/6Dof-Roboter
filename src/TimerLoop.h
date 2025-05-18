#pragma once

#include <Arduino.h>
#include <TimerOne.h>

namespace TimerLoop {
    void begin(void (*controlFunc)());
    void loop(void (*joystickFunc)(), void (*displayFunc)());
}