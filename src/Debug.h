#pragma once

#include <Arduino.h>

namespace Debug {
    extern bool enabled;

    void print(const char* msg);
    void println(const char* msg);

    void print(const __FlashStringHelper* msg);
    void println(const __FlashStringHelper* msg);

    void print(int val);
    void println(int val);

    void print(long val);
    void println(long val);

    void print(unsigned long val);
    void println(unsigned long val);

    void print(float val, int digits = 2);
    void println(float val, int digits = 2);

    void print(double val, int digits = 2);
    void println(double val, int digits = 2);
}