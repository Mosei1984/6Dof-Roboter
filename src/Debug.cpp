#include "Debug.h"

namespace Debug {
    bool enabled = true;

    void print(const char* msg)    { if (enabled) Serial.print(msg); }
    void println(const char* msg)  { if (enabled) Serial.println(msg); }

    void print(const __FlashStringHelper* msg)   { if (enabled) Serial.print(msg); }
    void println(const __FlashStringHelper* msg) { if (enabled) Serial.println(msg); }

    void print(int val)            { if (enabled) Serial.print(val); }
    void println(int val)          { if (enabled) Serial.println(val); }

    void print(long val)           { if (enabled) Serial.print(val); }
    void println(long val)         { if (enabled) Serial.println(val); }

    void print(unsigned long val)  { if (enabled) Serial.print(val); }
    void println(unsigned long val){ if (enabled) Serial.println(val); }

    void print(float val, int d)   { if (enabled) Serial.print(val, d); }
    void println(float val, int d) { if (enabled) Serial.println(val, d); }

    void print(double val, int d)  { if (enabled) Serial.print(val, d); }
    void println(double val, int d){ if (enabled) Serial.println(val, d); }
}