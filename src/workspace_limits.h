#pragma once
// Arbeitsbereichs-Grenzen (in mm)
constexpr float WSP_MIN_X = -200.0f, WSP_MAX_X = +200.0f;
constexpr float WSP_MIN_Y = -200.0f, WSP_MAX_Y = +200.0f;
constexpr float WSP_MIN_Z =   0.0f, WSP_MAX_Z = +300.0f;

// Einfaches float-Clamping
inline float clampf(float v, float lo, float hi) {
    return (v < lo ? lo : (v > hi ? hi : v));
}
