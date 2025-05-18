#pragma once
#include <stdint.h>

// Konfiguration für eine einzelne Joystick-Achse
struct AxisConfig {
    bool invert;   // true = Achse invertieren (umkehren)
    float scale;   // Skalierungsfaktor für den Achswert (Multiplikator)
};

// Anzahl der Achsen im Joystick (z.B. 2 Achsen: X und Y)
const int AXIS_COUNT = 2;

// Struktur für die gesamte Achsen-Konfiguration, inklusive Magic-Key für Gültigkeit
struct AxesConfig {
    uint32_t magic;              // Magic-Wert zur Validierung der gespeicherten Daten
    AxisConfig axis[AXIS_COUNT]; // Array mit Achsen-Einstellungen
};

// Globale Konfigurationsinstanz (Deklaration zur externen Nutzung)
extern AxesConfig axeCfg;

// Funktionen zum Laden/Speichern der Achsenkonfiguration aus/in EEPROM
void loadAxesConfig();
void saveAxesConfig();
