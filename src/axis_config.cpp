#include "axis_config.h"
#include <EEPROM.h>

// Magic-Schlüssel zur Validierung der EEPROM-Daten (beliebiger eindeutiger 32-Bit Wert)
constexpr uint32_t EEPROM_MAGIC = 0x12345678; 

// Definition und Initialisierung der globalen Achsenkonfiguration mit Standardwerten
AxesConfig axeCfg = {
    EEPROM_MAGIC,                      // Magic-Wert (sollte beim Laden überprüft werden)
    { 
        { false, 1.0f },               // Achse 0: nicht invertiert, Skalierung 1.0 (100%)
        { false, 1.0f }                // Achse 1: nicht invertiert, Skalierung 1.0 (100%)
    }
};

// Lädt die Achsenkonfiguration aus dem EEPROM (beginnend ab Adresse 0)
void loadAxesConfig() {
    EEPROM.get(0, axeCfg);  // Struktur aus EEPROM lesen
    if (axeCfg.magic != EEPROM_MAGIC) {
        // EEPROM-Daten ungültig oder noch nicht geschrieben – Standardwerte setzen
        axeCfg.magic = EEPROM_MAGIC;
        for (int i = 0; i < AXIS_COUNT; i++) {
            axeCfg.axis[i].invert = false;
            axeCfg.axis[i].scale  = 1.0f;
        }
        // (Optional: Hier könnte saveAxesConfig() aufgerufen werden, um die Default-Werte ins EEPROM zu schreiben)
    }
}

// Speichert die aktuelle Achsenkonfiguration dauerhaft im EEPROM
void saveAxesConfig() {
    axeCfg.magic = EEPROM_MAGIC;      // Magic-Wert aktualisieren (für den Fall, dass er geändert wurde)
    EEPROM.put(0, axeCfg);            // Struktur ab Adresse 0 ins EEPROM schreiben
    // Hinweis: Auf Teensy 4.1 ist kein explizites EEPROM.commit() erforderlich – 
    // die Daten werden direkt im Flash-EEPROM abgelegt.
}
