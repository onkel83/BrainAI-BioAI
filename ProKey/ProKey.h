#ifndef PROKEY_H
#define PROKEY_H

#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================================================
   TEIL 1: CONFIG (Konfiguration & Plattform-Abstraktion)
   Hier werden DLL-Export-Regeln und grundlegende Compiler-Einstellungen definiert.
   ========================================================================== */
#include <stdio.h>
#include <stdint.h>
#include <stddef.h>

// Versionierung des ProKey Systems
#define PROKEY_VERSION_MAJOR 1
#define PROKEY_VERSION_MINOR 0

// --------------------------------------------------------------------------
// DLL Export / Import Magie
// --------------------------------------------------------------------------
// Wir erkennen automatisch Windows für DLL-Export.
// Auf Embedded-Systemen (AVR/ARM statisch) wird das Makro leer definiert.

#if defined(_WIN32) || defined(_WIN64)
    #ifdef BUILD_PROKEY_DLL
        // Wir kompilieren die DLL gerade selbst (Export)
        #define PROKEY_API __declspec(dllexport)
    #else
        // Wir nutzen die DLL als Client (Import)
        #define PROKEY_API __declspec(dllimport)
    #endif
#else
    // Linux, Unix, AVR, ARM (Static Library oder ELF)
    // Hier werden keine speziellen Keywords benötigt.
    #define PROKEY_API
#endif

// Optional: Feature-Flags für das Stream-Format
// Wenn gesetzt, wird nur der rohe Hex-String ohne "KEY:" Prefix ausgegeben.
// #define PROKEY_CONFIG_RAW_OUTPUT 

/* ==========================================================================
   TEIL 2: STRUKTUR (Datentypen & Definitionen)
   Hier definieren wir unsere eigenen Typen, um unabhängig von 'int' oder 'long' zu bleiben.
   ========================================================================== */

// Der Kern-Datentyp: Ein 64-Bit unsigned Integer
// Wir nutzen typedef, damit wir den Typ später ändern könnten (z.B. auf 128 Bit struct),
// ohne den Anwender-Code zu brechen.
typedef uint64_t prokey_id_t;

// Rückgabewerte (Status Codes)
// Enums sind in C sicherer und lesbarer als nackte 0 oder 1 Rückgaben.
typedef enum {
    PROKEY_ERROR_HARDWARE = 0,  // Kritischer Hardware-Fehler (RNG defekt)
    PROKEY_SUCCESS        = 1,  // Key erfolgreich generiert und gestreamt
    PROKEY_ERROR_STREAM   = -1, // Fehler beim Schreiben in den Stream (File/UART)
    PROKEY_ERROR_PARAM    = -2  // Ungültige Parameter (z.B. Stream ist NULL)
} ProKey_Status;

// (Optional) Struktur für erweiterte Rückgaben, falls wir später mehr als nur 
// den Status brauchen (z.B. Zeitstempel oder Entropie-Qualität).
typedef struct {
    prokey_id_t key;
    ProKey_Status status;
} ProKey_Result_t;

/* ==========================================================================
   TEIL 3: INTERFACE (Öffentliche Schnittstelle)
   Die Funktionen, die der Anwender aufruft.
   ========================================================================== */

/**
 * @brief Generiert einen Hardware-nahen Key und schreibt ihn in den gegebenen Stream.
 * * Diese Funktion wählt automatisch die beste Entropiequelle für die Architektur:
 * - x86/x64: CPU RDRAND Instruktion
 * - AVR: ADC Thermisches Rauschen
 * - Generic: SRAM Garbage & Jitter
 * * @param output_stream Der Ziel-Stream (z.B. stdout, ein Datei-Handle oder UART-Stream).
 * @return ProKey_Status (PROKEY_SUCCESS bei Erfolg).
 */
PROKEY_API ProKey_Status ProKey_GenerateAndStream(FILE* output_stream);

/**
 * @brief Nur den Key holen, ohne zu streamen (Hilfsfunktion).
 * * Nützlich, wenn der Key binär weiterverarbeitet werden soll, statt textuell ausgegeben zu werden.
 * * @return prokey_id_t Der generierte 64-Bit Key (oder 0 im Fehlerfall).
 */
PROKEY_API prokey_id_t ProKey_GetRawKey(void);

#ifdef __cplusplus
}
#endif

#endif // PROKEY_H
