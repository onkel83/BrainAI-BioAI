#include "prokey.h"
#include <string.h>

/* ==========================================================================
   HELPER: Mix-Funktion (MurmurHash3-artig)
   Verteilt Bits gleichmäßig, damit "schlechter" Zufall besser aussieht.
   ========================================================================== */
static uint64_t mix(uint64_t h) {
    h ^= h >> 33;
    h *= 0xff51afd7ed558ccdULL;
    h ^= h >> 33;
    h *= 0xc4ceb9fe1a85ec53ULL;
    h ^= h >> 33;
    return h;
}

/* ==========================================================================
   ARCHITEKTUR-LOGIK
   ========================================================================== */

// --- 1. INTEL/AMD x86/x64 (Perfekte Hardware-Lösung) ---
#if defined(__x86_64__) || defined(_M_X64) || defined(__i386__) || defined(_M_IX86)
    #include <immintrin.h>
    static uint64_t get_entropy(void) {
        #if defined(__x86_64__) || defined(_M_X64)
            unsigned long long val;
            if (_rdrand64_step(&val)) return (uint64_t)val;
        #else
            unsigned int v1, v2; // 2x 32-Bit für 64-Bit Key
            if (_rdrand32_step(&v1) && _rdrand32_step(&v2)) {
                return ((uint64_t)v1 << 32) | v2;
            }
        #endif
        return 0; 
    }

// --- 2. AVR 8-BIT (ADC Rauschen Lösung) ---
#elif defined(__AVR__)
    #include <avr/io.h>
    #include <util/delay.h>
    static uint64_t get_entropy(void) {
        // Setup ADC Noise (A0 Floating)
        ADMUX = (1 << REFS0);
        ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1); 
        uint64_t k = 0;
        for (int i = 0; i < 64; i++) {
            ADCSRA |= (1 << ADSC);
            while (ADCSRA & (1 << ADSC));
            k = (k << 1) | (ADC & 1); // Nur LSB nehmen
            _delay_us(50);
        }
        return k;
    }

// --- 3. FALLBACK: GENERIC 16/32-BIT (ARM, RISC-V, etc.) ---
// Hier implementieren wir den "Time-Jitter & SRAM Garbage" Trick
#else
    static uint64_t get_entropy(void) {
        
        // A) UNINITIALIZED MEMORY (SRAM Garbage)
        // Der Inhalt ist zufällig oder Restdaten vom vorherigen Programm
        volatile uint32_t trash_var;
        
        // B) ADDRESS ENTROPY (ASLR / Stack Pointer)
        // Wo diese Variable im Speicher liegt
        uintptr_t addr = (uintptr_t)&trash_var;

        // C) TIME JITTER LOOP (Das Neue Feature!)
        // Wir zählen hoch. Da volatile genutzt wird, muss die CPU arbeiten.
        // Wenn währenddessen IRQs (Interrupts) feuern (z.B. UART, Timer), 
        // braucht die Loop unterschiedlich viele Takte.
        volatile uint32_t jitter_counter = 0;
        // Die Loop-Länge hängt vom SRAM-Müll ab -> Unvorhersehbare Dauer
        uint32_t loops = 1000 + (trash_var & 0xFF); 
        
        for(uint32_t i=0; i < loops; i++) {
            jitter_counter++; 
            // Ein billiger Befehl, der die Pipeline stresst
            jitter_counter ^= (i << 2); 
        }

        // D) COMPILE TIME SALT
        // Verhindert, dass zwei verschiedene Firmware-Versionen exakt gleich starten
        // __TIME__ ist ein String wie "10:30:05"
        uint64_t compile_salt = 0;
        char *t = __TIME__;
        while(*t) compile_salt = (compile_salt << 5) + *t++;

        // ALLES ZUSAMMENMISCHEN
        uint64_t raw = (uint64_t)trash_var ^ 
                       (uint64_t)addr ^ 
                       (uint64_t)jitter_counter ^ 
                       compile_salt;
                       
        return mix(raw);
    }
#endif

/* ==========================================================================
   API IMPLEMENTIERUNG
   IMPLEMENTIERUNG DES NEUEN INTERFACES
   ========================================================================== */

// 1. Nur den Key holen (Neue Funktion)
PROKEY_API prokey_id_t ProKey_GetRawKey(void) {
    uint64_t key = get_entropy();
    
    // Safety check
    if (key == 0) key = ~key; 
    
    return (prokey_id_t)key;
}

// 2. Generieren und Streamen (Update der bestehenden Funktion)
PROKEY_API ProKey_Status ProKey_GenerateAndStream(FILE* output_stream) {
    // Test 1: Ist der Stream gültig?
    if (output_stream == NULL) {
        return PROKEY_ERROR_PARAM;
    }

    // Key holen
    prokey_id_t key = ProKey_GetRawKey();

    // Test 2: Hardware Fehler? (Sollte durch Fallbacks kaum passieren)
    if (key == 0) {
        return PROKEY_ERROR_HARDWARE;
    }

    // Schreiben
    int written = fprintf(output_stream, "KEY:0x%016llX\n", (unsigned long long)key);

    // Test 3: Konnte geschrieben werden? (fprintf gibt negative Zahl bei Fehler zurück)
    if (written < 0) {
        return PROKEY_ERROR_STREAM;
    }

    return PROKEY_SUCCESS;
}
