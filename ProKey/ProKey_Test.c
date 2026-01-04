#include <stdio.h>
#include <string.h>
#include "prokey.h"

// Einfache Makros für farbige Ausgabe auf PC, normale auf AVR
#if defined(_WIN32) || defined(__linux__) || defined(__APPLE__)
    #define COLOR_GREEN "\033[0;32m"
    #define COLOR_RED   "\033[0;31m"
    #define COLOR_RESET "\033[0m"
#else
    #define COLOR_GREEN ""
    #define COLOR_RED   ""
    #define COLOR_RESET ""
#endif

// Statistik
int tests_run = 0;
int tests_passed = 0;

// Hilfsfunktion für Assertions
void assert_test(const char* test_name, int condition) {
    tests_run++;
    printf("TEST %02d: %-35s ", tests_run, test_name);
    if (condition) {
        printf("[%sPASS%s]\n", COLOR_GREEN, COLOR_RESET);
        tests_passed++;
    } else {
        printf("[%sFAIL%s]\n", COLOR_RED, COLOR_RESET);
    }
}

// --------------------------------------------------------------------------
// TEST CASES
// --------------------------------------------------------------------------

void test_raw_key_generation() {
    prokey_id_t k1 = ProKey_GetRawKey();
    // Der Key darf nicht 0 sein (0 ist Fehlerindikator)
    assert_test("Raw Key valid (!= 0)", k1 != 0);
}

void test_key_uniqueness() {
    // Wir ziehen zwei Keys schnell hintereinander.
    // Sie MÜSSEN unterschiedlich sein, sonst funktioniert der RNG nicht.
    prokey_id_t k1 = ProKey_GetRawKey();
    prokey_id_t k2 = ProKey_GetRawKey();
    
    assert_test("Entropy Check (k1 != k2)", k1 != k2);
}

void test_error_handling_null() {
    // Wir übergeben NULL als Stream -> Erwarten Fehlercode
    ProKey_Status status = ProKey_GenerateAndStream(NULL);
    assert_test("Error Handling (NULL Stream)", status == PROKEY_ERROR_PARAM);
}

void test_stdout_streaming() {
    printf("\n--- [Manuelle Prüfung Stream] ---\n");
    // Wir streamen direkt in die Konsole (stdout)
    ProKey_Status status = ProKey_GenerateAndStream(stdout);
    assert_test("Stream to stdout success", status == PROKEY_SUCCESS);
    printf("--- [Ende Manuelle Prüfung] ---\n");
}

#ifndef __AVR__
// Dateisystem-Tests (nur auf PC/Server sinnvoll)
void test_file_io() {
    const char* filename = "test_key.dump";
    FILE* f = fopen(filename, "w");
    
    if (f) {
        // Schreiben testen
        ProKey_Status status = ProKey_GenerateAndStream(f);
        fclose(f);
        assert_test("File Write Return Code", status == PROKEY_SUCCESS);

        // Lesen und Validieren
        f = fopen(filename, "r");
        char buffer[64] = {0};
        if (f) {
            fgets(buffer, sizeof(buffer), f);
            fclose(f);
            
            // Prüfen ob Format "KEY:0x..." eingehalten wurde
            int valid_format = (strstr(buffer, "KEY:0x") != NULL);
            assert_test("File Content Verification", valid_format);
        } else {
            assert_test("File Re-Open Verification", 0);
        }
        
        remove(filename); // Aufräumen
    } else {
        assert_test("File Creation Permission", 0);
    }
}
#endif

// --------------------------------------------------------------------------
// MAIN ENTRY
// --------------------------------------------------------------------------

int main(void) {
    // WICHTIG FÜR AVR: Hier müsste vorher die UART Init stehen
    // und stdout = &uart_stream zugewiesen sein (siehe main.c vorher).
    
    printf("\n=========================================\n");
    printf("   ProKey Test Suite v%d.%d\n", PROKEY_VERSION_MAJOR, PROKEY_VERSION_MINOR);
    #ifdef __AVR__
    printf("   Platform: AVR Embedded (No Filesystem)\n");
    #else
    printf("   Platform: PC / Server (Filesystem Active)\n");
    #endif
    printf("=========================================\n");

    // 1. Logik & Krypto Tests
    test_raw_key_generation();
    test_key_uniqueness();
    
    // 2. API Robustheit
    test_error_handling_null();

    // 3. I/O Tests
    #ifndef __AVR__
    test_file_io();
    #endif
    
    test_stdout_streaming();

    // Abschlussbericht
    printf("\n=========================================\n");
    printf("RESULT: %d/%d Tests Passed.\n", tests_passed, tests_run);
    
    if (tests_passed == tests_run) {
        printf("%sALL SYSTEMS GO.%s\n", COLOR_GREEN, COLOR_RESET);
        return 0; // Exit Code 0 für CI/CD Pipelines
    } else {
        printf("%sSYSTEM FAILURE.%s\n", COLOR_RED, COLOR_RESET);
        return -1;
    }
}
