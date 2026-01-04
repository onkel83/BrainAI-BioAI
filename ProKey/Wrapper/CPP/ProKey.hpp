#ifndef PROKEY_HPP
#define PROKEY_HPP

#include <iostream>
#include <string>
#include <sstream>
#include <iomanip>
#include <fstream>
#include <stdexcept>

// C-Interface einbinden
#include "prokey.h"

namespace prokey {

    /**
     * @brief Exception-Klasse für Hardware-Fehler beim RNG.
     */
    class HardwareException : public std::runtime_error {
    public:
        HardwareException() : std::runtime_error("ProKey Hardware RNG Failure: Could not generate entropy.") {}
    };

    /**
     * @brief Die C++ Klasse für ProKey. 
     * Kapselt den C-Aufruf und bietet moderne Methoden.
     */
    class Generator {
    public:
        // --- Core Funktionalität ---

        /**
         * @brief Holt einen rohen 64-Bit Key von der Hardware.
         * @throws prokey::HardwareException wenn die Hardware versagt (0 zurückgibt).
         */
        static uint64_t next() {
            uint64_t key = ProKey_GetRawKey();
            if (key == 0) {
                throw HardwareException();
            }
            return key;
        }

        // --- Convenience Methoden ---

        /**
         * @brief Gibt den Key als formatierten Hex-String zurück ("KEY:0x...")
         */
        static std::string nextString() {
            std::stringstream ss;
            ss << "KEY:0x" 
               << std::setfill('0') << std::setw(16) << std::uppercase << std::hex 
               << next();
            return ss.str();
        }

        /**
         * @brief Schreibt den Key direkt in einen C++ Stream (cout, fstream, stringstream).
         */
        static void toStream(std::ostream& os) {
            os << nextString() << std::endl;
        }

        /**
         * @brief Hilfsmethode: Öffnet eine Datei, schreibt den Key und schließt sie wieder.
         * Nutzt std::ofstream für RAII Sicherheit.
         */
        static void toFile(const std::string& filename, bool append = false) {
            std::ofstream file(filename, append ? std::ios::app : std::ios::out);
            if (!file.is_open()) {
                throw std::runtime_error("ProKey: Could not open file for writing: " + filename);
            }
            toStream(file);
        }
        
        // --- Kompatibilität mit Legacy C-Streams ---
        
        /**
         * @brief Wrapper für die originale C-Funktion ProKey_GenerateAndStream
         */
        static bool toCFile(FILE* c_stream) {
             return ProKey_GenerateAndStream(c_stream) == PROKEY_SUCCESS;
        }
    };
}

#endif // PROKEY_HPP
