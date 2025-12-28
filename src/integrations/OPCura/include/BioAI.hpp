#pragma once

#include <string>
#include <vector>
#include <fstream>
#include <stdexcept>
#include <iostream>
#include <iomanip>

// Einbindung der nativen Schnittstelle
#include "BioAI_Interface.h"

// Einfaches JSON-Parsing (Beispielhaft mit nlohmann/json)
#include <nlohmann/json.hpp> 
using json = nlohmann::json;

namespace BioAI {

    /**
     * @brief Hochperformanter C++ Wrapper für die BioAI-Engine (v0.7.6).
     * Implementiert das RAII-Prinzip für automatische Speicherverwaltung.
     */
    class BioBrainInstance {
    private:
        void* _brainHandle;
        uint64_t _licenseKey;

    public:
        /**
         * @brief Erzeugt eine neue Instanz und lädt den Schlüssel aus einer JSON-Datei.
         * @param jsonPath Pfad zur ISS-generierten key.json.
         */
        explicit BioBrainInstance(const std::string& jsonPath) : _brainHandle(nullptr), _licenseKey(0) {
            // 1. Key aus JSON laden
            std::ifstream file(jsonPath);
            if (!file.is_open()) throw std::runtime_error("BioAI: key.json nicht gefunden.");

            json keyData;
            file >> keyData;
            std::string rawKey = keyData["customer_key"];

            // 2. Hex-String zu uint64_t konvertieren (Entfernt C-Suffixe)
            if (rawKey.find("ULL") != std::string::npos)
                rawKey.erase(rawKey.find("ULL"), 3);

            _licenseKey = std::stoull(rawKey, nullptr, 16);

            // 3. Native Instanz über die API erzeugen
            _brainHandle = API_CreateBrain(_licenseKey);
            if (!_brainHandle) throw std::runtime_error("BioAI: Initialisierung fehlgeschlagen.");
        }

        /**
         * @brief Destruktor garantiert die Freigabe der nativen Ressourcen.
         */
        ~BioBrainInstance() {
            if (_brainHandle) {
                API_FreeBrain(_brainHandle);
                _brainHandle = nullptr;
            }
        }

        // Kopieren verhindern, um Handle-Konflikte zu vermeiden
        BioBrainInstance(const BioBrainInstance&) = delete;
        BioBrainInstance& operator=(const BioBrainInstance&) = delete;

        /**
         * @brief 0 = Training, 1 = Produktion (Fixed Structure).
         */
        void setMode(int mode) { API_SetMode(_brainHandle, mode); }

        /**
         * @brief Liefert die optimale Aktion basierend auf der aktuellen Wahrnehmung.
         */
        TokenID update(const std::vector<TokenID>& inputs) {
            return API_Update(_brainHandle, inputs.data(), static_cast<int>(inputs.size()));
        }

        /**
         * @brief Simuliert zukünftige Kausalitäten (Imagination).
         */
        TokenID simulate(const std::vector<TokenID>& inputs, int depth) {
            return API_Simulate(_brainHandle, inputs.data(), static_cast<int>(inputs.size()), depth);
        }

        /**
         * @brief Passt Verhaltensgewichte via Reinforcement Learning an.
         */
        void feedback(float reward, TokenID action) {
            API_Feedback(_brainHandle, reward, action);
        }

        /**
         * @brief Injiziert eine harte Regel (Reflex) direkt in das LTM.
         */
        void teach(TokenID input, TokenID action, float weight) {
            API_Teach(_brainHandle, input, action, weight);
        }

        /**
         * @brief Liest ein gelerntes Gewicht unter Anwendung des De-Salting aus.
         */
        float inspect(TokenID input, TokenID action) {
            return API_Inspect(_brainHandle, input, action);
        }

        /**
         * @brief Erzeugt einen Snapshot des Gehirns.
         */
        std::vector<uint8_t> serialize() {
            int size = 0;
            void* buffer = API_Serialize(_brainHandle, &size);
            if (!buffer) return {};

            std::vector<uint8_t> data(static_cast<uint8_t*>(buffer), static_cast<uint8_t*>(buffer) + size);
            API_FreeBuffer(buffer); // Puffer im C-Kern freigeben
            return data;
        }
    };
}