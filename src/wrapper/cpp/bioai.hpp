#pragma once

#include <vector>
#include <string>
#include <fstream>
#include <ctime>
#include <iostream>
#include <iomanip>
#include <cstring>
#include <stdexcept>

// --- C-INTERFACE IMPORT (Muss exakt zum Core passen!) ---
extern "C" {
    void* API_CreateBrain(uint64_t key);
    void API_FreeBrain(void* ptr);
    
    // Core Logic
    uint64_t API_Update(void* ptr, uint64_t* inputs, int count);
    uint64_t API_Simulate(void* ptr, uint64_t* inputs, int count, int depth); // NEU
    void API_Feedback(void* ptr, float reward, uint64_t action);
    void API_Teach(void* ptr, uint64_t input, uint64_t action, float weight);
    
    // Management
    void API_SetMode(void* ptr, int mode); // NEU
    float API_Inspect(void* ptr, uint64_t input, uint64_t action);
    
    // Sequencer / Planer
    void API_LoadPlan(void* ptr, uint64_t* steps, int count, int strict); // NEU
    void API_AbortPlan(void* ptr); // NEU
    int API_GetPlanStatus(void* ptr); // NEU

    // Serialisierung
    void* API_Serialize(void* ptr, int* outSize);
    void* API_Deserialize(void* data, int size);
    void API_FreeBuffer(void* buffer);
}

namespace BioAI {

    enum class BioMode : int {
        Training = 0,
        Production = 1
    };

    enum class Cluster : uint64_t {
        Object = 0x10,
        Action = 0x20,
        Time   = 0x30,
        Logic  = 0x40,
        Self   = 0x50
    };

    // Sub-Clusters Helper
    static const uint64_t CLUSTER_REFLEX = (uint64_t)Cluster::Logic | 0x0010000000000000ULL;

    // --- AGENT CLASS ---
    class Agent {
        void* brain;
        uint64_t id;

    public:
        // RAII: Konstruktor holt Ressource, Destruktor gibt sie frei
        Agent(uint64_t seed_id) : id(seed_id) {
            brain = API_CreateBrain(seed_id);
            if (!brain) throw std::runtime_error("BioAI Core Init Failed (OOM)");
        }

        ~Agent() {
            if (brain) API_FreeBrain(brain);
        }

        // --- CORE FEATURES ---

        // Denken (O(1))
        uint64_t Think(const std::vector<uint64_t>& inputs) {
            // const_cast ist hier sicher, da der Core inputs nicht verändert (nur liest)
            return API_Update(brain, const_cast<uint64_t*>(inputs.data()), (int)inputs.size());
        }

        // Simulieren (Prediction)
        uint64_t Simulate(const std::vector<uint64_t>& inputs, int depth) {
            return API_Simulate(brain, const_cast<uint64_t*>(inputs.data()), (int)inputs.size(), depth);
        }

        // Lernen (Feedback)
        void Learn(float reward, uint64_t action) {
            API_Feedback(brain, reward, action);
        }

        // Instinkt (Hardcoded Rules)
        void ForceInstinct(uint64_t input, uint64_t action, float weight) {
            API_Teach(brain, input, action, weight);
        }

        // Modus setzen (Industrial Safety)
        void SetMode(BioMode mode) {
            API_SetMode(brain, (int)mode);
        }

        // Debugging / Inspection
        float Inspect(uint64_t input, uint64_t action) {
            return API_Inspect(brain, input, action);
        }

        // --- SEQUENCER (Planer) ---

        void LoadPlan(const std::vector<uint64_t>& steps, bool strict) {
            API_LoadPlan(brain, const_cast<uint64_t*>(steps.data()), (int)steps.size(), strict ? 1 : 0);
        }

        void AbortPlan() {
            API_AbortPlan(brain);
        }

        int GetPlanStep() {
            return API_GetPlanStatus(brain);
        }

        // --- SERIALISIERUNG (Save/Load) ---

        std::vector<uint8_t> Serialize() {
            int size = 0;
            void* ptr = API_Serialize(brain, &size);
            
            std::vector<uint8_t> data;
            if (ptr && size > 0) {
                data.resize(size);
                std::memcpy(data.data(), ptr, size);
                API_FreeBuffer(ptr); // WICHTIG: C-Buffer freigeben
            }
            return data;
        }

        void Deserialize(const std::vector<uint8_t>& data) {
            if (data.empty()) return;
            
            // Neues Brain laden
            void* newBrain = API_Deserialize(const_cast<void*>(static_cast<const void*>(data.data())), (int)data.size());
            
            if (newBrain) {
                if (brain) API_FreeBrain(brain); // Altes löschen
                brain = newBrain;
            } else {
                throw std::runtime_error("Deserialization failed (Corrupt Data?)");
            }
        }

        // File Helper (PC Only)
        void SaveToFile(const std::string& filename) {
            auto data = Serialize();
            std::ofstream fout(filename, std::ios::out | std::ios::binary);
            fout.write((char*)data.data(), data.size());
        }

        void LoadFromFile(const std::string& filename) {
            std::ifstream fin(filename, std::ios::in | std::ios::binary);
            if (fin.is_open()) {
                std::vector<uint8_t> data((std::istreambuf_iterator<char>(fin)), std::istreambuf_iterator<char>());
                Deserialize(data);
            }
        }

        // --- STATIC HELPER ---

        // CreateToken (Korrigiert auf FNV-1a Hash, passend zu C#/Core)
        static uint64_t CreateToken(std::string name, Cluster c) {
            uint64_t hash = 14695981039346656037ULL;
            for (char ch : name) {
                hash ^= (uint8_t)ch;
                hash *= 1099511628211ULL;
            }
            // Cluster Maskierung
            return ((uint64_t)c << 56) | (hash & 0x00FFFFFFFFFFFFFFULL);
        }
    };
}
