#pragma once

#include <vector>
#include <string>
#include <fstream>
#include <iostream>
#include <cstring>
#include <stdexcept>
#include <cstdint>

// --- C-INTERFACE IMPORT (Muss exakt zum Core passen!) ---
extern "C" {
    // Lifecycle
    void* API_CreateBrain(uint64_t key);
    void API_FreeBrain(void* ptr);
    
    // Core Logic (const correctness applied)
    uint64_t API_Update(void* ptr, const uint64_t* inputs, int count);
    uint64_t API_Simulate(void* ptr, const uint64_t* inputs, int count, int depth);
    void API_Feedback(void* ptr, float reward, uint64_t action);
    void API_Teach(void* ptr, uint64_t input, uint64_t action, float weight);
    
    // Management
    void API_SetMode(void* ptr, int mode);
    float API_Inspect(void* ptr, uint64_t input, uint64_t action);
    
    // Sequencer / Planer
    void API_LoadPlan(void* ptr, const uint64_t* steps, int count, int strict);
    void API_AbortPlan(void* ptr);
    int API_GetPlanStatus(void* ptr);

    // Serialisierung
    // Serialize nimmt const ptr, da es das Gehirn nicht ändert
    void* API_Serialize(const void* ptr, int* outSize);
    // Deserialize nimmt const data, da es den Buffer nur liest
    void* API_Deserialize(const void* data, int size);
    
    void API_FreeBuffer(void* buffer);
}

namespace BioAI {

    enum class BioMode : int {
        Training = 0,
        Production = 1
    };

    // Cluster Definitionen (Müssen mit BioAI_Types.h übereinstimmen)
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

        // Kopieren verbieten (Brain Pointer Ownership ist unique)
        Agent(const Agent&) = delete;
        Agent& operator=(const Agent&) = delete;

    public:
        // RAII: Konstruktor holt Ressource, Destruktor gibt sie frei
        explicit Agent(uint64_t seed_id) : id(seed_id) {
            brain = API_CreateBrain(seed_id);
            if (!brain) throw std::runtime_error("BioAI Core Init Failed (OOM or DLL missing)");
        }

        ~Agent() {
            if (brain) API_FreeBrain(brain);
            brain = nullptr;
        }

        // Move-Semantik erlauben (optional, aber nützlich für std::vector<Agent>)
        Agent(Agent&& other) noexcept : brain(other.brain), id(other.id) {
            other.brain = nullptr;
        }

        Agent& operator=(Agent&& other) noexcept {
            if (this != &other) {
                if (brain) API_FreeBrain(brain);
                brain = other.brain;
                id = other.id;
                other.brain = nullptr;
            }
            return *this;
        }

        // --- CORE FEATURES ---

        // Denken (O(1))
        uint64_t Think(const std::vector<uint64_t>& inputs) {
            if (inputs.empty()) return 0;
            return API_Update(brain, inputs.data(), (int)inputs.size());
        }

        // Simulieren (Prediction)
        uint64_t Simulate(const std::vector<uint64_t>& inputs, int depth) {
            if (inputs.empty()) return 0;
            return API_Simulate(brain, inputs.data(), (int)inputs.size(), depth);
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
            if (steps.empty()) {
                API_AbortPlan(brain);
                return;
            }
            API_LoadPlan(brain, steps.data(), (int)steps.size(), strict ? 1 : 0);
        }

        void AbortPlan() {
            API_AbortPlan(brain);
        }

        int GetPlanStep() {
            return API_GetPlanStatus(brain);
        }

        // --- SERIALISIERUNG (Save/Load) ---

        std::vector<uint8_t> Serialize() const {
            int size = 0;
            // API_Serialize erwartet const void* im ersten Parameter (nach Update)
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
            
            // API_Deserialize erwartet const void* (nach Update)
            void* newBrain = API_Deserialize(data.data(), (int)data.size());
            
            if (newBrain) {
                if (brain) API_FreeBrain(brain); // Altes löschen
                brain = newBrain;
            } else {
                throw std::runtime_error("Deserialization failed (Corrupt Data or Version Mismatch)");
            }
        }

        // File Helper
        void SaveToFile(const std::string& filename) const {
            auto data = Serialize();
            std::ofstream fout(filename, std::ios::out | std::ios::binary);
            if (!fout) throw std::runtime_error("Cannot write to file");
            fout.write(reinterpret_cast<const char*>(data.data()), data.size());
        }

        void LoadFromFile(const std::string& filename) {
            std::ifstream fin(filename, std::ios::in | std::ios::binary);
            if (fin.is_open()) {
                std::vector<uint8_t> data((std::istreambuf_iterator<char>(fin)), std::istreambuf_iterator<char>());
                Deserialize(data);
            } else {
                throw std::runtime_error("Cannot open file for reading");
            }
        }

        // --- STATIC HELPER ---

        // CreateToken (FNV-1a Hash, deterministic across platforms)
        static uint64_t CreateToken(const std::string& name, Cluster c) {
            uint64_t hash = 14695981039346656037ULL;
            for (char ch : name) {
                hash ^= (uint8_t)ch;
                hash *= 1099511628211ULL;
            }
            // Cluster Maskierung (High Byte setzen, Rest vom Hash behalten)
            return ((uint64_t)c << 56) | (hash & 0x00FFFFFFFFFFFFFFULL);
        }
    };
}
