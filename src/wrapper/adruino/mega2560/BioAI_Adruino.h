#ifndef ARDUINO_BIO_H
#define ARDUINO_BIO_H

#include <stdint.h>
#include <stddef.h>

// --- C-INTERFACE (Direkter Link zur hochperformanten .a Library) ---
extern "C" {
    void* API_CreateBrain(uint64_t key);
    void API_FreeBrain(void* ptr);
    
    // Kern-Logik für O(1) Performance
    uint64_t API_Update(void* ptr, uint64_t* inputs, int count);
    uint64_t API_Simulate(void* ptr, uint64_t* inputs, int count, int depth);
    void API_Feedback(void* ptr, float reward, uint64_t action);
    void API_Teach(void* ptr, uint64_t input, uint64_t action, float weight);
    
    // Management & Sicherheit
    void API_SetMode(void* ptr, int mode);
    float API_Inspect(void* ptr, uint64_t input, uint64_t action);
    
    // Sequencer (Planer für strategische Abläufe)
    void API_LoadPlan(void* ptr, uint64_t* steps, int count, int strict);
    void API_AbortPlan(void* ptr);
    int API_GetPlanStatus(void* ptr);

    // Speicher-Management (Serialisierung)
    void* API_Serialize(void* ptr, int* outSize);
    void* API_Deserialize(void* data, int size);
    void API_FreeBuffer(void* buffer);
}

namespace BioAI {

    // Definition der Betriebsmodi
    enum class BioMode : int {
        Training = 0,
        Production = 1
    };

    // Cluster-Definitionen für die Kausal-Logik
    enum class Cluster : uint64_t {
        Object = 0x10,
        Action = 0x20,
        Time   = 0x30,
        Logic  = 0x40,
        Self   = 0x50
    };

    // Sub-Cluster für direkte Reflexe im Logic-Bereich
    static const uint64_t CLUSTER_REFLEX = (uint64_t)Cluster::Logic | 0x0010000000000000ULL;

    class Agent {
        void* brain;
        uint64_t id;

    public:
        // Initialisierung des Cores
        Agent(uint64_t seed_id) : id(seed_id) {
            brain = API_CreateBrain(seed_id);
        }

        ~Agent() {
            if (brain) API_FreeBrain(brain);
        }

        // --- CORE FEATURES ---

        // Klassisches O(1) Denken
        uint64_t Think(uint64_t* inputs, int count) {
            return API_Update(brain, inputs, count);
        }

        // Vorhersage/Simulation (Der strategische Vorteil)
        uint64_t Simulate(uint64_t* inputs, int count, int depth) {
            return API_Simulate(brain, inputs, count, depth);
        }

        // Rückkoppelung für Lernprozesse
        void Learn(float reward, uint64_t action) {
            API_Feedback(brain, reward, action);
        }

        // REFLEX-IMPFUNG: Direktes Ziel-Vorgabe System
        // Ermöglicht Energie-Effizienz oder Speed-Profile ohne Overhead
        void ForceInstinct(uint64_t input, uint64_t action, float weight) {
            API_Teach(brain, input, action, weight);
        }

        // Sicherheit: Umschalten zwischen Training und stabilem Betrieb
        void SetMode(BioMode mode) {
            API_SetMode(brain, (int)mode);
        }

        // --- SEQUENCER (Für komplexe Maschinen-Phasen) ---

        void LoadPlan(uint64_t* steps, int count, bool strict) {
            API_LoadPlan(brain, steps, count, strict ? 1 : 0);
        }

        void AbortPlan() {
            API_AbortPlan(brain);
        }

        int GetPlanStep() {
            return API_GetPlanStatus(brain);
        }

        // --- HILFSFUNKTIONEN ---

        // FNV-1a Hash zur schnellen Token-Erzeugung (O(1))
        static uint64_t CreateToken(const char* name, Cluster c) {
            uint64_t hash = 14695981039346656037ULL;
            while (*name) {
                hash ^= (uint8_t)*name++;
                hash *= 1099511628211ULL;
            }
            return ((uint64_t)c << 56) | (hash & 0x00FFFFFFFFFFFFFFULL);
        }
    };
}

#endif