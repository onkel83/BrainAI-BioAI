/**
 * @file main.cpp
 * @brief BioAI Industrial Bridge: Siemens S7 / OPC UA <=> SAP S/4HANA
 * @version 0.7.6
 */

#include <iostream>
#include <vector>
#include <thread>
#include <chrono>

 // BioAI C++ Wrapper (RAII) und native Schnittstellen
#include "BioAI.hpp" 

using namespace BioAI;

// --- SIMULIERTE INDUSTRIE-SCHNITTSTELLEN ---

/**
 * @brief Simuliert das Lesen aus einem Siemens S7 Datenbaustein (DB).
 */
bool PLC_Read_Sensor(int db, int offset) {
    // In der Realität: Snap7 oder OPC UA SDK nutzen
    return true;
}

/**
 * @brief Simuliert das Schreiben in die SPS (Aktoren/Befehle).
 */
void PLC_Write_Command(int db, int offset, int value) {
    std::cout << "[PLC] Schreibe Wert " << value << " in DB" << db << ".DBW" << offset << std::endl;
}

/**
 * @brief Simuliert eine REST-Meldung an SAP S/4HANA.
 */
void SAP_Notify_Event(uint64_t actionToken, float confidence) {
    std::cout << "[SAP] HANA OData Sync: Aktion 0x" << std::hex << actionToken
        << " mit Konfidenz " << std::dec << confidence << std::endl;
}

// --- HAUPTPROGRAMM ---

int main() {
    std::cout << "--- BioAI Industrial Bridge v0.7.6 startet ---" << std::endl;

    const std::string keyPath = "config/key.json";
    std::unique_ptr<BioBrainInstance> brain;

    try {
        // 1. Initialisierung des Kerns mit Lizenzschlüssel
        brain = std::make_unique<BioBrainInstance>(keyPath);

        // 2. Produktionsmodus aktivieren (Deterministisch & Echtzeit-Sicher)
        // Verhindert Heap-Allokationen während des Steuerzyklus.
        brain->setMode(1);

        std::cout << "[CORE] BioAI Modus: " << BIOAI_MODE_NAME << " (Fixed Structure)" << std::endl;

    }
    catch (const std::exception& e) {
        std::cerr << "CRITICAL ERROR: " << e.what() << std::endl;
        return 1;
    }

    // --- ECHTZEIT-STEUERZYKLUS ---
    bool running = true;
    while (running) {
        std::vector<TokenID> currentInputs;

        // 3. Wahrnehmung (Mapping S7 -> BioAI OBJECT Cluster)
        if (PLC_Read_Sensor(10, 0)) {
            // Beispiel: Sensor 1 registriert Aktivität
            currentInputs.push_back(CLUSTER_OBJECT | 0x0001);
        }

        // 4. Inferenz: BioAI entscheidet basierend auf gelernten Kausalitäten
        TokenID decision = brain->update(currentInputs);

        // 5. Dispatching: Entscheidung an Hardware oder SAP verteilen
        if (decision != 0) {

            // Prüfung auf Reflex-Priorität (z.B. Safety Stop)
            if (IS_REFLEX(decision)) {
                std::cout << "[ALARM] Sicherheits-Reflex ausgeloest!" << std::endl;
                PLC_Write_Command(10, 2, 999); // Not-Aus an SPS
            }
            // Reguläre Prozess-Aktion
            else if ((decision & 0xFF00000000000000ULL) == CLUSTER_ACTION) {
                PLC_Write_Command(10, 4, static_cast<int>(decision & 0xFFFF));

                // 6. Transparenz-Audit & SAP Synchronisation
                float conf = brain->inspect(currentInputs[0], decision);
                if (conf > 0.75f) {
                    SAP_Notify_Event(decision, conf);
                }
            }
        }

        // Zykluszeit einhalten (z.B. 100ms für industrielle Stabilität)
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    return 0;
}