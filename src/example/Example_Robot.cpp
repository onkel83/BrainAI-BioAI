/*
 * BioAI C++ Example: Robot Simulation
 * * BUILD INSTRUCTIONS / KOMPILIER-HINWEIS:
 * * Dieses Programm benötigt die BioAI Core Bibliothek (DLL/SO).
 * Sie müssen beim Kompilieren gegen die Bibliothek linken!
 * * Linux (GCC):
 * g++ main.cpp -std=c++11 -L. -lbioai -o robot
 * (Stellen Sie sicher, dass 'libbioai.so' im selben Ordner liegt oder installiert ist.)
 * * Windows (MSVC):
 * cl /EHsc main.cpp /link BioAI.lib
 * (Die 'BioAI.dll' muss zur Laufzeit im selben Ordner wie die .exe liegen.)
 */

#include <iostream>
#include <vector>
#include <string>
#include <iomanip> // Für std::setprecision

// Den Wrapper inkludieren. 
// WICHTIG: Die Shared Library (.dll/.so) muss zur Laufzeit im Ordner liegen.
#include "bioai.hpp" 

using namespace BioAI;

int main() {
    std::cout << "--- BioAI v0.5.1beta (C++ Wrapper) Robot Simulation ---\n" << std::endl;

    try {
        // 1. Agent initialisieren (RAII)
        // Der Konstruktor ruft intern API_CreateBrain auf.
        // Wenn wir den Scope verlassen, räumt der Destruktor (~Agent) automatisch auf.
        Agent robot(0xBADC0FFEE); // Hex-Seed für Reproduzierbarkeit

        // 2. Tokens definieren (Namen -> Hashes)
        // Der Wrapper nutzt FNV-1a Hashing. Wir müssen keine IDs mehr manuell verwalten.
        const uint64_t SENSOR_FREE = Agent::CreateToken("PATH_FREE", Cluster::Object);
        const uint64_t SENSOR_WALL = Agent::CreateToken("WALL_AHEAD", Cluster::Object);
        
        const uint64_t ACT_MOVE    = Agent::CreateToken("MOVE_FWD", Cluster::Action);
        const uint64_t ACT_TURN    = Agent::CreateToken("TURN_LEFT", Cluster::Action);

        // 3. Instinkte "Impfen" (Safety Layer)
        // Wir nutzen ForceInstinct, um die C-Funktion API_Teach zu kapseln.
        // Regel: Wenn WAND -> DREHEN (Gewicht 1.0 = Zwingend).
        robot.ForceInstinct(SENSOR_WALL, ACT_TURN, 1.0f);
        std::cout << "[Init] Instinkt aktiviert: WALL_AHEAD -> TURN_LEFT\n" << std::endl;

        // Simulations-Status
        int distance_to_wall = 5;

        // --- 4. Der Loop ---
        for (int step = 0; step < 15; ++step) {
            
            // A. Input Vektor vorbereiten (STL Vektoren statt C-Arrays)
            std::vector<uint64_t> inputs;
            
            if (distance_to_wall > 0) {
                inputs.push_back(SENSOR_FREE);
                std::cout << "[Step " << std::setw(2) << step << "] Sensor: PATH_FREE (Dist: " << distance_to_wall << ") -> ";
            } else {
                inputs.push_back(SENSOR_WALL);
                std::cout << "[Step " << std::setw(2) << step << "] Sensor: WALL_AHEAD!          -> ";
            }

            // B. Denken (Wrapper kapselt API_Update)
            uint64_t decision = robot.Think(inputs);

            // C. Physik & Belohnungslogik (Die Welt-Simulation)
            float reward = 0.0f;
            std::string action_name = "UNKNOWN";

            if (decision == ACT_MOVE) {
                action_name = "MOVE_FWD";
                if (distance_to_wall > 0) {
                    distance_to_wall--;
                    reward = 0.5f; // Positives Feedback für Fortschritt
                } else {
                    reward = -2.0f; // Crash (Sollte dank Instinkt nicht passieren)
                    std::cout << "CRASH! ";
                }
            } 
            else if (decision == ACT_TURN) {
                action_name = "TURN_LEFT";
                if (distance_to_wall == 0) {
                    distance_to_wall = 5; // Neue freie Bahn gefunden
                    reward = 1.0f; // Großes Lob für Rettung
                } else {
                    reward = -0.1f; // Unnötiges Drehen auf freier Fläche
                }
            } 
            else {
                // Falls die KI anfangs "zögert" (ID 0 oder andere Aktion)
                action_name = "IDLE"; 
                reward = -0.05f; 
            }

            std::cout << "Action: " << std::setw(10) << action_name 
                      << " | Reward: " << std::fixed << std::setprecision(2) << reward << std::endl;

            // D. Lernen (Wrapper kapselt API_Feedback)
            robot.Learn(reward, decision);

            // E. Debugging (Optional: Inspektion der Synapse)
            // Wir schauen, ob er gelernt hat, bei "Frei" zu laufen.
            if (distance_to_wall > 0 && step % 5 == 0) {
                float confidence = robot.Inspect(SENSOR_FREE, ACT_MOVE);
                std::cout << "   -> [Debug] Synapse (Free->Move) Strength: " << confidence << std::endl;
            }
        }

        // --- 5. Abschluss ---
        // Optional: Speichern des trainierten Gehirns für spätere Nutzung
        robot.SaveToFile("robot_brain.bio");
        std::cout << "\n[System] Brain saved to 'robot_brain.bio'." << std::endl;

    } catch (const std::exception& e) {
        // Der C++ Wrapper wirft std::runtime_error bei Fehlern (z.B. DLL fehlt)
        std::cerr << "CRITICAL ERROR: " << e.what() << std::endl;
        return -1;
    }

    // ~Agent() wird hier automatisch aufgerufen und gibt den Speicher im Core frei.
    return 0;
}
