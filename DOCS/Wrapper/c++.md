# BioAI C++ Header-Only Integration 🚀

**Version:** 0.7.5
**Type:** Modern C++11 Wrapper (Header-Only)
**License:** MIT (Wrapper) / Proprietary (Core)

---

## 1. Übersicht

Der C++ Wrapper (`BioAI.hpp`) bietet eine **Zero-Overhead** Abstraktion um die C-Core-Bibliothek. Er nutzt moderne C++ Features (RAII, Exceptions, `std::vector`), um die manuelle Speicherverwaltung des C-Interfaces sicher und komfortabel zu kapseln.

### Features
* **RAII Compliance:** Automatische Freigabe von Gehirnen und Buffern (kein manuelles `free` nötig).
* **Type Safety:** `const`-Correctness für alle lesenden Operationen.
* **STL Integration:** Direkte Nutzung von `std::vector` und `std::string`.
* **No Dependencies:** Benötigt nur die Standard-Library.

---

## 2. Installation

Da der Wrapper "Header-Only" ist, ist keine Kompilierung nötig.

1.  Kopieren Sie `BioAI.hpp` in Ihren `include`-Ordner.
2.  Kopieren Sie die passende Shared Library (`BioAI.dll` / `libbioai.so`) in das Ausführungsverzeichnis.
3.  Inkludieren Sie den Header:

```cpp
#include "BioAI.hpp"
````

-----

## 3\. Quick Start: Hello Brain

Ein minimales Beispiel, das einen Agenten erstellt, ihm einen Instinkt beibringt und ihn ausführt.

```cpp
#include "BioAI.hpp"
#include <iostream>

using namespace BioAI;

int main() {
    try {
        // 1. Agent initialisieren (Seed: 0xBADC0FFEE)
        Agent myAI(0xBADC0FFEE);

        // 2. Tokens definieren (Namen -> Hashes)
        uint64_t sensor_fire = Agent::CreateToken("FIRE", Cluster::Object);
        uint64_t action_run  = Agent::CreateToken("RUN",  Cluster::Action);

        // 3. Instinkt injizieren (Safety Rule)
        // "Wenn FEUER, dann RENNEN" (Gewicht 1.0 = Zwingend)
        myAI.ForceInstinct(sensor_fire, action_run, 1.0f);

        // 4. Denken (Input verarbeiten)
        std::vector<uint64_t> perception = { sensor_fire };
        uint64_t decision = myAI.Think(perception);

        // 5. Auswerten
        if (decision == action_run) {
            std::cout << "AI decided to RUN! (Correct)" << std::endl;
        } else {
            std::cout << "AI did something else." << std::endl;
        }

    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return -1;
    }
    return 0;
}
```

-----

## 4\. Advanced: Safety & Serialization

### Production Mode (Der Sicherheits-Schalter)

Für sicherheitskritische Anwendungen (ISO 26262) kann das Lernen deaktiviert werden.

```cpp
// Schaltet malloc() und Lern-Logik ab. 
// Das System ist jetzt 100% deterministisch.
myAI.SetMode(BioMode::Production);
```

### Cloning (Speichern & Laden)

Übertragen Sie den Zustand eines Agenten auf einen anderen (z.B. Fleet Learning).

```cpp
// Speichern
myAI.SaveToFile("brain_dump.bin");

// Laden
Agent anotherAI(0);
anotherAI.LoadFromFile("brain_dump.bin");
```

-----

## 5\. API Referenz

| Methode | Beschreibung | Komplexität |
| :--- | :--- | :--- |
| `Think(inputs)` | Verarbeitet Signale und wählt eine Aktion. | **O(1)** |
| `Simulate(inputs, depth)` | Simuliert die Zukunft ("Träumen"). | **O(Depth)** |
| `Learn(reward, action)` | Feedback-Learning (Hebbian). | **O(1)** |
| `ForceInstinct(...)` | Schreibt harte Regeln ins Langzeitgedächtnis. | **O(1)** |
| `Inspect(...)` | Liest das Gewicht einer Synapse (Debug). | **O(1)** |

-----

**BrainAI** - *Intelligence everywhere.*
Developed by **Sascha A. Köhne (winemp83)**
Product: **BioAI 0.7.5 (Industrial Closed Feature)**
📧 [koehne83@googlemail.com](mailto:koehne83@googlemail.com)

© 2025 BrainAI / Sascha A. Köhne. All rights reserved.