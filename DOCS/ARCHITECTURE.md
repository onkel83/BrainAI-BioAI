# BioAI Architecture Deep Dive 🧠

**Version:** 0.5.1 (Industrial Beta)
**Status:** Industrial Gold Standard (C99)

---

## The Philosophy: Efficiency First

Während moderne KI-Forschung (Deep Learning) auf massiven Matrizen-Operationen basiert, kehrt **BioAI** zu den Wurzeln der Kybernetik zurück.
Wir betrachten Intelligenz als **Signalverarbeitung**, nicht als Statistik.

---

## 1. The "Native Core" Concept

Der Kern von BioAI ist eine proprietäre Engine, geschrieben in **ANSI C (C99)**.

* **Keine Garbage Collection:** Speicher wird manuell und deterministisch verwaltet.
* **Keine Dependencies:** Der Core benötigt keine externen Bibliotheken (kein Python, kein Torch, kein NumPy).
* **Sparse Memory:** Das System belegt nur Speicher für Konzepte, die es tatsächlich kennt. Ein *leeres Gehirn* belegt nur wenige Bytes (Header).
* **Industrial Safety Mode:** Über das Flag `fixed_structure` (Production Mode) kann die Speicherverwaltung zur Laufzeit komplett deaktiviert werden (`malloc`-Ban). Dies garantiert 100% Schutz vor Speicherfragmentierung im Dauerbetrieb (24/7).

---

## 2. Scalable Precision (The 3 Tiers)

BioAI löst das Problem der Hardware-Fragmentierung durch eine **adaptive Typisierung**. Der gleiche Algorithmus läuft auf einem 8-Bit Mikrocontroller und einem 64-Bit Server, indem die Datentypen (`Index`) zur Kompilierzeit angepasst werden.

| Tier | Index-Typ | Max. Neuronen | Ziel-Hardware | Speicherbedarf |
| :--- | :--- | :--- | :--- | :--- |
| **IoT** | `uint8_t` | **255** | Arduino, AVR, ESP8266 | **< 2 KB** |
| **SmartHome** | `uint16_t` | **65.535** | ESP32, STM32, Raspberry Pi | **~ 64 KB** |
| **Ultra** | `uint32_t` | **4.2 Mrd.** | PC, Server, Cloud | RAM limitiert |

Dies ermöglicht es, Logik auf dem PC (Ultra) zu trainieren und nahtlos auf den Mikrocontroller (IoT) zu übertragen ("Brain Porting").

---

## 3. The Symbolic Processing Unit (Temporal Cortex)

Anstatt Inputs durch tiefe Schichten zu leiten, nutzt BioAI ein hochperformantes **Signal-Mapping-Verfahren**.

* **Vorteil:** Verarbeitungskomplexität ist **deterministisch O(1)** (Worst-Case Execution Time ist konstant).
* **Der Beweis:**
    1.  **Neuron-Zugriff:** Erfolgt über eine optimierte Funktion-> **O(1)**.
    2.  **Verarbeitung:** Jedes Neuron hat ein striktes Limit an Verbindungen (`MAX_SYNAPSES`, z.B. 16 bei IoT, 256 bei Ultra).
    3.  **Resultat:** Die Berechnungszeit hängt *nicht* von der Gesamtgröße des Gehirns ab.
* **Egal ob 10 oder 10 Millionen Konzepte:** Die Entscheidungsfindung für einen einzelnen Reiz dauert immer gleich lang. Das ist essenziell für harte Echtzeitsysteme.

---

## 4. The Associative Brain

Das "Gedächtnis" ist als Graph-Artig organisiert.

* **LTM (Long Term Memory):** Speichert validierte Strategien (Synapsen mit hohem Gewicht). Diese sind persistent.
* **STM (Short Term Memory):** Speichert temporäre Hypothesen.
* **Trace (Hippocampus):** Ein Ringbuffer speichert die letzten Aktionen, um verzögerte Belohnungen (Delayed Rewards) korrekt zuzuordnen.

**Lernverfahren:** BioAI nutzt eine Spezial Form des **Hebbian Learning** (*"Cells that fire together, wire together"*).
Wenn ein Reward (`API_Feedback`) eintrifft, verstärkt das System rückwirkend die Pfade im Trace. Nur wenn eine Verbindung im STM oft genug bestätigt wird (`LTM_CONSOLIDATE_HITS`), wird sie permanent ins LTM übernommen.

---

## 5. The Cluster Ontology

Um *Symbol Grounding* (das Verstehen von Bedeutung) zu ermöglichen, erzwingt BioAI eine strikte Typisierung von Signalen durch **Cluster-IDs** (High-Byte des 64-Bit Tokens).

| Cluster (Hex) | Name | Beschreibung | Beispiele |
| :--- | :--- | :--- | :--- |
| **0x10...** | **OBJECT** | Physische Dinge & Orte | Wand, Apfel, Küche, Temperatur |
| **0x20...** | **ACTION** | Motorische Befehle | Motor an, Mail senden, Bremsen |
| **0x30...** | **TIME** | Zeitliche Konzepte | Später, Jetzt, Tag, Nacht |
| **0x40...** | **LOGIC** | Regeln & Reflexe | Wenn/Dann, Not-Aus (Reflex) |
| **0x50...** | **SELF** | Innere Zustände | Hunger (Need), Auftrag (Goal), Status |

**Vorteil:** Das System kann **Hard-Safety-Regeln** (Cluster LOGIC/Reflex) implementieren, die alle anderen Entscheidungen überschreiben.

---

## 6. The Predictive Engine (Imagination)

BioAI verfügt über eine integrierte **Kausalitäts-Ebene**.

* **Funktion:** Jedes Neuron speichert nicht nur, was es *auslöst* (Synapse), sondern auch, was *danach passiert* (Prediction).
* **Simulation:** Durch die Methode `API_Simulate(depth)` kann der Agent diese Kette mental durchlaufen, bevor er handelt.
* **Sicherheit:** Die Rekursionstiefe ist durch `MAX_SIM_DEPTH` begrenzt, um Stack-Overflows auf kleinen Geräten physikalisch auszuschließen.

---

**BrainAI** - *Intelligence everywhere.*
Developed by **Sascha A. Köhne (winemp83)**
Product: **BioAI v0.5.1 (Industrial Beta)**
