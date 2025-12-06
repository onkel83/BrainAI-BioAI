# BioAI Architecture Deep Dive

**Version:** 0.0.2 (Alpha)
**Status:** Industrial Gold Standard (C99)

---

## The Philosophy: Efficiency First

Während moderne KI-Forschung (Deep Learning) auf massiven Matrizen-Operationen basiert, kehrt **BioAI** zu den Wurzeln der Kybernetik zurück.  
Wir betrachten Intelligenz als **Signalverarbeitung**, nicht als Statistik.

---

## 1. The "Native Core" Concept

Der Kern von BioAI ist eine proprietäre Engine, geschrieben in **ANSI C (C99)**.

- **Keine Garbage Collection:** Speicher wird manuell und deterministisch verwaltet.
- **Keine Dependencies:** Der Core benötigt keine externen Bibliotheken (kein Python, kein Torch, kein NumPy).
- **Sparse Memory:** Das System belegt nur Speicher für Konzepte, die es tatsächlich kennt.
  - Ein *leeres Gehirn* belegt nur wenige Bytes (Header).
- **Industrial Safety Mode (New):** Über das Flag `fixed_structure` kann die Speicherverwaltung zur Laufzeit komplett deaktiviert werden (`malloc`-Ban). Dies garantiert 100% Schutz vor Speicherfragmentierung im Dauerbetrieb.

---

## 2. The Symbolic Processing Unit (Temporal Cortex)

Anstatt Inputs durch tiefe Schichten zu leiten, nutzt BioAI ein hochperformantes **Signal-Mapping-Verfahren**, um sensorische Daten (Inputs) und zeitliche Abfolgen in eindeutige **64-Bit Tokens** zu verwandeln.

- **Vorteil:** Verarbeitungskomplexität ist **O(1)**.
- **Technik:** Eine interne Hashtable mit linearer Sondierung ermöglicht sofortigen Zugriff auf jedes Neuron.
- **Egal ob 10 oder 10.000 Konzepte:** Die Entscheidungsfindung dauert immer gleich lang (< 1 ms auf modernen MCUs).

---

## 3. The Associative Brain

Das "Gedächtnis" ist als **gerichteter Graph** organisiert.

- **LTM (Long Term Memory):** Speichert validierte Strategien (Synapsen mit hohem Gewicht). Diese sind persistent.
- **STM (Short Term Memory):** Speichert temporäre Hypothesen.
- **Trace (Hippocampus):** Ein Ringbuffer speichert die letzten Aktionen, um verzögerte Belohnungen (Delayed Rewards) korrekt zuzuordnen.

**Lernverfahren:** BioAI nutzt eine modifizierte Form des **Hebbian Learning** (*"Cells that fire together, wire together"*).
Wenn ein Reward (`API_Feedback`) eintrifft, verstärkt das System rückwirkend die Pfade im Trace. Nur wenn eine Verbindung im STM oft genug bestätigt wird (`LTM_CONSOLIDATE_HITS`), wird sie permanent.

---

## 4. The Cluster Ontology

Um *Symbol Grounding* (das Verstehen von Bedeutung) zu ermöglichen, erzwingt BioAI eine strikte Typisierung von Signalen durch **Cluster-IDs** (High-Byte des 64-Bit Tokens).

Dies deckt sich 1:1 mit den Definitionen in `BioAI_Types.h`:

| Cluster (Hex) | Name | Beschreibung | Beispiele |
| :--- | :--- | :--- | :--- |
| **0x10...** | **OBJECT** | Physische Dinge & Orte | Wand, Apfel, Küche, Temperatur |
| **0x20...** | **ACTION** | Motorische Befehle | Motor an, Mail senden, Bremsen |
| **0x30...** | **TIME** | Zeitliche Konzepte | Später, Jetzt, Tag, Nacht |
| **0x40...** | **LOGIC** | Regeln & Reflexe | Wenn/Dann, Not-Aus (Reflex) |
| **0x50...** | **SELF** | Innere Zustände | Hunger (Need), Auftrag (Goal), Status |

**Vorteil:** Das System kann generalisierte Regeln lernen (z. B. *"Priorisiere Cluster SELF (Needs) über Cluster ACTION"*), ohne jedes einzelne Bedürfnis neu lernen zu müssen.

---

## 5. The Predictive Engine (Imagination)

In Version 0.0.2 wurde eine **Kausalitäts-Ebene** eingeführt.

- **Funktion:** Jedes Neuron speichert nicht nur, was es *auslöst* (Synapse), sondern auch, was *danach passiert* (Prediction).
- **Simulation:** Durch die Methode `API_Simulate(depth)` kann der Agent diese Kette mental durchlaufen, bevor er handelt.
- **Code-Basis:** Rekursive Tiefensuche mit `MAX_SIM_DEPTH` Bremse (Stack-Schutz).

---

**BrainAI** - *Intelligence everywhere.*
Developed by **Sascha A. Köhne (winemp83)**
Product: **BioAI v0.0.2 (Alpha)**
