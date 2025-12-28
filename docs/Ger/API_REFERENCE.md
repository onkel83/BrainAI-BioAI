# BioAI.Core API Reference 🧠

**Version:** 0.7.6 (Industrial Closed Feature)
**Architecture:** Neuro-Symbolic / Sparse Associative Memory

---

## 1. Core Concepts

Die Architektur von BioAI basiert auf wenigen, abstrakten Primitiven, die eine universelle Kompatibilität garantieren.

- **TokenID (uint64):** Ein einzigartiger 64-Bit Hash, der jedes Konzept, Objekt oder jede Aktion im System repräsentiert.
- **Cluster:** Das höchste Byte einer TokenID definiert ihren Typ (z. B. `0x10` für Objekte, `0x20` für Aktionen, `0x50` für das Selbst).
- **Brain:** Die spärlich assoziative Speicher-Engine, die Verbindungen zwischen Inputs und Outputs verwaltet.
- **Tiers:** Die Engine ist in 3 Varianten verfügbar (**IoT**, **SmartHome**, **Ultra**), die sich nur in der Speicheradressierung (8/16/32-Bit) unterscheiden. Die API ist identisch.

---

## 2. 🛡️ Safety & Compliance Features

BioAI.Core wurde speziell unter Berücksichtigung von Anforderungen an sicherheitskritische Umgebungen (z. B. IEC 61508) und Auditierbarkeit entwickelt.

### A. Der "Run/Train Switch" (Inference Mode)

- **Training Mode (Mode 0):** Das System adaptiert seine Gewichte basierend auf Feedback. Neue Verbindungen werden geknüpft.
- **Production Mode (Mode 1):** Das Lernen wird **strukturell eingefroren**. `malloc` und `free` sind deaktiviert. Das System verhält sich ab diesem Punkt zu 100% deterministisch und speichersicher (keine Fragmentierung).

**Use Case:** Ein Roboter lernt im Labor. Sobald die Performance optimal ist, wird das Gehirn auf `Production` geschaltet und auf die Flotte ausgerollt.

### B. Universelle Logik (Emotionslose Entscheidungen)

- Keine versteckte Moral oder bias-behaftete Trainingsdaten. **Der Entwickler definiert die anfängliche Zielrichtung und Bias über explizit injizierte Instinkte und die Reward-Funktion.**
- Entscheidungen basieren ausschließlich auf den vom Entwickler definierten Clustern (`NEEDS`, `GOALS`) und Rewards.
- **Zero Hallucination Policy:** BioAI kann nur Aktionen wählen, die vorher explizit als Token definiert wurden.

---

## 3. The Transparency Layer (Glass Box Interface)

Da der BioAI-Kern als hochoptimierte Blackbox (Binary) ausgeliefert wird, stellen die Wrapper-Klassen die notwendige Transparenz sicher.

### Vocabulary Export (Decision Explainability)
- Das System führt intern ein "Telefonbuch" (Registry), das Hashes (`0xA4F...`) wieder in lesbare Namen (`Motor_Start`) zurückübersetzt.
- **Funktion:** `DumpVocabulary()` erzeugt eine Datei für Ingenieure.
- **Zweck:** Ermöglicht Post-Mortem-Analysen bei Fehlfunktionen durch Abgleich mit Logs.

### Inspection API
- Mit `Inspect()` kann der Status jeder einzelnen Synapse zur Laufzeit abgefragt werden.
- **Zweck:** Echtzeit-Monitoring von Lernfortschritten.

---

## 4. Universal API Methods

Unabhängig von der verwendeten Programmiersprache (Wrapper) stellt der Core diese Methoden bereit:

### A. Kognition & Lernen

#### `Think(inputs)` / `API_Update`
- Verarbeitet die aktuelle Wahrnehmung und liefert die optimale Aktion.
- **Komplexität:** **Deterministisch O(1)**. (Garantiert durch Hard-Caps für Synapsen pro Neuron).
- **Return:** TokenID der gewählten Aktion.

#### `Simulate(inputs, depth)`
- Führt eine Kausalitäts-Simulation durch ("Imagination").
- Prüft: *"Wenn ich jetzt X tue, was passiert in `depth` Schritten?"*
- **Use Case:** Vorausplanendes Handeln und Kollisionsvermeidung.

#### `Learn(reward, action)` / `API_Feedback`
- Wendet Reinforcement Learning auf das Kurzzeitgedächtnis (Trace) an.
- **Parameter:**
  - `reward`: Float (-10.0 bis +10.0). Positiv = Belohnung, Negativ = Bestrafung.
  - `action`: TokenID der Aktion, die bewertet wird.

#### `ForceInstinct(input, action, weight)` / `API_Teach`
- Injiziert eine harte Regel (Reflex) direkt in das Langzeitgedächtnis.
- **Gewicht 1.0:** Entspricht einem unverhandelbaren Gesetz (Hard Safety).
- **Use Case:** Sicherheitsprotokolle ("Wenn Feuer, dann Flucht").

### B. Sequencer (Ablaufsteuerung)

BioAI kann auch klassische Schrittketten (SPS-Modus) abarbeiten.

#### `LoadPlan(steps, strict)`
- Lädt eine feste Liste von Aktionen (z.B. `[Move_X, Drill, Move_Y]`).
- **Strict Mode (true):** BioAI arbeitet stur ab. Bricht nur bei Reflexen ab.
- **Adaptive Mode (false):** BioAI versucht den Plan zu erfüllen, darf aber bei Problemen kurzzeitig abweichen (Stabilisierung).

#### `AbortPlan()`
- Bricht den aktuellen Plan sofort ab und gibt die Kontrolle an die KI zurück.

### C. System & Management

#### `SetMode(mode)`
- Schaltet den Betriebsmodus um.
- `0`: Training (Dynamisch).
- `1`: Production (Statisch/Sicher).

#### `Serialize()` / `Deserialize()`
- **Klonen:** Speichert das gesamte Gehirn (Wissen, Pläne, Einstellungen) in ein Byte-Array oder lädt es.
- **Use Case:** "Master-Slave" Deployment in der Fabrik.

#### `CreateToken(string, cluster)`
- Generiert einen deterministischen Hash für ein Konzept.
- **Hinweis:** Garantiert, dass "Apfel" auf jedem Gerät (Arduino, PC, Cloud) dieselbe ID hat.

---

## 5. Language Specifics & Wrappers

Alle offiziellen Wrapper implementieren das Transparency Interface automatisch:

- **C++:** Header-only für Embedded Systems.
- **C# / .NET:** Für Unity, Godot & Windows.
- **Java:** Für Android & Enterprise.
- **Python:** Für Data Science.
- **Node.js:** Für Backend Services.
- **VB.NET:** Für industrielle HMI Panels.

---

## 📞 Contact

**BrainAI** - *-We don't need **BRUTEFORCE**, we know **Physiks**-*</br>
Developed by **Sascha A. Köhne (winemp83)**</br>
Product: **BioAI 0.7.6 (Industrial Closed Feature)**</br>
📧 [koehne83@googlemail.com](mailto:koehne83@googlemail.com)

&copy; 2025 BrainAI / Sascha A. Köhne. All rights reserved.
