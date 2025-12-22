# BioAI.Core API Reference 🧠

**Version:** 0.7.5 (Industrial Closed Feature)

**Architecture:** Neuro-Symbolic / Sparse Associative Memory (SAM)

---

## 1. Core Concepts

The BioAI architecture is built upon a few abstract primitives that guarantee universal compatibility across all supported hardware tiers.

* **TokenID (uint64):** A unique 64-bit hash representing every concept, object, or action within the system.
* **Cluster:** The highest byte of a `TokenID` defines its type (e.g., `0x10` for Objects, `0x20` for Actions, `0x50` for Self/States).
* **Brain:** The Sparse Associative Memory engine managing connections between inputs and outputs.
* **Tiers:** The engine is available in 3 variants (**IoT**, **SmartHome**, **Ultra**). They differ only in memory addressing (8/16/32-bit), while the API remains identical.

---

## 2. 🛡️ Safety & Compliance Features

BioAI.Core was specifically developed to meet the requirements of safety-critical environments (e.g., IEC 61508) and high-level auditability.

### A. The "Run/Train" Switch (Inference Mode)

* **Training Mode (Mode 0):** The system adapts weights based on feedback. New synaptic connections are formed.
* **Production Mode (Mode 1):** Structural learning is **physically frozen**. `malloc` and `free` are disabled. The system behaves 100% deterministically and is memory-safe (no fragmentation).

### B. Universal Logic (Emotionless Decisions)

* No hidden "moral alignment" or biased training data. **The developer defines the initial direction and bias through explicitly injected instincts and reward functions.**
* **Zero Hallucination Policy:** BioAI can only select actions that were previously and explicitly defined as tokens.

---

## 3. The Transparency Layer (Glass Box Interface)

Since the BioAI core is delivered as a highly optimized binary, the wrapper classes provide the necessary transparency for industrial auditing.

### Vocabulary Export (Decision Explainability)

* The system maintains an internal "registry" that translates hashes (e.g., `0xA4F...`) back into human-readable names (`Motor_Start`).
* **Function:** `DumpVocabulary()` generates a manifest for engineers.
* **Purpose:** Enables post-mortem analysis of malfunctions by matching hashes in the logs to actual concepts.

### Inspection API

* Use `Inspect()` to query the status and weight of every single synapse during runtime.
* **Purpose:** Real-time monitoring of learning progress and causal links.

---

## 4. Universal API Methods

Regardless of the programming language (Wrapper), the Core provides the following methods:

### A. Cognition & Learning

#### `Think(inputs)` / `API_Update`

* Processes the current perception and returns the optimal action.
* **Complexity:** **Deterministic **. Guaranteed by hard-caps on synapses per neuron.
* **Return:** `TokenID` of the chosen action.

#### `Simulate(inputs, depth)`

* Performs a causality simulation ("Imagination").
* Checks: *"If I do X now, what will happen in `depth` steps?"*

#### `Learn(reward, action)` / `API_Feedback`

* Applies Reinforcement Learning to the Short-Term Memory (Trace).
* **Parameters:**
* `reward`: Float (-10.0 to +10.0). Positive = Reward, Negative = Punishment.
* `action`: `TokenID` of the action being evaluated.



#### `ForceInstinct(input, action, weight)` / `API_Teach`

* Injects a hard rule (Reflex) directly into the Long-Term Memory.
* **Weight 1.0:** Represents a non-negotiable law (Hard Safety).

### B. Sequencer (Process Control)

BioAI can also handle classic step-chains (PLC Mode).

#### `LoadPlan(steps, strict)`

* Loads a fixed list of actions (e.g., `[Move_X, Drill, Move_Y]`).
* **Strict Mode (true):** BioAI executes the plan rigidly. It only aborts for high-priority reflexes.
* **Adaptive Mode (false):** BioAI attempts the plan but is allowed to deviate temporarily for stabilization.

---

## 5. Language Specifics & Wrappers

All official wrappers automatically implement the Transparency Interface:

* **C++:** Header-only for Embedded Systems.
* **C# / .NET:** For Unity, Godot & Windows Enterprise.
* **Java:** For Android & Enterprise Backend.
* **Python:** For Data Science & Rapid Prototyping.
* **Node.js:** For Backend Services and IoT Gateways.
* **VB.NET:** For legacy industrial HMI panels.

---

## 📞 Contact

**BrainAI** - *Intelligence everywhere.* **#WeKnowPhysiks** Developed by **Sascha A. Köhne (winemp83)** Product: **BioAI 0.7.5 (Industrial Closed Feature)** 📧 [koehne83@googlemail.com](mailto:koehne83@googlemail.com)

© 2025 BrainAI / Sascha A. Köhne. All rights reserved.

