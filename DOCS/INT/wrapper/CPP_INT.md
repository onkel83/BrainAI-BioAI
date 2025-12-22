# BioAI C++ Header-Only Integration 🚀

**Version:** 0.7.5 (Industrial Closed Feature)

**Type:** Modern C++11 Wrapper (Header-Only)

**License:** MIT (Wrapper) / Proprietary (Core)

**Status:** Industrial Gold Standard

---

## 1. Overview

The C++ wrapper (`BioAI.hpp`) provides a **zero-overhead abstraction** layer around the native C-Core. It utilizes modern C++ features—such as RAII, exceptions, and `std::vector` support—to encapsulate the manual memory management of the C-interface into a safe, comfortable, and high-performance class structure.

### Key Features

* **RAII Compliance:** Automatic lifecycle management for brains and buffers (no manual `free` or `destroy` calls required).
* **Type Safety:** Strict `const`-correctness for all read-only operations.
* **STL Integration:** Native support for `std::vector` and `std::string`.
* **Zero Dependencies:** Requires only the standard C++ library and the BioAI binary.

---

## 2. Installation

Since the wrapper is "Header-Only," no separate compilation of the wrapper itself is necessary.

1. Copy `BioAI.hpp` into your project's `include` directory.
2. Copy the appropriate shared library (`bioai.dll` for Windows or `libbioai.so` for Linux) into your binary execution path.
3. Include the header in your source code:

```cpp
#include "BioAI.hpp"

```

---

## 3. Quick Start: Hello Brain

A minimal example demonstrating how to initialize an agent, inject a safety instinct, and execute a deterministic decision cycle.

```cpp
#include "BioAI.hpp"
#include <iostream>
#include <vector>

using namespace BioAI;

int main() {
    try {
        // 1. Initialize Agent (Seed for determinism: 0xBADC0FFEE)
        Agent myAI(0xBADC0FFEE);

        // 2. Define Symbolic Tokens (Name -> 64-bit Hash mapping)
        // Tokens are deterministic across all platforms.
        uint64_t sensor_fire = Agent::CreateToken("FIRE", Cluster::Object);
        uint64_t action_run  = Agent::CreateToken("RUN",  Cluster::Action);

        // 3. Inject Instinct (Hard-Coded Safety Rule)
        // "If FIRE is detected, then RUN" (Weight 1.0 = Absolute Command)
        myAI.ForceInstinct(sensor_fire, action_run, 1.0f);

        // 4. Inference (Process Perception in constant time)
        std::vector<uint64_t> perception = { sensor_fire };
        uint64_t decision = myAI.Think(perception);

        // 5. Evaluation
        if (decision == action_run) {
            std::cout << "BioAI Decision: RUN! (Correct Behavior)" << std::endl;
        } else {
            std::cout << "BioAI Decision: Unknown state." << std::endl;
        }

    } catch (const std::exception& e) {
        std::cerr << "Runtime Error: " << e.what() << std::endl;
        return -1;
    }
    return 0;
}

```

---

## 4. Advanced: Safety & Serialization

### Production Mode (The Safety Switch)

For safety-critical applications (e.g., ISO 26262 or Industrial Automation), you can physically lock the system's state to prevent runtime deviations.

```cpp
// Disables malloc() calls and freezes learning logic.
// The system is now 100% deterministic with constant execution time.
myAI.SetMode(BioMode::Production);

```

### Brain Cloning & Fleet Learning

BioAI makes it easy to transfer the state of one agent to another, facilitating swarm intelligence or rapid deployment.

```cpp
// Save learned state to a binary file
myAI.SaveToFile("learned_instincts.bin");

// Load state into a fresh agent (Cross-Platform compatible)
Agent fleetAgent(0);
fleetAgent.LoadFromFile("learned_instincts.bin");

```

---

## 5. API Reference

| Method | Description | Complexity |
| --- | --- | --- |
| `Think(inputs)` | Processes signals and selects the optimal action. | ** (Constant)** |
| `Simulate(inputs, depth)` | Predicts future outcomes via causal chains. | **** |
| `Learn(reward, action)` | Feedback-driven reinforcement (Hebbian-based). | **** |
| `ForceInstinct(...)` | Writes immutable rules into Long-Term Memory. | **** |
| `Inspect(...)` | Reads the synaptic weight between two concepts. | **** |

---

**BrainAI** - *Intelligence everywhere.* **#WeKnowPhysiks** Developed by **Sascha A. Köhne (winemp83)** Product: **BioAI 0.7.5 (Industrial Closed Feature)** 📧 [koehne83@googlemail.com](mailto:koehne83@googlemail.com)

© 2025 BrainAI / Sascha A. Köhne. All rights reserved.
