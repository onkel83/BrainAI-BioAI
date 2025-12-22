# BioAI Python Integration 🐍

**Version:** 0.7.5 (Industrial Closed Feature)

**Platform:** Python 3.8+ (Windows / Linux / macOS / Raspberry Pi)

**Technology:** CTypes (Foreign Function Interface)

**Use Case:** Data Science, Rapid Prototyping, Research & Analysis

---

## 1. Overview

The BioAI Python wrapper provides a **pythonic interface** to the high-performance C-Core. It is designed for developers who want to train and validate AI logic in a comfortable high-level environment before deploying the exact same brain file to resource-constrained embedded devices using C++.

### Key Features

* **Zero Dependencies:** Uses the standard `ctypes` library. No `pip install` of heavy frameworks is required for the core functionality.
* **Hardware Simulation:** Load `BioAI_IoT.dll` to simulate the strict memory and neuron constraints of an Arduino or STM32 directly on your workstation.
* **Jupyter Ready:** Perfect for analyzing neural weights via the `inspect` API and visualizing the symbolic vocabulary registry.

---

## 2. Installation

1. **Wrapper:** Copy `bioai.py` into your project directory.
2. **Native Library:** Place the native binary (`bioai.dll` for Windows or `libbioai.so` for Linux) in the same folder.
* *Note:* You can rename any edition (e.g., `BioAI_Ultra.dll`) to `bioai.dll` to serve as the default.



---

## 3. Usage Example

```python
from bioai import BioAI, BioClusters, create_token

# 1. Initialize Brain with a specific seed for deterministic results
# Note: To test IoT constraints, use: BioAI(seed, "./BioAI_IoT.dll")
brain = BioAI(seed=0xCAFEBABE)

# 2. Define Symbolic Concepts
# Tokens are deterministic 64-bit hashes
SENSOR_LIGHT = create_token("LIGHT_LEVEL", BioClusters.OBJECT)
ACTION_LED   = create_token("LED_ON",      BioClusters.ACTION)

print(f"Token LIGHT: {SENSOR_LIGHT:#018x}")

# 3. Inject Instinct (Hard Safety/Rules)
# Weight 1.0 = Absolute law, non-negotiable
brain.force_instinct(SENSOR_LIGHT, ACTION_LED, 1.0)

# 4. Runtime Inference (Constant O(1) time)
perception = [SENSOR_LIGHT]
decision = brain.think(perception)

if decision == ACTION_LED:
    print("AI Decision: LED ON")
    # Provide positive reinforcement for the successful action
    brain.learn(1.0, decision) 

# 5. Export for Embedded Deployment
# The resulting .bin file is binary-compatible with the C++ IoT Core
brain.save("production_brain_iot.bin")

# 6. Cleanup unmanaged memory
brain.dispose()

```

---

## 4. API Reference

| Method | Description | Complexity |
| --- | --- | --- |
| `think(inputs)` | Primary inference. Processes perception and returns action token. | **** |
| `simulate(inputs, depth)` | Predictive engine. Mentally traverses causal chains. |  |
| `learn(reward, action)` | Applies reinforcement feedback to the recent trace. |  |
| `force_instinct(...)` | Injects permanent synaptic links (Instinct Layer). |  |
| `inspect(in, out)` | Returns the current weight of a specific connection. |  |
| `save(path)` | Exports the entire LTM as a platform-independent binary. |  |

---

## 🔬 Scientific Transparency: The Glass Box

Unlike traditional "Black Box" neural networks, BioAI in Python allows you to export the **Vocabulary Registry** using `dump_vocabulary()`. This maps the 64-bit hashes back to your string names, allowing for full auditability and explainability of every decision the AI makes.

---

**BrainAI** - *Intelligence everywhere.* **#WeKnowPhysiks** Developed by **Sascha A. Köhne (winemp83)** Product: **BioAI 0.7.5 (Industrial Closed Feature)** 📧 [koehne83@googlemail.com](mailto:koehne83@googlemail.com)

© 2025 BrainAI / Sascha A. Köhne. All rights reserved.

