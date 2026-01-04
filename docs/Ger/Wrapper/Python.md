# BioAI Python Integration 🐍

**Version:** 0.7.6
**Platform:** Python 3.8+ (Windows / Linux / Mac)
**Technology:** CTypes
**Use Case:** Data Science, Rapid Prototyping, Raspberry Pi

---

## 1. Overview

The BioAI Python wrapper provides a pythonic interface to the C-Core. It is ideal for testing logic on a PC before deploying the exact same brain file to an embedded device (C++).

### Features
* **Zero Dependencies:** Uses standard `ctypes`. No `pip install` required for the wrapper itself.
* **Tier Switching:** Load `BioAI_IoT.dll` to simulate constrained environments on your powerful dev machine.
* **Jupyter Ready:** Perfect for analyzing neural weights (`inspect`) and visualizing vocabulary.

---

## 2. Installation

1.  Copy `bioai.py` into your project.
2.  Place the native library (`bioai.dll` or `libbioai.so`) next to it.
    * *Tip:* You can rename `BioAI_Ultra.dll` to `bioai.dll`.

---

## 3. Usage Example

```python
from bioai import BioAI, BioClusters, create_token

# 1. Init Brain (using default 'bioai.dll')
# To test IoT constraints: BioAI(seed, "./BioAI_IoT.dll")
brain = BioAI(seed=0xCAFEBABE)

# 2. Define Concepts
SENSOR_LIGHT = create_token("LIGHT_LEVEL", BioClusters.OBJECT)
ACTION_LED   = create_token("LED_ON",      BioClusters.ACTION)

print(f"Token LIGHT: {SENSOR_LIGHT:#018x}")

# 3. Teach Instinct
brain.force_instinct(SENSOR_LIGHT, ACTION_LED, 1.0)

# 4. Runtime Loop
inputs = [SENSOR_LIGHT]
decision = brain.think(inputs)

if decision == ACTION_LED:
    print("AI decided: LED ON")
    brain.learn(1.0, decision) # Reward

# 5. Save for Embedded Deployment
brain.save("brain_iot.bin")
````

-----

## 4\. API Reference

| Method | Description |
| :--- | :--- |
| `think(inputs)` | Process inputs and return action token. |
| `simulate(inputs, depth)` | Predict future outcomes. |
| `learn(reward, action)` | Reinforcement learning. |
| `force_instinct(...)` | Hardcode safety rules. |
| `save(path)` | Export binary brain file (compatible with C++). |

-----

**BrainAI** - *-We don't need **BRUTEFORCE**, we know **Physiks**-*</br>
Developed by **Sascha A. Köhne (winemp83)**</br>
Product: **BioAI 0.7.6 (Industrial Closed Feature)**</br>
📧 [koehne83@googlemail.com](mailto:koehne83@googlemail.com)

&copy; 2025 BrainAI / Sascha A. Köhne. All rights reserved.
© 2025 BrainAI / Sascha A. Köhne. All rights reserved.
