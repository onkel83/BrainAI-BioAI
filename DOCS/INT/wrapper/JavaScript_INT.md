# BioAI Node.js Enterprise Integration 🟢

**Version:** 0.7.6 (Industrial Closed Feature)

**Platform:** Node.js 16+ (Server / Electron / Edge Gateways)

**Architecture:** FFI (Foreign Function Interface)

**Backend:** Multi-Tier (IoT / SmartHome / Ultra)

---

## 1. Architecture

The BioAI Node.js wrapper is a high-performance **Enterprise Class** that interfaces with the native C-Core via `ffi-napi`. This architecture allows JavaScript developers to leverage the deterministic power of BioAI without leaving the Node ecosystem.

### The 3 Deployment Tiers

You can specify which engine to load during instantiation, depending on your hardware constraints or simulation goals:

```javascript
const { BioAI } = require('./bioai');

// A. ULTRA (Server / Big Data / High-End Gateways) - Default
const brain = new BioAI(0x1234n); 

// B. IOT (Simulation of constrained Embedded hardware)
const brain = new BioAI(0x1234n, './bin/BioAI_IoT.dll'); 

```

---

## 2. API Features (Industrial Standards)

### Ontological Structure: Clusters & Sub-Clusters

The Node.js wrapper provides full access to the hierarchical token system. This allows for semantic grouping of inputs and outputs at the hardware level.

```javascript
const { BioClusters } = require('./bioai');

// Primary Clusters
const OBJ = BioClusters.OBJECT;

// Sub-Clusters for Advanced Logic
const NEED   = BioClusters.NEED;    // Internal physiological needs
const GOAL   = BioClusters.GOAL;    // Long-term objectives
const REFLEX = BioClusters.REFLEX;  // Hard-coded Safety overrides

```

### Type Safety: The 64-bit Precision Trap

In standard JavaScript, the `Number` type is a 64-bit float, which causes precision loss for integers larger than . To maintain the integrity of our **64-bit TokenIDs**, the BioAI wrapper strictly uses **`BigInt`**.

> **Important:** Always suffix your hex or decimal tokens with `n` (e.g., `0xCAFEn`). This ensures your AI logic remains deterministic and precise down to the bit level.

---

## 3. Quick Start

```javascript
const { BioAI, BioClusters, BioMode } = require('./bioai');

// 1. Create instance (Defaults to Ultra Tier)
// Note the 'n' suffix for BigInt license keys
const brain = new BioAI(0xCAFEBABEn);

// 2. Generate Deterministic Tokens
const SENSOR_TEMP = BioAI.createToken("TEMP_HIGH", BioClusters.OBJECT);
const ACTION_COOL = BioAI.createToken("COOL_DOWN", BioClusters.ACTION);

// 3. Training / Instinct Injection
// Weights are floats (0.0 to 1.0)
brain.forceInstinct(SENSOR_TEMP, ACTION_COOL, 1.0);

// 4. Runtime Inference (O(1) complexity)
const result = brain.think([SENSOR_TEMP]);

if (result === ACTION_COOL) {
    console.log("BioAI Response: COOLING ACTIVATED");
}

// 5. Explicit Resource Management
// Mandatory to prevent memory leaks in the unmanaged C-Core
brain.dispose();

```

---

## 4. Deployment & Requirements

To use the BioAI Core in your Node.js project, ensure the native dependencies are met:

1. **Dependencies:** `npm install ffi-napi ref-napi`
2. **Native Binaries:** Place the required library (`bioai.dll`, `libbioai.so`, etc.) in your project root or set the path explicitly in the constructor.
3. **Environment:** Ensure your Node.js environment has the necessary build tools (Python/C++) for `node-gyp` if `ffi-napi` needs local compilation.

---

## 🛡️ Hard Safety Note

Because BioAI operates on **Fixed Structure Logic** without dynamic memory allocation during production, it is immune to memory fragmentation. However, as the developer, you must ensure that `brain.dispose()` is called when a lifecycle ends to release the unmanaged heap memory allocated by the native C-Core.

---

**BrainAI** - *Intelligence everywhere.* **#WeKnowPhysiks** Developed by **Sascha A. Köhne (winemp83)** Product: **BioAI 0.7.6 (Industrial Closed Feature)** 📧 [koehne83@googlemail.com](mailto:koehne83@googlemail.com)

© 2025 BrainAI / Sascha A. Köhne. All rights reserved.

