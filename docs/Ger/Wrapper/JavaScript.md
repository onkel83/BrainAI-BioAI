# BioAI Node.js Enterprise Integration 🟢

**Version:** 0.7.6
**Platform:** Node.js 16+ (Server / Electron)
**Architecture:** FFI (Foreign Function Interface)
**Backend:** Multi-Tier (IoT / SmartHome / Ultra)

---

## 1. Architektur

Dieser Wrapper ist kein einfaches Skript, sondern eine **Enterprise-Klasse**, die das C-Backend über `ffi-napi` anbindet. 
Er unterstützt explizit die **Architektur-Trennung** von BioAI.

### Die 3 Einsatzwege (Tiers)
Sie können im Konstruktor entscheiden, welche Engine geladen wird:

```javascript
// A. ULTRA (Server / Big Data) - Standard
const brain = new BioAI(0x1234n); 

// B. IOT (Simulation von Embedded Constraints)
const brain = new BioAI(0x1234n, './bin/BioAI_IoT.dll'); 
````

-----

## 2\. API Features (Industrial Standard)

### Cluster & Sub-Cluster

Volle Unterstützung für die ontologische Struktur.

```javascript
const { BioClusters } = require('./bioai');

// Haupt-Cluster
const OBJ = BioClusters.OBJECT;

// Sub-Cluster (für komplexe Logik)
const NEED = BioClusters.NEED;   // Innere Bedürfnisse
const GOAL = BioClusters.GOAL;   // Langfristige Ziele
const REFLEX = BioClusters.REFLEX; // Hard-coded Safety
```

### Type Safety

Der Wrapper nutzt konsequent `BigInt` für alle Token-Operationen, um Präzisionsverluste bei 64-Bit Hashes zu vermeiden (ein bekanntes Problem bei Standard JS `Number`).

-----

## 3\. Quick Start

```javascript
const { BioAI, BioClusters, BioMode } = require('./bioai');

// 1. Instanz erstellen (Ultra Tier by default)
const brain = new BioAI(0xCAFEBABEn);

// 2. Token generieren
const SENSOR_TEMP = BioAI.createToken("TEMP_HIGH", BioClusters.OBJECT);
const ACTION_COOL = BioAI.createToken("COOL_DOWN", BioClusters.ACTION);

// 3. Training
brain.forceInstinct(SENSOR_TEMP, ACTION_COOL, 1.0);

// 4. Runtime
const result = brain.think([SENSOR_TEMP]);

if (result === ACTION_COOL) {
    console.log("System response: COOLING ACTIVATED");
}

// 5. Cleanup
brain.dispose();
```

-----

## 4\. Deployment

Stellen Sie sicher, dass `ffi-napi` installiert ist:
`npm install ffi-napi ref-napi`

Kopieren Sie die gewünschte DLL (`BioAI_Ultra.dll` etc.) in das Verzeichnis Ihres Node-Skripts oder benennen Sie sie in `bioai.dll` um.

-----

**BrainAI** - *-We don't need **BRUTEFORCE**, we know **Physiks**-*</br>
Developed by **Sascha A. Köhne (winemp83)**</br>
Product: **BioAI 0.7.6 (Industrial Closed Feature)**</br>
📧 [koehne83@googlemail.com](mailto:koehne83@googlemail.com)

&copy; 2025 BrainAI / Sascha A. Köhne. All rights reserved.