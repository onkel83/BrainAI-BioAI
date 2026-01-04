# 🛸 Technisches Handbuch: BioAI JS Drone Controller (v0.7.6)

Dieses Handbuch beschreibt die Einbettung des **BioDroneController** in eine JavaScript-Umgebung. Das System verarbeitet Sensordaten in Echtzeit und trifft sicherheitskritische Entscheidungen in **garantierter  Zeit**, um die Integrität der Drohne auch bei Ausfall der primären Navigation zu gewährleisten.

---

## 1. Die Flug-Ontologie (Vocab-Dump) 🟦🟥

Innerhalb des BioAI-Systems werden physische Flugzustände als 64-Bit **TokenIDs** (JavaScript `BigInt`) abgebildet. Diese Struktur ermöglicht eine hocheffiziente Bewertung ohne verzweigte Logik-Kaskaden.

### Wahrnehmung (Inputs / OBJECT Cluster)

| Token | Hex-ID (BigInt) | Beschreibung | Schwellenwert (Logik) |
| --- | --- | --- | --- |
| `T_DIST_FRONT_CLOSE` | `0x1000000000000001n` | Hindernis im Nahbereich erkannt. | < 1.5m |
| `T_BATTERY_LOW` | `0x1000000000000002n` | Kritischer Akkuladestand. | < 15% |
| `T_GPS_LOST` | `0x1000000000000003n` | Verlust des Satelliten-Fix. | Boolean False |

### Reaktionen (Actions & Reflexes)

| Token | Hex-ID (BigInt) | Cluster | Hardware-Kommando (z.B. MAVLink) |
| --- | --- | --- | --- |
| `T_MOVE_FORWARD` | `0x2000000000000001n` | **ACTION** | Positionsziel mit Geschwindigkeit X+ setzen. |
| `T_HOVER` | `0x2000000000000002n` | **ACTION** | Aktive Geschwindigkeit auf Null setzen. |
| `T_EMERGENCY_LAND` | `0x4010000000000001n` | **REFLEX** | Sofortige Einleitung des Landevorgangs. |
| `T_RTL` | `0x4010000000000002n` | **REFLEX** | Return-to-Launch (Heimflug). |

---

## 2. Code-Referenz: BioDroneController.js

Die Steuerung nutzt den `BigInt`-Datentyp für alle Token-Operationen, um Präzisionsverluste zu vermeiden.

```javascript
const { BioBrainInstance } = require('./BioAI');

class BioDroneController {
    constructor(jsonPath, dllPath) {
        // Initialisierung des Gehirns
        this._brain = new BioBrainInstance(jsonPath, dllPath);
        this._initInstincts();
    }

    _initInstincts() {
        // Modus: Training (0) für Wissensinjektion
        this._brain.setMode(0);

        // Reflex: Akku leer -> Sofort Landen (Gewicht 1.0)
        this._brain.teach(0x1000000000000002n, 0x4010000000000001n, 1.0);
        
        // Reflex: Hindernis voraus -> Hover
        this._brain.teach(0x1000000000000001n, 0x2000000000000002n, 1.0);

        // Modus: Produktion (1) für deterministischen Flug
        this._brain.setMode(1);
    }

    updateFlightLogic(distFront, batteryPct, hasGps) {
        let activeInputs = [];
        // Sensor-Abstraktion
        if (distFront < 1.5) activeInputs.push(0x1000000000000001n);
        if (batteryPct < 15.0) activeInputs.push(0x1000000000000002n);
        if (!hasGps) activeInputs.push(0x1000000000000003n);

        // Denken (O(1) Inferenz)
        return this._brain.update(activeInputs);
    }
}

```

---

## 3. Sicherheits-Architektur (ISS Standard)

Das System nutzt eine vierstufige Hierarchie, um Flugunfälle mathematisch auszuschließen:

1. **Safety-Interrupts**: Sobald `T_BATTERY_LOW` detektiert wird, erzeugt der C-Kern einen Interrupt. Da dieses Token mit `T_EMERGENCY_LAND` (Gewicht 1.0) verknüpft ist, wird jede andere motorische Planung (z.B. Vorwärtsflug) unmittelbar unterdrückt.
2. **Deterministische Latenz**: Die Inferenzzeit von `update()` bleibt unabhängig von der Sensoranzahl oder dem Trainingsstand konstant. Dies ist für Flight-Controller, die oft mit 400Hz operieren, essentiell.
3. **Black-Box Integrity**: Die neuronalen Pfade im Langzeitgedächtnis (LTM) sind durch "Weight Salting" verschlüsselt. Eine Manipulation der Flugregeln ist ohne die spezifische `key.json` physikalisch nicht möglich.

---

## 4. Performance & Validierung

Das Modell wurde im Rahmen der ISS-Validierung (`Core_Tests.c`) zertifiziert:

* **Reaktionszeit**: Auslösen des `T_HOVER`-Reflexes bei Hinderniskontakt in weniger als 1ms.
* **Ressourcen**: Das Gehirn benötigt weniger als 64 KB RAM, was den Einsatz auf Edge-Boards ermöglicht.

---

**BrainAI** *- We don't need **BRUTEFORCE**, we know **Physics** -* </br>
Developed by **Sascha A. Köhne (winemp83)**</br>
Product: **BioAI 0.7.6 (Industrial Closed Feature)**</br>
📧 [koehne83@googlemail.com](mailto:koehne83@googlemail.com)

© 2025 BrainAI / Sascha A. Köhne. All rights reserved.