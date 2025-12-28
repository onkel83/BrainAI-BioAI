# 📘 BioAI JSON Messaging Guide (v0.7.6)

Dieser Guide beschreibt das standardisierte JSON-Protokoll zur Kommunikation mit der BioAI-Engine.

## 1. JSON-Schema Definitionen

Um eine konsistente Kommunikation über verschiedene Plattformen hinweg zu gewährleisten, nutzt BioAI die folgenden JSON-Strukturen:

### A. Denkvorgang (Update Request)

Sendet Wahrnehmungen an die Engine und erwartet eine Entscheidung.

```json
{
  "command": "UPDATE",
  "inputs": [
    "0x1000000000000001", 
    "0x100000000000000A"
  ]
}

```

### B. Feedback (Learning Request)

Gibt eine Belohnung oder Bestrafung für eine Aktion zurück.

```json
{
  "command": "FEEDBACK",
  "reward": 1.0,
  "action": "0x2000000000000005"
}

```

---

## 2. API-Referenz: JSON Bridge

Der JSON-Wrapper abstrahiert die nativen Methoden und bietet eine zustandslose Schnittstelle.

| Feld | Typ | Beschreibung |
| --- | --- | --- |
| `command` | String | Der auszuführende Befehl (`UPDATE`, `FEEDBACK`, `TEACH`, `SET_MODE`). |
| `inputs` | Array | Liste von TokenIDs als Hex-Strings (Cluster-Masken beachten!). |
| `reward` | Float | Belohnungswert zwischen `-1.0` und `1.0`. |
| `status` | String | Rückgabewert der Bridge (`OK` oder `ERROR`). |

---

## 3. Sicherheits- & Integrationshinweise (ISS)

1. **Token-Validierung**: Die Bridge validiert, ob die eingehenden Hex-Strings korrekte 64-Bit-Werte sind, bevor sie an den C-Kern gereicht werden.
2. **Key-Sicherheit**: Auch im JSON-Modus wird die `key.json` zur Initialisierung benötigt. Ohne den korrekten mathematischen Anker liefert die Bridge nur verfälschte Ergebnisse.
3. **Latenz**: Die JSON-Abstraktion erzeugt einen geringfügigen Overhead durch das Parsing (ca. 0.1ms). Für zeitkritische Echtzeit-Anwendungen (z. B. Drohnen-Steuerung) sollte der direkte **C++ oder C# Wrapper** verwendet werden.

---

**BrainAI** *- We don't need **BRUTEFORCE**, we know **Physics** -*</br>
Developed by **Sascha A. Köhne (winemp83)**</br>
Product: **BioAI 0.7.6 (Industrial Closed Feature)**</br>
📧 [koehne83@googlemail.com](mailto:koehne83@googlemail.com)

© 2025 BrainAI / Sascha A. Köhne. All rights reserved.