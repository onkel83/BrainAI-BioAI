# 🌐 Technisches Handbuch: BioAI JSON Bridge (v0.7.6)

Die JSON Bridge abstrahiert die nativen Wrapper-Aufrufe und bietet eine plattformunabhängige, zustandslose Schnittstelle zur Kommunikation mit der BioAI-Engine. Dies ermöglicht die nahtlose Integration in Web-Frameworks, Microservices oder heterogene Systemlandschaften.

---

## 1. JSON-Schema Definitionen

Um eine konsistente Kommunikation zu gewährleisten, nutzt die Bridge standardisierte Strukturen für Anfragen und Antworten.

### A. Denkvorgang (Update Request)

Sendet Wahrnehmungen als Array von Hex-Strings an die Engine und erwartet eine Handlungsentscheidung.

* **Input**: `{"command": "UPDATE", "inputs": ["0x1000..."]}`
* **Output**: `{"status": "OK", "action": "0x2000..."}`

### B. Feedback (Learning Request)

Gibt eine Belohnung oder Bestrafung für eine spezifische Aktion zurück.

* **Input**: `{"command": "FEEDBACK", "reward": 1.0, "action": "0x2000..."}`
* **Output**: `{"status": "OK", "message": "Feedback applied"}`

---

## 2. API-Referenz: JSON Bridge

Die Bridge dient als Middleware und übernimmt das Parsing sowie die Typkonvertierung zwischen JSON-Datentypen und nativen 64-Bit-Tokens.

| Feld | Typ | Beschreibung |
| --- | --- | --- |
| `command` | String | Der Befehl (`UPDATE`, `FEEDBACK`, `SET_MODE`). |
| `inputs` | Array | Liste von TokenIDs als Hex-Strings (z.B. `"0x1000...n"`). |
| `reward` | Float | Belohnungswert zwischen `-1.0` und `1.0`. |
| `status` | String | Rückgabestatus (`OK` oder `ERROR`). |

---

## 3. Sicherheits- & Integrationshinweise (ISS)

* **Token-Validierung**: Die Bridge stellt sicher, dass eingehende Hex-Strings korrekt in `BigInt`-Werte umgewandelt werden, bevor sie an den C-Kern gereicht werden.
* **Key-Sicherheit**: Auch im JSON-Betrieb ist die `key.json` zwingend erforderlich. Ohne den mathematischen Anker liefert die Engine keine validen kausalen Ergebnisse.
* **Performance & Latenz**: Die Abstraktion durch JSON erzeugt einen geringfügigen Overhead (ca. 0,1ms) durch das Serialisieren und Parsen. Für extrem zeitkritische Anwendungen im Millisekundenbereich sollte der direkte Wrapper-Zugriff bevorzugt werden.

---

**BrainAI** *- We don't need **BRUTEFORCE**, we know **Physics** -*</br>
Developed by **Sascha A. Köhne (winemp83)**</br>
Product: **BioAI 0.7.6 (Industrial Closed Feature)**</br>
📧 [koehne83@googlemail.com](mailto:koehne83@googlemail.com)

© 2025 BrainAI / Sascha A. Köhne. All rights reserved.