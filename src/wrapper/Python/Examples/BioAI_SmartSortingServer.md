# 🏭 Technisches Referenzhandbuch: BioAI OPC UA Sorting Gateway (0.7.6)

In modernen Industrie-4.0-Umgebungen fungiert BioAI als **neuro-symbolischer Logik-Coprozessor**. Dieses Gateway ermöglicht es einer speicherprogrammierbaren Steuerung (SPS), komplexe Entscheidungssequenzen an den BioAI-Kern auszulagern, während gleichzeitig hardwarenahe Sicherheits-Reflexe in **garantierter  Zeit** verarbeitet werden.

---

## 1. Funktionale Architektur: Das Gateway-Prinzip

Die Integration erfolgt über einen standardisierten OPC UA Server, der als bidirektionale Datendrehscheibe zwischen der physikalischen Feldebene (SPS) und der BioAI-Logikschicht dient.

* **SPS-seitiger Trigger (Input Node)**: Die SPS schreibt Sensor-Informationen als diskrete 64-Bit TokenIDs in den `PLC_Sensor_Input`. Dies triggert sofort den Inferenz-Zyklus im BioAI-Kern.
* **BioAI Inferenz (Think)**: Die Engine evaluiert den Input gegen den aktiven Prozessplan und die Sicherheits-Reflexe. Die Entscheidung erfolgt ohne statistische Fluktuation oder zeitlichen Jitter.
* **Aktor-Befehl (Output Node)**: Die resultierende Action-TokenID wird in den `AI_Action_Output` geschrieben, von wo die SPS sie ausliest und direkt in physische Bewegungen (z. B. Weichenstellung) umsetzt.

---

## 2. Die Sortier-Ontologie (Token-Mapping) 🟦🟥

Zur Gewährleistung einer kollisionsfreien Logik nutzt das System das BioAI-Cluster-Modell. Jede ID ist mathematisch eindeutig einer logischen Domäne zugeordnet.

### Prozess-Sequenz (Action Cluster: `0x2000...`)

| TokenID (Hex) | Bezeichnung | Funktion im Sorter |
| --- | --- | --- |
| `0x2000...0001` | **T_SCAN_BARCODE** | Aktivierung des Scanners zur Paketerfassung. |
| `0x2000...0002` | **T_WEIGHT_CHECK** | Abfrage der Wiegezelle zur Plausibilitätsprüfung. |
| `0x2000...0003` | **T_SIZE_CHECK** | Lasermessung der Paketdimensionen. |
| `0x2000...0004` | **T_ROUTE_A** | Auswurf auf Transportband A (Normalversand). |
| `0x2000...0005` | **T_ROUTE_B** | Auswurf auf Transportband B (Express). |

### Sicherheit & Sensorik (Object/Reflex Cluster)

| TokenID (Hex) | Cluster | Verwendung |
| --- | --- | --- |
| `0x1000...0001` | **OBJECT** | **T_HAND_DETECTED**: Lichtschranken-Signal (Eingriff detektiert). |
| `0x4010...0001` | **REFLEX** | **T_EMERGENCY_STOP**: Sofortige Stillsetzung aller Antriebe. |

---

## 3. Die logische Entscheidungshierarchie

Das System verhindert Regelkonflikte durch eine strikte Priorisierung innerhalb der `bio_think_logic`:

1. **Reflex-Priorität (Safety First)**: Der Reflex `T_HAND_DETECTED -> T_EMERGENCY_STOP` wird mit einem Gewicht von  injiziert. Tritt dieses Signal auf, wird jede aktive Sortiersequenz sofort abgebrochen, da Reflexe mathematisch schwerer wiegen als Pläne.
2. **Sequenz-Treue (Strict Plan)**: Im Normalbetrieb folgt die KI dem geladenen 6-Schritt-Plan. Die SPS muss lediglich ein "OK" oder einen Sensorwert senden, um den nächsten logischen Schritt im deterministischen Ablauf zu triggern.
3. **Lern-Assoziation (LTM)**: Erst wenn kein Plan und kein Reflex greifen, entscheidet die KI basierend auf gelerntem Wissen im Langzeitgedächtnis.

---

## 4. Industrial Sovereign Security (ISS) Standard

Die OPC UA Integration erfüllt höchste Sicherheitsansprüche durch die Bindung an den Hardware-Key:

* **Weight Salting**: Die im RAM liegenden neuronalen Gewichte sind durch den `customer_key` aus der `key.json` verschleiert. Ein physischer Speicher-Dump ist ohne den korrekten Key unlesbar.
* **Production Freeze**: Durch `set_mode(1)` wird der Speicherzustand versiegelt. Es finden keine neuen Allokationen statt, was Jitter verhindert und die Anlage vor "Neural Drift" schützt.
* **Auditierbarkeit**: Jede via OPC UA getroffene Entscheidung kann über `inspect()` (De-Salting via Key) belegt werden. Dies ermöglicht die lückenlose Dokumentation für Qualitätssicherung und Zertifizierungen.

---

**BrainAI** *- We don't need **BRUTEFORCE**, we know **Physics** -*</br>
Developed by **Sascha A. Köhne (winemp83)**</br>
Product: **BioAI 0.7.6 (Industrial Closed Feature)**</br>
📧 [koehne83@googlemail.com](mailto:koehne83@googlemail.com)

© 2025 BrainAI / Sascha A. Köhne. All rights reserved.