# 🤖 Technisches Handbuch: BioAI ROS 2 Robotic Control (v0.7.6)

Die BioAI ROS 2 Welding Node dient als deterministische Steuerungsschicht für industrielle Roboteranwendungen. Sie ersetzt unsichere, verzweigte `if-else`-Logiken durch eine mathematische Entscheidungsmatrix, die Trajektorienvorgaben (Pläne) mit hochpriorisierten Sicherheits-Reflexen in **garantierter  Zeit** synchronisiert.

---

## 1. System-Architektur & Entscheidungs-Flow

In einer ROS 2 Umgebung fungiert BioAI als "Safety & Logic Brain" innerhalb des Callback-Loops. Das System verarbeitet Topic-Daten (Sensoren) und entscheidet über die Veröffentlichung von Joint-Commands (Aktoren).

### Die hierarchische Priorisierung

Die Engine nutzt den internen **Priority Stack**, um widersprüchliche Befehle aufzulösen:

1. **Reflex-Ebene (Höchste Priorität)**: Ein detektiertes Hindernis (z. B. via Laser-Scanner Topic) triggert einen Reflex im `0x4010` Cluster. Dieser unterbricht die Trajektorie sofort, unabhängig vom aktuellen Fortschritt.
2. **Plan-Ebene (Sequenzierung)**: Wenn keine Reflexe aktiv sind, arbeitet die KI den geladenen Schweißpfad (Trajektorie) Schritt für Schritt ab.
3. **Erfahrungs-Ebene (LTM)**: Bei unvorhergesehenen Abweichungen greift das System auf gelernte Gewichte zurück, um den Prozess stabil zu halten.

---

## 2. Ontologie & Token-Mapping 🟦🟥

Die Kommunikation erfolgt über 64-Bit TokenIDs (BigInt), die eine verlustfreie Zuordnung von ROS-Nachrichten zu neuronalen Zuständen ermöglichen.

| Domäne | TokenID (Hex/BigInt) | Cluster | ROS 2 Entsprechung (Beispiel) |
| --- | --- | --- | --- |
| **Sensoren** | `0x1000...0001n` | **OBJECT** | `/laser_scan` oder `/camera/depth`. |
| **Trajektorie** | `0x2000...0001n` | **ACTION** | `/joint_trajectory_controller/command`. |
| **Sicherheit** | `0x4010...0001n` | **REFLEX** | Emergency Stop / Hardware Interrupt. |

---

## 3. Performance & Speichersicherheit (ISS)

* **Deterministik**: Die Inferenzzeit bleibt für jeden ROS-Zyklus identisch, was Jitter im Callback-Loop eliminiert.
* **Null Speicher-Allokation**: Im `mode 1` (Production) finden keine dynamischen RAM-Zuweisungen statt, was die Systemstabilität in Langzeit-Einsätzen garantiert.
* **Auditierbarkeit**: Jede Entscheidung kann über `inspect()` physikalisch begründet werden, da die Logik mathematisch an die `key.json` gebunden ist.

---

**BrainAI** *- We don't need **BRUTEFORCE**, we know **Physics** -*</br>
Developed by **Sascha A. Köhne (winemp83)**</br>
Product: **BioAI 0.7.6 (Industrial Closed Feature)**</br>
📧 [koehne83@googlemail.com](mailto:koehne83@googlemail.com)

© 2025 BrainAI / Sascha A. Köhne. All rights reserved.