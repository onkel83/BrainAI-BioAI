# 🤖 Technisches Handbuch: BioAI ROS 2 Robotic Control (0.7.6)

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

Die Kommunikation erfolgt über 64-Bit TokenIDs, die eine verlustfreie und extrem schnelle Zuordnung von ROS-Nachrichten zu neuronalen Zuständen ermöglichen.

| Domäne | TokenID (Hex) | Cluster | ROS 2 Entsprechung (Beispiel) |
| --- | --- | --- | --- |
| **Sensoren** | `0x1000...` | **OBJECT** | `/laser_scan` oder `/camera/depth`. |
| **Trajektorie** | `0x2000...` | **ACTION** | `/joint_trajectory_controller/command`. |
| **Safety** | `0x4010...` | **REFLEX** | Emergency Stop / Hardware Interrupt. |

---

## 3. Implementierungs-Protokoll

### Wissensinjektion (Training Phase)

Bevor die Node aktiv wird, wird das Gehirn im `mode 0` (Training) mit den Sicherheits-Instinkten "geimpft". Durch die Methode `teach()` werden die Reflexe fest im Langzeitgedächtnis verankert.

### Deterministischer Flug (Production Phase)

Mit dem Wechsel in `mode 1` (Production) wird die neuronale Struktur eingefroren. Dies garantiert:

* **Keinen Jitter**: Die Inferenzzeit bleibt für jeden ROS-Zyklus identisch.
* **Null Speicher-Allokation**: Es entstehen keine Verzögerungen durch Garbage Collection oder dynamische RAM-Zuweisung.
* **Auditierbarkeit**: Jede Entscheidung kann über `inspect()` physikalisch begründet werden.

---

## 4. Sicherheit & Integrität (ISS Standard)

Die ROS 2 Welding Node erfüllt die strengen Anforderungen des **Industrial Sovereign Security** Standards:

* **Key-Bindung**: Die gesamte Schweißlogik ist mathematisch an die `key.json` gebunden. Ohne diesen Schlüssel ist die Binary für Dritte wertlos, da die neuronalen Gewichte verschlüsselt (gesalzen) vorliegen.
* **Offline-Betrieb**: Das System benötigt keinen Internetzugriff zur Validierung der Logik.
* **Integritäts-Check**: Der Build-Prozess stellt über `Core_Tests.c` sicher, dass die Reflex-Priorisierung mathematisch garantiert ist, bevor die Node deployed wird.

---

**BrainAI** *- We don't need **BRUTEFORCE**, we know **Physics** -*</br>
Developed by **Sascha A. Köhne (winemp83)**</br>
Product: **BioAI 0.7.6 (Industrial Closed Feature)**</br>
📧 [koehne83@googlemail.com](mailto:koehne83@googlemail.com)

© 2025 BrainAI / Sascha A. Köhne. All rights reserved.