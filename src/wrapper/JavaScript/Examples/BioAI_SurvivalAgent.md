# 🧬 Technisches Handbuch: BioAI JS Survival Agent Simulation (v0.7.6)

Dieses finale Beispiel führt alle architektonischen Konzepte der BioAI-Engine zusammen: **Bedürfnis-Steuerung, Reflex-Priorisierung und dynamisches Reinforcement-Learning**. Der Agent agiert als autonomes System, das interne Bedürfnisse (Hunger, Energie) gegen externe Reize (Nahrungssichtung) abwägt und dabei stets übergeordnete Sicherheits-Reflexe priorisiert.

## 1. Biologische Hierarchie (Priority Stack)

Der Agent nutzt die mathematische Befehlskette des Kerns, um sein Überleben zu sichern:

* **Reflex-Priorität (Überleben)**: Der Token `T_HEALTH_LOW` (Kritischer Status) triggert den Reflex `T_PANIC_REST` mit einem Gewicht von . Da Reflexe im `0x4010` Cluster liegen, unterdrücken sie mathematisch jede andere geplante Aktion, wie z.B. die Nahrungssuche, um einen sofortigen Systemzusammenbruch zu verhindern.
* **Adaptives Lernen (Erfahrung)**: Über die Methode `feedback()` lernt der Agent im laufenden Betrieb, welche Aktionen in welcher Situation erfolgreich sind. Er lernt beispielsweise, dass der Versuch zu essen (`T_EAT`) nur dann belohnt wird, wenn auch tatsächlich Nahrung (`T_FOOD_SEE`) wahrgenommen wird.

## 2. Cluster-Anwendung für Agenten 🟦🟥

BioAI verarbeitet Zustände als 64-Bit **TokenIDs**, die strikt nach Clustern getrennt sind, um eine fehlerfreie Adressierung im Langzeitgedächtnis (LTM) zu gewährleisten.

| Domäne | Cluster-Maske | Anwendung |
| --- | --- | --- |
| **Needs/Status** | `0x5000...` | Hunger, Energie, Kritische Gesundheit (Interne Zustände). |
| **Wahrnehmung** | `0x1000...` | Sichtbare Objekte in der Umwelt (z.B. Nahrung). |
| **Aktionen** | `0x2000...` | Motorische Befehle (Suchen, Essen, Schlafen). |
| **Sicherheit** | `0x4010...` | Angeborene Schutzinstinkte (Notfall-Ruhe). |

---

## 3. Performance & Speichersicherheit (ISS)

* **Garantiertes **: Die Inferenzzeit von `update()` bleibt unabhängig von der Menge der gesammelten Erfahrungen oder der Anzahl der aktiven Bedürfnisse immer konstant.
* **Null Speicher-Allokation**: Durch den `Production Freeze` (`setMode(1)`) nach der Instinkt-Injektion wird die neuronale Struktur versiegelt, was absolute Echtzeitstabilität garantiert.
* **Sovereign Security**: Die gelernten Überlebensstrategien sind mathematisch an die `key.json` gebunden. Ein Transfer der neuronalen Gewichte auf eine Engine mit einem anderen Schlüssel ist aufgrund des "Salting"-Verfahrens nicht möglich.

---

**BrainAI** *- We don't need **BRUTEFORCE**, we know **Physics** -*</br>
Developed by **Sascha A. Köhne (winemp83)**</br>
Product: **BioAI 0.7.6 (Industrial Closed Feature)**</br>
📧 [koehne83@googlemail.com](mailto:koehne83@googlemail.com)

© 2025 BrainAI / Sascha A. Köhne. All rights reserved.