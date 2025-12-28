# BioAI Developer Handbook: Patterns & Recipes 🧠

**Version:** 0.7.6 (Industrial Closed Feature)

---

Dieses Handbuch ist die universelle Anleitung, die Ihnen zeigt, wie Sie die vier Kernfunktionen der Engine nutzen, um komplexe Probleme zu lösen – **unabhängig von Ihrer Programmiersprache.**

Die Architektur basiert auf einer **hochoptimierten, graph-artigen Struktur**, die es ermöglicht, Beziehungen zwischen Millionen von Konzepten in konstanter Zeit (**O(1)**) abzurufen und zu verarbeiten.

---

## 0. Vorbereitung: Die Token-Sprache

BioAI arbeitet intern ausschließlich mit **TokenIDs** (64-Bit Zahlen). Sie müssen die Realität in diese Zahlen übersetzen. Die Cluster-Zuordnung (Ontologie) gibt dem System den Kontext.

| Sprache | Syntax: Erzeuge den Token 'HEAT_CRITICAL' |
| :--- | :--- |
| **C++** | `const uint64_t T_HEAT = BioAI::Agent::CreateToken("Heat_Critical", BioAI::Cluster::Object);` |
| **C# / .NET** | `ulong T_HEAT = BioClusters.CreateToken("Heat_Critical", BioClusters.OBJECT);` |
| **Python** | `T_HEAT = create_token("Heat_Critical", BioClusters.OBJECT)` |
| **Java / JS** | `long T_HEAT = BioAI.createToken("Heat_Critical", BioAI.CLUSTER_OBJECT);` |

---

## 1. Muster: Der Unbrechbare Reflex (Safety Interlock)

Dies ist das Gesetz der KI. Ein Reflex hat immer Vorrang vor allen Plänen oder gelernten Verhaltensweisen. Er wird direkt in das Langzeitgedächtnis (LTM) injiziert.

* **Konzept:** Eine unbrechbare, fest verdrahtete Verbindung im Graphen.
* **Anwendung:** Not-Aus-Schalter, Lebenssicherung, harte Compliance-Regeln (ISO 26262).

| Sprache | Code-Beispiel: Wenn HITZE > 100°C, dann STOPPEN! |
| :--- | :--- |
| **C++** | `brain.ForceInstinct(T_HEAT_CRIT, T_STOP, 1.0f);` |
| **C# / VB** | `brain.ForceInstinct(T_HEAT_CRIT, T_STOP, 1.0f);` |
| **Python** | `brain.force_instinct(T_HEAT_CRIT, T_STOP, 1.0)` |
| **Java** | `brain.forceInstinct(T_HEAT_CRIT, T_STOP, 1.0f);` |

---

## 2. Muster: Die Exakte Choreographie (Sequencer)

Für industrielle Prozesse, die keinen Raum für Varianz lassen. Die KI arbeitet strikt nach einem vorgegebenen Plan, bleibt aber für Reflexe (Not-Aus) empfänglich.

* **Konzept:** Die KI schaltet auf einen deterministischen "SPS-Modus" um und arbeitet ein Token-Array ab.
* **Anwendung:** CNC-Steuerung, Montageprozesse, Roboter-Arm-Pfade.

| Sprache | Code-Beispiel: Plan laden und Status prüfen |
| :--- | :--- |
| **C++** | `brain.LoadPlan(steps, true); status = brain.GetPlanStep();` |
| **C# / VB** | `brain.LoadPlan(steps, true); status = brain.GetPlanStep();` |
| **Python** | `brain.load_plan(steps, strict=True); status = brain.get_plan_step()` |
| **Java** | `brain.loadPlan(steps, true); status = brain.getPlanStatus();` |

---

## 3. Muster: Der Lern-Loop (Adaptive Erfahrung)

So bringen Sie dem Agenten bei, sich an neue Bedingungen anzupassen (z.B. Nutzerpräferenzen oder Motorverschleiß). BioAI nutzt hierfür eine **angepasste Hebbian-Lernmethode**, die zeitliche Zusammenhänge (Kausalität) erkennt und verstärkt.

* **Konzept:** Wahrnehmen (Think) → Handeln → Feedback (Reward/Punishment).
* **Anwendung:** Smart Home, Predictive Maintenance, Energie-Optimierung.

| Sprache | Code-Beispiel: Ausführen und Belohnen |
| :--- | :--- |
| **C++** | `uint64_t decision = brain.Think(inputs); brain.Learn(0.1f, decision);` |
| **C# / VB** | `ulong decision = brain.Think(inputs); brain.Learn(0.1f, decision);` |
| **Python** | `decision = brain.think(inputs); brain.learn(0.1, decision)` |
| **Java** | `long decision = brain.think(inputs); brain.learn(0.1f, decision);` |

---

## 4. Muster: Die Glaskugel (Vorausplanung)

Bevor der Agent handelt, überprüft er die Konsequenzen seiner Entscheidungen in einer internen Simulation.

* **Konzept:** Das System nutzt gelerntes Kausalitätswissen (Prediction), um den Graphen "in die Zukunft" zu traversieren, ohne physisch zu handeln.
* **Anwendung:** Kollisionsvermeidung, Akku-Management, Schach-artige Strategien.

| Sprache | Code-Beispiel: 2 Schritte in die Zukunft schauen |
| :--- | :--- |
| **C++** | `uint64_t bestAction = brain.Simulate(inputs, 2);` |
| **C# / VB** | `ulong bestAction = brain.Simulate(inputs, 2);` |
| **Python** | `best_action = brain.simulate(inputs, depth=2)` |
| **Java** | `long bestAction = brain.simulate(inputs, 2);` |

---

**BrainAI** - *Intelligence everywhere.*
Developed by **Sascha A. Köhne (winemp83)**
Product: **BioAI 0.7.6 (Industrial Closed Feature)**
📧 [koehne83@googlemail.com](mailto:koehne83@googlemail.com)

© 2025 BrainAI / Sascha A. Köhne. All rights reserved.
