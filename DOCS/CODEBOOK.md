# BioAI Developer Handbook: Muster und Rezepte (v0.0.2)

Dieses Handbuch ist die universelle Anleitung, die Ihnen zeigt, wie Sie die vier Kernfunktionen des Cores nutzen, um komplexe Probleme zu lösen – **unabhängig von Ihrer Programmiersprache.**

---

## 0. Vorbereitung: Die Token-Sprache

BioAI arbeitet nur mit **TokenIDs** (64-Bit Zahlen). Sie müssen die Realität in diese Zahlen übersetzen. Die Cluster geben den Kontext vor.

| Sprache | Syntax: Erzeuge den Token 'HEAT_CRITICAL' |
| :--- | :--- |
| **C++ / C** | `const uint64_t T_HEAT = BioAI::Agent::CreateToken("Heat_Critical", BioAI::Cluster::Object);` |
| **C# / .NET / VB** | `ulong T_HEAT = BioClusters.CreateToken("Heat_Critical", BioClusters.OBJECT);` |
| **Python** | `T_HEAT = bioai.create_token("Heat_Critical", bioai.BioClusters.OBJECT)` |
| **Java / JS (BigInt)** | `T_HEAT = BioAI.createToken("Heat_Critical", BioAI.CLUSTER_OBJECT)` |

---

## 1. Muster: Der Unbrechbare Reflex (Safety Interlock)

Dies ist das Gesetz der KI. Ein Reflex hat immer Vorrang vor allen Plänen.

* **Konzept:** Eine unbrechbare, fest verdrahtete Verbindung im Langzeitgedächtnis (LTM).
* **Anwendung:** Not-Aus-Schalter, Lebenssicherung, harte Compliance-Regeln.

| Sprache | Code-Beispiel: Wenn HITZE > 100°C, dann STOPPEN! |
| :--- | :--- |
| **C++ / ROS 2** | `brain.ForceInstinct(T_HEAT_CRIT, T_STOP, 1.0f);` |
| **C# / VB.NET** | `brain.ForceInstinct(T_HEAT_CRIT, T_STOP, 1.0f);` |
| **Python / SAP RFC** | `brain.force_instinct(T_HEAT_CRIT, T_STOP, 1.0)` |
| **Java / JS** | `brain.forceInstinct(T_HEAT_CRIT, T_STOP, 1.0f)` |

---

## 2. Muster: Die Exakte Choreographie (Sequencer)

Für industrielle Prozesse, die keinen Raum für Fehler lassen. Die KI arbeitet strikt nach einem vorgegebenen Plan.

* **Konzept:** Die KI schaltet auf "SPS-Modus" um und arbeitet ein Token-Array ab.
* **Anwendung:** CNC-Steuerung, Montageprozesse.

| Sprache | Code-Beispiel: Plan laden und Status prüfen |
| :--- | :--- |
| **C++ / ROS 2** | `brain.LoadPlan(steps, true); status = brain.GetPlanStep();` |
| **C# / VB.NET** | `brain.LoadPlan(steps, true); status = brain.GetPlanStep();` |
| **Python / SAP RFC** | `brain.load_plan(steps_array, strict=True); status = brain.get_plan_step()` |
| **Java / JS** | `brain.loadPlan(stepsArray, true); status = brain.getPlanStatus();` |

---

## 3. Muster: Der Lern-Loop (Adaptive Erfahrung)

So bringen Sie dem Agenten bei, sich an neue Bedingungen anzupassen (z.B. Nutzerpräferenzen oder Motorverschleiß).

* **Konzept:** Think → Handle → Feedback (Reward/Punishment).
* **Anwendung:** Smart Home, Predictive Maintenance.

| Sprache | Code-Beispiel: Ausführen und Belohnen |
| :--- | :--- |
| **C++ / ROS 2** | `uint64_t decision = brain.Think(inputs); brain.Learn(0.1f, decision);` |
| **C# / VB.NET** | `ulong decision = brain.Think(inputs); brain.Learn(0.1f, decision);` |
| **Python / SAP RFC** | `decision = brain.think(inputs); brain.learn(0.1, decision)` |
| **Java / JS** | `long decision = brain.think(inputs); brain.learn(0.1f, decision)` |

---

## 4. Muster: Die Glaskugel (Vorausplanung)

Bevor der Agent handelt, überprüft er die Konsequenzen seiner Entscheidungen (Simulation).

* **Konzept:** Das System nutzt gelerntes Kausalitätswissen (Prediction) zur Vorhersage von Folgezuständen.
* **Anwendung:** Kollisionsvermeidung, Akku-Management, Strategie.

| Sprache | Code-Beispiel: 2 Schritte in die Zukunft schauen |
| :--- | :--- |
| **C++ / ROS 2** | `uint64_t bestAction = brain.Simulate(inputs, 2);` |
| **C# / VB.NET** | `ulong bestAction = brain.Simulate(inputs, 2);` |
| **Python / SAP RFC** | `best_action = brain.simulate(inputs, depth=2)` |
| **Java / JS** | `long bestAction = brain.simulate(inputs, 2)` |

---

**BrainAI** - *Intelligence everywhere.*
Developed by **Sascha A. Köhne (winemp83)**
Product: **BioAI v0.0.2 (Alpha)**
