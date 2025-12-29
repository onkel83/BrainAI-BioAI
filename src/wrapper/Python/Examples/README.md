# 📂 BioAI Python Examples: Index & Navigation

Willkommen im Beispiel-Hub für die Python-Integration der BioAI-Engine (**0.7.6**). Dieses Verzeichnis enthält praxisnahe Implementierungen, die das gesamte Spektrum der neuro-symbolischen Steuerung abdecken – von industriellen Anlagen bis hin zu autonomen Agenten.

## 📋 Übersicht der Anwendungsbeispiele

Jedes Beispiel besteht aus einer ausführbaren **Python-Datei (.py)** und einer dazugehörigen **technischen Dokumentation (.md)**.

| Anwendungsbereich | Dokumentation (Deep Dive) | Quellcode | Kurzbeschreibung |
| --- | --- | --- | --- |
| **Industrie** | [📘 Advanced Production](./BioAI_AdvancedProductionLine.md) | [.py](./BioAI_AdvancedProductionLine.py) | Komplexe Fertigungsstraße mit multiplen Interlocks. |
| **Energie** | [☀️ Solar System](./BioAI_AutonomousSolarSystem.md) | [.py](./BioAI_AutonomousSolarSystem.py) | Autonomes Energiemanagement & Batterien-Schutz. |
| **Finanzen** | [📊 DATEV Check](./BioAI_DATEVSanityCheck.md) | [.py](./BioAI_DATEVSanityCheck.py) | Echtzeit-Validierung von Buchungssätzen in . |
| **Aeronautik** | [🛸 Drohnen-Steuerung](./BioAI_Dronen.md) | [.py](./BioAI_Dronen.py) | Failsafe-Logik & MAVLink-Anbindung für UAVs. |
| **Steuerung** | [🏭 Production Controller](./BioAI_IndustrialProductionController.md) | [.py](./BioAI_IndustrialProductionController.py) | Deterministische SPS-Ersatzlogik & Sequenzierung. |
| **Web/API** | [🌐 JSON Bridge](./BioAI_Json_Bridge.md) | [.py](./BioAI_Json_Bridge.py) | Messaging-Gateway für Cloud- & Web-Integrationen. |
| **Gaming** | [🎮 NPC Guide](./BioAI_NPCGuide.md) | [.py](./BioAI_NPCGuide.py) | Lernfähige Agenten mit echtem Überlebensinstinkt. |
| **Robotik** | [🤖 Robot Simulation](./BioAI_RobotSimulation.md) | [.py](./BioAI_RobotSimulation.py) | Kollisionsvermeidung & explorative Navigation. |
| **ROS 2** | [🦾 Welding Node](./BioAI_RosWeldingNode.md) | [.py](./BioAI_RosWeldingNode.py) | Trajektorien-Überwachung für Roboter-Schweißzellen. |
| **ERP** | [🧩 SAP Controller](./BioAI_SAP_FactoryController.md) | [.py](./BioAI_SAP_FactoryController.py) | Anbindung von Produktionslogik an Enterprise-Systeme. |
| **Infrastruktur** | [📦 Smart Sorting](./BioAI_SmartSortingServer.md) | [.py](./BioAI_SmartSortingServer.py) | Logistik-Sortieranlage via OPC UA Protokoll. |
| **Biologie** | [🧬 Survival Agent](./BioAI_SurvivalAgent.md) | [.py](./BioAI_SurvivalAgent.py) | Simulation biologischer Bedürfnisse & Reinforcement. |

---

## 🛠️ Gemeinsame Architektur-Grundlagen

Alle Beispiele nutzen den **BioAI v0.7.6 Core** und folgen denselben Sicherheitsprinzipien:

1. **Sovereign Security**: Jede Instanz benötigt die `key.json` aus dem `bin`-Ordner zur mathematischen De-Serialisierung der Gewichte.
2. **Priority Stack**: Entscheidungen werden hierarchisch getroffen (**Reflex > Plan > Erfahrung**).
3. **Deterministik**: Durch `set_mode(1)` (Production Mode) wird die neuronale Struktur eingefroren, was jeglichen Jitter und Speicher-Leaks eliminiert.

---

## 🚀 Ausführungshilfe

Um ein Beispiel zu starten, stellen Sie sicher, dass die `bioai.py` (Wrapper) und die entsprechende native Bibliothek (`.dll`/`.so`) im Pfad liegen.

```bash
# Beispiel: Starten der Roboter-Simulation
python BioAI_RobotSimulation.py

```

---

**BrainAI** *- We don't need **BRUTEFORCE**, we know **Physics** -*</br>
Developed by **Sascha A. Köhne (winemp83)**</br>
Product: **BioAI 0.7.6 (Industrial Closed Feature)**</br>
📧 [koehne83@googlemail.com](mailto:koehne83@googlemail.com)

© 2025 BrainAI / Sascha A. Köhne. All rights reserved.