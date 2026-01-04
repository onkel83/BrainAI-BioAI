# 📂 BioAI JavaScript (Node.js) Examples: Index & Navigation

Willkommen im Beispiel-Hub für die Node.js-Integration der BioAI-Engine (**v0.7.6**). Dieses Verzeichnis enthält industrielle Implementierungen, die das Spektrum der neuro-symbolischen Steuerung abdecken – optimiert für asynchrone Echtzeit-Angebote und Server-Side-Logik.

## 📋 Übersicht der Anwendungsbeispiele

Jedes Beispiel umfasst eine technische Dokumentation (** .md**) und das dazugehörige JavaScript-Skript (** .js**).

| Anwendungsbereich | Dokumentation (Deep Dive) | Quellcode | Kurzbeschreibung |
| --- | --- | --- | --- |
| **Industrie** | [📘 Advanced Production](./BioAI_AdvancedProductionLine.md) | [.js](./BioAI_AdvancedProductionLine.js) | Komplexe Fertigungsstraße mit Sicherheits-Interlocks. |
| **Energie** | [☀️ Solar System](./BioAI_AutonomousSolarSystem.md) | [.js](./BioAI_AutonomousSolarSystem.js) | Autonomes Energiemanagement & Batterien-Schutz. |
| **Finanzen** | [📊 DATEV Check](./BioAI_DATEVSanityCheck.md) | [.js](./BioAI_DATEVSanityCheck.js) | Echtzeit-Validierung von Buchungssätzen in . |
| **Aeronautik** | [🛸 Drohnen-Steuerung](./BioAI_Dronen.md) | [.md](./BioAI_Dronen.md) | Failsafe-Logik & MAVLink-Anbindung für UAVs. |
| **Steuerung** | [🏭 Production Controller](./BioAI_IndustrialProductionController.md) | [.js](./BioAI_IndustrialProductionController.js) | Deterministische Steuerung & Prozess-Auditierung. |
| **Web/API** | [🌐 JSON Bridge](./BioAI_Json_Bridge.md) | [.js](./BioAI_Json_Bridge.js) | Messaging-Gateway für Microservices & Web-Integration. |
| **Gaming** | [🎮 NPC Guide](./BioAI_NPCGuide.md) | [.js](./BioAI_NPCGuide.js) | Lernfähige Agenten mit autonomer Entscheidungshierarchie. |
| **Robotik** | [🤖 Robot Simulation](./BioAI_RobotSimulation.md) | [.js](./BioAI_RobotSimulation.js) | Kollisionsvermeidung & adaptives Reinforcement Learning. |
| **ROS 2** | [🦾 Welding Node](./BioAI_RosWeldingNode.md) | [.js](./BioAI_RosWeldingNode.js) | Trajektorien-Überwachung für Roboter-Schweißzellen. |
| **ERP** | [🧩 SAP Controller](./BioAI_SAP_FactoryController.md) | [.js](./BioAI_SAP_FactoryController.js) | Anbindung von Produktionslogik an SAP S/4HANA. |
| **Infrastruktur** | [📦 Smart Sorting](./BioAI_SmartSortingServer.md) | [.js](./BioAI_SmartSortingServer.js) | Logistik-Sortieranlage via OPC UA Protokoll. |
| **Biologie** | [🧬 Survival Agent](./BioAI_SurvivalAgent.md) | [.js](./BioAI_SurvivalAgent.js) | Simulation biologischer Bedürfnisse & Überlebenslogik. |

---

## 🛠️ Gemeinsame Architektur-Grundlagen

Die JavaScript-Integration nutzt den nativen **BioAI v0.7.6 Core** via FFI-NAPI und folgt strikten Sicherheits- und Performance-Standards:

1. **Sovereign Security**: Der Zugriff auf neuronale Gewichte erfordert die `key.json`. Das System nutzt mathematisches "Salting", um IP-Schutz zu garantieren.
2. **BigInt Handling**: Alle TokenIDs werden als `BigInt` (z. B. `0x1000...n`) verarbeitet, um die 64-Bit-Präzision des C-Kerns verlustfrei beizubehalten.
3. **Priority Stack**: Entscheidungen folgen der Hierarchie **Reflex > Plan > Erfahrung**.
4. **Deterministik & Echtzeit**: Durch den `Production Mode` (Mode 1) wird die Struktur versiegelt, was Jitter eliminiert und  Antwortzeiten garantiert.

---

## 🚀 Ausführungshilfe

Um ein Beispiel in Ihrer Node.js-Umgebung zu starten, müssen die Abhängigkeiten (`ffi-napi`, `ref-napi`) installiert sein und die native Bibliothek (`.dll`/`.so`) im Pfad liegen.

```bash
# Abhängigkeiten installieren
npm install ffi-napi ref-napi

# Beispiel starten (z. B. Roboter-Simulation)
node BioAI_RobotSimulation.js

```

---

**BrainAI** *- We don't need **BRUTEFORCE**, we know **Physics** -*</br>
Developed by **Sascha A. Köhne (winemp83)**</br>
Product: **BioAI 0.7.6 (Industrial Closed Feature)**</br>
📧 [koehne83@googlemail.com](mailto:koehne83@googlemail.com)

© 2025 BrainAI / Sascha A. Köhne. All rights reserved.