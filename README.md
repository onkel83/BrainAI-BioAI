# BioAI.Core 🧠

**Version 0.7.5 (Industrial Closed Feature)**

| BrainAI | BioAI |
| :---: | :---: |
| <img src="images/brainai_logo_bw.png" width="250" alt="BrainAI Logo"> | <img src="images/bioai_logo_bw.png" width="250" alt="BioAI Logo"> |
| *„BrainAI, we don't need Bruteforce we know Physics“* | *„BioAI doesn't guess. It survives.“* |</p>

---

**The Universal Neuro-Symbolic Engine for Edge AI & Swarm Robotics**

BioAI.Core ist eine hochperformante, deterministische KI-Engine, entwickelt für **Edge Computing** und **Echtzeit-Systeme**.

Im Gegensatz zu herkömmlichen LLMs (Transformer), die Gigabytes an Speicher benötigen, liefert BioAI eine vollständige neuro-symbolische Engine in einer **~20 KB bis 65 KB großen Binary**.
Sie läuft lokal, ohne Cloud, auf kleinster Hardware – vom Arduino bis zum Hochleistungsserver.

---

## ⚡ Key Performance Metrics

* **Scalable Precision:** Die Engine passt sich der Hardware an (8-Bit, 16-Bit, 32-Bit oder 64 Bit Indizierung).
* **Realtime Safety:** Entscheidungsfindung in **garantiertem O(1)** (konstante Zeit) durch Hard-Caps und Hash-Logik. Zertifizierbar nach ISO-Standards.
* **Universal:** Läuft auf Bare-Metal (Arduino), RTOS (ESP32), Linux und Windows.
* **No Hallucination:** Das System ist deterministisch. Es erfindet keine Fakten, sondern optimiert Ziele basierend auf verifizierten Inputs.

---

## 🏗️ Architecture Tiers (Die 3 Editionen)

BioAI.Core ist in drei Leistungsstufen verfügbar, um jeden Hardware-Bereich optimal abzudecken. Der Code ist identisch, die Skalierung erfolgt beim Kompilieren.

| Edition | Ziel-Hardware | Max. Neuronen | Index-Größe | Speicherbedarf (RAM) |
| :--- | :--- | :--- | :--- | :--- |
| **IoT** | Arduino, STM32, ESP8266 | **255** | 8-Bit | **< 2 KB** |
| **SmartHome** | ESP32, Raspberry Pi, HMI | **65.535** | 16-Bit | **~ 50 KB - 1 MB** |
| **Ultra** | PC, Server, Cloud AI | **4.294.967.295** | 32-Bit | RAM limitiert |
| **Next** | High-End Server, HPC | **18.446.744.073.709.551.615** | 64-Bit | RAM limitiert |

> **Hinweis:** Ein Gehirn, das auf der **Ultra**-Version trainiert wurde, kann auf **IoT**-Hardware laufen, sofern es die maximalen Neuronengrenzen (255) nicht überschreitet.

---

## 🌍 Universal Language Support

Der C-Kern ist über native Wrapper in fast jeder Umgebung nutzbar.
Klicken Sie auf die Sprache für die spezifische Integrations-Anleitung:

| Sprache | Dokumentation | Use Case |
| :--- | :--- | :--- |
| **C++** | [📘 **C++ Integration**](DOCS/Wrappers/c++.md) | Embedded Systems, High-Performance, Unreal Engine |
| **C# / .NET** | [📗 **C# & Unity Guide**](DOCS/Wrappers/c#.md) | Unity 3D, Godot, Windows Desktop, MAUI |
| **Java** | [☕ **Java JNA Guide**](DOCS/Wrappers/java.md) | Android Apps, Enterprise Backend (Spring) |
| **Python** | [🐍 **Python Guide**](DOCS/Wrappers/Python.md) | Data Science, Raspberry Pi, Rapid Prototyping |
| **Node.js** | [🟢 **Node.js Guide**](DOCS/Wrappers/JavaScript.md) | Backend Services, Electron Apps, Node-RED |
| **VB.NET** | [🏭 **Industrial Guide**](DOCS/Wrappers/vb.md) | Legacy Industrial Control (HMI/SCADA) |

Integrations Beispiele für spezielle Frameworks und Protokolle:

|Schnittstellen | Dokumentation | Use Case |
|:--- | :--- | :--- |
| **ROS2** | [🤖 **ROS2 Integration**](DOCS/Wrappers/ros2.md) | Swarm Robotics, Autonomous Systems |
| **SAP** | [🧩 **SAP Integration**](DOCS/Wrappers/sap.md) | Enterprise Automation, ERP Systems |
|**OPCUA**| [🔗 **OPCUA Guide**](DOCS/Wrappers/opcua.md) | Industrial IoT, SCADA Systems |

---

## 🚀 Use Cases

### 1. Industrial IoT & Smart Home
Selbstlernende Heizungssteuerung oder Netz-Stabilisierung (Smart Grid), die ohne Cloud-Verbindung läuft und Privatsphäre garantiert.

### 2. Swarm Robotics (Drones)
Hunderte Drohnen koordinieren sich dezentral (*Consent Protocol*), vermeiden Kollisionen und teilen Zielinformationen in Echtzeit ohne Master-Server.

### 3. Next-Gen NPCs
Spielcharaktere in Unity/Godot, die echte Bedürfnisse haben, lernen und soziale Strukturen bilden, ohne die CPU durch komplexe Behavior Trees zu belasten.

---

## 📚 Documentation

Hier finden Sie die detaillierten technischen Dokumente:

* [**Architecture Deep Dive**](DOCS/ARCHITECTURE.md) – *Why Efficiency beats Brute Force*
* [**API Reference**](DOCS/API_REFERENCE.md) – *Methods, Safety & Audit*
* [**Training Guide**](DOCS/TRAININGS_GUIDE.md) – *Instinct vs. Experience*
* [**Use Case: Smart Grid**](DOCS/BENCHMARK_SOLAR.md) – *BioAI vs. Cloud AI vs. Hardcoded*
* [**Simple Explainer**](DOCS/EXPLAIN_LIKE_IM_FIVE.md) – *Für Nicht-Techniker (ELI5)*
* [**CODEBOOK**](DOCS/CODEBOOK.md) – *Muster und Rezepte*

---

### 🚀 Update: Arduino Mega Integration (IoT-Edition-Spezial)

Die Portierung und Integration für den Arduino Mega 2560 wurde erfolgreich abgeschlossen.

* **Garantierte Echtzeit:** Die Entscheidungsfindung (Inferenz) erfolgt in einer stabilen, deterministischen Zeit von **~335 µs** pro Zyklus (O(1) Komplexität).
* **Minimaler Footprint:** Die IoT-Edition nutzt eine hochoptimierte Binary, die mit **weniger als 2 KB RAM** auskommt.
* **Hardware-nahe Implementierung:** Volle Unterstützung für neuro-symbolische Instinkte und Echtzeit-Lernen direkt auf dem ATmega2560.

> **Hinweis zur Verfügbarkeit:** > Aufgrund der spezialisierten Natur der Bare-Metal-Optimierung werden die spezifische Library (`.a`), der optimierte Header (`BioAI_Mega.h`) sowie die zugehörigen Dokumentationen aktuell **nur auf Anfrage** herausgegeben.

Bei Interesse an der IoT-Edition für industrielle Anwendungen oder Forschungsprojekte kontaktiere bitte den Entwickler direkt unter: **koehne83@googlemail.com**.

---
## 🛡️ License & Contact

BioAI.Core ist **Closed Source Technology**.
Die Binary ist für nicht-kommerzielle Nutzung frei verfügbar.
Für industrielle Lizenzen kontaktieren Sie bitte den Entwickler.

**BrainAI** - *Intelligence everywhere.*
Developed by **Sascha A. Köhne (winemp83)**
Product: **BioAI v0.7.5 (Industrial Closed Feature)**
📧 [koehne83@googlemail.com](mailto:koehne83@googlemail.com)

&copy; 2025 BrainAI / Sascha A. Köhne. All rights reserved.
