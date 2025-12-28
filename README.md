# BioAI.Core 🧠

**Version 0.7.6 (Industrial Closed Feature)**

| BrainAI | BioAI |
| :---: | :---: |
| <img src="images/brainai_logo_bw.png" width="250" alt="BrainAI Logo"> | <img src="images/bioai_logo_bw.png" width="250" alt="BioAI Logo"> |
| *„BrainAI, we don't need Bruteforce we know Physiks“* | *„BioAI doesn't guess. It survives.“* |</p>

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

| Edition | Ziel-Hardware | Max. Neuronen | Index-Größe | Speicherbedarf (RAM) | Verfügbarkeit |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **IoT** | Arduino, STM32, ESP8266 | **255** | 8-Bit | **< 2 KB** | *Aufanfrage* |
| **SmartHome** | ESP32, Raspberry Pi, HMI | **65.535** | 16-Bit | **~ 50 KB - 1 MB** | **Im Repository** |
| **Ultra** | PC, Server, Cloud AI | **4.294.967.295** | 32-Bit | RAM limitiert | **Im Repository** |
| **Next** | High-End Server, HPC | **18.446.744.073.709.551.615** | 64-Bit | RAM limitiert | *Aufanfrage* |

>[!IMPORTANT]
> **Hinweis:** Ein Gehirn, das auf der **Ultra**-Version trainiert wurde, kann auf **IoT**-Hardware laufen, sofern es die maximalen Neuronengrenzen (255) nicht überschreitet.

---

## 🌍 Universal Language Support

Der C-Kern ist über native Wrapper in fast jeder Umgebung nutzbar.
Klicken Sie auf die Sprache für die spezifische Integrations-Anleitung:

| Sprache | Dokumentation | Use Case |
| :--- | :--- | :--- |
| **C++** | [📘 **C++ Integration**](docs/Ger/Wrappers/c++.md) | Embedded Systems, High-Performance, Unreal Engine |
| **C# / .NET** | [📗 **C# & Unity Guide**](docs/Ger/Wrappers/c#.md) | Unity 3D, Godot, Windows Desktop, MAUI |
| **Java** | [☕ **Java JNA Guide**](docs/Ger/java.md) | Android Apps, Enterprise Backend (Spring) |
| **Python** | [🐍 **Python Guide**](docs/Ger/Python.md) | Data Science, Raspberry Pi, Rapid Prototyping |
| **Node.js** | [🟢 **Node.js Guide**](docs/Ger/Wrappers/JavaScript.md) | Backend Services, Electron Apps, Node-RED |
| **VB.NET** | [🏭 **Industrial Guide**](docs/Ger/Wrappers/vb.md) | Legacy Industrial Control (HMI/SCADA) |

Integrations Beispiele für spezielle Frameworks und Protokolle:

|Schnittstellen | Dokumentation | Use Case |
|:--- | :--- | :--- |
| **ROS2** | [🤖 **ROS2 Integration**](docs/Ger/Integrations/ROS2.md) | Swarm Robotics, Autonomous Systems |
| **SAP** | [🧩 **SAP Integration**](docs/Ger/Integrations/SAP.md) | Enterprise Automation, ERP Systems |
| **OPCUA** | [🔗 **OPCUA Guide**](docs/Ger/Integrations//OPCUA.md) | Industrial IoT, SCADA Systems |

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

**Hier finden Sie die detaillierten technischen Dokumente:**

* [**Architecture Deep Dive**](docs/Ger/ARCHITECTURE.md) – *Why Efficiency beats Brute Force*
* [**API Reference**](docs/Ger/API_REFERENCE.md) – *Methods, Safety & Audit*
* [**Training Guide**](docs/Ger/TRAININGS_GUIDE.md) – *Instinct vs. Experience*
* [**Use Case: Smart Grid**](docs/Ger/BENCHMARK_SOLAR.md) – *BioAI vs. Cloud AI vs. Hardcoded*
* [**Simple Explainer**](docs/Ger/EXPLAIN_LIKE_IM_FIVE.md) – *Für Nicht-Techniker (ELI5)*
* [**CODEBOOK**](docs/Ger/CODEBOOK.md) – *Muster und Rezepte*
* [**Beispiele**](docs/Ger/Examples.md) – *Beispiele und Vorschläge zur Integration*

**Wrapper Dokumentationen:**

* [**CPP**](docs/Ger/Wrapper/c++.md) – *CPP Wrapper und Beispiele*
* [**CSharp**](docs/Ger/Wrapper/c#.md) – *CSharp Wrapper und Beispiele*
* [**Java**](docs/Ger/Wrapper/java.md) – *Java Wrapper und Beispiele*
* [**JavaScript**](docs/Ger/JavaScript.md) – *JavaScript Wrapper und Beispiele*
* [**Python**](docs/Ger/Python.md) – *Python Wrapper und Beispiele*
* [**VB.net**](docs/Ger/vb.md) – *VB.net Wrapper und Beispiele*

**ROS2 / SAP / OPCUA Dokumentationen:**


* [**ROS2**](docs/Ger/Integrations/ROS2.md) – *Ros2 Beispiel Integration*
* [**SAP**](docs/Ger/Integrations/SAP.md) – *SAP Beispiel Integration*
* [**OPCura**](docs/Ger/Integrations/OPCura.md) – *OPCura Beispiel Integration*


---
### 🚀 Update: Arduino Mega Integration (IoT-Edition-Spezial)

Die Portierung und Integration für den Arduino Mega 2560 wurde erfolgreich abgeschlossen.

* **Garantierte Echtzeit:** Die Entscheidungsfindung (Inferenz) erfolgt in einer stabilen, deterministischen Zeit von **~335 µs** pro Zyklus (O(1) Komplexität).
* **Minimaler Footprint:** Die IoT-Edition nutzt eine hochoptimierte Binary, die mit **weniger als 2 KB RAM** auskommt.
* **Hardware-nahe Implementierung:** Volle Unterstützung für neuro-symbolische Instinkte und Echtzeit-Lernen direkt auf dem ATmega2560.

>[!IMPORTANT]
> **Hinweis zur Verfügbarkeit:** > Aufgrund der spezialisierten Natur der Bare-Metal-Optimierung werden die spezifische Library (`.a`), der optimierte Header (`BioAI_Mega.h`) sowie die zugehörigen Dokumentationen aktuell **nur auf Anfrage** herausgegeben.


---
## 🛡️ License & Contact

BioAI.Core ist **Closed Source Technology**.
Die Binary ist für nicht-kommerzielle Nutzung frei verfügbar.
Für industrielle Lizenzen kontaktieren Sie bitte den Entwickler.

---

**BrainAI** - *-We don't need **BRUTEFORCE**, we know **Physiks**-*</br>
Developed by **Sascha A. Köhne (winemp83)**</br>
Product: **BioAI 0.7.6 (Industrial Closed Feature)**</br>
📧 [koehne83@googlemail.com](mailto:koehne83@googlemail.com)

&copy; 2025 BrainAI / Sascha A. Köhne. All rights reserved.
