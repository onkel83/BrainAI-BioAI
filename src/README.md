# 📂 BioAI Language Wrapper Suite

Willkommen im Integrations-Hub der BioAI-Engine. Dieser Ordner enthält die offiziellen Brücken-Implementierungen, um den hochperformanten C-Kern in modernen Hochsprachen zu nutzen.

## 1. Verfügbare Sprach-Integrationen

Jeder Wrapper ist in sich geschlossen und enthält eine eigene **README.md** mit spezifischen Installations- und Anwendungsbeispielen.

* **[C++ (RAII)](src/wrapper/Cpp/README.md)**: Native Performance mit automatischer Speicherverwaltung für moderne C++17 Standards.
* **[C# (.NET)](src/wrapper/CSharp/README.md)**: Optimiert für Windows Desktop, Cloud-Services und Unity-Integrationen.
* **[Java (JNA)](src/wrapper/JAVA/README.md)**: Plattformunabhängige Integration für JVM-basierte Enterprise-Systeme.
* **[JavaScript (Node.js)](src/wrapper/JavaScript/README.md)**: Event-basierte Anbindung für Echtzeit-Server und WebSocket-Gateways.
* **[Python (ctypes)](src/wrapper/Python/README.md)**: Leichtgewichtiger Wrapper ohne externe Abhängigkeiten für Data-Science und Prototyping.
* **[VB.NET](src/wrapper/VB/README.md)**: Klassische Integration für industrielle Legacy-Systeme und moderne Windows-Dienste.

---

## 2. Einheitliches Architektur-Konzept (The BioAI Way)

Unabhängig von der gewählten Sprache folgen alle Wrapper den gleichen Sicherheits- und Logik-Prinzipien der Engine (v0.7.6):

### Die TokenID Cluster 🟦🟥

Jede Wahrnehmung muss einem dieser 64-Bit Cluster zugeordnet werden, um eine korrekte Verarbeitung im Kern zu garantieren:

| Cluster | Maske (Hex) | Bedeutung | Beispiel |
| --- | --- | --- | --- |
| **OBJECT** | `0x1000...` | **Zustand / Objekt** | Sensordaten, Kameradetektionen, Status-Flags. |
| **ACTION** | `0x2000...` | **Handlung** | Steuersignale, Methodenaufrufe, physische Bewegungen. |
| **TIME** | `0x3000...` | **Zeitlicher Kontext** | Timer, Zyklen, Sequenz-Abstände. |
| **LOGIC** | `0x4000...` | **Regelwerk** | Logische Verknüpfungen und statische Abläufe. |
| **SELF** | `0x5000...` | **Eigenzustand** | Akkuladung, Systemgesundheit, Zielvorgaben. |

> **Sicherheitshinweis:** Ein **Reflex** (höchste Priorität) wird über die Sub-Maske `0x4010...` definiert. Ein solcher Reflex überschreibt bei einem Gewicht von  sofort jede gelernte Erfahrung.


### Die "Must-Have" Artefakte

Für jede Integration benötigen Sie zwingend die Dateien aus dem `bin`-Ordner des Hauptprojekts:

1. **Die Tier-Binary**: (z.B. `libs/Betriebssystem/BioAI_Ultra.dll` oder `.so`).
2. **Die Key-Datei**: (`key.json`), welche den mathematischen Anker für das De-Salting der Gewichte enthält.

---

## 3. Sicherheit & Performance

* **Konstante Laufzeit**: Alle Wrapper garantieren einen Aufruf des Kerns in  Komplexität.
* **Sovereign Security**: Der in den Wrappern implementierte Key-Load-Mechanismus schützt Ihre IP vor unbefugtem Auslesen (Weight Obfuscation).
* **Memory Safety**: Die Wrapper nutzen sprachenspezifische Mechanismen (RAII, Dispose, close(), try-with-resources), um Memory Leaks im unverwalteten C-Speicher zu verhindern.

---

**BrainAI** *- We don't need **BRUTEFORCE**, we know **Physics** -*</br>
Developed by **Sascha A. Köhne (winemp83)**</br>
Product: **BioAI 0.7.6 (Industrial Closed Feature)**</br>
📧 [koehne83@googlemail.com](mailto:koehne83@googlemail.com)

© 2025 BrainAI / Sascha A. Köhne. All rights reserved.
