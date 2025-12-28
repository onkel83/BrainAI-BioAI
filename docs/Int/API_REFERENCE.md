# BioAI.Core | Öffentliche API-Referenz 🧠

**Version:** 0.7.6 (Industrial Release)

**Architektur:** Sparse Associative Memory (SAM) / Neuro-Symbolic Engine

**Hardware-Unterstützung:** Multi-Tier (8-Bit bis 64-Bit)

---

## 1. System-Übersicht & Hardware-Tiers

Die BioAI-Engine passt ihre Kapazität automatisch an die zugrunde liegende Hardware-Architektur an. Lizenznehmer können zwischen verschiedenen Tiers wählen, um Speicherverbrauch und Rechenleistung zu optimieren.

| Tier | Zielplattform | Index-Breite | Max. Kapazität | Merkmale |
| --- | --- | --- | --- | --- |
| **IoT** | Embedded / AVR | 8-Bit | Niedrig | Minimaler RAM-Footprint |
| **SmartHome** | Edge / ESP32 | 16-Bit | Mittel | Optimiert für lokale Automation |
| **Ultra** | Desktop / Server | 32-Bit | Hoch | Standard für industrielle Analytik |
| **Next** | HPC / 64-Bit | 64-Bit | Maximum | Double-Precision für komplexe Simulationen |

---

## 2. Datenstrukturen & Adressierung

### TokenID (uint64_t)

Die gesamte Kommunikation mit der Engine erfolgt über 64-Bit-Identifikatoren, sogenannte **Tokens**. Diese Tokens repräsentieren sensorische Eingaben, interne Zustände oder motorische Aktionen.

### Cluster-Organisation

Tokens sind in logische Cluster unterteilt, die über die höchstwertigen Bits (MSB) adressiert werden:

* **Objekt-Cluster:** Repräsentation von Entitäten und Sensordaten.
* **Aktions-Cluster:** Definition ausführbarer Operationen.
* **Logik- & Reflex-Cluster:** Hochpriorisierte Steuerungsbefehle.
* **Sicherheits-Reflexe:** Spezielle Token-Masken lösen unmittelbare Reaktionen aus, die Standard-Entscheidungen überschreiben können.



---

## 3. Funktions-Schnittstellen (API)

### A. Lifecycle & Steuerung

* **`API_CreateBrain(key)`**: Initialisiert eine Instanz. Der erforderliche Lizenzschlüssel dient der Integritätsprüfung und der internen Datenabsicherung.
* **`API_FreeBrain(brainPtr)`**: Beendet die Instanz und gibt alle Ressourcen sicher frei.
* **`API_SetMode(brainPtr, mode)`**: Wechselt zwischen **Lernmodus** (dynamisch) und **Produktionsmodus** (Fixed Structure). Im Produktionsmodus findet kein strukturelles Wachstum statt, was 100% deterministisches Verhalten garantiert.

### B. Interaktion & Kognition

* **`API_Update(inputs, count)`**: Verarbeitet aktuelle Reize und liefert die optimale Aktion zurück. Die Funktion berücksichtigt dabei gelerntes Wissen, aktive Pläne und Reflexe.
* **`API_Simulate(inputs, count, depth)`**: Ermöglicht eine vorausschauende Analyse ("Imagination"), um die langfristigen Auswirkungen einer Aktion über mehrere Schritte zu bewerten.

### C. Lernen & Training

* **`API_Feedback(reward, action)`**: Wendet Reinforcement-Learning auf die letzte Aktionskette an. Positive Werte verstärken Verhaltensweisen, negative Werte führen zur Abschwächung.
* **`API_Teach(input, action, weight)`**: Erlaubt die explizite Programmierung von Wissen oder Sicherheitsregeln direkt in das Langzeitgedächtnis.
* **`API_Inspect(input, action)`**: Bietet volle Transparenz ("Glass Box"), indem die aktuelle Stärke einer spezifischen neuronalen Verbindung abgefragt werden kann.

---

## 4. Industrielle Sicherheit & Persistenz

### Integritätssicherung

Die BioAI-Engine nutzt ein internes Verschlüsselungsverfahren (Salting), um die im RAM gespeicherten neuronalen Gewichte gegen unbefugtes Auslesen und Manipulation zu schützen. Dieses Verfahren ist untrennbar mit dem bereitgestellten Lizenzschlüssel verknüpft.

### Serialisierung

Der gesamte Zustand eines "Brains" kann exportiert und wieder importiert werden.

* **`API_Serialize()`**: Erzeugt einen plattformunabhängigen Binär-Snapshot.
* **`API_Deserialize()`**: Lädt einen Zustand und validiert dabei die Tier-Kompatibilität, um Fehlkonfigurationen auf Zielhardware zu vermeiden.

---

## 5. Rechtliche Hinweise

Dieses Dokument beschreibt die öffentliche Schnittstelle der BioAI Core-Bibliothek. Die interne Implementierung, die spezifischen Salting-Algorithmen sowie die mathematischen Modelle zur Kausalitätsanalyse sind proprietäres Eigentum von BrainAI und durch Urheberrechte sowie Betriebsgeheimnisse geschützt.

**BrainAI** *- We don't need **BRUTEFORCE**, we know **Physics** -*</br>
Developed by **Sascha A. Köhne (winemp83)**</br>
Product: **BioAI 0.7.6 (Industrial Closed Feature)**</br>
📧 [koehne83@googlemail.com](mailto:koehne83@googlemail.com)

© 2025 BrainAI / Sascha A. Köhne. All rights reserved.