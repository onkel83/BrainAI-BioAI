# 🏭 BioAI Advanced Production Line Demo (0.7.6)

**Status:** Industrial Demo (Python-Integration)

**Technologie:** BioAI Core (Native C-Kernel) + Python Wrapper

**Zielsetzung:** Demonstration von deterministischen Prozessabläufen (Sequencing) kombiniert mit priorisierten Sicherheits-Reflexen.

---

## 1. Übersicht

Dieses Projekt demonstriert die Steuerung einer 6-stufigen Fertigungsstraße mittels der BioAI Sparse Associative Memory (SAM) Architektur. Das System nutzt eine hybride Logik:

* **Deterministisches Sequencing:** Ein vordefinierter Plan führt die Maschine durch Standard-Schritte (Holen, Scannen, Bohren, Fräsen, Reinigen, Ablegen).
* **Echtzeit-Reflexe:** Ein autonomer Sicherheitsinstinkt (Lichtschranke) überwacht den Prozess und kann die aktive Sequenz bei Gefahr in  unterbrechen.

---

## 2. System-Architektur

### Token-Mapping & Cluster

Die Kommunikation mit dem Kern erfolgt über 64-Bit **TokenIDs**, die strikt in Cluster unterteilt sind:

| Kategorie | Cluster-Maske | Beispiel-Token | Beschreibung |
| --- | --- | --- | --- |
| **Aktionen** | `0x2000...` | `T_DRILL` | Motorische Befehle an die Anlage. |
| **Sensoren** | `0x1000...` | `T_LIGHT_BARRIER` | Eingangsdaten von der Sensorik. |
| **Sicherheit** | `0x4010...` | `T_EMERGENCY_STOP` | Hochpriorisierte Reflexe (Hard Safety). |

---

## 3. Sicherheits-Features

### Deterministische Stabilität (Mode 1)

Nachdem die Sicherheitsregeln injiziert wurden, schaltet das System in den **Production Mode** (`set_mode(1)`). In diesem Zustand ist die neuronale Struktur eingefroren, und der Kern führt keine dynamischen Speicherallokationen mehr durch. Dies garantiert eine konstante Zykluszeit und verhindert Abstürze durch Speicherfragmentierung.

### Schutz geistigen Eigentums (Salting)

Die gelernten Verhaltensgewichte im Arbeitsspeicher sind durch den industriellen Lizenzschlüssel (aus `key.json`) verschleiert. Ein unbefugter Speicher-Dump der Engine liefert ohne den korrekten Key keine nutzbaren Prozessdaten.

---

## 4. Installation & Ausführung

### Voraussetzungen

* **Native Library:** `BioAI_ULTRA.dll` (Windows) oder `libbioai_core.so` (Linux) im Projektpfad.
* **Lizenzschlüssel:** Eine gültige `key.json` mit Ihrem `CUSTOMER_LICENSE_KEY`.
* **Python-Umgebung:** Python 3.10+ mit installierten `ctypes` Modulen.

### Dateistruktur

```text
/project
├── bioai.py                # Python-Wrapper (API Bindings)
├── production_demo.py      # Die Advanced Production Line Logik
├── BioAI_ULTRA.dll         # Native Engine (v0.7.6)
└── config/
    └── key.json            # Sicherheitsschlüssel

```

### Start der Demo

Führen Sie das Hauptskript aus:

```bash
python production_demo.py

```

---

## 5. API-Referenz (Auszug)

* **`load_plan(steps, strict)`**: Lädt eine Aktionsliste in den internen Sequenzer.
* **`update(inputs)`**: Führt einen Denkzyklus aus. Liefert entweder den nächsten Planschritt oder einen feuernden Reflex zurück.
* **`teach(input, action, weight)`**: Injiziert eine statische Regel (z. B. Sicherheits-Instinkt) direkt in das Langzeitgedächtnis.
* **`get_plan_status()`**: Gibt den aktuellen Fortschritt innerhalb der geladenen Sequenz zurück.

---

**BrainAI** *- We don't need **BRUTEFORCE**, we know **Physics** -*</br>
Developed by **Sascha A. Köhne (winemp83)**</br>
Product: **BioAI 0.7.6 (Industrial Closed Feature)**</br>
📧 [koehne83@googlemail.com](mailto:koehne83@googlemail.com)

© 2025 BrainAI / Sascha A. Köhne. All rights reserved.

