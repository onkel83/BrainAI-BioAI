# 📘 BioAI Python Integration Guide (v0.7.6)

Der Python-Wrapper ermöglicht eine hochperformante Integration der BioAI-Engine in moderne Datenverarbeitungs-Pipelines. Er nutzt das „Opaque Handle“-Konzept, um die interne Speicherverwaltung des Kerns vor der Python-Runtime zu verbergen und gleichzeitig maximale Performance zu garantieren.

## 1. Vokabular & Cluster-Konzept 🟦🟥

In BioAI werden alle Informationen als **TokenID** (64-Bit Integer) verarbeitet. Um eine konsistente Logik zu gewährleisten, müssen Daten den folgenden Clustern zugeordnet werden:

| Cluster | Maske (Hex) | Beschreibung | Beispiel |
| --- | --- | --- | --- |
| **OBJECT** | `0x1000...` | Statische Zustände oder Sensordaten. | `0x1000...001` (Temperatur_Hoch) |
| **ACTION** | `0x2000...` | Aktive Handlungen des Systems. | `0x2000...005` (Kühlung_An) |
| **TIME** | `0x3000...` | Zeitliche Informationen oder Intervalle. | `0x3000...00A` (100ms_Zyklus) |
| **LOGIC** | `0x4000...` | Logische Verknüpfungen und Regeln. | `0x4000...002` (UND_Gatter) |
| **SELF** | `0x5000...` | Interne Zustände des Agenten. | `0x5000...002` (Batterie_Kritisch) |

> **Wichtig:** Ein **Reflex** (unbrechbare Regel) nutzt die Sub-Maske `0x4010...`. Ein Token mit dieser Maske und einem Gewicht von  unterbricht sofort alle anderen Denkprozesse oder aktiven Pläne.

---

## 2. API-Referenz: `BioBrainInstance`

Die Klasse `BioBrainInstance` fungiert als Context Manager und verwaltet die Kommunikation mit der Tier-spezifischen Bibliothek (z. B. `BioAI_ULTRA.dll`).

### Initialisierung & Lifecycle

* **`__init__(json_path, dll_path)`**:
* Lädt den `customer_key` aus der `key.json` und konvertiert ihn in einen 64-Bit Integer.
* Initialisiert den nativen Kern über `API_CreateBrain`.


* **`close()` / `__exit__**`:
* Ruft `API_FreeBrain` auf, um sämtliche Ressourcen (Neuronen, Synapsen) im C-Kern freizugeben.


* **`set_mode(mode)`**:
* `0`: **Training** – Das Gehirn lernt aus Interaktionen und bildet neue Synapsen.
* `1`: **Produktion** – Das Gehirn ist versiegelt (`fixed_structure`). Es finden keine neuen Speicherallokationen statt, was deterministisches Echtzeitverhalten garantiert.



### Verarbeitungs-Methoden

* **`update(inputs)`**: Verarbeitet eine Liste von TokenIDs und liefert die ID der optimalen Aktion in  zurück.
* **`simulate(inputs, depth)`**: Berechnet die Kausalitätskette über eine definierte Tiefe (Imagination).
* **`feedback(reward, action)`**: Wendet Reinforcement Learning auf die gewählte Aktion an.
* **`teach(input_id, action_id, weight)`**: Injiziert Wissen direkt in das Langzeitgedächtnis (LTM). Ein Gewicht von `1.0` erzeugt einen Reflex.

### Sequenzer (Planung)

* **`load_plan(steps, strict)`**: Lädt eine feste Aktionssequenz in den internen Sequenzer.
* **`abort_plan()`**: Bricht die aktuelle Plan-Ausführung sofort ab.
* **`get_plan_status()`**: Gibt den Index des aktuellen Plan-Schritts zurück (-1 wenn kein Plan aktiv ist).

### Persistenz & Inspektion

* **`serialize()`**: Erzeugt einen binären Snapshot des Wissens.
* **`deserialize(data)`**: Rekonstruiert ein Brain-Objekt aus einem binären Byte-Stream.
* **`inspect(input_id, action_id)`**: Liest ein gelerntes Gewicht unter Anwendung des De-Salting aus.

---

## 3. Sicherheits- & Performance-Richtlinien

### Thread-Sicherheit

Die native Engine ist **nicht thread-safe** pro Instanz. In Python muss der Zugriff bei Multi-Threading durch ein `threading.Lock` geschützt werden.

### Weight Obfuscation (Salting)

Jedes Gewicht wird im Speicher durch den individuellen Lizenzschlüssel geschützt. Der Salt-Faktor berechnet sich intern als `1.0f + ((key % 97) * 0.001f)`. Dies verhindert das einfache Auslesen von Prozessgeheimnissen über Speicher-Dumps.

### Ressourcen-Management

Nutzen Sie bevorzugt das `with`-Statement. Dies garantiert den Aufruf von `API_FreeBrain` auch im Falle einer Exception und verhindert Memory Leaks im unverwalteten C-Speicher.

---

**BrainAI** *- We don't need **BRUTEFORCE**, we know **Physics** -* </br>
Developed by **Sascha A. Köhne (winemp83)**</br>
Product: **BioAI 0.7.6 (Industrial Closed Feature)**</br>
📧 [koehne83@googlemail.com](mailto:koehne83@googlemail.com)

© 2025 BrainAI / Sascha A. Köhne. All rights reserved.