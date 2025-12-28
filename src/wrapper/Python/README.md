# 📘 BioAI Python Integration Guide (v0.7.6)

Der Python-Wrapper ermöglicht eine hochperformante Integration der BioAI-Engine in moderne Datenverarbeitungs-Pipelines. Er nutzt das "Opaque Handle"-Konzept, um die interne Speicherverwaltung des Kerns vor der Python-Runtime zu verbergen und gleichzeitig  Performance zu garantieren.

## 1. Vokabular & Cluster-Konzept 🟦🟥

In BioAI werden alle Informationen als **TokenID** (64-Bit Integer) verarbeitet. Um eine konsistente Logik zu gewährleisten, sollten Ihre Daten den folgenden Clustern zugeordnet werden:

| Cluster | Maske (Hex) | Beschreibung | Beispiel |
| --- | --- | --- | --- |
| **OBJECT** | `0x1000...` | Statische Zustände oder Sensordaten. | `0x1000...001` (Temperatur_Hoch). |
| **ACTION** | `0x2000...` | Aktive Handlungen des Systems. | `0x2000...005` (Kühlung_An). |
| **LOGIC** | `0x4000...` | Regeln; Reflex-Submaske: `0x4010...`. | `0x4010...FF` (Not-Halt_Reflex). |
| **SELF** | `0x5000...` | Interne Zustände des Agenten. | `0x5000...002` (Batterie_Kritisch). |

---

## 2. API-Referenz: `BioBrainInstance`

Die Klasse `BioBrainInstance` fungiert als Context Manager und verwaltet die Kommunikation mit der Tier-spezifischen Bibliothek (z. B. `BioAI_ULTRA.dll`).

### Initialisierung & Lifecycle

* **`__init__(json_path, dll_path)`**:
* Lädt den `customer_key` aus der `key.json` und konvertiert ihn in einen 64-Bit Integer.
* Initialisiert den nativen Kern über `API_CreateBrain`.


* **`close()` / `__exit__**`:
* Ruft `API_FreeBrain` auf, um sämtliche belegte Ressourcen (Neuronen, Synapsen) im C-Kern freizugeben.


* **`set_mode(mode)`**:
* `0`: **Training** – Gehirn lernt aus Interaktionen.
* `1`: **Produktion** – Gehirn ist versiegelt (`fixed_structure`). Keine neuen Speicherallokationen möglich.



### Verarbeitungs-Methoden

* **`update(inputs)`**: Nimmt eine Liste von Integers entgegen und liefert die TokenID der optimalen Aktion zurück.
* **`simulate(inputs, depth)`**: Berechnet zukünftige Konsequenzen über eine definierte Simulationstiefe.
* **`feedback(reward, action)`**: Wendet Belohnung oder Bestrafung auf die gewählte Aktion an.
* **`teach(input_id, action_id, weight)`**: Injiziert Wissen direkt in das Langzeitgedächtnis (LTM). Ein Gewicht von `1.0` erzeugt einen Reflex.

### Persistenz & Inspektion

* **`serialize()`**: Erzeugt einen binären Snapshot des Wissens. Der interne native Puffer wird nach dem Kopieren in Python-Bytes automatisch via `API_FreeBuffer` freigegeben.
* **`inspect(input_id, action_id)`**: Ermöglicht das Auslesen eines Gewichts unter Berücksichtigung des mathematischen Lizenz-Salts.

---

## 3. Sicherheits- & Performance-Richtlinien

### Thread-Sicherheit

Die native Engine ist **nicht thread-safe** pro Instanz. In Python muss der Zugriff auf eine `BioBrainInstance` bei Multi-Threading durch ein `threading.Lock` geschützt werden.

### Weight Obfuscation (Salting)

Jedes Gewicht wird im Speicher durch den individuellen Lizenzschlüssel geschützt, um Reverse Engineering zu verhindern:


### Ressourcen-Management

Nutzen Sie bevorzugt das `with`-Statement. Dies garantiert den Aufruf von `API_FreeBrain` auch im Falle einer Python-Exception und verhindert Memory Leaks im unverwalteten C-Speicher.

---

**BrainAI** *- We don't need **BRUTEFORCE**, we know **Physics** -*</br>
Developed by **Sascha A. Köhne (winemp83)**</br>
Product: **BioAI 0.7.6 (Industrial Closed Feature)**</br>
📧 [koehne83@googlemail.com](mailto:koehne83@googlemail.com)

© 2025 BrainAI / Sascha A. Köhne. All rights reserved.