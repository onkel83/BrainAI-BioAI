# 📘 BioAI JavaScript (Node.js) Guide (v0.7.6)

Dieser Guide beschreibt die Nutzung des Node.js-Wrappers für die BioAI-Engine. Alle Zeitangaben und Verarbeitungen erfolgen in konstanter Zeit , was eine hohe Skalierbarkeit für Echtzeitanwendungen ermöglicht.

## 1. Vokabular & Cluster-Konzept 🟦🟥

In JavaScript werden TokenIDs als `BigInt` (z. B. `0x1000...n`) verarbeitet. Die Engine nutzt Cluster-Masken zur logischen Trennung der neuronalen Domänen:

| Cluster | Maske (Hex) | Bedeutung | Beispiel |
| --- | --- | --- | --- |
| **OBJECT** | `0x1000...` | **Zustand / Objekt** | Sensordaten, Kameradetektionen, Status-Flags. |
| **ACTION** | `0x2000...` | **Handlung** | Steuersignale, Methodenaufrufe, physische Bewegungen. |
| **TIME** | `0x3000...` | **Zeitlicher Kontext** | Timer, Zyklen, Sequenz-Abstände. |
| **LOGIC** | `0x4000...` | **Regelwerk** | Logische Verknüpfungen und statische Abläufe. |
| **SELF** | `0x5000...` | **Eigenzustand** | Akkuladung, Systemgesundheit, Zielvorgaben. |

> **Sicherheitshinweis:** Ein **Reflex** (höchste Priorität) wird über die Sub-Maske `0x4010...` definiert. Ein solcher Reflex überschreibt bei einem Gewicht von  sofort jede gelernte Erfahrung.

---

## 2. API-Referenz: `BioBrainInstance`

Die Klasse verwaltet ein natives Handle auf den BioAI-Kern und bietet nun vollen Zugriff auf den industriellen Sequenzer.

### Initialisierung & Lifecycle

* **`new BioBrainInstance(jsonPath, dllPath)`**: Lädt den ISS-Key aus der `key.json` und initialisiert den nativen C-Kern.
* **`setMode(mode)`**:
* `0`: **Training** (Plastischer Modus).
* `1`: **Produktion** (Versiegelt). Blockiert weitere Speicherallokationen für maximale Echtzeitstabilität.


* **`deserialize(data)`**: Rekonstruiert ein Gehirn aus einem zuvor gespeicherten Byte-Stream (Buffer).
* **`close()`**: Zerstört das Gehirn im C-Kern und gibt allen nativen Speicher frei.

### Kern-Operationen (Inferenz & Lernen)

* **`update(inputs)`**: Verarbeitet Wahrnehmungen und liefert die TokenID der optimalen Aktion in .
* **`simulate(inputs, depth)`**: Berechnet zukünftige Kausalitäten (Imagination) über die definierte Tiefe.
* **`feedback(reward, action)`**: Wendet Reinforcement Learning an, um Verhaltensgewichte anzupassen.
* **`teach(input, action, weight)`**: Injiziert eine harte Regel (Reflex) direkt in das Langzeitgedächtnis.
* **`inspect(input, action)`**: Liest ein gelerntes Gewicht unter Anwendung des De-Salting aus.

### Sequenzer & Plan-Steuerung (Neu)

Diese Funktionen ermöglichen die Steuerung exakter Fertigungsabläufe:

* **`loadPlan(steps, strict)`**: Lädt eine Liste von TokenIDs (`steps`) als feste Sequenz. `strict=true` erzwingt die exakte Reihenfolge.
* **`getPlanStatus()`**: Gibt den Index des aktuellen Plan-Schritts zurück. Liefert `-1`, wenn kein Plan aktiv ist.
* **`abortPlan()`**: Bricht die aktuelle Plan-Ausführung sofort ab.

### Persistenz

* **`serialize()`**: Erzeugt einen binären Snapshot des aktuellen Wissensstands als Node.js `Buffer`.

---

## 3. Sicherheits- & Performance-Hinweise

1. **Memory Management**: Node.js verwaltet den JavaScript-Speicher automatisch, aber den **C-Kern nicht**. Rufen Sie immer `close()` auf, um Speicherlecks im unverwalteten RAM zu vermeiden.
2. **64-Bit Integers**: Verwenden Sie in JavaScript immer das Suffix `n` (z. B. `0x2000...n`) für TokenIDs, um Präzisionsverluste zu vermeiden.
3. **Key-Integrität**: Der Schlüssel in `key.json` ist für die De-Serialisierung zwingend erforderlich. Ohne den korrekten Schlüssel sind Snapshots mathematisch unbrauchbar.

---

**BrainAI** *- We don't need **BRUTEFORCE**, we know **Physics** -* </br>
Developed by **Sascha A. Köhne (winemp83)**</br>
Product: **BioAI 0.7.6 (Industrial Closed Feature)**</br>
📧 [koehne83@googlemail.com](mailto:koehne83@googlemail.com)

© 2025 BrainAI / Sascha A. Köhne. All rights reserved.