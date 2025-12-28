# 📘 BioAI JavaScript (Node.js) Guide (v0.7.6)

Dieser Guide beschreibt die Nutzung des Node.js-Wrappers für die BioAI-Engine. Alle Zeitangaben und Verarbeitungen erfolgen in konstanter Zeit , was eine hohe Skalierbarkeit für Echtzeitanwendungen ermöglicht.

## 1. Vokabular & Cluster-Konzept 🟦🟥

In JavaScript werden TokenIDs als `BigInt` (z. B. `0x1000...n`) verarbeitet. Die Engine nutzt Cluster-Masken zur logischen Trennung:

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

Die Klasse verwaltet ein natives Handle auf den BioAI-Kern und schützt das System vor unbefugtem Zugriff durch mathematisches Salting.

### Initialisierung & Lifecycle

* **`new BioBrainInstance(jsonPath, dllPath)`**: Lädt den ISS-Key aus der `key.json` und initialisiert den nativen C-Kern.
* **`setMode(mode)`**:
* `0`: **Training** (Plastischer Modus).
* `1`: **Produktion** (Versiegelt). Blockiert weitere Speicherallokationen für maximale Echtzeitstabilität.


* **`close()`**: Zerstört das Gehirn im C-Kern und gibt allen Speicher (LTM, STM, Hashtabellen) frei.

### Kern-Operationen

* **`update(inputs)`**: Verarbeitet Wahrnehmungen und liefert die TokenID der optimalen Aktion.
* **`simulate(inputs, depth)`**: Berechnet zukünftige Kausalitäten über die definierte Tiefe.
* **`feedback(reward, action)`**: Wendet Belohnung oder Bestrafung an, um Verhaltensgewichte anzupassen.

---

## 3. Sicherheits- & Performance-Hinweise

1. **Memory Management**: Node.js verwaltet den JavaScript-Speicher automatisch, aber der **C-Kern nicht**. Rufen Sie immer `close()` auf, wenn eine Instanz nicht mehr benötigt wird, um Speicherlecks zu vermeiden.
2. **64-Bit Integers**: Verwenden Sie in JavaScript immer das Suffix `n` (z. B. `0x2000...n`) für TokenIDs, um Präzisionsverluste zu vermeiden, die bei standardmäßigen 64-Bit-Floats auftreten würden.
3. **Key-Integrität**: Der in `key.json` gespeicherte Schlüssel ist essentiell für die De-Serialisierung. Ohne diesen Schlüssel können trainierte Modelle nicht korrekt geladen werden.

---

**BrainAI** *- We don't need **BRUTEFORCE**, we know **Physics** -*</br>
Developed by **Sascha A. Köhne (winemp83)**</br>
Product: **BioAI 0.7.6 (Industrial Closed Feature)**</br>
📧 [koehne83@googlemail.com](mailto:koehne83@googlemail.com)

© 2025 BrainAI / Sascha A. Köhne. All rights reserved.
