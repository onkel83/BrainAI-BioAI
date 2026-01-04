# 🎮 Technisches Handbuch: BioAI JS NPC Intelligence (v0.7.6)

Dieses Handbuch beschreibt die Architektur und Implementierung von autonomen, neuro-symbolischen NPCs in Node.js. BioAI nutzt eine mathematische Kausalitäts-Matrix, um Handlungen in **garantierter  Zeit** zu berechnen, anstatt rechenintensive Behavior Trees zu verwenden.

---

## 1. Die 3-Ebenen Entscheidungs-Hierarchie

Der NPC operiert auf drei logischen Ebenen, die im Kern hierarchisch verarbeitet werden:

1. **Die Reflex-Ebene (Instinkt)**: Unverhandelbare Überlebensregeln. Diese werden via `teach()` mit Gewicht `1.0` injiziert. Erkennt der Kern einen Reflex-Input (z. B. `T_HP_LOW`), bricht er andere Planungen ab, um die Schutzreaktion auszuführen.
2. **Die Bias-Ebene (Kontext)**: Kontextabhängige Tendenzen (z. B. "Nachtvorsicht"). Durch geringere Gewichte (z. B. `0.4`) wird die KI in eine Richtung gelenkt, ohne deterministische Regeln zu erzwingen.
3. **Die Erfahrungs-Ebene (Lernen)**: Das dynamische Gedächtnis. Über `feedback()` passt der NPC im laufenden Betrieb seine Verhaltensgewichte an, um aus Erfolg und Schmerz zu lernen.

---

## 2. Ontologie & Token-Definition (Vocab-Dump) 🟦🟥

BioAI verarbeitet keine Strings, sondern 64-Bit **TokenIDs**, die strikt nach Clustern maskiert sind.

### Wahrnehmung (Inputs)

| Token | Hex-ID (BigInt) | Cluster | Beschreibung |
| --- | --- | --- | --- |
| `T_SEE_PLAYER` | `0x1000...0001n` | **OBJECT** | Visuelle Detektion eines Spielers. |
| `T_SEE_GOLD` | `0x1000...0002n` | **OBJECT** | Detektion von Ressourcen. |
| `T_HP_LOW` | `0x5000...0001n` | **SELF/NEED** | Kritische Gesundheit (interner Status). |
| `T_IS_NIGHT` | `0x3000...0001n` | **TIME** | Zeitlicher Kontext. |

### Reaktionen (Actions)

| Token | Hex-ID (BigInt) | Cluster | Beschreibung |
| --- | --- | --- | --- |
| `T_ACTION_ATTACK` | `0x2000...0001n` | **ACTION** | Aktive Aggressions-Handlung. |
| `T_REFLEX_HEAL` | `0x4010...0001n` | **REFLEX** | Priorisierter Heilvorgang (Interrupt). |

---

## 3. Performance & Speichersicherheit (ISS)

* **Deterministik**: Identische Inputs führen bei gleichem Trainingsstand immer zur identischen Aktion, was die Synchronisation in Multiplayer-Umgebungen vereinfacht.
* **Memory Safety**: Da Node.js den unverwalteten C-Speicher nicht automatisch bereinigt, ist der Aufruf von `close()` zwingend erforderlich, um Speicherlecks zu verhindern.
* **Sovereign Security**: Jedes NPC-Gehirn ist an die `key.json` gebunden; ohne diesen mathematischen Anker können keine gelernten Modelle geladen werden.

---

**BrainAI** *- We don't need **BRUTEFORCE**, we know **Physics** -*</br>
Developed by **Sascha A. Köhne (winemp83)**</br>
Product: **BioAI 0.7.6 (Industrial Closed Feature)**</br>
📧 [koehne83@googlemail.com](mailto:koehne83@googlemail.com)

© 2025 BrainAI / Sascha A. Köhne. All rights reserved.