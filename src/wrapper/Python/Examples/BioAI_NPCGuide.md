# 🎮 BioAI Technical Manual: Autonomous NPC Intelligence (0.7.6)

Dieses Handbuch beschreibt die Architektur und Implementierung von **BioNpcEntity**, einer neuro-symbolischen Entität, die für Echtzeit-Entscheidungen in komplexen Simulationsumgebungen optimiert ist. Im Gegensatz zu klassischen Verhaltensbäumen (Behavior Trees) nutzt BioAI eine mathematische Kausalitäts-Matrix, um Handlungen in **garantierter  Zeit** zu berechnen.

---

## 1. Die 3-Ebenen Entscheidungs-Hierarchie

Der NPC operiert auf drei diskreten logischen Ebenen, die im BioAI-Kern hierarchisch verarbeitet werden, um Regelkonflikte deterministisch aufzulösen:

1. **Die Reflex-Ebene (Instinkt)**: Unverhandelbare Überlebensregeln. Diese werden über die Methode `teach()` mit einem Gewicht von `1.0` injiziert. Sobald ein Reflex-Input (z. B. `T_HP_LOW`) erkannt wird, bricht der Kern alle anderen Planungen ab und führt die Schutzreaktion aus.
2. **Die Bias-Ebene (Kontext)**: Kontextabhängige Tendenzen (z. B. "Vorsicht bei Nacht"). Diese werden mit geringeren Gewichten (z. B. `0.4`) definiert und dienen als statistisches "Hintergrundrauschen", das die KI in eine bestimmte Richtung lenkt, ohne sie zu zwingen.
3. **Die Erfahrungs-Ebene (Lernen)**: Das dynamische Gedächtnis. Über die Methode `feedback()` lernt der NPC im laufenden Betrieb, welche Aktionen (z. B. Handel oder Angriff) in der aktuellen Situation zum Erfolg führen.

---

## 2. Ontologie & Token-Definition (Vocab-Dump) 🟦🟥

BioAI verarbeitet keine Strings oder Objekte, sondern 64-Bit **TokenIDs**, die strikt nach Clustern maskiert sind, um die Hardware-Adressierung zu optimieren.

### Wahrnehmung (Inputs)

| Token | Hex-ID (Beispiel) | Cluster | Beschreibung |
| --- | --- | --- | --- |
| `T_SEE_PLAYER` | `0x1000...0001` | **OBJECT** | Visuelle Detektion eines Spielers. |
| `T_SEE_GOLD` | `0x1000...0002` | **OBJECT** | Visuelle Detektion von Ressourcen. |
| `T_HP_LOW` | `0x5000...0001` | **SELF/NEED** | Interner Status: Kritische Gesundheit. |
| `T_IS_NIGHT` | `0x3000...0001` | **TIME** | Zeitlicher Kontext: Nachtzyklus. |

### Reaktionen (Actions)

| Token | Hex-ID (Beispiel) | Cluster | Beschreibung |
| --- | --- | --- | --- |
| `T_ACTION_ATTACK` | `0x2000...0001` | **ACTION** | Aktive Aggressions-Handlung. |
| `T_ACTION_FLEE` | `0x2000...0002` | **ACTION** | Rückzugsbewegung. |
| `T_REFLEX_HEAL` | `0x4010...0001` | **LOGIC/REFLEX** | Priorisierter Heilvorgang (Interrupt). |

---

## 3. Implementierungs-Logik (Python Workflow)

### Initialisierung & Personality Injection

Während der Initialisierung wird der Kern in den `set_mode(0)` (Training) versetzt. Hier findet die "Erziehung" des NPCs statt. Durch `teach()` werden die neuronalen Pfade für Instinkte und Biases fest im Langzeitgedächtnis (LTM) verankert. Danach wird das Gehirn über `set_mode(1)` (Production) versiegelt, um Speicherfluktuationen während der Spielschleife zu verhindern.

### Der Update-Zyklus (Sense -> Think -> Act)

1. **Sense**: Die Simulationsumgebung sammelt Booleans (z. B. `is_night`) und mappt diese auf die definierten TokenIDs.
2. **Think**: Die Methode `update(inputs)` sendet die Liste der Tokens an den C-Kern. Die Engine berechnet in einem einzigen Durchlauf ohne Backpropagation die Aktion mit dem höchsten assoziativen Gewicht.
3. **Act & Learn**: Das Ergebnis wird ausgeführt. Falls der NPC beispielsweise angreift und Schaden erleidet, wird über `feedback(-0.5, T_ACTION_ATTACK)` die Verbindung zwischen der aktuellen Situation und dieser Aktion geschwächt.

---

## 4. Performance & Speichersicherheit (ISS)

* **Deterministik**: Identische Inputs führen bei gleichem Trainingsstand immer zur exakt gleichen Aktion. Dies ermöglicht eine perfekte Synchronisation in Multiplayer-Umgebungen.
* **Memory Safety**: Der Python-Wrapper nutzt den `close()` Mechanismus, um das native `BioBrain`-Handle im C-Speicher freizugeben. Dies verhindert "Memory Leaks", die bei NPCs in Langzeit-Simulationen oft kritisch sind.
* **Sovereign Security**: Jedes NPC-Gehirn ist mathematisch an die `key.json` gebunden. Ein NPC mit "Schlüssel A" kann keine Erfahrungen oder Modelle von "Schlüssel B" laden.

---

**BrainAI** *- We don't need **BRUTEFORCE**, we know **Physics** -*</br>
Developed by **Sascha A. Köhne (winemp83)**</br>
Product: **BioAI 0.7.6 (Industrial Closed Feature)**</br>
📧 [koehne83@googlemail.com](mailto:koehne83@googlemail.com)

© 2025 BrainAI / Sascha A. Köhne. All rights reserved.