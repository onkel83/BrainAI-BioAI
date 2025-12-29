# 🏭 Technisches Referenzhandbuch: BioAI JS Advanced Production (v0.7.6)

In diesem Szenario fungiert BioAI als deterministische Logikschicht, die eine 6-stufige Fertigungssequenz steuert. Das System kombiniert die Flexibilität eines programmierbaren Sequenzers mit der Unverhandelbarkeit von Sicherheits-Reflexen, die in **garantierter  Zeit** verarbeitet werden.

---

## 1. Die Prozess-Ontologie (Token-Management) 🟦🟥

Die Steuerung basiert auf der Transformation physikalischer Zustände in 64-Bit **TokenIDs**, die innerhalb isolierter Cluster verarbeitet werden.

### Fertigungs-Schritte (Action Cluster: `0x2000...`)

| TokenID (Hex/BigInt) | Bezeichnung | Funktion |
| --- | --- | --- |
| `0x2000000000000001n` | **T_PICK_PART** | Ansteuerung des Greifers zur Materialaufnahme. |
| `0x2000000000000002n` | **T_SCAN_QR** | Trigger für die optische Qualitätssicherung. |
| `0x2000000000000003n` | **T_DRILL** | Aktivierung der Bohreinheit. |
| `0x2000000000000004n` | **T_MILL** | Start des Fräsvorgangs. |
| `0x2000000000000005n` | **T_CLEAN** | Druckluftreinigung des Werkstücks. |
| `0x2000000000000006n` | **T_PLACE_DONE** | Ablage des fertigen Teils im Ausgangskorb. |

### Sicherheitssensorik (Object/Reflex Cluster)

| TokenID (Hex/BigInt) | Cluster | Verwendung |
| --- | --- | --- |
| `0x10000000000000A1n` | **OBJECT** | **T_SENSOR_LIGHT_BARRIER**: Signal einer Lichtschranke. |
| `0x4010000000000099n` | **REFLEX** | **T_EMERGENCY_STOP**: Unmittelbare Stillsetzung der Anlage. |

---

## 2. Die logische Entscheidungshierarchie (Priority Stack)

Das System verhindert logische Konflikte durch eine strikte Priorisierung innerhalb der `bio_think_logic`:

1. **Reflex-Ebene (Interrupt)**: Der Reflex `SENSOR -> EMERGENCY_STOP` wird mit einem Gewicht von  injiziert. Da Reflexe im `0x4010` Cluster liegen, besitzen sie mathematischen Vorrang vor jeder geplanten Handlung. Sobald die Lichtschranke unterbrochen wird, unterdrückt der Kern sofort den nächsten geplanten Produktionsschritt.
2. **Plan-Ebene (Sequenzer)**: Im Normalbetrieb folgt die KI dem über `loadPlan` geladenen Pfad. Der Sequenzer stellt sicher, dass Schritte nur in der korrekten Reihenfolge ausgeführt werden ("Strict Mode").

---

## 3. Implementierungs-Workflow & API-Nutzung

### Phase 1: Wissensinjektion (Setup)

Während der Initialisierung wird der Kern in den `setMode(0)` (Training) versetzt. Hier findet die "Erziehung" der Anlage statt. Durch `teach()` werden die Sicherheits-Instinkte fest im Langzeitgedächtnis (LTM) verankert.

### Phase 2: Deterministischer Betrieb (Production)

Durch den Wechsel in `setMode(1)` (Production) wird die neuronale Struktur eingefroren. Dies garantiert:

* **Echtzeit-Stabilität**: Die Inferenzzeit für `update()` bleibt für jeden Schritt konstant.
* **Keine Allokation**: Es finden keine neuen Speicherzuweisungen im C-Kern statt, was Jitter und Speicher-Leaks eliminiert.

### Phase 3: Sequenz-Management

Der Sequenzer wird über drei Kernfunktionen gesteuert:

* `loadPlan(steps, strict)`: Initialisiert die 6-stufige Kette.
* `getPlanStatus()`: Überwacht den aktuellen Fortschritt im Node.js Event-Loop.
* `abortPlan()`: Wird beim Eintreten eines Reflexes aufgerufen, um die Sequenz sicher zu beenden.

---

## 4. Sicherheit & Ressourcen-Integrität (ISS)

Die Node.js-Integration erfüllt die Anforderungen des **Industrial Sovereign Security** Standards:

* **Key-Bindung**: Die gesamte Produktionslogik ist mathematisch an die `key.json` gebunden. Ein unbefugtes Auslesen der neuronalen Gewichte ist ohne den korrekten Key aufgrund des "Salting"-Verfahrens physikalisch unmöglich.
* **Memory Safety**: Da JavaScript-Objekte keinen Einfluss auf den nativen C-Speicher haben, ist der explizite Aufruf von `close()` (mapping auf `API_FreeBrain`) zwingend erforderlich, um die native Hashtabelle und Neuronen-Traces freizugeben.
* **BigInt-Präzision**: Durch die konsequente Nutzung von `BigInt` (Suffix `n`) werden Rundungsfehler vermieden, die bei Standard-JavaScript-Zahlen (64-Bit Floats) die 64-Bit TokenIDs beschädigen könnten.

---

**BrainAI** *- We don't need **BRUTEFORCE**, we know **Physics** -* </br>
Developed by **Sascha A. Köhne (winemp83)**</br>
Product: **BioAI 0.7.6 (Industrial Closed Feature)**</br>
📧 [koehne83@googlemail.com](mailto:koehne83@googlemail.com)

© 2025 BrainAI / Sascha A. Köhne. All rights reserved.