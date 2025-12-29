# ☀️ Technisches Referenzhandbuch: BioAI JS Autonomous Solar System (v0.7.6)

In diesem Szenario dient BioAI als autonomer Energieregler für eine Photovoltaikanlage. Das System reagiert in Echtzeit auf schwankende Umweltbedingungen und schützt die physische Integrität des Speichersystems durch unverhandelbare Sicherheits-Reflexe. Durch die neuro-symbolische Verarbeitung erfolgt jede Entscheidung in **garantierter  Zeit**, unabhängig von der Komplexität der gelernten Strategien.

---

## 1. Die Energie-Ontologie (Token-Mapping) 🟦🟥

Die Steuerung basiert auf der Transformation von analogen Sensorwerten in 64-Bit **TokenIDs**, die innerhalb isolierter Cluster verarbeitet werden. In JavaScript werden diese Werte konsequent als `BigInt` (Suffix `n`) behandelt, um die 64-Bit-Präzision des C-Kerns zu erhalten.

### Sensoren (Object Cluster: `0x1000...`)

| TokenID (BigInt) | Bezeichnung | Bedingung / Schwellenwert |
| --- | --- | --- |
| `0x1000000000000001n` | **T_SUN_HIGH** | Lichteinstrahlung > 50.000 Lux. |
| `0x1000000000000002n` | **T_SUN_LOW** | Lichteinstrahlung < 5.000 Lux. |
| `0x1000000000000003n` | **T_BATT_CRITICAL** | Batteriestand unter 10%. |
| `0x1000000000000004n` | **T_BATT_FULL** | Batteriestand über 95%. |

### Strategien & Sicherheit (Action/Reflex Cluster)

| TokenID (BigInt) | Cluster | Auswirkung |
| --- | --- | --- |
| `0x2000000000000001n` | **ACTION** | **PERFORMANCE**: Maximale Systemlast, hohe Abtastrate. |
| `0x2000000000000002n` | **ACTION** | **ECO**: Reduzierter Stromverbrauch, Funkintervalle minimiert. |
| `0x4010000000000001n` | **REFLEX** | **HARD_SHUTDOWN**: Trennung der Last zum Zellschutz. |

---

## 2. Kausale Hierarchie & Reflex-Logik

Das System implementiert eine zwei-stufige Entscheidungslogik innerhalb der `bio_think_logic`, um Sicherheit und Effizienz zu vereinen:

1. **Harter Reflex (Safety First)**: Die Verknüpfung von `T_BATT_CRITICAL` mit `T_HARD_SHUTDOWN` erfolgt mit dem Maximalgewicht von . Da dieser Reflex im Logik-Cluster (`0x4010...`) verankert ist, unterdrückt er mathematisch jede andere geplante Aktion, sobald die Batterie den kritischen Schwellenwert erreicht.
2. **Adaptiver Bias (Efficiency)**: Die Tendenzen für `ECO` oder `PERFORMANCE` sind als "weiche" Instinkte mit geringeren Gewichten ( bis ) hinterlegt. Dies ermöglicht es dem System, durch `feedback()` (Reinforcement Learning) zu lernen, welche Strategie bei welcher Wetterlage die Batterielebensdauer optimal schont.

---

## 3. Implementierungs-Workflow (Node.js)

### Phase 1: Wissensinjektion (Setup)

Während der Initialisierung wird der Kern über `setMode(0)` in den Training-Modus versetzt. Mit der Methode `teach()` werden die fundamentalen Überlebensregeln (Reflexe) und die Basis-Tendenzen injiziert.

### Phase 2: Produktions-Betrieb

Durch den Aufruf von `setMode(1)` wird die neuronale Struktur eingefroren. In diesem Modus findet kein strukturelles Wachstum mehr statt, was folgende Vorteile für industrielle Anwendungen bietet:

* **Deterministik**: Identische Sensor-Inputs führen bei gleichem Wissensstand immer zur identischen Aktion.
* **Null-Allokation**: Es findet keine dynamische Speicherverwaltung im C-Kern mehr statt, was Jitter eliminiert.

### Phase 3: Zyklische Inferenz

In jedem Simulationsschritt werden die aktuellen Sensorwerte aggregiert und als Liste von `BigInt` an `update()` übergeben. Das Ergebnis ist die TokenID der optimalen Handlung, die unmittelbar an die Hardware-Schnittstelle weitergereicht werden kann.

---

## 4. Sicherheit & Ressourcen-Integrität (ISS)

Die Node.js-Integration erfüllt die Anforderungen des **Industrial Sovereign Security** Standards:

* **BigInt-Präzision**: Durch die Nutzung von `BigInt` (Suffix `n`) werden Rundungsfehler vermieden, die bei Standard-Zahlen (64-Bit Floats) die 64-Bit TokenIDs beschädigen könnten.
* **Key-Bindung**: Die gesamte Logik ist mathematisch an die `key.json` gebunden. Ohne den korrekten Schlüssel liefert der Kern aufgrund des "Salting"-Verfahrens keine verwertbaren Ergebnisse.
* **Memory Safety**: Der explizite Aufruf von `shutdown()` (mapping auf `close()`) ist zwingend erforderlich, um den unverwalteten C-Speicher und die Hashtabellen freizugeben.

---

**BrainAI** *- We don't need **BRUTEFORCE**, we know **Physics** -* </br>
Developed by **Sascha A. Köhne (winemp83)**</br>
Product: **BioAI 0.7.6 (Industrial Closed Feature)**</br>
📧 [koehne83@googlemail.com](mailto:koehne83@googlemail.com)

© 2025 BrainAI / Sascha A. Köhne. All rights reserved.