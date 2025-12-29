# 📘 BioAI Technical Manual: Autonomous Robot Navigation (0.7.6)

Diese Dokumentation beschreibt die Implementierung einer autonomen Navigationslogik unter Verwendung des BioAI-Kerns. Im Fokus steht die Kombination aus unveränderlichen Sicherheits-Instinkten und dynamischer Erfahrungs-Optimierung in Echtzeit.

## 1. Entscheidungs-Hierarchie & Safety Layer

Das System löst Navigationskonflikte durch eine mathematische Gewichtung, die in vier Stufen unterteilt ist:

1. **Reflex-Ebene**: Über `teach()` wird der Reflex `WALL_AHEAD -> TURN_LEFT` mit einem Gewicht von  injiziert. Da dies das mathematische Maximum für Inferenz-Traces darstellt, wird jede andere gelernte Aktion bei Wandkontakt unterdrückt.
2. **Lernebene (Reinforcement)**: Über `feedback()` passt der Kern die Synapsen zwischen `PATH_FREE` und `MOVE_FWD` an. Die KI lernt, dass Vorwärtsbewegung bei freier Bahn zu einem positiven Belohnungssignal führt.

## 2. Ontologie (Token-Mapping) 🟦🟥

BioAI nutzt ein 64-Bit Adressierungsschema, um Sensordaten und Aktionen in Clustern zu isolieren. Dies verhindert "Crosstalk" zwischen verschiedenen logischen Domänen.

| Domäne | TokenID (Hex) | Cluster | Funktion |
| --- | --- | --- | --- |
| **SENSOR_FREE** | `0x1000...0001` | **OBJECT** | Wahrnehmung einer freien Flug-/Fahrbahn. |
| **SENSOR_WALL** | `0x1000...0002` | **OBJECT** | Detektion eines Hindernisses im Nahbereich. |
| **ACT_MOVE** | `0x2000...0001` | **ACTION** | Primärer Antriebsvektor. |
| **ACT_TURN** | `0x2000...0002` | **ACTION** | Rotationsmanöver zur Kurskorrektur. |

## 3. Mathematische Validierung (De-Salting)

Wenn die Methode `inspect()` aufgerufen wird, greift der Kern auf die internen Gewichte zu. Da BioAI zum Schutz des geistigen Eigentums (IP) Gewichte im Speicher "salzt", wird für die Rückgabe der korrekte `customer_key` aus der `key.json` verwendet, um den realen Lernfortschritt anzuzeigen:

## 4. Persistenz & Speichersicherheit (ISS)

* **Garantiertes O(1)**: Unabhängig von der Komplexität der gelernten Pfade bleibt die Antwortzeit des Roboters pro Schritt konstant.
* **Deterministik**: Durch die Initialisierung mit dem Key aus der `key.json` verhält sich der Roboter bei identischen Belohnungsmustern zu 100% reproduzierbar.
* **Ressourcen-Management**: Der Python-Wrapper stellt sicher, dass beim Aufruf von `close()` das native `BioBrain`-Handle und alle damit verbundenen Hashtabellen im C-Speicher gelöscht werden.

---

**BrainAI** *- We don't need **BRUTEFORCE**, we know **Physics** -*</br>
Developed by **Sascha A. Köhne (winemp83)**</br>
Product: **BioAI 0.7.6 (Industrial Closed Feature)**</br>
📧 [koehne83@googlemail.com](mailto:koehne83@googlemail.com)

© 2025 BrainAI / Sascha A. Köhne. All rights reserved.