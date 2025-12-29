# 🤖 Technisches Handbuch: BioAI JS Robot Navigation (v0.7.6)

Diese Dokumentation beschreibt die Implementierung einer autonomen Navigationslogik für mobile Agenten unter Verwendung des BioAI-Kerns in Node.js. Der Fokus liegt auf der Synergie zwischen starren Sicherheits-Instinkten und adaptivem Echtzeit-Lernen durch Reinforcement.

---

## 1. Entscheidungs-Hierarchie & Safety Layer

Das System löst Navigationskonflikte durch eine mathematische Gewichtung, die in zwei Hauptstufen unterteilt ist:

1. **Reflex-Ebene (Angeboren)**: Über `teach()` wird der Reflex `WALL_AHEAD -> TURN_LEFT` mit einem Gewicht von  injiziert. Da dies das mathematische Maximum darstellt, wird jede andere gelernte Aktion bei Wandkontakt unmittelbar unterdrückt, um die physische Integrität zu wahren.
2. **Lernebene (Erworben)**: Über `feedback()` passt der Kern die Synapsenstärke zwischen `SENSOR_FREE` und `ACT_MOVE` dynamisch an. Der Roboter lernt eigenständig, dass Vorwärtsbewegung bei freier Bahn zu einem positiven Belohnungssignal führt, während unnötiges Drehen bestraft wird.

---

## 2. Ontologie (Token-Mapping) 🟦🟥

BioAI nutzt ein 64-Bit Adressierungsschema, um Sensordaten und Aktionen in Clustern zu isolieren und "Crosstalk" zu vermeiden.

| Domäne | TokenID (Hex/BigInt) | Cluster | Funktion |
| --- | --- | --- | --- |
| **SENSOR_FREE** | `0x1000...0001n` | **OBJECT** | Wahrnehmung einer freien Fahrtrichtung. |
| **SENSOR_WALL** | `0x1000...0002n` | **OBJECT** | Detektion eines Hindernisses im Nahbereich. |
| **ACT_MOVE** | `0x2000...0001n` | **ACTION** | Primärer Antriebs-Vektor (Vorwärts). |
| **ACT_TURN** | `0x2000...0002n` | **ACTION** | Rotationsmanöver zur Kurskorrektur. |

---

## 3. Mathematische Validierung (De-Salting)

Zur Sicherung des geistigen Eigentums (IP) werden Gewichte im Speicher der Engine "gesalzen". Beim Aufruf von `inspect()` wird der korrekte `customer_key` aus der `key.json` verwendet, um den realen Lernfortschritt für den Entwickler oder Auditor lesbar zu machen. Eine hohe Synapsenstärke (z. B. ) zeigt eine gefestigte neuronale Verbindung zwischen einem Reiz und einer erfolgreichen Reaktion an.

---

## 4. Persistenz & Betriebssicherheit (ISS)

* **Garantiertes **: Die Antwortzeit des Roboters pro Schritt bleibt unabhängig von der Menge der gesammelten Erfahrungen immer identisch.
* **Deterministik**: Durch die Initialisierung mit dem Schlüssel aus der `key.json` verhält sich das System bei identischen Reiz-Reaktions-Mustern zu 100% reproduzierbar.
* **Persistenz**: Über `serialize()` kann der gesamte Wissensstand (LTM) in einen binären Snapshot exportiert werden, um ihn auf andere Einheiten der gleichen Serie zu übertragen.

---

**BrainAI** *- We don't need **BRUTEFORCE**, we know **Physics** -*</br>
Developed by **Sascha A. Köhne (winemp83)**</br>
Product: **BioAI 0.7.6 (Industrial Closed Feature)**</br>
📧 [koehne83@googlemail.com](mailto:koehne83@googlemail.com)

© 2025 BrainAI / Sascha A. Köhne. All rights reserved.