# 🧬 Technisches Handbuch: BioAI Survival Agent Simulation (v0.7.6)

Die Survival Agent Simulation demonstriert die Fähigkeit von BioAI, biologisch inspirierte Entscheidungsprozesse in einer deterministischen, industriellen Umgebung abzubilden. Der Agent agiert als autonomes System, das interne Bedürfnisse (Hunger, Energie) gegen externe Reize (Nahrungssichtung) abwägt und dabei stets die übergeordneten Sicherheits-Reflexe priorisiert.

---

## 1. Die Überlebens-Ontologie (Vocab-Dump) 🟦🟥

Der Agent verarbeitet Zustände als 64-Bit **TokenIDs**, die strikt nach Clustern getrennt sind, um eine fehlerfreie Adressierung im Langzeitgedächtnis (LTM) zu gewährleisten.

### Interne Zustände & Bedürfnisse (Need/Status Cluster)

| TokenID (Hex) | Bezeichnung | Cluster | Bedeutung |
| --- | --- | --- | --- |
| `0x5000...0001` | **T_HUNGRY** | **NEED** | Interner Reiz: Energiedefizit detektiert. |
| `0x5000...0002` | **T_TIRED** | **NEED** | Interner Reiz: Ruhebedarf detektiert. |
| `0x5000...0003` | **T_HEALTH_LOW** | **STATUS** | Kritischer Systemzustand (Schwellenwert erreicht). |

### Umwelt & Handlungen (Object/Action Cluster)

| TokenID (Hex) | Bezeichnung | Cluster | Funktion |
| --- | --- | --- | --- |
| `0x1000...0001` | **T_FOOD_SEE** | **OBJECT** | Externer Reiz: Ressource in Sensorenreichweite. |
| `0x2000...0001` | **T_SEARCH** | **ACTION** | Aktive Exploration der Umgebung. |
| `0x2000...0002` | **T_EAT** | **ACTION** | Versuch der Ressourcenaufnahme. |
| `0x2000...0003` | **T_SLEEP** | **ACTION** | Regenerationsphase. |

---

## 2. Die Hierarchische Entscheidungs-Matrix

Das Überleben des Agenten wird durch den **Priority Stack** des BioAI-Kerns gesichert. Konflikte zwischen Hunger und Erschöpfung werden mathematisch aufgelöst:

### Ebene 1: Sicherheits-Reflexe (Injected Instincts)

Über `teach()` (Mapping auf `API_Teach`) wird der Reflex `T_HEALTH_LOW -> T_PANIC_REST` mit einem Gewicht von  injiziert. Da Reflexe im `0x4010` Cluster liegen, überschreiben sie jede andere aktive Planung. Dies garantiert, dass der Agent bei kritischer Gesundheit ruht, selbst wenn er Futter sieht.

### Ebene 2: Dynamische Erfahrung (Reinforcement Learning)

Der Agent nutzt `feedback()`, um die Effizienz seiner Handlungen zu bewerten:

* **Positive Verstärkung**: Das Fressen (`T_EAT`) bei sichtbarem Futter (`T_FOOD_SEE`) führt zu einem Reward von .
* **Negative Verstärkung**: Ein Fressversuch ohne sichtbares Futter wird mit  bestraft, um unnötigen Energieverbrauch zu vermeiden.

---

## 3. Ausführungs-Logik & Echtzeit-Garantie

Der Agent durchläuft einen zyklischen **Sense-Think-Act** Prozess:

1. **Sense**: Erfassung der internen Floats (Hunger/Energie) und Abbildung auf diskrete TokenIDs.
2. **Think**: Der Aufruf von `update()` vergleicht die aktiven Tokens gegen das LTM. Die Entscheidung erfolgt in **konstanter Zeit **, unabhängig davon, wie viele Erfahrungen der Agent bereits gesammelt hat.
3. **Act**: Ausführung der gewählten `Action-TokenID` und Rückführung der Konsequenzen als Belohnungssignal.

---

## 4. Industrielle Integrität & Sicherheit (ISS)

Obwohl es sich um eine Simulation handelt, erfüllt der Agent die Standards des **Industrial Sovereign Security**:

* **Production Freeze**: Durch `set_mode(1)` wird die neuro-symbolische Struktur nach der Injektion der Instinkte versiegelt. Es finden keine neuen Speicherallokationen statt, was absolute Echtzeitstabilität garantiert.
* **Sovereign Key**: Die gelernten Verhaltensmuster sind mathematisch an die `key.json` gebunden. Ein Transfer des "Wissens" auf eine Engine mit einem anderen Schlüssel würde zu unvorhersehbarem Fehlverhalten führen.
* **Ressourcen-Management**: Der Python-Wrapper nutzt RAII-Prinzipien, um sicherzustellen, dass das native `BioBrain`-Handle beim Beenden der Simulation über `API_FreeBrain` sauber bereinigt wird.

---

**BrainAI** *- We don't need **BRUTEFORCE**, we know **Physics** -*</br>
Developed by **Sascha A. Köhne (winemp83)**</br>
Product: **BioAI 0.7.6 (Industrial Closed Feature)**</br>
📧 [koehne83@googlemail.com](mailto:koehne83@googlemail.com)

© 2025 BrainAI / Sascha A. Köhne. All rights reserved.