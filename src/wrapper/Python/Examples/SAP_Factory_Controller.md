# 🏭 Dokumentation: BIOAI SAP Factory Controller (v0.7.6)

Diese Dokumentation beschreibt die Funktionsweise und Integration des autonomen Fertigungs-Controllers, der als Bindeglied zwischen **SAP S/4HANA** (ERP-Ebene) und der **physischen Produktion** (Shop Floor) agiert.

## 1. Systemübersicht

Der Controller nutzt die **BioAI Core Engine**, um deterministische Prozessabläufe mit intelligenter Reflex-Überwachung zu kombinieren. Er kommuniziert über das SAP NW RFC SDK (`pyrfc`) direkt mit dem ERP-System, um Materialbestände zu prüfen und Fertigmeldungen abzusetzen.

### Kernmerkmale

* **Hybride Logik:** Kombiniert starre Ablaufpläne (Sequencing) mit priorisierten Sicherheitsinstinkten.
* **Deterministische Echtzeit:** Garantierte Antwortzeiten von  durch  Rechenkomplexität.
* **Industrielle Sicherheit:** Schutz von Geschäftsgeheimnissen durch Key-basiertes Gewichts-Salting im Arbeitsspeicher.

---

## 2. Ontologie und Cluster-Mapping

Die Steuerung basiert auf dem 64-Bit **Token-System** der BioAI-Engine. Die im Code definierten Tokens sind logisch in Cluster unterteilt, um die kognitive Integrität zu wahren:

| Kategorie | Cluster-Maske | Implementierte Tokens (Beispiele) |
| --- | --- | --- |
| **Produktions-Aktionen** | `0x2000...` | `T_PICK`, `T_WELD`, `T_SCAN`, `T_PACK` |
| **Objekte / Sensoren** | `0x1000...` | `T_SAP_STOCK_LOW`, `T_HAND_DETECTED` |
| **Sicherheits-Reflexe** | `0x4010...` | `T_SAFETY_STOP` |

---

## 3. Funktionsweise des Controllers

### A. Initialisierung und Sicherheit (RAII)

Beim Start wird die `BioBrainInstance` mit dem industriellen Lizenzschlüssel aus der `key.json` initialisiert.

1. **Instinkt-Injektion:** Im Training-Modus (`set_mode(0)`) wird der Sicherheits-Reflex (Lichtschranke/Hand-Erkennung) fest im Langzeitgedächtnis verankert.
2. **Produktions-Versiegelung:** Durch `set_mode(1)` wird das System in den **Fixed Structure Mode** versetzt. In diesem Modus sind keine dynamischen Speicherallokationen mehr möglich, was absolute Stabilität im Dauerbetrieb garantiert.

### B. Perzeption (SAP-Integration)

Vor Produktionsstart führt der Controller eine Bestandsabfrage via RFC-Aufruf (`BAPI_MATERIAL_GET_DETAIL`) durch. Nur bei ausreichender Materialverfügbarkeit wird der Fertigungsplan geladen.

### C. Execution Loop und Reflex-Priorisierung

Während der Plan-Ausführung überwacht der Controller permanent die Sensor-Eingänge.

* **Normalbetrieb:** Die Engine liefert den nächsten Schritt des geladenen Plans zurück.
* **Ereignis-Intervention:** Sobald ein kritisches Signal wie `T_HAND_DETECTED` auftritt, unterbricht die interne `bio_think_logic` den Plan sofort und gibt den `T_SAFETY_STOP` zurück. Der Controller reagiert mit einem sofortigen `abort_plan()`.

---

## 4. Schutz von Geschäftsgeheimnissen

Der Controller schützt proprietäre Prozessoptimierungen durch mathematische Verschleierung:

* **Salting:** Jedes neuronale Gewicht im RAM wird mit einem aus dem `license_key` abgeleiteten Faktor multipliziert.
* **Memory-Sicherheit:** Ohne den physisch im Gateway hinterlegten Schlüssel ist ein Auslesen der gelernten "Best Practices" der Fertigung unmöglich.

---

## 5. Deployment-Struktur

Für den Betrieb ist folgende Ordnerstruktur zwingend erforderlich:

* `SAP_Factory_Controller.py`: Die Hauptlogik (AutonomousLine).
* `bioai.py`: Der erweiterte Python-Wrapper.
* `BioAI_ULTRA.dll` (oder `.so`): Die native Rechenkern-Bibliothek.
* `key.json`: Der industrielle Sicherheitsschlüssel.

---

**BrainAI** *- We don't need **BRUTEFORCE**, we know **Physics** -*</br>
Developed by **Sascha A. Köhne (winemp83)**</br>
Product: **BioAI 0.7.6 (Industrial Closed Feature)**</br>
📧 [koehne83@googlemail.com](mailto:koehne83@googlemail.com)

© 2025 BrainAI / Sascha A. Köhne. All rights reserved.