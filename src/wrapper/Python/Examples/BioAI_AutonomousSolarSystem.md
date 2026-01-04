# ☀️ BioAI Python Solar Demo (v0.7.6)

Dieses Beispiel demonstriert eine autonome Energiesteuerung für ein Solarsystem, implementiert mit dem **BioAI Python-Wrapper**. Das System entscheidet in Echtzeit über Betriebsmodi und schützt die Hardware durch unverhandelbare Sicherheits-Reflexe.

## 1. Neuro-Symbolische Konfiguration

Um eine deterministische Steuerung zu gewährleisten, nutzt das System das **Cluster-Mapping** von BioAI. Daten werden nicht als Fließkommazahlen, sondern als diskrete 64-Bit **TokenIDs** verarbeitet.

### Verwendete Cluster-Masken

| Token | Maske (Hex) | Kategorie | Beschreibung |
| --- | --- | --- | --- |
| `T_SUN_...` | `0x1000...` | **OBJECT** | Umweltzustände (Lichteinstrahlung). |
| `T_BATT_...` | `0x1000...` | **OBJECT** | Systemstatus (Batteriestand). |
| `T_MODE_...` | `0x2000...` | **ACTION** | Wählbare Betriebsstrategien. |
| `T_HARD_SHUTDOWN` | `0x4010...` | **REFLEX** | Kritischer Hardware-Schutz (Höchste Prio). |

---

## 2. Entscheidungshierarchie im System

Das System folgt der strikten BioAI-Befehlskette, um Regelkonflikte mathematisch auszuschließen:

1. **Reflex-Ebene (Sicherheit)**: Ein injizierter Reflex (`teach` mit Gewicht 1.0) für `T_BATT_CRITICAL` erzwingt den `T_HARD_SHUTDOWN`, ungeachtet aller anderen Faktoren.
2. **Erfahrungs-Ebene (Lernen)**: Das System lernt über `feedback`, ob der `T_MODE_PERFORMANCE` bei aktuellem Akkustand vorteilhaft ist.
3. **Bias-Ebene (Instinkt)**: Vor-injizierte Tendenzen sorgen dafür, dass das System bei hoher Sonneneinstrahlung bevorzugt in den Leistungsmodus wechselt.

---

## 3. Integration & Ausführung

### Voraussetzungen

* **`bioai.py`**: Der native Python-Wrapper.
* **Tier-Binary**: Die entsprechende Bibliothek (z. B. `BioAI_ULTRA.dll` oder `.so`).
* **Sovereign Security**: Eine gültige `key.json` im `bin`-Verzeichnis.

### Programmablauf

```python
# Initialisierung mit Key-Injektion
solar_system = AutonomousSolarSystem("bin/key.json", "bin/BioAI_ULTRA.dll")

# Zyklische Verarbeitung (Sense -> Think -> Act)
# Zeitkomplexität: Garantiert O(1)
solar_system.process_cycle(lux=60000, battery_percent=85)

```

---

## 4. Sicherheits- & Performance-Metriken (ISS-Standard)

* **Deterministik**: Bei identischen Sensorwerten und gleichem Erfahrungsstand wird das System immer die exakt gleiche Entscheidung treffen.
* **Memory Safety**: Durch den Aufruf von `_brain.close()` werden alle nativen Ressourcen (Hashtabellen und Neuronen-Traces) sicher im C-Kern freigegeben.
* **Offline First**: Die gesamte Logik läuft lokal auf dem Edge-Device. Es werden keine Telemetriedaten zur Entscheidungsfindung an externe Server gesendet.

---

**BrainAI** *- We don't need **BRUTEFORCE**, we know **Physics** -*</br>
Developed by **Sascha A. Köhne (winemp83)**</br>
Product: **BioAI 0.7.6 (Industrial Closed Feature)**</br>
📧 [koehne83@googlemail.com](mailto:koehne83@googlemail.com)

© 2025 BrainAI / Sascha A. Köhne. All rights reserved.