# 📘 BioAI VB.NET Integration Guide (v0.7.6)

Dieser Guide beschreibt die Einbindung der BioAI-Engine in VB.NET-Projekte (Windows Desktop, Services oder IoT Core).

## 1. Vokabular-Referenz (The Concept Dump) 🟦🟥

Nutzen Sie zur Strukturierung Ihrer Daten 64-Bit **UInt64** (TokenIDs). Die Engine erwartet die Einordnung in Cluster-Masken:

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

Die Klasse kapselt den nativen Kern und verbirgt die Komplexität der Speicherverwaltung.

### Lifecycle & Setup

* **`New BioBrainInstance(jsonPath)`**: Lädt den Schlüssel aus der `key.json` und erzeugt die native Gehirn-Instanz.
* **`SetMode(mode)`**:
* `0`: **Training** (Plastisch).
* `1`: **Produktion** (Versiegelt). Keine weiteren Speicherallokationen, garantiert deterministisches Verhalten.


* **`Dispose()`**: Gibt alle nativen Ressourcen frei. Nutzen Sie vorzugsweise den `Using`-Block.

### Kern-Operationen

* **`Update(inputs)`**: Verarbeitet Wahrnehmungen und liefert die optimale Aktion in .
* **`Simulate(inputs, depth)`**: Berechnet die Kausalitätskette über eine definierte Tiefe.
* **`Feedback(reward, action)`**: Passt das Verhalten über Reinforcement Learning an.

---

## 3. Sicherheits- & Performance-Hinweise

1. **Thread-Sicherheit**: Eine Instanz ist **nicht** thread-safe. Schützen Sie parallele Zugriffe durch `SyncLock` oder nutzen Sie separate Instanzen.
2. **Key-Integrität**: Der Schlüssel in der `key.json` ist mathematisch mit dem Gehirn-Zustand verknüpft. Ein Laden von Daten mit dem falschen Schlüssel führt zu fehlerhaften Gewichten.
3. **Deployment**: Stellen Sie sicher, dass die `BioAI_ULTRA.dll` (oder ein anderes Tier) im selben Verzeichnis wie Ihre `.exe` liegt.

---

**BrainAI** - *Intelligence everywhere.*
Entwickelt von **Sascha A. Köhne (winemp83)**
Produkt: **BioAI v0.7.6 (Industrial Stable)**