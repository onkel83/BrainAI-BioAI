# 📘 BioAI Java Integration Guide (v0.7.6)

Diese Dokumentation beschreibt die Integration der BioAI-Engine in Java-Applikationen. Der Wrapper dient als performante Brücke zwischen der JVM (Java Virtual Machine) und dem nativen C-Kern.

## 1. Vokabular-Referenz (The Concept Dump) 🟦🟥

In BioAI wird die Realität nicht in Texten, sondern in 64-Bit **TokenIDs** verarbeitet. Um die Engine zu füttern, müssen Sie Ihre Daten in die folgenden fünf Cluster kategorisieren:

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

Die Klasse `BioBrainInstance` nutzt JNA, um das native `BioBrain` zu verwalten und implementiert `AutoCloseable` für die Speichersicherheit.

### Initialisierung & Setup

* **`new BioBrainInstance(jsonPath)`**:
* Lädt den `customer_key` aus der ISS-generierten `key.json`.
* Konvertiert den Hex-String sicher in das Java-`long`-Format (64-Bit).
* Initialisiert den nativen Rechenkern.


* **`setMode(int mode)`**:
* `0`: **Training** (Plastischer Modus) – KI lernt aus Feedback.
* `1`: **Produktion** (Fixed Structure) – Kern ist versiegelt. Keine neuen Allokationen, maximale Echtzeitstabilität.



### Kern-Operationen

* **`update(long[] inputs)`**: Verarbeitet Wahrnehmungen und liefert die TokenID der optimalen Aktion in .
* **`simulate(long[] inputs, int depth)`**: "Imaginations-Modus". Berechnet Konsequenzen von Handlungen über die definierte Tiefe.
* **`feedback(float reward, long action)`**: Verstärkt (+) oder schwächt (-) Verhaltensmuster über Reinforcement Learning.

### Wissen & Persistenz

* **`teach(input, action, weight)`**: Injiziert Wissen direkt in das Langzeitgedächtnis (LTM). Ein Gewicht von `1.0f` erzeugt einen unbrechbaren Reflex.
* **`inspect(input, action)`**: Liest das aktuell gelernte Gewicht einer Verbindung aus. Der Wert wird mit dem Lizenzschlüssel automatisch "entsalzt".
* **`serialize()`**: Erzeugt einen Byte-Array-Snapshot des gesamten Gehirns.

---

## 3. Speichermanagement (JNA & Memory Ownership)

Da Java-Objekte im Heap der JVM liegen, die BioAI-Engine aber im nativen Arbeitsspeicher operiert, gelten strikte Regeln für die Ressourcenfreigabe:

1. **Native Freigabe**: Die Methode `close()` ruft intern `API_FreeBrain` auf. Dies löscht alle Neuronen, Synapsen und Hashtabellen im C-Kern.
2. **Buffer-Handling**: Beim Aufruf von `serialize()` wird der native Puffer sofort nach dem Kopieren in das Java-`byte[]` über `API_FreeBuffer` freigegeben, um Memory Leaks zu verhindern.
3. **Try-with-Resources**: Nutzen Sie immer die Try-with-Resources-Syntax, um sicherzustellen, dass das Gehirn auch bei Laufzeitfehlern sauber geschlossen wird.

---

## 4. Sicherheitshinweise (ISS-Standard)

1. **Thread-Sicherheit**: Eine `BioBrainInstance` darf **nicht** zeitgleich von mehreren Java-Threads genutzt werden. Für parallele Agenten müssen separate Instanzen erzeugt werden.
2. **Key-Integrität**: Ohne den korrekten Schlüssel aus der `key.json` ist ein Laden von gespeicherten Modellen nicht möglich, da die Gewichte mathematisch an den Key gebunden sind.
3. **Tier-Architektur**: Stellen Sie sicher, dass die für Ihre Hardware passende Bibliothek (z. B. `BioAI_ULTRA.so` oder `.dll`) im `java.library.path` liegt.

---

**BrainAI** *- We don't need **BRUTEFORCE**, we know **Physics** -*</br>
Developed by **Sascha A. Köhne (winemp83)**</br>
Product: **BioAI 0.7.6 (Industrial Closed Feature)**</br>
📧 [koehne83@googlemail.com](mailto:koehne83@googlemail.com)

© 2025 BrainAI / Sascha A. Köhne. All rights reserved.

