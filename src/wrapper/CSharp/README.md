# 📘 BioAI C# Integration Guide (v0.7.6)

Diese Dokumentation beschreibt die Nutzung des C#-Wrappers für die BioAI-Engine. Der Wrapper dient als Brücke zwischen der verwalteten .NET-Umgebung und dem nativen C-Rechenkern.

## 1. Vokabular-Referenz (The Concept Dump) 🟦🟥

Um die Engine effektiv zu nutzen, müssen Eingangsdaten (Inputs) und Aktionen in das 64-Bit **TokenID**-Format übersetzt werden. BioAI kategorisiert die Realität in fünf primäre Cluster:

| Cluster | Maske (Hex) | Bedeutung | Beispiel |
| --- | --- | --- | --- |
| **OBJECT** | `0x1000...` | **Das Ding / Der Zustand** | Sensordaten, Temperatur, Objekterkennung. |
| **ACTION** | `0x2000...` | **Das Tun** | Motorsteuerung, Datenbank-Eintrag, Not-Aus. |
| **TIME** | `0x3000...` | **Das Wann** | Zeitstempel, Intervalle, Schichtpläne. |
| **LOGIC** | `0x4000...` | **Die Regel** | Wenn-Dann-Verknüpfungen, logische Gatter. |
| **SELF** | `0x5000...` | **Das Ich** | Interne Zustände, Batteriestand, Zielerreichung. |

> **Wichtig:** Ein **Reflex** (unbrechbare Regel) nutzt die Sub-Maske `0x4010...`. Ein Token mit dieser Maske und einem Gewicht  unterbricht sofort alle anderen Denkprozesse.

---

## 2. API-Referenz: `BioBrainInstance`

Die Klasse `BioBrainInstance` kapselt ein natives `BioBrain` und stellt alle Kernfunktionen sicher zur Verfügung.

### Initialisierung & Setup

* **`new BioBrainInstance(jsonPath)`**:
* Liest den `customer_key` aus der `key.json`.
* Initialisiert den nativen Kern mit dem entsprechenden Lizenzschlüssel.


* **`SetMode(int mode)`**:
* `0`: **Training** (Plastischer Modus).
* `1`: **Produktion** (Fixed Structure). Deaktiviert Speicher-Allokationen für maximale Stabilität.



### Kern-Operationen

* **`Update(ulong[] inputs)`**: Verarbeitet die aktuelle Wahrnehmung und gibt die TokenID der optimalen Aktion zurück. Arbeitet in konstanter Zeit .
* **`Simulate(ulong[] inputs, int depth)`**: Führt eine interne Simulation ("Imagination") durch, um Konsequenzen über mehrere Schritte zu bewerten.
* **`Feedback(float reward, ulong action)`**: Passt das Verhalten an. Positive Werte verstärken die Assoziation, negative schwächen sie ab.

### Wissen & Reflexe

* **`Teach(input, action, weight)`**: Injiziert Wissen direkt in das Langzeitgedächtnis (LTM). Ein Gewicht von `1.0f` erzeugt einen Reflex.
* **`Inspect(input, action)`**: Gibt das aktuell gelernte Gewicht zurück. Der Wert wird automatisch "entsalzt", sofern der korrekte Lizenzschlüssel geladen wurde.

---

## 3. Speichermanagement & Persistenz

Da der C#-Wrapper mit unverwaltetem C-Speicher arbeitet, müssen die folgenden Regeln beachtet werden:

### Serialisierung (Sichern & Laden)

```csharp
// Exportiert den aktuellen Zustand in ein Byte-Array
byte[] data = brain.Serialize(); 

// Wichtig: Der native Puffer wird innerhalb der Methode automatisch 
// über API_FreeBuffer freigegeben, um Leaks zu vermeiden.

```

### Ressourcen-Bereinigung

Die Klasse implementiert `IDisposable`. Durch den Aufruf von `Dispose()` (oder die Nutzung eines `using`-Blocks) wird die native Funktion `API_FreeBrain` aufgerufen, die alle Neuronen, Synapsen und Hashtabellen im C-Kern sicher löscht.

---

## 4. Sicherheitshinweise (ISS-Standard)

1. **Thread-Sicherheit**: Eine Instanz von `BioBrainInstance` ist **nicht thread-safe**. Parallele Zugriffe auf dieselbe Instanz müssen durch externe Locks geschützt werden.
2. **Key-Integrität**: Der Lizenzschlüssel in der `key.json` muss exakt dem Schlüssel entsprechen, mit dem das Gehirn trainiert wurde. Ein falscher Schlüssel führt zu mathematisch verfälschten Gewichten beim Auslesen oder Laden.
3. **Tier-Kompatibilität**: Stellen Sie sicher, dass die `BioAI_ULTRA.dll` (oder ein anderes Tier) im Ausgabeverzeichnis Ihrer Anwendung liegt, da der Wrapper per `DllImport` fest darauf referenziert.

---

**BrainAI** *- We don't need **BRUTEFORCE**, we know **Physics** -*</br>
Developed by **Sascha A. Köhne (winemp83)**</br>
Product: **BioAI 0.7.6 (Industrial Closed Feature)**</br>
📧 [koehne83@googlemail.com](mailto:koehne83@googlemail.com)

© 2025 BrainAI / Sascha A. Köhne. All rights reserved.