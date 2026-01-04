# 📘 BioAI C++ Integration Guide (v0.7.6)

Dieser Guide beschreibt die Nutzung des C++ Wrappers für die BioAI-Engine. Da C++ die native Sprache des Kerns ist, bietet dieser Wrapper die geringste Latenz und höchste Effizienz.

## 1. Vokabular & Cluster-Konzept 🟦🟥

Nutzen Sie die 64-Bit **TokenIDs**, um Informationen zu strukturieren. Jede ID sollte einer der folgenden Masken entsprechen:

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

Der Wrapper nutzt das **Resource Acquisition Is Initialization (RAII)** Prinzip: Ressourcen werden beim Erzeugen der Instanz belegt und beim Zerstören automatisch freigegeben.

### Lifecycle & Management

* **`BioBrainInstance(jsonPath)`**: Lädt den ISS-Sicherheitsschlüssel und initialisiert den Kern.
* **`setMode(int mode)`**: Schaltet zwischen Lernen (`0`) und Produktion (`1`) um. In Modus 1 ist das Gehirn gegen Speicherallokationen gesperrt (Echtzeitsicherheit).
* **Destruktor (`~`)**: Ruft automatisch `API_FreeBrain` auf, um Speicherlecks zu verhindern.

### Logik-Funktionen

* **`update(inputs)`**: Verarbeitet Wahrnehmungen und liefert die optimale Aktion in .
* **`simulate(inputs, depth)`**: Berechnet die Kausalitätskette über mehrere Schritte ("Imagination").
* **`feedback(reward, action)`**: Passt Gewichte über Reinforcement Learning an. Positive Werte verstärken das Verhalten.

### Persistenz & Sicherheit

* **`serialize()`**: Liefert ein `std::vector<uint8_t>` mit den Zustandsdaten zurück. Der interne native Puffer wird automatisch über `API_FreeBuffer` bereinigt.
* **`inspect(input, action)`**: Liest ein Gewicht aus. Der Wert wird mathematisch "entsalzt", sofern der korrekte Schlüssel geladen wurde.

---

## 3. Sicherheits- & Performance-Hinweise

1. **Thread-Sicherheit**: Eine Instanz ist **nicht thread-safe**. Verwenden Sie Mutexe oder separate Instanzen für parallele Verarbeitung.
2. **Key-Integrität**: Der Schlüssel in der `key.json` ist mathematisch mit den Gewichten verknüpft. Ein Laden von Daten mit dem falschen Schlüssel führt zu mathematisch verfälschtem Verhalten.
3. **Inlining**: Die Nutzung des C++ Wrappers erzeugt fast keinen Overhead, da die meisten Methoden direkt auf die nativen C-Funktionen durchgreifen.

---

**BrainAI** *- We don't need **BRUTEFORCE**, we know **Physics** -*</br>
Developed by **Sascha A. Köhne (winemp83)**</br>
Product: **BioAI 0.7.6 (Industrial Closed Feature)**</br>
📧 [koehne83@googlemail.com](mailto:koehne83@googlemail.com)

© 2025 BrainAI / Sascha A. Köhne. All rights reserved.
