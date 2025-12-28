# Sicherheitsrichtlinie (Security Policy) 🛡️

BioAI wurde für **sicherheitskritische Edge-Anwendungen** (Industrielles IoT, Robotik, Smart Grids) entwickelt. Daher behandeln wir Sicherheitslücken nicht nur als Software-Fehler, sondern als potenzielle Risiken für die physische Sicherheit (Safety) von Mensch und Maschine.

## 🛡️ Unterstützte Versionen

Wir unterstützen derzeit Sicherheits-Updates für die folgenden Versionen des BioAI Cores und der Wrapper:

| Version | Unterstützt | Anmerkungen |
| --- | --- | --- |
| **v0.7.6 (Industrial Closed Feature)** | ✅ | Aktueller stabiler Release |
| < 0.7.6 | ❌ | Veraltet (Deprecated). Nicht in der Produktion verwenden. |

---

## 🚨 Meldung einer Schwachstelle

**Eröffnen Sie KEINE öffentlichen GitHub-Issues für Sicherheitslücken.**
Wenn Sie eine Schwachstelle entdeckt haben, welche die Integrität, Sicherheit oder Verfügbarkeit eines Systems mit BioAI gefährden könnte, melden Sie diese bitte vertraulich.

### Vorgehensweise

Bitte senden Sie eine E-Mail an den leitenden Architekten:

* **E-Mail:** [koehne83@googlemail.com](mailto:koehne83@googlemail.com)
* **Betreff:** `[SECURITY] BioAI Vulnerability Report`

### Erforderliche Informationen

Bitte geben Sie so viele Details wie möglich an:

1. **Komponente:** Betrifft es den *Open Source Wrapper* (C#/Python/Java/JS) oder den *proprietären C-Core* (`bioai.dll` / `.so`)?
2. **Schweregrad:** Kann die *Sicherheits-Reflex-Ebene* umgangen werden? Verursacht es einen Absturz (DoS)? Können falsche Token injiziert werden?
3. **Proof of Concept (PoC):** Ein minimales Code-Beispiel, eine bösartige Brain-Dump-Datei oder eine Beschreibung zur Reproduktion des Problems.

### Unser Reaktionsprozess

1. **Eingangsbestätigung:** Wir bestätigen den Erhalt Ihrer Meldung innerhalb von 48 Stunden.
2. **Verifizierung:** Wir prüfen die Schwachstelle intern.
3. **Patching:**
* **Wrapper:** Wir veröffentlichen umgehend einen Fix im öffentlichen Repository.
* **Core:** Wir patchen die proprietäre Binary und veröffentlichen eine neue Version (z. B. v0.7.6).


4. **Offenlegung:** Sobald der Patch für Kunden/Nutzer verfügbar ist, werden wir Sie (falls gewünscht) in den Release-Notes nennen.

---

## 🔒 Spezifischer Sicherheitsumfang (Scope)

### 1. Der C-Core (Binary)

Der Core operiert im **Fixed Structure Mode** (kein `malloc`/`free` während der Laufzeit), um Angriffe auf die Speicherintegrität physikalisch auszuschließen.

* **Kritisch:** Jede Methode, die einen **Buffer Overflow** (Pufferüberlauf) oder ein **Memory Leak** im Core auslöst (insbesondere über `API_Deserialize`), wird als kritisch eingestuft.
* **Kritisch:** Jede Methode, die einen **ForceInstinct (Reflex)** umgeht, gilt als kritische Sicherheitsverletzung.
* **Hoch:** Das Brechen der **-Echtzeitgarantie** (z. B. durch Erzwingen einer Endlosschleife oder exzessiver Rechenzeit) stellt einen Denial-of-Service (DoS) gegen den physischen Regelkreis dar.

### 2. Die Wrapper (Source)

Die Wrapper bilden die Schnittstelle zwischen dem Betriebssystem und dem Core.

* **Hoch:** Schwachstellen, die eine **Token-Injektion** (Vortäuschen von Sensordaten) über die API-Grenzen hinweg ermöglichen.
* **Mittel:** DLL-Hijacking-Schwachstellen im Lademechanismus der Bibliotheken.

---

## ⚠️ Hinweis zu „Safety“ vs. „Security“

BioAI unterscheidet zwischen **Safety** (Vermeidung von Schäden für die Umgebung) und **Security** (Schutz vor böswilligem Zugriff).
In unserer Architektur führt ein *Security*-Bruch (z. B. das Modifizieren der LTM-Gewichte durch einen Exploit) jedoch unmittelbar zu einem *Safety*-Risiko (z. B. ein Roboter ignoriert das Stopp-Signal).

**Wir behandeln alle Sicherheitsberichte mit höchster Priorität.**

---

**BrainAI** - *-We don't need **BRUTEFORCE**, we know **Physiks**-*
Developed by **Sascha A. Köhne (winemp83)**
Product: **BioAI 0.7.6 (Industrial Closed Feature)**
📧 [koehne83@googlemail.com](mailto:koehne83@googlemail.com)

&copy; 2025 BrainAI / Sascha A. Köhne. All rights reserved.
