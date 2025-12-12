# BioAI Training Strategy Guide 🧠

**Version:** 0.5.1 (Industrial Beta)

Wie wird ein BioAI-Agent intelligent? Wir nutzen ein erweitertes **4-Ebenen-Modell**, das biologische Prinzipien mit technischer Präzision und industrieller Sicherheit vereint.

---

## Ebene 1: Instinkte (Injected Knowledge)
Angeborenes Wissen, das ab Sekunde 0 verfügbar ist. Dies ist die Basis für **Safety First**.

* **Konzept:** Harte, deterministische Regeln, die direkt im Langzeitgedächtnis (LTM) verankert sind. Sie benötigen kein Training.
* **Code:** `API_Teach(brain, input, action, 1.0f)`
* **Anwendung:**
    * **Sicherheit (Hard Safety):** "Wenn Sensor > 100°C, dann NOT-AUS." (Gewicht 1.0 = Unverhandelbares Gesetz).
    * **Bias (Soft Safety):** "Wenn du nicht weiter weißt, dreh dich nach rechts." (Gewicht 0.3 = Tendenz).
* **Vorteil:** Auditierbar. Der TÜV kann den Quellcode prüfen und sehen: Diese Regel ist fest im System.

---

## Ebene 2: Erfahrung (Reinforcement Learning)
Lernen durch Versuch und Irrtum im laufenden Betrieb.

* **Konzept:** Der Agent probiert etwas aus. Basierend auf dem Ergebnis (Reward/Punishment) wird die Verbindung angepasst.
* **Code:** `API_Feedback(brain, reward, action)`
* **Prozess:**
    1.  Agent handelt (`API_Update`).
    2.  Sensor misst Ergebnis (z.B. "Temperatur gesunken").
    3.  System gibt Feedback:
        * **Positiv (+1.0):** Verbindung wird verstärkt.
        * **Negativ (-1.0):** Verbindung wird gehemmt.
* **Speicher-Schutz (Rauschunterdrückung):** BioAI nutzt ein Kurzzeitgedächtnis (STM). Nur wenn eine Erfahrung mehrfach bestätigt wird (`LTM_CONSOLIDATE_HITS`), wandert sie ins permanente Gedächtnis (LTM). Das verhindert, dass Zufälle ("Rauschen") gelernt werden.

---

## Ebene 3: Imagination (Simulation & Planung)
Die Fähigkeit, Konsequenzen vorherzusehen, *bevor* man handelt.

* **Konzept:** Der Agent nutzt sein gelerntes Kausalitäts-Wissen, um die Zukunft virtuell zu simulieren.
* **Code:** `API_Simulate(brain, inputs, count, depth)`
* **Anwendung:**
    * Ein Roboter steht vor einem Abgrund.
    * Statt zu springen (und zerstört zu werden), simuliert er den Sprung.
    * Ergebnis der Simulation (Tiefe 2): "Schmerz/Tod".
    * Entscheidung: Er bleibt stehen.
* **Safety:** Die Simulationstiefe ist durch `MAX_SIM_DEPTH` begrenzt, um Stack-Overflows auf IoT-Geräten physikalisch auszuschließen.

---

## Ebene 4: Schwarm-Wissen (Social Propagation)
Wissen ist nicht an einen einzelnen Agenten gebunden.

* **Konzept:** Da `TokenID` (64-Bit Hash) universell ist, können Erfahrungen mathematisch geteilt werden.
* **Mechanik:**
    1.  Agent A macht einen Fehler: "Input X -> Aktion Y führt zu Schaden."
    2.  Agent A sendet `TokenID(X)` und `TokenID(Y)` an Agent B (über WLAN/LoRa).
    3.  Agent B injiziert dieses Wissen: `API_Teach(X, Y, -1.0)`.
* **Ergebnis:** Der gesamte Schwarm lernt aus dem Fehler eines einzelnen Individuums, ohne ihn selbst machen zu müssen ("Fleet Learning").

---

**BrainAI** - *Intelligence everywhere.*
Developed by **Sascha A. Köhne (winemp83)**
Product: **BioAI v0.5.1 (Industrial Beta)**
📧 [koehne83@googlemail.com](mailto:koehne83@googlemail.com)

&copy; 2025 BrainAI / Sascha A. Köhne. All rights reserved.
