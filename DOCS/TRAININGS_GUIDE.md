# BioAI Training Strategy Guide

Wie wird ein BioAI-Agent schlau? Wir nutzen ein erweitertes 3-Stufen-Modell, das biologische Prinzipien mit technischer Präzision vereint.

---

## Ebene 1: Instinkte (Injected Knowledge)
Angeborenes Wissen, das ab Sekunde 0 verfügbar ist. Dies ist die Basis für Sicherheit ("Safety First").

* **Konzept:** Harte Regeln, die im Langzeitgedächtnis (LTM) verankert sind.
* **Code:** `API_Teach(brain, input, action, 1.0f)`
* **Anwendung:**
    * **Sicherheit:** "Wenn Sensor > 100°C, dann NOT-AUS." (Gewicht 1.0 = Unverhandelbares Gesetz).
    * **Bias:** "Wenn du nicht weiter weißt, dreh dich nach rechts." (Gewicht 0.3 = Tendenz).

---

## Ebene 2: Erfahrung (Reinforcement Learning)
Lernen durch Versuch und Irrtum im laufenden Betrieb.

* **Konzept:** Der Agent probiert etwas aus. Basierend auf dem Ergebnis wird die Verbindung gestärkt oder geschwächt.
* **Code:** `API_Feedback(brain, reward, action)`
* **Prozess:**
    1.  Agent handelt (`API_Update`).
    2.  Sensor misst Ergebnis (z.B. "Temperatur gesunken").
    3.  System gibt Feedback:
        * **Positiv (+1.0):** Verbindung wird verstärkt.
        * **Negativ (-1.0):** Verbindung wird gehemmt.
* **Speicher-Schutz:** BioAI nutzt ein Kurzzeitgedächtnis (STM). Nur wenn eine Erfahrung mehrfach bestätigt wird (Consolidation Hits), wandert sie ins permanente Gedächtnis (LTM). Das verhindert das Lernen von Zufällen ("Rauschen").

---

## Ebene 3: Imagination (Simulation & Planung) ✨ *Neu in v0.0.2*
Die Fähigkeit, Konsequenzen vorherzusehen, bevor man handelt.

* **Konzept:** Der Agent nutzt sein gelerntes Kausalitäts-Wissen, um die Zukunft zu simulieren.
* **Code:** `API_Simulate(brain, inputs, count, depth)`
* **Anwendung:**
    * Ein Roboter steht vor einem Abgrund.
    * Statt zu springen (und zerstört zu werden), simuliert er den Sprung.
    * Ergebnis der Simulation: "Schmerz/Tod".
    * Entscheidung: Er bleibt stehen.
* **Vorteil:** Ermöglicht intelligentes Verhalten ohne teure Fehlversuche in der Realität.

---

## Ebene 4: Schwarm-Wissen (Social Propagation)
<p align="center"><img src="../images/brainai_logo_bw.png" width="100" alt="BrainAI"></p>

Wissen ist nicht an einen einzelnen Agenten gebunden.

* **Konzept:** Da `TokenID` (64-Bit Hash) universell ist, können Erfahrungen geteilt werden.
* **Mechanik:**
    * Agent A lernt: "Input X -> Aktion Y ist schlecht."
    * Agent A sendet `TokenID(X)` und `TokenID(Y)` an Agent B.
    * Agent B ruft `API_Teach(X, Y, -1.0)` auf.
* **Ergebnis:** Der gesamte Schwarm lernt aus dem Fehler eines einzelnen Individuums.

---

**BrainAI** - *Intelligence everywhere.*
Developed by **Sascha A. Köhne (winemp83)**
Product: **BioAI v0.0.2 (Alpha)**
