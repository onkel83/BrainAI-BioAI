
# Mitwirken an BioAI 🧠

Vielen Dank für dein Interesse an BioAI!

BioAI besteht aus zwei Bereichen mit strikt unterschiedlichen Regeln für Beiträge, um die Integrität und Sicherheit (Safety) des Systems zu schützen.

## 🛑 TEIL A: Der BioAI Core (Eingeschränkt)

Der C-Core (`bioai_"Version".dll/so` und Header) ist **proprietäre Technologie** und geistiges Eigentum von **BrainAI**.

* **Status:** **READ-ONLY** (Nur lesend).
* **Richtlinie:** Wir akzeptieren **KEINE** Pull Requests, Modifikationen oder „Optimierungen“ für die Core-Logik.
* **Warum?** Der Kern ist nach spezifischen Sicherheitsstandards zertifiziert (Determinismus, Speichersicherheit, ). Jede externe Änderung könnte die strengen industriellen Sicherheitsgarantien verletzen.
* **Fehler:** Wenn du einen Fehler im Core vermutest, öffne bitte ein **Issue** und beschreibe das Verhalten. Versuche **nicht**, den Fehler selbst im Quellcode zu beheben.

**Jeder Pull Request, der versucht, die Logik des C-Cores zu ändern, wird umgehend geschlossen.**

---

## 🟢 TEIL B: Wrapper & Tools (Offen für Beiträge)

Die Wrapper (C#, Python, C++, Java usw.) und Demo-Anwendungen sind **Open Source** (MIT-Lizenz). Wir begrüßen Beiträge der Community, um BioAI auf weiteren Plattformen zugänglich zu machen.

### Wie du zu den Wrappern beitragen kannst:

1. **Neue Plattformen:** Du möchtest BioAI unter Rust, Go oder Swift nutzen? Erstelle gerne einen neuen Wrapper, der die Schnittstelle zur kompilierten Bibliothek (Binary) bildet.
2. **Benutzerfreundlichkeit:** Verbessere das Python-pip-Paket oder die NuGet-Integration.
3. **Beispiele:** Erstelle Demo-Projekte (z. B. für Arduino oder Unity), welche die *bestehende* Core-Binary nutzen.

### Richtlinien für Beiträge

1. **Fork & Branch:** Erstelle einen Feature-Branch für deinen Wrapper oder dein Tool.
2. **Keine Core-Änderungen:** Stelle sicher, dass dein PR das Verzeichnis `libs/` nicht berührt.
3. **Dokumentation:** Wenn du einen Wrapper hinzufügst, lege bitte eine `README` bei, die erklärt, wie man ihn benutzt.

---

## 🐛 Fehler melden (Issues)

Wenn du einen Fehler findest, nutze bitte den Issue-Tracker:

* **Kategorie:** [Wrapper] oder [Core-Verhalten]
* **Beschreibung:** Klare Beschreibung des Fehlers.
* **Reproduktion:** Ein Code-Snippet, das die öffentliche API nutzt.

---

## ⚖️ Lizenzvereinbarung

Durch das Einreichen eines Pull Requests für die Wrapper oder Tools erklärst du dich damit einverstanden, dass dein Beitrag unter der **MIT-Lizenz** lizenziert wird.
Der BioAI Core bleibt strikt unter der **proprietären Lizenz** von BrainAI.

**Das BrainAI-Team**

---
