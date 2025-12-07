-----

# 📘 BioAI Developer Guide: Architektur & Migration

**Version:** 0.0.2 (Alpha)
**Modul:** BioAI Core Integration

-----

## 1\. Executive Summary: Der Paradigmenwechsel

BioAI ist keine klassische "Library", die man aufruft, um ein statisches Ergebnis zu erhalten. Es ist eine **Engine für adaptive Autonomie**.

Die Integration erfordert einen Wechsel in der Denkweise des Entwicklers:

  * **VON:** Prozeduraler Kontrolle ("Wenn Sensor A \> 50, dann Motor An")
  * **ZU:** Zielorientierter Führung ("Halte die Temperatur bei 50. Hier sind deine Sensoren und Schalter. Lerne den Rest.")

Diese Dokumentation beschreibt die **Referenz-Architektur**, mit der robuste, skalierbare und sichere industrielle KI-Systeme auf Basis des BioAI-Wrappers gebaut werden.

-----

## 2\. Die Architektur-Blaupause (Reference Pattern)

Um BioAI sauber zu integrieren, empfehlen wir eine strikte Trennung von **Sensorik**, **Logik** und **Aktorik**. Entwickler sollten ihre Anwendung gegen diese Interfaces bauen, um Modularität zu gewährleisten.

### 2.1 Die Core-Interfaces

Diese Interfaces definieren den "Baukasten", aus dem ein BioAI-System besteht.

```csharp
namespace BioAI.Abstractions;

/// <summary>
/// Repräsentiert einen "Sinn". Wandelt physikalische Daten in neuronale Token um.
/// </summary>
public interface IBioSensor
{
    string Name { get; }
    // Liefert das aktive Token (z.B. "TEMP_HOT") oder 0, wenn der Sensor inaktiv ist.
    ulong GetSignal(); 
}

/// <summary>
/// Repräsentiert einen "Muskel". Führt eine physische Handlung aus.
/// </summary>
public interface IBioAction
{
    string Name { get; }
    ulong Token { get; }
    void Execute();
}

/// <summary>
/// Definiert das Ziel. Bewertet eine Handlung im Kontext der Situation.
/// </summary>
public interface IBioRewardFunction
{
    // Gibt einen Wert zwischen -1.0 (Bestrafung) und +1.0 (Belohnung) zurück.
    float CalculateReward(ulong inputState, ulong actionTaken);
}
```

### 2.2 Der BioAgent (Basis-Klasse)

Dies ist die empfohlene Implementierung für einen autonomen Agenten. Sie kapselt den `Think -> Act -> Learn` Loop.

```csharp
public abstract class BioAgent : IDisposable
{
    protected BioBrain _brain;
    private List<IBioSensor> _sensors = new();
    private List<IBioAction> _actions = new();
    
    // Safety Layer: Hard-coded Reflexe, die das Netz überschreiben
    private Dictionary<ulong, ulong> _reflexes = new();

    public BioAgent(ulong id) {
        _brain = new BioBrain(id);
        _brain.SetMode(BioMode.Training);
    }

    public void RegisterSensor(IBioSensor s) => _sensors.Add(s);
    public void RegisterAction(IBioAction a) => _actions.Add(a);
    
    // Programmiert einen unumstößlichen Reflex (Safety First)
    public void AddSafetyReflex(string sensorName, string actionName, float priority) {
        // ... (Implementierung via ForceInstinct) ...
    }

    // Der "Herzschlag" des Agenten
    public void Update() 
    {
        // 1. WAHRNEHMUNG
        ulong currentInput = 0;
        foreach(var s in _sensors) {
            ulong sig = s.GetSignal();
            if (sig != 0) { currentInput = sig; break; } // Priorität oder Mischung
        }

        // 2. ENTSCHEIDUNG (BioAI Core)
        ulong actionToken = _brain.Think(currentInput);

        // 3. HANDLUNG
        var action = _actions.FirstOrDefault(a => a.Token == actionToken);
        if (action != null) action.Execute();

        // 4. LERNEN (Feedback)
        float reward = CalculateReward(currentInput, actionToken);
        _brain.Learn(reward, actionToken);
    }

    protected abstract float CalculateReward(ulong input, ulong action);
}
```

-----

## 3\. Der BioAI-Zyklus (Visuell)

Um die Arbeitsweise zu verstehen, hilft dieses Flussdiagramm des Datenstroms innerhalb eines `Update()` Ticks:

1.  **Welt:** Temperatur steigt.
2.  **Sensor:** `TempSensor` feuert Token `HEAT_CRITICAL`.
3.  **Brain:** Empfängt Token. Prüft Reflexe. Wenn kein Reflex, prüft neuronale Pfade. Wählt `FAN_ON`.
4.  **Aktor:** `FanAction` schaltet Strom an.
5.  **Feedback:** Hat die Temperatur sich verbessert?
      * *Ja:* `Reward = +1.0` (Verbindung stärken).
      * *Nein:* `Reward = -0.5` (Verbindung schwächen, Energie verschwendet).

-----

## 4\. Migration: Vorher vs. Nachher

Wie stellen Entwickler konkreten Code um? Hier sind die Szenarien.

### Szenario A: Zustandsmaschine (State Machine)

**🔴 Klassisch (C\# Switch-Case):**
Starr und schwer zu erweitern.

```csharp
switch (currentState) {
    case State.Idle:
        if (Battery < 20) State = State.Charging;
        break;
    case State.Charging:
        if (Battery > 90) State = State.Working;
        break;
}
```

**🟢 BioAI (Organisch):**
Flexibel. Der Agent lernt selbst, wann er laden muss.

```csharp
// Wir definieren nur die Sensoren
RegisterSensor(new BatterySensor()); // Liefert BAT_LOW, BAT_OK, BAT_FULL

// Wir definieren das Ziel (Reward)
protected override float CalculateReward(...) {
    // Überleben ist alles
    if (Battery <= 0) return -10.0f; // Tod
    if (Battery < 20 && action == CHARGE) return 2.0f; // Richtiges Verhalten
    return 0.1f;
}
// Das Brain findet den Zustandsübergang selbst.
```

### Szenario B: Sicherheitskritische Steuerung (Safety)

**🔴 Klassisch (Hard Coded Safety):**
Der Code muss die Steuerung *unterbrechen*.

```csharp
void ControlLoop() {
    if (rpm > 1500) {
        EmergencyStop(); // Bypassed die normale Logik
        return; 
    }
    // ... normale PID Regelung ...
}
```

**🟢 BioAI (Integrierte Reflexe):**
Die Sicherheit ist *Teil* des Gehirns.

```csharp
// Im Konstruktor:
// "Wenn der Sensor OVERSPEED meldet, ist die EINZIGE valide Antwort STOP."
AddSafetyReflex("SENSOR_OVERSPEED", "ACTION_STOP", 100.0f);

// Im Loop:
// Wir müssen nichts tun. Wenn der Sensor feuert, zwingt der Core die Aktion.
// Der Vorteil: Das Netz "sieht", dass es gestoppt hat und lernt, 
// die Situation (Overspeed) vorher schon zu vermeiden (Prediction).
```

-----

## 5\. Best Practices & Stolpersteine

### 1\. Exploration zulassen

Ein frisches BioBrain weiß nichts. Es wird am Anfang Fehler machen (z.B. Motor geht aus).

  * **Tipp:** Nutze am Anfang `ForceInstinct` für ein Basis-Wissen ("Pre-Training"), oder akzeptiere eine Chaos-Phase, wenn du echte Innovation (neue Lösungswege) willst.

### 2\. Die Reward-Funktion ist der Schlüssel

Der häufigste Fehler ist eine schlechte Belohnung.

  * *Schlecht:* "Belohne immer, wenn Motor läuft." (Führt zu Überhitzung).
  * *Gut:* "Belohne, wenn Motor läuft UND Temperatur \< 80°C." (Führt zu Effizienz).

### 3\. Inputs Clustern

Überschütte das Brain nicht mit `float`-Werten (23.41, 23.42, ...). Das ist Rauschen.

  * **Tipp:** Cluster die Werte in semantische Bereiche: `LOW`, `OPTIMAL`, `HIGH`, `CRITICAL`. BioAI arbeitet mit Zuständen, nicht mit Fließkommazahlen.

### 4\. Safety First

Vertraue bei kritischen Systemen (ITER, Pressen, Motoren) niemals *nur* dem gelernten Netz.

  * **Pflicht:** Nutze immer die **Reflex-Layer** (`ForceInstinct` mit hoher Prio) für physikalische Limits. Das ist deine Versicherung.

-----

## 6\. Fazit

Der Einsatz von BioAI bedeutet nicht, weniger zu programmieren. Es bedeutet, **weniger If-Statements und mehr Architektur** zu programmieren. Sie bauen den Körper (Sensoren/Aktoren) und definieren den Sinn des Lebens (Reward). BioAI füllt die Lücke dazwischen mit Leben.
