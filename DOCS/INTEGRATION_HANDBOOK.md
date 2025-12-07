# 📘 BioAI Developer Guide: Architektur & Migration 🧠
[Image of BrainAI Logo]
**Version:** 0.0.2 (Alpha)
**Modul:** BioAI Core Integration

-----

## 1. Executive Summary: Der Paradigmenwechsel

BioAI ist keine klassische "Library", die man aufruft, um ein statisches Ergebnis zu erhalten. Es ist eine **Engine für adaptive Autonomie**.

Die Integration erfordert einen Wechsel in der Denkweise des Entwicklers:

* **VON:** Prozeduraler Kontrolle ("Wenn Sensor A > 50, dann Motor An")
* **ZU:** Zielorientierter Führung ("Halte die Temperatur bei 50. Hier sind deine Sensoren und Schalter. Lerne den Rest.")

Diese Dokumentation beschreibt die **Referenz-Architektur**, mit der robuste, skalierbare und sichere industrielle KI-Systeme auf Basis des BioAI-Wrappers gebaut werden.

-----

## 2. Die Architektur-Blaupause (Reference Pattern)

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
public
