# BioAI Integration Guide: C# Interop Wrappers

**Version:** 0.7.6 (Industrial Closed Feature)  
**Module:** BrainAI.BioAI Core Interface  
**Status:** Enterprise / Gold Release

---

## 📋 Übersicht

Dieses Dokument definiert die technische Integration der nativen **BioAI-Core-Bibliothek** in .NET-basierte Umgebungen. Es werden drei spezifische Laufzeitumgebungen unterstützt:

1.  **.NET Standard Console / Server** (Windows/Linux Services)
2.  **.NET MAUI / Android** (Mobile Embedded Systems)
3.  **Unity Engine** (Simulation & Interactive)

Alle Wrapper basieren auf demselben `BioBrain` C#-Interface, unterscheiden sich jedoch signifikant im **Deployment der nativen Binärdateien (.dll, .so)** und im **Lebenszyklus-Management**.

---

## 1. Umgebung: .NET Console / Windows Server

Verwendung für Backend-Services, Test-Suites oder Standalone-Desktop-Anwendungen ohne GUI-Framework-Abhängigkeiten.

### ⚙️ Deployment-Voraussetzungen
* **Wrapper-Datei:** `BioBrain.cs` (Standard/Console Variante)
* **Native Bibliothek:**
    * Windows: `bioai.dll`
    * Linux: `libbioai.so`

**Pfad-Konvention:** Die native Bibliothek muss sich zwingend im **Root-Verzeichnis der ausführbaren Datei** befinden (neben der `.exe` bzw. dem Entry-Point DLL).

### 💻 Implementierungs-Muster

```csharp
using BrainAI.BioAI;

public class CoreSystem
{
    public void Initialize()
    {
        try 
        {
            // Initialisierung mit Lizenzschlüssel (Hex-Format)
            // Verwendung des 'using'-Blocks für deterministische Ressourcenfreigabe (RAII)
            using (var brain = new BioBrain(0xCB91_0000_1234_FFFF))
            {
                brain.SetMode(BioMode.Production);
                Console.WriteLine("[SYSTEM] BioBrain Kernel loaded successfully.");

                // Core Logic Loop
                PerformThinkingCycle(brain);
            }
        }
        catch (DllNotFoundException ex)
        {
            // Kritisch: Native Dependencies fehlen
            Console.WriteLine($"[FATAL] Native Library Linking Failed: {ex.Message}");
            Environment.Exit(1);
        }
    }

    private void PerformThinkingCycle(BioBrain brain)
    {
        ulong inputSignal = BioClusters.CreateToken("SYSTEM_READY", BioClusters.STATUS);
        ulong action = brain.Think(inputSignal);
    }
}
````

-----

## 2\. Umgebung: .NET MAUI / Android

Verwendung für mobile Applikationen oder embedded Android-Systeme. Hier greift der Wrapper auf das Android NDK (Native Development Kit) Interface zu.

### ⚙️ Deployment-Voraussetzungen

  * **Wrapper-Datei:** `BioBrain.cs` (MAUI Variante)
  * **Architektur:** Android erfordert spezifische Builds für verschiedene CPU-Architekturen.

**Verzeichnisstruktur im Projekt:**
Die `.so` Dateien müssen als **AndroidNativeLibrary** markiert sein (Build Action).

```text
MyMauiProject/
├── Platforms/
│   └── Android/
│       └── jniLibs/
│           ├── arm64-v8a/
│           │   └── libbioai.so   <-- (Moderne Geräte)
│           ├── armeabi-v7a/
│           │   └── libbioai.so   <-- (Ältere Geräte)
│           └── x86_64/
│               └── libbioai.so   <-- (Emulator)
```

### 💻 Implementierungs-Muster

In mobilen Umgebungen muss der Lebenszyklus an die **App-States** (Resume/Pause) gekoppelt werden, um Speicherlecks zu verhindern.

```csharp
public class BioService : IDisposable
{
    private BioBrain _brain;

    public void Startup()
    {
        // Laden erfolgt lazy beim ersten Zugriff oder explizit im Startup
        if (_brain == null)
        {
            _brain = new BioBrain(0xA1B2C3D4);
            _brain.SetMode(BioMode.Training); // Mobile Learning enabled
        }
    }

    public ulong ProcessSensorData(ulong sensorToken)
    {
        if (_brain == null) return 0;
        return _brain.Think(sensorToken);
    }

    // WICHTIG: Dispose beim Schließen der App oder des Services aufrufen
    public void Dispose()
    {
        _brain?.Dispose();
        _brain = null;
    }
}
```

-----

## 3\. Umgebung: Unity Engine (3D/Simulation)

Verwendung für visuelle Simulationen, Spiele oder digitale Zwillinge. Unity verwaltet das Laden von Plugins automatisch, erfordert aber strikte Pfade für I/O-Operationen.

### ⚙️ Deployment-Voraussetzungen

  * **Wrapper-Datei:** `BioBrain.cs` (Unity Variante)
  * **Native Bibliothek:** Alle Plattform-Varianten gehören in den `Plugins` Ordner.

**Verzeichnisstruktur (Unity Assets):**

```text
Assets/
├── Scripts/
│   └── BioBrain.cs
└── Plugins/
    ├── Windows/
    │   └── bioai.dll
    ├── Android/
    │   └── libbioai.so
    └── Linux/
        └── libbioai.so
```

> **Hinweis:** Stellen Sie im Unity Inspector sicher, dass für jede DLL/SO unter "Select platforms for plugin" die korrekte Plattform angehakt ist (z.B. bioai.dll -\> nur Windows x86\_64).

### 💻 Implementierungs-Muster (MonoBehaviour)

```csharp
using UnityEngine;
using BrainAI.BioAI;
using System.IO;

public class BioAIController : MonoBehaviour
{
    private BioBrain _brain;
    
    // Serialisierte Referenz für Inspector-Settings
    [SerializeField] private ulong _licenseKey = 0; 
    [SerializeField] private bool _enableTraining = false;

    void Awake()
    {
        try 
        {
            _brain = new BioBrain(_licenseKey);
            _brain.SetMode(_enableTraining ? BioMode.Training : BioMode.Production);
        }
        catch (System.Exception ex)
        {
            Debug.LogError($"[BioAI] Kernel Error: {ex.Message}");
            this.enabled = false;
        }
    }

    void OnApplicationQuit()
    {
        // Zwingend erforderlich für sauberen Shutdown des Native Threads
        _brain?.Dispose();
    }

    // Persistenz muss Application.persistentDataPath nutzen
    public void SaveState()
    {
        byte[] data = _brain.Serialize();
        File.WriteAllBytes(Path.Combine(Application.persistentDataPath, "brain.bio"), data);
    }
}
```

-----

## 📚 Core API Referenz

Die folgenden Methoden sind in allen Wrappern identisch implementiert und stellen die Schnittstelle zum `bioai` Core dar.

| Methode | Parameter | Beschreibung | Komplexität |
| :--- | :--- | :--- | :--- |
| **`CreateToken`** | `string name`, `ulong cluster` | Erzeugt einen deterministischen 64-Bit Hash. Identische Strings erzeugen immer identische Tokens. | O(1) |
| **`Think`** | `params ulong[] inputs` | **Core Loop.** Verarbeitet sensorische Inputs und liefert das Token der optimalen Aktion zurück. | O(n) |
| **`Learn`** | `float reward`, `ulong action` | Feedback-Loop. `reward` ( -1.0 bis +1.0) verstärkt oder hemmt die neuronale Verbindung zur letzten Aktion. | O(1) |
| **`ForceInstinct`** | `input`, `action`, `weight` | **Hard-Wiring.** Erstellt eine permanente Verbindung, die nicht durch Training überschrieben, aber moduliert werden kann. | O(1) |
| **`Serialize`** | *-* | Extrahiert den gesamten neuronalen Zustand in ein `byte[]` für Persistenz oder Netzwerkübertragung. | O(n) |
| **`Dispose`** | *-* | Gibt den unmanaged Speicher (C++ Heap) frei. **Muss** aufgerufen werden. | - |

-----

## 🔒 Best Practices (Gold Standard)

1.  **Deterministische Tokens:** Verwenden Sie `BioClusters.CreateToken` nur während der Initialisierungsphase und cachen Sie die `ulong`-Werte. Vermeiden Sie String-Hashing im "Hot Path" (z.B. `Update`-Loops).
2.  **Thread Safety:** Eine `BioBrain`-Instanz ist **nicht** thread-safe. Wenn Sie Multithreading benötigen, verwenden Sie eine Instanz pro Thread oder implementieren Sie Locking (`lock(_brain)`) im C\#-Wrapper.
3.  **Error Handling:** Fangen Sie `DllNotFoundException` immer ab. Dies ist der häufigste Fehler bei Deployment auf neuen Systemen.
4.  **Memory Leak Prevention:** Da `BioBrain` unmanaged Memory allokiert, ist die Verwendung von `using(...)` oder explizitem `.Dispose()` im `OnDestroy/OnApplicationQuit` zwingend erforderlich.

-----

<br>

<div align="center"\

### BrainAI Solutions

**BrainAI** - *Intelligence everywhere.*
Developed by **Sascha A. Köhne (winemp83)**
Product: **BioAI 0.7.6 (Industrial Closed Feature)**
📧 [koehne83@googlemail.com](mailto:koehne83@googlemail.com)

© 2025 BrainAI / Sascha A. Köhne. All rights reserved.
*BioAI Core Technology is a proprietary system.*

[ Documentation ](https://github.com/onkel83/BrainAI-BioAI/tree/main/DOCS) • [ API Reference ](https://github.com/onkel83/BrainAI-BioAI/blob/main/DOCS/API_REFERENCE.md) • [ Enterprise Support ](mailto:koehne83@googlemail.com)