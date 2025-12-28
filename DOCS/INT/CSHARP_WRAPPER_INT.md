# BioAI Integration Guide: C# Interop Wrappers

**Version:** 0.7.6 (Industrial Closed Feature)

**Module:** BrainAI.BioAI Core Interface

**Status:** Enterprise / Gold Release

---

## 📋 Overview

This document defines the technical integration of the native **BioAI Core Library** into .NET-based environments. Three specific runtime environments are supported:

1. **.NET Standard Console / Server** (Windows/Linux Services)
2. **.NET MAUI / Android** (Mobile Embedded Systems)
3. **Unity Engine** (Simulation & Interactive)

All wrappers are based on the same `BioBrain` C# interface but differ significantly in the **deployment of native binaries (.dll, .so)** and **lifecycle management**.

---

## 1. Environment: .NET Console / Windows Server

Used for backend services, test suites, or standalone desktop applications without GUI framework dependencies.

### ⚙️ Deployment Prerequisites

* **Wrapper File:** `BioBrain.cs` (Standard/Console variant)
* **Native Library:**
* Windows: `bioai.dll`
* Linux: `libbioai.so`



**Path Convention:** The native library must be located in the **root directory of the executable** (alongside the `.exe` or entry-point DLL).

### 💻 Implementation Pattern

```csharp
using BrainAI.BioAI;

public class CoreSystem
{
    public void Initialize()
    {
        try 
        {
            // Initialization with License Key (Hex format)
            // Using a 'using' block for deterministic resource cleanup (RAII)
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
            // Critical: Native dependencies missing
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

```

---

## 2. Environment: .NET MAUI / Android

Used for mobile applications or embedded Android systems. Here, the wrapper accesses the Android NDK (Native Development Kit) interface.

### ⚙️ Deployment Prerequisites

* **Wrapper File:** `BioBrain.cs` (MAUI variant)
* **Architecture:** Android requires specific builds for different CPU architectures.

**Project Directory Structure:**
The `.so` files must be marked as **AndroidNativeLibrary** (Build Action).

```text
MyMauiProject/
├── Platforms/
│   └── Android/
│       └── jniLibs/
│           ├── arm64-v8a/
│           │   └── libbioai.so   <-- (Modern devices)
│           ├── armeabi-v7a/
│           │   └── libbioai.so   <-- (Legacy devices)
│           └── x86_64/
│               └── libbioai.so   <-- (Emulator)

```

### 💻 Implementation Pattern

In mobile environments, the lifecycle must be coupled with **App-States** (Resume/Pause) to prevent memory leaks.

```csharp
public class BioService : IDisposable
{
    private BioBrain _brain;

    public void Startup()
    {
        // Lazy loading on first access or explicit startup
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

    // IMPORTANT: Call Dispose when closing the app or service
    public void Dispose()
    {
        _brain?.Dispose();
        _brain = null;
    }
}

```

---

## 3. Environment: Unity Engine (3D/Simulation)

Used for visual simulations, games, or digital twins. Unity manages plugin loading automatically but requires strict paths for I/O operations.

### ⚙️ Deployment Prerequisites

* **Wrapper File:** `BioBrain.cs` (Unity variant)
* **Native Library:** All platform variants belong in the `Plugins` folder.

**Project Directory Structure (Unity Assets):**

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

> **Note:** In the Unity Inspector, ensure that for each DLL/SO under "Select platforms for plugin", the correct platform is checked (e.g., bioai.dll -> only Windows x86_64).

### 💻 Implementation Pattern (MonoBehaviour)

```csharp
using UnityEngine;
using BrainAI.BioAI;
using System.IO;

public class BioAIController : MonoBehaviour
{
    private BioBrain _brain;
    
    // Serialized references for Inspector settings
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
        // Mandatory for clean shutdown of Native threads
        _brain?.Dispose();
    }

    // Persistence must use Application.persistentDataPath
    public void SaveState()
    {
        byte[] data = _brain.Serialize();
        File.WriteAllBytes(Path.Combine(Application.persistentDataPath, "brain.bio"), data);
    }
}

```

---

## 📚 Core API Reference

The following methods are implemented identically across all wrappers and provide the interface to the `bioai` core.

| Method | Parameters | Description | Complexity |
| --- | --- | --- | --- |
| **`CreateToken`** | `string name`, `ulong cluster` | Generates a deterministic 64-bit hash. Identical strings always yield identical tokens. |  |
| **`Think`** | `params ulong[] inputs` | **Core Loop.** Processes sensory inputs and returns the token of the optimal action. |  |
| **`Learn`** | `float reward`, `ulong action` | Feedback loop. `reward` (-1.0 to +1.0) reinforces or inhibits the neural connection. |  |
| **`ForceInstinct`** | `input`, `action`, `weight` | **Hard-Wiring.** Creates a permanent connection that cannot be overridden by training. |  |
| **`Serialize`** | *-* | Extracts the neural state into a `byte[]` for persistence or network transfer. |  |
| **`Dispose`** | *-* | Frees unmanaged memory (C++ Heap). **Must** be called. | - |

---

## 🔒 Best Practices (Gold Standard)

1. **Deterministic Tokens:** Use `BioClusters.CreateToken` during the initialization phase and cache the `ulong` values. Avoid string hashing in the "Hot Path" (e.g., `Update` loops).
2. **Thread Safety:** A `BioBrain` instance is **not** thread-safe. If you require multithreading, use one instance per thread or implement locking (`lock(_brain)`) in the C# wrapper.
3. **Error Handling:** Always catch `DllNotFoundException`. This is the most common error when deploying to new systems.
4. **Memory Leak Prevention:** Since `BioBrain` allocates unmanaged memory, using `using(...)` or explicit `.Dispose()` in `OnDestroy/OnApplicationQuit` is mandatory.

---

**BrainAI** - *Intelligence everywhere.* Developed by **Sascha A. Köhne (winemp83)** Product: **BioAI 0.7.6 (Industrial Closed Feature)** **#WeKnowPhysiks** 📧 [koehne83@googlemail.com](mailto:koehne83@googlemail.com)

© 2025 BrainAI / Sascha A. Köhne. All rights reserved.

---
