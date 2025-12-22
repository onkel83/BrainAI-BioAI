# BioAI .NET & Unity Integration 🧠

**Version:** 0.7.5 (Industrial Closed Feature)

**Platform:** .NET 6/7/8/9, Unity 2021+, MAUI/Xamarin

**Backend:** Ultra / SmartHome / IoT (Interchangeable)

---

## 1. Architecture: "One Wrapper, Three Engines"

The BioAI C# Wrapper (`BioAI.cs`) is universal. It serves as a bridge to the high-performance native C library. You determine the power of your AI simply by **swapping the DLL file**. The C# code remains identical regardless of the tier.

### Hardware Tiers

| Tier | Native Binary Name | Use Case | Neuron Limit |
| --- | --- | --- | --- |
| **Ultra** | `BioAI_Ultra.dll` | PC, Server, High-End Clusters | **~ 4 Billion** |
| **SmartHome** | `BioAI_SmartHome.dll` | Gateways, Raspberry Pi, HMI | **65,535** |
| **IoT** | `BioAI_IoT.dll` | Wearables, Low-End Sensors | **255** (8-bit) |

---

## 2. Installation & Setup

### Step A: Import the Wrapper

Add the `BioAI.cs` file to your project.

* **Unity:** Place it in `Assets/Scripts/BioAI/`.
* **.NET:** Add it to your project directory or a dedicated `Services` folder.

### Step B: Choose Your Engine (IMPORTANT!)

The wrapper looks for a native binary named `bioai` (e.g., `bioai.dll` or `libbioai.so`).

1. Pick your edition (e.g., `BioAI_Ultra.dll`).
2. **Rename the file** to `bioai.dll`.
3. Place it in the correct search path:
* **Unity:** `Assets/Plugins/x86_64/bioai.dll`
* **.NET Console:** Root directory (set "Copy to Output Directory" to "Copy if newer").



---

## 3. Unity Quick Start 🎮

BioAI does **not** run on the C# Managed Heap (Garbage Collector). Instead, it operates in unmanaged C memory. This guarantees constant frame rates without the dreaded "GC Spikes" common in traditional C# AI implementations.

```csharp
using UnityEngine;
using BrainAI.BioAI;

public class EnemyAI : MonoBehaviour
{
    private BioBrain _brain;
    
    // Cache tokens for maximum performance
    private ulong _tokenPlayerSeen;
    private ulong _tokenAttack;

    void Start()
    {
        // 1. Initialization
        _brain = new BioBrain(0x12345678);

        // 2. Define Vocabulary
        _tokenPlayerSeen = BioClusters.CreateToken("PLAYER_VISIBLE", BioClusters.OBJECT);
        _tokenAttack     = BioClusters.CreateToken("ATTACK", BioClusters.ACTION);

        // 3. Injected Instinct: Attack on sight
        _brain.ForceInstinct(_tokenPlayerSeen, _tokenAttack, 0.9f);
    }

    void Update()
    {
        // 4. Perception
        bool seesPlayer = CheckLineOfSight();
        if (!seesPlayer) return;

        // 5. Decision (Execution time < 0.01ms)
        // Complexity is deterministic O(1)
        ulong decision = _brain.Think(_tokenPlayerSeen);

        // 6. Action
        if (decision == _tokenAttack)
        {
            PerformAttack();
            // 7. Feedback: Reinforce the decision
            _brain.Learn(1.0f, decision);
        }
    }

    // MANDATORY: Explicitly free unmanaged memory
    void OnDestroy()
    {
        _brain.Dispose();
    }
}

```

---

## 4. Best Practices

### 1. Thread Safety

The BioAI Core is **Thread-Safe**. You can offload `Think()` and `Simulate()` calls to **Unity Jobs** or background threads to keep the main thread dedicated to rendering.

### 2. Error Handling

Always catch `DllNotFoundException`. This is the most frequent deployment error, usually caused by a missing library in the target directory.

```csharp
try {
    brain = new BioBrain(seed);
} catch (DllNotFoundException) {
    Debug.LogError("BioAI Core missing! Please verify 'bioai.dll' in Plugins.");
}

```

### 3. Vocabulary Export

For debugging, save the internal vocabulary to a persistent path on shutdown to map 64-bit hashes back to readable strings.

```csharp
BioClusters.DumpVocabulary(Path.Combine(Application.persistentDataPath, "vocab_audit.txt"));

```

---

## 5. API Summary

* `SetMode(BioMode.Production)`: Activates the "Safety Freeze" (Inference only, no dynamic allocation).
* `Serialize()`: Returns a `byte[]`. Perfect for savegames, network sync, or cloud backups.
* `Deserialize(byte[])`: Restores a brain state instantly.

---

**BrainAI** - *Intelligence everywhere.* **#WeKnowPhysiks** Developed by **Sascha A. Köhne (winemp83)** Product: **BioAI 0.7.5 (Industrial Closed Feature)** 📧 [koehne83@googlemail.com](mailto:koehne83@googlemail.com)

© 2025 BrainAI / Sascha A. Köhne. All rights reserved.

