# BioAI VB.NET Industrial Integration 🏭

**Version:** 0.7.6 (Industrial Closed Feature)

**Platform:** .NET Framework 4.8 / .NET 6+ / .NET 8+

**Use Case:** HMI Panels, SCADA Systems, Legacy Industrial PCs

**Backend:** Ultra / SmartHome / IoT (Interchangeable)

---

## 1. Overview

The VB.NET wrapper allows for the seamless integration of high-performance AI into classic industrial environments. It is designed to operate **without external dependencies** (NuGet-free), allowing it to be integrated directly into existing HMI projects via a single source file.

### Key Features

* **Legacy Support:** Runs on older industrial Windows PCs (Windows 7, Windows 10 IoT, Embedded).
* **Deterministic Disposal:** Implements the `IDisposable` pattern to ensure that unmanaged C-memory is released immediately—crucial for 24/7 industrial stability.
* **Bit-Exact Hashing:** Token generation is identical across C++, C#, and VB.NET. A token generated on a desktop management system will be perfectly understood by the PLC-level (IoT Tier).

---

## 2. Installation

### Step A: Import the Wrapper

Copy the `BioAI.vb` file into your project.

### Step B: Native Engine Setup

The wrapper looks for `bioai.dll` in the application execution directory (`bin\Debug` or `bin\Release`).

1. Pick your edition (e.g., `BioAI_SmartHome.dll` for HMI panels).
2. **Rename the file** to `bioai.dll`.
3. Place it alongside your application's `.exe` file.

---

## 3. Quick Start (HMI Example)

**Scenario:** An HMI panel monitors a motor. It must respond *instantly* to overheating while *analyzing* vibration patterns.

```vb
Imports System
Imports System.IO
Imports BrainAI.BioAI

Module Program
    Sub Main()
        ' 1. Seed/License Key for reproducibility
        Dim licenseKey As UInt64 = &HCAFEBABEC0FFEEUL

        ' Using "Using" ensures deterministic memory cleanup (Native free)
        Using brain As New BioBrain(licenseKey)
            
            Console.WriteLine("BioAI Engine ready.")

            ' 2. Define Vocabulary (Name -> 64-bit Hash)
            Dim SENSOR_TEMP As UInt64 = BioClusters.CreateToken("TEMP_HIGH", BioClusters.OBJECT_C)
            Dim ACTION_STOP As UInt64 = BioClusters.CreateToken("EMERGENCY_STOP", BioClusters.ACTION_C)

            ' 3. Inject Instinct (Safety Layer)
            ' "If temperature is critical, then STOP." (Weight 1.0 = Immutable Law)
            brain.ForceInstinct(SENSOR_TEMP, ACTION_STOP, 1.0F)

            ' 4. Cyclic Process (e.g., inside an HMI Loop or Timer)
            ' Simulation: Sensor reports critical heat
            Dim currentInputs As UInt64() = { SENSOR_TEMP }
            
            ' Inference/Thinking (Takes < 1ms, Complexity O(1))
            Dim decision As UInt64 = brain.Think(currentInputs)

            ' 5. Execution
            If decision = ACTION_STOP Then
                Console.WriteLine("ALERT: Machine stopped due to temperature!")
                ' Send command to PLC...
                
                ' Reinforcement: Confirm the decision was correct
                brain.Learn(1.0F, decision)
            End If

            ' Optional: Secure state for a persistent restart
            Dim backup As Byte() = brain.Serialize()
            File.WriteAllBytes("machine_state.bin", backup)

        End Using ' API_FreeBrain is called automatically here
    End Sub
End Module

```

---

## 4. Hardware Scaling (Tiers)

Industrial VB.NET projects often run on hardware with limited resources. You can control the footprint by swapping the native DLL without changing a single line of VB code:

* **`BioAI_IoT.dll`**: Recommended for older HMI panels with limited RAM (< 512 MB). (Max 255 Neurons).
* **`BioAI_SmartHome.dll`**: The standard for modern Industrial PCs. (Max 65,535 Neurons).
* **`BioAI_Ultra.dll`**: Reserved for high-end control systems and localized servers.

---

## 5. API Reference

| Method | Description |
| --- | --- |
| `Think(inputs)` | Processes perception and returns the resulting Action TokenID in ****. |
| `Simulate(inputs, depth)` | Checks consequences via "What-if" simulation. |
| `Learn(reward, action)` | Reinforces or inhibits synaptic links. |
| `ForceInstinct(...)` | Writes immutable rules directly into the Long-Term Memory (LTM). |
| `Serialize()` | Returns a `Byte()` array (Snapshot) for persistence. |
| `SetMode(mode)` | Swaps between Training and deterministic Production modes. |

---

**BrainAI** - *Intelligence everywhere.* **#WeKnowPhysiks** Developed by **Sascha A. Köhne (winemp83)** Product: **BioAI 0.7.6 (Industrial Closed Feature)** 📧 [koehne83@googlemail.com](mailto:koehne83@googlemail.com)

© 2025 BrainAI / Sascha A. Köhne. All rights reserved.
