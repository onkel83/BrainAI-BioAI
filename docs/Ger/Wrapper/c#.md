# BioAI .NET & Unity Integration 🧠

**Version:** 0.7.6
**Platform:** .NET 6/7/8, Unity 2021+, MAUI/Xamarin
**Backend:** Ultra / SmartHome / IoT (Austauschbar)

---

## 1. Architektur: "One Wrapper, Three Engines"

Der C# Wrapper (`BioAI.cs`) ist universell. Er kommuniziert mit der darunterliegenden C-Bibliothek. Sie bestimmen die Leistungsfähigkeit Ihrer KI, indem Sie **nur die DLL austauschen**. Der C#-Code bleibt identisch.

### Die Engine-Tiers

| Tier | DLL-Name (Original) | Use Case | Neuronen Limit |
| :--- | :--- | :--- | :--- |
| **Ultra** | `BioAI_Ultra.dll` | PC, Server, High-End AI | **4.000.000.000** |
| **SmartHome** | `BioAI_SmartHome.dll` | Raspberry Pi, Gateways | **65.535** |
| **IoT** | `BioAI_IoT.dll` | Wearables, Low-End | **255** (8-Bit) |

---

## 2. Installation & Setup

### Schritt A: Wrapper Einbinden
Fügen Sie die Datei `BioAI.cs` zu Ihrem Projekt hinzu.
* **Unity:** Ordner `Assets/Scripts/BioAI/`
* **.NET:** Einfach ins Projektverzeichnis.

### Schritt B: Engine Wählen (WICHTIG!)
Der Wrapper sucht standardmäßig nach einer Datei namens `bioai` (also `bioai.dll` oder `libbioai.so`).

1.  Wählen Sie Ihre Edition (z.B. `BioAI_Ultra.dll`).
2.  **Benennen Sie die Datei um** in `bioai.dll`.
3.  Platzieren Sie sie korrekt:
    * **Unity:** `Assets/Plugins/x86_64/bioai.dll`
    * **.NET Console:** Direkt neben die `Program.cs` (und "Copy to Output Directory" aktivieren).

---

## 3. Unity Quick Start 🎮

BioAI läuft in Unity **nicht** auf dem Garbage Collector Heap, sondern im unmanaged Memory. Das garantiert konstante Frameraten ohne GC-Spikes.

```csharp
using UnityEngine;
using BrainAI.BioAI;

public class EnemyAI : MonoBehaviour
{
    private BioBrain _brain;
    
    // Tokens cachen (Performance!)
    private ulong _tokenPlayerSeen;
    private ulong _tokenAttack;

    void Start()
    {
        // 1. Initialisierung
        _brain = new BioBrain(0x12345678);

        // 2. Vokabular anlegen
        _tokenPlayerSeen = BioClusters.CreateToken("PLAYER_VISIBLE", BioClusters.OBJECT);
        _tokenAttack     = BioClusters.CreateToken("ATTACK", BioClusters.ACTION);

        // 3. Instinkt: Angriff bei Sichtkontakt
        _brain.ForceInstinct(_tokenPlayerSeen, _tokenAttack, 0.9f);
    }

    void Update()
    {
        // 4. Wahrnehmung
        bool seesPlayer = CheckLineOfSight();
        if (!seesPlayer) return;

        // 5. Entscheidung (Dauert < 0.01ms)
        ulong decision = _brain.Think(_tokenPlayerSeen);

        // 6. Handlung
        if (decision == _tokenAttack)
        {
            PerformAttack();
            // 7. Lernen: War der Angriff erfolgreich?
            _brain.Learn(1.0f, decision);
        }
    }

    // WICHTIG: Speicher freigeben!
    void OnDestroy()
    {
        _brain.Dispose();
    }
}
````

-----

## 4\. Best Practices

### 1\. Thread Safety

Der Wrapper ist **Thread-Safe**. Sie können `Think()` und `Simulate()` problemlos in **Unity Jobs** oder Background-Threads ausführen, um den Main-Thread zu entlasten.

### 2\. Exception Handling

Fangen Sie `DllNotFoundException` ab. Dies ist der häufigste Fehler (falscher Pfad zur DLL).

```csharp
try {
    brain = new BioBrain(seed);
} catch (Exception ex) {
    Debug.LogError("BioAI Core fehlt! Bitte 'bioai.dll' in Plugins prüfen.");
}
```

### 3\. Vokabular Export

Für Debugging-Zwecke sollten Sie das interne Vokabular beim Beenden speichern:

```csharp
BioClusters.DumpVocabulary(Path.Combine(Application.persistentDataPath, "vocab.txt"));
```

-----

## 5\. API Übersicht

  * `SetMode(BioMode.Production)`: Aktiviert den TÜV-sicheren Modus (kein Lernen, kein Malloc).
  * `Serialize()`: Gibt ein `byte[]` zurück. Ideal zum Speichern in Savegames oder Datenbanken.
  * `Deserialize(byte[])`: Lädt ein Gehirn exakt im gleichen Zustand wieder.

-----

**BrainAI** - *-We don't need **BRUTEFORCE**, we know **Physiks**-*</br>
Developed by **Sascha A. Köhne (winemp83)**</br>
Product: **BioAI 0.7.6 (Industrial Closed Feature)**</br>
📧 [koehne83@googlemail.com](mailto:koehne83@googlemail.com)

&copy; 2025 BrainAI / Sascha A. Köhne. All rights reserved.
