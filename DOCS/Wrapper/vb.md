# BioAI VB.NET Industrial Integration 🏭

**Version:** 0.7.5
**Platform:** .NET Framework 4.8 / .NET 6+
**Use Case:** HMI Panels, SCADA Systems, Legacy Industrial PCs
**Backend:** Ultra / SmartHome / IoT (Austauschbar)

---

## 1. Übersicht

Der VB.NET Wrapper ermöglicht die Integration von High-Performance KI in klassische Industrie-Umgebungen. Er ist so konzipiert, dass er **ohne externe Abhängigkeiten** (NuGet) auskommt und direkt in bestehende HMI-Projekte eingebunden werden kann.

### Features
* **Legacy Support:** Läuft auch auf älteren Windows-Industrie-PCs (Win 7/10/IoT).
* **Deterministic Disposal:** Implementiert das `IDisposable`-Pattern, um sicherzustellen, dass C-Speicher sofort freigegeben wird (wichtig für 24/7 Betrieb).
* **Bit-Exact Hashing:** Die Token-Generierung ist identisch zu C++/C#. Ein Token, der auf dem Desktop generiert wurde, wird von der SPS (IoT Tier) verstanden.

---

## 2. Installation

### Schritt A: Wrapper Einbinden
Kopieren Sie die Datei `BioAI.vb` in Ihr Projekt.

### Schritt B: Native Engine
Der Wrapper sucht nach `bioai.dll` im Ausführungsverzeichnis (`bin/Debug` oder `bin/Release`).

1.  Wählen Sie Ihre Edition (z.B. `BioAI_SmartHome.dll` für Panels).
2.  **Benennen Sie die Datei um** in `bioai.dll`.
3.  Legen Sie sie neben die `.exe` Datei Ihrer Anwendung.

---

## 3. Quick Start (HMI Beispiel)

Ein typisches Szenario: Ein HMI-Panel überwacht einen Motor und soll bei Überhitzung *sofort* reagieren, aber bei Vibration erst *analysieren*.

```vb
Imports System
Imports BrainAI.BioAI

Module Program
    Sub Main()
        ' 1. Seed für Reproduzierbarkeit
        Dim licenseKey As UInt64 = &HCAFEBABEC0FFEEUL

        ' Nutzung von "Using" garantiert Speicherfreigabe (wie RAII in C++)
        Using brain As New BioBrain(licenseKey)
            
            Console.WriteLine("BioAI Engine ready.")

            ' 2. Vokabular definieren
            Dim SENSOR_TEMP As UInt64 = BioClusters.CreateToken("TEMP_HIGH", BioClusters.OBJECT_C)
            Dim ACTION_STOP As UInt64 = BioClusters.CreateToken("EMERGENCY_STOP", BioClusters.ACTION_C)

            ' 3. Instinkt (Safety Layer)
            ' "Wenn Temperatur kritisch, dann STOP." (Gewicht 1.0 = Gesetz)
            brain.ForceInstinct(SENSOR_TEMP, ACTION_STOP, 1.0F)

            ' 4. Zyklischer Prozess (z.B. in HMI Loop)
            ' Simulation: Sensor meldet Hitze
            Dim currentInputs As UInt64() = { SENSOR_TEMP }
            
            ' Denken (Dauert < 1ms)
            Dim decision As UInt64 = brain.Think(currentInputs)

            ' 5. Handeln
            If decision = ACTION_STOP Then
                Console.WriteLine("ALERT: Machine stopped due to temperature!")
                ' SPS-Befehl senden...
                
                ' Lernen: Entscheidung war richtig
                brain.Learn(1.0F, decision)
            End If

            ' Optional: Zustand sichern für Neustart
            Dim backup As Byte() = brain.Serialize()
            File.WriteAllBytes("machine_state.bin", backup)

        End Using ' Hier wird API_FreeBrain automatisch aufgerufen
    End Sub
End Module
````

-----

## 4\. Architektur-Wahl (Tiers)

In VB.NET Projekten wird oft Hardware mit begrenzten Ressourcen verwendet. Sie können die Leistung steuern, indem Sie die DLL austauschen:

  * **`BioAI_IoT.dll`**: Nutzen Sie diese Version für HMI-Panels mit wenig RAM (\< 512 MB). (Max 255 Neuronen).
  * **`BioAI_SmartHome.dll`**: Standard für Industrie-PCs. (Max 65k Neuronen).
  * **`BioAI_Ultra.dll`**: Nur für Leitsysteme / Server.

Der VB.NET Code muss dafür **nicht** geändert werden.

-----

## 5\. API Referenz

| Methode | Beschreibung |
| :--- | :--- |
| `Think(inputs)` | Verarbeitet Signale und liefert TokenID. |
| `Simulate(inputs, depth)` | Prüft Konsequenzen ("Was wäre wenn?"). |
| `Learn(reward, action)` | Bestärkt oder hemmt Verbindungen. |
| `ForceInstinct(...)` | Schreibt Regeln direkt ins Langzeitgedächtnis. |
| `Serialize()` | Gibt ein `Byte()` Array zurück (Snapshot). |
| `Deserialize(data)` | Lädt einen Snapshot. |

-----

**BrainAI** - *Intelligence everywhere.*
Developed by **Sascha A. Köhne (winemp83)**
Product: **BioAI 0.7.5 (Industrial Closed Feature)**
📧 [koehne83@googlemail.com](mailto:koehne83@googlemail.com)

© 2025 BrainAI / Sascha A. Köhne. All rights reserved.
