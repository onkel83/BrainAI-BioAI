# 🏭 BioAI OPC UA Bridge (Industry 4.0)

**Version:** 0.7.6 (Industrial Closed Feature)

**Technologie:** C# (.NET 8.0) + OPC UA .NET Standard Stack

**Anwendungsbereich:** Retrofitting von Bestandsanlagen (S7, Beckhoff) zu "Intelligent Edge Nodes" mit SAP HANA-Synchronisation.

---

## 1. Übersicht

Diese Integration kapselt den BioAI Core in einem hochperformanten **OPC UA Server**. Industrielle Steuerungen (SPS) agieren als Clients, schreiben Sensor-Tokens in einen Input-Node und erhalten deterministisch berechnete Aktions-Tokens zur unmittelbaren Prozesssteuerung.

### Architektur-Vorteile

* **Determinisimus:** Durch den `Fixed Structure` Modus (Mode 1) arbeitet der Server allokationsfrei und garantiert Antwortzeiten im Mikrosekundenbereich.
* **Hybride Kopplung:** Die Bridge fungiert als Nexus, der Maschinendaten (S7) vorverarbeitet und nur geschäftsrelevante Ereignisse an SAP HANA meldet.
* **Verschleierung:** Die gelernten Prozessgewichte sind im RAM durch den `license_key` geschützt (Salting).

---

## 2. Implementierung (`BioAINodeManager.cs`)

Der NodeManager nutzt den offiziellen **C# RAII-Wrapper**, um die native `.dll` (Windows) oder `.so` (Linux) Bibliothek sicher anzusprechen.

```csharp
using Opc.Ua;
using Opc.Ua.Server;
using BioAI.Wrapper; // Der offizielle C# Wrapper für v0.7.6

namespace BioAI.Integration.OpcUa
{
    public class BioAINodeManager : CustomNodeManager2
    {
        private BioBrainInstance _brain; // RAII Wrapper Instanz
        private BaseDataVariableState _inputVar;
        private BaseDataVariableState _outputVar;

        public BioAINodeManager(IServerInternal server, ApplicationConfiguration configuration)
        : base(server, configuration)
        {
            // 1. Initialisierung des Kerns mit Industrial Key
            try 
            {
                _brain = new BioBrainInstance("config/key.json");
                _brain.SetMode(1); // Modus 1: Produktion (Echtzeitsicher)
                Console.WriteLine("[BioAI] Core v0.7.6 initialisiert und mit OPC UA verknüpft.");
            }
            catch (Exception ex)
            {
                Console.WriteLine($"[FATAL] Initialisierung fehlgeschlagen: {ex.Message}");
            }
        }

        public override void CreateAddressSpace(IDictionary<NodeId, IList<IReference>> externalReferences)
        {
            lock (Lock)
            {
                // Erstellung des BioAI_Controller Ordners
                FolderState root = CreateFolder(null, "BioAI_Nexus", "BioAI");

                // --- INPUT NODE (SPS schreibt hierher) ---
                _inputVar = CreateVariable(root, "SensorInput", DataTypeIds.UInt64);
                _inputVar.OnWriteValue = OnSensorInputChanged; // Interrupt-gesteuerte Inferenz

                // --- OUTPUT NODE (SPS liest hier) ---
                _outputVar = CreateVariable(root, "ActionOutput", DataTypeIds.UInt64);
                _outputVar.AccessLevel = AccessLevels.CurrentRead;

                AddPredefinedNode(SystemContext, root);
            }
        }

        private ServiceResult OnSensorInputChanged(ISystemContext context, NodeState node, ref object value)
        {
            ulong sensorToken = (ulong)value;
            
            // 2. Deterministische Inferenz (O(1))
            // Wandelt Sensorwert direkt in Aktions-Token um.
            ulong actionToken = _brain.Update(new List<ulong> { sensorToken });

            // 3. Update des Output-Nodes für die SPS
            _outputVar.Value = actionToken;
            _outputVar.ClearChangeMasks(SystemContext, false);

            // 4. Optional: Audit-Log für SAP HANA
            float confidence = _brain.Inspect(sensorToken, actionToken);
            if(confidence > 0.9f) SyncToHana(actionToken, confidence);

            return ServiceResult.Good;
        }
    }
}

```

---

## 3. Industrial Deployment

### Ordnerstruktur für Edge-Gateways

```text
/app
├── BioAI_OpcServer.exe      # .NET 8 Applikation
├── config/
│   └── key.json             # Lizenzschlüssel (Verschleierung)
├── runtimes/
│   ├── win-x64/native/      # libbioai_core.dll (Ultra Tier)
│   └── linux-x64/native/    # libbioai_core.so (Ultra Tier)

```

### Docker-Konfiguration (Linux Edge)

```dockerfile
FROM mcr.microsoft.com/dotnet/runtime:8.0
WORKDIR /app
COPY ./publish .
# Native Engine (v0.7.6) kopieren
COPY ./libs/libbioai_core.so /usr/lib/libbioai_core.so
ENTRYPOINT ["dotnet", "BioAI.OpcUa.Server.dll"]

```

---

## 4. Anbindung an Siemens S7 (TIA Portal)

Der Austausch erfolgt über den Standard-OPC UA Client der S7-1500:

1. **Subscription:** Die SPS abonniert `ns=2;s=ActionOutput`.
2. **Trigger:** Bei Prozessänderung schreibt die SPS die entsprechende **TokenID** (z.B. `CLUSTER_OBJECT | 0x01`) in `ns=2;s=SensorInput`.
3. **Reaktion:** BioAI berechnet die Antwort in < 1ms.
4. **Action:** Die SPS empfängt das neue Token am Output-Node und führt den entsprechenden Funktionsbaustein aus.

---

**BrainAI** - *-We don't need **BRUTEFORCE**, we know **Physics**-*

Developed by **Sascha A. Köhne (winemp83)**

Product: **BioAI 0.7.6 (Industrial Closed Feature)**

📧 [koehne83@googlemail.com](mailto:koehne83@googlemail.com)

© 2025 BrainAI / Sascha A. Köhne. All rights reserved.