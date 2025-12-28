# BioAI OPC UA Bridge (Industry 4.0) 🏭

**Version:** 0.7.6 (Industrial Closed Feature)

**Technology:** C# (.NET 6+) + OPC UA .NET Standard Stack

**Use Case:** Retrofitting existing PLCs (Siemens S7, Beckhoff TwinCAT, Rockwell) with Edge AI.

---

## 1. Overview

The BioAI OPC UA Bridge wraps the **BioAI Core** within an **OPC UA Server**.
Legacy industrial controllers (PLCs) connect to this server as clients. They write sensor data (mapped to TokenIDs) to an **Input Node** and immediately receive a calculated decision from an **Output Node**.

### Architecture & Data Flow

* **PLC (Client):** Writes a `SensorInput` (UInt64 Token).
* **BioAI (Server):** Triggers the `Think()` operation via an interrupt-driven write event.
* **Result:** Updates the `ActionOutput` (UInt64 Token) within microseconds ( complexity).
* **PLC (Client):** Reads the output and executes the corresponding physical command.

---

## 2. Implementation (`BioAINodeManager.cs`)

This NodeManager integrates into the standard [OPC UA .NET Standard Stack](https://github.com/OPCFoundation/UA-.NETStandard), exposing the AI Core to the factory network.

```csharp
using System;
using System.Collections.Generic;
using Opc.Ua;
using Opc.Ua.Server;
using BrainAI.BioAI; // The BioAI C# Wrapper

namespace BioAI.Integration.OpcUa
{
    public class BioAINodeManager : CustomNodeManager2
    {
        private BioBrain _brain;
        private BaseDataVariableState _inputVar;
        private BaseDataVariableState _outputVar;
        
        public BioAINodeManager(IServerInternal server, ApplicationConfiguration configuration)
        : base(server, configuration)
        {
            SystemContext.NodeIdFactory = this;

            // 1. Initialize BioAI Core (SmartHome or Ultra Tier required)
            try 
            {
                _brain = new BioBrain(0xPLC_SECRET_KEY);
                _brain.SetMode(BioMode.Production); // Freeze learning for deterministic safety
                Console.WriteLine("[BioAI] Core successfully attached to OPC UA Address Space.");
            }
            catch (Exception ex)
            {
                Console.WriteLine($"[FATAL] BioAI Core Initialization Failed: {ex.Message}");
            }
        }

        public override void CreateAddressSpace(IDictionary<NodeId, IList<IReference>> externalReferences)
        {
            lock (Lock)
            {
                // Create Root Folder "BioAI_Controller"
                FolderState root = CreateFolder(null, "BioAI_Controller", "BioAI");
                AddRootNotifier(root);

                // --- INPUT NODE (PLC writes here) ---
                _inputVar = CreateVariable(root, "SensorInput", DataTypeIds.UInt64);
                _inputVar.AccessLevel = AccessLevels.CurrentReadOrWrite;
                _inputVar.Value = (ulong)0;
                
                // Hook into the Write Event for Real-Time Reaction
                _inputVar.OnWriteValue = OnSensorInputChanged;

                // --- OUTPUT NODE (PLC reads here) ---
                _outputVar = CreateVariable(root, "ActionOutput", DataTypeIds.UInt64);
                _outputVar.AccessLevel = AccessLevels.CurrentRead; // Read-Only for the PLC
                _outputVar.Value = (ulong)0;

                AddPredefinedNode(SystemContext, root);
            }
        }

        /// <summary>
        /// Real-Time Trigger: Executes immediately when the PLC writes a value.
        /// </summary>
        private ServiceResult OnSensorInputChanged(ISystemContext context, NodeState node, NumericRange indexRange, QualifiedName dataEncoding, ref object value)
        {
            try 
            {
                ulong sensorToken = (ulong)value;
                
                if (sensorToken != 0 && _brain != null)
                {
                    // 2. Execute O(1) Inference
                    ulong actionToken = _brain.Think(sensorToken);

                    // 3. Update Output Node immediately for the PLC to read
                    _outputVar.Value = actionToken;
                    _outputVar.ClearChangeMasks(SystemContext, false); // Push update to subscribers
                }
                
                return ServiceResult.Good;
            }
            catch (Exception ex)
            {
                return new ServiceResult(StatusCodes.BadInternalError, ex.Message);
            }
        }
    }
}

```

---

## 3. Deployment & Connectivity

### Industrial PC (Windows/Linux)

Deploy the compiled server alongside the native `bioai.dll` (Windows) or `libbioai.so` (Linux). This setup is ideal for localized Edge Gateways sitting between the shop floor and the corporate network.

### Dockerized Edge (Container)

For modern infrastructure, use the following Docker configuration to deploy the bridge:

```dockerfile
FROM mcr.microsoft.com/dotnet/runtime:6.0
COPY ./publish/ /app/
# Ensure the Linux .so Native Library is present
COPY ./libs/libbioai.so /app/
WORKDIR /app
ENTRYPOINT ["dotnet", "BioAIOpcUaServer.dll"]

```

---

## 4. PLC Integration Example (Siemens S7-1500)

Using the **OPC UA Client** instructions in TIA Portal:

1. **Establish Connection:** Connect to `opc.tcp://<edge-gateway-ip>:4840`.
2. **Write Trigger:** Use `UA_Write` to send the Sensor TokenID to `ns=2;s=SensorInput`.
3. **Read Action:** Use `UA_Read` to retrieve the Action TokenID from `ns=2;s=ActionOutput`.
4. **Logic Mapping:** Map the returned ID (e.g., `0xA1...`) to a local function block (e.g., `StopConveyor()`).

---

**BrainAI** - *Intelligence everywhere.* **#WeKnowPhysiks** Developed by **Sascha A. Köhne (winemp83)** Product: **BioAI 0.7.6 (Industrial Closed Feature)** 📧 [koehne83@googlemail.com](mailto:koehne83@googlemail.com)

© 2025 BrainAI / Sascha A. Köhne. All rights reserved.
