# BioAI OPC UA Bridge (Industry 4.0) 🏭

**Version:** 0.7.6 (Industrial Closed Feature)
**Technology:** C# (.NET 6+) + OPC UA .NET Standard Stack
**Use Case:** Retrofitting existing PLCs (Siemens S7, Beckhoff TwinCAT) with Edge AI.

---

## 1. Overview

This integration wraps the BioAI Core in an **OPC UA Server**.
Legacy industrial controllers (PLCs) can connect to this server as clients. They write sensor tokens to an Input-Node and immediately receive the calculated Action-Token from an Output-Node.

### Architecture
* **SPS (Client):** Writes `SensorInput` (UInt64).
* **BioAI (Server):** Triggers `Think()` on write event (Interrupt-driven).
* **Result:** Updates `ActionOutput` (UInt64) within microseconds.

---

## 2. Implementation (`BioAIServer.cs`)

This NodeManager integrates seamlessly into the [OPC UA .NET Standard Reference Server](https://github.com/OPCFoundation/UA-.NETStandard).

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

            // 1. Initialize BioAI Core (IoT or SmartHome Tier DLL required)
            try 
            {
                _brain = new BioBrain(0xPLC_SECRET_KEY);
                _brain.SetMode(BioMode.Production); // Freeze learning for safety
                Console.WriteLine("[BioAI] Core attached to OPC UA Address Space.");
            }
            catch (Exception ex)
            {
                Console.WriteLine($"[FATAL] BioAI Init Failed: {ex.Message}");
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
                _inputVar.UserAccessLevel = AccessLevels.CurrentReadOrWrite;
                _inputVar.Value = (ulong)0;
                
                // Hook into the Write Event for Real-Time Reaction
                _inputVar.OnWriteValue = OnSensorInputChanged;

                // --- OUTPUT NODE (PLC reads here) ---
                _outputVar = CreateVariable(root, "ActionOutput", DataTypeIds.UInt64);
                _outputVar.AccessLevel = AccessLevels.CurrentRead; // Read-Only for PLC
                _outputVar.Value = (ulong)0;

                // Add to Server Address Space
                AddPredefinedNode(SystemContext, root);
            }
        }

        /// <summary>
        /// Real-Time Trigger: Called immediately when PLC writes a value.
        /// </summary>
        private ServiceResult OnSensorInputChanged(ISystemContext context, NodeState node, NumericRange indexRange, QualifiedName dataEncoding, ref object value)
        {
            try 
            {
                ulong sensorToken = (ulong)value;
                
                if (sensorToken != 0 && _brain != null)
                {
                    // 2. O(1) Execution Step
                    ulong actionToken = _brain.Think(sensorToken);

                    // 3. Update Output Node immediately
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

        // --- OPC UA Helpers ---

        private FolderState CreateFolder(NodeState parent, string path, string name)
        {
            FolderState folder = new FolderState(parent);
            folder.SymbolicName = name;
            folder.ReferenceTypeId = ReferenceTypeIds.Organizes;
            folder.TypeDefinitionId = ObjectTypeIds.FolderType;
            folder.NodeId = new NodeId(path, NamespaceIndex);
            folder.BrowseName = new QualifiedName(name, NamespaceIndex);
            folder.DisplayName = new LocalizedText(name);
            folder.WriteMask = AttributeWriteMask.None;
            folder.UserWriteMask = AttributeWriteMask.None;
            folder.EventNotifier = EventNotifiers.None;

            if (parent != null) parent.AddChild(folder);
            return folder;
        }

        private BaseDataVariableState CreateVariable(NodeState parent, string name, NodeId type)
        {
            BaseDataVariableState variable = new BaseDataVariableState(parent);
            variable.SymbolicName = name;
            variable.ReferenceTypeId = ReferenceTypeIds.Organizes;
            variable.TypeDefinitionId = VariableTypeIds.BaseDataVariableType;
            variable.NodeId = new NodeId(name, NamespaceIndex);
            variable.BrowseName = new QualifiedName(name, NamespaceIndex);
            variable.DisplayName = new LocalizedText(name);
            variable.DataType = type;
            variable.ValueRank = ValueRanks.Scalar;
            variable.AccessLevel = AccessLevels.CurrentReadOrWrite;
            variable.UserAccessLevel = AccessLevels.CurrentReadOrWrite;

            if (parent != null) parent.AddChild(variable);
            return variable;
        }
    }
}
````

-----

## 3\. Deployment

### Windows (Industrial PC)

1.  Compile the OPC UA Server project.
2.  Place `BioAI.dll` (SmartHome or Ultra edition) in the output folder.
3.  Run the `.exe` as a Service.

### Docker (Edge Gateway)

```dockerfile
FROM [mcr.microsoft.com/dotnet/runtime:6.0](https://mcr.microsoft.com/dotnet/runtime:6.0)
COPY ./bin/Release/net6.0/publish/ /app/
# Copy the Linux .so Native Library
COPY ./libs/libbioai.so /app/
WORKDIR /app
ENTRYPOINT ["dotnet", "OpcUaServer.dll"]
```

-----

## 4\. Usage in PLC (Example: Siemens S7-1500)

Using the **OPC UA Client** block in TIA Portal:

1.  **Connect** to `opc.tcp://<gateway-ip>:4840`.
2.  **Write** Sensor Token ID to `ns=2;s=SensorInput`.
3.  **Read** Action Token ID from `ns=2;s=ActionOutput`.
4.  **Map** the Action ID to a function block call (e.g., `0xA1...` -\> `StartConveyor()`).

-----

**BrainAI** - *-We don't need **BRUTEFORCE**, we know **Physiks**-*</br>
Developed by **Sascha A. Köhne (winemp83)**</br>
Product: **BioAI 0.7.6 (Industrial Closed Feature)**</br>
📧 [koehne83@googlemail.com](mailto:koehne83@googlemail.com)

&copy; 2025 BrainAI / Sascha A. Köhne. All rights reserved.