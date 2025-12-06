/*
 * BIOAI OPC UA SERVER ADAPTER
 * Macht BioAI kompatibel zu Siemens, Beckhoff & Co.
 */

using System;
using System.Collections.Generic;
using System.Threading;
using Opc.Ua;
using Opc.Ua.Configuration;
using Opc.Ua.Server;
using BrainAI.BioAI; // Unser C# Wrapper

namespace BioAI.Integration.OpcUa
{
    // Minimaler OPC UA Server Wrapper
    public class BioAINodeManager : CustomNodeManager2
    {
        private BioBrain _brain;
        private BaseDataVariableState _inputVar;
        private BaseDataVariableState _outputVar;
        
        public BioAINodeManager(IServerInternal server, ApplicationConfiguration configuration)
        : base(server, configuration)
        {
            // BioAI Kern starten
            _brain = new BioBrain(12345);
            _brain.SetMode(BioMode.Production); // Safe Mode für Industrie
            Console.WriteLine("BioAI Core attached to OPC UA.");
        }

        public override void CreateAddressSpace(IDictionary<NodeId, IList<IReference>> externalReferences)
        {
            lock (Lock)
            {
                // Root Ordner
                FolderState root = CreateFolder(null, "BioAI_Controller", "BioAI");
                AddRootNotifier(root);

                // --- INPUT NODE (Hier schreibt die SPS rein) ---
                _inputVar = CreateVariable(root, "SensorInput", "Sensor TokenID", DataTypeIds.UInt64, ValueRanks.Scalar);
                _inputVar.AccessLevel = AccessLevels.CurrentReadOrWrite;
                _inputVar.UserAccessLevel = AccessLevels.CurrentReadOrWrite;
                _inputVar.Value = (ulong)0;

                // --- OUTPUT NODE (Hier liest die SPS die Entscheidung) ---
                _outputVar = CreateVariable(root, "ActionOutput", "Action TokenID", DataTypeIds.UInt64, ValueRanks.Scalar);
                _outputVar.AccessLevel = AccessLevels.CurrentRead; // Nur lesbar für externe
                _outputVar.Value = (ulong)0;
            }
        }

        // Der "Takt" der SPS (z.B. 100ms)
        private void SimulationTimerCallback(object state)
        {
            try 
            {
                // 1. Wert aus OPC Node lesen (Input von Maschine)
                ulong sensorToken = (ulong)_inputVar.Value;

                if (sensorToken != 0) 
                {
                    // 2. BioAI Denken
                    ulong actionToken = _brain.Think(sensorToken);

                    // 3. Ergebnis in OPC Node schreiben (Output an Maschine)
                    _outputVar.Value = actionToken;
                    
                    // Änderungen publizieren (Push an Clients)
                    _outputVar.ClearChangeMasks(SystemContext, false);
                }
            }
            catch (Exception ex)
            {
                Console.WriteLine($"Cycle Error: {ex.Message}");
            }
        }
        
        // Boilerplate Helper für Nodes...
        private FolderState CreateFolder(NodeState parent, string path, string name) { /* ... */ return new FolderState(parent); }
        private BaseDataVariableState CreateVariable(NodeState parent, string path, string name, NodeId type, int rank) { /* ... */ return new BaseDataVariableState(parent); }
    }
}
