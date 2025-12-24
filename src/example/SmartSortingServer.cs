using System;
using System.Collections.Generic;
using Opc.Ua;
using Opc.Ua.Server;
using BrainAI.BioAI; // BioAI C# Wrapper

namespace BioAI.OPCUA.SortingDemo
{
    public class SortingNodeManager : CustomNodeManager2
    {
        private BioBrain _brain;
        private BaseDataVariableState _inputNode;
        private BaseDataVariableState _outputNode;

        // --- 1. TOKEN DEFINITIONEN (Die 6 Schritte der Sortierung) ---
        private static readonly ulong T_SCAN_BARCODE = BioClusters.CreateToken("SCAN_BARCODE", BioClusters.ACTION);
        private static readonly ulong T_WEIGHT_CHECK = BioClusters.CreateToken("WEIGHT_CHECK", BioClusters.ACTION);
        private static readonly ulong T_SIZE_CHECK   = BioClusters.CreateToken("SIZE_CHECK",   BioClusters.ACTION);
        private static readonly ulong T_ROUTE_A      = BioClusters.CreateToken("ROUTE_TO_GATE_A", BioClusters.ACTION);
        private static readonly ulong T_ROUTE_B      = BioClusters.CreateToken("ROUTE_TO_GATE_B", BioClusters.ACTION);
        private static readonly ulong T_LOG_EXIT     = BioClusters.CreateToken("LOG_EXIT_STATUS", BioClusters.ACTION);

        // Sensoren & Sicherheit
        private static readonly ulong T_HAND_DETECTED = BioClusters.CreateToken("SENSOR_HAND_IN_SORTER", BioClusters.OBJECT);
        private static readonly ulong T_EMERGENCY_STOP = BioClusters.CreateToken("REFLEX_STOP", BioClusters.REFLEX);

        public SortingNodeManager(IServerInternal server, ApplicationConfiguration configuration)
            : base(server, configuration)
        {
            InitializeBioAI();
        }

        private void InitializeBioAI()
        {
            // Initialisierung des Gehirns mit Sicherheits-Reflexen
            _brain = new BioBrain(0xOPC_UA_2025);
            _brain.SetMode(BioMode.Training);

            // Ebene 1: Instinkte - Wenn Hand erkannt, dann SOFORT-STOPP
            _brain.ForceInstinct(T_HAND_DETECTED, T_EMERGENCY_STOP, 1.0f);

            // Produktion einfrieren für maximale Stabilität
            _brain.SetMode(BioMode.Production);
            
            // 6-Schritt-Sequenz als Standard-Prozess laden
            var sequence = new[] { T_SCAN_BARCODE, T_WEIGHT_CHECK, T_SIZE_CHECK, T_ROUTE_A, T_ROUTE_B, T_LOG_EXIT };
            _brain.LoadPlan(sequence, strict: true);
        }

        public override void CreateAddressSpace(IDictionary<NodeId, IList<IReference>> externalReferences)
        {
            lock (Lock)
            {
                // Haupt-Ordner für die SPS-Kommunikation
                FolderState root = new FolderState(null) {
                    SymbolicName = "SortingAI",
                    NodeId = new NodeId("SortingAI", NamespaceIndex),
                    DisplayName = new LocalizedText("BioAI Sorting Controller")
                };

                // --- INPUT NODE (SPS schreibt hierher) ---
                _inputNode = CreateVariable(root, "PLC_Sensor_Input", DataTypeIds.UInt64);
                _inputNode.AccessLevel = AccessLevels.CurrentReadOrWrite;
                // Interrupt-basiertes Event: Bei jedem Schreibvorgang der SPS "denkt" die KI
                _inputNode.OnWriteValue = OnPlcInputReceived;

                // --- OUTPUT NODE (SPS liest von hier) ---
                _outputNode = CreateVariable(root, "AI_Action_Output", DataTypeIds.UInt64);
                _outputNode.AccessLevel = AccessLevels.CurrentRead;

                AddPredefinedNode(SystemContext, root);
            }
        }

        /// <summary>
        /// Real-Time Trigger: Wird aufgerufen, sobald die SPS ein Sensor-Token sendet.
        /// </summary>
        private ServiceResult OnPlcInputReceived(ISystemContext context, NodeState node, NumericRange indexRange, QualifiedName dataEncoding, ref object value)
        {
            ulong plcToken = (ulong)value;

            // KI berechnet die nächste Aktion in O(1) Zeitkomplexität
            ulong action = _brain.Think(plcToken);

            // Ergebnis sofort in den Output-Node schreiben, damit die SPS reagieren kann
            _outputNode.Value = action;
            _outputNode.ClearChangeMasks(SystemContext, false);

            if (action == T_EMERGENCY_STOP) {
                Console.WriteLine("!!! [OPC UA] REFLEX STOPP AUSGELÖST !!!");
            }

            return ServiceResult.Good;
        }

        // Hilfsmethode zur Erstellung von OPC UA Variablen
        private BaseDataVariableState CreateVariable(NodeState parent, string name, NodeId type)
        {
            var variable = new BaseDataVariableState(parent) {
                SymbolicName = name,
                NodeId = new NodeId(name, NamespaceIndex),
                BrowseName = new QualifiedName(name, NamespaceIndex),
                DisplayName = new LocalizedText(name),
                DataType = type,
                ValueRank = ValueRanks.Scalar
            };
            if (parent != null) parent.AddChild(variable);
            return variable;
        }
    }
}
