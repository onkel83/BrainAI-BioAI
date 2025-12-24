using System;
using System.Collections.Generic;
using BrainAI.BioAI; // BioAI C# Wrapper

namespace BioAI.RosDemo
{
    // Simulation einer ROS 2 Node Schnittstelle
    public class RosWeldingNode : IDisposable
    {
        private BioBrain _brain;

        // --- 1. TOKEN DEFINITIONEN (Schweißpfad & Sicherheit) ---
        // Aktionen für die 20mm Linie (5mm Schritte zur Steuerung)
        private static readonly ulong T_WELD_START = BioClusters.CreateToken("WELD_P0_START", BioClusters.ACTION);
        private static readonly ulong T_WELD_P1    = BioClusters.CreateToken("WELD_P1_5MM", BioClusters.ACTION);
        private static readonly ulong T_WELD_P2    = BioClusters.CreateToken("WELD_P2_10MM", BioClusters.ACTION);
        private static readonly ulong T_WELD_P3    = BioClusters.CreateToken("WELD_P3_15MM", BioClusters.ACTION);
        private static readonly ulong T_WELD_END   = BioClusters.CreateToken("WELD_P4_20MM_END", BioClusters.ACTION);

        // Sensoren & Reflexe
        private static readonly ulong T_ROS_HAND_SCAN = BioClusters.CreateToken("ROS_TOPIC_HAND_DETECTED", BioClusters.OBJECT);
        private static readonly ulong T_EMERGENCY_STOP = BioClusters.CreateToken("WELD_EMERGENCY_STOP", BioClusters.REFLEX);

        public RosWeldingNode()
        {
            _brain = new BioBrain(0x2025_5446); // "WELD SAFE" Key
            SetupRosSafety();
        }

        private void SetupRosSafety()
        {
            _brain.SetMode(BioMode.Training); // Trainingsmodus für Wissensinjektion

            // Ebene 1: Instinkte - Der Sicherheits-Reflex (Hand -> Stop)
            // Ein Reflex überschreibt in der bio_think_logic jede andere Planung.
            _brain.ForceInstinct(T_ROS_HAND_SCAN, T_EMERGENCY_STOP, 1.0f);

            _brain.SetMode(BioMode.Production); // Einfrieren für deterministische Ausführung
        }

        /// <summary>
        /// Simuliert den ROS 2 Callback-Loop
        /// </summary>
        public void ExecuteWeldingTask()
        {
            // 20mm Schweißlinie als Sequenz laden
            var path20mm = new List<ulong> { T_WELD_START, T_WELD_P1, T_WELD_P2, T_WELD_P3, T_WELD_END };
            _brain.LoadPlan(path20mm.ToArray(), strict: true); 

            Console.WriteLine("[ROS2] Schweiß-Task (20mm) gestartet...");

            while (_brain.GetPlanStep() != -1)
            {
                // In ROS 2: Abfrage der Topics (z.B. Laser-Scanner oder Kamera)
                bool handInZone = CheckRosLaserScanner();
                
                List<ulong> inputs = new List<ulong>();
                if (handInZone) inputs.Add(T_ROS_HAND_SCAN);

                // BioAI entscheidet basierend auf ROS-Inputs
                ulong decision = _brain.Think(inputs.ToArray());

                if (decision == T_EMERGENCY_STOP)
                {
                    PublishRosStop(); // Sende Stopp-Signal an Roboter-Hardware
                    _brain.AbortPlan(); // Breche die 20mm Sequenz ab
                    break;
                }

                PublishRosJointCommand(decision); // Sende Bewegungsbefehl
                System.Threading.Thread.Sleep(200); // Pfad-Verarbeitungszeit
            }
        }

        // --- ROS 2 Schnittstellen-Simulatoren ---
        private bool CheckRosLaserScanner() {
            // Simuliert eine Hand-Erkennung bei 10mm (Schritt 2)
            return _brain.GetPlanStep() == 2; 
        }

        private void PublishRosJointCommand(ulong action) {
            Console.WriteLine($"[ROS2 PUB] Sende Trajektorie für Schritt: {GetActionName(action)}");
        }

        private void PublishRosStop() {
            Console.WriteLine("[ROS2 PUB] !!! EMERGENCY_STOP an JointController gesendet !!!");
        }

        private string GetActionName(ulong t) => t == T_WELD_START ? "START" : t == T_WELD_END ? "20mm ENDE" : "PFAD_PUNKT";
        public void Dispose() => _brain?.Dispose();
    }
}
