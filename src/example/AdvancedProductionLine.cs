using System;
using System.Collections.Generic;
using System.Threading;
using BrainAI.BioAI;

namespace BioAI.ProductionDemo
{
    public class AdvancedProductionLine : IDisposable
    {
        private BioBrain _brain;

        // --- 1. TOKEN DEFINITIONEN (Die 6 Produktionsschritte) ---
        private static readonly ulong T_PICK_PART    = BioClusters.CreateToken("PICK_PART", BioClusters.ACTION);
        private static readonly ulong T_SCAN_QR     = BioClusters.CreateToken("SCAN_QR_CODE", BioClusters.ACTION);
        private static readonly ulong T_DRILL        = BioClusters.CreateToken("DRILL_HOLE", BioClusters.ACTION);
        private static readonly ulong T_MILL         = BioClusters.CreateToken("MILL_SURFACE", BioClusters.ACTION);
        private static readonly ulong T_CLEAN        = BioClusters.CreateToken("CLEAN_PART", BioClusters.ACTION);
        private static readonly ulong T_PLACE_DONE   = BioClusters.CreateToken("PLACE_IN_BIN", BioClusters.ACTION);

        // --- 2. SICHERHEIT & SENSOREN ---
        private static readonly ulong T_SENSOR_LIGHT_BARRIER = BioClusters.CreateToken("LIGHT_BARRIER_BREACH", BioClusters.OBJECT);
        private static readonly ulong T_EMERGENCY_STOP       = BioClusters.CreateToken("EMERGENCY_STOP", BioClusters.REFLEX);
        private static readonly ulong T_PART_DEFECT          = BioClusters.CreateToken("PART_DEFECT_DETECTED", BioClusters.OBJECT);

        public AdvancedProductionLine()
        {
            _brain = new BioBrain(0x1337BEEF); // Lizenzschlüssel zur Initialisierung
            SetupProductionSafety();
        }

        private void SetupProductionSafety()
        {
            _brain.SetMode(BioMode.Training); // Vorbereitung für Wissensinjektion

            // Sicherheits-Reflex (Ebene 1: Instinkte): 
            // Höchste Priorität (1.0f) für den Schutzbereich.
            _brain.ForceInstinct(T_SENSOR_LIGHT_BARRIER, T_EMERGENCY_STOP, 1.0f);

            _brain.SetMode(BioMode.Production); // Struktur einfrieren für den Betrieb
            Console.WriteLine("[SYSTEM] Sicherheits-Reflexe aktiv. System im Production Mode.");
        }

        public void RunProductionCycle()
        {
            // Definition der 6-stufigen Sequenz
            var sequence = new List<ulong> { 
                T_PICK_PART, T_SCAN_QR, T_DRILL, T_MILL, T_CLEAN, T_PLACE_DONE 
            };

            // Laden des Plans (Sequencer-Muster)
            _brain.LoadPlan(sequence.ToArray(), strict: true); 
            Console.WriteLine("[PLAN] 6-Schritt-Plan geladen. Starte Fertigung...");

            while (_brain.GetPlanStep() != -1) // Solange der Plan aktiv ist
            {
                // In einer echten Anlage kämen hier die realen Sensorwerte
                List<ulong> inputs = new List<ulong>();
                
                // Simulation: Jemand tritt in die Lichtschranke bei Schritt 4 (Fräsen)
                if (_brain.GetPlanStep() == 4) 
                {
                    Console.WriteLine("\n[!] WARNUNG: Lichtschranke unterbrochen!");
                    inputs.Add(T_SENSOR_LIGHT_BARRIER);
                }

                // KI entscheidet: Normaler Schritt oder Reflex-Reaktion?
                ulong action = _brain.Think(inputs.ToArray());

                if (action == T_EMERGENCY_STOP)
                {
                    Console.WriteLine("=====================================");
                    Console.WriteLine("!!! NOT-AUS DURCH BIOAI REFLEX !!!");
                    Console.WriteLine("=====================================");
                    _brain.AbortPlan(); // Sofortiger Stopp aller Sequenzen
                    break;
                }

                // Ausgabe der aktuellen Aktion
                LogAction(action);
                Thread.Sleep(500); // Bearbeitungszeit simulieren
            }
        }

        private void LogAction(ulong action)
        {
            int step = _brain.GetPlanStep();
            string name = action == T_PICK_PART ? "Material holen" :
                          action == T_SCAN_QR  ? "QR-Code scannen" :
                          action == T_DRILL    ? "Bohren" :
                          action == T_MILL     ? "Fräsen" :
                          action == T_CLEAN    ? "Reinigen" :
                          action == T_PLACE_DONE ? "Ablegen" : "Unbekannt";
            
            Console.WriteLine($"[Schritt {step}] Führe aus: {name}");
        }

        public void Dispose() => _brain?.Dispose();
    }
}
