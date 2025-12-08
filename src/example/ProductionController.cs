using System;
using System.Collections.Generic;
using System.Threading;
// WICHTIG: Die Klasse 'BrainAI.BioAI' wird als separater Wrapper (BioAI.cs) im Projekt erwartet.
using BrainAI.BioAI; 

namespace BioAI_ProductionDemo
{
    // =========================================================================
    // BioAI - Industriesteuerung Beispielcode (C# CLI)
    // Demonstriert die Anwendung der vier Kernmuster: Reflex, Sequencer, Think und Inspect.
    // =========================================================================
    public class ProductionController
    {
        // --- 1. TOKEN DEFINITIONEN (Die Ontologie des Systems) ---
        // Diese Tokens werden via FNV-1a Hash in eindeutige 64-Bit IDs (TokenID) umgewandelt.
        // Die Cluster (OBJECT/ACTION/REFLEX) sind essentiell für die KI-Logik.
        private static readonly ulong T_TEMP_CRIT = BioClusters.CreateToken("TEMP_CRIT", BioClusters.OBJECT); // Sensor-Input
        private static readonly ulong T_HAND_IN = BioClusters.CreateToken("HAND_IN_BAY", BioClusters.OBJECT);   // Sensor-Input
        private static readonly ulong T_PART_OK = BioClusters.CreateToken("PART_OK", BioClusters.OBJECT);     // Reward-Ziel (Input)
        private static readonly ulong T_WELD_STEP = BioClusters.CreateToken("WELDING_STEP", BioClusters.ACTION); // Aktor-Befehl
        private static readonly ulong T_DRILL_STEP = BioClusters.CreateToken("DRILLING_STEP", BioClusters.ACTION); // Aktor-Befehl
        private static readonly ulong T_EMERGENCY = BioClusters.CreateToken("EMERGENCY_STOP", BioClusters.REFLEX); // Höchste Priorität (Reflex)
        private static readonly ulong T_MOTOR_ON = BioClusters.CreateToken("MOTOR_ON", BioClusters.ACTION);
        private static readonly ulong T_MOTOR_OFF = BioClusters.CreateToken("MOTOR_OFF", BioClusters.ACTION);

        // Audit-Tokens: Wird genutzt, um die korrekte Funktion von API_Teach zu beweisen.
        private static readonly ulong T_LERN_INPUT = BioClusters.CreateToken("PART_SLIGHTLY_LOOSE", BioClusters.OBJECT);
        private static readonly ulong T_LERN_ACTION = BioClusters.CreateToken("TIGHTEN_BOLT", BioClusters.ACTION);

        public static void Main(string[] args)
        {
            Console.WriteLine("--- BioAI Industriesteuerung Beispielcode (C#) ---");
            Console.WriteLine("Demonstriert Reflex, Sequencer und LTM-Auditierbarkeit.");
            Console.WriteLine("-------------------------------------------------");
            
            using var brain = new BioBrain(424242);
            
            // =========================================================================
            // --- 2. SETUP: WISSENSINJEKTION UND FREEZE (Das Safety-Protokoll) ---
            // =========================================================================
            
            // Schalte zuerst in den Trainingsmodus, da API_Teach neue Neuronen erfordert.
            brain.SetMode(BioMode.Training); 

            // Muster 1: Der Unbrechbare Reflex (ForceInstinct / Safety Interlock)
            // Stellt sicher, dass das System auf HARD-SAFETY-Inputs (z.B. Lichtschranke) sofort reagiert.
            // Gewicht 1.0f garantiert höchste Priorität.
            brain.ForceInstinct(T_HAND_IN, T_EMERGENCY, 1.0f);
            brain.ForceInstinct(T_TEMP_CRIT, T_EMERGENCY, 1.0f);

            // Audit-Wissen injizieren (Demonstriert die Funktion der API_Teach / ForceInstinct)
            // Dies ist das Wissen, das wir später im Post-Audit überprüfen.
            brain.ForceInstinct(T_LERN_INPUT, T_LERN_ACTION, 0.45f); 

            // Aktiviere den Production Mode: Die Struktur des Gehirns wird eingefroren.
            // Garantiert 100% Determinismus und Speichersicherheit (keine neuen Neuronen erlaubt).
            brain.SetMode(BioMode.Production);
            Console.WriteLine("[SETUP] Hard-Safety-Reflexe & Audit-Wissen injiziert.");
            Console.WriteLine("[SETUP] BioBrain in den Production Mode (Deterministisch) eingefroren.\n");


            // Muster 2: Exakte Choreographie (Sequencer)
            // Definiert den festen Fertigungsablauf. Die KI arbeitet diesen ab (ersetzt SPS-Code).
            var productionSteps = new List<ulong> { T_MOTOR_ON, T_WELD_STEP, T_DRILL_STEP, T_MOTOR_OFF };
            brain.LoadPlan(productionSteps.ToArray(), strict: true); 
            Console.WriteLine($"[PLAN] {productionSteps.Count} Schritte geladen. Produktion beginnt...");

            // Temporäre Token-Namen für die Konsole (Hilft beim Lesen der Audit-Ergebnisse)
            var tokenNames = new Dictionary<ulong, string>
            {
                { T_MOTOR_ON, "MOTOR_ON" }, { T_WELD_STEP, "WELDING_STEP" }, { T_DRILL_STEP, "DRILLING_STEP" },
                { T_EMERGENCY, "EMERGENCY_STOP" }, { T_HAND_IN, "HAND_IN_BAY" }, { T_TEMP_CRIT, "TEMP_CRIT" },
                { T_MOTOR_OFF, "MOTOR_OFF" }, { T_LERN_INPUT, "PART_SLIGHTLY_LOOSE" }, { T_LERN_ACTION, "TIGHTEN_BOLT" }
            };

            // =========================================================================
            // --- 3. SIMULATIONS-SCHLEIFE (Demonstriert Think und Reflex-Priorität) ---
            // =========================================================================
            while (brain.GetPlanStep() != -1)
            {
                var currentInputs = new List<ulong> { };
                
                // Simuliere: Bei Schritt 2 (Schweißen) tritt ein kritischer Fehler auf.
                if (brain.GetPlanStep() == 2) 
                { 
                    currentInputs.Add(T_TEMP_CRIT); 
                }
                
                // Muster 3: Denken (Think) - O(1)-Entscheidung
                ulong decision = brain.Think(currentInputs.ToArray());

                if (decision == T_EMERGENCY)
                {
                    // Das ist die Beweisführung des Reflexes
                    float safetyWeight = brain.Inspect(T_TEMP_CRIT, T_EMERGENCY);
                    
                    Console.WriteLine("\n=============================================");
                    Console.WriteLine($"[!! KRITISCHER ABBRUCH !!] Reflex: {tokenNames[decision]}");
                    // Muster 4: Audit (Inspect) - Nachweis der Entscheidung
                    Console.WriteLine($"[AUDIT] Gewicht {tokenNames[T_TEMP_CRIT]} -> {tokenNames[T_EMERGENCY]}: {safetyWeight:F2}");
                    Console.WriteLine("=============================================");
                    
                    break;
                }
                
                // Normaler Ablauf (wird durch den Sequencer gesteuert)
                if (tokenNames.TryGetValue(decision, out string actionName))
                {
                    Console.WriteLine($"[STEP {brain.GetPlanStep():D2}] ACTION: {actionName}");
                }
                
                Thread.Sleep(50);
            }

            // =========================================================================
            // --- 4. POST-AUDIT (Beweis der LTM-Injektion) ---
            // =========================================================================
            
            Console.WriteLine("\n\n--- POST-AUDIT (LTM-Injektions-Verifikation) ---");
            
            // 1. Audit: Überprüfe das vor dem Freeze injizierte Audit-Wissen.
            // Die KI muss im Production Mode diesen Wert liefern können, um auditierbar zu sein.
            float learnWeight = brain.Inspect(T_LERN_INPUT, T_LERN_ACTION); 
            
            Console.WriteLine("[LERN-AUDIT] Audit-Wissen wurde vor dem Freeze injiziert.");
            Console.WriteLine($"[INSPECT] Gewicht {tokenNames[T_LERN_INPUT]} -> {tokenNames[T_LERN_ACTION]}: {learnWeight:F3}");
            
            if (learnWeight > 0.001f)
            {
                Console.WriteLine("=> ERFOLG: Injektion von Wissen in das LTM funktioniert im Setup-Prozess.");
            }
            else
            {
                Console.WriteLine("=> FEHLER: Der Audit-Wert wurde auf 0 zurückgesetzt (LTM-Problem).");
            }
            
            // Finale Dokumentation
            BioClusters.DumpVocabulary("BioAI_Vocabulary_Audit.txt");
        }
    }
}
