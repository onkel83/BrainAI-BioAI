using System;
using System.Collections.Generic;
using System.Threading;
using BrainAI.BioAI;

namespace BioAI.SurvivalDemo
{
    public class SurvivalAgent : IDisposable
    {
        private BioBrain _brain;
        private Random _rng = new Random();

        // --- 1. TOKEN DEFINITIONEN (Bedürfnisse & Umwelt) ---
        // Zustände (OBJECTS / NEEDS / STATUS)
        private static readonly ulong T_HUNGRY   = BioClusters.CreateToken("NEED_HUNGER", BioClusters.NEED);
        private static readonly ulong T_TIRED    = BioClusters.CreateToken("NEED_ENERGY", BioClusters.NEED);
        private static readonly ulong T_FOOD_SEE = BioClusters.CreateToken("SEE_FOOD", BioClusters.OBJECT);
        private static readonly ulong T_HEALTH_LOW = BioClusters.CreateToken("STATUS_DYING", BioClusters.STATUS);

        // Aktionen (ACTIONS)
        private static readonly ulong T_SEARCH   = BioClusters.CreateToken("ACTION_SEARCH", BioClusters.ACTION);
        private static readonly ulong T_EAT      = BioClusters.CreateToken("ACTION_EAT", BioClusters.ACTION);
        private static readonly ulong T_SLEEP    = BioClusters.CreateToken("ACTION_SLEEP", BioClusters.ACTION);

        // Reflexe (Höchste Priorität)
        private static readonly ulong T_PANIC_REST = BioClusters.CreateToken("REFLEX_EMERGENCY_REST", BioClusters.REFLEX);

        // Agenten-Status
        private float _hunger = 0; // 0 = satt, 100 = verhungert
        private float _energy = 100; // 100 = fit, 0 = erschöpft
        private bool _isAlive = true;

        public SurvivalAgent()
        {
            _brain = new BioBrain(0xDEADC0DE);
            InitializeKnowledge();
        }

        private void InitializeKnowledge()
        {
            // Ebene 1: Instinkte (Injected Knowledge)
            _brain.SetMode(BioMode.Training);

            // "Wenn Gesundheit kritisch, dann Notfall-Ruhe" (Unverhandelbar)
            _brain.ForceInstinct(T_HEALTH_LOW, T_PANIC_REST, 1.0f);

            // Bias: "Wenn Hunger, versuch zu essen" (Eine Tendenz, kein Gesetz)
            _brain.ForceInstinct(T_HUNGRY, T_EAT, 0.3f);

            _brain.SetMode(BioMode.Production);
            Console.WriteLine("[AGENT] Instinkte geladen. Überlebenskampf beginnt...");
        }

        public void RunSimulation()
        {
            int ticks = 0;
            while (_isAlive && ticks < 100)
            {
                ticks++;
                UpdateEnvironment();

                // 2. Wahrnehmung: Welche Tokens sind gerade aktiv?
                List<ulong> perception = new List<ulong>();
                if (_hunger > 50) perception.Add(T_HUNGRY);
                if (_energy < 30) perception.Add(T_TIRED);
                if (_hunger > 80 || _energy < 10) perception.Add(T_HEALTH_LOW);
                
                // Zufällige Chance, Nahrung zu sehen
                bool foodVisible = _rng.NextDouble() > 0.7;
                if (foodVisible) perception.Add(T_FOOD_SEE);

                // 3. Denken: BioAI entscheidet basierend auf Erfahrung & Instinkt
                ulong action = _brain.Think(perception.ToArray());

                // 4. Aktion ausführen & Feedback geben
                ExecuteAction(action, foodVisible);

                Console.WriteLine($"Tick {ticks:D2} | Hunger: {_hunger:F0} | Energy: {_energy:F0} | Action: {GetTokenName(action)}");
                
                if (_hunger >= 100 || _energy <= 0) _isAlive = false;
                Thread.Sleep(200);
            }
            Console.WriteLine(_isAlive ? "[AGENT] Simulation beendet." : "[AGENT] Der Agent ist verstorben.");
        }

        private void ExecuteAction(ulong action, bool foodAvailable)
        {
            if (action == T_EAT)
            {
                if (foodAvailable) {
                    _hunger = Math.Max(0, _hunger - 40);
                    _brain.Learn(1.0f, T_EAT); // Positives Feedback: Essen war erfolgreich!
                } else {
                    _hunger += 5; // Energie verschwendet für nichts
                    _brain.Learn(-0.5f, T_EAT); // Negatives Feedback: Hier gab es gar nichts zu essen
                }
            }
            else if (action == T_SLEEP || action == T_PANIC_REST)
            {
                _energy = Math.Min(100, _energy + 30);
                _hunger += 5;
            }
            else // SEARCH
            {
                _energy -= 10;
                _hunger += 10;
            }
        }

        private void UpdateEnvironment()
        {
            _hunger += 5; // Natürlicher Hungerzuwachs
            _energy -= 2; // Natürlicher Energieverbrauch
        }

        private string GetTokenName(ulong t) => t == T_EAT ? "EAT" : t == T_SLEEP ? "SLEEP" : t == T_PANIC_REST ? "PANIC_REST" : "SEARCH";
        public void Dispose() => _brain?.Dispose();
    }
}
