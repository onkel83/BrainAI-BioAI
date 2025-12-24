using System;
using System.Collections.Generic;
using BrainAI.BioAI; // Native BioAI Wrapper

namespace BioAI.GameDemo
{
    public class BioNpcEntity : IDisposable
    {
        private BioBrain _brain;
        
        // --- 1. ONTOLOGIE (Die Welt des NPCs) ---
        // Wahrnehmung (OBJECTS / NEEDS)
        private static readonly ulong T_SEE_PLAYER  = BioClusters.CreateToken("SEE_PLAYER", BioClusters.OBJECT);
        private static readonly ulong T_SEE_GOLD    = BioClusters.CreateToken("SEE_GOLD_COIN", BioClusters.OBJECT);
        private static readonly ulong T_HP_LOW      = BioClusters.CreateToken("NEED_HEALING", BioClusters.NEED);
        private static readonly ulong T_IS_NIGHT     = BioClusters.CreateToken("TIME_NIGHT", BioClusters.TIME);

        // Aktionen (ACTIONS)
        private static readonly ulong T_ACTION_ATTACK = BioClusters.CreateToken("ACT_ATTACK", BioClusters.ACTION);
        private static readonly ulong T_ACTION_FLEE   = BioClusters.CreateToken("ACT_FLEE", BioClusters.ACTION);
        private static readonly ulong T_ACTION_GATHER = BioClusters.CreateToken("ACT_GATHER", BioClusters.ACTION);
        private static readonly ulong T_ACTION_TRADE  = BioClusters.CreateToken("ACT_TRADE", BioClusters.ACTION);

        // Reflexe (Angeborenes Verhalten)
        private static readonly ulong T_REFLEX_HEAL   = BioClusters.CreateToken("REFLEX_SELF_PRESERVATION", BioClusters.REFLEX);

        public BioNpcEntity(ulong npcId)
        {
            _brain = new BioBrain(npcId); // Eindeutige ID für das "Gehirn"
            InitializeNpcPersonality();
        }

        private void InitializeNpcPersonality()
        {
            _brain.SetMode(BioMode.Training); // Initiales Setup

            // Ebene 1: Instinkte (Hard-Wiring)
            // Wenn HP kritisch -> IMMER Fliehen/Heilen (Gewicht 1.0)
            _brain.ForceInstinct(T_HP_LOW, T_REFLEX_HEAL, 1.0f);

            // Bias: Nachts ist der NPC vorsichtiger (Tendenz zur Flucht bei Sichtung)
            _brain.ForceInstinct(T_IS_NIGHT, T_ACTION_FLEE, 0.4f);

            _brain.SetMode(BioMode.Production); // NPC ist bereit für die Welt
        }

        /// <summary>
        /// Die Update-Schleife des NPCs (wird z.B. alle 500ms aufgerufen)
        /// </summary>
        public void OnUpdate(bool playerVisible, bool goldVisible, float currentHp, bool nightTime)
        {
            // 2. Wahrnehmung: Aktuelle Situation erfassen
            List<ulong> inputs = new List<ulong>();
            if (playerVisible) inputs.Add(T_SEE_PLAYER);
            if (goldVisible)   inputs.Add(T_SEE_GOLD);
            if (currentHp < 20) inputs.Add(T_HP_LOW);
            if (nightTime)     inputs.Add(T_IS_NIGHT);

            // 3. Denken: BioAI wählt die beste Aktion
            ulong action = _brain.Think(inputs.ToArray());

            // 4. Handeln & Lernen (Ebene 2: Erfahrung)
            ProcessAction(action, playerVisible);
        }

        private void ProcessAction(ulong action, bool playerNearby)
        {
            if (action == T_REFLEX_HEAL) {
                Console.WriteLine("[NPC] NUTZT HEILTRANK! (Reflex)");
            }
            else if (action == T_ACTION_ATTACK) {
                Console.WriteLine("[NPC] GREIFT AN!");
                // Lernen: Wenn Angriff gegen Spieler schmerzhaft war, negatives Feedback
                _brain.Learn(-0.5f, T_ACTION_ATTACK); 
            }
            else if (action == T_ACTION_TRADE && playerNearby) {
                Console.WriteLine("[NPC] BIETET HANDEL AN.");
                // Lernen: Erfolgreicher Handel gibt positive Verstärkung
                _brain.Learn(0.8f, T_ACTION_TRADE);
            }
            else if (action == T_ACTION_GATHER) {
                Console.WriteLine("[NPC] SAMMELT GOLD.");
            }
        }

        public void Dispose() => _brain?.Dispose();
    }
}
