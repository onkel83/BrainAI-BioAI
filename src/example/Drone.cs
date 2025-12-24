using System;
using System.Collections.Generic;
using BrainAI.BioAI;

namespace BioAI.DroneExample
{
    public class BioDroneController : IDisposable
    {
        private BioBrain _brain;
        
        // --- 1. TOKEN DEFINITIONEN (Ontologie) ---
        // Sensoren (OBJECTS)
        private static readonly ulong T_DIST_FRONT_CLOSE = BioClusters.CreateToken("DIST_FRONT_CLOSE", BioClusters.OBJECT);
        private static readonly ulong T_BATTERY_LOW      = BioClusters.CreateToken("BATTERY_LOW", BioClusters.OBJECT);
        private static readonly ulong T_GPS_LOST         = BioClusters.CreateToken("GPS_SIGNAL_LOST", BioClusters.OBJECT);

        // Aktionen (ACTIONS)
        private static readonly ulong T_MOVE_FORWARD     = BioClusters.CreateToken("MOVE_FORWARD", BioClusters.ACTION);
        private static readonly ulong T_HOVER            = BioClusters.CreateToken("HOVER", BioClusters.ACTION);
        private static readonly ulong T_DESCEND          = BioClusters.CreateToken("DESCEND", BioClusters.ACTION);

        // Sicherheits-Reflexe (REFLEX)
        private static readonly ulong T_EMERGENCY_LAND   = BioClusters.CreateToken("EMERGENCY_LAND", BioClusters.REFLEX);
        private static readonly ulong T_RTL              = BioClusters.CreateToken("RETURN_TO_LAUNCH", BioClusters.REFLEX);

        public BioDroneController(ulong licenseKey)
        {
            _brain = new BioBrain(licenseKey);
            InitializeInstincts();
        }

        private void InitializeInstincts()
        {
            // Ebene 1: Instinkte (Injected Knowledge für Safety First)
            _brain.SetMode(BioMode.Training);

            // Reflex: Wenn Akku kritisch -> Sofort Landen (Gewicht 1.0 = Unverhandelbar)
            _brain.ForceInstinct(T_BATTERY_LOW, T_EMERGENCY_LAND, 1.0f);
            
            // Reflex: Wenn Hindernis vorne -> In den Schwebeflug (Hover)
            _brain.ForceInstinct(T_DIST_FRONT_CLOSE, T_HOVER, 1.0f);

            // Bias: Wenn GPS weg -> Komm nach Hause (RTL)
            _brain.ForceInstinct(T_GPS_LOST, T_RTL, 0.8f);

            // Nach der Injektion: Struktur einfrieren für deterministisches Verhalten im Flug
            _brain.SetMode(BioMode.Production);
        }

        /// <summary>
        /// Hauptschleife: Wandelt MAVLink/Sensor-Daten in BioAI-Entscheidungen um.
        /// </summary>
        public void UpdateFlightLogic(float distanceFront, float batteryPercent, bool hasGps)
        {
            List<ulong> activeInputs = new List<ulong>();

            // 1. Sensor-Abstraktion (Wandelt Floats in Tokens um)
            if (distanceFront < 1.5f) activeInputs.Add(T_DIST_FRONT_CLOSE);
            if (batteryPercent < 15.0f) activeInputs.Add(T_BATTERY_LOW);
            if (!hasGps) activeInputs.Add(T_GPS_LOST);

            // 2. Denken (Think Cycle)
            ulong decision = _brain.Think(activeInputs.ToArray());

            // 3. Aktion an Flight Controller (MAVLink Integration Point)
            ExecuteCommand(decision);
        }

        private void ExecuteCommand(ulong actionToken)
        {
            if (actionToken == T_EMERGENCY_LAND) 
                Console.WriteLine("[MAVLINK] CMD_NAV_LAND (Kritisch)");
            else if (actionToken == T_HOVER) 
                Console.WriteLine("[MAVLINK] SET_POSITION_TARGET (Vel: 0,0,0)");
            else if (actionToken == T_MOVE_FORWARD)
                Console.WriteLine("[MAVLINK] SET_POSITION_TARGET (Vel: 2,0,0)");
            else if (actionToken == T_RTL)
                Console.WriteLine("[MAVLINK] CMD_NAV_RETURN_TO_LAUNCH");
        }

        public void Dispose() => _brain?.Dispose();
    }
}
