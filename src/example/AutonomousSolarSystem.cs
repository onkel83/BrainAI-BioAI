using System;
using System.Collections.Generic;
using System.Threading;
using BrainAI.BioAI;

namespace BioAI.SolarDemo
{
    public class AutonomousSolarSystem : IDisposable
    {
        private BioBrain _brain;

        // --- 1. TOKEN DEFINITIONEN (Umwelt & Energie) ---
        // Sensoren (OBJECTS / STATUS)
        private static readonly ulong T_SUN_HIGH      = BioClusters.CreateToken("SUN_INTENSITY_HIGH", BioClusters.OBJECT);
        private static readonly ulong T_SUN_LOW       = BioClusters.CreateToken("SUN_INTENSITY_LOW", BioClusters.OBJECT);
        private static readonly ulong T_BATT_CRITICAL  = BioClusters.CreateToken("BATTERY_CRITICAL", BioClusters.STATUS);
        private static readonly ulong T_BATT_FULL      = BioClusters.CreateToken("BATTERY_FULL", BioClusters.STATUS);

        // Aktionen (ACTIONS)
        private static readonly ulong T_MODE_PERFORMANCE = BioClusters.CreateToken("MODE_PERFORMANCE", BioClusters.ACTION);
        private static readonly ulong T_MODE_ECO         = BioClusters.CreateToken("MODE_POWER_SAVE", BioClusters.ACTION);
        private static readonly ulong T_CHARGE_ONLY      = BioClusters.CreateToken("ACTION_CHARGE_ONLY", BioClusters.ACTION);

        // Sicherheits-Reflexe (Unverhandelbare Hard-Safety)
        private static readonly ulong T_HARD_SHUTDOWN    = BioClusters.CreateToken("REFLEX_HARD_SHUTDOWN", BioClusters.REFLEX);

        public AutonomousSolarSystem()
        {
            _brain = new BioBrain(0x5014_7E57); // "SOLAR TEST" Lizenzschlüssel
            InitializeEnergyLogic();
        }

        private void InitializeEnergyLogic()
        {
            // Ebene 1: Instinkte (Injected Knowledge)
            _brain.SetMode(BioMode.Training);

            // Reflex: Wenn Batterie kritisch -> Sofortiger Shutdown zum Zellschutz (Gewicht 1.0)
            _brain.ForceInstinct(T_BATT_CRITICAL, T_HARD_SHUTDOWN, 1.0f);

            // Bias: Wenn die Sonne stark scheint, tendiere zum Performance-Modus
            _brain.ForceInstinct(T_SUN_HIGH, T_MODE_PERFORMANCE, 0.4f);

            // Bias: Wenn die Sonne schwach ist, tendiere zum ECO-Modus
            _brain.ForceInstinct(T_SUN_LOW, T_MODE_ECO, 0.5f);

            _brain.SetMode(BioMode.Production); // Einfrieren für stabilen Betrieb
            Console.WriteLine("[SOLAR] Energiemanagement-Kernel aktiv.");
        }

        public void ProcessCycle(float lux, float batteryPercent)
        {
            List<ulong> activeInputs = new List<ulong>();

            // 1. Sensor-Abstraktion
            if (lux > 50000) activeInputs.Add(T_SUN_HIGH);
            else if (lux < 5000) activeInputs.Add(T_SUN_LOW);

            if (batteryPercent < 10) activeInputs.Add(T_BATT_CRITICAL);
            else if (batteryPercent > 95) activeInputs.Add(T_BATT_FULL);

            // 2. Denken: Die KI wählt die effizienteste Strategie
            ulong action = _brain.Think(activeInputs.ToArray());

            // 3. Ausführung & Feedback
            ExecuteEnergyAction(action, batteryPercent);
        }

        private void ExecuteEnergyAction(ulong action, float currentBattery)
        {
            if (action == T_HARD_SHUTDOWN)
            {
                Console.WriteLine("!!! [REFLEX] KRITISCHER ENERGIEZUSTAND: SHUTDOWN !!!");
                // Hier Hardware-Register für Tiefschlaf setzen
            }
            else if (action == T_MODE_PERFORMANCE)
            {
                Console.WriteLine("[SOLAR] Mode: Performance (Alle Sensoren aktiv, 100Hz)");
                // Belohnung geben, wenn die Batterie trotz Performance stabil bleibt
                if (currentBattery > 50) _brain.Learn(0.2f, T_MODE_PERFORMANCE);
            }
            else if (action == T_MODE_ECO)
            {
                Console.WriteLine("[SOLAR] Mode: ECO (Sendeintervall reduziert auf 10 Min)");
            }
        }

        public void Dispose() => _brain?.Dispose();
    }
}
