import time
from bioai import BioBrainInstance

class AutonomousSolarSystem:
    # --- 1. TOKEN DEFINITIONEN (Cluster-Masken gemäß ISS-Standard) ---
    # OBJECT Cluster (0x1000...)
    T_SUN_HIGH      = 0x1000000000000001
    T_SUN_LOW       = 0x1000000000000002
    T_BATT_CRITICAL = 0x1000000000000003
    T_BATT_FULL     = 0x1000000000000004

    # ACTION Cluster (0x2000...)
    T_MODE_PERFORMANCE = 0x2000000000000001
    T_MODE_ECO         = 0x2000000000000002
    T_CHARGE_ONLY      = 0x2000000000000003

    # REFLEX / LOGIC Cluster (0x4010...)
    T_HARD_SHUTDOWN    = 0x4010000000000001

    def __init__(self, json_path: str, dll_path: str):
        # Initialisierung über den Python-Wrapper
        self._brain = BioBrainInstance(json_path, dll_path)
        self._initialize_energy_logic()

    def _initialize_energy_logic(self):
        """Ebene 1: Instinkte & Basisregeln injizieren."""
        # Modus: Training (0)
        self._brain.set_mode(0)

        # Reflex: Wenn Batterie kritisch -> Sofortiger Shutdown (Gewicht 1.0)
        # teach(input, action, weight) entspricht ForceInstinct
        self._brain.teach(self.T_BATT_CRITICAL, self.T_HARD_SHUTDOWN, 1.0)

        # Bias: Wenn die Sonne stark scheint, tendiere zum Performance-Modus
        self._brain.teach(self.T_SUN_HIGH, self.T_MODE_PERFORMANCE, 0.4)

        # Bias: Wenn die Sonne schwach ist, tendiere zum ECO-Modus
        self._brain.teach(self.T_SUN_LOW, self.T_MODE_ECO, 0.5)

        # Modus: Produktion (1) für stabilen Betrieb
        self._brain.set_mode(1)
        print("[SOLAR] Python Energiemanagement-Kernel aktiv.")

    def process_cycle(self, lux: float, battery_percent: float):
        """Ein Simulations-Zyklus: Sensor -> Denken -> Handeln."""
        active_inputs = []

        # 1. Sensor-Abstraktion (Mapping auf TokenIDs)
        if lux > 50000:
            active_inputs.append(self.T_SUN_HIGH)
        elif lux < 5000:
            active_inputs.append(self.T_SUN_LOW)

        if battery_percent < 10:
            active_inputs.append(self.T_BATT_CRITICAL)
        elif battery_percent > 95:
            active_inputs.append(self.T_BATT_FULL)

        # 2. Denken: KI wählt die effizienteste Strategie (O(1))
        # update() entspricht Think()
        action = self._brain.update(active_inputs)

        # 3. Ausführung & Feedback
        self._execute_energy_action(action, battery_percent)

    def _execute_energy_action(self, action: int, current_battery: float):
        if action == self.T_HARD_SHUTDOWN:
            print("!!! [REFLEX] KRITISCHER ENERGIEZUSTAND: SHUTDOWN !!!")
            # Hardware-Schnittstelle hier ansprechen
        
        elif action == self.T_MODE_PERFORMANCE:
            print(f"[SOLAR] Mode: Performance (Batt: {current_battery}%)")
            # Belohnung geben (Learn), wenn die Batterie stabil bleibt
            if current_battery > 50:
                # feedback(reward, action) entspricht Learn()
                self._brain.feedback(0.2, self.T_MODE_PERFORMANCE)
        
        elif action == self.T_MODE_ECO:
            print(f"[SOLAR] Mode: ECO (Batt: {current_battery}%)")

    def shutdown(self):
        """Saubere Freigabe der nativen Ressourcen."""
        self._brain.close()

# --- HAUPTPROGRAMM ---
if __name__ == "__main__":
    # Pfade zu den ISS-Artefakten
    DLL_PATH = "bin/BioAI_ULTRA.dll"
    KEY_PATH = "bin/key.json"

    solar_system = AutonomousSolarSystem(KEY_PATH, DLL_PATH)

    try:
        # Test-Szenario 1: Viel Sonne, volle Batterie
        print("\nSzenario: Mittagssonne...")
        solar_system.process_cycle(60000, 80)

        # Test-Szenario 2: Nacht, Batterie wird schwach
        print("\nSzenario: Dämmerung & niedriger Stand...")
        solar_system.process_cycle(1000, 45)

        # Test-Szenario 3: KRITISCH (Reflex-Check)
        print("\nSzenario: Tiefentladungsschutz...")
        solar_system.process_cycle(2000, 5)

    finally:
        solar_system.shutdown()