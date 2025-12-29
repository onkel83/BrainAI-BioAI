import time
from bioai import BioBrainInstance

class ProductionController:
    # --- 1. TOKEN DEFINITIONEN (Die Ontologie des Systems) ---
    # OBJECT Cluster: Sensor-Inputs
    T_TEMP_CRIT  = 0x1000000000000001
    T_HAND_IN    = 0x1000000000000002
    T_PART_LOOSE = 0x1000000000000003 # Audit-Token
    
    # ACTION Cluster: Aktor-Befehle
    T_MOTOR_ON   = 0x2000000000000001
    T_WELD_STEP  = 0x2000000000000002
    T_DRILL_STEP = 0x2000000000000003
    T_MOTOR_OFF  = 0x2000000000000004
    T_TIGHTEN    = 0x2000000000000005 # Audit-Aktion
    
    # REFLEX Cluster: Hard-Safety (0x4010...)
    T_EMERGENCY  = 0x4010000000000001

    def __init__(self, key_path: str, dll_path: str):
        # Initialisierung des Rechenkerns
        self._brain = BioBrainInstance(key_path, dll_path)
        self._setup_safety_and_audit()

    def _setup_safety_and_audit(self):
        """Phase 1: Wissensinjektion & Freeze (Safety-Protokoll)."""
        # 1. Trainingsmodus aktivieren für Injektion
        self._brain.set_mode(0)

        # Muster 1: Der Unbrechbare Reflex (Safety Interlock)
        # Gewicht 1.0 garantiert höchste Priorität über alle anderen Logiken.
        self._brain.teach(self.T_HAND_IN, self.T_EMERGENCY, 1.0)
        self._brain.teach(self.T_TEMP_CRIT, self.T_EMERGENCY, 1.0)

        # Audit-Wissen injizieren (Nachweis der LTM-Auditierbarkeit)
        self._brain.teach(self.T_PART_LOOSE, self.T_TIGHTEN, 0.45)

        # 2. Production Mode: Struktur einfrieren (Deterministisch)
        self._brain.set_mode(1)
        print("[SETUP] Hard-Safety-Reflexe & Audit-Wissen injiziert.")
        print("[SETUP] BioBrain in den Production Mode eingefroren.\n")

    def run_production_cycle(self):
        """Simulation der Fertigungsstraße mit Reflex-Überwachung."""
        # Muster 2: Sequencer (Vorgegebene Choreographie)
        # Hinweis: Die KI arbeitet diese Liste ab
        plan = [self.T_MOTOR_ON, self.T_WELD_STEP, self.T_DRILL_STEP, self.T_MOTOR_OFF]
        
        token_names = {
            self.T_MOTOR_ON: "MOTOR_ON", self.T_WELD_STEP: "WELDING_STEP",
            self.T_DRILL_STEP: "DRILLING_STEP", self.T_EMERGENCY: "EMERGENCY_STOP",
            self.T_MOTOR_OFF: "MOTOR_OFF", self.T_TEMP_CRIT: "TEMP_CRIT"
        }

        print(f"[PLAN] {len(plan)} Schritte geladen. Produktion beginnt...")

        for i, expected_action in enumerate(plan):
            current_inputs = []
            
            # Simulation: Bei Schritt 1 (Schweißen) tritt ein kritischer Fehler auf.
            if i == 1:
                current_inputs.append(self.T_TEMP_CRIT)

            # Muster 3: Denken (Think) - O(1) Entscheidung
            decision = self._brain.update(current_inputs)

            if decision == self.T_EMERGENCY:
                # Muster 4: Audit (Inspect) - Nachweis der Entscheidung
                safety_weight = self._brain.inspect(self.T_TEMP_CRIT, self.T_EMERGENCY)
                
                print("\n" + "="*45)
                print(f"[!! KRITISCHER ABBRUCH !!] Reflex: {token_names[decision]}")
                print(f"[AUDIT] Gewicht TEMP_CRIT -> EMERGENCY_STOP: {safety_weight:.2f}")
                print("="*45)
                return # Sicherheits-Abbruch der Schleife

            # Normaler Ablauf
            action_name = token_names.get(decision, "UNKNOWN")
            print(f"[STEP {i:02}] ACTION: {action_name}")
            time.sleep(0.05)

    def perform_post_audit(self):
        """Beweisführung der LTM-Integrität nach der Laufzeit."""
        print("\n--- POST-AUDIT (LTM-Injektions-Verifikation) ---")
        
        # Überprüfung des vor dem Freeze injizierten Wissens
        learn_weight = self._brain.inspect(self.T_PART_LOOSE, self.T_TIGHTEN)
        
        print(f"[INSPECT] Gewicht PART_LOOSE -> TIGHTEN_BOLT: {learn_weight:.3f}")
        
        if learn_weight > 0.4:
            print("=> ERFOLG: Injektion von Wissen in das LTM verifiziert.")
        else:
            print("=> FEHLER: Wissensverlust detektiert.")

    def shutdown(self):
        self._brain.close()

if __name__ == "__main__":
    controller = ProductionController("bin/key.json", "bin/BioAI_ULTRA.dll")
    try:
        controller.run_production_cycle()
        controller.perform_post_audit()
    finally:
        controller.shutdown()