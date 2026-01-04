import time
import random
from bioai import BioBrainInstance

class SurvivalAgent:
    # --- 1. TOKEN DEFINITIONEN (Bedürfnisse & Umwelt) ---
    # NEEDS / STATUS Cluster
    T_HUNGRY     = 0x5000000000000001 # NEED Cluster
    T_TIRED      = 0x5000000000000002 # NEED Cluster
    T_HEALTH_LOW = 0x5000000000000003 # STATUS Cluster
    
    # OBJECTS Cluster
    T_FOOD_SEE   = 0x1000000000000001
    
    # ACTIONS Cluster
    T_SEARCH     = 0x2000000000000001
    T_EAT        = 0x2000000000000002
    T_SLEEP      = 0x2000000000000003
    
    # REFLEX Cluster (0x4010...)
    T_PANIC_REST = 0x4010000000000001

    def __init__(self, json_path: str, dll_path: str):
        # Initialisierung des Gehirns
        self._brain = BioBrainInstance(json_path, dll_path)
        
        # Agenten-Status
        self._hunger = 0    # 0 = satt, 100 = verhungert
        self._energy = 100  # 100 = fit, 0 = erschöpft
        self._is_alive = True
        
        self._initialize_knowledge()

    def _initialize_knowledge(self):
        """Ebene 1: Instinkte & Basis-Tendenzen injizieren."""
        # Trainingsmodus für Setup
        self._brain.set_mode(0)

        # Reflex: Wenn Gesundheit kritisch -> Notfall-Ruhe (Gewicht 1.0)
        self._brain.teach(self.T_HEALTH_LOW, self.T_PANIC_REST, 1.0)

        # Bias: Grundtendenz zum Essen bei Hunger (Gewicht 0.3)
        self._brain.teach(self.T_HUNGRY, self.T_EAT, 0.3)

        # Produktion einfrieren
        self._brain.set_mode(1)
        print("[AGENT] Instinkte geladen. Überlebenskampf beginnt...")

    def run_simulation(self):
        """Hauptschleife des Überlebenskampfes."""
        ticks = 0
        while self._is_alive and ticks < 50:
            ticks += 1
            self._update_environment()

            # 2. Wahrnehmung (Perception Mapping)
            perception = []
            if self._hunger > 50: perception.append(self.T_HUNGRY)
            if self._energy < 30: perception.append(self.T_TIRED)
            
            # Kritischer Status (Reflex-Trigger)
            if self._hunger > 80 or self._energy < 10:
                perception.append(self.T_HEALTH_LOW)
            
            # Externe Welt-Objekte
            food_visible = random.random() > 0.7
            if food_visible: perception.append(self.T_FOOD_SEE)

            # 3. Denken (O(1) Entscheidung)
            action = self._brain.update(perception)

            # 4. Aktion & Feedback (Ebene 2: Erfahrungslernen)
            self._execute_action(action, food_visible)

            # Anzeige
            action_name = self._get_token_name(action)
            print(f"Tick {ticks:02} | Hunger: {self._hunger:3.0f} | Energy: {self._energy:3.0f} | Action: {action_name}")
            
            if self._hunger >= 100 or self._energy <= 0:
                self._is_alive = False
            
            time.sleep(0.1)

        print("[AGENT] Simulation beendet." if self._is_alive else "[AGENT] Der Agent ist verstorben.")

    def _execute_action(self, action: int, food_available: bool):
        if action == self.T_EAT:
            if food_available:
                self._hunger = max(0, self._hunger - 40)
                # Belohnung: Essen war erfolgreich!
                self._brain.feedback(1.0, self.T_EAT)
            else:
                self._hunger += 5 # Energie verschwendet
                # Bestrafung: Aktion ohne Erfolg
                self._brain.feedback(-0.5, self.T_EAT)
        
        elif action == self.T_SLEEP or action == self.T_PANIC_REST:
            self._energy = min(100, self._energy + 30)
            self._hunger += 5
        
        else: # T_SEARCH oder Unbekannt
            self._energy -= 10
            self._hunger += 10

    def _update_environment(self):
        self._hunger += 5  # Natürliche Entropie
        self._energy -= 2

    def _get_token_name(self, t: int) -> str:
        mapping = {self.T_EAT: "EAT", self.T_SLEEP: "SLEEP", 
                   self.T_PANIC_REST: "PANIC_REST", self.T_SEARCH: "SEARCH"}
        return mapping.get(t, "IDLE")

    def shutdown(self):
        self._brain.close()

if __name__ == "__main__":
    agent = SurvivalAgent("bin/key.json", "bin/BioAI_ULTRA.dll")
    try:
        agent.run_simulation()
    finally:
        agent.shutdown()