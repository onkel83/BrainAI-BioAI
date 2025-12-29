import os
import json
from bioai import BioBrainInstance

class RobotSimulation:
    # --- 1. ONTOLOGIE (Cluster-Definitionen) ---
    # Wahrnehmung (OBJECTS)
    T_SENSOR_FREE = 0x1000000000000001
    T_SENSOR_WALL = 0x1000000000000002

    # Aktionen (ACTIONS)
    T_ACT_MOVE    = 0x2000000000000001
    T_ACT_TURN    = 0x2000000000000002

    def __init__(self, key_path="bin/key.json", dll_path="bin/BioAI_ULTRA.dll"):
        print("--- BioAI v0.7.6 Python Robot Simulation ---\n")
        
        # Initialisierung des Rechenkerns
        self.robot = BioBrainInstance(key_path, dll_path)
        self._initialize_instincts()

    def _initialize_instincts(self):
        """Ebene 1: Injektion der Sicherheits-Logik (Safety Layer)."""
        # Modus: Training (0)
        self.robot.set_mode(0)

        # Regel: Wenn WAND -> DREHEN (Gewicht 1.0 = Zwingender Reflex)
        # teach(input, action, weight) kapselt API_Teach
        self.robot.teach(self.T_SENSOR_WALL, self.T_ACT_TURN, 1.0)
        print("[Init] Instinkt aktiviert: WALL_AHEAD -> TURN_LEFT\n")

        # Modus: Produktion (1) für stabilen Betrieb
        self.robot.set_mode(1)

    def run(self, steps=15):
        distance_to_wall = 5

        for step in range(steps):
            # A. Sensorik vorbereiten
            inputs = []
            if distance_to_wall > 0:
                inputs.append(self.T_SENSOR_FREE)
                state_msg = f"PATH_FREE (Dist: {distance_to_wall})"
            else:
                inputs.append(self.T_SENSOR_WALL)
                state_msg = "WALL_AHEAD!"

            # B. Denken (O(1) Entscheidung)
            decision = self.robot.update(inputs)

            # C. Physik & Belohnungslogik
            reward = 0.0
            action_name = "UNKNOWN"

            if decision == self.T_ACT_MOVE:
                action_name = "MOVE_FWD"
                if distance_to_wall > 0:
                    distance_to_wall -= 1
                    reward = 0.5  # Belohnung für Fortschritt
                else:
                    reward = -2.0 # Crash (Sollte dank Reflex blockiert sein)
                    print("CRASH! ", end="")
            
            elif decision == self.T_ACT_TURN:
                action_name = "TURN_LEFT"
                if distance_to_wall == 0:
                    distance_to_wall = 5 # Neue Bahn gefunden
                    reward = 1.0  # Hohe Belohnung für Gefahrenvermeidung
                else:
                    reward = -0.1 # Unnötiges Drehen

            # D. Lernen (Feedback-Schleife)
            # feedback(reward, action) kapselt API_Feedback
            self.robot.feedback(reward, decision)

            print(f"[Step {step:2}] Sensor: {state_msg:25} -> Action: {action_name:10} | Reward: {reward:5.2f}")

            # E. Debug: Inspektion der Synapsenstärke (De-Salting aktiv)
            if distance_to_wall > 0 and step % 5 == 0:
                confidence = self.robot.inspect(self.T_SENSOR_FREE, self.T_ACT_MOVE)
                print(f"   -> [Debug] Synapse (Free->Move) Strength: {confidence:.4f}")

        # F. Speichern (Serialisierung)
        brain_data = self.robot.serialize()
        if brain_data:
            with open("robot_brain.bio", "wb") as f:
                f.write(brain_data)
            print("\n[System] Brain serialized and saved to 'robot_brain.bio'.")

    def shutdown(self):
        # Ressourcenfreigabe im Core
        self.robot.close()

if __name__ == "__main__":
    sim = RobotSimulation()
    try:
        sim.run()
    finally:
        sim.shutdown()