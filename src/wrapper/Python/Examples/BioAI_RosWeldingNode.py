import time
from bioai import BioBrainInstance

class RosWeldingNode:
    # --- 1. TOKEN DEFINITIONEN (Ontologie) ---
    # ACTIONS: Trajektorien-Punkte (5mm Schritte)
    T_WELD_START = 0x2000000000000001
    T_WELD_P1    = 0x2000000000000002
    T_WELD_P2    = 0x2000000000000003
    T_WELD_P3    = 0x2000000000000004
    T_WELD_END   = 0x2000000000000005

    # OBJECTS & REFLEXES
    T_ROS_HAND_SCAN  = 0x1000000000000001 # Laser-Scanner Topic Input
    T_EMERGENCY_STOP = 0x4010000000000001 # Hard-Reflex (Interrupt)

    def __init__(self, json_path: str, dll_path: str):
        # Initialisierung mit dem Security-Key
        self._brain = BioBrainInstance(json_path, dll_path)
        self._setup_ros_safety()

    def _setup_ros_safety(self):
        """Ebene 1: Injektion der Sicherheits-Instinkte."""
        self._brain.set_mode(0) # Training

        # Der Sicherheits-Reflex: Überschreibt jede aktive Planung
        # Gewicht 1.0 = Höchste Priorität in der bio_think_logic
        self._brain.teach(self.T_ROS_HAND_SCAN, self.T_EMERGENCY_STOP, 1.0)

        self._brain.set_mode(1) # Produktion (Deterministischer Freeze)
        print("[ROS2] Schweiß-Safety-Kernel stabilisiert.")

    def execute_welding_task(self):
        """Simuliert den ROS 2 Trajektorien-Callback."""
        # 20mm Pfad als Sequenz laden
        path_20mm = [self.T_WELD_START, self.T_WELD_P1, self.T_WELD_P2, self.T_WELD_P3, self.T_WELD_END]
        
        # In v0.7.6 erfolgt die Planung über die Inferenz-Logik
        print("[ROS2] Schweiß-Task (20mm) gestartet...")

        for i, step_token in enumerate(path_20mm):
            # 1. Topic-Abfrage (Simulierter Laser-Scanner)
            hand_detected = self._check_ros_laser_scanner(i)
            
            inputs = []
            if hand_detected:
                inputs.append(self.T_ROS_HAND_SCAN)

            # 2. BioAI Denkvorgang (O(1) Entscheidung)
            decision = self._brain.update(inputs)

            # 3. Reflex-Prüfung
            if decision == self.T_EMERGENCY_STOP:
                self._publish_ros_stop()
                print("[ROS2] !! TASK ABGEBROCHEN !!")
                break

            # 4. Normaler Pfad-Befehl
            self._publish_ros_joint_command(decision)
            time.sleep(0.2) # Pfad-Verarbeitungszeit

    # --- ROS 2 Schnittstellen-Simulatoren ---
    def _check_ros_laser_scanner(self, step_index):
        # Simuliert eine Hand-Erkennung bei Schritt 2 (10mm Punkt)
        return step_index == 2

    def _publish_ros_joint_command(self, action):
        names = {self.T_WELD_START: "START", self.T_WELD_END: "20mm ENDE"}
        action_name = names.get(action, "PFAD_PUNKT")
        print(f"[ROS2 PUB] Sende Trajektorie für: {action_name}")

    def _publish_ros_stop(self):
        print("[ROS2 PUB] !!! EMERGENCY_STOP an JointController gesendet !!!")

    def shutdown(self):
        self._brain.close()

if __name__ == "__main__":
    node = RosWeldingNode("bin/key.json", "bin/BioAI_ULTRA.dll")
    try:
        node.execute_welding_task()
    finally:
        node.shutdown()