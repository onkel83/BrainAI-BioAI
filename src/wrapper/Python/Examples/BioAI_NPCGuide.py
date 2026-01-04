import time
from bioai import BioBrainInstance

class BioNpcEntity:
    # --- 1. ONTOLOGIE (Die Welt des NPCs) ---
    # Wahrnehmung (OBJECTS / NEEDS / TIME)
    T_SEE_PLAYER  = 0x1000000000000001 # OBJECT Cluster
    T_SEE_GOLD    = 0x1000000000000002 # OBJECT Cluster
    T_HP_LOW      = 0x5000000000000001 # SELF/NEED Cluster
    T_IS_NIGHT    = 0x3000000000000001 # TIME Cluster

    # Aktionen (ACTIONS)
    T_ACTION_ATTACK = 0x2000000000000001
    T_ACTION_FLEE   = 0x2000000000000002
    T_ACTION_GATHER = 0x2000000000000003
    T_ACTION_TRADE  = 0x2000000000000004

    # Reflexe (Angeborenes Verhalten)
    T_REFLEX_HEAL   = 0x4010000000000001 # LOGIC/REFLEX Cluster

    def __init__(self, json_path: str, dll_path: str):
        # Initialisierung des Gehirns
        self._brain = BioBrainInstance(json_path, dll_path)
        self._initialize_npc_personality()

    def _initialize_npc_personality(self):
        """Initialisiert die Basis-Persönlichkeit und Instinkte."""
        # Modus: Training (0)
        self._brain.set_mode(0)

        # Ebene 1: Instinkte (Hard-Wiring)
        # Wenn HP kritisch -> IMMER Heilen (Reflex mit Gewicht 1.0)
        self._brain.teach(self.T_HP_LOW, self.T_REFLEX_HEAL, 1.0)

        # Bias: Nachts ist der NPC vorsichtiger (Tendenz zur Flucht bei Sichtung)
        self._brain.teach(self.T_IS_NIGHT, self.T_ACTION_FLEE, 0.4)

        # Modus: Produktion (1) - NPC ist bereit für die Welt
        self._brain.set_mode(1)
        print("[NPC] BioAI-Brain initialisiert und einsatzbereit.")

    def on_update(self, player_visible: bool, gold_visible: bool, current_hp: float, night_time: bool):
        """Die Update-Schleife des NPCs."""
        # 2. Wahrnehmung: Aktuelle Situation erfassen
        inputs = []
        if player_visible: inputs.append(self.T_SEE_PLAYER)
        if gold_visible:   inputs.append(self.T_SEE_GOLD)
        if current_hp < 20: inputs.append(self.T_HP_LOW)
        if night_time:     inputs.append(self.T_IS_NIGHT)

        # 3. Denken: BioAI wählt die beste Aktion (O(1))
        action = self._brain.update(inputs)

        # 4. Handeln & Lernen (Ebene 2: Erfahrung)
        self._process_action(action, player_visible)

    def _process_action(self, action: int, player_nearby: bool):
        if action == self.T_REFLEX_HEAL:
            print("[NPC] NUTZT HEILTRANK! (Priorisierter Reflex)")
        
        elif action == self.T_ACTION_ATTACK:
            print("[NPC] GREIFT AN!")
            # Lernen: Negatives Feedback bei schmerzhafter Erfahrung
            self._brain.feedback(-0.5, self.T_ACTION_ATTACK)
        
        elif action == self.T_ACTION_TRADE and player_nearby:
            print("[NPC] BIETET HANDEL AN.")
            # Lernen: Erfolgreicher Handel gibt positive Verstärkung
            self._brain.feedback(0.8, self.T_ACTION_TRADE)
        
        elif action == self.T_ACTION_GATHER:
            print("[NPC] SAMMELT GOLD.")
        
        elif action == self.T_ACTION_FLEE:
            print("[NPC] FLIEHT IN DIE SCHATTEN...")

    def shutdown(self):
        """Saubere Freigabe der nativen Ressourcen."""
        self._brain.close()

if __name__ == "__main__":
    NPC = BioNpcEntity("bin/key.json", "bin/BioAI_ULTRA.dll")
    
    # Simulation: Spieler nachts mit wenig HP gesichtet
    print("\n--- Simulation 1: Gefahr in der Nacht ---")
    NPC.on_update(player_visible=True, gold_visible=False, current_hp=15.0, night_time=True)

    # Simulation: Handel am Tag
    print("\n--- Simulation 2: Friedlicher Handel am Tag ---")
    NPC.on_update(player_visible=True, gold_visible=True, current_hp=100.0, night_time=False)
    
    NPC.shutdown()