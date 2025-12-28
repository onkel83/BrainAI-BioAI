import time
import os
from bioai import BioBrainInstance, CLUSTER_ACTION, CLUSTER_OBJECT, SUB_LOGIC_REFLEX

class AdvancedProductionLine:
    """
    Industrielle Demo einer automatisierten Fertigungsstraße.
    Kombiniert deterministische Abläufe (Pläne) mit Echtzeit-Reflexen.
    """

    # --- 1. TOKEN DEFINITIONEN (Die 6 Produktionsschritte) ---
    # Wir nutzen den CLUSTER_ACTION (0x20...) für motorische Befehle
    T_PICK_PART    = CLUSTER_ACTION | 0x01
    T_SCAN_QR      = CLUSTER_ACTION | 0x02
    T_DRILL        = CLUSTER_ACTION | 0x03
    T_MILL         = CLUSTER_ACTION | 0x04
    T_CLEAN        = CLUSTER_ACTION | 0x05
    T_PLACE_DONE   = CLUSTER_ACTION | 0x06

    # --- 2. SICHERHEIT & SENSOREN ---
    # Sensorik liegt im CLUSTER_OBJECT (0x10...)
    T_SENSOR_LIGHT_BARRIER = CLUSTER_OBJECT | 0xA1
    # Reflexe nutzen die Maske SUB_LOGIC_REFLEX (0x4010...)
    T_EMERGENCY_STOP       = SUB_LOGIC_REFLEX | 0x99 

    def __init__(self, json_path: str, dll_path: str):
        # Initialisierung über den RAII-Wrapper
        self._brain = BioBrainInstance(json_path, dll_path)
        self._setup_production_safety()

    def _setup_production_safety(self):
        """Konfiguriert die Sicherheitsinstinkte der Anlage."""
        # In den Training-Modus schalten, um Wissen zu injizieren
        self._brain.set_mode(0) 

        # Sicherheits-Reflex (Ebene 1): 
        # Wenn Lichtschranke unterbrochen -> Not-Aus mit Priorität 1.0
        self._brain.teach(self.T_SENSOR_LIGHT_BARRIER, self.T_EMERGENCY_STOP, 1.0)

        # In Produktion schalten: Struktur wird deterministisch und allokationsfrei
        self._brain.set_mode(1) 
        print("[SYSTEM] Sicherheits-Reflexe aktiv. System im Production Mode.")

    def run_production_cycle(self):
        """Führt eine vollständige 6-stufige Fertigungssequenz aus."""
        # Definition der Sequenz
        sequence = [
            self.T_PICK_PART, self.T_SCAN_QR, self.T_DRILL, 
            self.T_MILL, self.T_CLEAN, self.T_PLACE_DONE
        ]

        # Laden des Plans in den Sequenzer (Strict Mode)
        self._brain.load_plan(sequence, strict=True)
        print("[PLAN] 6-Schritt-Plan geladen. Starte Fertigung...")

        # Loop solange der Plan aktiv ist
        while self._brain.get_plan_status() != -1:
            inputs = []
            # Wir simulieren den Plan-Schritt Index (0-basiert)
            current_step = self._brain.get_plan_status()

            # Simulation: Jemand tritt in die Lichtschranke bei Schritt 4 (Index 3: Fräsen)
            if current_step == 3:
                print("\n[!] WARNUNG: Lichtschranke unterbrochen!")
                inputs.append(self.T_SENSOR_LIGHT_BARRIER)

            # KI entscheidet: Folgt sie dem Plan oder feuert ein Reflex?
            action = self._brain.update(inputs)

            # Prüfung auf den Not-Aus Reflex
            if action == self.T_EMERGENCY_STOP:
                print("=====================================")
                print("!!! NOT-AUS DURCH BIOAI REFLEX !!!")
                print("=====================================")
                self._brain.abort_plan() # Sofortiger Stopp der Sequenz
                break

            # Aktion ausführen und loggen
            self._log_action(action, current_step)
            time.sleep(0.5) # Bearbeitungszeit simulieren

    def _log_action(self, action: int, step: int):
        """Hilfsmethode zur Visualisierung der gewählten Aktionen."""
        names = {
            self.T_PICK_PART: "Material holen",
            self.T_SCAN_QR: "QR-Code scannen",
            self.T_DRILL: "Bohren",
            self.T_MILL: "Fräsen",
            self.T_CLEAN: "Reinigen",
            self.T_PLACE_DONE: "Ablegen"
        }
        name = names.get(action, "Unbekannt")
        print(f"[Schritt {step}] Führe aus: {name}")

    def shutdown(self):
        """Sauberes Freigeben der Ressourcen."""
        self._brain.close()

# --- HAUPTPROGRAMM ---
if __name__ == "__main__":
    # Pfad zur key.json und der passenden DLL (z.B. ULTRA Tier)
    DEMO_KEY = "key.json"
    DEMO_DLL = "BioAI_ULTRA.dll"

    # Sicherstellen, dass die Dateien existieren
    if os.path.exists(DEMO_KEY) and os.path.exists(DEMO_DLL):
        line = AdvancedProductionLine(DEMO_KEY, DEMO_DLL)
        try:
            line.run_production_cycle()
        finally:
            line.shutdown()
    else:
        print(f"Fehler: {DEMO_KEY} oder {DEMO_DLL} nicht gefunden.")