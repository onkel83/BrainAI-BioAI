# BIOAI SAP FACTORY CONTROLLER
# Anforderungen: pyrfc, bioai (Python-Wrapper)

import time
import sys
from pyrfc import Connection
from bioai import BioAI, BioClusters, create_token

# --- 1. SETUP & ONTOLOGIE ---
# Aktionen (Produktionslinie 6 Schritte)
T_PICK    = create_token("ACTION_PICK_PART",    BioClusters.ACTION)
T_CLEAN   = create_token("ACTION_CLEANING",     BioClusters.ACTION)
T_WELD    = create_token("ACTION_WELDING",      BioClusters.ACTION)
T_POLISH  = create_token("ACTION_POLISHING",    BioClusters.ACTION)
T_SCAN    = create_token("ACTION_QUALITY_SCAN", BioClusters.ACTION)
T_PACK    = create_token("ACTION_PACKAGING",    BioClusters.ACTION)

# SAP & Sicherheit
T_SAP_STOCK_LOW = create_token("SAP_STOCK_CRITICAL", BioClusters.OBJECT)
T_SAFETY_STOP   = create_token("SAFETY_REFLEX_STOP", BioClusters.REFLEX)
T_HAND_DETECTED = create_token("SENSOR_HAND_IN_ZONE", BioClusters.OBJECT)

# --- 2. SAP KONFIGURATION ---
SAP_CONFIG = {
    'user': 'BIOAI_BOT', 'passwd': 'Password123!', 'ashost': '192.168.1.10',
    'sysnr': '00', 'client': '100'
}

class AutonomousLine:
    def __init__(self):
        self.brain = BioAI(seed=0xFACTORY)
        # Sicherheits-Instinkt: Hand im Arbeitsraum = Sofort-Stopp
        self.brain.force_instinct(T_HAND_DETECTED, T_SAFETY_STOP, 1.0)
        self.conn = Connection(**SAP_CONFIG)

    def run_production(self):
        print("[Factory] Startvorgang: Prüfe Rohmaterial in SAP...")
        
        # A. PERZEPTION: Bestandsabfrage via SAP RFC
        res = self.conn.call('BAPI_MATERIAL_GET_DETAIL', MATERIAL='RAW_PART_01', PLANT='1000')
        stock = float(res.get('MATERIAL_VALUATIONDATA', {}).get('TOTAL_STOCK', 0))

        if stock < 1.0:
            print("[SAP] Kein Material vorhanden. Triggere Bestellung...")
            return # Hier könnte BioAI autonom eine Bestellung (PO) auslösen

        # B. PLANUNG: 6-Schritte-Sequenz laden
        production_plan = [T_PICK, T_CLEAN, T_WELD, T_POLISH, T_SCAN, T_PACK]
        self.brain.load_plan(production_plan, strict=True)
        
        print(f"[Line] Plan geladen. Starte Fertigung bei Bestand: {stock}")

        # C. EXECUTION LOOP
        while self.brain.get_plan_status() != -1:
            # Sensor-Simulation (z.B. Laser-Scanner Daten)
            sensors = [] 
            # Falls z.B. eine Hand erkannt wird: sensors.append(T_HAND_DETECTED)

            # KI-Entscheidung (Echtzeit O(1))
            action = self.brain.think(sensors)

            if action == T_SAFETY_STOP:
                print("!!! [REFLEX] SICHERHEITSSTOPP: PRODUKTION EINGEFROREN !!!")
                break

            self.execute_hardware_step(action)
            time.sleep(1)

        # D. REPORTING: Rückmeldung an SAP (Produktion abgeschlossen)
        print("[SAP] Melde Fertigstellung an S/4HANA...")
        # Hier Aufruf von BAPI_PRODORDCONF_CREATE_TT zur Rückmeldung
        print("[Success] Produktionszyklus abgeschlossen.")

    def execute_hardware_step(self, token):
        step_names = {T_PICK: "Hole Teil", T_WELD: "Schweißen", T_PACK: "Verpacken"}
        print(f" -> Ausführung: {step_names.get(token, 'Prozessschritt')}")

if __name__ == "__main__":
    line = AutonomousLine()
    line.run_production()
