# 📘 BIOAI SAP FACTORY CONTROLLER (v0.7.6)
# Anforderungen: pyrfc, bioai.py (erweiterter Wrapper)

import time
import sys
from pyrfc import Connection
from bioai import BioBrainInstance, CLUSTER_OBJECT, CLUSTER_ACTION, SUB_LOGIC_REFLEX

# --- 1. SETUP & ONTOLOGIE ---
# Aktionen (Produktionslinie 6 Schritte)
T_PICK   = CLUSTER_ACTION | 0x01
T_CLEAN  = CLUSTER_ACTION | 0x02
T_WELD   = CLUSTER_ACTION | 0x03
T_POLISH = CLUSTER_ACTION | 0x04
T_SCAN   = CLUSTER_ACTION | 0x05
T_PACK   = CLUSTER_ACTION | 0x06

# SAP & Sicherheit
T_SAP_STOCK_LOW = CLUSTER_OBJECT | 0x11
T_HAND_DETECTED = CLUSTER_OBJECT | 0xA1
T_SAFETY_STOP   = SUB_LOGIC_REFLEX | 0x99 

# --- 2. SAP KONFIGURATION ---
SAP_CONFIG = {
    'user': 'BIOAI_BOT', 'passwd': 'Password123!', 'ashost': '192.168.1.10',
    'sysnr': '00', 'client': '100'
}

class AutonomousLine:
    def __init__(self):
        # Initialisierung über RAII-Wrapper
        # Lädt Key automatisch aus key.json für das Salting
        self.brain = BioBrainInstance("key.json", "BioAI_ULTRA.dll")
        
        # Sicherheits-Instinkt im Training-Mode injizieren
        self.brain.set_mode(0) # Training
        self.brain.teach(T_HAND_DETECTED, T_SAFETY_STOP, 1.0)
        
        # Produktion schalten: Deterministisch & Allokationsfrei
        self.brain.set_mode(1) 
        
        try:
            self.conn = Connection(**SAP_CONFIG)
            print("[SAP] Verbindung hergestellt.")
        except Exception as e:
            print(f"[ERROR] SAP Verbindung fehlgeschlagen: {e}")
            sys.exit(1)

    def run_production(self):
        print("[Factory] Startvorgang: Prüfe Rohmaterial in SAP...")
        
        # A. PERZEPTION: Bestandsabfrage via SAP RFC
        res = self.conn.call('BAPI_MATERIAL_GET_DETAIL', MATERIAL='RAW_PART_01', PLANT='1000')
        stock = float(res.get('MATERIAL_VALUATIONDATA', {}).get('TOTAL_STOCK', 0))

        if stock < 1.0:
            print(f"[SAP] Bestand zu niedrig ({stock}). Abbruch.")
            return

        # B. PLANUNG: 6-Schritte-Sequenz laden
        production_plan = [T_PICK, T_CLEAN, T_WELD, T_POLISH, T_SCAN, T_PACK]
        self.brain.load_plan(production_plan, strict=True)
        
        print(f"[Line] Plan geladen. Starte Fertigung bei Bestand: {stock}")

        # C. EXECUTION LOOP
        while self.brain.get_plan_status() != -1:
            sensors = [] 
            
            # Simulation: Hand-Erkennung bei Schritt 3 (Schweißen)
            if self.brain.get_plan_status() == 2:
                # sensors.append(T_HAND_DETECTED) # Zum Testen einkommentieren
                pass

            # Inferenz (O(1))
            # Prüft zuerst auf Reflexe, dann auf den Plan
            action = self.brain.update(sensors)

            if action == T_SAFETY_STOP:
                print("!!! [REFLEX] SICHERHEITSSTOPP: PRODUKTION EINGEFROREN !!!")
                self.brain.abort_plan()
                break

            self.execute_hardware_step(action)
            time.sleep(0.5)

        # D. REPORTING
        if self.brain.get_plan_status() == -1:
            print("[SAP] Melde Fertigstellung an S/4HANA...")
            print("[Success] Produktionszyklus abgeschlossen.")

    def execute_hardware_step(self, token):
        step_names = {
            T_PICK: "Hole Teil", T_CLEAN: "Reinigen", 
            T_WELD: "Schweißen", T_POLISH: "Polieren",
            T_SCAN: "Qualitätsscan", T_PACK: "Verpacken"
        }
        print(f" -> Schritt {self.brain.get_plan_status()}: {step_names.get(token, 'Prozessschritt')}")

    def __del__(self):
        # Saubere Freigabe der C-Ressourcen
        if hasattr(self, 'brain'):
            self.brain.close()

if __name__ == "__main__":
    line = AutonomousLine()
    line.run_production()