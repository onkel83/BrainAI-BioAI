# BIOAI SAP BRIDGE
# Dependencies: pip install pyrfc bioai

import sys
import time
from pyrfc import Connection, ABAPApplicationError, LogonError
import bioai # Unser Wrapper

# --- CONFIG ---
SAP_CONFIG = {
    'user': 'BIOAI_USER',
    'passwd': 'PASSWORD123',
    'ashost': '192.168.1.100',
    'sysnr': '00',
    'client': '100'
}

# --- LOGIC ---
def main():
    print("[SAP-Bridge] Connecting to SAP S/4HANA...")
    try:
        conn = Connection(**SAP_CONFIG)
        print("[SAP-Bridge] Connected.")
    except LogonError as e:
        print(f"[FATAL] SAP Login failed: {e}")
        sys.exit(1)

    # BioAI Starten
    brain = bioai.BioAI(seed=999)
    
    # Tokens vorbereiten
    T_STOCK_LOW = bioai.BioAI.create_token("Stock_Low", bioai.BioAI.CLUSTER_OBJECT)
    T_STOCK_OK  = bioai.BioAI.create_token("Stock_OK",  bioai.BioAI.CLUSTER_OBJECT)
    T_ORDER     = bioai.BioAI.create_token("Order_Material", bioai.BioAI.CLUSTER_ACTION)
    
    print("[SAP-Bridge] Entering Loop...")

    while True:
        try:
            # 1. Daten aus SAP holen (RFC Call)
            # Beispiel: Materialbestand abfragen
            result = conn.call('BAPI_MATERIAL_GET_DETAIL', MATERIAL='ROBOT_ARM_V2', PLANT='1000')
            stock_level = float(result['MATERIAL_GENERAL_DATA']['NET_WEIGHT']) # Beispiel-Feld
            
            print(f"[SAP-Data] Stock Level: {stock_level}")

            # 2. Tokenisierung
            current_state = T_STOCK_OK
            if stock_level < 50.0:
                current_state = T_STOCK_LOW
            
            # 3. BioAI Entscheidung
            action = brain.think([current_state])
            
            # 4. SAP steuern
            if action == T_ORDER:
                print(">> DECISION: Ordering new parts...")
                # SAP Bestellung auslösen
                # conn.call('BAPI_PO_CREATE1', ...) 
                # (Fake Call zur Sicherheit im Beispiel)
                
                # Feedback Loop: Hat es geklappt?
                brain.learn(1.0, action)
            else:
                print(">> DECISION: Wait.")

            time.sleep(10) # Alle 10 Sekunden prüfen

        except ABAPApplicationError as e:
            print(f"[SAP Error] {e}")
            time.sleep(60)
        except KeyboardInterrupt:
            print("Stopping...")
            break

if __name__ == "__main__":
    main()
