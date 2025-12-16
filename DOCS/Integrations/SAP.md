# BioAI SAP Bridge (S/4HANA Integration) 🏭

**Version:** 0.5.5 (Industrial Beta / stable fix)
**Technology:** Python (PyRFC) + BioAI Core
**Use Case:** Autonomous Supply Chain Management

---

## 1. Overview

This integration demonstrates how **BioAI** can act as an autonomous agent within an **SAP S/4HANA** environment.
Instead of static MRP runs, BioAI continuously monitors stock levels via RFC (Remote Function Call) and makes real-time procurement decisions based on learned patterns (e.g., delivery delays, seasonal spikes).

### Prerequisites
* **SAP System:** Access to an S/4HANA or ECC instance.
* **User:** A CPIC or Dialog user with RFC authorizations (`S_RFC`).
* **Software:**
    * Python 3.8+
    * `pip install pyrfc` (Requires SAP NW RFC SDK)
    * `bioai.py` and `bioai.dll` (from this repository)

---

## 2. The Bridge Code

This script connects the SAP "Material Management" (MM) module with the BioAI brain.

```python
# BIOAI SAP BRIDGE
# Dependencies: pip install pyrfc

import sys
import time
from pyrfc import Connection, ABAPApplicationError, LogonError, CommunicationError
from bioai import BioAI, BioClusters, create_token

# --- SAP CONFIGURATION ---
SAP_CONFIG = {
    'user':     'BIOAI_BOT',
    'passwd':   'Secret123!',
    'ashost':   '192.168.1.100',
    'sysnr':    '00',
    'client':   '100',
    'lang':     'EN'
}

# --- MAIN LOOP ---
def main():
    print("[SAP-Bridge] Connecting to SAP S/4HANA...")
    
    try:
        conn = Connection(**SAP_CONFIG)
        print("[SAP-Bridge] Connection established.")
    except LogonError as e:
        print(f"[FATAL] Login failed: {e}")
        sys.exit(1)
    except CommunicationError as e:
        print(f"[FATAL] Network error: {e}")
        sys.exit(1)

    # 1. Initialize BioAI (using the local DLL)
    # Using a fixed seed ensures reproducible behavior during testing.
    brain = BioAI(seed=0xSAP2025)
    
    # 2. Define Ontology (Vocabulary)
    # These tokens map physical SAP concepts to the AI's internal logic.
    T_STOCK_CRITICAL = create_token("Stock_Critical", BioClusters.OBJECT)
    T_STOCK_NORMAL   = create_token("Stock_Normal",   BioClusters.OBJECT)
    T_ACTION_ORDER   = create_token("Create_PO",      BioClusters.ACTION)
    
    # 3. Inject Safety Rule (Instinct)
    # "If stock is critical, ALWAYS order." (Hard Safety)
    brain.force_instinct(T_STOCK_CRITICAL, T_ACTION_ORDER, 1.0)
    
    print("[SAP-Bridge] Autonomous Agent active. Monitoring Loop started.")

    while True:
        try:
            # --- STEP A: PERCEPTION (RFC Read) ---
            # Call BAPI to get current stock level for a material
            result = conn.call('BAPI_MATERIAL_GET_DETAIL', 
                               MATERIAL='ROBOT_ARM_V2', 
                               PLANT='1000', 
                               VALUATIONAREA='1000')
            
            # Parse SAP Return Structure
            # Note: In a real scenario, handle 'RETURN' messages for errors.
            current_stock = float(result.get('MATERIAL_VALUATIONDATA', {}).get('TOTAL_STOCK', 0))
            
            print(f"[Sensor] Material: ROBOT_ARM_V2 | Stock: {current_stock}")

            # --- STEP B: TOKENIZATION ---
            # Map continuous value to discrete state token
            current_state = T_STOCK_NORMAL
            if current_stock < 50.0:
                current_state = T_STOCK_CRITICAL
            
            # --- STEP C: COGNITION (Think) ---
            # The AI decides based on instincts (safety) or experience (optimization)
            # O(1) Execution Time
            action = brain.think([current_state])
            
            # --- STEP D: ACTION (RFC Write) ---
            if action == T_ACTION_ORDER:
                print(">> DECISION: Triggering Purchase Order (PO)...")
                
                # In production: Call BAPI_PO_CREATE1 here
                # result_po = conn.call('BAPI_PO_CREATE1', ...)
                
                # Simulate success for this demo
                po_success = True 
                
                if po_success:
                    print("   [SAP] PO Created successfully.")
                    # Reinforce the behavior: Action led to a valid state
                    brain.learn(1.0, action)
            else:
                print(">> DECISION: Wait / No Action.")

            # Heartbeat
            time.sleep(10) 

        except CommunicationError:
            print("[Network] Connection lost. Retrying in 60s...")
            time.sleep(60)
            # Reconnect logic would go here
        except ABAPApplicationError as e:
            print(f"[SAP Error] ABAP Exception: {e}")
            time.sleep(10)
        except KeyboardInterrupt:
            print("\n[SAP-Bridge] Shutdown requested.")
            # Save brain state before exit
            brain.save("sap_agent.bin")
            break

if __name__ == "__main__":
    main()
````

-----

## 3\. Deployment Strategy

1.  **Environment:** Run this script on a secure gateway server or within a Docker container that has access to the SAP network subnet.
2.  **Library Path:** Ensure `bioai.dll` (Windows) or `libbioai.so` (Linux) is in the same folder or in `LD_LIBRARY_PATH`.
3.  **Security:** Use a restricted SAP Service User (`System` type) with minimal RFC privileges for the specific BAPIs used.

-----

**BrainAI** - *Intelligence everywhere.*
Developed by **Sascha A. Köhne (winemp83)**
Product: **BioAI v0.5.5 (Industrial Beta)**
