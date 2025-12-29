import time
from bioai import BioBrainInstance

class DatevSanityCheck:
    # --- 1. TOKEN DEFINITIONEN (Kategorien & Logik) ---
    # OBJECT Cluster: Buchungskonten
    T_REVENUE_4400 = 0x1000000000004400
    T_BANK_1200    = 0x1000000000001200
    T_VAT_19       = 0x1000000000000013

    # ACTION Cluster: Validierungsergebnisse
    T_VALID        = 0x2000000000000001
    
    # LOGIC Cluster: Fehler-Reflexe (0x4010...)
    T_ERROR_NO_VAT = 0x4010000000000001

    @staticmethod
    def run_example(key_path: str, dll_path: str):
        # Initialisierung über Context-Manager (RAII)
        with BioBrainInstance(key_path, dll_path) as brain:
            
            # --- 2. DEFINITION DER KAUSALEN LOGIK (Instincts) ---
            # Modus: Training
            brain.set_mode(0)

            # Regel 1: Erlös ohne Steuer -> Fehler (Gewicht 1.0)
            brain.teach(DatevSanityCheck.T_REVENUE_4400, 
                        DatevSanityCheck.T_ERROR_NO_VAT, 1.0)

            # Regel 2: Erlös + MwSt -> Gültig (Gewicht 1.0)
            # Hinweis: BioAI verschmilzt kombinierte Inputs zu einem kausalen Token
            combined_input = DatevSanityCheck.T_REVENUE_4400 ^ DatevSanityCheck.T_VAT_19
            brain.teach(combined_input, DatevSanityCheck.T_VALID, 1.0)

            # Modus: Produktion (Fixierung der Logik)
            brain.set_mode(1)

            print("--- BioAI v0.7.6: Starting Causal Sanity Check (Python) ---")

            # --- 3. ECHTZEIT-VALIDIERUNG ---
            
            # Testfall A: Unvollständige Buchung (Erlös ohne Steuer)
            inputs_a = [DatevSanityCheck.T_REVENUE_4400]
            result_a = brain.update(inputs_a)
            
            status_a = "ALARM: Missing VAT" if result_a == DatevSanityCheck.T_ERROR_NO_VAT else "Unknown"
            print(f"Result A (Revenue only): {status_a}")

            # Testfall B: Korrekte Buchung (Erlös + Steuer + Bank)
            inputs_b = [DatevSanityCheck.T_REVENUE_4400, DatevSanityCheck.T_VAT_19, DatevSanityCheck.T_BANK_1200]
            result_b = brain.update(inputs_b)
            
            status_b = "PASS: Logically Consistent" if result_b == DatevSanityCheck.T_VALID else "FAIL"
            print(f"Result B (Complete Booking): {status_b}")

            # --- 4. PERFORMANCE-NOTE ---
            # Die Verarbeitung erfolgt in O(1) Zeitkomplexität.
            # Selbst bei Millionen von Buchungssätzen bleibt die Latenz konstant.

if __name__ == "__main__":
    # Pfade zu den ISS-Artefakten
    DatevSanityCheck.run_example("bin/key.json", "bin/BioAI_ULTRA.dll")