const { BioBrainInstance } = require('./BioAI'); // Importiert den v0.7.6 Wrapper

class DatevSanityCheck {
    // --- 1. TOKEN DEFINITIONEN (Kategorien & Logik) ---
    // OBJECT Cluster: Buchungskonten (DATEV-Standard)
    static T_REVENUE_4400 = 0x1000000000004400n;
    static T_BANK_1200 = 0x1000000000001200n;
    static T_VAT_19 = 0x1000000000000013n;

    // ACTION Cluster: Validierungsergebnisse
    static T_VALID = 0x2000000000000001n;

    // LOGIC Cluster: Fehler-Reflexe (0x4010...)
    static T_ERROR_NO_VAT = 0x4010000000000001n;

    /**
     * Führt die Validierungssimulation durch.
     * @param {string} keyPath Pfad zur key.json.
     * @param {string} dllPath Pfad zur BioAI_ULTRA.dll.
     */
    static runExample(keyPath, dllPath) {
        const brain = new BioBrainInstance(keyPath, dllPath);

        try {
            // --- 2. DEFINITION DER KAUSALEN LOGIK (Instincts) ---
            brain.setMode(0); // Modus: Training

            // Regel 1: Erlös ohne Steuer -> Fehler (Gewicht 1.0)
            brain.teach(this.T_REVENUE_4400, this.T_ERROR_NO_VAT, 1.0);

            // Regel 2: Erlös + MwSt -> Gültig (Gewicht 1.0)
            // Hinweis: BioAI verschmilzt kombinierte Inputs via XOR zu einem kausalen Token
            const combinedInput = this.T_REVENUE_4400 ^ this.T_VAT_19;
            brain.teach(combinedInput, this.T_VALID, 1.0);

            brain.setMode(1); // Modus: Produktion (Fixierung der Logik)

            console.log("--- BioAI v0.7.6: Starting Causal Sanity Check (JavaScript) ---");

            // --- 3. ECHTZEIT-VALIDIERUNG ---

            // Testfall A: Unvollständige Buchung (Erlös ohne Steuer)
            const inputsA = [this.T_REVENUE_4400];
            const resultA = brain.update(inputsA);

            const statusA = (resultA === this.T_ERROR_NO_VAT) ? "ALARM: Missing VAT" : "Unknown";
            console.log(`Result A (Revenue only): ${statusA}`);

            // Testfall B: Korrekte Buchung (Erlös + Steuer + Bank)
            const inputsB = [this.T_REVENUE_4400, this.T_VAT_19, this.T_BANK_1200];
            const resultB = brain.update(inputsB);

            const statusB = (resultB === this.T_VALID) ? "PASS: Logically Consistent" : "FAIL";
            console.log(`Result B (Complete Booking): ${statusB}`);

            // Performance-Hinweis: Verarbeitung in O(1) Zeitkomplexität
        } finally {
            brain.close(); // Saubere Ressourcenfreigabe
        }
    }
}

// --- Ausführung ---
const KEY_PATH = "key.json";
const DLL_PATH = "BioAI_ULTRA.dll";
DatevSanityCheck.runExample(KEY_PATH, DLL_PATH);