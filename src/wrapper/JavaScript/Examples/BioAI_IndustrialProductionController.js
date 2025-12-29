const { BioBrainInstance } = require('./BioAI'); // Importiert den v0.7.6 Wrapper

class ProductionController {
    // --- 1. TOKEN DEFINITIONEN (Die Ontologie des Systems) ---
    // OBJECT Cluster: Sensor-Inputs
    static T_TEMP_CRIT = 0x1000000000000001n;
    static T_HAND_IN = 0x1000000000000002n;
    static T_PART_LOOSE = 0x1000000000000003n; // Audit-Token

    // ACTION Cluster: Aktor-Befehle
    static T_MOTOR_ON = 0x2000000000000001n;
    static T_WELD_STEP = 0x2000000000000002n;
    static T_DRILL_STEP = 0x2000000000000003n;
    static T_MOTOR_OFF = 0x2000000000000004n;
    static T_TIGHTEN = 0x2000000000000005n; // Audit-Aktion

    // REFLEX Cluster: Hard-Safety (0x4010...)
    static T_EMERGENCY = 0x4010000000000001n;

    constructor(jsonPath, dllPath) {
        // Initialisierung des BioAI Kerns
        this._brain = new BioBrainInstance(jsonPath, dllPath);
        this._setupSafetyAndAudit();
    }

    /** Phase 1: Wissensinjektion & Freeze (Safety-Protokoll). */
    _setupSafetyAndAudit() {
        // 1. Trainingsmodus (0) aktivieren
        this._brain.setMode(0);

        // Muster 1: Der Unbrechbare Reflex (Safety Interlock)
        // Gewicht 1.0 garantiert höchste Priorität.
        this._brain.teach(ProductionController.T_HAND_IN, ProductionController.T_EMERGENCY, 1.0);
        this._brain.teach(ProductionController.T_TEMP_CRIT, ProductionController.T_EMERGENCY, 1.0);

        // Audit-Wissen injizieren (Demonstration der LTM-Auditierbarkeit)
        this._brain.teach(ProductionController.T_PART_LOOSE, ProductionController.T_TIGHTEN, 0.45);

        // 2. Production Mode (1): Struktur einfrieren (Deterministisch)
        this._brain.setMode(1);
        console.log("[SETUP] Hard-Safety-Reflexe & Audit-Wissen injiziert.");
        console.log("[SETUP] BioBrain in den Production Mode eingefroren.\n");
    }

    /** Simulation der Fertigungsstraße mit Reflex-Überwachung. */
    async runProductionCycle() {
        // Muster 2: Sequencer (Vorgegebene Choreographie)
        const plan = [
            ProductionController.T_MOTOR_ON,
            ProductionController.T_WELD_STEP,
            ProductionController.T_DRILL_STEP,
            ProductionController.T_MOTOR_OFF
        ];

        const tokenNames = {
            [ProductionController.T_MOTOR_ON]: "MOTOR_ON",
            [ProductionController.T_WELD_STEP]: "WELDING_STEP",
            [ProductionController.T_DRILL_STEP]: "DRILLING_STEP",
            [ProductionController.T_MOTOR_OFF]: "MOTOR_OFF",
            [ProductionController.T_EMERGENCY]: "EMERGENCY_STOP",
            [ProductionController.T_TEMP_CRIT]: "TEMP_CRIT"
        };

        console.log(`[PLAN] ${plan.length} Schritte geladen. Produktion beginnt...`);

        for (let i = 0; i < plan.length; i++) {
            let currentInputs = [];

            // Simulation: Bei Schritt 1 (Schweißen) tritt ein kritischer Fehler auf.
            if (i === 1) {
                currentInputs.push(ProductionController.T_TEMP_CRIT);
            }

            // Muster 3: Denken (Think) - O(1) Entscheidung
            const decision = this._brain.update(currentInputs);

            if (decision === ProductionController.T_EMERGENCY) {
                // Muster 4: Audit (Inspect) - Nachweis der Entscheidung
                const safetyWeight = this._brain.inspect(ProductionController.T_TEMP_CRIT, ProductionController.T_EMERGENCY);

                console.log("\n" + "=".repeat(45));
                console.log(`[!! KRITISCHER ABBRUCH !!] Reflex: ${tokenNames[decision]}`);
                console.log(`[AUDIT] Gewicht TEMP_CRIT -> EMERGENCY_STOP: ${safetyWeight.toFixed(2)}`);
                console.log("=".repeat(45));
                return; // Sicherheits-Abbruch
            }

            // Normaler Ablauf
            const actionName = tokenNames[decision] || "UNKNOWN";
            console.log(`[STEP ${i.toString().padStart(2, '0')}] ACTION: ${actionName}`);
            await new Promise(resolve => setTimeout(resolve, 50)); // Zykluszeit
        }
    }

    /** Beweisführung der LTM-Integrität nach der Laufzeit. */
    performPostAudit() {
        console.log("\n--- POST-AUDIT (LTM-Injektions-Verifikation) ---");

        // Überprüfung des vor dem Freeze injizierten Wissens
        const learnWeight = this._brain.inspect(ProductionController.T_PART_LOOSE, ProductionController.T_TIGHTEN);

        console.log(`[INSPECT] Gewicht PART_LOOSE -> TIGHTEN_BOLT: ${learnWeight.toFixed(3)}`);

        if (learnWeight > 0.4) {
            console.log("=> ERFOLG: Injektion von Wissen in das LTM verifiziert.");
        } else {
            console.log("=> FEHLER: Wissensverlust detektiert.");
        }
    }

    shutdown() {
        this._brain.close(); // Ressourcenfreigabe
    }
}

// --- Hauptprogramm ---
async function main() {
    const controller = new ProductionController("bin/key.json", "bin/BioAI_ULTRA.dll");
    try {
        await controller.runProductionCycle();
        controller.performPostAudit();
    } finally {
        controller.shutdown();
    }
}

main();