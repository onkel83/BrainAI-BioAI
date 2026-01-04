const { BioBrainInstance, CLUSTERS } = require('./BioAI'); // Nutzt den v0.7.6 Wrapper

/**
 * Autonomer Fertigungs-Controller für die SAP-Integration.
 * Verbindet ERP-Logik mit deterministischer Shop-Floor-Steuerung.
 */
class AutonomousLine {
    // --- 1. SETUP & ONTOLOGIE ---
    // Aktionen (Produktionslinie 6 Schritte)
    static T_PICK = CLUSTERS.ACTION | 0x01n;
    static T_CLEAN = CLUSTERS.ACTION | 0x02n;
    static T_WELD = CLUSTERS.ACTION | 0x03n;
    static T_POLISH = CLUSTERS.ACTION | 0x04n;
    static T_SCAN = CLUSTERS.ACTION | 0x05n;
    static T_PACK = CLUSTERS.ACTION | 0x06n;

    // SAP & Sicherheit
    static T_SAP_STOCK_LOW = CLUSTERS.OBJECT | 0x11n;
    static T_HAND_DETECTED = CLUSTERS.OBJECT | 0xA1n;
    static T_SAFETY_STOP = CLUSTERS.SUB_LOGIC_REFLEX | 0x99n;

    constructor(keyPath, dllPath) {
        // Initialisierung des BioAI-Kerns
        this._brain = new BioBrainInstance(keyPath, dllPath);
        this._initializeLine();
    }

    /** Ebene 1: Injektion der industriellen Logik & Reflexe. */
    _initializeLine() {
        this._brain.setMode(0); // Training

        // Reflex 1: Arbeitssicherheit (Handerkennung -> Sofortstopp)
        this._brain.teach(AutonomousLine.T_HAND_DETECTED, AutonomousLine.T_SAFETY_STOP, 1.0);

        // Reflex 2: Materialwirtschaft (SAP Bestand niedrig -> Produktion pausieren)
        this._brain.teach(AutonomousLine.T_SAP_STOCK_LOW, AutonomousLine.T_SAFETY_STOP, 0.8);

        this._brain.setMode(1); // Production Mode (Freeze)
        console.log("[SYSTEM] SAP Factory Controller stabilisiert.");
    }

    /** * Simuliert einen Fertigungsauftrag aus SAP. 
     *
     */
    async runProductionOrder(orderId) {
        console.log(`\n[SAP] Verarbeite Auftrag: ${orderId}`);

        // Definition des Fertigungsablaufs
        const plan = [
            AutonomousLine.T_PICK, AutonomousLine.T_CLEAN, AutonomousLine.T_WELD,
            AutonomousLine.T_POLISH, AutonomousLine.T_SCAN, AutonomousLine.T_PACK
        ];

        // Plan in den Sequenzer laden
        this._brain.loadPlan(plan, true);

        while (this._brain.getPlanStatus() !== -1) {
            let sensors = [];
            let currentStep = this._brain.getPlanStatus();

            // Simulation: Hand-Erkennung bei Schritt 3 (Schweißen)
            if (currentStep === 2) {
                // sensors.push(AutonomousLine.T_HAND_DETECTED); // Zum Testen aktivieren
            }

            // Inferenz (O(1)): Prüft Reflexe gegen den Plan
            const action = this._brain.update(sensors);

            if (action === AutonomousLine.T_SAFETY_STOP) {
                console.log("!!! [REFLEX] SICHERHEITSSTOPP: PRODUKTION EINGEFROREN !!!");
                this._brain.abortPlan(); // Sequenz abbrechen
                break;
            }

            this._executeHardwareStep(action, currentStep);
            await new Promise(resolve => setTimeout(resolve, 500));
        }

        if (this._brain.getPlanStatus() === -1) {
            console.log("[SAP] Melde Fertigstellung an S/4HANA...");
            console.log("[SUCCESS] Produktionszyklus abgeschlossen.");
        }
    }

    /** Simulation der Hardware-Ansteuerung. */
    _executeHardwareStep(token, stepIndex) {
        const stepNames = {
            [AutonomousLine.T_PICK]: "Hole Teil",
            [AutonomousLine.T_CLEAN]: "Reinigen",
            [AutonomousLine.T_WELD]: "Schweißen",
            [AutonomousLine.T_POLISH]: "Polieren",
            [AutonomousLine.T_SCAN]: "Qualitätsscan",
            [AutonomousLine.T_PACK]: "Verpacken"
        };
        console.log(` -> Schritt ${stepIndex + 1}: ${stepNames[token] || "Unbekannt"}`);
    }

    shutdown() {
        this._brain.close();
    }
}

// Start der Simulation
const factory = new AutonomousLine("key.json", "BioAI_ULTRA.dll");
factory.runProductionOrder("ORDER_2025_001").finally(() => factory.shutdown());