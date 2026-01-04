const BioBrainInstance = require('./BioAI'); // Importiert den angepassten Wrapper

class AutonomousSolarSystem {
    /**
     * --- 1. TOKEN DEFINITIONEN (Cluster-Masken gemäß ISS-Standard) ---
     * Wir nutzen BigInt (n-Suffix), um 64-Bit Präzision zu garantieren.
     */
    // OBJECT Cluster (0x1000...)
    static T_SUN_HIGH = 0x1000000000000001n;
    static T_SUN_LOW = 0x1000000000000002n;
    static T_BATT_CRITICAL = 0x1000000000000003n;
    static T_BATT_FULL = 0x1000000000000004n;

    // ACTION Cluster (0x2000...)
    static T_MODE_PERFORMANCE = 0x2000000000000001n;
    static T_MODE_ECO = 0x2000000000000002n;
    static T_CHARGE_ONLY = 0x2000000000000003n;

    // REFLEX / LOGIC Cluster (0x4010...)
    static T_HARD_SHUTDOWN = 0x4010000000000001n;

    constructor(jsonPath, dllPath) {
        // Initialisierung über den JavaScript-Wrapper
        this._brain = new BioBrainInstance(jsonPath, dllPath);
        this._initializeEnergyLogic();
    }

    /** Ebene 1: Instinkte & Basisregeln injizieren. */
    _initializeEnergyLogic() {
        // Modus: Training (0)
        this._brain.setMode(0);

        // Reflex: Wenn Batterie kritisch -> Sofortiger Shutdown (Gewicht 1.0)
        this._brain.teach(AutonomousSolarSystem.T_BATT_CRITICAL, AutonomousSolarSystem.T_HARD_SHUTDOWN, 1.0);

        // Bias: Wenn die Sonne stark scheint, tendiere zum Performance-Modus
        this._brain.teach(AutonomousSolarSystem.T_SUN_HIGH, AutonomousSolarSystem.T_MODE_PERFORMANCE, 0.4);

        // Bias: Wenn die Sonne schwach ist, tendiere zum ECO-Modus
        this._brain.teach(AutonomousSolarSystem.T_SUN_LOW, AutonomousSolarSystem.T_MODE_ECO, 0.5);

        // Modus: Produktion (1) für stabilen Betrieb
        this._brain.setMode(1);
        console.log("[SOLAR] JavaScript Energiemanagement-Kernel aktiv.");
    }

    /** Ein Simulations-Zyklus: Sensor -> Denken -> Handeln. */
    processCycle(lux, batteryPercent) {
        let activeInputs = [];

        // 1. Sensor-Abstraktion (Mapping auf TokenIDs)
        if (lux > 50000) activeInputs.push(AutonomousSolarSystem.T_SUN_HIGH);
        else if (lux < 5000) activeInputs.push(AutonomousSolarSystem.T_SUN_LOW);

        if (batteryPercent < 10) activeInputs.push(AutonomousSolarSystem.T_BATT_CRITICAL);
        else if (batteryPercent > 95) activeInputs.push(AutonomousSolarSystem.T_BATT_FULL);

        // 2. Denken: KI wählt die effizienteste Strategie (O(1))
        const action = this._brain.update(activeInputs);

        // 3. Ausführung & Feedback
        this._executeEnergyAction(action, batteryPercent);
    }

    _executeEnergyAction(action, currentBattery) {
        // Wichtig: Vergleich erfolgt ebenfalls als BigInt
        if (action === AutonomousSolarSystem.T_HARD_SHUTDOWN) {
            console.log("!!! [REFLEX] KRITISCHER ENERGIEZUSTAND: SHUTDOWN !!!");
        }
        else if (action === AutonomousSolarSystem.T_MODE_PERFORMANCE) {
            console.log(`[SOLAR] Mode: Performance (Batt: ${currentBattery}%)`);
            // Belohnung geben (Learn), wenn die Batterie stabil bleibt
            if (currentBattery > 50) {
                this._brain.feedback(0.2, AutonomousSolarSystem.T_MODE_PERFORMANCE);
            }
        }
        else if (action === AutonomousSolarSystem.T_MODE_ECO) {
            console.log(`[SOLAR] Mode: ECO (Batt: ${currentBattery}%)`);
        }
    }

    /** Saubere Freigabe der nativen Ressourcen. */
    shutdown() {
        this._brain.close();
    }
}

// --- HAUPTPROGRAMM ---
async function main() {
    const DLL_PATH = "BioAI_ULTRA.dll";
    const KEY_PATH = "key.json";

    const solarSystem = new AutonomousSolarSystem(KEY_PATH, DLL_PATH);

    try {
        console.log("\nSzenario 1: Mittagssonne...");
        solarSystem.processCycle(60000, 80);

        console.log("\nSzenario 2: Dämmerung & niedriger Stand...");
        solarSystem.processCycle(1000, 45);

        console.log("\nSzenario 3: Tiefentladungsschutz...");
        solarSystem.processCycle(2000, 5);
    } catch (err) {
        console.error("Fehler in der Solar-Simulation:", err);
    } finally {
        solarSystem.shutdown();
    }
}

main();