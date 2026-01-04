const { BioBrainInstance, CLUSTERS } = require('./BioAI'); // Importiert den angepassten Wrapper
const fs = require('fs');

/**
 * Industrielle Demo einer automatisierten Fertigungsstraße.
 * Kombiniert deterministische Abläufe (Pläne) mit Echtzeit-Reflexen.
 */
class AdvancedProductionLine {
    constructor(jsonPath, dllPath) {
        // --- 1. TOKEN DEFINITIONEN (Die 6 Produktionsschritte) ---
        // Wir nutzen CLUSTERS.ACTION (0x20...) für motorische Befehle
        this.T_PICK_PART = CLUSTERS.ACTION | 0x01n;
        this.T_SCAN_QR = CLUSTERS.ACTION | 0x02n;
        this.T_DRILL = CLUSTERS.ACTION | 0x03n;
        this.T_MILL = CLUSTERS.ACTION | 0x04n;
        this.T_CLEAN = CLUSTERS.ACTION | 0x05n;
        this.T_PLACE_DONE = CLUSTERS.ACTION | 0x06n;

        // --- 2. SICHERHEIT & SENSOREN ---
        // Sensorik liegt im CLUSTERS.OBJECT (0x10...)
        this.T_SENSOR_LIGHT_BARRIER = CLUSTERS.OBJECT | 0xA1n;
        // Reflexe nutzen die Maske CLUSTERS.SUB_LOGIC_REFLEX (0x4010...)
        this.T_EMERGENCY_STOP = CLUSTERS.SUB_LOGIC_REFLEX | 0x99n;

        // Initialisierung über den Wrapper
        this._brain = new BioBrainInstance(jsonPath, dllPath);
        this._setupProductionSafety();
    }

    /** Konfiguriert die Sicherheitsinstinkte der Anlage. */
    _setupProductionSafety() {
        // In den Training-Modus schalten (0), um Wissen zu injizieren
        this._brain.setMode(0);

        // Sicherheits-Reflex (Ebene 1): 
        // Wenn Lichtschranke unterbrochen -> Not-Aus mit Priorität 1.0
        this._brain.teach(this.T_SENSOR_LIGHT_BARRIER, this.T_EMERGENCY_STOP, 1.0);

        // In Produktion schalten (1): Struktur wird deterministisch
        this._brain.setMode(1);
        console.log("[SYSTEM] Sicherheits-Reflexe aktiv. System im Production Mode.");
    }

    /** Hilfsmethode für Simulationstimer */
    _sleep(ms) {
        return new Promise(resolve => setTimeout(resolve, ms));
    }

    /** Führt eine vollständige 6-stufige Fertigungssequenz aus. */
    async runProductionCycle() {
        // Definition der Sequenz als BigInt Array
        const sequence = [
            this.T_PICK_PART, this.T_SCAN_QR, this.T_DRILL,
            this.T_MILL, this.T_CLEAN, this.T_PLACE_DONE
        ];

        // Laden des Plans in den Sequenzer (Strict Mode)
        this._brain.loadPlan(sequence, true);
        console.log("[PLAN] 6-Schritt-Plan geladen. Starte Fertigung...");

        // Loop solange der Plan aktiv ist (getPlanStatus liefert den Index oder -1)
        while (this._brain.getPlanStatus() !== -1) {
            let inputs = [];
            let currentStep = this._brain.getPlanStatus();

            // Simulation: Jemand tritt in die Lichtschranke bei Schritt 4 (Index 3: Fräsen)
            if (currentStep === 3) {
                console.log("\n[!] WARNUNG: Lichtschranke unterbrochen!");
                inputs.push(this.T_SENSOR_LIGHT_BARRIER);
            }

            // KI entscheidet: Folgt sie dem Plan oder feuert ein Reflex?
            let action = this._brain.update(inputs);

            // Prüfung auf den Not-Aus Reflex
            if (action === this.T_EMERGENCY_STOP) {
                console.log("=====================================");
                console.log("!!! NOT-AUS DURCH BIOAI REFLEX !!!");
                console.log("=====================================");
                this._brain.abortPlan(); // Sofortiger Stopp der Sequenz
                break;
            }

            // Aktion ausführen und loggen
            this._logAction(action, currentStep);
            await this._sleep(500); // Bearbeitungszeit simulieren
        }

        if (this._brain.getPlanStatus() === -1) {
            console.log("[PLAN] Fertigung erfolgreich abgeschlossen.");
        }
    }

    /** Hilfsmethode zur Visualisierung der gewählten Aktionen. */
    _logAction(action, step) {
        const names = {
            [this.T_PICK_PART]: "Material holen",
            [this.T_SCAN_QR]: "QR-Code scannen",
            [this.T_DRILL]: "Bohren",
            [this.T_MILL]: "Fräsen",
            [this.T_CLEAN]: "Reinigen",
            [this.T_PLACE_DONE]: "Ablegen"
        };
        const name = names[action] || "Unbekannt";
        console.log(`[Schritt ${step}] Führe aus: ${name}`);
    }

    /** Sauberes Freigeben der Ressourcen. */
    shutdown() {
        this._brain.close();
    }
}

// --- HAUPTPROGRAMM ---
async function main() {
    const DEMO_KEY = "key.json";
    const DEMO_DLL = "BioAI_ULTRA.dll";

    if (fs.existsSync(DEMO_KEY)) {
        const line = new AdvancedProductionLine(DEMO_KEY, DEMO_DLL);
        try {
            await line.runProductionCycle();
        } finally {
            line.shutdown();
        }
    } else {
        console.error(`Fehler: ${DEMO_KEY} nicht gefunden.`);
    }
}

main();