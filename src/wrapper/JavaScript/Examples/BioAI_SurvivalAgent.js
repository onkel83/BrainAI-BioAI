const { BioBrainInstance } = require('./BioAI'); // Nutzt den v0.7.6 Wrapper

class SurvivalAgent {
    // --- 1. TOKEN DEFINITIONEN (Bedürfnisse & Umwelt) ---
    // NEEDS / STATUS Cluster
    static T_HUNGRY = 0x5000000000000001n;
    static T_TIRED = 0x5000000000000002n;
    static T_HEALTH_LOW = 0x5000000000000003n;

    // OBJECTS Cluster
    static T_FOOD_SEE = 0x1000000000000001n;

    // ACTIONS Cluster
    static T_SEARCH = 0x2000000000000001n;
    static T_EAT = 0x2000000000000002n;
    static T_SLEEP = 0x2000000000000003n;

    // REFLEX Cluster (0x4010...)
    static T_PANIC_REST = 0x4010000000000001n;

    constructor(jsonPath, dllPath) {
        // Initialisierung des Gehirns
        this._brain = new BioBrainInstance(jsonPath, dllPath);

        // Agenten-Status
        this._hunger = 0;    // 0 = satt, 100 = verhungert
        this._energy = 100;  // 100 = fit, 0 = erschöpft
        this._isAlive = true;

        this._initializeKnowledge();
    }

    /** Ebene 1: Instinkte & Basis-Tendenzen injizieren. */
    _initializeKnowledge() {
        this._brain.setMode(0); // Training

        // Reflex: Wenn Gesundheit kritisch -> Notfall-Ruhe (Gewicht 1.0)
        this._brain.teach(SurvivalAgent.T_HEALTH_LOW, SurvivalAgent.T_PANIC_REST, 1.0);

        // Bias: Grundtendenz zum Essen bei Hunger (Gewicht 0.3)
        this._brain.teach(SurvivalAgent.T_HUNGRY, SurvivalAgent.T_EAT, 0.3);

        this._brain.setMode(1); // Produktion
        console.log("[AGENT] Instinkte geladen. Überlebenskampf beginnt...");
    }

    /** Hauptschleife des Überlebenskampfes. */
    async runSimulation() {
        let ticks = 0;
        while (this._isAlive && ticks < 50) {
            ticks++;
            this._updateEnvironment();

            // 2. Wahrnehmung (Perception Mapping)
            let perception = [];
            if (this._hunger > 50) perception.push(SurvivalAgent.T_HUNGRY);
            if (this._energy < 30) perception.push(SurvivalAgent.T_TIRED);

            if (this._hunger > 80 || this._energy < 10) {
                perception.push(SurvivalAgent.T_HEALTH_LOW);
            }

            const foodVisible = Math.random() > 0.7;
            if (foodVisible) perception.push(SurvivalAgent.T_FOOD_SEE);

            // 3. Denken (O(1) Entscheidung)
            const action = this._brain.update(perception);

            // 4. Aktion & Feedback (Erfahrungslernen)
            this._executeAction(action, foodVisible);

            const actionName = this._getBigIntTokenName(action);
            console.log(`Tick ${ticks.toString().padStart(2, '0')} | Hunger: ${this._hunger.toFixed(0).padStart(3)} | Energy: ${this._energy.toFixed(0).padStart(3)} | Action: ${actionName}`);

            if (this._hunger >= 100 || this._energy <= 0) {
                this._isAlive = false;
            }

            await new Promise(resolve => setTimeout(resolve, 100));
        }

        console.log(this._isAlive ? "[AGENT] Simulation beendet." : "[AGENT] Der Agent ist verstorben.");
    }

    _executeAction(action, foodAvailable) {
        if (action === SurvivalAgent.T_EAT) {
            if (foodAvailable) {
                this._hunger = Math.max(0, this._hunger - 40);
                this._brain.feedback(1.0, SurvivalAgent.T_EAT); // Belohnung
            } else {
                this._hunger += 5;
                this._brain.feedback(-0.5, SurvivalAgent.T_EAT); // Bestrafung
            }
        }
        else if (action === SurvivalAgent.T_SLEEP || action === SurvivalAgent.T_PANIC_REST) {
            this._energy = Math.min(100, this._energy + 30);
            this._hunger += 5;
        }
        else { // T_SEARCH
            this._energy -= 10;
            this._hunger += 10;
        }
    }

    _updateEnvironment() {
        this._hunger += 5;
        this._energy -= 2;
    }

    _getBigIntTokenName(t) {
        if (t === SurvivalAgent.T_EAT) return "EAT";
        if (t === SurvivalAgent.T_SLEEP) return "SLEEP";
        if (t === SurvivalAgent.T_PANIC_REST) return "PANIC_REST";
        return "SEARCH";
    }

    shutdown() {
        this._brain.close();
    }
}

// Start
const agent = new SurvivalAgent("key.json", "BioAI_ULTRA.dll");
agent.runSimulation().finally(() => agent.shutdown());