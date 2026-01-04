const { BioBrainInstance } = require('./BioAI'); // Nutzt den v0.7.6 Wrapper

class BioNpcEntity {
    /**
     * --- 1. ONTOLOGIE (Die Welt des NPCs) ---
     * TokenIDs werden als BigInt definiert, um 64-Bit Präzision zu wahren.
     */
    // Wahrnehmung (OBJECTS / NEEDS / TIME)
    static T_SEE_PLAYER = 0x1000000000000001n; // OBJECT Cluster
    static T_SEE_GOLD = 0x1000000000000002n; // OBJECT Cluster
    static T_HP_LOW = 0x5000000000000001n; // SELF/NEED Cluster
    static T_IS_NIGHT = 0x3000000000000001n; // TIME Cluster

    // Aktionen (ACTIONS)
    static T_ACTION_ATTACK = 0x2000000000000001n;
    static T_ACTION_FLEE = 0x2000000000000002n;
    static T_ACTION_GATHER = 0x2000000000000003n;
    static T_ACTION_TRADE = 0x2000000000000004n;

    // Reflexe (Angeborenes Verhalten)
    static T_REFLEX_HEAL = 0x4010000000000001n; // LOGIC/REFLEX Cluster

    constructor(jsonPath, dllPath) {
        // Initialisierung des Gehirns
        this._brain = new BioBrainInstance(jsonPath, dllPath);
        this._initializeNpcPersonality();
    }

    /** Initialisiert die Basis-Persönlichkeit und Instinkte. */
    _initializeNpcPersonality() {
        // Modus: Training (0) für Wissensinjektion
        this._brain.setMode(0);

        // Ebene 1: Instinkte (Hard-Wiring)
        // Wenn HP kritisch -> IMMER Heilen (Reflex mit Gewicht 1.0)
        this._brain.teach(BioNpcEntity.T_HP_LOW, BioNpcEntity.T_REFLEX_HEAL, 1.0);

        // Ebene 2: Biases (Kontext)
        // Nachts ist der NPC vorsichtiger (Tendenz zur Flucht bei Sichtung)
        this._brain.teach(BioNpcEntity.T_IS_NIGHT, BioNpcEntity.T_ACTION_FLEE, 0.4);

        // Modus: Produktion (1) - NPC ist bereit
        this._brain.setMode(1);
        console.log("[NPC] BioAI-Brain initialisiert und einsatzbereit.");
    }

    /** Die Update-Schleife des NPCs. */
    onUpdate(playerVisible, goldVisible, currentHp, nightTime) {
        // 2. Wahrnehmung: Aktuelle Situation erfassen
        let inputs = [];
        if (playerVisible) inputs.push(BioNpcEntity.T_SEE_PLAYER);
        if (goldVisible) inputs.push(BioNpcEntity.T_SEE_GOLD);
        if (currentHp < 20) inputs.push(BioNpcEntity.T_HP_LOW);
        if (nightTime) inputs.append(BioNpcEntity.T_IS_NIGHT);

        // 3. Denken: BioAI wählt die beste Aktion (O(1))
        const action = this._brain.update(inputs);

        // 4. Handeln & Lernen (Ebene 3: Erfahrung)
        this._processAction(action, playerVisible);
    }

    _processAction(action, playerNearby) {
        if (action === BioNpcEntity.T_REFLEX_HEAL) {
            console.log("[NPC] NUTZT HEILTRANK! (Priorisierter Reflex)");
        }
        else if (action === BioNpcEntity.T_ACTION_ATTACK) {
            console.log("[NPC] GREIFT AN!");
            // Lernen: Negatives Feedback bei schmerzhafter Erfahrung
            this._brain.feedback(-0.5, BioNpcEntity.T_ACTION_ATTACK);
        }
        else if (action === BioNpcEntity.T_ACTION_TRADE && playerNearby) {
            console.log("[NPC] BIETET HANDEL AN.");
            // Lernen: Erfolgreicher Handel gibt positive Verstärkung
            this._brain.feedback(0.8, BioNpcEntity.T_ACTION_TRADE);
        }
        else if (action === BioNpcEntity.T_ACTION_GATHER) {
            console.log("[NPC] SAMMELT GOLD.");
        }
    }

    shutdown() {
        this._brain.close();
    }
}