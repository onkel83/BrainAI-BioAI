const { BioBrainInstance } = require('./BioAI'); // Nutzt den v0.7.6 Wrapper
const fs = require('fs');

class RobotSimulation {
    /**
     * --- 1. ONTOLOGIE (Cluster-Definitionen) ---
     * Wahrnehmung (OBJECTS) und Aktionen (ACTIONS) als BigInt.
     */
    static T_SENSOR_FREE = 0x1000000000000001n;
    static T_SENSOR_WALL = 0x1000000000000002n;

    static T_ACT_MOVE = 0x2000000000000001n;
    static T_ACT_TURN = 0x2000000000000002n;

    constructor(keyPath = "key.json", dllPath = "BioAI_ULTRA.dll") {
        console.log("--- BioAI v0.7.6 JavaScript Robot Simulation ---\n");

        // Initialisierung des Rechenkerns
        this.robot = new BioBrainInstance(keyPath, dllPath);
        this._initializeInstincts();
    }

    /** Ebene 1: Injektion der Sicherheits-Logik (Safety Layer). */
    _initializeInstincts() {
        // Modus: Training (0)
        this.robot.setMode(0);

        // Regel: Wenn WAND -> DREHEN (Gewicht 1.0 = Zwingender Reflex)
        this.robot.teach(RobotSimulation.T_SENSOR_WALL, RobotSimulation.T_ACT_TURN, 1.0);

        // Modus: Produktion (1) für stabiles Lernen
        this.robot.setMode(1);
    }

    /** Simuliert die Interaktion zwischen Agent und Umwelt. */
    async runSimulation(steps = 20) {
        let distanceToWall = 3; // Startdistanz

        for (let step = 1; step <= steps; step++) {
            // A. Wahrnehmung (Mapping der Distanz auf Token)
            let inputs = [];
            let stateMsg = "";

            if (distanceToWall > 0) {
                inputs.push(RobotSimulation.T_SENSOR_FREE);
                stateMsg = `FREE (Dist: ${distanceToWall})`;
            } else {
                inputs.push(RobotSimulation.T_SENSOR_WALL);
                stateMsg = "WALL DETECTED!";
            }

            // B. Denken (O(1) Inferenz)
            const decision = this.robot.update(inputs);

            // C. Ausführung & Simulation der Physik
            let reward = 0.0;
            let actionName = "UNKNOWN";

            if (decision === RobotSimulation.T_ACT_MOVE) {
                actionName = "MOVE_FWD";
                if (distanceToWall > 0) {
                    distanceToWall--;
                    reward = 0.2; // Belohnung für Vorwärtskommen
                } else {
                    reward = -1.0; // Bestrafung für Kollision (Sollte durch Reflex verhindert sein)
                    console.log("CRASH! ");
                }
            }
            else if (decision === RobotSimulation.T_ACT_TURN) {
                actionName = "TURN_LEFT";
                if (distanceToWall === 0) {
                    distanceToWall = 5; // Neue Bahn gefunden
                    reward = 1.0;  // Hohe Belohnung für Gefahrenvermeidung
                } else {
                    reward = -0.1; // Unnötiges Drehen
                }
            }

            // D. Lernen (Feedback-Schleife)
            this.robot.feedback(reward, decision);

            console.log(`[Step ${step.toString().padStart(2, ' ')}] Sensor: ${stateMsg.padEnd(25)} -> Action: ${actionName.padEnd(10)} | Reward: ${reward.toFixed(2)}`);

            // E. Debug: Inspektion der Synapsenstärke (De-Salting aktiv)
            if (distanceToWall > 0 && step % 5 === 0) {
                const confidence = this.robot.inspect(RobotSimulation.T_SENSOR_FREE, RobotSimulation.T_ACT_MOVE);
                console.log(`   -> [Debug] Synapse (Free->Move) Strength: ${confidence.toFixed(4)}`);
            }

            await new Promise(resolve => setTimeout(resolve, 100));
        }

        // F. Speichern (Serialisierung)
        const brainData = this.robot.serialize();
        if (brainData) {
            fs.writeFileSync("robot_brain.bio", brainData);
            console.log("\n[System] Brain serialized and saved to robot_brain.bio");
        }
    }

    shutdown() {
        this.robot.close();
    }
}

// Start
async function main() {
    const sim = new RobotSimulation();
    try {
        await sim.runSimulation(30);
    } finally {
        sim.shutdown();
    }
}

main();