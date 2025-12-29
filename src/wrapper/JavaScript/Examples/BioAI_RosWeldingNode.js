const { BioBrainInstance } = require('./BioAI'); // Nutzt den v0.7.6 Wrapper

class RosWeldingNode {
    /**
     * --- 1. TOKEN DEFINITIONEN (Ontologie) ---
     * ACTIONS: Trajektorien-Punkte (5mm Schritte)
     */
    static T_WELD_START = 0x2000000000000001n;
    static T_WELD_P1 = 0x2000000000000002n;
    static T_WELD_P2 = 0x2000000000000003n;
    static T_WELD_P3 = 0x2000000000000004n;
    static T_WELD_END = 0x2000000000000005n;

    // OBJECTS & REFLEXES
    static T_ROS_HAND_SCAN = 0x1000000000000001n; // Laser-Scanner Topic Input
    static T_EMERGENCY_STOP = 0x4010000000000001n; // Hard-Reflex (Interrupt)

    constructor(jsonPath, dllPath) {
        // Initialisierung mit dem Security-Key
        this._brain = new BioBrainInstance(jsonPath, dllPath);
        this._setupRosSafety();
    }

    /** Ebene 1: Injektion der Sicherheits-Instinkte. */
    _setupRosSafety() {
        this._brain.setMode(0); // Trainingsmodus

        // Der Sicherheits-Reflex: Überschreibt jede aktive Planung
        // Gewicht 1.0 = Höchste Priorität in der bio_think_logic
        this._brain.teach(RosWeldingNode.T_ROS_HAND_SCAN, RosWeldingNode.T_EMERGENCY_STOP, 1.0);

        this._brain.setMode(1); // Produktion (Deterministischer Freeze)
        console.log("[ROS2] Schweiß-Safety-Kernel stabilisiert.");
    }

    /** Simuliert den ROS 2 Callback-Loop. */
    async executeWeldingTask() {
        // 20mm Pfad als Sequenz laden
        const path20mm = [
            RosWeldingNode.T_WELD_START, RosWeldingNode.T_WELD_P1,
            RosWeldingNode.T_WELD_P2, RosWeldingNode.T_WELD_P3, RosWeldingNode.T_WELD_END
        ];

        // Sequenzer laden (Strict Mode)
        this._brain.loadPlan(path20mm, true);
        console.log("[ROS2] Schweiß-Task (20mm) gestartet...");

        while (this._brain.getPlanStatus() !== -1) {
            let currentStep = this._brain.getPlanStatus();

            // 1. Topic-Abfrage (Simulierter Laser-Scanner)
            const handDetected = (currentStep === 2); // Simuliert Hand bei 10mm

            let inputs = [];
            if (handDetected) {
                inputs.push(RosWeldingNode.T_ROS_HAND_SCAN);
            }

            // 2. BioAI Denkvorgang (O(1) Entscheidung)
            const decision = this._brain.update(inputs);

            // 3. Reflex-Prüfung
            if (decision === RosWeldingNode.T_EMERGENCY_STOP) {
                this._publishRosStop();
                this._brain.abortPlan(); // Sequenz abbrechen
                console.log("[ROS2] !! TASK ABGEBROCHEN !!");
                break;
            }

            // 4. Normaler Pfad-Befehl
            this._publishRosJointCommand(decision);
            await new Promise(resolve => setTimeout(resolve, 200)); // Pfad-Verarbeitungszeit
        }
    }

    _publishRosJointCommand(action) {
        const names = { [RosWeldingNode.T_WELD_START]: "START", [RosWeldingNode.T_WELD_END]: "20mm ENDE" };
        const actionName = names[action] || "PFAD_PUNKT";
        console.log(`[ROS2 PUB] Sende Trajektorie für: ${actionName}`);
    }

    _publishRosStop() {
        console.log("[ROS2 PUB] !!! EMERGENCY_STOP an JointController gesendet !!!");
    }

    shutdown() {
        this._brain.close();
    }
}

// Start
const node = new RosWeldingNode("key.json", "BioAI_ULTRA.dll");
node.executeWeldingTask().finally(() => node.shutdown());