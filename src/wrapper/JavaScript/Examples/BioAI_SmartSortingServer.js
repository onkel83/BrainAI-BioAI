const { BioBrainInstance } = require('./BioAI'); // Nutzt den v0.7.6 Wrapper
const opcua = require("node-opcua");

class BioOpcSortingServer {
    // --- 1. TOKEN DEFINITIONEN (Die 6 Schritte der Sortierung) ---
    // ACTIONS: Prozessschritte
    static T_SCAN_BARCODE = 0x2000000000000001n;
    static T_WEIGHT_CHECK = 0x2000000000000002n;
    static T_SIZE_CHECK = 0x2000000000000003n;
    static T_ROUTE_A = 0x2000000000000004n;
    static T_ROUTE_B = 0x2000000000000005n;
    static T_LOG_EXIT = 0x2000000000000006n;

    // OBJECTS & REFLEXES
    static T_HAND_DETECTED = 0x1000000000000001n; // Sensor-Input
    static T_EMERGENCY_STOP = 0x4010000000000001n; // Hard-Reflex (Safety)

    constructor(keyPath, dllPath) {
        // Initialisierung des BioAI Kerns
        this._brain = new BioBrainInstance(keyPath, dllPath);
        this._initializeBioLogic();

        // OPC UA Server Setup
        this.server = new opcua.OPCUAServer({
            port: 4840,
            resourcePath: "/freeopcua/server/",
            buildInfo: { productPrefix: "BioAI", productName: "SortingGateway" }
        });
    }

    /** Injektion der industriellen Sicherheits-Logik. */
    _initializeBioLogic() {
        this._brain.setMode(0); // Training

        // Ebene 1: Instinkte - Wenn Hand erkannt -> SOFORT-STOPP
        // Gewicht 1.0 = Höchste Priorität
        this._brain.teach(BioOpcSortingServer.T_HAND_DETECTED, BioOpcSortingServer.T_EMERGENCY_STOP, 1.0);

        this._brain.setMode(1); // Produktion (Deterministischer Freeze)
        console.log("[BioAI] Sorting-Kernel stabilisiert und im Production Mode.");
    }

    async start() {
        await this.server.initialize();
        const addressSpace = this.server.engine.addressSpace;
        const namespace = addressSpace.getOwnNamespace();

        // Objekt-Ordner für die SPS-Kommunikation
        const root = namespace.addObject({
            organizedBy: addressSpace.rootFolder.objects,
            browseName: "SortingAI"
        });

        // --- INPUT NODE (SPS schreibt hierher) ---
        const inputNode = namespace.addVariable({
            componentOf: root,
            browseName: "PLC_Sensor_Input",
            dataType: "UInt64",
            value: { dataType: opcua.DataType.UInt64, value: 0n }
        });

        // --- OUTPUT NODE (SPS liest von hier) ---
        const outputNode = namespace.addVariable({
            componentOf: root,
            browseName: "AI_Action_Output",
            dataType: "UInt64",
            value: { dataType: opcua.DataType.UInt64, value: 0n }
        });

        // Event-Handler: Bei jedem Schreibvorgang der SPS "denkt" die KI
        inputNode.on("value_changed", (newValue) => {
            const plcToken = BigInt(newValue.value.value);
            if (plcToken !== 0n) {
                // BioAI 'Think' Zyklus in O(1)
                const action = this._brain.update([plcToken]);

                outputNode.setValueFromSource({
                    dataType: opcua.DataType.UInt64,
                    value: action
                });

                if (action === BioOpcSortingServer.T_EMERGENCY_STOP) {
                    console.log("!!! [OPC UA] REFLEX STOPP AUSGELÖST !!!");
                }
            }
        });

        await this.server.start();
        console.log("[OPC UA] Server gestartet: opc.tcp://localhost:4840");
    }

    shutdown() {
        this._brain.close();
        this.server.shutdown();
    }
}

// Start
const controller = new BioOpcSortingServer("key.json", "BioAI_ULTRA.dll");
controller.start().catch(console.error);