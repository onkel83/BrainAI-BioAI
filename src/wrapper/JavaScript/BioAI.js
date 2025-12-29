const ffi = require('ffi-napi');
const ref = require('ref-napi');
const fs = require('fs');
const path = require('path');

/**
 * INDUSTRIELLE KONSTANTEN (BioAI_Config.h)
 * Diese Masken definieren die Cluster-Zugehörigkeit der TokenIDs.
 *
 */
const CLUSTERS = {
    OBJECT: 0x1000000000000000n,
    ACTION: 0x2000000000000000n,
    TIME: 0x3000000000000000n,
    LOGIC: 0x4000000000000000n,
    SELF: 0x5000000000000000n,
    SUB_LOGIC_REFLEX: 0x4010000000000000n
};

// Definition der nativen Typen
const uint64 = ref.types.uint64;
const voidPtr = ref.refType(ref.types.void);
const intPtr = ref.refType(ref.types.int);

class BioBrainInstance {
    /**
     * Erweiteter JavaScript-Wrapper für die BioAI-Engine (v0.7.6).
     * Unterstützt Inferenz, Training, Sequenzer und Persistenz.
     *
     */
    constructor(jsonPath, dllPath = "BioAI_ULTRA.dll") {
        // 1. Bibliothek laden
        try {
            this._lib = ffi.Library(path.resolve(dllPath), {
                // Basis-Management
                'API_CreateBrain': [voidPtr, [uint64]],
                'API_FreeBrain': ['void', [voidPtr]],
                'API_SetMode': ['void', [voidPtr, 'int']],
                // Kognition & Simulation
                'API_Update': [uint64, [voidPtr, ref.refType(uint64), 'int']],
                'API_Simulate': [uint64, [voidPtr, ref.refType(uint64), 'int', 'int']],
                // Lernen & Inspektion
                'API_Feedback': ['void', [voidPtr, 'float', uint64]],
                'API_Teach': ['void', [voidPtr, uint64, uint64, 'float']],
                'API_Inspect': ['float', [voidPtr, uint64, uint64]],
                // Sequenzer (Neu in v0.7.6)
                'API_LoadPlan': ['void', [voidPtr, ref.refType(uint64), 'int', 'int']],
                'API_AbortPlan': ['void', [voidPtr]],
                'API_GetPlanStatus': ['int', [voidPtr]],
                // Serialisierung & Speicher
                'API_Serialize': [voidPtr, [voidPtr, intPtr]],
                'API_Deserialize': [voidPtr, [voidPtr, 'int']],
                'API_FreeBuffer': ['void', [voidPtr]]
            });
        } catch (e) {
            throw new Error(`BioAI: Bibliothek ${dllPath} konnte nicht geladen werden: ${e.message}`);
        }

        // 2. Key aus JSON laden
        const keyData = JSON.parse(fs.readFileSync(jsonPath, 'utf8'));
        const rawKey = keyData.customer_key.replace("ULL", "");
        this.licenseKey = BigInt(rawKey);

        // 3. Brain instanziieren
        this._brainHandle = this._lib.API_CreateBrain(this.licenseKey);
        if (this._brainHandle.isNull()) {
            throw new Error("BioAI: Initialisierung des Rechenkerns fehlgeschlagen.");
        }
    }

    /** 0 = Training, 1 = Produktion (Fixed Structure). */
    setMode(mode) {
        this._lib.API_SetMode(this._brainHandle, mode);
    }

    // --- Kognitive Funktionen ---

    /** Verarbeitet Inputs und liefert die optimale Aktion in O(1). */
    update(inputs) {
        const inputBuffer = Buffer.alloc(inputs.length * 8);
        for (let i = 0; i < inputs.length; i++) {
            inputBuffer.writeBigUInt64LE(BigInt(inputs[i]), i * 8);
        }
        return this._lib.API_Update(this._brainHandle, inputBuffer, inputs.length);
    }

    /** Führt eine interne Simulation (Imagination) durch. */
    simulate(inputs, depth) {
        const inputBuffer = Buffer.alloc(inputs.length * 8);
        for (let i = 0; i < inputs.length; i++) {
            inputBuffer.writeBigUInt64LE(BigInt(inputs[i]), i * 8);
        }
        return this._lib.API_Simulate(this._brainHandle, inputBuffer, inputs.length, depth);
    }

    // --- Sequenzer / Plan-Steuerung (Neu in v0.7.6) ---

    /** Lädt eine feste Aktionssequenz in den Sequenzer. */
    loadPlan(steps, strict = true) {
        const stepsBuffer = Buffer.alloc(steps.length * 8);
        for (let i = 0; i < steps.length; i++) {
            stepsBuffer.writeBigUInt64LE(BigInt(steps[i]), i * 8);
        }
        this._lib.API_LoadPlan(this._brainHandle, stepsBuffer, steps.length, strict ? 1 : 0);
    }

    /** Bricht die aktuelle Plan-Ausführung sofort ab. */
    abortPlan() {
        this._lib.API_AbortPlan(this._brainHandle);
    }

    /** Gibt den Index des aktuellen Plan-Schritts zurück (-1 wenn inaktiv). */
    getPlanStatus() {
        return this._lib.API_GetPlanStatus(this._brainHandle);
    }

    // --- Lernen & Training ---

    /** Reinforcement Learning: Passt Verhalten basierend auf Reward an. */
    feedback(reward, action) {
        this._lib.API_Feedback(this._brainHandle, reward, BigInt(action));
    }

    /** Injiziert eine harte Regel (Reflex) direkt in das LTM. */
    teach(input, action, weight) {
        this._lib.API_Teach(this._brainHandle, BigInt(input), BigInt(action), weight);
    }

    /** Liest ein gelerntes Gewicht unter De-Salting aus. */
    inspect(input, action) {
        return this._lib.API_Inspect(this._brainHandle, BigInt(input), BigInt(action));
    }

    // --- Persistenz ---

    /** Erzeugt einen binären Snapshot des Wissens. */
    serialize() {
        const sizePtr = ref.alloc('int');
        const buffer = this._lib.API_Serialize(this._brainHandle, sizePtr);
        if (buffer.isNull()) return null;

        const size = sizePtr.deref();
        const result = Buffer.from(ref.reinterpret(buffer, size, 0));

        // Nativen Puffer sofort freigeben
        this._lib.API_FreeBuffer(buffer);
        return result;
    }

    /** Rekonstruiert ein Brain aus einem Byte-Stream. */
    deserialize(data) {
        if (!this._brainHandle.isNull()) {
            this._lib.API_FreeBrain(this._brainHandle);
        }
        this._brainHandle = this._lib.API_Deserialize(data, data.length);
        if (this._brainHandle.isNull()) {
            throw new Error("BioAI: Deserialisierung fehlgeschlagen.");
        }
    }

    /** Gibt alle nativen Ressourcen frei. */
    close() {
        if (this._brainHandle && !this._brainHandle.isNull()) {
            this._lib.API_FreeBrain(this._brainHandle);
            this._brainHandle = ref.NULL_POINTER;
        }
    }
}

module.exports = { BioBrainInstance, CLUSTERS };