const ffi = require('ffi-napi');
const ref = require('ref-napi');
const fs = require('fs');
const path = require('path');

// Definition der nativen Typen für JavaScript/Node.js
const uint64 = ref.types.uint64;
const voidPtr = ref.refType(ref.types.void);

class BioBrainInstance {
    /**
     * Erzeugt eine neue Instanz und lädt den Schlüssel aus der key.json.
     * @param {string} jsonPath Pfad zur ISS-generierten key.json.
     * @param {string} dllPath Pfad zur nativen DLL/SO.
     */
    constructor(jsonPath, dllPath = "BioAI_ULTRA.dll") {
        // 1. Bibliothek laden
        this._lib = ffi.Library(path.resolve(dllPath), {
            'API_CreateBrain': [voidPtr, [uint64]],
            'API_FreeBrain': ['void', [voidPtr]],
            'API_SetMode': ['void', [voidPtr, 'int']],
            'API_Update': [uint64, [voidPtr, ref.refType(uint64), 'int']],
            'API_Simulate': [uint64, [voidPtr, ref.refType(uint64), 'int', 'int']],
            'API_Feedback': ['void', [voidPtr, 'float', uint64]],
            'API_Teach': ['void', [voidPtr, uint64, uint64, 'float']],
            'API_Inspect': ['float', [voidPtr, uint64, uint64]],
            'API_Serialize': [voidPtr, [voidPtr, ref.refType('int')]],
            'API_FreeBuffer': ['void', [voidPtr]]
        });

        // 2. Key aus JSON laden
        const keyData = JSON.parse(fs.readFileSync(jsonPath, 'utf8'));
        const rawKey = keyData.customer_key.replace("ULL", ""); // Entfernt C-Suffix
        this.licenseKey = BigInt(rawKey);

        // 3. Brain instanziieren
        this._brainHandle = this._lib.API_CreateBrain(this.licenseKey);
        if (this._brainHandle.isNull()) {
            throw new Error("BioAI: Initialisierung der Brain-Instanz fehlgeschlagen.");
        }
    }

    /** 0 = Training, 1 = Produktion (Fixed Structure). */
    setMode(mode) {
        this._lib.API_SetMode(this._brainHandle, mode);
    }

    /** Verarbeitet Inputs und liefert die optimale Aktion in O(1). */
    update(inputs) {
        const inputBuffer = Buffer.alloc(inputs.length * 8);
        for (let i = 0; i < inputs.length; i++) {
            inputBuffer.writeBigUInt64LE(BigInt(inputs[i]), i * 8);
        }
        return this._lib.API_Update(this._brainHandle, inputBuffer, inputs.length);
    }

    /** Führt eine Kausalitäts-Simulation durch. */
    simulate(inputs, depth) {
        const inputBuffer = Buffer.alloc(inputs.length * 8);
        for (let i = 0; i < inputs.length; i++) {
            inputBuffer.writeBigUInt64LE(BigInt(inputs[i]), i * 8);
        }
        return this._lib.API_Simulate(this._brainHandle, inputBuffer, inputs.length, depth);
    }

    /** Passt das Verhalten über Reinforcement Learning an. */
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

    /** Erzeugt einen Snapshot des Gehirns. */
    serialize() {
        const sizePtr = ref.alloc('int');
        const buffer = this._lib.API_Serialize(this._brainHandle, sizePtr);
        if (buffer.isNull()) return null;

        const size = sizePtr.deref();
        const result = Buffer.from(ref.reinterpret(buffer, size, 0));

        // Native Freigabe
        this._lib.API_FreeBuffer(buffer);
        return result;
    }

    /** Gibt alle nativen Ressourcen frei. */
    close() {
        if (!this._brainHandle.isNull()) {
            this._lib.API_FreeBrain(this._brainHandle);
            this._brainHandle = ref.NULL_POINTER;
        }
    }
}

module.exports = BioBrainInstance;