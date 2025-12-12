/**
 * BioAI Node.js Wrapper (v1.0.0)
 * Industrial Grade / Multi-Tier Support
 * * Requires: ffi-napi, ref-napi
 * npm install ffi-napi ref-napi
 */

const ffi = require('ffi-napi');
const ref = require('ref-napi');
const fs = require('fs');
const path = require('path');

// --- 1. TYPES & CONSTANTS ---

// Native Types Mapping
const types = {
    uint64: ref.types.uint64,
    float: ref.types.float,
    int: ref.types.int,
    void: ref.types.void,
    voidPtr: ref.refType(ref.types.void),
    intPtr: ref.refType(ref.types.int)
};

/** * Betriebsmodi der Engine 
 */
const BioMode = {
    TRAINING: 0,
    PRODUCTION: 1
};

/**
 * Cluster-Ontologie (Identisch zu C-Core)
 * Nutzung von BigInt für präzise 64-Bit Maskierung.
 */
const BioClusters = {
    // Haupt-Kategorien
    OBJECT: 0x1000000000000000n,
    ACTION: 0x2000000000000000n,
    TIME:   0x3000000000000000n,
    LOGIC:  0x4000000000000000n,
    SELF:   0x5000000000000000n,

    // Sub-Kategorien (Bitwise OR Logic)
    REFLEX: 0x4000000000000000n | 0x0010000000000000n, // LOGIC | SUB_REFLEX
    NEED:   0x5000000000000000n | 0x0010000000000000n, // SELF | SUB_NEED
    GOAL:   0x5000000000000000n | 0x0020000000000000n, // SELF | SUB_GOAL
    STATUS: 0x5000000000000000n | 0x0030000000000000n  // SELF | SUB_STATUS
};

// Globales Vokabular für Debugging
const _vocabulary = new Map();

// --- 2. NATIVE LOADER ---

let lib = null;

/**
 * Lädt die native Core-Bibliothek.
 * Hier wird entschieden, welcher Tier (Ultra/IoT) geladen wird.
 * * @param {string} [customPath] Optionaler Pfad zur .dll/.so. Wenn leer, sucht er 'bioai' im Systempfad.
 */
function loadLibrary(customPath) {
    const defaultName = process.platform === 'win32' ? 'bioai.dll' : 'libbioai.so';
    const targetPath = customPath || path.resolve(__dirname, defaultName);

    try {
        lib = ffi.Library(targetPath, {
            'API_CreateBrain':   [types.voidPtr, [types.uint64]],
            'API_FreeBrain':     [types.void,    [types.voidPtr]],
            'API_SetMode':       [types.void,    [types.voidPtr, types.int]],
            
            'API_Update':        [types.uint64,  [types.voidPtr, 'pointer', types.int]],
            'API_Simulate':      [types.uint64,  [types.voidPtr, 'pointer', types.int, types.int]],
            
            'API_Feedback':      [types.void,    [types.voidPtr, types.float, types.uint64]],
            'API_Teach':         [types.void,    [types.voidPtr, types.uint64, types.uint64, types.float]],
            'API_Inspect':       [types.float,   [types.voidPtr, types.uint64, types.uint64]],
            
            'API_LoadPlan':      [types.void,    [types.voidPtr, 'pointer', types.int, types.int]],
            'API_AbortPlan':     [types.void,    [types.voidPtr]],
            'API_GetPlanStatus': [types.int,     [types.voidPtr]],

            'API_Serialize':     [types.voidPtr, [types.voidPtr, types.intPtr]],
            'API_Deserialize':   [types.voidPtr, ['pointer', types.int]],
            'API_FreeBuffer':    [types.void,    [types.voidPtr]]
        });
        return true;
    } catch (e) {
        console.error(`[BioAI] CRITICAL: Failed to load native library from '${targetPath}'.`);
        console.error("Make sure you have renamed 'BioAI_Ultra.dll' (or IoT/SmartHome) to 'bioai.dll'.");
        throw e;
    }
}

// --- 3. CLASS DEFINITION ---

class BioBrain {
    
    /**
     * Erstellt eine neue KI-Instanz.
     * @param {BigInt|number} seed Lizenzschlüssel/Seed (z.B. 0x1234n)
     * @param {string} [dllPath] Optional: Pfad zur spezifischen Tier-DLL (z.B. './bin/BioAI_IoT.dll')
     */
    constructor(seed, dllPath = null) {
        if (!lib) loadLibrary(dllPath); // Lazy Loading beim ersten Instanziieren
        
        this.handle = lib.API_CreateBrain(BigInt(seed));
        if (this.handle.isNull()) {
            throw new Error("[BioAI] Out Of Memory: Could not create Brain instance.");
        }
        this.disposed = false;
    }

    /**
     * Schaltet den Modus um (ISO Safety).
     * @param {number} mode 0 = Training, 1 = Production
     */
    setMode(mode) {
        this._check();
        lib.API_SetMode(this.handle, mode);
    }

    /**
     * Verarbeitet Inputs und trifft eine Entscheidung (O(1)).
     * @param {BigInt[]} inputs Array von TokenIDs
     * @returns {BigInt} TokenID der Aktion
     */
    think(inputs) {
        this._check();
        if (!inputs || inputs.length === 0) return 0n;
        
        const buf = this._arrayToBuffer(inputs);
        // Rückgabewert explizit zu BigInt casten
        return BigInt(lib.API_Update(this.handle, buf, inputs.length));
    }

    /**
     * Simuliert die Kausalitätskette ("Träumen").
     * @param {BigInt[]} inputs Start-Situation
     * @param {number} depth Tiefe der Simulation
     * @returns {BigInt} Beste Aktion basierend auf Zukunft
     */
    simulate(inputs, depth) {
        this._check();
        if (!inputs || inputs.length === 0) return 0n;

        const buf = this._arrayToBuffer(inputs);
        return BigInt(lib.API_Simulate(this.handle, buf, inputs.length, depth));
    }

    /**
     * Bestärkt oder bestraft die letzte Handlung.
     * @param {number} reward Wert zwischen -1.0 und 1.0
     * @param {BigInt} action Die ausgeführte Aktion
     */
    learn(reward, action) {
        this._check();
        lib.API_Feedback(this.handle, reward, BigInt(action));
    }

    /**
     * Injiziert einen Instinkt (Hardcoded Rule).
     * @param {BigInt} input Auslöser
     * @param {BigInt} action Reaktion
     * @param {number} weight Stärke (1.0 = Gesetz)
     */
    forceInstinct(input, action, weight) {
        this._check();
        lib.API_Teach(this.handle, BigInt(input), BigInt(action), weight);
    }

    /**
     * Debugging: Liest das Gewicht einer Synapse.
     */
    inspect(input, action) {
        this._check();
        return lib.API_Inspect(this.handle, BigInt(input), BigInt(action));
    }

    // --- SEQUENCER ---

    loadPlan(steps, strict) {
        this._check();
        if (!steps || steps.length === 0) return;
        const buf = this._arrayToBuffer(steps);
        lib.API_LoadPlan(this.handle, buf, steps.length, strict ? 1 : 0);
    }

    abortPlan() {
        this._check();
        lib.API_AbortPlan(this.handle);
    }

    getPlanStep() {
        this._check();
        return lib.API_GetPlanStatus(this.handle);
    }

    // --- SERIALIZATION ---

    /**
     * Serialisiert das Gehirn in einen Buffer (Zero-Copy).
     * @returns {Buffer} Binärdaten
     */
    serialize() {
        this._check();
        const sizePtr = Buffer.alloc(4); // int pointer
        const dataPtr = lib.API_Serialize(this.handle, sizePtr);

        if (dataPtr.isNull()) return null;

        try {
            const size = sizePtr.readInt32LE(0);
            if (size <= 0) return null;

            // Daten in Managed Buffer kopieren
            const data = ref.reinterpret(dataPtr, size);
            return Buffer.from(data);
        } finally {
            // WICHTIG: C-Speicher freigeben
            lib.API_FreeBuffer(dataPtr);
        }
    }

    /**
     * Lädt ein Gehirn aus einem Buffer.
     * @param {Buffer} data Binärdaten
     * @returns {BioBrain} Neue Instanz
     */
    static deserialize(data) {
        if (!data || data.length === 0) throw new Error("Data empty");
        if (!lib) loadLibrary(); // Sicherstellen, dass Lib geladen ist

        // Daten müssen für C lesbar sein (Buffer ist standardmäßig okay in Node ffi)
        const newHandle = lib.API_Deserialize(data, data.length);

        if (newHandle.isNull()) {
            throw new Error("Deserialization failed. Version mismatch or corrupt data.");
        }

        // Trick: Private Instanz erzeugen und Handle unterschieben
        const instance = new BioBrain(0n); 
        lib.API_FreeBrain(instance.handle); // Leeres Brain verwerfen
        instance.handle = newHandle;        // Geladenes Brain setzen
        return instance;
    }

    // --- HELPER ---

    save(filePath) {
        const data = this.serialize();
        if (data) fs.writeFileSync(filePath, data);
    }

    load(filePath) {
        const data = fs.readFileSync(filePath);
        // Wir laden in eine neue Instanz und tauschen das Handle aus
        const loaded = BioBrain.deserialize(data);
        
        lib.API_FreeBrain(this.handle);
        this.handle = loaded.handle;
        
        // Verhindern, dass die temporäre 'loaded' Instanz das Handle freed
        loaded.handle = ref.NULL; 
    }

    dispose() {
        if (!this.disposed && !this.handle.isNull()) {
            lib.API_FreeBrain(this.handle);
            this.handle = ref.NULL;
            this.disposed = true;
        }
    }

    _check() {
        if (this.disposed) throw new Error("BioBrain is disposed.");
    }

    _arrayToBuffer(arr) {
        const buf = Buffer.alloc(arr.length * 8);
        arr.forEach((val, i) => buf.writeBigUInt64LE(BigInt(val), i * 8));
        return buf;
    }

    // --- STATIC TOOLS ---

    /**
     * Erstellt Token mit FNV-1a Hash (Bit-Identisch zu C++/C#).
     */
    static createToken(name, cluster) {
        if (!name) return 0n;

        const offset_basis = 14695981039346656037n;
        const prime = 1099511628211n;
        const mask64 = 0xFFFFFFFFFFFFFFFFn;

        let hash = offset_basis;
        const buffer = Buffer.from(name, 'utf8');

        for (let i = 0; i < buffer.length; i++) {
            hash = hash ^ BigInt(buffer[i]);
            hash = (hash * prime) & mask64; // Simulieren des 64-Bit Overflows
        }

        const finalToken = (hash & 0x00FFFFFFFFFFFFFFn) | BigInt(cluster);

        if (!_vocabulary.has(finalToken)) {
            _vocabulary.set(finalToken, name);
        }
        return finalToken;
    }

    static dumpVocabulary(filePath) {
        let content = "BioAI Vocabulary Dump (Node.js)\n-------------------------------\n";
        // Sortieren nach TokenID
        const sorted = [..._vocabulary.entries()].sort((a, b) => (a[0] < b[0] ? -1 : 1));
        
        for (const [key, val] of sorted) {
            const hex = key.toString(16).toUpperCase().padStart(16, '0');
            content += `0x${hex} | ${val}\n`;
        }
        fs.writeFileSync(filePath, content);
    }
}

// Export als sauberes Modul
module.exports = { 
    BioAI, 
    BioMode, 
    BioClusters 
};
