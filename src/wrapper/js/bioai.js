const ffi = require('ffi-napi');
const ref = require('ref-napi');
const fs = require('fs');
const path = require('path');

// --- TYPEN ---
const uint64 = ref.types.uint64;
const voidPtr = ref.refType(ref.types.void);
const float = ref.types.float;
const int = ref.types.int;
const intPtr = ref.refType(int);

// --- DLL BINDING ---
// Wir versuchen, die Lib intelligent zu finden (Linux/Windows)
const libName = process.platform === 'win32' ? 'bioai.dll' : 'libbioai.so';
const libPath = path.join(__dirname, '../bin', process.platform === 'win32' ? 'windows' : 'linux', libName);

let lib;
try {
    lib = ffi.Library(libPath, {
        'API_CreateBrain': [voidPtr, [uint64]],
        'API_FreeBrain':   ['void', [voidPtr]],
        'API_SetMode':     ['void', [voidPtr, int]],
        
        'API_Update':      [uint64, [voidPtr, 'pointer', int]],
        'API_Simulate':    [uint64, [voidPtr, 'pointer', int, int]], // NEU
        
        'API_Feedback':    ['void', [voidPtr, float, uint64]],
        'API_Teach':       ['void', [voidPtr, uint64, uint64, float]],
        'API_Inspect':     [float,  [voidPtr, uint64, uint64]],
        
        'API_LoadPlan':    ['void', [voidPtr, 'pointer', int, int]], // NEU
        'API_AbortPlan':   ['void', [voidPtr]], // NEU
        'API_GetPlanStatus': [int,  [voidPtr]], // NEU

        'API_Serialize':   [voidPtr, [voidPtr, intPtr]],
        'API_Deserialize': [voidPtr, ['pointer', int]],
        'API_FreeBuffer':  ['void', [voidPtr]]
    });
} catch (e) {
    console.error(`[BioAI] FATAL: Could not load library at ${libPath}.`, e);
    process.exit(1);
}

// --- KONSTANTEN ---
const CLUSTER = {
    OBJECT: 0x1000000000000000n,
    ACTION: 0x2000000000000000n,
    TIME:   0x3000000000000000n,
    LOGIC:  0x4000000000000000n,
    SELF:   0x5000000000000000n
};

const MODE = {
    TRAINING: 0,
    PRODUCTION: 1
};

// Vokabelheft (Registry)
const vocabulary = new Map();

class BioAI {
    
    constructor(seed) {
        this.handle = lib.API_CreateBrain(seed);
        if (this.handle.isNull()) throw new Error("BioAI Init Failed (Out of Memory?)");
    }

    setMode(mode) {
        if (!this.handle) return;
        lib.API_SetMode(this.handle, mode);
    }

    think(inputs) {
        if (!this.handle) return 0n;
        const buf = this._toBuffer(inputs);
        return BigInt(lib.API_Update(this.handle, buf, inputs.length));
    }

    simulate(inputs, depth) {
        if (!this.handle) return 0n;
        const buf = this._toBuffer(inputs);
        return BigInt(lib.API_Simulate(this.handle, buf, inputs.length, depth));
    }

    learn(reward, action) {
        if (!this.handle) return;
        lib.API_Feedback(this.handle, reward, BigInt(action));
    }

    forceInstinct(input, action, weight) {
        if (!this.handle) return;
        lib.API_Teach(this.handle, BigInt(input), BigInt(action), weight);
    }

    inspect(input, action) {
        if (!this.handle) return 0.0;
        return lib.API_Inspect(this.handle, BigInt(input), BigInt(action));
    }

    // --- Sequencer ---
    loadPlan(steps, strict) {
        if (!this.handle) return;
        const buf = this._toBuffer(steps);
        lib.API_LoadPlan(this.handle, buf, steps.length, strict ? 1 : 0);
    }

    abortPlan() {
        if (!this.handle) return;
        lib.API_AbortPlan(this.handle);
    }
    
    getPlanStep() {
         if (!this.handle) return -1;
         return lib.API_GetPlanStatus(this.handle);
    }

    // --- System ---

    save(pathStr) {
        if (!this.handle) return;
        const sizeBuf = Buffer.alloc(4);
        const dataPtr = lib.API_Serialize(this.handle, sizeBuf);
        
        if (dataPtr.isNull()) return;

        const size = sizeBuf.readInt32LE(0);
        const data = ref.reinterpret(dataPtr, size);
        fs.writeFileSync(pathStr, Buffer.from(data)); // Copy to managed buffer
        
        lib.API_FreeBuffer(dataPtr);
    }

    load(pathStr) {
        const data = fs.readFileSync(pathStr);
        // Wir erstellen ein neues Handle
        const newHandle = lib.API_Deserialize(data, data.length);
        
        if (!newHandle.isNull()) {
            if (this.handle) lib.API_FreeBrain(this.handle);
            this.handle = newHandle;
        } else {
            throw new Error("Deserialization failed.");
        }
    }

    dispose() {
        if (this.handle && !this.handle.isNull()) {
            lib.API_FreeBrain(this.handle);
            this.handle = null;
        }
    }

    // Helper: JS Array -> C uint64 Buffer
    _toBuffer(arr) {
        const buf = Buffer.alloc(arr.length * 8);
        arr.forEach((v, i) => buf.writeBigUInt64LE(BigInt(v), i * 8));
        return buf;
    }

    // --- STATIC ---
    
    /**
     * Erstellt Token (FNV-1a Hash).
     * Identisch zu C++/C#.
     */
    static createToken(name, cluster) {
        if (!name) return 0n;
        
        // FNV-1a 64-bit constants
        const offset_basis = 14695981039346656037n;
        const prime = 1099511628211n;
        
        let hash = offset_basis;
        const buffer = Buffer.from(name, 'utf8');
        
        for (let i = 0; i < buffer.length; i++) {
            hash = hash ^ BigInt(buffer[i]);
            hash = hash * prime;
            // Wir simulieren hier den 64-Bit Überlauf (Wrapping)
            hash = hash & 0xFFFFFFFFFFFFFFFFn;
        }

        // Maskierung & Cluster
        const finalToken = (hash & 0x00FFFFFFFFFFFFFFn) | BigInt(cluster);
        
        // Registry füllen
        if (!vocabulary.has(finalToken)) {
            vocabulary.set(finalToken, name);
        }
        
        return finalToken;
    }

    static dumpVocabulary(pathStr) {
        let content = "BioAI Token Export (JS)\n-----------------------\n";
        // Sortieren für schöne Ausgabe
        const sortedKeys = Array.from(vocabulary.keys()).sort();
        
        for (const key of sortedKeys) {
            const name = vocabulary.get(key);
            // Hex Formatierung in JS ist etwas fummelig
            const hex = key.toString(16).toUpperCase().padStart(16, '0');
            content += `0x${hex} | ${name}\n`;
        }
        fs.writeFileSync(pathStr, content);
    }
}

module.exports = { BioAI, BioMode: MODE, BioClusters: CLUSTER };
