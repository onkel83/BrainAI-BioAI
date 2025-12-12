const ffi = require('ffi-napi');
const ref = require('ref-napi');
const fs = require('fs');
const path = require('path');

// --- TYPES ---
const uint64 = ref.types.uint64;
const voidPtr = ref.refType(ref.types.void);
const float = ref.types.float;
const int = ref.types.int;
const intPtr = ref.refType(int);

// --- DLL BINDING ---
// Automatic detection of platform and library name
const libName = process.platform === 'win32' ? 'bioai.dll' : 'libbioai.so';
// Assuming the binary is in a 'bin' folder relative to this script
const libPath = path.join(__dirname, '../bin', process.platform === 'win32' ? 'windows' : 'linux', libName);

let lib;
try {
    lib = ffi.Library(libPath, {
        'API_CreateBrain': [voidPtr, [uint64]],
        'API_FreeBrain':   ['void', [voidPtr]],
        'API_SetMode':     ['void', [voidPtr, int]],
        
        'API_Update':      [uint64, [voidPtr, 'pointer', int]],
        'API_Simulate':    [uint64, [voidPtr, 'pointer', int, int]], 
        
        'API_Feedback':    ['void', [voidPtr, float, uint64]],
        'API_Teach':       ['void', [voidPtr, uint64, uint64, float]],
        'API_Inspect':     [float,  [voidPtr, uint64, uint64]],
        
        'API_LoadPlan':    ['void', [voidPtr, 'pointer', int, int]], 
        'API_AbortPlan':   ['void', [voidPtr]],
        'API_GetPlanStatus': [int,  [voidPtr]], 

        'API_Serialize':   [voidPtr, [voidPtr, intPtr]],
        'API_Deserialize': [voidPtr, ['pointer', int]],
        'API_FreeBuffer':  ['void', [voidPtr]]
    });
} catch (e) {
    console.error(`[BioAI] FATAL: Could not load native library at ${libPath}.\nEnsure you have renamed 'BioAI_Ultra.dll' (or IoT/SmartHome) to '${libName}'.`, e);
    process.exit(1);
}

// --- CONSTANTS ---
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

// Vocabulary Registry (for debugging)
const vocabulary = new Map();

class BioAI {
    
    constructor(seed) {
        this.handle = lib.API_CreateBrain(seed);
        if (this.handle.isNull()) throw new Error("BioAI Init Failed (Out of Memory or DLL Error)");
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
        
        const sizeBuf = Buffer.alloc(4); // Allocate int pointer
        const dataPtr = lib.API_Serialize(this.handle, sizeBuf);
        
        if (dataPtr.isNull()) return;

        const size = sizeBuf.readInt32LE(0); // Read output size
        
        // Read native memory into Buffer
        const data = ref.reinterpret(dataPtr, size);
        fs.writeFileSync(pathStr, Buffer.from(data)); 
        
        lib.API_FreeBuffer(dataPtr); // Important: Free C memory
    }

    load(pathStr) {
        if (!fs.existsSync(pathStr)) throw new Error(`File not found: ${pathStr}`);
        
        const data = fs.readFileSync(pathStr);
        
        // Deserialize creates a NEW brain handle
        const newHandle = lib.API_Deserialize(data, data.length);
        
        if (!newHandle.isNull()) {
            if (this.handle) lib.API_FreeBrain(this.handle); // Free old brain
            this.handle = newHandle;
        } else {
            throw new Error("Deserialization failed (Corrupt data or version mismatch).");
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

    // --- STATIC HELPERS ---
    
    /**
     * Creates a deterministic 64-bit FNV-1a Hash Token.
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
            hash = (hash * prime) & mask64; // Strict overflow simulation
        }

        // Apply Cluster Mask
        const finalToken = (hash & 0x00FFFFFFFFFFFFFFn) | BigInt(cluster);
        
        if (!vocabulary.has(finalToken)) {
            vocabulary.set(finalToken, name);
        }
        
        return finalToken;
    }

    static dumpVocabulary(pathStr) {
        let content = "BioAI Token Export (Node.js)\n----------------------------\n";
        
        // Sort by Token ID for cleaner output
        const sortedKeys = Array.from(vocabulary.keys()).sort((a, b) => (a < b ? -1 : a > b ? 1 : 0));
        
        for (const key of sortedKeys) {
            const name = vocabulary.get(key);
            // BigInt to Hex String
            const hex = key.toString(16).toUpperCase().padStart(16, '0');
            content += `0x${hex} | ${name}\n`;
        }
        fs.writeFileSync(pathStr, content);
    }
}

module.exports = { BioAI, BioMode: MODE, BioClusters: CLUSTER };
