const express = require('express');
const ffi = require('ffi-napi');
const ref = require('ref-napi');
const fs = require('fs');

const app = express();
app.use(express.json());

// --- NATIVE BINDINGS (BioAI_Interface.h) ---
const libPath = './bin/BioAI_ULTRA.dll';
const bioai = ffi.Library(libPath, {
    'API_CreateBrain': ['pointer', ['uint64']],
    'API_FreeBrain': ['void', ['pointer']],
    'API_SetMode': ['void', ['pointer', 'int']],
    'API_Update': ['uint64', ['pointer', 'pointer', 'int']],
    'API_Feedback': ['void', ['pointer', 'float', 'uint64']],
    'API_Inspect': ['float', ['pointer', 'uint64', 'uint64']]
});

// --- INITIALISIERUNG ---
let brainHandle = null;
try {
    const keyData = JSON.parse(fs.readFileSync('./config/key.json', 'utf8'));
    // Konvertiere Hex-Key zu BigInt für uint64_t
    const key = BigInt(keyData.customer_key.replace("0x", "").replace("ULL", ""));
    brainHandle = bioai.API_CreateBrain(key);
    bioai.API_SetMode(brainHandle, 1); // Produktionsmodus (Deterministisch)
    console.log("BioAI Core für SAP erfolgreich initialisiert.");
} catch (err) {
    console.error("Initialisierungsfehler:", err);
    process.exit(1);
}

// --- API ENDPUNKTE ---

/**
 * SAP-Inferenz: Sendet Business-Tokens und erhält eine Aktions-Empfehlung
 * POST /api/v1/predict
 */
app.post('/api/v1/predict', (req, res) => {
    const { tokens } = req.body; // Array von Token-Strings (z.B. ["0x1000...", "0x5000..."])

    // Konvertierung der Strings in uint64_t Buffer
    const tokenCount = tokens.length;
    const tokenBuffer = Buffer.alloc(tokenCount * 8);
    tokens.forEach((t, i) => {
        tokenBuffer.writeBigUInt64LE(BigInt(t), i * 8);
    });

    const action = bioai.API_Update(brainHandle, tokenBuffer, tokenCount);
    res.json({
        action: "0x" + action.toString(16).toUpperCase(),
        timestamp: new Date().toISOString()
    });
});

/**
 * SAP-Feedback: Bestätigt oder korrigiert die Entscheidung (Reinforcement Learning)
 * POST /api/v1/feedback
 */
app.post('/api/v1/feedback', (req, res) => {
    const { reward, action } = req.body;
    bioai.API_Feedback(brainHandle, parseFloat(reward), BigInt(action));
    res.sendStatus(200);
});

/**
 * SAP-Audit: Inspiziert die Entscheidungsgrundlage (Transparenz)
 * GET /api/v1/inspect
 */
app.get('/api/v1/inspect', (req, res) => {
    const { input, action } = req.query;
    const weight = bioai.API_Inspect(brainHandle, BigInt(input), BigInt(action));
    res.json({ weight: weight.toFixed(4) });
});

const PORT = process.env.PORT || 3000;
app.listen(PORT, () => console.log(`BioAI SAP Gateway läuft auf Port ${PORT}`));