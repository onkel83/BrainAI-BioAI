const { BioBrainInstance } = require('./BioAI'); // Nutzt den v0.7.6 Wrapper

class BioAIJsonBridge {
    /**
     * JSON-Messaging Wrapper für BioAI (v0.7.6).
     * Ermöglicht die Interaktion mit dem Kern über standardisierte JSON-Objekte.
     *
     */
    constructor(keyPath, dllPath) {
        this._brain = new BioBrainInstance(keyPath, dllPath);
    }

    /**
     * Verarbeitet eine JSON-Anfrage und führt die entsprechende API-Aktion aus.
     * @param {string} jsonString Das eingehende JSON-Kommando.
     * @returns {string} Das Ergebnis als JSON-String.
     */
    processJsonRequest(jsonString) {
        try {
            const request = JSON.parse(jsonString);
            const command = request.command;

            if (command === "UPDATE") {
                // Konvertiert Hex-Strings aus JSON in BigInt Tokens
                const inputs = (request.inputs || []).map(x => BigInt(x));
                const action = this._brain.update(inputs);

                // Rückgabe der Action-ID als Hex-String
                return JSON.stringify({
                    status: "OK",
                    action: `0x${action.toString(16)}`
                });
            }

            else if (command === "FEEDBACK") {
                const reward = parseFloat(request.reward || 0.0);
                const action = BigInt(request.action || "0x0");
                this._brain.feedback(reward, action);

                return JSON.stringify({ status: "OK", message: "Feedback applied" });
            }

            else if (command === "SET_MODE") {
                const mode = parseInt(request.mode || 1);
                this._brain.setMode(mode);

                return JSON.stringify({ status: "OK", mode: mode });
            }

            return JSON.stringify({ status: "ERROR", message: "Unknown command" });

        } catch (err) {
            return JSON.stringify({ status: "ERROR", message: err.message });
        }
    }

    shutdown() {
        this._brain.close();
    }
}

// --- Beispielnutzung ---
// const bridge = new BioAIJsonBridge("key.json", "BioAI_ULTRA.dll");
// const request = JSON.stringify({ command: "UPDATE", inputs: ["0x1000000000000001"] });
// console.log(bridge.processJsonRequest(request));