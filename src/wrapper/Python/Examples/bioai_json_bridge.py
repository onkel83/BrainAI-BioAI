import json
from bioai_runtime import BioBrainInstance # Nutzt den zuvor erstellten Python-Wrapper

class BioAIJsonBridge:
    """
    JSON-Messaging Wrapper für BioAI (v0.7.6).
    Ermöglicht die Interaktion mit dem Kern über standardisierte JSON-Objekte.
    """

    def __init__(self, key_path: str, dll_path: str):
        self._brain = BioBrainInstance(key_path, dll_path)

    def process_json_request(self, json_string: str) -> str:
        """
        Verarbeitet eine JSON-Anfrage und führt die entsprechende API-Aktion aus.
        """
        try:
            request = json.loads(json_string)
            command = request.get("command")
            
            if command == "UPDATE":
                # Konvertiert Hex-Strings aus JSON in uint64 Tokens
                inputs = [int(x, 16) for x in request.get("inputs", [])]
                action = self._brain.update(inputs)
                return json.dumps({"status": "OK", "action": hex(action)})

            elif command == "FEEDBACK":
                reward = float(request.get("reward", 0.0))
                action = int(request.get("action", "0x0"), 16)
                self._brain.feedback(reward, action)
                return json.dumps({"status": "OK", "message": "Feedback applied"})

            elif command == "SET_MODE":
                mode = int(request.get("mode", 1))
                self._brain.set_mode(mode)
                return json.dumps({"status": "OK", "mode": mode})

            return json.dumps({"status": "ERROR", "message": "Unknown command"})

        except Exception as e:
            return json.dumps({"status": "ERROR", "message": str(e)})

    def close(self):
        self._brain.close()