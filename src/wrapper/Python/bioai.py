import ctypes
import json
import os
from typing import List, Optional

# --- INDUSTRIELLE KONSTANTEN (BioAI_Config.h) ---
# Diese Masken definieren die Cluster-Zugehörigkeit der TokenIDs
CLUSTER_OBJECT      = 0x1000000000000000
CLUSTER_ACTION      = 0x2000000000000000
CLUSTER_TIME        = 0x3000000000000000
CLUSTER_LOGIC       = 0x4000000000000000
CLUSTER_SELF        = 0x5000000000000000
SUB_LOGIC_REFLEX    = 0x4010000000000000

class BioBrainInstance:
    """
    Erweiterter Python-Wrapper für die BioAI-Engine (v0.7.6).
    Unterstützt Inferenz, Training, Sequenzer und Persistenz.
    """

    def __init__(self, json_path: str, dll_path: str = "BioAI_ULTRA.dll"):
        """
        Initialisiert die Instanz und lädt den Sicherheitsschlüssel.
        :param json_path: Pfad zur ISS-generierten key.json.
        :param dll_path: Pfad zur nativen Bibliothek (z.B. BioAI_ULTRA.dll).
        """
        # 1. Laden der nativen Bibliothek
        try:
            self._lib = ctypes.CDLL(os.path.abspath(dll_path))
        except OSError as e:
            raise RuntimeError(f"BioAI: Bibliothek {dll_path} konnte nicht geladen werden: {e}")

        # 2. Key aus JSON laden und konvertieren
        with open(json_path, 'r') as f:
            key_data = json.load(f)
            raw_key = key_data["customer_key"]
            # Konvertierung von Hex-String zu 64-Bit Integer
            self.license_key = int(raw_key.replace("ULL", ""), 16)

        # 3. Funktions-Signaturen definieren
        self._setup_api()

        # 4. Internes Gehirn instanziieren
        self._brain_handle = self._lib.API_CreateBrain(self.license_key)
        if not self._brain_handle:
            raise RuntimeError("BioAI: Initialisierung des Rechenkerns fehlgeschlagen.")

    def _setup_api(self):
        """Konfiguriert Argumente und Rückgabetypen der C-Funktionen."""
        
        # Basis-Management
        self._lib.API_CreateBrain.argtypes = [ctypes.c_uint64]
        self._lib.API_CreateBrain.restype = ctypes.c_void_p
        self._lib.API_FreeBrain.argtypes = [ctypes.c_void_p]
        self._lib.API_SetMode.argtypes = [ctypes.c_void_p, ctypes.c_int]

        # Kognition & Simulation
        self._lib.API_Update.argtypes = [ctypes.c_void_p, ctypes.POINTER(ctypes.c_uint64), ctypes.c_int]
        self._lib.API_Update.restype = ctypes.c_uint64
        self._lib.API_Simulate.argtypes = [ctypes.c_void_p, ctypes.POINTER(ctypes.c_uint64), ctypes.c_int, ctypes.c_int]
        self._lib.API_Simulate.restype = ctypes.c_uint64

        # Lernen & Inspektion
        self._lib.API_Feedback.argtypes = [ctypes.c_void_p, ctypes.c_float, ctypes.c_uint64]
        self._lib.API_Teach.argtypes = [ctypes.c_void_p, ctypes.c_uint64, ctypes.c_uint64, ctypes.c_float]
        self._lib.API_Inspect.argtypes = [ctypes.c_void_p, ctypes.c_uint64, ctypes.c_uint64]
        self._lib.API_Inspect.restype = ctypes.c_float

        # Sequenzer (Neu hinzugefügt)
        self._lib.API_LoadPlan.argtypes = [ctypes.c_void_p, ctypes.POINTER(ctypes.c_uint64), ctypes.c_int, ctypes.c_int]
        self._lib.API_AbortPlan.argtypes = [ctypes.c_void_p]
        self._lib.API_GetPlanStatus.argtypes = [ctypes.c_void_p]
        self._lib.API_GetPlanStatus.restype = ctypes.c_int

        # Serialisierung & Speicher
        self._lib.API_Serialize.argtypes = [ctypes.c_void_p, ctypes.POINTER(ctypes.c_int)]
        self._lib.API_Serialize.restype = ctypes.c_void_p
        self._lib.API_Deserialize.argtypes = [ctypes.c_void_p, ctypes.c_int]
        self._lib.API_Deserialize.restype = ctypes.c_void_p
        self._lib.API_FreeBuffer.argtypes = [ctypes.c_void_p]

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()

    def close(self):
        """Gibt alle nativen Ressourcen frei."""
        if hasattr(self, '_brain_handle') and self._brain_handle:
            self._lib.API_FreeBrain(self._brain_handle)
            self._brain_handle = None

    def set_mode(self, mode: int):
        """0 = Training, 1 = Produktion (Fixed Structure)."""
        self._lib.API_SetMode(self._brain_handle, mode)

    # --- Kognitive Funktionen ---

    def update(self, inputs: List[int]) -> int:
        """Verarbeitet Inputs und liefert die optimale Aktion in O(1)."""
        input_array = (ctypes.c_uint64 * len(inputs))(*inputs)
        return self._lib.API_Update(self._brain_handle, input_array, len(inputs))

    def simulate(self, inputs: List[int], depth: int) -> int:
        """Führt eine interne Simulation (Imagination) durch."""
        input_array = (ctypes.c_uint64 * len(inputs))(*inputs)
        return self._lib.API_Simulate(self._brain_handle, input_array, len(inputs), depth)

    # --- Sequenzer / Plan-Steuerung ---

    def load_plan(self, steps: List[int], strict: bool = True):
        """Lädt eine feste Aktionssequenz in den Sequenzer."""
        steps_array = (ctypes.c_uint64 * len(steps))(*steps)
        self._lib.API_LoadPlan(self._brain_handle, steps_array, len(steps), 1 if strict else 0)

    def abort_plan(self):
        """Bricht die aktuelle Plan-Ausführung sofort ab."""
        self._lib.API_AbortPlan(self._brain_handle)

    def get_plan_status(self) -> int:
        """Gibt den Index des aktuellen Plan-Schritts zurück (-1 wenn inaktiv)."""
        return self._lib.API_GetPlanStatus(self._brain_handle)

    # --- Lernen & Training ---

    def feedback(self, reward: float, action: int):
        """Reinforcement Learning: Passt Verhalten basierend auf Reward an."""
        self._lib.API_Feedback(self._brain_handle, reward, action)

    def teach(self, input_id: int, action_id: int, weight: float):
        """Injiziert eine harte Regel (Reflex) direkt in das LTM."""
        self._lib.API_Teach(self._brain_handle, input_id, action_id, weight)

    def inspect(self, input_id: int, action_id: int) -> float:
        """Liest ein gelerntes Gewicht unter Anwendung des De-Salting aus."""
        return self._lib.API_Inspect(self._brain_handle, input_id, action_id)

    # --- Persistenz ---

    def serialize(self) -> Optional[bytes]:
        """Erzeugt einen binären Snapshot des Wissens."""
        size = ctypes.c_int()
        buffer_ptr = self._lib.API_Serialize(self._brain_handle, ctypes.byref(size))
        if not buffer_ptr:
            return None
        
        # Daten in Python-Bytes kopieren
        result = ctypes.string_at(buffer_ptr, size.value)
        # Nativen Puffer sofort freigeben
        self._lib.API_FreeBuffer(buffer_ptr)
        return result

    def deserialize(self, data: bytes):
        """Rekonstruiert ein Brain aus einem Byte-Stream."""
        # Falls bereits ein Brain existiert, dieses zuerst freigeben
        if hasattr(self, '_brain_handle') and self._brain_handle:
            self._lib.API_FreeBrain(self._brain_handle)
        
        self._brain_handle = self._lib.API_Deserialize(data, len(data))
        if not self._brain_handle:
            raise RuntimeError("BioAI: Deserialisierung fehlgeschlagen.")