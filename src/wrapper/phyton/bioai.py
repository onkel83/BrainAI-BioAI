import ctypes
import os
import sys
import datetime
from enum import IntEnum

# --- TYPEN ---
c_uint64 = ctypes.c_uint64
c_float = ctypes.c_float
c_void_p = ctypes.c_void_p
c_int = ctypes.c_int
c_char_p = ctypes.c_char_p

# --- KONSTANTEN (Ontologie) ---
class BioMode(IntEnum):
    TRAINING = 0
    PRODUCTION = 1

class BioClusters:
    OBJECT = 0x1000000000000000
    ACTION = 0x2000000000000000
    TIME   = 0x3000000000000000
    LOGIC  = 0x4000000000000000
    SELF   = 0x5000000000000000

    # Sub-Clusters
    REFLEX = LOGIC | 0x0010000000000000
    NEED   = SELF  | 0x0010000000000000
    GOAL   = SELF  | 0x0020000000000000
    STATUS = SELF  | 0x0030000000000000

# Vokabelheft (Registry)
_vocabulary = {}

def create_token(name, cluster):
    """
    Erstellt eine deterministische TokenID (FNV-1a Hash).
    Identisch zu C++ und C# Implementierung.
    """
    if not name: return 0
    
    # FNV-1a 64-bit Hash
    hash_val = 14695981039346656037
    prime = 1099511628211
    
    # Python int ist beliebig groß, wir müssen auf 64-bit beschränken
    mask = 0xFFFFFFFFFFFFFFFF 

    for char in name.encode('utf-8'):
        hash_val = hash_val ^ char
        hash_val = (hash_val * prime) & mask

    # Cluster ins High-Byte
    final_token = (hash_val & 0x00FFFFFFFFFFFFFF) | cluster
    
    # Registry füllen
    if final_token not in _vocabulary:
        _vocabulary[final_token] = name
        
    return final_token

def dump_vocabulary(path):
    """Exportiert die Token-Namen in eine Datei."""
    with open(path, "w", encoding="utf-8") as f:
        f.write("BioAI Token Export\n------------------\n")
        # Sortieren nach ID
        for key in sorted(_vocabulary.keys()):
            name = _vocabulary[key]
            # Cluster raten für Anzeige
            cluster_name = "MISC"
            c = key & 0xFF00000000000000
            if c == BioClusters.OBJECT: cluster_name = "OBJECT"
            elif c == BioClusters.ACTION: cluster_name = "ACTION"
            elif c == BioClusters.LOGIC: cluster_name = "LOGIC"
            elif c == BioClusters.SELF: cluster_name = "SELF"
            
            f.write(f"0x{key:016X} | {cluster_name:<8} | {name}\n")

# --- WRAPPER KLASSE ---

class BioAI:
    def __init__(self, seed=12345, lib_path=None):
        if lib_path is None:
            # Intelligente Pfad-Suche
            base = os.path.dirname(os.path.abspath(__file__))
            sys_name = sys.platform
            
            if sys_name == "win32":
                path = os.path.join(base, "../bin/windows/bioai.dll")
            elif sys_name == "linux":
                path = os.path.join(base, "../bin/linux/libbioai.so")
            elif sys_name == "darwin": # Mac
                path = os.path.join(base, "../bin/mac/libbioai.dylib")
            else:
                raise Exception(f"Unsupported OS: {sys_name}")
            
            lib_path = os.path.abspath(path)

        try:
            self.lib = ctypes.CDLL(lib_path)
        except OSError as e:
            print(f"[BioAI] CRITICAL: Could not load library at {lib_path}")
            raise e

        # Signaturen definieren (Argtypes/Restypes)
        self.lib.API_CreateBrain.restype = c_void_p
        self.lib.API_CreateBrain.argtypes = [c_uint64]

        self.lib.API_FreeBrain.argtypes = [c_void_p]

        self.lib.API_SetMode.argtypes = [c_void_p, c_int]

        self.lib.API_Update.restype = c_uint64
        self.lib.API_Update.argtypes = [c_void_p, ctypes.POINTER(c_uint64), c_int]

        self.lib.API_Simulate.restype = c_uint64
        self.lib.API_Simulate.argtypes = [c_void_p, ctypes.POINTER(c_uint64), c_int, c_int]

        self.lib.API_Feedback.argtypes = [c_void_p, c_float, c_uint64]
        self.lib.API_Teach.argtypes = [c_void_p, c_uint64, c_uint64, c_float]
        
        self.lib.API_Inspect.restype = c_float
        self.lib.API_Inspect.argtypes = [c_void_p, c_uint64, c_uint64]

        # Sequencer
        self.lib.API_LoadPlan.argtypes = [c_void_p, ctypes.POINTER(c_uint64), c_int, c_int]
        self.lib.API_AbortPlan.argtypes = [c_void_p]
        self.lib.API_GetPlanStatus.restype = c_int
        self.lib.API_GetPlanStatus.argtypes = [c_void_p]

        # Serializer
        self.lib.API_Serialize.restype = c_void_p
        self.lib.API_Serialize.argtypes = [c_void_p, ctypes.POINTER(c_int)]
        
        self.lib.API_Deserialize.restype = c_void_p
        self.lib.API_Deserialize.argtypes = [c_void_p, c_int] # Pointer kann auch byte-array sein
        
        self.lib.API_FreeBuffer.argtypes = [c_void_p]

        # Init
        self.brain = self.lib.API_CreateBrain(seed)
        if not self.brain:
            raise MemoryError("BioAI Core Init failed (Handle is NULL)")

    def set_mode(self, mode: BioMode):
        self.lib.API_SetMode(self.brain, int(mode))

    def think(self, inputs):
        arr = (c_uint64 * len(inputs))(*inputs)
        return self.lib.API_Update(self.brain, arr, len(inputs))

    def simulate(self, inputs, depth=2):
        arr = (c_uint64 * len(inputs))(*inputs)
        return self.lib.API_Simulate(self.brain, arr, len(inputs), depth)

    def learn(self, reward, action):
        self.lib.API_Feedback(self.brain, c_float(reward), c_uint64(action))

    def force_instinct(self, input_token, action_token, weight):
        self.lib.API_Teach(self.brain, c_uint64(input_token), c_uint64(action_token), c_float(weight))

    def inspect(self, input_token, action_token):
        return self.lib.API_Inspect(self.brain, c_uint64(input_token), c_uint64(action_token))

    # --- Sequencer ---
    def load_plan(self, steps, strict=True):
        if not steps: return
        arr = (c_uint64 * len(steps))(*steps)
        self.lib.API_LoadPlan(self.brain, arr, len(steps), 1 if strict else 0)

    def abort_plan(self):
        self.lib.API_AbortPlan(self.brain)

    def get_plan_step(self):
        return self.lib.API_GetPlanStatus(self.brain)

    # --- System ---
    def save(self, filepath):
        size = c_int(0)
        buf_ptr = self.lib.API_Serialize(self.brain, ctypes.byref(size))
        
        if not buf_ptr: return
        
        # Daten kopieren
        data = ctypes.string_at(buf_ptr, size.value)
        self.lib.API_FreeBuffer(buf_ptr)
        
        with open(filepath, "wb") as f:
            f.write(data)

    def load(self, filepath):
        with open(filepath, "rb") as f:
            data = f.read()
        
        # Buffer erstellen (ctypes char array)
        data_buffer = (ctypes.c_char * len(data)).from_buffer_copy(data)
        
        new_ptr = self.lib.API_Deserialize(ctypes.cast(data_buffer, c_void_p), len(data))
        
        if not new_ptr:
            raise Exception("Deserialization failed.")
            
        # Altes Brain freigeben, neues setzen
        if self.brain:
            self.lib.API_FreeBrain(self.brain)
        self.brain = new_ptr

    def dispose(self):
        if self.brain:
            self.lib.API_FreeBrain(self.brain)
            self.brain = None

    def __del__(self):
        self.dispose()
