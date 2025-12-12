import ctypes
import os
import sys
from enum import IntEnum
import platform

# --- TYPES ---
c_uint64 = ctypes.c_uint64
c_float = ctypes.c_float
c_void_p = ctypes.c_void_p
c_int = ctypes.c_int
c_char_p = ctypes.c_char_p

# --- CONSTANTS ---
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

# Vocabulary Registry (Global)
_vocabulary = {}

def create_token(name: str, cluster: int) -> int:
    """
    Creates a deterministic 64-bit TokenID (FNV-1a Hash).
    Matches C++/C# implementation bit-exact.
    """
    if not name: return 0
    
    # FNV-1a 64-bit constants
    hash_val = 14695981039346656037
    prime = 1099511628211
    mask64 = 0xFFFFFFFFFFFFFFFF 

    for char in name.encode('utf-8'):
        hash_val = hash_val ^ char
        hash_val = (hash_val * prime) & mask64

    # Apply Cluster Mask to High Byte
    # Ensure cluster is treated as 64-bit unsigned
    final_token = (hash_val & 0x00FFFFFFFFFFFFFF) | cluster
    
    # Add to registry
    if final_token not in _vocabulary:
        _vocabulary[final_token] = name
        
    return final_token

def dump_vocabulary(path):
    """Exports token registry to file for debugging."""
    try:
        with open(path, "w", encoding="utf-8") as f:
            f.write("BioAI Token Export\n------------------\n")
            for key in sorted(_vocabulary.keys()):
                name = _vocabulary[key]
                f.write(f"0x{key:016X} | {name}\n")
    except Exception as e:
        print(f"[BioAI] Failed to dump vocabulary: {e}")

# --- WRAPPER CLASS ---

class BioAI:
    def __init__(self, seed=12345, lib_path=None):
        """
        Initialize BioAI Core.
        :param seed: License Key / Seed
        :param lib_path: Optional path to specific DLL (e.g. './BioAI_IoT.dll')
        """
        if lib_path is None:
            # Auto-Detect Platform Standard Name
            if sys.platform == "win32":
                lib_name = "bioai.dll"
            elif sys.platform.startswith("linux"):
                lib_name = "libbioai.so"
            elif sys.platform == "darwin":
                lib_name = "libbioai.dylib"
            else:
                raise OSError(f"Unsupported OS: {sys.platform}")
            
            # Try to find in current directory first, then rely on OS loader
            if os.path.exists(lib_name):
                lib_path = os.path.abspath(lib_name)
            else:
                lib_path = lib_name # Let OS find it in PATH/LD_LIBRARY_PATH

        try:
            self.lib = ctypes.CDLL(lib_path)
        except OSError as e:
            print(f"[BioAI] CRITICAL: Could not load library '{lib_path}'.")
            print("Ensure you have renamed 'BioAI_Ultra.dll' (or IoT) to 'bioai.dll' or provide full path.")
            raise e

        # Define Signatures (Strict Types)
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
        self.lib.API_Deserialize.argtypes = [c_void_p, c_int]
        
        self.lib.API_FreeBuffer.argtypes = [c_void_p]

        # Init Core
        self.brain = self.lib.API_CreateBrain(seed)
        if not self.brain:
            raise MemoryError("BioAI Core Init failed (OOM or DLL Error)")

    def set_mode(self, mode: BioMode):
        if self.brain:
            self.lib.API_SetMode(self.brain, int(mode))

    def think(self, inputs: list[int]) -> int:
        if not self.brain: return 0
        arr = (c_uint64 * len(inputs))(*inputs)
        return self.lib.API_Update(self.brain, arr, len(inputs))

    def simulate(self, inputs: list[int], depth=2) -> int:
        if not self.brain: return 0
        arr = (c_uint64 * len(inputs))(*inputs)
        return self.lib.API_Simulate(self.brain, arr, len(inputs), depth)

    def learn(self, reward: float, action: int):
        if self.brain:
            self.lib.API_Feedback(self.brain, c_float(reward), c_uint64(action))

    def force_instinct(self, input_token: int, action_token: int, weight: float):
        if self.brain:
            self.lib.API_Teach(self.brain, c_uint64(input_token), c_uint64(action_token), c_float(weight))

    def inspect(self, input_token: int, action_token: int) -> float:
        if not self.brain: return 0.0
        return self.lib.API_Inspect(self.brain, c_uint64(input_token), c_uint64(action_token))

    # --- Sequencer ---
    def load_plan(self, steps: list[int], strict=True):
        if not self.brain or not steps: return
        arr = (c_uint64 * len(steps))(*steps)
        self.lib.API_LoadPlan(self.brain, arr, len(steps), 1 if strict else 0)

    def abort_plan(self):
        if self.brain:
            self.lib.API_AbortPlan(self.brain)

    def get_plan_step(self) -> int:
        if not self.brain: return -1
        return self.lib.API_GetPlanStatus(self.brain)

    # --- System ---
    def save(self, filepath: str):
        if not self.brain: return
        
        size = c_int(0)
        buf_ptr = self.lib.API_Serialize(self.brain, ctypes.byref(size))
        
        if not buf_ptr: return
        
        # Copy to Python Bytes
        data = ctypes.string_at(buf_ptr, size.value)
        
        # Free C Memory
        self.lib.API_FreeBuffer(buf_ptr)
        
        with open(filepath, "wb") as f:
            f.write(data)

    def load(self, filepath: str):
        if not os.path.exists(filepath):
            raise FileNotFoundError(f"File not found: {filepath}")
            
        with open(filepath, "rb") as f:
            data = f.read()
        
        # Create ctypes buffer from bytes
        data_buffer = (ctypes.c_char * len(data)).from_buffer_copy(data)
        
        # Deserialize creates NEW brain handle
        new_ptr = self.lib.API_Deserialize(ctypes.cast(data_buffer, c_void_p), len(data))
        
        if not new_ptr:
            raise RuntimeError("Deserialization failed (Corrupt Data).")
            
        # Swap handles safely
        if self.brain:
            self.lib.API_FreeBrain(self.brain)
        self.brain = new_ptr

    def dispose(self):
        if self.brain:
            self.lib.API_FreeBrain(self.brain)
            self.brain = None

    def __del__(self):
        self.dispose()
