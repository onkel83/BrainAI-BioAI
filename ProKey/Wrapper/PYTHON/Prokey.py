import ctypes
import os
import platform
import sys

# ---------------------------------------------------------------------------
# EXCEPTION CLASSES
# ---------------------------------------------------------------------------

class ProKeyException(Exception):
    """Basis-Klasse für alle ProKey Fehler."""
    pass

class ProKeyHardwareException(ProKeyException):
    """Wird geworfen, wenn der Hardware-RNG versagt (Rückgabe 0)."""
    pass

class ProKeyLibraryNotFound(ProKeyException):
    """Wird geworfen, wenn die DLL/.so Datei nicht gefunden wurde."""
    pass

# ---------------------------------------------------------------------------
# LIBRARY LOADING LOGIC
# ---------------------------------------------------------------------------

def _load_library():
    """Lädt die passende Shared Library für das aktuelle OS."""
    system = platform.system()
    
    # Namen festlegen
    if system == "Windows":
        lib_name = "prokey.dll"
    elif system == "Linux":
        lib_name = "libprokey.so"
    elif system == "Darwin": # MacOS (falls später unterstützt)
        lib_name = "libprokey.dylib"
    else:
        raise ProKeyException(f"Nicht unterstütztes Betriebssystem: {system}")

    # Pfad-Strategie: Wir suchen zuerst im Ordner, in dem dieses Skript liegt
    current_dir = os.path.dirname(os.path.abspath(__file__))
    lib_path = os.path.join(current_dir, lib_name)

    # Wenn nicht dort, versuchen wir es ohne Pfad (System-Pfad / LD_LIBRARY_PATH)
    if not os.path.exists(lib_path):
        lib_path = lib_name

    try:
        dll = ctypes.CDLL(lib_path)
        return dll
    except OSError as e:
        raise ProKeyLibraryNotFound(
            f"Konnte die Bibliothek '{lib_name}' nicht laden. \n"
            f"Suchpfad: {current_dir}\n"
            f"Systemfehler: {e}"
        )

# ---------------------------------------------------------------------------
# WRAPPER CLASS
# ---------------------------------------------------------------------------

class Generator:
    """
    Python Interface für die ProKey Hardware-RNG Engine.
    """
    
    _lib = None

    @classmethod
    def _init_lib(cls):
        """Initialisiert die C-Library Bindings beim ersten Aufruf."""
        if cls._lib is None:
            cls._lib = _load_library()
            
            # Funktionssignatur definieren: uint64_t ProKey_GetRawKey(void)
            cls._lib.ProKey_GetRawKey.argtypes = []
            cls._lib.ProKey_GetRawKey.restype = ctypes.c_uint64

    @classmethod
    def next(cls) -> int:
        """
        Holt den rohen 64-Bit Key von der Hardware.
        
        Returns:
            int: Ein 64-Bit unsigned integer.
        
        Raises:
            ProKeyHardwareException: Wenn die Hardware 0 zurückgibt.
        """
        cls._init_lib()
        
        key = cls._lib.ProKey_GetRawKey()
        
        if key == 0:
            raise ProKeyHardwareException("ProKey Hardware RNG Failure: Could not generate entropy.")
            
        return key

    @classmethod
    def next_string(cls) -> str:
        """
        Holt den Key und gibt ihn als formatierten Hex-String zurück.
        Format: KEY:0x...
        """
        val = cls.next()
        # Formatierung passend zur C-Implementierung (16 Stellen Hex, Uppercase)
        return f"KEY:0x{val:016X}"

    @classmethod
    def to_stream(cls, file_obj):
        """
        Schreibt den Key in ein file-like object (z.B. sys.stdout oder open() file).
        """
        file_obj.write(cls.next_string() + "\n")

    @classmethod
    def to_file(cls, filename: str, append: bool = False):
        """
        Öffnet eine Datei, schreibt den Key und schließt sie.
        """
        mode = "a" if append else "w"
        with open(filename, mode, encoding="utf-8") as f:
            cls.to_stream(f)

# ---------------------------------------------------------------------------
# QUICK TEST (Wenn man python prokey.py direkt ausführt)
# ---------------------------------------------------------------------------
if __name__ == "__main__":
    try:
        print("--- ProKey Python Wrapper Test ---")
        print(f"Key (String): {Generator.next_string()}")
        print(f"Key (Int):    {Generator.next()}")
        print("--- Success ---")
    except Exception as e:
        print(f"FEHLER: {e}")
