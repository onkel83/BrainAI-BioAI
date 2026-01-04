package com.prokey.sdk;

import com.sun.jna.Library;
import com.sun.jna.Native;
import com.sun.jna.Platform;
import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;
import java.io.OutputStream;
import java.io.OutputStreamWriter;

// --------------------------------------------------------------------------
// EXCEPTIONS
// --------------------------------------------------------------------------

class ProKeyHardwareException extends RuntimeException {
    public ProKeyHardwareException() {
        super("ProKey Hardware RNG Failure: Could not generate entropy.");
    }
}

// --------------------------------------------------------------------------
// NATIVE INTERFACE (JNA MAPPING)
// --------------------------------------------------------------------------

interface ProKeyLibrary extends Library {
    // Wir laden die Library "prokey". JNA ergänzt automatisch .dll oder .so
    ProKeyLibrary INSTANCE = Native.load("prokey", ProKeyLibrary.class);

    // Mapping der C-Funktion: uint64_t ProKey_GetRawKey(void)
    // Java hat kein uint64, wir nutzen long (64-bit signed), die Bits bleiben gleich.
    long ProKey_GetRawKey();
}

// --------------------------------------------------------------------------
// PUBLIC WRAPPER CLASS
// --------------------------------------------------------------------------

public class Generator {

    /**
     * Holt den rohen 64-Bit Key von der Hardware.
     * @return long (Die rohen 64 Bits). Achtung: In Java signed, daher ggf. negativ.
     * @throws ProKeyHardwareException Wenn die Hardware 0 zurückgibt.
     */
    public static long next() {
        try {
            long key = ProKeyLibrary.INSTANCE.ProKey_GetRawKey();
            
            if (key == 0) {
                throw new ProKeyHardwareException();
            }
            return key;
        } catch (UnsatisfiedLinkError e) {
            throw new RuntimeException("ProKey Native Library nicht gefunden! " +
                "Bitte prokey.dll (Windows) oder libprokey.so (Linux) in den Classpath legen.", e);
        }
    }

    /**
     * Generiert einen Key und gibt ihn als Hex-String zurück.
     * Format: KEY:0x...
     */
    public static String nextString() {
        long key = next();
        // Wir formatieren es als Hex. Java's %X behandelt Longs korrekt bitweise.
        return String.format("KEY:0x%016X", key);
    }

    /**
     * Schreibt den Key direkt in einen OutputStream (z.B. System.out).
     */
    public static void toStream(OutputStream out) throws IOException {
        BufferedWriter writer = new BufferedWriter(new OutputStreamWriter(out));
        writer.write(nextString());
        writer.newLine();
        writer.flush();
    }

    /**
     * Hängt einen neuen Key an eine Datei an.
     */
    public static void appendToFile(String filename) throws IOException {
        try (BufferedWriter writer = new BufferedWriter(new FileWriter(filename, true))) {
            writer.write(nextString());
            writer.newLine();
        }
    }
}
