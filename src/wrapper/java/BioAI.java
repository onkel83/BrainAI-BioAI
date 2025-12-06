package com.brainai.bioai;

import com.sun.jna.Library;
import com.sun.jna.Native;
import com.sun.jna.Pointer;
import com.sun.jna.ptr.IntByReference;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.util.HashMap;
import java.util.Map;

/**
 * BioAI Java Wrapper (v0.0.2 Alpha)
 * The Universal Neuro-Symbolic Engine.
 */
public class BioAI implements AutoCloseable {

    // =============================================================
    // 1. CLUSTER & CONSTANTS
    // =============================================================
    
    public static final long CLUSTER_OBJECT = 0x1000000000000000L;
    public static final long CLUSTER_ACTION = 0x2000000000000000L;
    public static final long CLUSTER_TIME   = 0x3000000000000000L;
    public static final long CLUSTER_LOGIC  = 0x4000000000000000L;
    public static final long CLUSTER_SELF   = 0x5000000000000000L;

    public static final long SUB_REFLEX     = CLUSTER_LOGIC | 0x0010000000000000L;
    public static final long SUB_NEED       = CLUSTER_SELF  | 0x0010000000000000L;
    public static final long SUB_GOAL       = CLUSTER_SELF  | 0x0020000000000000L;

    public enum BioMode {
        TRAINING(0),
        PRODUCTION(1);
        
        public final int value;
        BioMode(int v) { this.value = v; }
    }

    // Vocabulary for Debugging
    private static final Map<Long, String> vocabulary = new HashMap<>();

    /**
     * Erstellt eine TokenID (FNV-1a Hash).
     * Identisch zur C++ und C# Implementierung.
     */
    public static long createToken(String name, long cluster) {
        if (name == null || name.isEmpty()) return 0;

        // FNV-1a 64-bit Hash Constants
        long hash = 0xcbf29ce484222325L; // FNV_offset_basis
        long prime = 0x100000001b3L;     // FNV_prime

        byte[] bytes = name.getBytes(StandardCharsets.UTF_8);
        for (byte b : bytes) {
            hash ^= (b & 0xff); // unsigned cast
            hash *= prime;
        }

        // Maskierung & Cluster setzen
        long finalToken = (hash & 0x00FFFFFFFFFFFFFFL) | cluster;
        
        synchronized(vocabulary) {
            vocabulary.put(finalToken, name);
        }
        return finalToken;
    }

    public static void dumpVocabulary(String path) {
        StringBuilder sb = new StringBuilder();
        sb.append("BioAI Token Export\n------------------\n");
        synchronized(vocabulary) {
            for (Map.Entry<Long, String> entry : vocabulary.entrySet()) {
                sb.append(String.format("0x%016X | %s\n", entry.getKey(), entry.getValue()));
            }
        }
        try {
            Files.write(Paths.get(path), sb.toString().getBytes());
        } catch (Exception e) {
            System.err.println("Failed to dump vocabulary: " + e.getMessage());
        }
    }

    // =============================================================
    // 2. NATIVE INTERFACE (JNA)
    // =============================================================

    private interface BioLib extends Library {
        // Lädt automatisch "libbioai.so" (Linux/Android) oder "bioai.dll" (Windows)
        BioLib INSTANCE = Native.load("bioai", BioLib.class);

        Pointer API_CreateBrain(long key);
        void API_FreeBrain(Pointer ptr);
        
        void API_SetMode(Pointer ptr, int mode);
        
        long API_Update(Pointer ptr, long[] inputs, int count);
        long API_Simulate(Pointer ptr, long[] inputs, int count, int depth);
        
        void API_Feedback(Pointer ptr, float reward, long action);
        void API_Teach(Pointer ptr, long input, long action, float weight);
        float API_Inspect(Pointer ptr, long input, long action);
        
        // Sequencer
        void API_LoadPlan(Pointer ptr, long[] steps, int count, int strict);
        void API_AbortPlan(Pointer ptr);
        int API_GetPlanStatus(Pointer ptr);

        // Serialization
        Pointer API_Serialize(Pointer ptr, IntByReference outSize);
        Pointer API_Deserialize(Pointer data, int size);
        void API_FreeBuffer(Pointer buffer);
    }

    // =============================================================
    // 3. WRAPPER CLASS
    // =============================================================

    private Pointer brain;
    private boolean closed = false;

    /**
     * Erstellt einen neuen Agenten.
     * @param licenseKey Lizenzschlüssel (Salt).
     */
    public BioAI(long licenseKey) {
        brain = BioLib.INSTANCE.API_CreateBrain(licenseKey);
        if (brain == null) throw new OutOfMemoryError("BioAI Core Init Failed.");
    }

    // Private Konstruktor für Deserialisierung
    private BioAI(Pointer ptr) {
        this.brain = ptr;
    }

    public void setMode(BioMode mode) {
        check();
        BioLib.INSTANCE.API_SetMode(brain, mode.value);
    }

    public long think(long... inputs) {
        check();
        if (inputs == null || inputs.length == 0) return 0;
        return BioLib.INSTANCE.API_Update(brain, inputs, inputs.length);
    }

    public long simulate(long[] inputs, int depth) {
        check();
        if (inputs == null || inputs.length == 0) return 0;
        return BioLib.INSTANCE.API_Simulate(brain, inputs, inputs.length, depth);
    }

    public void learn(float reward, long action) {
        check();
        BioLib.INSTANCE.API_Feedback(brain, reward, action);
    }

    public void forceInstinct(long input, long action, float weight) {
        check();
        BioLib.INSTANCE.API_Teach(brain, input, action, weight);
    }

    public float inspect(long input, long action) {
        check();
        return BioLib.INSTANCE.API_Inspect(brain, input, action);
    }

    // --- Sequencer ---
    
    public void loadPlan(long[] steps, boolean strict) {
        check();
        if (steps != null)
            BioLib.INSTANCE.API_LoadPlan(brain, steps, steps.length, strict ? 1 : 0);
    }
    
    public void abortPlan() {
        check();
        BioLib.INSTANCE.API_AbortPlan(brain);
    }

    // --- Serialization ---

    public void saveToFile(String path) throws Exception {
        check();
        IntPtrByReference sizeRef = new IntByReference();
        Pointer buffer = BioLib.INSTANCE.API_Serialize(brain, sizeRef);
        
        if (buffer == null) throw new Exception("Serialization returned NULL");
        
        try {
            byte[] data = buffer.getByteArray(0, sizeRef.getValue());
            Files.write(Paths.get(path), data);
        } finally {
            BioLib.INSTANCE.API_FreeBuffer(buffer);
        }
    }

    public static BioAI loadFromFile(String path) throws Exception {
        byte[] data = Files.readAllBytes(Paths.get(path));
        
        // JNA Memory Handling für den Input Buffer
        com.sun.jna.Memory mem = new com.sun.jna.Memory(data.length);
        mem.write(0, data, 0, data.length);
        
        Pointer newPtr = BioLib.INSTANCE.API_Deserialize(mem, data.length);
        if (newPtr == null) throw new Exception("Deserialization failed.");
        
        return new BioAI(newPtr);
    }

    // --- Cleanup ---

    private void check() {
        if (closed) throw new IllegalStateException("BioBrain is closed.");
    }

    @Override
    public void close() {
        if (!closed && brain != null) {
            BioLib.INSTANCE.API_FreeBrain(brain);
            brain = null;
            closed = true;
        }
    }
}
