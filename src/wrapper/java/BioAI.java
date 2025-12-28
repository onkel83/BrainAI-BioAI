package com.bioai.runtime;

import com.sun.jna.Library;
import com.sun.jna.Native;
import com.sun.jna.Pointer;
import com.sun.jna.ptr.IntByReference;
import com.fasterxml.jackson.databind.ObjectMapper;

import java.io.File;
import java.io.IOException;
import java.util.Map;

/**
 * Java-Wrapper für die BioAI-Engine (v0.7.6).
 * Implementiert AutoCloseable für automatische Ressourcenfreigabe.
 */
public class BioBrainInstance implements AutoCloseable {

    // Definition der nativen Schnittstelle
    private interface BioAILibrary extends Library {
        // Name der Bibliothek (ohne Endung, z.B. BioAI_ULTRA)
        BioAILibrary INSTANCE = Native.load("BioAI_ULTRA", BioAILibrary.class);

        Pointer API_CreateBrain(long key);
        void API_FreeBrain(Pointer brainPtr);
        void API_SetMode(Pointer brainPtr, int mode);
        long API_Update(Pointer brainPtr, long[] inputs, int count);
        long API_Simulate(Pointer brainPtr, long[] inputs, int count, int depth);
        void API_Feedback(Pointer brainPtr, float reward, long action);
        void API_Teach(Pointer brainPtr, long input, long action, float weight);
        float API_Inspect(Pointer brainPtr, long input, long action);
        Pointer API_Serialize(Pointer brainPtr, IntByReference outSize);
        Pointer API_Deserialize(byte[] data, int size);
        void API_FreeBuffer(Pointer buffer);
    }

    private Pointer brainHandle;
    private final long licenseKey;

    /**
     * Initialisiert eine neue Brain-Instanz durch Laden des Schlüssels aus einer JSON-Datei.
     * @param jsonPath Pfad zur ISS-generierten key.json.
     */
    public BioBrainInstance(String jsonPath) throws IOException {
        // 1. Key aus JSON extrahieren
        ObjectMapper mapper = new ObjectMapper();
        Map<String, String> keyData = mapper.readValue(new File(jsonPath), Map.class);
        String rawKey = keyData.get("customer_key");

        // 2. Hex-String zu long konvertieren (Entfernt C-Suffixe)
        this.licenseKey = Long.parseUnsignedLong(rawKey.replace("0x", "").replace("ULL", ""), 16);

        // 3. Native Instanz erzeugen
        this.brainHandle = BioAILibrary.INSTANCE.API_CreateBrain(this.licenseKey);
        
        if (this.brainHandle == null) {
            throw new RuntimeException("BioAI: Initialisierung der nativen Instanz fehlgeschlagen.");
        }
    }

    /**
     * Schaltet zwischen Training (0) und Produktion (1) um.
     */
    public void setMode(int mode) {
        BioAILibrary.INSTANCE.API_SetMode(brainHandle, mode);
    }

    /**
     * Verarbeitet Eingaben und liefert die optimale Aktion in O(1).
     */
    public long update(long[] inputs) {
        return BioAILibrary.INSTANCE.API_Update(brainHandle, inputs, inputs.length);
    }

    /**
     * Führt eine Kausalitäts-Simulation durch.
     */
    public long simulate(long[] inputs, int depth) {
        return BioAILibrary.INSTANCE.API_Simulate(brainHandle, inputs, inputs.length, depth);
    }

    /**
     * Passt das Verhalten über Reinforcement Learning an.
     */
    public void feedback(float reward, long action) {
        BioAILibrary.INSTANCE.API_Feedback(brainHandle, reward, action);
    }

    /**
     * Injiziert einen festen Reflex direkt in das LTM.
     */
    public void teach(long input, long action, float weight) {
        BioAILibrary.INSTANCE.API_Teach(brainHandle, input, action, weight);
    }

    /**
     * Liest ein gelerntes Gewicht unter De-Salting aus.
     */
    public float inspect(long input, long action) {
        return BioAILibrary.INSTANCE.API_Inspect(brainHandle, input, action);
    }

    /**
     * Erzeugt einen binären Snapshot des Wissenszustands.
     */
    public byte[] serialize() {
        IntByReference sizeRef = new IntByReference();
        Pointer buffer = BioAILibrary.INSTANCE.API_Serialize(brainHandle, sizeRef);
        
        if (buffer == null) return null;

        int size = sizeRef.getValue();
        byte[] result = buffer.getByteArray(0, size);
        
        // Native Freigabe des Puffers
        BioAILibrary.INSTANCE.API_FreeBuffer(buffer);
        return result;
    }

    /**
     * Schließt die Instanz und gibt nativen Speicher frei.
     */
    @Override
    public void close() {
        if (brainHandle != null) {
            BioAILibrary.INSTANCE.API_FreeBrain(brainHandle);
            brainHandle = null;
        }
    }
}