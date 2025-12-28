using System;
using System.IO;
using System.Runtime.InteropServices;
using System.Text.Json;

namespace BioAI.Runtime
{
    /// <summary>
    /// Repräsentiert die Struktur der Lizenzdatei (key.json).
    /// Diese Datei wird während des ISS-Build-Vorgangs generiert.
    /// </summary>
    public class BioAIKey
    {
        public string customer_key { get; set; }
    }

    /// <summary>
    /// Verwaltete Instanz eines BioAI-Rechenkerns. 
    /// Implementiert IDisposable für ein sicheres Ressourcenmanagement.
    /// </summary>
    public class BioBrainInstance : IDisposable
    {
        // Name der nativen Bibliothek. Muss dem gewählten Hardware-Tier entsprechen.
        private const string DllName = "BioAI_ULTRA.dll";

        #region Native Methoden (C99 Interface)
        // Diese Sektion bildet die Brücke zum hochperformanten C-Kern.

        [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
        private static extern IntPtr API_CreateBrain(ulong key);

        [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
        private static extern void API_FreeBrain(IntPtr brainPtr);

        [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
        private static extern void API_SetMode(IntPtr brainPtr, int mode);

        [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
        private static extern ulong API_Update(IntPtr brainPtr, ulong[] inputs, int count);

        [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
        private static extern ulong API_Simulate(IntPtr brainPtr, ulong[] inputs, int count, int depth);

        [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
        private static extern void API_Feedback(IntPtr brainPtr, float reward, ulong action);

        [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
        private static extern void API_Teach(IntPtr brainPtr, ulong input, ulong action, float weight);

        [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
        private static extern float API_Inspect(IntPtr brainPtr, ulong input, ulong action);

        [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
        private static extern IntPtr API_Serialize(IntPtr brainPtr, out int outSize);

        [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
        private static extern IntPtr API_Deserialize(byte[] data, int size);

        [DllImport(DllName, CallingConvention = CallingConvention.Cdecl)]
        private static extern void API_FreeBuffer(IntPtr buffer);

        #endregion

        // Unverwalteter Zeiger auf die interne BioBrain-Struktur (Opaque Handle).
        private IntPtr _brainHandle;

        /// <summary>
        /// Der mathematische Schlüssel, der für das De-Salting der Gewichte benötigt wird.
        /// </summary>
        public ulong LicenseKey { get; private set; }

        /// <summary>
        /// Initialisiert eine neue Instanz des BioAI-Kerns.
        /// </summary>
        /// <param name="jsonPath">Pfad zur generierten key.json Datei.</param>
        public BioBrainInstance(string jsonPath)
        {
            // 1. Laden und Parsen der Lizenzinformationen
            string jsonContent = File.ReadAllText(jsonPath);
            var keyObj = JsonSerializer.Deserialize<BioAIKey>(jsonContent);

            // 2. Konvertierung des Hex-Strings in einen 64-Bit Ganzzahlwert.
            // Entfernt C-spezifische Suffixe (ULL), falls vorhanden.
            this.LicenseKey = Convert.ToUInt64(keyObj.customer_key.Replace("ULL", ""), 16);

            // 3. Instanziierung im unverwalteten Speicher des Kerns.
            _brainHandle = API_CreateBrain(this.LicenseKey);

            if (_brainHandle == IntPtr.Zero)
                throw new Exception("BioAI: Initialisierung der Brain-Instanz fehlgeschlagen.");
        }

        /// <summary>
        /// Schaltet zwischen Training (0) und Produktion (1) um. 
        /// Modus 1 deaktiviert Speicher-Allokationen für maximale Stabilität.
        /// </summary>
        public void SetMode(int mode) => API_SetMode(_brainHandle, mode);

        /// <summary>
        /// Verarbeitet eine Liste von Wahrnehmungen (Inputs) in konstanter Zeit O(1).
        /// </summary>
        /// <returns>Die TokenID der optimalen Aktion.</returns>
        public ulong Update(ulong[] inputs) => API_Update(_brainHandle, inputs, inputs.Length);

        /// <summary>
        /// Führt eine interne Simulation möglicher Zukünfte durch (Imagination).
        /// </summary>
        public ulong Simulate(ulong[] inputs, int depth) => API_Simulate(_brainHandle, inputs, inputs.Length, depth);

        /// <summary>
        /// Verstärkt oder schwächt Verhaltensweisen basierend auf Erfolg oder Misserfolg.
        /// </summary>
        public void Feedback(float reward, ulong action) => API_Feedback(_brainHandle, reward, action);

        /// <summary>
        /// Injiziert eine harte Regel (Reflex) direkt in das Langzeitgedächtnis.
        /// </summary>
        public void Teach(ulong input, ulong action, float weight) => API_Teach(_brainHandle, input, action, weight);

        /// <summary>
        /// Erlaubt das Auslesen gelernten Wissens unter Berücksichtigung des Lizenzschlüssel-Salts.
        /// </summary>
        public float Inspect(ulong input, ulong action) => API_Inspect(_brainHandle, input, action);

        /// <summary>
        /// Erzeugt einen vollständigen binären Snapshot des aktuellen Wissenszustands.
        /// </summary>
        /// <returns>Byte-Array zur permanenten Speicherung.</returns>
        public byte[] Serialize()
        {
            // Puffer im unverwalteten C-Speicher anfordern
            IntPtr buffer = API_Serialize(_brainHandle, out int size);
            if (buffer == IntPtr.Zero) return null;

            // Daten in den verwalteten .NET-Speicher kopieren
            byte[] managedArray = new byte[size];
            Marshal.Copy(buffer, managedArray, 0, size);

            // WICHTIG: Den nativen Puffer sofort wieder freigeben, um Memory Leaks zu vermeiden.
            API_FreeBuffer(buffer);

            return managedArray;
        }

        /// <summary>
        /// Gibt alle nativen Ressourcen frei und zerstört die Gehirn-Instanz sicher.
        /// </summary>
        public void Dispose()
        {
            if (_brainHandle != IntPtr.Zero)
            {
                API_FreeBrain(_brainHandle);
                _brainHandle = IntPtr.Zero;
            }
        }
    }
}