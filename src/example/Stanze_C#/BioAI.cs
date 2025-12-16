using System;
using System.Collections.Generic;
using System.Linq;
using System.Runtime.InteropServices;
using System.Text;
using System.IO;

namespace BrainAI.BioAI
{
    // 1. KONSTANTEN
    public enum BioMode : int { Training = 0, Production = 1 }

    public static class BioClusters
    {
        // Konstanten zur Klassifizierung von Tokens
        public const ulong OBJECT = 0x1000000000000000;
        public const ulong ACTION = 0x2000000000000000;
        public const ulong TIME = 0x3000000000000000;
        public const ulong LOGIC = 0x4000000000000000;
        public const ulong SELF = 0x5000000000000000;

        // Unterkategorien
        public const ulong REFLEX = LOGIC | 0x0010000000000000;
        public const ulong NEED = SELF | 0x0010000000000000;
        public const ulong GOAL = SELF | 0x0020000000000000;
        public const ulong STATUS = SELF | 0x0030000000000000;

        private static Dictionary<ulong, string> _vocabulary = new Dictionary<ulong, string>();

        /// <summary>
        /// Erstellt ein eindeutiges 64-Bit-Token aus einem Namen und weist ihm einen Cluster zu.
        /// </summary>
        public static ulong CreateToken(string name, ulong cluster)
        {
            if (string.IsNullOrEmpty(name)) return 0;
            unchecked
            {
                // FNV-1a Hash-Algorithmus (64-Bit)
                ulong hash = 14695981039346656037;
                byte[] bytes = Encoding.UTF8.GetBytes(name);
                foreach (byte b in bytes) { hash ^= b; hash *= 1099511628211; }
                
                // Setzt die oberen 8 Bits auf den Cluster-Wert
                ulong finalToken = (hash & 0x00FFFFFFFFFFFFFF) | cluster;
                
                if (!_vocabulary.ContainsKey(finalToken)) _vocabulary.Add(finalToken, name);
                return finalToken;
            }
        }

        /// <summary>
        /// Speichert das Vokabular in einer Datei.
        /// </summary>
        public static void DumpVocabulary(string path)
        {
            StringBuilder sb = new StringBuilder();
            foreach (var item in _vocabulary.OrderBy(k => k.Key))
                sb.AppendLine($"0x{item.Key:X16} | {item.Value}");
            File.WriteAllText(path, sb.ToString());
        }
    }

    // 2. WRAPPER (Standard)
    /// <summary>
    /// Wrapper-Klasse für die BioAI-Bibliothek (libbioai.dll).
    /// </summary>
    public class BioBrain : IDisposable
    {
        private IntPtr _handle;
        private bool _disposed = false;

        public BioBrain(ulong licenseKey)
        {
            try
            {
                // Aufruf der nativen Funktion zum Erstellen einer Brain-Instanz
                _handle = NativeMethods.API_CreateBrain(licenseKey);
            }
            catch (DllNotFoundException)
            {
                // Auf Windows wird die Datei libbioai.dll erwartet
                throw new Exception("CRITICAL: libbioai.dll not found in application path.");
            }
            if (_handle == IntPtr.Zero) throw new OutOfMemoryException("Init Failed");
        }

        public void SetMode(BioMode mode) { Check(); NativeMethods.API_SetMode(_handle, (int)mode); }
        public ulong Think(params ulong[] inputs) { Check(); return NativeMethods.API_Update(_handle, inputs, inputs.Length); }
        public ulong Simulate(ulong[] inputs, int depth) { Check(); return NativeMethods.API_Simulate(_handle, inputs, inputs.Length, depth); }
        public void Learn(float reward, ulong action) { Check(); NativeMethods.API_Feedback(_handle, reward, action); }
        public void ForceInstinct(ulong input, ulong action, float weight) { Check(); NativeMethods.API_Teach(_handle, input, action, weight); }
        public float Inspect(ulong input, ulong action) { Check(); return NativeMethods.API_Inspect(_handle, input, action); }

        public byte[] Serialize()
        {
            Check();
            int size;
            IntPtr ptr = NativeMethods.API_Serialize(_handle, out size);
            if (ptr == IntPtr.Zero) return null;
            byte[] data = new byte[size];
            
            // Kopieren der Daten vom nativen Speicher in ein C#-Byte-Array
            Marshal.Copy(ptr, data, 0, size);
            NativeMethods.API_FreeBuffer(ptr); // Freigabe des nativen Buffers
            return data;
        }

        public static BioBrain Deserialize(byte[] data)
        {
            if (data == null || data.Length == 0) throw new ArgumentException("Data empty or null");
            
            // Allokieren von unmanaged memory
            IntPtr buf = Marshal.AllocHGlobal(data.Length);
            try
            {
                // Kopieren des C#-Byte-Arrays in den unmanaged Buffer
                Marshal.Copy(data, 0, buf, data.Length);
                
                // Aufruf der nativen Deserialisierungsfunktion
                IntPtr newPtr = NativeMethods.API_Deserialize(buf, data.Length);
                if (newPtr == IntPtr.Zero) throw new Exception("Load failed");
                
                // Erstellen eines neuen BioBrain-Objekts und Zuweisen des Handles
                var brain = new BioBrain(0); // Dummy-Konstruktoraufruf
                NativeMethods.API_FreeBrain(brain._handle); // Freigabe des Dummy-Handles
                brain._handle = newPtr; // Zuweisung des deserialisierten Handles
                return brain;
            }
            finally { 
                // Freigabe des zuvor allokierten unmanaged Buffers
                Marshal.FreeHGlobal(buf); 
            }
        }

        public void LoadPlan(ulong[] steps, bool strict) { Check(); if (steps != null) NativeMethods.API_LoadPlan(_handle, steps, steps.Length, strict ? 1 : 0); }
        public void AbortPlan() { Check(); NativeMethods.API_AbortPlan(_handle); }
        public int GetPlanStep() { Check(); return NativeMethods.API_GetPlanStatus(_handle); }

        private void Check() { if (_disposed) throw new ObjectDisposedException("BioBrain"); }
        
        // Implementierung von IDisposable
        public void Dispose() { Dispose(true); GC.SuppressFinalize(this); }
        protected virtual void Dispose(bool disposing)
        {
            if (!_disposed && _handle != IntPtr.Zero) 
            { 
                NativeMethods.API_FreeBrain(_handle); 
                _handle = IntPtr.Zero; 
                _disposed = true; 
            }
        }
        ~BioBrain() { Dispose(false); }
    }

    // 3. IMPORTS (Standard für Windows Konsole)
    internal static class NativeMethods
    {
        // Anpassen des Bibliotheksnamens von "bioai" auf "libbioai.dll" für Windows
        // In .NET Core/Framework sucht der DllImport-Mechanismus auf Windows nach bioai.dll
        const string LibName = "bioai"; 

        [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
        internal static extern IntPtr API_CreateBrain(ulong key);
        [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
        internal static extern void API_FreeBrain(IntPtr ptr);
        [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
        internal static extern void API_SetMode(IntPtr ptr, int mode);
        [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
        internal static extern ulong API_Update(IntPtr ptr, [In] ulong[] inputs, int count);
        [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
        internal static extern ulong API_Simulate(IntPtr ptr, [In] ulong[] inputs, int count, int depth);
        [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
        internal static extern void API_Feedback(IntPtr ptr, float reward, ulong action);
        [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
        internal static extern void API_Teach(IntPtr ptr, ulong input, ulong action, float weight);
        [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
        internal static extern float API_Inspect(IntPtr ptr, ulong input, ulong action);
        [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
        internal static extern IntPtr API_Serialize(IntPtr ptr, out int outSize);
        [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
        internal static extern IntPtr API_Deserialize(IntPtr data, int size);
        [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
        internal static extern void API_FreeBuffer(IntPtr buffer);
        [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
        internal static extern void API_LoadPlan(IntPtr ptr, [In] ulong[] steps, int count, int strict);
        [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
        internal static extern void API_AbortPlan(IntPtr ptr);
        [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
        internal static extern int API_GetPlanStatus(IntPtr ptr);
    }
    
    // 4. BEISPIEL FÜR KONSOLENANWENDUNG (Für die Demonstration)
    // Sie können diesen Codeblock in Ihre Program.cs einfügen, um zu testen.
    /*
    public class Program
    {
        public static void Main(string[] args)
        {
            // Fügen Sie libbioai.dll zum Ausführungsverzeichnis der Anwendung hinzu!

            Console.WriteLine("BioAI Wrapper Test (Windows Console)");
            
            // Beispiel-Tokens erstellen
            ulong tokenWalk = BioClusters.CreateToken("WALK", BioClusters.ACTION);
            ulong tokenHungry = BioClusters.CreateToken("HUNGRY", BioClusters.NEED);
            ulong tokenFood = BioClusters.CreateToken("FOOD", BioClusters.OBJECT);

            Console.WriteLine($"Token WALK: 0x{tokenWalk:X16}");
            Console.WriteLine($"Token HUNGRY: 0x{tokenHungry:X16}");
            Console.WriteLine("---");

            try
            {
                // Erstellen einer BioBrain-Instanz (Lizenzschlüssel 0x12345678)
                using (var brain = new BioBrain(0x12345678))
                {
                    Console.WriteLine("BioBrain successfully initialized.");
                    
                    // Beispiel-Instinkt lehren: Wenn hungrig, gehe
                    brain.ForceInstinct(tokenHungry, tokenWalk, 1.0f);
                    Console.WriteLine("Instinct 'HUNGRY -> WALK' taught.");

                    // 'Denken' mit dem Input 'HUNGRY'
                    ulong action = brain.Think(tokenHungry);

                    Console.WriteLine("---");
                    if (action == tokenWalk)
                    {
                        Console.WriteLine("Brain acted: WALK (Success)");
                    }
                    else
                    {
                        Console.WriteLine($"Brain acted: 0x{action:X16} (Expected WALK)");
                    }

                    // Vokabular speichern
                    BioClusters.DumpVocabulary("vocabulary.txt");
                    Console.WriteLine("Vocabulary saved to vocabulary.txt");
                }
            }
            catch (Exception ex)
            {
                Console.WriteLine($"Error: {ex.Message}");
            }
            
            Console.WriteLine("Press any key to exit...");
            Console.ReadKey();
        }
    }
    */
}
