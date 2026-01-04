using System;
using System.IO;
using System.Runtime.InteropServices;
using System.Text;

namespace ProKey.SDK
{
    /// <summary>
    /// Exception, die geworfen wird, wenn die Hardware-Entropie versagt.
    /// </summary>
    public class ProKeyHardwareException : Exception
    {
        public ProKeyHardwareException() : base("ProKey Hardware RNG Failure: Could not generate entropy.") { }
    }

    /// <summary>
    /// Der Managed Wrapper für die ProKey Native Library.
    /// </summary>
    public static class Generator
    {
        // ---------------------------------------------------------
        // NATIVE INTERFACE (P/Invoke)
        // ---------------------------------------------------------
        
        // Der Name der DLL (ohne .dll oder .so Erweiterung, .NET findet das selbst)
        private const string DllName = "prokey";

        [DllImport(DllName, CallingConvention = CallingConvention.Cdecl, EntryPoint = "ProKey_GetRawKey")]
        private static extern ulong Native_GetRawKey();

        // ---------------------------------------------------------
        // PUBLIC API
        // ---------------------------------------------------------

        /// <summary>
        /// Holt den rohen 64-Bit Key direkt von der Hardware.
        /// </summary>
        /// <exception cref="ProKeyHardwareException">Wenn RNG fehlschlägt (Rückgabe 0).</exception>
        public static ulong Next()
        {
            try
            {
                ulong key = Native_GetRawKey();
                
                // Hardware-Fehler Check (analog zum C Code)
                if (key == 0)
                {
                    throw new ProKeyHardwareException();
                }
                
                return key;
            }
            catch (DllNotFoundException)
            {
                throw new FileNotFoundException($"Die Native Library '{DllName}' wurde nicht gefunden. Bitte kopieren Sie prokey.dll (Windows) oder libprokey.so (Linux) in das Ausführungsverzeichnis.");
            }
        }

        /// <summary>
        /// Generiert den Key und gibt ihn als formatierten Hex-String zurück.
        /// Format: "KEY:0x..."
        /// </summary>
        public static string NextString()
        {
            ulong key = Next();
            return $"KEY:0x{key:X16}";
        }

        /// <summary>
        /// Schreibt den Key direkt in einen TextWriter (Console, FileStream, etc.).
        /// </summary>
        public static void ToStream(TextWriter writer)
        {
            if (writer == null) throw new ArgumentNullException(nameof(writer));
            writer.WriteLine(NextString());
        }

        /// <summary>
        /// Komfort-Funktion: Hängt einen neuen Key an eine Datei an.
        /// </summary>
        public static void AppendToFile(string filePath)
        {
            using (StreamWriter sw = File.AppendText(filePath))
            {
                ToStream(sw);
            }
        }
    }
}
