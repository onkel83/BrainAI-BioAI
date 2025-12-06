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
        public const ulong OBJECT = 0x1000000000000000;
        public const ulong ACTION = 0x2000000000000000;
        public const ulong TIME = 0x3000000000000000;
        public const ulong LOGIC = 0x4000000000000000;
        public const ulong SELF = 0x5000000000000000;

        public const ulong REFLEX = LOGIC | 0x0010000000000000;
        public const ulong NEED = SELF | 0x0010000000000000;
        public const ulong GOAL = SELF | 0x0020000000000000;
        public const ulong STATUS = SELF | 0x0030000000000000;

        private static Dictionary<ulong, string> _vocabulary = new Dictionary<ulong, string>();

        public static ulong CreateToken(string name, ulong cluster)
        {
            if (string.IsNullOrEmpty(name)) return 0;
            unchecked
            {
                ulong hash = 14695981039346656037;
                byte[] bytes = Encoding.UTF8.GetBytes(name);
                foreach (byte b in bytes) { hash ^= b; hash *= 1099511628211; }
                ulong finalToken = (hash & 0x00FFFFFFFFFFFFFF) | cluster;
                if (!_vocabulary.ContainsKey(finalToken)) _vocabulary.Add(finalToken, name);
                return finalToken;
            }
        }

        public static void DumpVocabulary(string path)
        {
            StringBuilder sb = new StringBuilder();
            foreach (var item in _vocabulary.OrderBy(k => k.Key))
                sb.AppendLine($"0x{item.Key:X16} | {item.Value}");
            File.WriteAllText(path, sb.ToString());
        }
    }

    // 2. WRAPPER (Standard, ohne Loader-Hacks)
    public class BioBrain : IDisposable
    {
        private IntPtr _handle;
        private bool _disposed = false;

        public BioBrain(ulong licenseKey)
        {
            try
            {
                _handle = NativeMethods.API_CreateBrain(licenseKey);
            }
            catch (DllNotFoundException)
            {
                throw new Exception("CRITICAL: libbioai.so not found in APK native library path.");
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
            Marshal.Copy(ptr, data, 0, size);
            NativeMethods.API_FreeBuffer(ptr);
            return data;
        }

        public static BioBrain Deserialize(byte[] data)
        {
            if (data == null) throw new ArgumentException("Data empty");
            IntPtr buf = Marshal.AllocHGlobal(data.Length);
            try
            {
                Marshal.Copy(data, 0, buf, data.Length);
                IntPtr newPtr = NativeMethods.API_Deserialize(buf, data.Length);
                if (newPtr == IntPtr.Zero) throw new Exception("Load failed");
                var brain = new BioBrain(0);
                NativeMethods.API_FreeBrain(brain._handle);
                brain._handle = newPtr;
                return brain;
            }
            finally { Marshal.FreeHGlobal(buf); }
        }

        public void LoadPlan(ulong[] steps, bool strict) { Check(); if (steps != null) NativeMethods.API_LoadPlan(_handle, steps, steps.Length, strict ? 1 : 0); }
        public void AbortPlan() { Check(); NativeMethods.API_AbortPlan(_handle); }
        public int GetPlanStep() { Check(); return NativeMethods.API_GetPlanStatus(_handle); }

        private void Check() { if (_disposed) throw new ObjectDisposedException("BioBrain"); }
        public void Dispose() { Dispose(true); GC.SuppressFinalize(this); }
        protected virtual void Dispose(bool disposing)
        {
            if (!_disposed && _handle != IntPtr.Zero) { NativeMethods.API_FreeBrain(_handle); _handle = IntPtr.Zero; _disposed = true; }
        }
        ~BioBrain() { Dispose(false); }
    }

    // 3. IMPORTS (Standard)
    internal static class NativeMethods
    {
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
}
