Imports System
Imports System.Collections.Generic
Imports System.IO
Imports System.Runtime.InteropServices
Imports System.Text
Imports System.Linq

Namespace BrainAI.BioAI

    ' =============================================================
    ' 1. KONSTANTEN & ENUMS
    ' =============================================================

    Public Enum BioMode As Integer
        Training = 0
        Production = 1
    End Enum

    Public NotInheritable Class BioClusters
        ' Cluster Definitions (High Byte)
        Public Const OBJECT_C As UInt64 = &H1000000000000000UL
        Public Const ACTION_C As UInt64 = &H2000000000000000UL
        Public Const TIME_C   As UInt64 = &H3000000000000000UL
        Public Const LOGIC_C  As UInt64 = &H4000000000000000UL
        Public Const SELF_C   As UInt64 = &H5000000000000000UL

        ' Sub-Clusters
        Public Const REFLEX   As UInt64 = LOGIC_C Or &H1000000000000000UL
        Public Const NEED     As UInt64 = SELF_C  Or &H1000000000000000UL
        Public Const GOAL     As UInt64 = SELF_C  Or &H2000000000000000UL
        Public Const STATUS   As UInt64 = SELF_C  Or &H3000000000000000UL

        Private Shared _vocabulary As New Dictionary(Of UInt64, String)()

        ' Deterministischer Token Generator (FNV-1a)
        ' VB.NET wirft bei Überlauf Fehler, daher nutzen wir einen Trick oder BigInteger,
        ' aber hier ist die optimierte Version.
        Public Shared Function CreateToken(name As String, cluster As UInt64) As UInt64
            If String.IsNullOrEmpty(name) Then Return 0

            ' FNV-1a Constants
            Dim hash As UInt64 = 14695981039346656037UL
            Dim prime As UInt64 = 1099511628211UL
            
            Dim bytes As Byte() = Encoding.UTF8.GetBytes(name)

            ' Unchecked Block gibt es in VB nicht direkt so einfach, 
            ' aber wir nutzen den mathematischen Modulo-Trick für Sicherheit
            For Each b As Byte In bytes
                hash = hash Xor b
                ' Simulation von Unchecked Multiplication durch Ignorieren des Overflows
                ' In VB Projekteinstellungen sollte "Integer Overflow Checks" AUS sein,
                ' aber um sicher zu gehen:
                Try
                    hash = hash * prime
                Catch ex As OverflowException
                    ' Fallback für restriktive VB-Umgebungen:
                    ' Wir schneiden einfach ab (Dirty, aber für Hash okay in VB)
                    hash = (hash And &HFFFFFFFFFFFFFFFFUL) * prime 
                End Try
            Next

            ' Maskieren und Cluster setzen
            Dim finalToken As UInt64 = (hash And &HFFFFFFFFFFFFFFUL) Or cluster

            If Not _vocabulary.ContainsKey(finalToken) Then
                _vocabulary.Add(finalToken, name)
            End If

            Return finalToken
        End Function

        Public Shared Sub DumpVocabulary(path As String)
            Dim sb As New StringBuilder()
            sb.AppendLine("BioAI Token Export (VB.NET)")
            sb.AppendLine("---------------------------")
            
            For Each kvp In _vocabulary.OrderBy(Function(x) x.Key)
                sb.AppendLine($"0x{kvp.Key:X16} | {kvp.Value}")
            Next
            
            File.WriteAllText(path, sb.ToString())
        End Sub
    End Class

    ' =============================================================
    ' 2. BIOAI WRAPPER KLASSE
    ' =============================================================

    Public Class BioBrain
        Implements IDisposable

        Private _ptr As IntPtr
        Private _disposed As Boolean = False

        Public Sub New(key As UInt64)
            _ptr = NativeMethods.API_CreateBrain(key)
            If _ptr = IntPtr.Zero Then Throw New OutOfMemoryException("BioAI Init Failed.")
        End Sub

        ' --- API METHODEN ---

        Public Sub SetMode(mode As BioMode)
            Check()
            NativeMethods.API_SetMode(_ptr, CInt(mode))
        End Sub

        Public Function Think(inputs As UInt64()) As UInt64
            Check()
            If inputs Is Nothing OrElse inputs.Length = 0 Then Return 0
            Return NativeMethods.API_Update(_ptr, inputs, inputs.Length)
        End Function

        Public Function Simulate(inputs As UInt64(), depth As Integer) As UInt64
            Check()
            If inputs Is Nothing OrElse inputs.Length = 0 Then Return 0
            Return NativeMethods.API_Simulate(_ptr, inputs, inputs.Length, depth)
        End Function

        Public Sub Learn(reward As Single, action As UInt64)
            Check()
            NativeMethods.API_Feedback(_ptr, reward, action)
        End Sub

        Public Sub ForceInstinct(input As UInt64, action As UInt64, weight As Single)
            Check()
            NativeMethods.API_Teach(_ptr, input, action, weight)
        End Sub

        Public Function Inspect(input As UInt64, action As UInt64) As Single
            Check()
            Return NativeMethods.API_Inspect(_ptr, input, action)
        End Function

        ' --- SEQUENCER (PLANER) ---

        Public Sub LoadPlan(steps As UInt64(), strict As Boolean)
            Check()
            If steps IsNot Nothing Then
                NativeMethods.API_LoadPlan(_ptr, steps, steps.Length, If(strict, 1, 0))
            End If
        End Sub

        Public Sub AbortPlan()
            Check()
            NativeMethods.API_AbortPlan(_ptr)
        End Sub

        Public Function GetPlanStep() As Integer
            Check()
            Return NativeMethods.API_GetPlanStatus(_ptr)
        End Function

        ' --- SYSTEM ---

        Public Function Serialize() As Byte()
            Check()
            Dim size As Integer = 0
            Dim bufPtr As IntPtr = NativeMethods.API_Serialize(_ptr, size)
            
            If bufPtr = IntPtr.Zero OrElse size = 0 Then Return Nothing

            Try
                Dim data(size - 1) As Byte
                Marshal.Copy(bufPtr, data, 0, size)
                Return data
            Finally
                NativeMethods.API_FreeBuffer(bufPtr)
            End Try
        End Function

        Public Shared Function Deserialize(data As Byte()) As BioBrain
            If data Is Nothing OrElse data.Length = 0 Then Throw New ArgumentException("Data empty")
            
            Dim bufPtr As IntPtr = Marshal.AllocHGlobal(data.Length)
            Try
                Marshal.Copy(data, 0, bufPtr, data.Length)
                Dim newPtr As IntPtr = NativeMethods.API_Deserialize(bufPtr, data.Length)
                
                If newPtr = IntPtr.Zero Then Throw New Exception("Load failed")
                
                ' Wrapper erstellen und Pointer unterschieben
                Dim brain As New BioBrain(0)
                NativeMethods.API_FreeBrain(brain._ptr) ' Den leeren Dummy löschen
                brain._ptr = newPtr
                Return brain
            Finally
                Marshal.FreeHGlobal(bufPtr)
            End Try
        End Function

        ' --- CLEANUP ---

        Private Sub Check()
            If _disposed Then Throw New ObjectDisposedException("BioBrain")
        End Sub

        Protected Overridable Sub Dispose(disposing As Boolean)
            If Not _disposed Then
                If _ptr <> IntPtr.Zero Then
                    NativeMethods.API_FreeBrain(_ptr)
                    _ptr = IntPtr.Zero
                End If
                _disposed = True
            End If
        End Sub

        Public Sub Dispose() Implements IDisposable.Dispose
            Dispose(True)
            GC.SuppressFinalize(Me)
        End Sub

        Protected Overrides Sub Finalize()
            Dispose(False)
        End Sub

    End Class

    ' =============================================================
    ' 3. NATIVE METHODS (P/Invoke)
    ' =============================================================
    
    Friend Class NativeMethods
        ' Automatische Wahl: Windows sucht bioai.dll, Linux libbioai.so
        Const DLL_NAME As String = "bioai"

        <DllImport(DLL_NAME, CallingConvention:=CallingConvention.Cdecl)>
        Public Shared Function API_CreateBrain(key As UInt64) As IntPtr
        End Function

        <DllImport(DLL_NAME, CallingConvention:=CallingConvention.Cdecl)>
        Public Shared Sub API_FreeBrain(ptr As IntPtr)
        End Sub

        <DllImport(DLL_NAME, CallingConvention:=CallingConvention.Cdecl)>
        Public Shared Sub API_SetMode(ptr As IntPtr, mode As Integer)
        End Sub

        <DllImport(DLL_NAME, CallingConvention:=CallingConvention.Cdecl)>
        Public Shared Function API_Update(ptr As IntPtr, inputs() As UInt64, count As Integer) As UInt64
        End Function

        <DllImport(DLL_NAME, CallingConvention:=CallingConvention.Cdecl)>
        Public Shared Function API_Simulate(ptr As IntPtr, inputs() As UInt64, count As Integer, depth As Integer) As UInt64
        End Function

        <DllImport(DLL_NAME, CallingConvention:=CallingConvention.Cdecl)>
        Public Shared Sub API_Feedback(ptr As IntPtr, reward As Single, action As UInt64)
        End Sub

        <DllImport(DLL_NAME, CallingConvention:=CallingConvention.Cdecl)>
        Public Shared Sub API_Teach(ptr As IntPtr, input As UInt64, action As UInt64, weight As Single)
        End Sub

        <DllImport(DLL_NAME, CallingConvention:=CallingConvention.Cdecl)>
        Public Shared Function API_Inspect(ptr As IntPtr, input As UInt64, action As UInt64) As Single
        End Function

        <DllImport(DLL_NAME, CallingConvention:=CallingConvention.Cdecl)>
        Public Shared Function API_Serialize(ptr As IntPtr, ByRef size As Integer) As IntPtr
        End Function

        <DllImport(DLL_NAME, CallingConvention:=CallingConvention.Cdecl)>
        Public Shared Function API_Deserialize(data As IntPtr, size As Integer) As IntPtr
        End Function

        <DllImport(DLL_NAME, CallingConvention:=CallingConvention.Cdecl)>
        Public Shared Sub API_FreeBuffer(ptr As IntPtr)
        End Sub

        ' Planer
        <DllImport(DLL_NAME, CallingConvention:=CallingConvention.Cdecl)>
        Public Shared Sub API_LoadPlan(ptr As IntPtr, steps() As UInt64, count As Integer, strict As Integer)
        End Sub

        <DllImport(DLL_NAME, CallingConvention:=CallingConvention.Cdecl)>
        Public Shared Sub API_AbortPlan(ptr As IntPtr)
        End Sub

        <DllImport(DLL_NAME, CallingConvention:=CallingConvention.Cdecl)>
        Public Shared Function API_GetPlanStatus(ptr As IntPtr) As Integer
        End Function
    End Class

End Namespace
