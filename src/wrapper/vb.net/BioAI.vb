Imports System
Imports System.IO
Imports System.Runtime.InteropServices
Imports System.Text.Json

Namespace BioAI.Runtime

    ''' <summary>
    ''' Struktur für das Einlesen der ISS-generierten key.json.
    ''' </summary>
    Public Class BioAIKey
        Public Property customer_key As String
    End Class

    ''' <summary>
    ''' Verwaltete Instanz des BioAI-Kerns für VB.NET.
    ''' </summary>
    Public Class BioBrainInstance
        Implements IDisposable

        ' Name der nativen Bibliothek (Tier-spezifisch)
        Private Const DllName As String = "BioAI_ULTRA.dll"

#Region "Native API Imports"

        <DllImport(DllName, CallingConvention:=CallingConvention.Cdecl)>
        Private Shared Function API_CreateBrain(ByVal key As UInt64) As IntPtr
        End Function

        <DllImport(DllName, CallingConvention:=CallingConvention.Cdecl)>
        Private Shared Sub API_FreeBrain(ByVal brainPtr As IntPtr)
        End Sub

        <DllImport(DllName, CallingConvention:=CallingConvention.Cdecl)>
        Private Shared Sub API_SetMode(ByVal brainPtr As IntPtr, ByVal mode As Integer)
        End Sub

        <DllImport(DllName, CallingConvention:=CallingConvention.Cdecl)>
        Private Shared Function API_Update(ByVal brainPtr As IntPtr, ByVal inputs As UInt64(), ByVal count As Integer) As UInt64
        End Function

        <DllImport(DllName, CallingConvention:=CallingConvention.Cdecl)>
        Private Shared Function API_Simulate(ByVal brainPtr As IntPtr, ByVal inputs As UInt64(), ByVal count As Integer, ByVal depth As Integer) As UInt64
        End Function

        <DllImport(DllName, CallingConvention:=CallingConvention.Cdecl)>
        Private Shared Sub API_Feedback(ByVal brainPtr As IntPtr, ByVal reward As Single, ByVal action As UInt64)
        End Sub

        <DllImport(DllName, CallingConvention:=CallingConvention.Cdecl)>
        Private Shared Sub API_Teach(ByVal brainPtr As IntPtr, ByVal input As UInt64, ByVal action As UInt64, ByVal weight As Single)
        End Sub

        <DllImport(DllName, CallingConvention:=CallingConvention.Cdecl)>
        Private Shared Function API_Inspect(ByVal brainPtr As IntPtr, ByVal input As UInt64, ByVal action As UInt64) As Single
        End Function

        <DllImport(DllName, CallingConvention:=CallingConvention.Cdecl)>
        Private Shared Function API_Serialize(ByVal brainPtr As IntPtr, ByRef outSize As Integer) As IntPtr
        End Function

        <DllImport(DllName, CallingConvention:=CallingConvention.Cdecl)>
        Private Shared Sub API_FreeBuffer(ByVal buffer As IntPtr)
        End Sub

#End Region

        Private _brainHandle As IntPtr
        Public Property LicenseKey As UInt64

        ''' <summary>
        ''' Erzeugt eine neue Instanz und lädt den Schlüssel aus der key.json.
        ''' </summary>
        Public Sub New(ByVal jsonPath As String)
            ' 1. Key aus JSON laden
            Dim jsonContent As String = File.ReadAllText(jsonPath)
            Dim keyObj = JsonSerializer.Deserialize(Of BioAIKey)(jsonContent)

            ' 2. Hex-Konvertierung (Entfernt C-Suffixe)
            Dim cleanKey As String = keyObj.customer_key.Replace("ULL", "").Replace("0x", "")
            Me.LicenseKey = Convert.ToUInt64(cleanKey, 16)

            ' 3. Kern initialisieren
            _brainHandle = API_CreateBrain(Me.LicenseKey)
            If _brainHandle = IntPtr.Zero Then
                Throw New Exception("BioAI: Initialisierung der nativen Instanz fehlgeschlagen.")
            End If
        End Sub

        Public Sub SetMode(ByVal mode As Integer)
            API_SetMode(_brainHandle, mode)
        End Sub

        Public Function Update(ByVal inputs As UInt64()) As UInt64
            Return API_Update(_brainHandle, inputs, inputs.Length)
        End Function

        Public Function Simulate(ByVal inputs As UInt64(), ByVal depth As Integer) As UInt64
            Return API_Simulate(_brainHandle, inputs, inputs.Length, depth)
        End Function

        Public Sub Feedback(ByVal reward As Single, ByVal action As UInt64)
            API_Feedback(_brainHandle, reward, action)
        End Sub

        Public Sub Teach(ByVal input As UInt64, ByVal action As UInt64, ByVal weight As Single)
            API_Teach(_brainHandle, input, action, weight)
        End Sub

        Public Function Inspect(ByVal input As UInt64, ByVal action As UInt64) As Single
            Return API_Inspect(_brainHandle, input, action)
        End Function

        Public Function Serialize() As Byte()
            Dim size As Integer = 0
            Dim buffer As IntPtr = API_Serialize(_brainHandle, size)
            If buffer = IntPtr.Zero Then Return Nothing

            Dim managedArray(size - 1) As Byte
            Marshal.Copy(buffer, managedArray, 0, size)
            API_FreeBuffer(buffer) ' Speicher im C-Kern freigeben
            Return managedArray
        End Function

        Public Sub Dispose() Implements IDisposable.Dispose
            If _brainHandle <> IntPtr.Zero Then
                API_FreeBrain(_brainHandle)
                _brainHandle = IntPtr.Zero
            End If
        End Sub
    End Class
End Namespace