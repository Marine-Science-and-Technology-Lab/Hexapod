Imports System
Imports System.Text 'StringBuilder
Imports System.Runtime.InteropServices 'DLL import
Imports System.IO 'File.Exists

Imports UB = System.Byte
Imports UW = System.UInt16
Imports SW = System.Int16
Imports SL = System.Int32
Imports UL = System.UInt32

#If PLATFORM = "x86" Then
Imports GReturn = System.Int32
Imports GCon = System.IntPtr
Imports GSize = System.UInt32
Imports GOption = System.Int32
Imports GCStringOut = System.Text.StringBuilder
Imports GCStringIn = System.String
Imports GBufOut = System.Text.StringBuilder
Imports GBufIn = System.String
Imports GStatus = System.Byte
' IMPORTANT! Be sure that the paths below are correct
Public Module LibraryPath
    Public Const GclibDllPath_ As String = "C:\Program Files (x86)\Galil\gclib\dll\x86\gclib.dll"
    Public Const GcliboDllPath_ As String = "C:\Program Files (x86)\Galil\gclib\dll\x86\gclibo.dll"
End Module

#ElseIf PLATFORM = "x64" Then
Imports GReturn = System.Int32
Imports GCon = System.IntPtr
Imports GSize = System.UInt32
Imports GOption = System.Int32
Imports GCStringOut = System.Text.StringBuilder
Imports GCStringIn = System.String
Imports GBufOut = System.Text.StringBuilder
Imports GBufIn = System.String
Imports GStatus = System.Byte
' IMPORTANT! Be sure that the paths below are correct
Public Module LibraryPath
    Public Const GclibDllPath_ As String = "C:\Program Files (x86)\Galil\gclib\dll\x64\gclib.dll"
    Public Const GcliboDllPath_ As String = "C:\Program Files (x86)\Galil\gclib\dll\x64\gclibo.dll"
End Module
#End If

''' <summary>
''' Provides a class that binds to gclib's unmanaged dll. Wraps each call and provides a more user-friendly interface for use in Visual Basic.
''' </summary>
''' <remarks>
''' The Gclib class assumes the default installation of gclib, "C:\Program Files (x86)\Galil\gclib\". 
''' If the dlls are elsewhere, change the path strings GclibDllPath_, and GcliboDllPath_.
''' </remarks>
Public Class Gclib

#Region "VB wrappers of gclib C calls"

#Region "Private properties"
    Private Const BufferSize_ As Integer = 500000 'size of "char *" buffer. Big enough to fit entire 4000 program via UL/LS, or 24000 elements of array data.
    Private Buffer_ As New System.Text.StringBuilder(BufferSize_) 'used to pass a "char *" to gclib.
    Private ByteArray_(512) As Byte 'byte array to hold data record and response to GRead
    Private ConnectionHandle_ As GCon 'keep track of the gclib connection handle.
    Private ConnectionStatus_ As Boolean 'keep track of the status of gclib's connection.
#End Region

    ''' <summary>
    ''' Constructor of the gclib wrapper class.
    ''' </summary>
    ''' <remarks>Checks to ensure gclib dlls are in the correct location.</remarks>
    ''' <exception cref="System.Exception">Will throw an exception if either dll isn't found.</exception>
    Public Sub New()
        If Not File.Exists(GclibDllPath_) Then
            Throw New System.Exception("Could not find gclib dll at " & GclibDllPath_)
        End If

        If Not File.Exists(GcliboDllPath_) Then
            Throw New System.Exception("Could not find gclibo dll at " & GcliboDllPath_)
        End If
    End Sub

    ''' <summary>
    ''' Return a string array of available connection addresses.
    ''' </summary>
    ''' <returns>String array containing all available Galil Ethernet controllers, PCI controllers, and COM ports.</returns>
    ''' <remarks>Wrapper around gclib GAddresses(),
    ''' http://www.galil.com/sw/pub/all/doc/gclib/html/gclibo_8h.html#a6a6114683ed5749519b64f19512c24d6
    ''' An empty array is returned on error.</remarks>
    Public Function GAddresses() As String()
        Dim rc As GReturn = DllGAddresses(Buffer_, BufferSize_)
        If rc = G_NO_ERROR Then
            Return Buffer_.ToString().Split({vbCr, vbLf}, System.StringSplitOptions.RemoveEmptyEntries)
        Else
            Return New String() {}
        End If
    End Function

    ''' <summary>
    ''' Downloads array data to a pre-dimensioned array in the controller's array table. 
    ''' </summary>
    ''' <param name="array_name">String containing the name of the array to download. Must match the array name used in DM.</param>
    ''' <param name="data">A list of doubles, to be downloaded.</param>
    ''' <param name="first">The first element of the array for sub-array downloads.</param>
    ''' <param name="last">The last element of the array for sub-array downloads.</param>
    ''' <remarks>Wrapper around gclib GArrayDownload(),
    ''' http://www.galil.com/sw/pub/all/doc/gclib/html/gclib_8h.html#a6ea5ae6d167675e4c27ccfaf2f240f8a 
    ''' The array must already exist on the controller, see DM and LA.</remarks>
    ''' <exception cref="System.Exception">Will throw an exception if anything other than G_NO_ERROR is received from gclib.</exception>
    Public Sub GArrayDownload(array_name As String, ByRef data As List(Of Double), Optional first As Int16 = -1, Optional last As Int16 = -1)
        Dim ArrayData As New System.Text.StringBuilder(BufferSize_) 'for converting to ASCII
        Dim len As Integer = data.Count()
        For i As Integer = 0 To len - 1
            ArrayData.Append(data(i).ToString("F4")) 'format to fixed point
            If i < len - 1 Then
                ArrayData.Append(",") 'delimiter
            End If
        Next
        Dim rc As GReturn = DllGArrayDownload(ConnectionHandle_, array_name, first, last, ArrayData.ToString())
        If Not rc = G_NO_ERROR Then
            Throw New System.Exception(GError(rc))
        End If
    End Sub

    ''' <summary>
    ''' Allows downloading of a program array file to the controller.
    ''' </summary>
    ''' <param name="Path">The full filepath of the array csv file.</param>
    ''' <remarks>Wrapper around gclib GArrayDownload(),
    ''' http://www.galil.com/sw/pub/all/doc/gclib/html/gclibo_8h.html#a14b448ab8c7e6cf495865af301be398e
    ''' </remarks>
    ''' <exception cref="System.Exception">Will throw an exception if anything other than G_NO_ERROR is received from gclib.</exception>
    Public Sub GArrayDownloadFile(Path As String)
        Dim rc As GReturn = DllGArrayDownloadFile(ConnectionHandle_, Path)
        If rc <> G_NO_ERROR Then
            Throw New System.Exception(GError(rc))
        End If
    End Sub

    ''' <summary>
    ''' Uploads array data from the controller's array table. 
    ''' </summary>
    ''' <param name="array_name">String containing the name of the array to upload.</param>
    ''' <param name="first">The first element of the array for sub-array uploads.</param>
    ''' <param name="last">The last element of the array for sub-array uploads.</param>
    ''' <returns>The desired array as a list of doubles.</returns>
    ''' <remarks>Wrapper around gclib GArrayUpload(),
    ''' http://www.galil.com/sw/pub/all/doc/gclib/html/gclib_8h.html#af215806ec26ba06ed3f174ebeeafa7a7
    ''' </remarks>
    ''' <exception cref="System.Exception">Will throw an exception if anything other than G_NO_ERROR is received from gclib.</exception>
    Public Function GArrayUpload(array_name As String, Optional first As Int16 = -1, Optional last As Int16 = -1) As List(Of Double)
        Dim array As New List(Of Double)
        Dim rc As GReturn = DllGArrayUpload(ConnectionHandle_, array_name, first, last, 1, Buffer_, BufferSize_) '1 = comma delim
        If Not rc = G_NO_ERROR Then
            Throw New System.Exception(GError(rc))
        End If
        Dim tokens As String() = Buffer_.ToString.Split({","}, System.StringSplitOptions.RemoveEmptyEntries)
        Dim value As Double
        For Each s As String In tokens
            If Not Double.TryParse(s, value) Then
                Throw New System.Exception("Could not parse " & s & " into double")
            End If
            array.Add(value)
        Next
        Return array
    End Function

    ''' <summary>
    ''' Allows uploading of a program array file from the controller to an array CSV file.
    ''' </summary>
    ''' <param name="Path">The full filepath of the array csv file to save.</param>
    ''' <param name="Names">A space separated list of the array names to upload. A null string uploads all arrays in the array table (LA).</param>
    ''' <remarks>Wrapper around gclib GArrayUpload().
    ''' http://www.galil.com/sw/pub/all/doc/gclib/html/gclib_8h.html#af215806ec26ba06ed3f174ebeeafa7a7
    ''' </remarks>
    ''' <exception cref="System.Exception">Will throw an exception if anything other than G_NO_ERROR is received from gclib.</exception>
    Public Sub GArrayUploadFile(Path As String, Names As String)
        Dim rc As GReturn = DllGArrayUploadFile(ConnectionHandle_, Path, Names)
        If rc <> G_NO_ERROR Then
            Throw New System.Exception(GError(rc))
        End If
    End Sub

    ''' <summary>
    ''' Assigns IP address over the Ethernet to a controller at a given MAC address. 
    ''' </summary>
    ''' <param name="ip">The ip address to assign. The hardware should not yet have an IP address. </param>
    ''' <param name="mac">The MAC address of the hardware.</param>
    ''' <remarks>Wrapper around gclib GAssign(),
    ''' http://www.galil.com/sw/pub/all/doc/gclib/html/gclibo_8h.html#acc996b7c22cfed8e5573d096ef1ab759
    ''' </remarks>
    ''' <exception cref="System.Exception">Will throw an exception if anything other than G_NO_ERROR is received from gclib.</exception>
    Public Sub GAssign(ip As String, mac As String)
        Dim rc As GReturn = DllGAssign(ip, mac)
        If Not rc = G_NO_ERROR Then
            Throw New System.Exception(GError(rc))
        End If
    End Sub

    ''' <summary>
    ''' Used to close a connection to Galil hardware.
    ''' </summary>
    ''' <remarks>Wrapper around gclib GClose(),
    ''' http://www.galil.com/sw/pub/all/doc/gclib/html/gclib_8h.html#a24a437bcde9637b0db4b94176563a052
    ''' Be sure to call GClose() whenever a connection is finished.</remarks>
    Public Sub GClose()
        If ConnectionStatus_ Then
            DllGClose(ConnectionHandle_)
        End If

        ConnectionStatus_ = False
    End Sub

    ''' <summary>
    ''' Used for command-and-response transactions.
    ''' </summary>
    ''' <param name="Command">The command to send to the controller. Do not append a carriage return. Use only ASCII-based commmands.</param>
    ''' <param name="Trim">If true, the response will be trimmed of the trailing colon and any leading or trailing whitespace.</param>
    ''' <returns>The command's response.</returns>
    ''' <remarks>Wrapper around gclib GCommand(),
    ''' http://www.galil.com/sw/pub/all/doc/gclib/html/gclib_8h.html#a5ac031e76efc965affdd73a1bec084a8
    ''' </remarks>
    ''' <exception cref="System.Exception">Will throw an exception if anything other than G_NO_ERROR is received from gclib.</exception>
    Public Function GCommand(Command As String, Optional Trim As Boolean = True) As String
        Dim rc As GReturn = DllGCommand(ConnectionHandle_, Command, Buffer_, BufferSize_, 0)
        If rc <> G_NO_ERROR Then
            Throw New System.Exception(GError(rc))
        End If
        If Trim Then
            Dim r As String = Buffer_.ToString()
            If r(r.Count() - 1) = ":" Then
                r = r.Substring(0, r.Count() - 1)
            End If
            Return r.Trim()
        Else
            Return Buffer_.ToString()
        End If
    End Function


    ''' <summary>
    ''' Used for command-And-response transactions.
    ''' </summary>
    ''' <param name="Command">The command to send to the controller. Do Not append a carriage return. Use only ASCII-based commmands.</param>
    ''' <returns>The command's response parsed as an integer.</returns>
    ''' <remarks>Wrapper around gclib GCmdI(),
    ''' http://www.galil.com/sw/pub/all/doc/gclib/html/gclib_8h.html#a5ac031e76efc965affdd73a1bec084a8
    ''' </remarks>
    Public Function GCmdI(Command As String) As Int16
        Return Convert.ToInt16(Convert.ToDouble(GCommand(Command)))
    End Function

    ''' <summary>
    ''' Used for command-And-response transactions.
    ''' </summary>
    ''' <param name="Command">The command to send to the controller. Do Not append a carriage return. Use only ASCII-based commmands.</param>
    ''' <returns>The command's response parsed as a double.</returns>
    ''' <remarks>Wrapper around gclib GCmdD(),
    ''' http://www.galil.com/sw/pub/all/doc/gclib/html/gclib_8h.html#a5ac031e76efc965affdd73a1bec084a8
    ''' </remarks>
    Public Function GCmdD(Command As String) As Double
        Return Convert.ToDouble(GCommand(Command))
    End Function

    ''' <summary>
    ''' Provides a human-readable error message from a gclib error code.
    ''' </summary>
    ''' <param name="ErrorCode">The gclib error code, as returned from a call to the gclib.</param>
    ''' <returns>Error message string.</returns>
    ''' <remarks>
    ''' Wrapper around gclib GError(),
    ''' http://www.galil.com/sw/pub/all/doc/gclib/html/gclibo_8h.html#afef1bed615bd72134f3df6d3a5723cba
    '''  This function is private, all public calls that throw errors use this command for setting the exception message.
    ''' </remarks>
    Private Function GError(ErrorCode As GReturn) As String
        DllGError(ErrorCode, Buffer_, BufferSize_)
        Return ErrorCode.ToString & " " & Buffer_.ToString() & vbCrLf
    End Function

    ''' <summary>
    ''' Upgrade firmware. 
    ''' </summary>
    ''' <param name="filepath ">The full filepath of the firmware hex file.</param>
    ''' <remarks>Wrapper around gclib GFirmwareDownload(),
    ''' http://www.galil.com/sw/pub/all/doc/gclib/html/gclib_8h.html#a1878a2285ff17897fa4fb20182ba6fdf
    ''' </remarks>
    ''' <exception cref="System.Exception">Will throw an exception if anything other than G_NO_ERROR is received from gclib.</exception>
    Public Sub GFirmwareDownload(filepath As String)
        Dim rc As GReturn = DllGFirmwareDownload(ConnectionHandle_, filepath)
        If rc <> G_NO_ERROR Then
            Throw New System.Exception(GError(rc))
        End If
    End Sub

    ''' <summary>Provides a useful connection string.</summary>
    ''' <remarks>Wrapper around gclib GInfo(),
    ''' http://www.galil.com/sw/pub/all/doc/gclib/html/gclibo_8h.html#a08abfcff8a1a85a01987859473167518
    ''' </remarks>
    ''' <returns>String containing connection information, e.g. "192.168.0.43, DMC4020 Rev 1.2c, 291". A null string indicates an error was returned from the library.</returns>
    Public Function GInfo() As String
        Dim rc As GReturn = DllGInfo(ConnectionHandle_, Buffer_, BufferSize_)
        If rc = G_NO_ERROR Then
            Return Buffer_.ToString()
        Else
            Return ""
        End If
    End Function

    ''' <summary>
    ''' Provides access to PCI and UDP interrupts from the controller. 
    ''' </summary>
    ''' <returns>The status byte from the controller. Zero will be returned if a status byte is not read.</returns>
    ''' <remarks>Wrapper around gclib GInterrupt(),
    ''' http://www.galil.com/sw/pub/all/doc/gclib/html/gclib_8h.html#a5bcf802404a96343e7593d247b67f132
    ''' -s ALL or -s EI must be specified in the address argument of GOpen() to receive interrupts.</remarks>
    Public Function GInterrupt() As Byte
        Dim StatusByte As Byte = 0
        Dim rc As GReturn = DllGInterrupt(ConnectionHandle_, StatusByte)
        If rc = G_NO_ERROR Then
            Return StatusByte
        Else
            Return 0
        End If
    End Function

    ''' <summary>
    ''' Provides a list of all Galil controllers requesting IP addresses via BOOT-P or DHCP. 
    ''' </summary>
    ''' <returns>Each line of the returned data will be of the form "model, serial_number, mac". </returns>
    ''' <remarks>Wrapper around gclib GIpRequests(),
    ''' http://www.galil.com/sw/pub/all/doc/gclib/html/gclibo_8h.html#a0afb4c82642a4ef86f997c39a5518952
    ''' An empty array is returned on error.
    ''' Call will take roughly 5 seconds to return.</remarks>
    Public Function GIpRequests() As String()
        Dim rc As GReturn = DllGIpRequests(Buffer_, BufferSize_)
        If rc = G_NO_ERROR Then
            Return Buffer_.ToString().Split({vbCr, vbLf}, System.StringSplitOptions.RemoveEmptyEntries)
        Else
            Return New String() {}
        End If
    End Function

    ''' <summary>
    ''' Provides access to unsolicited messages.
    ''' </summary>
    ''' <returns>String containing all messages received by controller.</returns>
    ''' <remarks>Wrapper around gclib GMessage(),
    ''' http://www.galil.com/sw/pub/all/doc/gclib/html/gclib_8h.html#aabc5eaa09ddeca55ab8ee048b916cbcd
    '''An empty string is returned on error.
    ''' -s ALL or -s MG must be specified in the address argument of GOpen() to receive messages.</remarks>
    Public Function GMessage() As String
        Dim rc As GReturn = DllGMessage(ConnectionHandle_, Buffer_, BufferSize_)
        If rc = G_NO_ERROR Then
            Return Buffer_.ToString
        Else
            Return ""
        End If
    End Function

    ''' <summary>
    ''' Blocking call that returns once all axes specified have completed their motion. 
    ''' </summary>
    ''' <param name="axes">A string containing a multiple-axes mask. Every character in the string should be a valid argument to MG_BGm, i.e. XYZWABCDEFGHST.</param>
    ''' <remarks>Wrapper around gclib GMotionComplete(),
    ''' http://www.galil.com/sw/pub/all/doc/gclib/html/gclibo_8h.html#a19c220879442987970706444197f397a
    ''' </remarks>
    ''' <exception cref="System.Exception">Will throw an exception if anything other than G_NO_ERROR is received from gclib.</exception>
    Public Sub GMotionComplete(axes As String)
        Dim rc As GReturn = DllGMotionComplete(ConnectionHandle_, axes)
        If Not rc = G_NO_ERROR Then
            Throw New System.Exception(GError(rc))
        End If
    End Sub

    ''' <summary>
    ''' Used to open a connection to Galil hardware.
    ''' </summary>
    ''' <param name="address">Address string including any connection switches. See gclib documentation for GOpen().</param>
    ''' <remarks>Wrapper around gclib GOpen(),
    ''' http://www.galil.com/sw/pub/all/doc/gclib/html/gclib_8h.html#aef4aec8a85630eed029b7a46aea7db54
    ''' </remarks>
    ''' <exception cref="System.Exception">Will throw an exception if anything other than G_NO_ERROR is received from gclib.</exception>
    Public Sub GOpen(address As String)
        Dim rc As GReturn = DllGOpen(address, ConnectionHandle_)
        If rc <> G_NO_ERROR Then
            Throw New System.Exception(GError(rc))
        Else
            ConnectionStatus_ = True
        End If
    End Sub

    ''' <summary>
    ''' Allows downloading of a DMC program from a string buffer.
    ''' </summary>
    ''' <param name="program">The program to download.</param>
    ''' <param name="preprocessor">Preprocessor directives. Use nullstring for none.</param>
    ''' <remarks>Wrapper around gclib GProgramDownload(),
    ''' http://www.galil.com/sw/pub/all/doc/gclib/html/gclib_8h.html#acafe19b2dd0537ff458e3c8afe3acfeb
    ''' </remarks>
    ''' <exception cref="System.Exception">Will throw an exception if anything other than G_NO_ERROR is received from gclib.</exception>
    Public Sub GProgramDownload(ByRef program As String, Optional preprocessor As String = "")
        Dim rc As GReturn = DllGProgramDownload(ConnectionHandle_, program, preprocessor)
        If rc <> G_NO_ERROR Then
            Throw New System.Exception(GError(rc))
        End If
    End Sub

    ''' <summary>
    ''' Allows downloading of a DMC program from file.
    ''' </summary>
    ''' <param name="file_path">The full filepath of the DMC file.</param>
    ''' <param name="preprocessor">Preprocessor directives. Use nullstring for none.</param>
    ''' <remarks>Wrapper around gclib GProgramDownloadFile(),
    ''' http://www.galil.com/sw/pub/all/doc/gclib/html/gclibo_8h.html#a8e44e2e321df9e7b8c538bf2d640633f
    ''' </remarks>
    ''' <exception cref="System.Exception">Will throw an exception if anything other than G_NO_ERROR is received from gclib.</exception>
    Public Sub GProgramDownloadFile(file_path As String, Optional preprocessor As String = "")
        Dim rc As GReturn = DllGProgramDownloadFile(ConnectionHandle_, file_path, preprocessor)
        If rc <> G_NO_ERROR Then
            Throw New System.Exception(GError(rc))
        End If
    End Sub

    ''' <summary>
    ''' Allows uploading of a DMC program to a string.
    ''' </summary>
    ''' <remarks>Wrapper around gclib GProgramUpload(),
    ''' http://www.galil.com/sw/pub/all/doc/gclib/html/gclib_8h.html#a80a653ce387a2bd16bde2793c6de77e9
    ''' </remarks>
    ''' <exception cref="System.Exception">Will throw an exception if anything other than G_NO_ERROR is received from gclib.</exception>
    Public Function GProgramUpload() As String
        Dim rc As GReturn = DllGProgramUpload(ConnectionHandle_, Buffer_, BufferSize_)
        If rc <> G_NO_ERROR Then
            Throw New System.Exception(GError(rc))
        Else
            Return Buffer_.ToString()
        End If
    End Function

    ''' <summary>
    ''' Allows uploading of a DMC program to a file.
    ''' </summary>
    ''' <param name="file_path">The full filepath of the DMC file to save.</param>
    ''' <remarks>Wrapper around gclib GProgramUploadFile(),
    ''' http://www.galil.com/sw/pub/all/doc/gclib/html/gclibo_8h.html#a38c5565afc11762fa19d37fbaa3c9aa3
    ''' </remarks>
    ''' <exception cref="System.Exception">Will throw an exception if anything other than G_NO_ERROR is received from gclib.</exception>
    Public Sub GProgramUploadFile(file_path As String)
        Dim rc As GReturn = DllGProgramUploadFile(ConnectionHandle_, file_path)
        If rc <> G_NO_ERROR Then
            Throw New System.Exception(GError(rc))
        End If
    End Sub

    ''' <summary>
    ''' Performs a read on the connection. 
    ''' </summary>
    ''' <returns>String containing the read data, or a nullstring if nothing was read or an error occured.</returns>
    ''' <remarks>Wrapper around gclib GRead(), 
    ''' http://www.galil.com/sw/pub/all/doc/gclib/html/gclib_8h.html#adab6ec79b7e1bc7f0266684dd3434923
    ''' </remarks>
    Public Function GRead() As Byte()
        Dim read As UInteger
        Dim rc As GReturn = DllGRead(ConnectionHandle_, ByteArray_, ByteArray_.Length, read)
        If rc = G_NO_ERROR Then
            Dim ReturnData(read - 1) As Byte 'create an array of the correct size
            For i As Integer = 0 To read - 1
                ReturnData(i) = ByteArray_(i) 'copy over the data
            Next
            Return ReturnData
        Else
            Return Nothing
        End If
    End Function

    ''' <summary>
    ''' Used for retrieving data records from the controller.
    ''' </summary>
    ''' <returns>A struct containing the information of the retrieved data record.</returns>
    ''' <param name="async">False to user QR, True to use DR.</param>
    ''' <remarks>Wrapper around gclib GRecord(),
    ''' http://www.galil.com/sw/pub/all/doc/gclib/html/gclib_8h.html#a1f39cd57dcfa55d065c972a020b1f8ee
    ''' To use async, -s ALL or -s DR must be specified in the address argument of GOpen(),
    ''' and the records must be started via DR or RecordRate().
    ''' </remarks>
    ''' <exception cref="System.Exception">Will throw an exception if anything other than G_NO_ERROR is received from gclib.</exception>
    Public Function GRecord(Of T As GDataRecord)(async As Boolean) As T
        Dim method As UShort = 0 'QR mode
        If async Then
            method = 1 'DR mode
        End If

        Dim rc As GReturn = DllGRecord(ConnectionHandle_, ByteArray_, method)
        If rc <> G_NO_ERROR Then
            Throw New System.Exception(GError(rc))
        End If
        Return ByteArrayToDataRecord(Of T)(ByteArray_)
    End Function

    ''' <summary>
    ''' Sets the asynchronous data record to a user-specified period via DR. 
    ''' </summary>
    ''' <param name="period_ms">Period, in milliseconds, to set up for the asynchronous data record.</param>
    ''' <remarks>Wrapper around gclib GRecordRate(),
    ''' http://www.galil.com/sw/pub/all/doc/gclib/html/gclibo_8h.html#ada86dc9d33ac961412583881963a1b8a
    ''' Takes TM and product type into account and sets the DR period to the period requested by the user, if possible.</remarks>
    ''' <exception cref="System.Exception">Will throw an exception if anything other than G_NO_ERROR is received from gclib.</exception>
    Public Sub GRecordRate(period_ms As Double)
        Dim rc As GReturn = DllGRecordRate(ConnectionHandle_, period_ms)
        If Not rc = G_NO_ERROR Then
            Throw New System.Exception(GError(rc))
        End If
    End Sub

    ''' <summary>
    ''' Set the timeout of communication transactions. Use -1 to set the original timeout from GOpen().
    ''' </summary>
    ''' <param name="timeout_ms ">New timeout in miliseconds.</param>
    ''' <remarks>Wrapper around gclib GTimeout(),
    ''' http://www.galil.com/sw/pub/all/doc/gclib/html/gclibo_8h.html#a179aa2d1b8e2227944cc06a7ceaf5640
    ''' </remarks>
    Public Sub GTimeout(timeout_ms As Int16)
        DllGTimeout(ConnectionHandle_, timeout_ms)
    End Sub

    ''' <summary>Used to get the gclib version.</summary>
    ''' <returns>The library version, e.g. "104.73.179". A null string indicates an error was returned from the library.</returns>
    ''' <remarks>Wrapper around gclib GVersion(),
    ''' http://www.galil.com/sw/pub/all/doc/gclib/html/gclibo_8h.html#a1784b39416b77af20efc98a05f8ce475
    ''' </remarks>
    Public Function GVersion() As String
        Dim rc As GReturn = DllGVersion(Buffer_, BufferSize_)
        If rc = G_NO_ERROR Then
            Return Buffer_.ToString()
        Else
            Return ""
        End If
    End Function

    ''' <summary>
    ''' Performs a write on the connection. 
    ''' </summary>
    ''' <param name="buffer">The user's write buffer. To send a Galil command, a terminating carriage return is usually required. </param>
    ''' <remarks>Wrapper around gclib GWrite(), 
    ''' http://www.galil.com/sw/pub/all/doc/gclib/html/gclib_8h.html#abe28ebaecd5b3940adf4e145d40e5456 
    ''' </remarks>
    ''' <exception cref="System.Exception">Will throw an exception if anything other than G_NO_ERROR is received from gclib.</exception>
    Public Sub GWrite(ByRef buffer As String)
        Dim rc As GReturn = DllGWrite(ConnectionHandle_, buffer, buffer.Length())
        If Not rc = G_NO_ERROR Then
            Throw New System.Exception(GError(rc))
        End If
    End Sub

    ''' <summary>
    ''' Allows downloading of a Galil compressed backup (gcb) file to the controller.
    ''' </summary>
    ''' <param name="Path">The full filepath of the gcb file.</param>
    ''' <param name="Options">A bit mask indicating which sectors of the gcb file to restore to the controller.</param>
    ''' <returns>The controller information stored in the gcb file.</returns>
    ''' <remarks>Wrapper around gclib GSetupDownloadFile(),
    ''' 
    ''' If options is specified as 0, the return string will have a number appended corresponding to a bit mask of the available gcb sectors
    ''' </remarks>
    ''' <exception cref="System.Exception">Will throw an exception if anything other than G_NO_ERROR is received from gclib.</exception>
    Public Function GSetupDownloadFile(Path As String, Options As Int32) As String()
        'Dim _info As New System.Text.StringBuilder(BufferSize_)
        Dim rc As GReturn = DllGSetupDownloadFile(ConnectionHandle_, Path, Options, Buffer_, BufferSize_)
        Dim ret_buf As String = Buffer_.ToString()
        ret_buf = Replace(ret_buf, vbCrLf, ", ")

        If Not Options = 0 Then
            If rc <> G_NO_ERROR Then
                Throw New System.Exception(GError(rc))
            End If
        Else
            ret_buf += """options""" + "," + rc.ToString() + vbLf
        End If

        Return ret_buf.Split({vbLf}, System.StringSplitOptions.RemoveEmptyEntries)
    End Function

    ''' <summary>
    ''' Connects gclib to a New gcaps server
    ''' </summary>
    ''' <param name="server_name">Name of the server to connect.</param>
    ''' <remarks>Wrapper around gclib GSetServer(),
    ''' Call GSetServer("Local") to connect gclib back to local gcaps server
    ''' </remarks>
    ''' <exception cref="System.Exception">Will throw an exception if anything other than G_NO_ERROR Is received from gclib.</exception>
    Public Sub GSetServer(server_name As String)
        Dim rc As GReturn = DllGSetServer(server_name)

        If Not rc = G_NO_ERROR Then
            Throw New System.Exception(GError(rc))
        End If
    End Sub

    ''' <summary>
    ''' Retrieves the name of your local gcaps server And whether Or Not it Is currently published
    ''' </summary>
    ''' <returns>A string in the form "<server_name>, <isPublished>"</returns>
    ''' <exception cref="System.Exception">Will throw an exception if anything other than G_NO_ERROR Is received from gclib.</exception>
    Public Function GServerStatus() As String
        Dim rc As GReturn = DllGServerStatus(Buffer_, BufferSize_)

        If Not rc = G_NO_ERROR Then
            Throw New System.Exception(GError(rc))
        End If

        Return Buffer_.ToString()
    End Function

    ''' <summary>
    ''' Retrieves a list of gcaps servers that are advertising themselves on the local network
    ''' </summary>
    ''' <returns>A list of available gcaps server names</returns>
    ''' <exception cref="System.Exception">Will throw an exception if anything other than G_NO_ERROR Is received from gclib.</exception>
    Public Function GListServers() As String()
        Dim rc As GReturn = DllGListServers(Buffer_, BufferSize_)

        If Not rc = G_NO_ERROR Then
            Throw New System.Exception(GError(rc))
        End If

        Dim delimiters As Char() = New Char() {vbLf, vbNewLine}
        Return Buffer_.ToString().Split(delimiters, System.StringSplitOptions.RemoveEmptyEntries)
    End Function

    ''' <summary>
    ''' Publishes Or removes local gcaps server from the network
    ''' </summary>
    ''' <param name="server_name">Name to publish server under.</param>
    ''' <param name="publish">True=publish server, False=remove server.</param>
    ''' <param name="save">Save this configuration for future server reboots.</param>
    ''' <exception cref="System.Exception">Will throw an exception if anything other than G_NO_ERROR Is received from gclib.</exception>
    Public Sub GPublishServer(server_name As String, publish As Boolean, save As Boolean)
        Dim rc As GReturn = DllGPublishServer(server_name, Convert.ToInt16(publish), Convert.ToInt16(save))

        If Not rc = G_NO_ERROR Then
            Throw New System.Exception(GError(rc))
        End If
    End Sub

    ''' <summary>
    ''' Returns a list of IP Addresses that currently have an open connection to your hardware.
    ''' </summary>
    ''' <returns>Returns a list of IP Addresses that currently have an open connection to your hardware.</returns>
    ''' <exception cref="System.Exception">Will throw an exception if anything other than G_NO_ERROR Is received from gclib.</exception>
    Public Function GRemoteConnections() As String()
        Dim rc As GReturn = DllGRemoteConnections(Buffer_, BufferSize_)

        If Not rc = G_NO_ERROR Then
            Throw New System.Exception(GError(rc))
        End If

        Dim delimiters As Char() = New Char() {vbLf, vbNewLine}
        Return Buffer_.ToString().Split(delimiters, System.StringSplitOptions.RemoveEmptyEntries)
    End Function

#End Region

#Region "DLL Imports"

    'Import declarations for gclib functions. Functions are private to this class and are prefixed with "Dll" to distinguish from VB functions.

#Region "Error Codes"
    ''' <summary>Functions are checked for G_NO_ERROR.</summary>
    ''' <remarks>Some functions throw exceptions if G_NO_ERROR is not returned.</remarks>
    Private Const G_NO_ERROR As Integer = 0
#End Region

    <DllImport(GcliboDllPath_, EntryPoint:="GAddresses", CharSet:=CharSet.Ansi, CallingConvention:=CallingConvention.StdCall)>
    Private Shared Function DllGAddresses(addresses As GCStringOut, addresses_len As GSize) As GReturn
    End Function

    <DllImport(GclibDllPath_, EntryPoint:="GArrayDownload", CharSet:=CharSet.Ansi, CallingConvention:=CallingConvention.StdCall)>
    Private Shared Function DllGArrayDownload(g As GCon, array_name As GCStringIn, first As GOption,
                                                     last As GOption, buffer As GCStringIn) As GReturn
    End Function

    <DllImport(GcliboDllPath_, EntryPoint:="GArrayDownloadFile", CharSet:=CharSet.Ansi, CallingConvention:=CallingConvention.StdCall)>
    Private Shared Function DllGArrayDownloadFile(g As GCon, path As GCStringIn) As GReturn
    End Function

    <DllImport(GclibDllPath_, EntryPoint:="GArrayUpload", CharSet:=CharSet.Ansi, CallingConvention:=CallingConvention.StdCall)>
    Private Shared Function DllGArrayUpload(g As GCon, array_name As GCStringIn, first As GOption,
                                                     last As GOption, delim As GOption, buffer As GCStringOut, bufferLength As GSize) As GReturn
    End Function

    <DllImport(GcliboDllPath_, EntryPoint:="GArrayUploadFile", CharSet:=CharSet.Ansi, CallingConvention:=CallingConvention.StdCall)>
    Private Shared Function DllGArrayUploadFile(g As GCon, path As GCStringIn, names As GCStringIn) As GReturn
    End Function

    <DllImport(GcliboDllPath_, EntryPoint:="GAssign", CharSet:=CharSet.Ansi, CallingConvention:=CallingConvention.StdCall)>
    Private Shared Function DllGAssign(ip As GCStringIn, mac As GCStringIn) As GReturn
    End Function

    <DllImport(GclibDllPath_, EntryPoint:="GClose", CharSet:=CharSet.Ansi, CallingConvention:=CallingConvention.StdCall)>
    Private Shared Function DllGClose(g As GCon) As GReturn
    End Function

    <DllImport(GclibDllPath_, EntryPoint:="GCommand", CharSet:=CharSet.Ansi, CallingConvention:=CallingConvention.StdCall)>
    Private Shared Function DllGCommand(g As GCon, command As GCStringIn, buffer As GCStringOut,
                                                      bufferLength As GSize, ByRef bytesReturned As GSize) As GReturn
    End Function

    <DllImport(GcliboDllPath_, EntryPoint:="GError", CharSet:=CharSet.Ansi, CallingConvention:=CallingConvention.StdCall)>
    Private Shared Sub DllGError(error_code As GReturn, errorbuf As GCStringOut, error_len As GSize)
    End Sub

    <DllImport(GclibDllPath_, EntryPoint:="GFirmwareDownload", CharSet:=CharSet.Ansi, CallingConvention:=CallingConvention.StdCall)>
    Private Shared Function DllGFirmwareDownload(g As GCon, path As GCStringIn) As GReturn
    End Function

    <DllImport(GcliboDllPath_, EntryPoint:="GInfo", CharSet:=CharSet.Ansi, CallingConvention:=CallingConvention.StdCall)>
    Private Shared Function DllGInfo(g As GCon, info As GCStringOut, infoLength As GSize) As GReturn
    End Function

    <DllImport(GclibDllPath_, EntryPoint:="GInterrupt", CharSet:=CharSet.Ansi, CallingConvention:=CallingConvention.StdCall)>
    Private Shared Function DllGInterrupt(g As GCon, ByRef status_byte As GStatus) As GReturn
    End Function

    <DllImport(GcliboDllPath_, EntryPoint:="GIpRequests", CharSet:=CharSet.Ansi, CallingConvention:=CallingConvention.StdCall)>
    Private Shared Function DllGIpRequests(requests As GCStringOut, requests_len As GSize) As GReturn
    End Function

    <DllImport(GclibDllPath_, EntryPoint:="GMessage", CharSet:=CharSet.Ansi, CallingConvention:=CallingConvention.StdCall)>
    Private Shared Function DllGMessage(g As GCon, buffer As GCStringOut, bufferLength As GSize) As GReturn
    End Function

    <DllImport(GcliboDllPath_, EntryPoint:="GMotionComplete", CharSet:=CharSet.Ansi, CallingConvention:=CallingConvention.StdCall)>
    Private Shared Function DllGMotionComplete(g As GCon, axes As GCStringIn) As GReturn
    End Function

    <DllImport(GclibDllPath_, EntryPoint:="GOpen", CharSet:=CharSet.Ansi, CallingConvention:=CallingConvention.StdCall)>
    Private Shared Function DllGOpen(address As GCStringIn, ByRef g As GCon) As GReturn
    End Function

    <DllImport(GclibDllPath_, EntryPoint:="GProgramDownload", CharSet:=CharSet.Ansi, CallingConvention:=CallingConvention.StdCall)>
    Private Shared Function DllGProgramDownload(g As GCon, program As GCStringIn, preprocessor As GCStringIn) As GReturn
    End Function

    <DllImport(GcliboDllPath_, EntryPoint:="GProgramDownloadFile", CharSet:=CharSet.Ansi, CallingConvention:=CallingConvention.StdCall)>
    Private Shared Function DllGProgramDownloadFile(g As GCon, path As GCStringIn, preprocessor As GCStringIn) As GReturn
    End Function

    <DllImport(GclibDllPath_, EntryPoint:="GProgramUpload", CharSet:=CharSet.Ansi, CallingConvention:=CallingConvention.StdCall)>
    Private Shared Function DllGProgramUpload(g As GCon, buffer As GCStringOut, bufferLength As GSize) As GReturn
    End Function

    <DllImport(GcliboDllPath_, EntryPoint:="GProgramUploadFile", CharSet:=CharSet.Ansi, CallingConvention:=CallingConvention.StdCall)>
    Private Shared Function DllGProgramUploadFile(g As GCon, path As GCStringIn) As GReturn
    End Function

    <DllImport(GclibDllPath_, EntryPoint:="GRead", CharSet:=CharSet.Ansi, CallingConvention:=CallingConvention.StdCall)>
    Private Shared Function DllGRead(g As GCon, buffer As Byte(), buffer_len As GSize, ByRef bytes_read As GSize) As GReturn
    End Function

    <DllImport(GclibDllPath_, EntryPoint:="GRecord", CharSet:=CharSet.Ansi, CallingConvention:=CallingConvention.StdCall)>
    Private Shared Function DllGRecord(g As GCon, record As Byte(), method As GOption) As GReturn
    End Function

    <DllImport(GcliboDllPath_, EntryPoint:="GRecordRate", CharSet:=CharSet.Ansi, CallingConvention:=CallingConvention.StdCall)>
    Private Shared Function DllGRecordRate(g As GCon, period_ms As Double) As GReturn
    End Function

    <DllImport(GcliboDllPath_, EntryPoint:="GTimeout", CharSet:=CharSet.Ansi, CallingConvention:=CallingConvention.StdCall)>
    Private Shared Sub DllGTimeout(g As GCon, timeoutMs As Short)
    End Sub

    <DllImport(GcliboDllPath_, EntryPoint:="GVersion", CharSet:=CharSet.Ansi, CallingConvention:=CallingConvention.StdCall)>
    Private Shared Function DllGVersion(ver As GCStringOut, ver_len As GSize) As GReturn
    End Function

    <DllImport(GclibDllPath_, EntryPoint:="GWrite", CharSet:=CharSet.Ansi, CallingConvention:=CallingConvention.StdCall)>
    Private Shared Function DllGWrite(g As GCon, buffer As GCStringIn, buffer_len As GSize) As GReturn
    End Function

    <DllImport(GcliboDllPath_, EntryPoint:="GSetupDownloadFile", CharSet:=CharSet.Ansi, CallingConvention:=CallingConvention.StdCall)>
    Private Shared Function DllGSetupDownloadFile(g As GCon, path As GCStringIn, options As GOption, info As GCStringOut, info_len As GSize) As GReturn
    End Function

    <DllImport(GcliboDllPath_, EntryPoint:="GSetServer", CharSet:=CharSet.Ansi, CallingConvention:=CallingConvention.StdCall)>
    Private Shared Function DllGSetServer(server_name As GCStringIn) As GReturn
    End Function

    <DllImport(GcliboDllPath_, EntryPoint:="GServerStatus", CharSet:=CharSet.Ansi, CallingConvention:=CallingConvention.StdCall)>
    Private Shared Function DllGServerStatus(status As GCStringOut, status_len As GSize) As GReturn
    End Function

    <DllImport(GcliboDllPath_, EntryPoint:="GListServers", CharSet:=CharSet.Ansi, CallingConvention:=CallingConvention.StdCall)>
    Private Shared Function DllGListServers(servers As GCStringOut, servers_len As GSize) As GReturn
    End Function

    <DllImport(GcliboDllPath_, EntryPoint:="GPublishServer", CharSet:=CharSet.Ansi, CallingConvention:=CallingConvention.StdCall)>
    Private Shared Function DllGPublishServer(server_name As GCStringIn, publish As GOption, save As GOption) As GReturn
    End Function

    <DllImport(GcliboDllPath_, EntryPoint:="GRemoteConnections", CharSet:=CharSet.Ansi, CallingConvention:=CallingConvention.StdCall)>
    Private Shared Function DllGRemoteConnections(connections As GCStringOut, connections_len As GSize) As GReturn
    End Function

#End Region

#Region "Data Record"

    Private Function ByteArrayToDataRecord(Of T As GDataRecord)(array As Byte()) As T
        Dim handle As GCHandle = GCHandle.Alloc(array, GCHandleType.Pinned)
        Try
            Return Marshal.PtrToStructure(Of T)(handle.AddrOfPinnedObject())
        Finally
            handle.Free()
        End Try
    End Function

    Public Interface GDataRecord
        Function byte_array() As Byte()
    End Interface

    Private Shared Function StructToByteArray(record As GDataRecord)
        Dim size As Integer = Marshal.SizeOf(record)
        Dim arr(size) As Byte

        Dim ptr As IntPtr = Marshal.AllocHGlobal(size)
        Marshal.StructureToPtr(record, ptr, True)
        Marshal.Copy(ptr, arr, 0, size)
        Marshal.FreeHGlobal(ptr)
        Return arr
    End Function

    ' Data record struct for DMC-4000 controllers, including 4000, 4200, 4103, And 500x0.
    <StructLayout(LayoutKind.Sequential, Pack:=1)>
    Public Structure GDataRecord4000
        Implements GDataRecord

        Public header_0 As UB                   '/*00*/       1st Byte of Header.
        Public header_1 As UB                   '/*01*/       2nd Byte of Header.
        Public header_2 As UB                   '/*02*/       3rd Byte of Header.
        Public header_3 As UB                   '/*03*/       4th Byte of Header.

        Public sample_number As UW              '/*04-05*/    sample number.

        Public input_bank_0 As UB               '/*06*/       general input bank 0 (inputs 1-8).
        Public input_bank_1 As UB               '/*07*/       general input bank 1 (inputs 9-16).
        Public input_bank_2 As UB               '/*08*/       general input bank 2 (inputs 17-24).
        Public input_bank_3 As UB               '/*09*/       general input bank 3 (inputs 25-32).
        Public input_bank_4 As UB               '/*10*/       general input bank 4 (inputs 33-40).
        Public input_bank_5 As UB               '/*11*/       general input bank 5 (inputs 41-48).
        Public input_bank_6 As UB               '/*12*/       general input bank 6 (inputs 49-56).
        Public input_bank_7 As UB               '/*13*/       general input bank 7 (inputs 57-64).
        Public input_bank_8 As UB               '/*14*/       general input bank 8 (inputs 65-72).
        Public input_bank_9 As UB               '/*15*/       general input bank 9 (inputs 73-80).

        Public output_bank_0 As UB              '/*16*/       general output bank 0 (outputs 1-8).
        Public output_bank_1 As UB              '/*17*/       general output bank 1 (outputs 9-16).
        Public output_bank_2 As UB              '/*18*/       general output bank 2 (outputs 17-24).
        Public output_bank_3 As UB              '/*19*/       general output bank 3 (outputs 25-32).
        Public output_bank_4 As UB              '/*20*/       general output bank 4 (outputs 33-40).
        Public output_bank_5 As UB              '/*21*/       general output bank 5 (outputs 41-48).
        Public output_bank_6 As UB              '/*22*/       general output bank 6 (outputs 49-56).
        Public output_bank_7 As UB              '/*23*/       general output bank 7 (outputs 57-64).
        Public output_bank_8 As UB              '/*24*/       general output bank 8 (outputs 65-72).
        Public output_bank_9 As UB              '/*25*/       general output bank 9 (outputs 73-80).

        Public reserved_0 As SW                 '/*26-27*/    Reserved.
        Public reserved_2 As SW                 '/*28-29*/    Reserved.
        Public reserved_4 As SW                 '/*30-31*/    Reserved.
        Public reserved_6 As SW                 '/*32-33*/    Reserved.
        Public reserved_8 As SW                 '/*34-35*/    Reserved.
        Public reserved_10 As SW                '/*36-37*/    Reserved.
        Public reserved_12 As SW                '/*38-39*/    Reserved.
        Public reserved_14 As SW                '/*40-41*/    Reserved.

        Public ethernet_status_a As UB          '/*42*/       Ethernet Handle A Status.
        Public ethernet_status_b As UB          '/*43*/       Ethernet Handle B Status.
        Public ethernet_status_c As UB          '/*44*/       Ethernet Handle C Status.
        Public ethernet_status_d As UB          '/*45*/       Ethernet Handle D Status.
        Public ethernet_status_e As UB          '/*46*/       Ethernet Handle E Status.
        Public ethernet_status_f As UB          '/*47*/       Ethernet Handle F Status.
        Public ethernet_status_g As UB          '/*48*/       Ethernet Handle G Status.
        Public ethernet_status_h As UB          '/*49*/       Ethernet Handle H Status.

        Public error_code As UB                 '/*50*/       error code.
        Public thread_status As UB              '/*51*/       thread status
        Public amplifier_status As UL           '/*52-55*/    Amplifier Status.

        Public contour_segment_count As UL      '/*56-59*/    Segment Count for Contour Mode.
        Public contour_buffer_available As UW   '/*60-61*/    Buffer space remaining, Contour Mode.

        Public s_plane_segment_count As UW      '/*62-63*/    segment count of coordinated move for S plane.
        Public s_plane_move_status As UW        '/*64-65*/    coordinated move status for S plane.
        Public s_distance As SL                 '/*66-69*/    distance traveled in coordinated move for S plane.
        Public s_plane_buffer_available As UW   '/*70-71*/    Buffer space remaining, S Plane.

        Public t_plane_segment_count As UW      '/*72-73*/    segment count of coordinated move for T plane.
        Public t_plane_move_status As UW        '/*74-75*/    Coordinated move status for T plane.
        Public t_distance As SL                 '/*76-79*/    distance traveled in coordinated move for T plane.
        Public t_plane_buffer_available As UW   '/*80-81*/    Buffer space remaining, T Plane.

        Public axis_a_status As UW              '/*82-83*/    A axis status.
        Public axis_a_switches As UB            '/*84*/       A axis switches.
        Public axis_a_stop_code As UB           '/*85*/       A axis stop code.
        Public axis_a_reference_position As SL  '/*86-89*/    A axis reference position.
        Public axis_a_motor_position As SL      '/*90-93*/    A axis motor position.
        Public axis_a_position_error As SL      '/*94-97*/    A axis position error.
        Public axis_a_aux_position As SL        '/*98-101*/   A axis auxiliary position.
        Public axis_a_velocity As SL            '/*102-105*/  A axis velocity.
        Public axis_a_torque As SL              '/*106-109*/  A axis torque.
        Public axis_a_analog_in As UW           '/*110-111*/  A axis analog input.
        Public axis_a_halls As UB               '/*112*/      A Hall Input Status.
        Public axis_a_reserved As UB            '/*113*/      Reserved.
        Public axis_a_variable As SL            '/*114-117*/  A User-defined variable (ZA).

        Public axis_b_status As UW              '/*118-119*/  B axis status.
        Public axis_b_switches As UB            '/*120*/      B axis switches.
        Public axis_b_stop_code As UB           '/*121*/      B axis stop code.
        Public axis_b_reference_position As SL  '/*122-125*/  B axis reference position.
        Public axis_b_motor_position As SL      '/*126-129*/  B axis motor position.
        Public axis_b_position_error As SL      '/*130-133*/  B axis position error.
        Public axis_b_aux_position As SL        '/*134-137*/  B axis auxiliary position.
        Public axis_b_velocity As SL            '/*138-141*/  B axis velocity.
        Public axis_b_torque As SL              '/*142-145*/  B axis torque.
        Public axis_b_analog_in As UW           '/*146-147*/  B axis analog input.
        Public axis_b_halls As UB               '/*148*/      B Hall Input Status.
        Public axis_b_reserved As UB            '/*149*/      Reserved.
        Public axis_b_variable As SL            '/*150-153*/  B User-defined variable (ZA).

        Public axis_c_status As UW              '/*154-155*/  C axis status.
        Public axis_c_switches As UB            '/*156*/      C axis switches.
        Public axis_c_stop_code As UB           '/*157*/      C axis stop code.
        Public axis_c_reference_position As SL  '/*158-161*/  C axis reference position.
        Public axis_c_motor_position As SL      '/*162-165*/  C axis motor position.
        Public axis_c_position_error As SL      '/*166-169*/  C axis position error.
        Public axis_c_aux_position As SL        '/*170-173*/  C axis auxiliary position.
        Public axis_c_velocity As SL            '/*174-177*/  C axis velocity.
        Public axis_c_torque As SL              '/*178-181*/  C axis torque.
        Public axis_c_analog_in As UW           '/*182-183*/  C axis analog input.
        Public axis_c_halls As UB               '/*184*/      C Hall Input Status.
        Public axis_c_reserved As UB            '/*185*/      Reserved.
        Public axis_c_variable As SL            '/*186-189*/  C User-defined variable (ZA).

        Public axis_d_status As UW              '/*190-191*/  D axis status.
        Public axis_d_switches As UB            '/*192*/      D axis switches.
        Public axis_d_stop_code As UB           '/*193*/      D axis stop code.
        Public axis_d_reference_position As SL  '/*194-197*/  D axis reference position.
        Public axis_d_motor_position As SL      '/*198-201*/  D axis motor position.
        Public axis_d_position_error As SL      '/*202-205*/  D axis position error.
        Public axis_d_aux_position As SL        '/*206-209*/  D axis auxiliary position.
        Public axis_d_velocity As SL            '/*210-213*/  D axis velocity.
        Public axis_d_torque As SL              '/*214-217*/  D axis torque.
        Public axis_d_analog_in As UW           '/*218-219*/  D axis analog input.
        Public axis_d_halls As UB               '/*220*/      D Hall Input Status.
        Public axis_d_reserved As UB            '/*221*/      Reserved.
        Public axis_d_variable As SL            '/*222-225*/  D User-defined variable (ZA).

        Public axis_e_status As UW              '/*226-227*/  E axis status.
        Public axis_e_switches As UB            '/*228*/      E axis switches.
        Public axis_e_stop_code As UB           '/*229*/      E axis stop code.
        Public axis_e_reference_position As SL  '/*230-233*/  E axis reference position.
        Public axis_e_motor_position As SL      '/*234-237*/  E axis motor position.
        Public axis_e_position_error As SL      '/*238-241*/  E axis position error.
        Public axis_e_aux_position As SL        '/*242-245*/  E axis auxiliary position.
        Public axis_e_velocity As SL            '/*246-249*/  E axis velocity.
        Public axis_e_torque As SL              '/*250-253*/  E axis torque.
        Public axis_e_analog_in As UW           '/*254-255*/  E axis analog input.
        Public axis_e_halls As UB               '/*256*/      E Hall Input Status.
        Public axis_e_reserved As UB            '/*257*/      Reserved.
        Public axis_e_variable As SL            '/*258-261*/  E User-defined variable (ZA).

        Public axis_f_status As UW              '/*262-263*/  F axis status.
        Public axis_f_switches As UB            '/*264*/      F axis switches.
        Public axis_f_stop_code As UB           '/*265*/      F axis stop code.
        Public axis_f_reference_position As SL  '/*266-269*/  F axis reference position.
        Public axis_f_motor_position As SL      '/*270-273*/  F axis motor position.
        Public axis_f_position_error As SL      '/*274-277*/  F axis position error.
        Public axis_f_aux_position As SL        '/*278-281*/  F axis auxiliary position.
        Public axis_f_velocity As SL            '/*282-285*/  F axis velocity.
        Public axis_f_torque As SL              '/*286-289*/  F axis torque.
        Public axis_f_analog_in As UW           '/*290-291*/  F axis analog input.
        Public axis_f_halls As UB               '/*292*/      F Hall Input Status.
        Public axis_f_reserved As UB            '/*293*/      Reserved.
        Public axis_f_variable As SL            '/*294-297*/  F User-defined variable (ZA).

        Public axis_g_status As UW              '/*298-299*/  G axis status.
        Public axis_g_switches As UB            '/*300*/      G axis switches.
        Public axis_g_stop_code As UB           '/*301*/      G axis stop code.
        Public axis_g_reference_position As SL  '/*302-305*/  G axis reference position.
        Public axis_g_motor_position As SL      '/*306-309*/  G axis motor position.
        Public axis_g_position_error As SL      '/*310-313*/  G axis position error.
        Public axis_g_aux_position As SL        '/*314-317*/  G axis auxiliary position.
        Public axis_g_velocity As SL            '/*318-321*/  G axis velocity.
        Public axis_g_torque As SL              '/*322-325*/  G axis torque.
        Public axis_g_analog_in As UW           '/*326-327*/  G axis analog input.
        Public axis_g_halls As UB               '/*328*/      G Hall Input Status.
        Public axis_g_reserved As UB            '/*329*/      Reserved.
        Public axis_g_variable As SL            '/*330-333*/  G User-defined variable (ZA).

        Public axis_h_status As UW              '/*334-335*/  H axis status.
        Public axis_h_switches As UB            '/*336*/      H axis switches.
        Public axis_h_stop_code As UB           '/*337*/      H axis stop code.
        Public axis_h_reference_position As SL  '/*338-341*/  H axis reference position.
        Public axis_h_motor_position As SL      '/*342-345*/  H axis motor position.
        Public axis_h_position_error As SL      '/*346-349*/  H axis position error.
        Public axis_h_aux_position As SL        '/*350-353*/  H axis auxiliary position.
        Public axis_h_velocity As SL            '/*354-357*/  H axis velocity.
        Public axis_h_torque As SL              '/*358-361*/  H axis torque.
        Public axis_h_analog_in As UW           '/*362-363*/  H axis analog input.
        Public axis_h_halls As UB               '/*364*/      H Hall Input Status.
        Public axis_h_reserved As UB            '/*365*/      Reserved.
        Public axis_h_variable As SL            '/*366-369*/  H User-defined variable (ZA).

        Public Function byte_array() As Byte() Implements GDataRecord.byte_array
            Return StructToByteArray(Me)
        End Function
    End Structure

    ' Data record struct for DMC-52000 controller. Same as DMC-4000, with bank indicator added at byte 40.
    <StructLayout(LayoutKind.Sequential, Pack:=1)>
    Public Structure GDataRecord52000
        Implements GDataRecord

        Public header_0 As UB                   '/*00*/       1st Byte of Header.
        Public header_1 As UB                   '/*01*/       2nd Byte of Header.
        Public header_2 As UB                   '/*02*/       3rd Byte of Header.
        Public header_3 As UB                   '/*03*/       4th Byte of Header.

        Public sample_number As UW              '/*04-05*/    sample number.

        Public input_bank_0 As UB               '/*06*/       general input bank 0 (inputs 1-8).
        Public input_bank_1 As UB               '/*07*/       general input bank 1 (inputs 9-16).
        Public input_bank_2 As UB               '/*08*/       general input bank 2 (inputs 17-24).
        Public input_bank_3 As UB               '/*09*/       general input bank 3 (inputs 25-32).
        Public input_bank_4 As UB               '/*10*/       general input bank 4 (inputs 33-40).
        Public input_bank_5 As UB               '/*11*/       general input bank 5 (inputs 41-48).
        Public input_bank_6 As UB               '/*12*/       general input bank 6 (inputs 49-56).
        Public input_bank_7 As UB               '/*13*/       general input bank 7 (inputs 57-64).
        Public input_bank_8 As UB               '/*14*/       general input bank 8 (inputs 65-72).
        Public input_bank_9 As UB               '/*15*/       general input bank 9 (inputs 73-80).

        Public output_bank_0 As UB              '/*16*/       general output bank 0 (outputs 1-8).
        Public output_bank_1 As UB              '/*17*/       general output bank 1 (outputs 9-16).
        Public output_bank_2 As UB              '/*18*/       general output bank 2 (outputs 17-24).
        Public output_bank_3 As UB              '/*19*/       general output bank 3 (outputs 25-32).
        Public output_bank_4 As UB              '/*20*/       general output bank 4 (outputs 33-40).
        Public output_bank_5 As UB              '/*21*/       general output bank 5 (outputs 41-48).
        Public output_bank_6 As UB              '/*22*/       general output bank 6 (outputs 49-56).
        Public output_bank_7 As UB              '/*23*/       general output bank 7 (outputs 57-64).
        Public output_bank_8 As UB              '/*24*/       general output bank 8 (outputs 65-72).
        Public output_bank_9 As UB              '/*25*/       general output bank 9 (outputs 73-80).

        Public reserved_0 As SW                 '/*26-27*/    Reserved.
        Public reserved_2 As SW                 '/*28-29*/    Reserved.
        Public reserved_4 As SW                 '/*30-31*/    Reserved.
        Public reserved_6 As SW                 '/*32-33*/    Reserved.
        Public reserved_8 As SW                 '/*34-35*/    Reserved.
        Public reserved_10 As SW                '/*36-37*/    Reserved.
        Public reserved_12 As SW                '/*38-39*/    Reserved.
        Public ethercat_bank As UB              '/*40*/        EtherCAT Bank Indicator.
        Public reserved_14 As UB                '/*41*/       Reserved.

        Public ethernet_status_a As UB          '/*42*/       Ethernet Handle A Status.
        Public ethernet_status_b As UB          '/*43*/       Ethernet Handle B Status.
        Public ethernet_status_c As UB          '/*44*/       Ethernet Handle C Status.
        Public ethernet_status_d As UB          '/*45*/       Ethernet Handle D Status.
        Public ethernet_status_e As UB          '/*46*/       Ethernet Handle E Status.
        Public ethernet_status_f As UB          '/*47*/       Ethernet Handle F Status.
        Public ethernet_status_g As UB          '/*48*/       Ethernet Handle G Status.
        Public ethernet_status_h As UB          '/*49*/       Ethernet Handle H Status.

        Public error_code As UB                 '/*50*/       error code.
        Public thread_status As UB              '/*51*/       thread status
        Public amplifier_status As UL           '/*52-55*/    Amplifier Status.

        Public contour_segment_count As UL      '/*56-59*/    Segment Count for Contour Mode.
        Public contour_buffer_available As UW   '/*60-61*/    Buffer space remaining, Contour Mode.

        Public s_plane_segment_count As UW      '/*62-63*/    segment count of coordinated move for S plane.
        Public s_plane_move_status As UW        '/*64-65*/    coordinated move status for S plane.
        Public s_distance As SL                 '/*66-69*/    distance traveled in coordinated move for S plane.
        Public s_plane_buffer_available As UW   '/*70-71*/    Buffer space remaining, S Plane.

        Public t_plane_segment_count As UW      '/*72-73*/    segment count of coordinated move for T plane.
        Public t_plane_move_status As UW        '/*74-75*/    Coordinated move status for T plane.
        Public t_distance As SL                 '/*76-79*/    distance traveled in coordinated move for T plane.
        Public t_plane_buffer_available As UW   '/*80-81*/    Buffer space remaining, T Plane.

        Public axis_a_status As UW              '/*82-83*/    A axis status.
        Public axis_a_switches As UB            '/*84*/       A axis switches.
        Public axis_a_stop_code As UB           '/*85*/       A axis stop code.
        Public axis_a_reference_position As SL  '/*86-89*/    A axis reference position.
        Public axis_a_motor_position As SL      '/*90-93*/    A axis motor position.
        Public axis_a_position_error As SL      '/*94-97*/    A axis position error.
        Public axis_a_aux_position As SL        '/*98-101*/   A axis auxiliary position.
        Public axis_a_velocity As SL            '/*102-105*/  A axis velocity.
        Public axis_a_torque As SL              '/*106-109*/  A axis torque.
        Public axis_a_analog_in As UW           '/*110-111*/  A axis analog input.
        Public axis_a_halls As UB               '/*112*/      A Hall Input Status.
        Public axis_a_reserved As UB            '/*113*/      Reserved.
        Public axis_a_variable As SL            '/*114-117*/  A User-defined variable (ZA).

        Public axis_b_status As UW              '/*118-119*/  B axis status.
        Public axis_b_switches As UB            '/*120*/      B axis switches.
        Public axis_b_stop_code As UB           '/*121*/      B axis stop code.
        Public axis_b_reference_position As SL  '/*122-125*/  B axis reference position.
        Public axis_b_motor_position As SL      '/*126-129*/  B axis motor position.
        Public axis_b_position_error As SL      '/*130-133*/  B axis position error.
        Public axis_b_aux_position As SL        '/*134-137*/  B axis auxiliary position.
        Public axis_b_velocity As SL            '/*138-141*/  B axis velocity.
        Public axis_b_torque As SL              '/*142-145*/  B axis torque.
        Public axis_b_analog_in As UW           '/*146-147*/  B axis analog input.
        Public axis_b_halls As UB               '/*148*/      B Hall Input Status.
        Public axis_b_reserved As UB            '/*149*/      Reserved.
        Public axis_b_variable As SL            '/*150-153*/  B User-defined variable (ZA).

        Public axis_c_status As UW              '/*154-155*/  C axis status.
        Public axis_c_switches As UB            '/*156*/      C axis switches.
        Public axis_c_stop_code As UB           '/*157*/      C axis stop code.
        Public axis_c_reference_position As SL  '/*158-161*/  C axis reference position.
        Public axis_c_motor_position As SL      '/*162-165*/  C axis motor position.
        Public axis_c_position_error As SL      '/*166-169*/  C axis position error.
        Public axis_c_aux_position As SL        '/*170-173*/  C axis auxiliary position.
        Public axis_c_velocity As SL            '/*174-177*/  C axis velocity.
        Public axis_c_torque As SL              '/*178-181*/  C axis torque.
        Public axis_c_analog_in As UW           '/*182-183*/  C axis analog input.
        Public axis_c_halls As UB               '/*184*/      C Hall Input Status.
        Public axis_c_reserved As UB            '/*185*/      Reserved.
        Public axis_c_variable As SL            '/*186-189*/  C User-defined variable (ZA).

        Public axis_d_status As UW              '/*190-191*/  D axis status.
        Public axis_d_switches As UB            '/*192*/      D axis switches.
        Public axis_d_stop_code As UB           '/*193*/      D axis stop code.
        Public axis_d_reference_position As SL  '/*194-197*/  D axis reference position.
        Public axis_d_motor_position As SL      '/*198-201*/  D axis motor position.
        Public axis_d_position_error As SL      '/*202-205*/  D axis position error.
        Public axis_d_aux_position As SL        '/*206-209*/  D axis auxiliary position.
        Public axis_d_velocity As SL            '/*210-213*/  D axis velocity.
        Public axis_d_torque As SL              '/*214-217*/  D axis torque.
        Public axis_d_analog_in As UW           '/*218-219*/  D axis analog input.
        Public axis_d_halls As UB               '/*220*/      D Hall Input Status.
        Public axis_d_reserved As UB            '/*221*/      Reserved.
        Public axis_d_variable As SL            '/*222-225*/  D User-defined variable (ZA).

        Public axis_e_status As UW              '/*226-227*/  E axis status.
        Public axis_e_switches As UB            '/*228*/      E axis switches.
        Public axis_e_stop_code As UB           '/*229*/      E axis stop code.
        Public axis_e_reference_position As SL  '/*230-233*/  E axis reference position.
        Public axis_e_motor_position As SL      '/*234-237*/  E axis motor position.
        Public axis_e_position_error As SL      '/*238-241*/  E axis position error.
        Public axis_e_aux_position As SL        '/*242-245*/  E axis auxiliary position.
        Public axis_e_velocity As SL            '/*246-249*/  E axis velocity.
        Public axis_e_torque As SL              '/*250-253*/  E axis torque.
        Public axis_e_analog_in As UW           '/*254-255*/  E axis analog input.
        Public axis_e_halls As UB               '/*256*/      E Hall Input Status.
        Public axis_e_reserved As UB            '/*257*/      Reserved.
        Public axis_e_variable As SL            '/*258-261*/  E User-defined variable (ZA).

        Public axis_f_status As UW              '/*262-263*/  F axis status.
        Public axis_f_switches As UB            '/*264*/      F axis switches.
        Public axis_f_stop_code As UB           '/*265*/      F axis stop code.
        Public axis_f_reference_position As SL  '/*266-269*/  F axis reference position.
        Public axis_f_motor_position As SL      '/*270-273*/  F axis motor position.
        Public axis_f_position_error As SL      '/*274-277*/  F axis position error.
        Public axis_f_aux_position As SL        '/*278-281*/  F axis auxiliary position.
        Public axis_f_velocity As SL            '/*282-285*/  F axis velocity.
        Public axis_f_torque As SL              '/*286-289*/  F axis torque.
        Public axis_f_analog_in As UW           '/*290-291*/  F axis analog input.
        Public axis_f_halls As UB               '/*292*/      F Hall Input Status.
        Public axis_f_reserved As UB            '/*293*/      Reserved.
        Public axis_f_variable As SL            '/*294-297*/  F User-defined variable (ZA).

        Public axis_g_status As UW              '/*298-299*/  G axis status.
        Public axis_g_switches As UB            '/*300*/      G axis switches.
        Public axis_g_stop_code As UB           '/*301*/      G axis stop code.
        Public axis_g_reference_position As SL  '/*302-305*/  G axis reference position.
        Public axis_g_motor_position As SL      '/*306-309*/  G axis motor position.
        Public axis_g_position_error As SL      '/*310-313*/  G axis position error.
        Public axis_g_aux_position As SL        '/*314-317*/  G axis auxiliary position.
        Public axis_g_velocity As SL            '/*318-321*/  G axis velocity.
        Public axis_g_torque As SL              '/*322-325*/  G axis torque.
        Public axis_g_analog_in As UW           '/*326-327*/  G axis analog input.
        Public axis_g_halls As UB               '/*328*/      G Hall Input Status.
        Public axis_g_reserved As UB            '/*329*/      Reserved.
        Public axis_g_variable As SL            '/*330-333*/  G User-defined variable (ZA).

        Public axis_h_status As UW              '/*334-335*/  H axis status.
        Public axis_h_switches As UB            '/*336*/      H axis switches.
        Public axis_h_stop_code As UB           '/*337*/      H axis stop code.
        Public axis_h_reference_position As SL  '/*338-341*/  H axis reference position.
        Public axis_h_motor_position As SL      '/*342-345*/  H axis motor position.
        Public axis_h_position_error As SL      '/*346-349*/  H axis position error.
        Public axis_h_aux_position As SL        '/*350-353*/  H axis auxiliary position.
        Public axis_h_velocity As SL            '/*354-357*/  H axis velocity.
        Public axis_h_torque As SL              '/*358-361*/  H axis torque.
        Public axis_h_analog_in As UW           '/*362-363*/  H axis analog input.
        Public axis_h_halls As UB               '/*364*/      H Hall Input Status.
        Public axis_h_reserved As UB            '/*365*/      Reserved.
        Public axis_h_variable As SL            '/*366-369*/  H User-defined variable (ZA).

        Public Function byte_array() As Byte() Implements GDataRecord.byte_array
            Return StructToByteArray(Me)
        End Function
    End Structure


    'Data record struct for DMC-1806 controller.
    '
    'The 18x6 Data record Is the same as 4000 except the following.
    '-# No header bytes. Firmware strips it in DR. Software removes it from QR.
    '-# No Ethernet status (bytes 42-49).
    '-# No amplfifier status (bytes 52-55).
    '-# No axis-specific hall input status.
    <StructLayout(LayoutKind.Sequential, Pack:=1)>
    Public Structure GDataRecord1806
        Implements GDataRecord

        Public sample_number As UW              '/*00-01*/    sample number.

        Public input_bank_0 As UB               '/*02*/       general input bank 0 (inputs 1-8).
        Public input_bank_1 As UB               '/*03*/       general input bank 1 (inputs 9-16).
        Public input_bank_2 As UB               '/*04*/       general input bank 2 (inputs 17-24).
        Public input_bank_3 As UB               '/*05*/       general input bank 3 (inputs 25-32).
        Public input_bank_4 As UB               '/*06*/       general input bank 4 (inputs 33-40).
        Public input_bank_5 As UB               '/*07*/       general input bank 5 (inputs 41-48).
        Public input_bank_6 As UB               '/*08*/       general input bank 6 (inputs 49-56).
        Public input_bank_7 As UB               '/*09*/       general input bank 7 (inputs 57-64).
        Public input_bank_8 As UB               '/*10*/       general input bank 8 (inputs 65-72).
        Public input_bank_9 As UB               '/*11*/       general input bank 9 (inputs 73-80).

        Public output_bank_0 As UB              '/*12*/       general output bank 0 (outputs 1-8).
        Public output_bank_1 As UB              '/*13*/       general output bank 1 (outputs 9-16).
        Public output_bank_2 As UB              '/*14*/       general output bank 2 (outputs 17-24).
        Public output_bank_3 As UB              '/*15*/       general output bank 3 (outputs 25-32).
        Public output_bank_4 As UB              '/*16*/       general output bank 4 (outputs 33-40).
        Public output_bank_5 As UB              '/*17*/       general output bank 5 (outputs 41-48).
        Public output_bank_6 As UB              '/*18*/       general output bank 6 (outputs 49-56).
        Public output_bank_7 As UB              '/*19*/       general output bank 7 (outputs 57-64).
        Public output_bank_8 As UB              '/*20*/       general output bank 8 (outputs 65-72).
        Public output_bank_9 As UB              '/*21*/       general output bank 9 (outputs 73-80).

        Public reserved_0 As SW                 '/*22-23*/    Reserved.
        Public reserved_2 As SW                 '/*24-25*/    Reserved.
        Public reserved_4 As SW                 '/*26-27*/    Reserved.
        Public reserved_6 As SW                 '/*28-29*/    Reserved.
        Public reserved_8 As SW                 '/*30-31*/    Reserved.
        Public reserved_10 As SW                '/*32-33*/    Reserved.
        Public reserved_12 As SW                '/*34-35*/    Reserved.
        Public reserved_14 As SW                '/*36-37*/    Reserved.

        Public reserved_16 As UB                '/*38*/       Reserved.
        Public reserved_17 As UB                '/*39*/       Reserved.
        Public reserved_18 As UB                '/*40*/       Reserved.
        Public reserved_19 As UB                '/*41*/       Reserved.
        Public reserved_20 As UB                '/*42*/       Reserved.
        Public reserved_21 As UB                '/*43*/       Reserved.
        Public reserved_22 As UB                '/*44*/       Reserved.
        Public reserved_23 As UB                '/*45*/       Reserved.

        Public error_code As UB                 '/*46*/       error code.
        Public thread_status As UB              '/*47*/       thread status.
        Public reserved_24 As UL                '/*48-51*/    Reserved.

        Public contour_segment_count As UL      '/*52-55*/    Segment Count for Contour Mode.
        Public contour_buffer_available As UW   '/*56-57*/    Buffer space remaining, Contour Mode.

        Public s_plane_segment_count As UW      '/*58-59*/    segment count of coordinated move for S plane.
        Public s_plane_move_status As UW        '/*60-61*/    coordinated move status for S plane.
        Public s_distance As SL                 '/*62-65*/    distance traveled in coordinated move for S plane.
        Public s_plane_buffer_available As UW   '/*66-67*/    Buffer space remaining, S Plane.

        Public t_plane_segment_count As UW      '/*68-69*/    segment count of coordinated move for T plane.
        Public t_plane_move_status As UW        '/*70-71*/    Coordinated move status for T plane.
        Public t_distance As SL                 '/*72-75*/    distance traveled in coordinated move for T plane.
        Public t_plane_buffer_available As UW   '/*76-77*/    Buffer space remaining, T Plane.

        Public axis_a_status As UW              '/*78-79*/    A axis status.
        Public axis_a_switches As UB            '/*80*/       A axis switches.
        Public axis_a_stop_code As UB           '/*81*/       A axis stop code.
        Public axis_a_reference_position As SL  '/*82-85*/    A axis reference position.
        Public axis_a_motor_position As SL      '/*86-89*/    A axis motor position.
        Public axis_a_position_error As SL      '/*90-93*/    A axis position error.
        Public axis_a_aux_position As SL        '/*94-97*/    A axis auxiliary position.
        Public axis_a_velocity As SL            '/*98-101*/   A axis velocity.
        Public axis_a_torque As SL              '/*102-105*/  A axis torque.
        Public axis_a_analog_in As UW           '/*106-107*/  A axis analog input.
        Public axis_a_reserved_0 As UB          '/*108*/      Reserved.
        Public axis_a_reserved_1 As UB          '/*109*/      Reserved.
        Public axis_a_variable As SL            '/*110-113*/  A User-defined variable (ZA).

        Public axis_b_status As UW              '/*114-115*/  B axis status.
        Public axis_b_switches As UB            '/*116*/      B axis switches.
        Public axis_b_stop_code As UB           '/*117*/      B axis stop code.
        Public axis_b_reference_position As SL  '/*118-121*/  B axis reference position.
        Public axis_b_motor_position As SL      '/*122-125*/  B axis motor position.
        Public axis_b_position_error As SL      '/*126-129*/  B axis position error.
        Public axis_b_aux_position As SL        '/*130-133*/  B axis auxiliary position.
        Public axis_b_velocity As SL            '/*134-137*/  B axis velocity.
        Public axis_b_torque As SL              '/*138-141*/  B axis torque.
        Public axis_b_analog_in As UW           '/*142-143*/  B axis analog input.
        Public axis_b_reserved_0 As UB          '/*144*/      Reserved.
        Public axis_b_reserved_1 As UB          '/*145*/      Reserved.
        Public axis_b_variable As SL            '/*146-149*/  B User-defined variable (ZA).

        Public axis_c_status As UW              '/*150-151*/  C axis status.
        Public axis_c_switches As UB            '/*152*/      C axis switches.
        Public axis_c_stop_code As UB           '/*153*/      C axis stop code.
        Public axis_c_reference_position As SL  '/*154-157*/  C axis reference position.
        Public axis_c_motor_position As SL      '/*158-161*/  C axis motor position.
        Public axis_c_position_error As SL      '/*162-165*/  C axis position error.
        Public axis_c_aux_position As SL        '/*166-169*/  C axis auxiliary position.
        Public axis_c_velocity As SL            '/*170-173*/  C axis velocity.
        Public axis_c_torque As SL              '/*174-177*/  C axis torque.
        Public axis_c_analog_in As UW           '/*178-179*/  C axis analog input.
        Public axis_c_reserved_0 As UB          '/*180*/      Reserved.
        Public axis_c_reserved_1 As UB          '/*181*/      Reserved.
        Public axis_c_variable As SL            '/*182-185*/  C User-defined variable (ZA).

        Public axis_d_status As UW              '/*186-187*/  D axis status.
        Public axis_d_switches As UB            '/*188*/      D axis switches.
        Public axis_d_stop_code As UB           '/*189*/      D axis stop code.
        Public axis_d_reference_position As SL  '/*190-193*/  D axis reference position.
        Public axis_d_motor_position As SL      '/*194-197*/  D axis motor position.
        Public axis_d_position_error As SL      '/*198-201*/  D axis position error.
        Public axis_d_aux_position As SL        '/*202-205*/  D axis auxiliary position.
        Public axis_d_velocity As SL            '/*206-209*/  D axis velocity.
        Public axis_d_torque As SL              '/*210-213*/  D axis torque.
        Public axis_d_analog_in As UW           '/*214-215*/  D axis analog input.
        Public axis_d_reserved_0 As UB          '/*216*/      Reserved.
        Public axis_d_reserved_1 As UB          '/*217*/      Reserved.
        Public axis_d_variable As SL            '/*218-221*/  D User-defined variable (ZA).

        Public axis_e_status As UW              '/*222-223*/  E axis status.
        Public axis_e_switches As UB            '/*224*/      E axis switches.
        Public axis_e_stop_code As UB           '/*225*/      E axis stop code.
        Public axis_e_reference_position As SL  '/*226-229*/  E axis reference position.
        Public axis_e_motor_position As SL      '/*230-233*/  E axis motor position.
        Public axis_e_position_error As SL      '/*234-237*/  E axis position error.
        Public axis_e_aux_position As SL        '/*238-241*/  E axis auxiliary position.
        Public axis_e_velocity As SL            '/*242-245*/  E axis velocity.
        Public axis_e_torque As SL              '/*256-249*/  E axis torque.
        Public axis_e_analog_in As UW           '/*250-251*/  E axis analog input.
        Public axis_e_reserved_0 As UB          '/*252*/      Reserved.
        Public axis_e_reserved_1 As UB          '/*253*/      Reserved.
        Public axis_e_variable As SL            '/*254-257*/  E User-defined variable (ZA).

        Public axis_f_status As UW              '/*258-259*/  F axis status.
        Public axis_f_switches As UB            '/*260*/      F axis switches.
        Public axis_f_stop_code As UB           '/*261*/      F axis stop code.
        Public axis_f_reference_position As SL  '/*262-265*/  F axis reference position.
        Public axis_f_motor_position As SL      '/*266-269*/  F axis motor position.
        Public axis_f_position_error As SL      '/*270-273*/  F axis position error.
        Public axis_f_aux_position As SL        '/*274-277*/  F axis auxiliary position.
        Public axis_f_velocity As SL            '/*278-281*/  F axis velocity.
        Public axis_f_torque As SL              '/*282-285*/  F axis torque.
        Public axis_f_analog_in As UW           '/*286-287*/  F axis analog input.
        Public axis_f_reserved_0 As UB          '/*288*/      Reserved.
        Public axis_f_reserved_1 As UB          '/*289*/      Reserved.
        Public axis_f_variable As SL            '/*290-293*/  F User-defined variable (ZA).

        Public axis_g_status As UW              '/*294-295*/  G axis status.
        Public axis_g_switches As UB            '/*296*/      G axis switches.
        Public axis_g_stop_code As UB           '/*297*/      G axis stop code.
        Public axis_g_reference_position As SL  '/*298-301*/  G axis reference position.
        Public axis_g_motor_position As SL      '/*302-305*/  G axis motor position.
        Public axis_g_position_error As SL      '/*306-309*/  G axis position error.
        Public axis_g_aux_position As SL        '/*310-313*/  G axis auxiliary position.
        Public axis_g_velocity As SL            '/*314-317*/  G axis velocity.
        Public axis_g_torque As SL              '/*318-321*/  G axis torque.
        Public axis_g_analog_in As UW           '/*322-323*/  G axis analog input.
        Public axis_g_reserved_0 As UB          '/*324*/      Reserved.
        Public axis_g_reserved_1 As UB          '/*325*/      Reserved.
        Public axis_g_variable As SL            '/*326-329*/  G User-defined variable (ZA).

        Public axis_h_status As UW              '/*330-331*/  H axis status.
        Public axis_h_switches As UB            '/*332*/      H axis switches.
        Public axis_h_stop_code As UB           '/*333*/      H axis stop code.
        Public axis_h_reference_position As SL  '/*334-337*/  H axis reference position.
        Public axis_h_motor_position As SL      '/*338-341*/  H axis motor position.
        Public axis_h_position_error As SL      '/*342-345*/  H axis position error.
        Public axis_h_aux_position As SL        '/*346-349*/  H axis auxiliary position.
        Public axis_h_velocity As SL            '/*350-353*/  H axis velocity.
        Public axis_h_torque As SL              '/*354-357*/  H axis torque.
        Public axis_h_analog_in As UW           '/*358-359*/  H axis analog input.
        Public axis_h_reserved_0 As UB          '/*360*/      Reserved.
        Public axis_h_reserved_1 As UB          '/*361*/      Reserved.
        Public axis_h_variable As SL            '/*362-365*/  H User-defined variable (ZA).

        Public Function byte_array() As Byte() Implements GDataRecord.byte_array
            Return StructToByteArray(Me)
        End Function
    End Structure

    ' Data record struct for DMC-2103 controllers.
    <StructLayout(LayoutKind.Sequential, Pack:=1)>
    Public Structure GDataRecord2103
        Implements GDataRecord

        Public header_0 As UB                   '/*00*/       1st Byte of Header.
        Public header_1 As UB                   '/*01*/       2nd Byte of Header.
        Public header_2 As UB                   '/*02*/       3rd Byte of Header.
        Public header_3 As UB                   '/*03*/       4th Byte of Header.

        Public sample_number As UW              '/*04-05*/    sample number.

        Public input_bank_0 As UB               '/*06*/       general input bank 0 (inputs 1-8).
        Public input_bank_1 As UB               '/*07*/       general input bank 1 (inputs 9-16).
        Public input_bank_2 As UB               '/*08*/       general input bank 2 (inputs 17-24).
        Public input_bank_3 As UB               '/*09*/       general input bank 3 (inputs 25-32).
        Public input_bank_4 As UB               '/*10*/       general input bank 4 (inputs 33-40).
        Public input_bank_5 As UB               '/*11*/       general input bank 5 (inputs 41-48).
        Public input_bank_6 As UB               '/*12*/       general input bank 6 (inputs 49-56).
        Public input_bank_7 As UB               '/*13*/       general input bank 7 (inputs 57-64).
        Public input_bank_8 As UB               '/*14*/       general input bank 8 (inputs 65-72).
        Public input_bank_9 As UB               '/*15*/       general input bank 9 (inputs 73-80).

        Public output_bank_0 As UB              '/*16*/       general output bank 0 (outputs 1-8).
        Public output_bank_1 As UB              '/*17*/       general output bank 1 (outputs 9-16).
        Public output_bank_2 As UB              '/*18*/       general output bank 2 (outputs 17-24).
        Public output_bank_3 As UB              '/*19*/       general output bank 3 (outputs 25-32).
        Public output_bank_4 As UB              '/*20*/       general output bank 4 (outputs 33-40).
        Public output_bank_5 As UB              '/*21*/       general output bank 5 (outputs 41-48).
        Public output_bank_6 As UB              '/*22*/       general output bank 6 (outputs 49-56).
        Public output_bank_7 As UB              '/*23*/       general output bank 7 (outputs 57-64).
        Public output_bank_8 As UB              '/*24*/       general output bank 8 (outputs 65-72).
        Public output_bank_9 As UB              '/*25*/       general output bank 9 (outputs 73-80).

        Public error_code As UB                 '/*26*/       error code.
        Public general_status As UB             '/*27*/       general status

        Public s_plane_segment_count As UW      '/*28-29*/    segment count of coordinated move for S plane.
        Public s_plane_move_status As UW        '/*30-31*/    coordinated move status for S plane.
        Public s_distance As SL                 '/*32-35*/    distance traveled in coordinated move for S plane.

        Public t_plane_segment_count As UW      '/*36-37*/    segment count of coordinated move for T plane.
        Public t_plane_move_status As UW        '/*38-39*/    Coordinated move status for T plane.
        Public t_distance As SL                 '/*40-43*/    distance traveled in coordinated move for T plane.

        Public axis_a_status As UW              '/*44-45*/    A axis status.
        Public axis_a_switches As UB            '/*46*/       A axis switches.
        Public axis_a_stop_code As UB           '/*47*/       A axis stop code.
        Public axis_a_reference_position As SL  '/*48-51*/    A axis reference position.
        Public axis_a_motor_position As SL      '/*52-55*/    A axis motor position.
        Public axis_a_position_error As SL      '/*56-59*/    A axis position error.
        Public axis_a_aux_position As SL        '/*60-63*/    A axis auxiliary position.
        Public axis_a_velocity As SL            '/*64-67*/    A axis velocity.
        Public axis_a_torque As SW              '/*68-69*/    A axis torque.
        Public axis_a_analog_in As UW           '/*70-71*/    A axis analog input.

        Public axis_b_status As UW              '/*72-73*/    B axis status.
        Public axis_b_switches As UB            '/*74*/       B axis switches.
        Public axis_b_stop_code As UB           '/*75*/       B axis stop code.
        Public axis_b_reference_position As SL  '/*76-79*/    B axis reference position.
        Public axis_b_motor_position As SL      '/*80-83*/    B axis motor position.
        Public axis_b_position_error As SL      '/*84-87*/    B axis position error.
        Public axis_b_aux_position As SL        '/*88-91*/    B axis auxiliary position.
        Public axis_b_velocity As SL            '/*92-95*/    B axis velocity.
        Public axis_b_torque As SW              '/*96-97*/    B axis torque.
        Public axis_b_analog_in As UW           '/*98-99*/    B axis analog input.

        Public axis_c_status As UW              '/*100-101*/  C axis status.
        Public axis_c_switches As UB            '/*102*/      C axis switches.
        Public axis_c_stop_code As UB           '/*103*/      C axis stop code.
        Public axis_c_reference_position As SL  '/*104-107*/  C axis reference position.
        Public axis_c_motor_position As SL      '/*108-111*/  C axis motor position.
        Public axis_c_position_error As SL      '/*112-115*/  C axis position error.
        Public axis_c_aux_position As SL        '/*116-119*/  C axis auxiliary position.
        Public axis_c_velocity As SL            '/*120-123*/  C axis velocity.
        Public axis_c_torque As SW              '/*124-125*/  C axis torque.
        Public axis_c_analog_in As UW           '/*126-127*/  C axis analog input.

        Public axis_d_status As UW              '/*128-129*/  D axis status.
        Public axis_d_switches As UB            '/*130*/      D axis switches.
        Public axis_d_stop_code As UB           '/*131*/      D axis stop code.
        Public axis_d_reference_position As SL  '/*132-135*/  D axis reference position.
        Public axis_d_motor_position As SL      '/*136-139*/  D axis motor position.
        Public axis_d_position_error As SL      '/*140-143*/  D axis position error.
        Public axis_d_aux_position As SL        '/*144-147*/  D axis auxiliary position.
        Public axis_d_velocity As SL            '/*148-151*/  D axis velocity.
        Public axis_d_torque As SW              '/*152-153*/  D axis torque.
        Public axis_d_analog_in As UW           '/*154-155*/  D axis analog input.

        Public axis_e_status As UW              '/*156-157*/  E axis status.
        Public axis_e_switches As UB            '/*158*/      E axis switches.
        Public axis_e_stop_code As UB           '/*159*/      E axis stop code.
        Public axis_e_reference_position As SL  '/*160-163*/  E axis reference position.
        Public axis_e_motor_position As SL      '/*164-167*/  E axis motor position.
        Public axis_e_position_error As SL      '/*168-171*/  E axis position error.
        Public axis_e_aux_position As SL        '/*172-175*/  E axis auxiliary position.
        Public axis_e_velocity As SL            '/*176-179*/  E axis velocity.
        Public axis_e_torque As SW              '/*180-181*/  E axis torque.
        Public axis_e_analog_in As UW           '/*182-183*/  E axis analog input.

        Public axis_f_status As UW              '/*184-185*/  F axis status.
        Public axis_f_switches As UB            '/*186*/      F axis switches.
        Public axis_f_stop_code As UB           '/*187*/      F axis stop code.
        Public axis_f_reference_position As SL  '/*188-191*/  F axis reference position.
        Public axis_f_motor_position As SL      '/*192-195*/  F axis motor position.
        Public axis_f_position_error As SL      '/*196-199*/  F axis position error.
        Public axis_f_aux_position As SL        '/*200-203*/  F axis auxiliary position.
        Public axis_f_velocity As SL            '/*204-207*/  F axis velocity.
        Public axis_f_torque As SW              '/*208-209*/  F axis torque.
        Public axis_f_analog_in As UW           '/*210-211*/  F axis analog input.

        Public axis_g_status As UW              '/*212-213*/  G axis status.
        Public axis_g_switches As UB            '/*214*/      G axis switches.
        Public axis_g_stop_code As UB           '/*215*/      G axis stop code.
        Public axis_g_reference_position As SL  '/*216-219*/  G axis reference position.
        Public axis_g_motor_position As SL      '/*220-223*/  G axis motor position.
        Public axis_g_position_error As SL      '/*224-227*/  G axis position error.
        Public axis_g_aux_position As SL        '/*228-231*/  G axis auxiliary position.
        Public axis_g_velocity As SL            '/*232-235*/  G axis velocity.
        Public axis_g_torque As SW              '/*236-237*/  G axis torque.
        Public axis_g_analog_in As UW           '/*238-239*/  G axis analog input.

        Public axis_h_status As UW              '/*240-241*/  H axis status.
        Public axis_h_switches As UB            '/*242*/      H axis switches.
        Public axis_h_stop_code As UB           '/*243*/      H axis stop code.
        Public axis_h_reference_position As SL  '/*244-247*/  H axis reference position.
        Public axis_h_motor_position As SL      '/*248-251*/  H axis motor position.
        Public axis_h_position_error As SL      '/*252-255*/  H axis position error.
        Public axis_h_aux_position As SL        '/*256-259*/  H axis auxiliary position.
        Public axis_h_velocity As SL            '/*260-263*/  H axis velocity.
        Public axis_h_torque As SW              '/*264-265*/  H axis torque.
        Public axis_h_analog_in As UW           '/*266-267*/  H axis analog input.

        Public Function byte_array() As Byte() Implements GDataRecord.byte_array
            Return StructToByteArray(Me)
        End Function
    End Structure

    ' Data record struct for DMC-1802 controllers.
    '
    'The 18x2 Data record Is the Same as 2103 except the following.
    '-# No header bytes. Software removes it from QR.
    '-# No analog in axis data.
    <StructLayout(LayoutKind.Sequential, Pack:=1)>
    Public Structure GDataRecord1802
        Implements GDataRecord

        Public sample_number As UW              '/*00-01*/    sample number.

        Public input_bank_0 As UB               '/*02*/       general input bank 0 (inputs 1-8).
        Public input_bank_1 As UB               '/*03*/       general input bank 1 (inputs 9-16).
        Public input_bank_2 As UB               '/*04*/       general input bank 2 (inputs 17-24).
        Public input_bank_3 As UB               '/*05*/       general input bank 3 (inputs 25-32).
        Public input_bank_4 As UB               '/*06*/       general input bank 4 (inputs 33-40).
        Public input_bank_5 As UB               '/*07*/       general input bank 5 (inputs 41-48).
        Public input_bank_6 As UB               '/*08*/       general input bank 6 (inputs 49-56).
        Public input_bank_7 As UB               '/*09*/       general input bank 7 (inputs 57-64).
        Public input_bank_8 As UB               '/*10*/       general input bank 8 (inputs 65-72).
        Public input_bank_9 As UB               '/*11*/       general input bank 9 (inputs 73-80).

        Public output_bank_0 As UB              '/*12*/       general output bank 0 (outputs 1-8).
        Public output_bank_1 As UB              '/*13*/       general output bank 1 (outputs 9-16).
        Public output_bank_2 As UB              '/*14*/       general output bank 2 (outputs 17-24).
        Public output_bank_3 As UB              '/*15*/       general output bank 3 (outputs 25-32).
        Public output_bank_4 As UB              '/*16*/       general output bank 4 (outputs 33-40).
        Public output_bank_5 As UB              '/*17*/       general output bank 5 (outputs 41-48).
        Public output_bank_6 As UB              '/*18*/       general output bank 6 (outputs 49-56).
        Public output_bank_7 As UB              '/*19*/       general output bank 7 (outputs 57-64).
        Public output_bank_8 As UB              '/*20*/       general output bank 8 (outputs 65-72).
        Public output_bank_9 As UB              '/*21*/       general output bank 9 (outputs 73-80).

        Public error_code As UB                 '/*22*/       error code.
        Public general_status As UB             '/*23*/       general status

        Public s_plane_segment_count As UW      '/*24-25*/    segment count of coordinated move for S plane.
        Public s_plane_move_status As UW        '/*26-27*/    coordinated move status for S plane.
        Public s_distance As SL                 '/*28-31*/    distance traveled in coordinated move for S plane.

        Public t_plane_segment_count As UW      '/*32-33*/    segment count of coordinated move for T plane.
        Public t_plane_move_status As UW        '/*34-35*/    Coordinated move status for T plane.
        Public t_distance As SL                 '/*36-39*/    distance traveled in coordinated move for T plane.

        Public axis_a_status As UW              '/*40-41*/    A axis status.
        Public axis_a_switches As UB            '/*42*/       A axis switches.
        Public axis_a_stop_code As UB           '/*43*/       A axis stop code.
        Public axis_a_reference_position As SL  '/*44-47*/    A axis reference position.
        Public axis_a_motor_position As SL      '/*48-51*/    A axis motor position.
        Public axis_a_position_error As SL      '/*52-55*/    A axis position error.
        Public axis_a_aux_position As SL        '/*56-59*/    A axis auxiliary position.
        Public axis_a_velocity As SL            '/*60-63*/    A axis velocity.
        Public axis_a_torque As SW              '/*64-65*/    A axis torque.
        Public axis_a_reserved_0 As UB          '/*66*/       Reserved.
        Public axis_a_reserved_1 As UB          '/*67*/       Reserved.

        Public axis_b_status As UW              '/*68-69*/    B axis status.
        Public axis_b_switches As UB            '/*70*/       B axis switches.
        Public axis_b_stop_code As UB           '/*71*/       B axis stop code.
        Public axis_b_reference_position As SL  '/*72-75*/    B axis reference position.
        Public axis_b_motor_position As SL      '/*76-79*/    B axis motor position.
        Public axis_b_position_error As SL      '/*80-83*/    B axis position error.
        Public axis_b_aux_position As SL        '/*84-87*/    B axis auxiliary position.
        Public axis_b_velocity As SL            '/*88-91*/    B axis velocity.
        Public axis_b_torque As SW              '/*92-93*/    B axis torque.
        Public axis_b_reserved_0 As UB          '/*94*/       Reserved.
        Public axis_b_reserved_1 As UB          '/*95*/       Reserved.

        Public axis_c_status As UW              '/*96-97*/    C axis status.
        Public axis_c_switches As UB            '/*98*/       C axis switches.
        Public axis_c_stop_code As UB           '/*99*/       C axis stop code.
        Public axis_c_reference_position As SL  '/*100-103*/  C axis reference position.
        Public axis_c_motor_position As SL      '/*104-107*/  C axis motor position.
        Public axis_c_position_error As SL      '/*108-111*/  C axis position error.
        Public axis_c_aux_position As SL        '/*112-115*/  C axis auxiliary position.
        Public axis_c_velocity As SL            '/*116-119*/  C axis velocity.
        Public axis_c_torque As SW              '/*120-121*/  C axis torque.
        Public axis_c_reserved_0 As UB          '/*122*/      Reserved.
        Public axis_c_reserved_1 As UB          '/*123*/      Reserved.

        Public axis_d_status As UW              '/*124-125*/  D axis status.
        Public axis_d_switches As UB            '/*126*/      D axis switches.
        Public axis_d_stop_code As UB           '/*127*/      D axis stop code.
        Public axis_d_reference_position As SL  '/*128-131*/  D axis reference position.
        Public axis_d_motor_position As SL      '/*132-135*/  D axis motor position.
        Public axis_d_position_error As SL      '/*136-139*/  D axis position error.
        Public axis_d_aux_position As SL        '/*140-143*/  D axis auxiliary position.
        Public axis_d_velocity As SL            '/*144-147*/  D axis velocity.
        Public axis_d_torque As SW              '/*148-149*/  D axis torque.
        Public axis_d_reserved_0 As UB          '/*150*/      Reserved.
        Public axis_d_reserved_1 As UB          '/*151*/      Reserved.

        Public Function byte_array() As Byte() Implements GDataRecord.byte_array
            Return StructToByteArray(Me)
        End Function
    End Structure

    ' Data record struct for DMC-30010 controllers. 
    <StructLayout(LayoutKind.Sequential, Pack:=1)>
    Public Structure GDataRecord30000
        Implements GDataRecord

        Public header_0 As UB                   '/*00*/       1st Byte of Header.
        Public header_1 As UB                   '/*01*/       2nd Byte of Header.
        Public header_2 As UB                   '/*02*/       3rd Byte of Header.
        Public header_3 As UB                   '/*03*/       4th Byte of Header.

        Public sample_number As UW              '/*04-05*/    sample number.

        Public input_bank_0 As UB               '/*06*/       general input bank 0 (inputs 1-8).
        Public input_bank_1 As UB               '/*07*/       general input bank 1 (inputs 9-16).

        Public output_bank_0 As UB              '/*08*/       general output bank 0 (outputs 1-8).
        Public output_bank_1 As UB              '/*09*/       general output bank 1 (outputs 9-16).

        Public error_code As UB                 '/*10*/       error code.
        Public thread_status As UB              '/*11*/       thread status.

        Public input_analog_2 As UW             '/*12-13*/     Analog input 2. 1 is in axis data, see axis_a_analog_in.

        Public output_analog_1 As UW            '/*14-15*/     Analog output 1.
        Public output_analog_2 As UW            '/*16-17*/     Analog output 2.

        Public amplifier_status As UL           '/*18-21*/    Amplifier Status.

        Public contour_segment_count As UL      '/*22-25*/    Segment Count for Contour Mode.
        Public contour_buffer_available As UW   '/*26-27*/    Buffer space remaining, Contour Mode.

        Public s_plane_segment_count As UW      '/*28-29*/    segment count of coordinated move for S plane.
        Public s_plane_move_status As UW        '/*30-31*/    coordinated move status for S plane.
        Public s_distance As SL                 '/*32-35*/    distance traveled in coordinated move for S plane.
        Public s_plane_buffer_available As UW   '/*36-37*/    Buffer space remaining, S Plane.

        Public axis_a_status As UW              '/*38-39*/    A axis status.
        Public axis_a_switches As UB            '/*40*/       A axis switches.
        Public axis_a_stop_code As UB           '/*41*/       A axis stop code.
        Public axis_a_reference_position As SL  '/*42-45*/    A axis reference position.
        Public axis_a_motor_position As SL      '/*46-49*/    A axis motor position.
        Public axis_a_position_error As SL      '/*50-53*/    A axis position error.
        Public axis_a_aux_position As SL        '/*54-57*/    A axis auxiliary position.
        Public axis_a_velocity As SL            '/*58-61*/    A axis velocity.
        Public axis_a_torque As SL              '/*62-65*/    A axis torque.
        Public axis_a_analog_in As UW           '/*66-67*/    A axis analog input.
        Public axis_a_halls As UB               '/*68*/       A Hall Input Status.
        Public axis_a_reserved As UB            '/*69*/       Reserved.
        Public axis_a_variable As SL            '/*70-73*/    A User-defined variable (ZA).

        Public Function byte_array() As Byte() Implements GDataRecord.byte_array
            Return StructToByteArray(Me)
        End Function
    End Structure

    ' Data record struct for RIO-471xx and RIO-472xx PLCs. Includes encoder fields.
    <StructLayout(LayoutKind.Sequential, Pack:=1)>
    Public Structure GDataRecord47000_ENC
        Implements GDataRecord

        Public header_0 As UB                   '/*00*/        1st Byte of Header.
        Public header_1 As UB                   '/*01*/        2nd Byte of Header.
        Public header_2 As UB                   '/*02*/        3rd Byte of Header.
        Public header_3 As UB                   '/*03*/        4th Byte of Header.

        Public sample_number As UW              '/*04-05*/     Sample number.
        Public error_code As UB                 '/*06*/        Error code.
        Public general_status As UB             '/*07*/        General status.

        Public output_analog_0 As UW            '/*08-09*/     Analog output 0.
        Public output_analog_1 As UW            '/*10-11*/     Analog output 1.
        Public output_analog_2 As UW            '/*12-13*/     Analog output 2.
        Public output_analog_3 As UW            '/*14-15*/     Analog output 3.
        Public output_analog_4 As UW            '/*16-17*/     Analog output 4.
        Public output_analog_5 As UW            '/*18-19*/     Analog output 5.
        Public output_analog_6 As UW            '/*20-21*/     Analog output 6.
        Public output_analog_7 As UW            '/*22-23*/     Analog output 7.

        Public input_analog_0 As UW             '/*24-25*/     Analog input 0.
        Public input_analog_1 As UW             '/*26-27*/     Analog input 1.
        Public input_analog_2 As UW             '/*28-29*/     Analog input 2.
        Public input_analog_3 As UW             '/*30-31*/     Analog input 3.
        Public input_analog_4 As UW             '/*32-33*/     Analog input 4.
        Public input_analog_5 As UW             '/*34-35*/     Analog input 5.
        Public input_analog_6 As UW             '/*36-37*/     Analog input 6.
        Public input_analog_7 As UW             '/*38-39*/     Analog input 7.

        Public output_bank_0 As UW              '/*40-41*/     Digital outputs 0-15;

        Public input_bank_0 As UW               '/*42-43*/     Digital inputs 0-15;

        Public pulse_count_0 As UL              '/*44-47*/     Pulse counter (see PC).
        Public zc_variable As SL                '/*48-51*/     ZC User-defined variable (see ZC).
        Public zd_variable As SL                '/*52-55*/     ZD User-defined variable (see ZD).

        Public encoder_0 As SL                  '/*56-59*/     Encoder channel 0. Data only valid for parts with -BISS, -QUAD, or -SSI.
        Public encoder_1 As SL                  '/*60-63*/     Encoder channel 1. Data only valid for parts with -BISS, -QUAD, or -SSI.
        Public encoder_2 As SL                  '/*64-67*/     Encoder channel 2. Data only valid for parts with -BISS, -QUAD, or -SSI.
        Public encoder_3 As SL                  '/*68-71*/     Encoder channel 3. Data only valid for parts with -BISS, -QUAD, or -SSI.

        Public Function byte_array() As Byte() Implements GDataRecord.byte_array
            Return StructToByteArray(Me)
        End Function
    End Structure

    ' Data record struct for RIO-47300. Includes encoder fields.
    <StructLayout(LayoutKind.Sequential, Pack:=1)>
    Public Structure GDataRecord47300_ENC
        Implements GDataRecord

        Public header_0 As UB                   '/*00*/        1st Byte of Header.
        Public header_1 As UB                   '/*01*/        2nd Byte of Header.
        Public header_2 As UB                   '/*02*/        3rd Byte of Header.
        Public header_3 As UB                   '/*03*/        4th Byte of Header.

        Public sample_number As UW              '/*04-05*/     Sample number.
        Public error_code As UB                 '/*06*/        Error code.
        Public general_status As UB             '/*07*/        General status.

        Public output_analog_0 As UW            '/*08-09*/     Analog output 0.
        Public output_analog_1 As UW            '/*10-11*/     Analog output 1.
        Public output_analog_2 As UW            '/*12-13*/     Analog output 2.
        Public output_analog_3 As UW            '/*14-15*/     Analog output 3.
        Public output_analog_4 As UW            '/*16-17*/     Analog output 4.
        Public output_analog_5 As UW            '/*18-19*/     Analog output 5.
        Public output_analog_6 As UW            '/*20-21*/     Analog output 6.
        Public output_analog_7 As UW            '/*22-23*/     Analog output 7.

        Public input_analog_0 As UW             '/*24-25*/     Analog input 0.
        Public input_analog_1 As UW             '/*26-27*/     Analog input 1.
        Public input_analog_2 As UW             '/*28-29*/     Analog input 2.
        Public input_analog_3 As UW             '/*30-31*/     Analog input 3.
        Public input_analog_4 As UW             '/*32-33*/     Analog input 4.
        Public input_analog_5 As UW             '/*34-35*/     Analog input 5.
        Public input_analog_6 As UW             '/*36-37*/     Analog input 6.
        Public input_analog_7 As UW             '/*38-39*/     Analog input 7.

        Public output_bank_0 As UW              '/*40-41*/     Digital outputs 0-15;
        Public output_bank_1 As UW              '/*42-43*/     Digital outputs 16-23;

        Public input_bank_0 As UW               '/*44-45*/     Digital inputs 0-15;
        Public input_bank_1 As UW               '/*46-47*/     Digital inputs 16-23;

        Public pulse_count_0 As UL              '/*48-51*/     Pulse counter (see PC).
        Public zc_variable As SL                '/*52-55*/     ZC User-defined variable (see ZC).
        Public zd_variable As SL                '/*56-59*/     ZD User-defined variable (see ZD).

        Public encoder_0 As SL                  '/*60-63*/     Encoder channel 0. Data only valid for parts with -BISS, -QUAD, or -SSI.
        Public encoder_1 As SL                  '/*64-67*/     Encoder channel 1. Data only valid for parts with -BISS, -QUAD, or -SSI.
        Public encoder_2 As SL                  '/*68-71*/     Encoder channel 2. Data only valid for parts with -BISS, -QUAD, or -SSI.
        Public encoder_3 As SL                  '/*72-75*/     Encoder channel 3. Data only valid for parts with -BISS, -QUAD, or -SSI.

        Public Function byte_array() As Byte() Implements GDataRecord.byte_array
            Return StructToByteArray(Me)
        End Function
    End Structure

    ' Data record struct for RIO-47300 with 24EX I/O daughter board.
    <StructLayout(LayoutKind.Sequential, Pack:=1)>
    Public Structure GDataRecord47300_24EX
        Implements GDataRecord

        Public header_0 As UB                   '/*00*/        1st Byte of Header.
        Public header_1 As UB                   '/*01*/        2nd Byte of Header.
        Public header_2 As UB                   '/*02*/        3rd Byte of Header.
        Public header_3 As UB                   '/*03*/        4th Byte of Header.

        Public sample_number As UW              '/*04-05*/     Sample number.
        Public error_code As UB                 '/*06*/        Error code.
        Public general_status As UB             '/*07*/        General status.

        Public output_analog_0 As UW            '/*08-09*/     Analog output 0.
        Public output_analog_1 As UW            '/*10-11*/     Analog output 1.
        Public output_analog_2 As UW            '/*12-13*/     Analog output 2.
        Public output_analog_3 As UW            '/*14-15*/     Analog output 3.
        Public output_analog_4 As UW            '/*16-17*/     Analog output 4.
        Public output_analog_5 As UW            '/*18-19*/     Analog output 5.
        Public output_analog_6 As UW            '/*20-21*/     Analog output 6.
        Public output_analog_7 As UW            '/*22-23*/     Analog output 7.

        Public input_analog_0 As UW             '/*24-25*/     Analog input 0.
        Public input_analog_1 As UW             '/*26-27*/     Analog input 1.
        Public input_analog_2 As UW             '/*28-29*/     Analog input 2.
        Public input_analog_3 As UW             '/*30-31*/     Analog input 3.
        Public input_analog_4 As UW             '/*32-33*/     Analog input 4.
        Public input_analog_5 As UW             '/*34-35*/     Analog input 5.
        Public input_analog_6 As UW             '/*36-37*/     Analog input 6.
        Public input_analog_7 As UW             '/*38-39*/     Analog input 7.

        Public output_bank_0 As UW              '/*40-41*/     Digital outputs 0-15.
        Public output_bank_1 As UW              '/*42-43*/     Digital outputs 16-23.

        Public input_bank_0 As UW               '/*44-45*/     Digital inputs 0-15.
        Public input_bank_1 As UW               '/*46-47*/     Digital inputs 16-23.

        Public pulse_count_0 As UL              '/*48-51*/     Pulse counter (see PC)8.
        Public zc_variable As SL                '/*52-55*/     ZC User-defined variable (see ZC).
        Public zd_variable As SL                '/*56-59*/     ZD User-defined variable (see ZD).

        Public output_bank_2 As UW              '/*60-61*/     Digital outputs 24-39. Data only valid for parts with 24EXOUT.
        Public output_back_3 As UW              '/*62-63*/     Digital outputs 40-47. Data only valid for parts with 24EXOUT.

        Public input_bank_2 As UW               '/*64-65*/     Digital inputs 24-39. Data only valid for parts with 24EXIN.
        Public input_bank_3 As UW               '/*66-67*/     Digital inputs 40-47. Data only valid for parts with 24EXIN.

        Public Function byte_array() As Byte() Implements GDataRecord.byte_array
            Return StructToByteArray(Me)
        End Function
    End Structure

    'Data record struct for RIO-47162.
    <StructLayout(LayoutKind.Sequential, Pack:=1)>
    Public Structure GDataRecord47162
        Implements GDataRecord

        Public header_0 As UB                   '/*00*/        1st Byte of Header.
        Public header_1 As UB                   '/*01*/        2nd Byte of Header.
        Public header_2 As UB                   '/*02*/        3rd Byte of Header.
        Public header_3 As UB                   '/*03*/        4th Byte of Header.

        Public sample_number As UW              '/*04-05*/     Sample number.
        Public error_code As UB                 '/*06*/        Error code.
        Public general_status As UB             '/*07*/        General status.

        Public output_analog_0 As UW            '/*08-09*/     Analog output 0.
        Public output_analog_1 As UW            '/*10-11*/     Analog output 1.
        Public output_analog_2 As UW            '/*12-13*/     Analog output 2.
        Public output_analog_3 As UW            '/*14-15*/     Analog output 3.
        Public output_analog_4 As UW            '/*16-17*/     Analog output 4.
        Public output_analog_5 As UW            '/*18-19*/     Analog output 5.
        Public output_analog_6 As UW            '/*20-21*/     Analog output 6.
        Public output_analog_7 As UW            '/*22-23*/     Analog output 7.

        Public input_analog_0 As UW             '/*24-25*/     Analog input 0.
        Public input_analog_1 As UW             '/*26-27*/     Analog input 1.
        Public input_analog_2 As UW             '/*28-29*/     Analog input 2.
        Public input_analog_3 As UW             '/*30-31*/     Analog input 3.
        Public input_analog_4 As UW             '/*32-33*/     Analog input 4.
        Public input_analog_5 As UW             '/*34-35*/     Analog input 5.
        Public input_analog_6 As UW             '/*36-37*/     Analog input 6.
        Public input_analog_7 As UW             '/*38-39*/     Analog input 7.

        Public output_byte_0 As UB              '/*40*/        Digital outputs 0-7.
        Public output_byte_1 As UB              '/*41*/        Digital outputs 8-15.
        Public output_byte_2 As UB              '/*42*/        Digital outputs 16-23.

        Public input_byte_0 As UB               '/*43*/        Digital inputs 0-7.
        Public input_byte_1 As UB               '/*44*/        Digital inputs 8-15.
        Public input_byte_2 As UB               '/*45*/        Digital inputs 16-23.
        Public input_byte_3 As UB               '/*46*/        Digital inputs 24-31.
        Public input_byte_4 As UB               '/*47*/        Digital inputs 32-39.

        Public pulse_count_0 As UL              '/*48-51*/     Pulse counter (see PC).
        Public zc_variable As SL                '/*52-55*/     ZC User-defined variable (see ZC).
        Public zd_variable As SL                '/*56-59*/     ZD User-defined variable (see ZD).

        Public encoder_0 As SL                  '/*60-63*/     Encoder channel 0. Data only valid for parts with -BISS, -QUAD, or -SSI.
        Public encoder_1 As SL                  '/*64-67*/     Encoder channel 1. Data only valid for parts with -BISS, -QUAD, or -SSI.
        Public encoder_2 As SL                  '/*68-71*/     Encoder channel 2. Data only valid for parts with -BISS, -QUAD, or -SSI.
        Public encoder_3 As SL                  '/*72-75*/     Encoder channel 3. Data only valid for parts with -BISS, -QUAD, or -SSI.

        Public Function byte_array() As Byte() Implements GDataRecord.byte_array
            Return StructToByteArray(Me)
        End Function
    End Structure
#End Region

End Class
