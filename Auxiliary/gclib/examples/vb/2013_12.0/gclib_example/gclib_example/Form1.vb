''' <summary>
''' Demonstrates using gclib in a Windows Form, including using a second thread to free the GUI.
''' </summary>
Public Class MainForm

#Region "UI"

    'Runs when form loads
    Private Sub MainForm_Load(sender As Object, e As EventArgs) Handles MyBase.Load
        Print("Enter a FULL GOpen() address above and click Go", PrintStyle.Instruction)
        Print("NOTE: This demo will attempt to move Axis A", PrintStyle.Instruction)
    End Sub

    'Opens Galil's help to show GOpen() options
    Private Sub HelpLabel_Click(sender As Object, e As EventArgs) Handles HelpLabel.Click
        'link to GOpen() documentation.
        System.Diagnostics.Process.Start("http://www.galil.com/sw/pub/all/doc/gclib/html/gclib_8h_aef4aec8a85630eed029b7a46aea7db54.html#aef4aec8a85630eed029b7a46aea7db54")
    End Sub

    'Runs when user clicks Go button
    Private Sub GoButton_Click(sender As Object, e As EventArgs) Handles GoButton.Click
        If AddressTextBox.Text.Length = 0 Then
            Print("Enter a FULL GOpen() address above and click Go", PrintStyle.Instruction)
            Return
        End If
        RunDemo(AddressTextBox.Text)
    End Sub

    'Various print styles.
    Private Enum PrintStyle
        Instruction
        Normal
        GalilData
        GclibData
        Err
    End Enum

    ''' <summary>
    ''' Thread safe printing call.
    ''' </summary>
    ''' <param name="Message">The message to print</param>
    ''' <param name="Style">The style enum to indicate how to print</param>
    ''' <param name="SuppressCrLf">If true, the string will be printed without a trailing cr+lf</param>
    Private Sub Print(Message As String, Optional Style As PrintStyle = PrintStyle.Normal, Optional SuppressCrLf As Boolean = False)
        If Output.InvokeRequired Then 'tests if call is coming from another thread
            Output.Invoke(New Printer(AddressOf Print), Message, Style, SuppressCrLf) 'invoke the call on the correct thread
        Else 'we're on the right thread, just print
            Dim color As Color
            Select Case Style
                Case PrintStyle.Instruction
                    color = Drawing.Color.Black
                Case PrintStyle.GalilData
                    color = Drawing.Color.Green
                Case PrintStyle.Normal
                    color = Drawing.Color.Blue
                Case PrintStyle.Err
                    color = Drawing.Color.Red
                Case PrintStyle.GclibData
                    color = Drawing.Color.Magenta
                Case Else
                    color = Drawing.Color.Blue
            End Select
            Output.SelectionStart = Output.Text.Length
            Output.SelectionColor = color
            Output.AppendText(Message)
            If Not SuppressCrLf Then
                Output.AppendText(vbCrLf)
            End If
        End If 'invoke check
    End Sub
#End Region

#Region "Threading"
    ''' <summary>
    ''' Delegate used to print status when the status is generated in a thread other than the UI thread.
    ''' </summary>
    ''' <param name="Message">Message to print</param>
    ''' <param name="Style">Print Style</param>
    ''' <param name="SuppressCrLf">If true, the string will be printed without a trailing cr+lf</param>
    Private Delegate Sub Printer(Message As String, Style As PrintStyle, SuppressCrLf As Boolean)

    ''' <summary>
    ''' Fires up the demo via the background worker thread
    ''' </summary>
    ''' <param name="address">The full GOpen() addresss</param>
    ''' <remarks>Runs in UI thread</remarks>
    Private Sub RunDemo(address As String)
        MainToolStrip.Enabled = False
        Output.Clear()
        GclibBackgroundWorker.RunWorkerAsync(address)
    End Sub

    ''' <summary>
    ''' Runs in second thread to call the demo.
    ''' </summary>
    Private Sub GclibBackgroundWorker_DoWork(sender As Object, e As System.ComponentModel.DoWorkEventArgs) Handles GclibBackgroundWorker.DoWork
        Print("Running Demo with address " & e.Argument, PrintStyle.Normal)
        TheDemo(e.Argument) 'call the actual demo code
    End Sub

    ''' <summary>
    ''' Runs in the main thread after the second thread returns.
    ''' </summary>
    Private Sub GclibBackgroundWorker_RunWorkerCompleted(sender As Object, e As System.ComponentModel.RunWorkerCompletedEventArgs) Handles GclibBackgroundWorker.RunWorkerCompleted
        Print("Demo thread done.", PrintStyle.Normal)
        MainToolStrip.Enabled = True
    End Sub
#End Region

#Region "Demo Code"

    ''' <summary>
    ''' Runs in a different thread than the UI, allowing the UI to stay active
    ''' </summary>
    ''' <param name="address">The full GOpen() addresss</param>
    Private Sub TheDemo(address As String)

        Dim gclib As Gclib = Nothing 'keep gclib calls all in one thread.
        Try
            gclib = New Gclib 'constructor can throw, so keep it in a Try block
            Print("gclib version: ", PrintStyle.Normal, True)
            Print(gclib.GVersion, PrintStyle.GclibData)

            '*** Uncomment below for network utilities ***
            'Print("Controllers requesting IP addresses...")
            'Dim macs As String() = gclib.GIpRequests()
            'If macs.Length = 0 Then
            '    Print("None")
            'Else
            '    For Each m As String In macs
            '        Print(m, PrintStyle.GclibData)
            '    Next
            'End If

            'gclib.GAssign("192.168.0.42", "00:50:4c:20:01:23") 'Assign an IP to an unassigned controller

            Print("Available connections:")
            Dim addrs As String() = gclib.GAddresses()
            If addrs.Length = 0 Then
                Print("None")
            Else
                For Each a As String In addrs
                    Print(a, PrintStyle.GclibData)
                Next
            End If

            Print("Opening connection to """ & address & """... ", PrintStyle.Normal, True)
            gclib.GOpen(address)
            Print("Connected.", PrintStyle.Normal)
            Print(gclib.GInfo(), PrintStyle.GalilData)

            'gclib.GCommand("BN") 'send BN if IP address was assigned above

            Print("Sending ""MG TIME""", PrintStyle.Normal)
            Print(gclib.GCommand("MG TIME", False), PrintStyle.GalilData)

            Print("Downloading Program... ", , True)
            gclib.GProgramDownload("i=0" & vbCr & "#A;MG i{N};i=i+1;WT10;JP#A,i<10;EN", "")

            Print("Uploading Program")
            Print(gclib.GProgramUpload(), PrintStyle.GalilData)

            Print("Blocking GMessage call")
            gclib.GCommand("XQ")
            System.Threading.Thread.Sleep(200) 'wait a bit to queue up some messages
            Print(gclib.GMessage(), PrintStyle.GalilData) 'get them all in one blocking read

            Print("Downloading Program... ", , True)
            gclib.GProgramDownload("WT 1000; MG TIME; EN", "") 'prints a messsage after 1 second

            Print("Uploading Program")
            Print(gclib.GProgramUpload(), PrintStyle.GalilData)

            Print("Non-blocking GMessage call", , True)
            gclib.GCommand("XQ")
            gclib.GTimeout(0) 'set a zero timeout for a non-blocking read
            Dim msg As String = ""
            While (msg = "")
                msg = gclib.GMessage()
                Print(".", PrintStyle.Normal, True)
                System.Threading.Thread.Sleep(20) 'do something useful here...
            End While
            Print("Message: ", PrintStyle.Normal, True)
            Print(msg.Trim(), PrintStyle.GalilData)
            gclib.GTimeout(-1) 'put the timeout back
            'NOTE: Both GRecord and GInterrupt also have non-blocking mode with 0 timeout.

            Print("Downloading Program... ", , True)
            gclib.GProgramDownload("WT 1000; UI 8; EN", "") 'fires an interrupt after 1 second

            Print("Uploading Program")
            Print(gclib.GProgramUpload(), PrintStyle.GalilData)

            Print("Non-blocking GInterrupt call", , True)
            gclib.GCommand("XQ")
            gclib.GTimeout(0) 'set a zero timeout for a non-blocking read
            Dim b As Byte = 0
            While (b = 0)
                b = gclib.GInterrupt()
                Print(".", PrintStyle.Normal, True)
                System.Threading.Thread.Sleep(20) 'do something useful here...
            End While
            Print("Byte: ", PrintStyle.Normal, True)
            Print(b.ToString("X02"), PrintStyle.GalilData)
            gclib.GTimeout(-1) 'put the timeout back

            Print("Getting some synchronous data records")
            Dim DataRecord As Gclib.GDataRecord4000
            For i = 0 To 10
                DataRecord = gclib.GRecord(Of Gclib.GDataRecord4000)(False)
                Print(DataRecord.sample_number.ToString() & " ", PrintStyle.GalilData, True) 'byte 4 and 5 are typically TIME counter
                'need help accessing the data record? Contact softwaresupport@galil.com
                System.Threading.Thread.Sleep(10)
            Next
            Print("")

            Print("Getting some asynchronous data records")
            gclib.GRecordRate(10) 'set up data records every 10 ms
            For i = 0 To 10
                DataRecord = gclib.GRecord(Of Gclib.GDataRecord4000)(True)
                Print(DataRecord.sample_number.ToString() & " ", PrintStyle.GalilData, True) 'byte 4 and 5 are typically TIME counter
                'no need to delay, asynchronous mode is dispatched by the Galil's RTOS.
            Next
            gclib.GRecordRate(0) 'turn off data records
            Print("")

            Print("Downloading an array... ", , True)
            Dim array As New List(Of Double)
            For i As Double = 0 To 9
                array.Add(i * 2)
            Next
            gclib.GCommand("DA *[];DM array[10]") 'arrays must be dimensioned prior to download
            gclib.GArrayDownload("array", array)

            Print("Ok. Uploading array")
            array = gclib.GArrayUpload("array")
            For Each d As Double In array
                Print(d.ToString("F4") & " ", PrintStyle.GalilData, True)
            Next
            Print("")

            Print("Performing a write... ", , True)
            gclib.GWrite("QR" & vbCr) 'QR returns the binary data record
            Print("Ok. Reading binary data... ", , True)
            Dim data As Byte() = gclib.GRead()
            Print("Ok. Read " & data.Length() & " bytes.")

            Print("Preparing A axis. This could cause errors if the axis is not initialized...", , True)
            gclib.GCommand("AB;MO;SHA") 'compound commands are possible though typically not recommended
            Print("Ok")
            gclib.GCommand("PRA=5000")
            gclib.GCommand("SPA=5000")
            Print("Profiling a move on axis A... ", , True)
            gclib.GCommand("BGA")
            Print("Waiting for motion to complete... ", , True)
            gclib.GMotionComplete("A")
            Print("done")
            Print("Going back... ", , True)
            gclib.GCommand("PRA=-5000")
            gclib.GCommand("BGA")
            gclib.GMotionComplete("A")
            Print("done")
        Catch ex As Exception
            Print("Error: " & ex.Message, PrintStyle.Err)
        Finally
            If Not gclib Is Nothing Then
                gclib.GClose() 'don't forget to close connections!
            End If
        End Try
    End Sub
#End Region

End Class
