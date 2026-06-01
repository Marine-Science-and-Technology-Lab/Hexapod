Partial Public Module Examples
    Public Function Record_Position(gclib As Gclib, fileA As String, fileB As String)
        Dim writerA = New System.IO.StreamWriter(fileA, False)
        Dim writerB = New System.IO.StreamWriter(fileB, False)

        Dim recording = 1

        gclib.GProgramDownload(
            "RC 0;' Disable Recording" + vbCr +
            "DP 0, 0;' Set current position to 0" + vbCr +
            "DM posA[1000], posB[1000];' Define a new array that will hold positional data" + vbCr +
            "RA posA[], posB[];' Sets position array to be where recorded data will be stored" + vbCr +
            "RD _TPA, _TPB;' Defines Position to be the type of data that will be recorded" + vbCr +
            "RC 1,-1000;' Begins recording at 512Hz in continuous mode" + vbCr +
            "MO AB;' Turns motors off" + vbCr +
            "AI -1;' Waits for active low on Input 1" + vbCr +
            "RC 0;' Disable Recording after Input 1 goes low" + vbCr +
            "EN;' End program"
        )

        gclib.GCommand("XQ")

        Dim rd = 0
        Dim previous_rd = 0
        Dim leading_comma = False

        Console.WriteLine("Begin recording")

        Do
            System.Threading.Thread.Sleep(1000) 'Sleep while we wait for roughly half the array to be written
            rd = gclib.GCmdI("MG _RD") 'Gets address of next value in the position array

            'Get values from posA[] array and write to file
            Write_Array_To_File(gclib, writerA, "posA", previous_rd, rd, leading_comma)

            'Get values from posB[] array and write to file
            Write_Array_To_File(gclib, writerB, "posB", previous_rd, rd, leading_comma)

            leading_comma = True

            recording = gclib.GCmdI("MG _RC") 'Check status of RC

            previous_rd = rd

        Loop While recording > 0 'While recording is active

        Console.WriteLine("End recording")

        writerA.Close()
        writerB.Close()

        Return Examples.GALIL_EXAMPLE_OK
    End Function

    Private Sub Write_Array_To_File(gclib As Gclib, writer As System.IO.StreamWriter, array_name As String, previous_rd As Integer, rd As Integer, leading_comma As Boolean)
        Dim values = New List(Of Double)

        If previous_rd < rd Then 'No array wrap around
            'Grab list of doubles from controller and add it to values
            values.AddRange(gclib.GArrayUpload(array_name, previous_rd, rd - 1))
        Else
            'Grab list of doubles from controller and add it to values
            values.AddRange(gclib.GArrayUpload(array_name, previous_rd, 999))

            If rd <> 0 Then
                'Grab list of doubles from controller and add it to values
                values.AddRange(gclib.GArrayUpload(array_name, 0, rd - 1))
            End If
        End If

        For i = 0 To values.Count() - 1
            If leading_comma Then
                writer.Write(", ")
            End If

            leading_comma = True

            writer.Write(String.Format("{0:0.000}", values(i)))
        Next
    End Sub
End Module
