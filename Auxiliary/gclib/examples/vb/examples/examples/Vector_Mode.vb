Partial Public Module Examples
    Public Function Vector_Mode(gclib As Gclib, file As String) As Integer
        gclib.GCommand("ST") 'Stop all motors
        gclib.GCommand("SH AB") 'Set servo here
        gclib.GCommand("DP 0,0") 'Start position at absolute zero

        gclib.GCommand("CAS") 'Defines S As active coordinate system
        gclib.GCommand("VS 20000") 'Defines vector speed
        gclib.GCommand("VA 200000") 'Defines vector acceleration
        gclib.GCommand("VD 200000") 'Defines vector decelerlation
        gclib.GCommand("VM AB") 'Begin vector segment

        Using reader As System.IO.StreamReader = New System.IO.StreamReader(file)
            'Stores the available space of the vector buffer in the capacity variable
            Dim capacity = gclib.GCmdI("MG _LMS")
            Load_Buffer(gclib, reader, capacity)

            Console.WriteLine("Begin Motion")
            gclib.GCommand("BG S")

            Do 'Load buffer with more commands
                System.Threading.Thread.Sleep(100)

                'Stores the available space of the vector buffer in the capacity variable
                capacity = gclib.GCmdI("MG _LMS")
            Loop While Load_Buffer(gclib, reader, capacity)

            gclib.GCommand("VE") 'Segment End
            gclib.GMotionComplete("S")
            Console.WriteLine("Motion Complete")

            Return GALIL_EXAMPLE_OK
        End Using
    End Function

    Private Function Load_Buffer(gclib As Gclib, reader As System.IO.StreamReader, capacity As Integer) As Boolean
        Dim s_cmd As String = ""
        'Fully load the vector buffer leaving room for one VE command
        For i = capacity To 1 Step -1
            'If there Is another line of the text file
            If reader.Peek() >= 0 Then
                s_cmd = reader.ReadLine()
                'Run the command on each line of the text file
                gclib.GCommand(s_cmd)
            Else
                Return False
            End If
        Next

        Return True
    End Function
End Module
