Partial Public Module Examples
    Public Function Contour(gclib As Gclib, fileA As String, fileB As String) As Integer
        Record_Position(gclib, fileA, fileB) 'Record positional data on Axis A and B

        Dim positions_A = System.IO.File.ReadAllText(fileA).Split(",").ToList()
        Dim positions_B = System.IO.File.ReadAllText(fileB).Split(",").ToList()

        gclib.GCommand("SH AB") 'Set servo here
        gclib.GCommand("PA 0, 0") 'Set current position To 0
        gclib.GMotionComplete("AB") 'Wait For motion To complete
        gclib.GCommand("CM AB") 'Put axis A & B In contour mode
        gclib.GCommand("DT -1") 'Pauses contour mode To pre-load buffer
        gclib.GCommand("CD 0,0") 'Pre load buffer With zeros To prevent under buffering
        gclib.GCommand("CD 0,0") 'Pre load buffer With zeros To prevent under buffering
        gclib.GCommand("CD 0,0") 'Pre load buffer With zeros To prevent under buffering
        gclib.GCommand("DT 1") 'Sets the time interval For contour mode To be 2 samples

        Dim capacity = 0 'Holds the capacity of the contour buffer
        Dim cmd = 0 'Holds the counter for which position to send next

        If positions_A.Count <> positions_B.Count() Then
            Console.WriteLine("Error: The two datasets are not the same size")
            Return Examples.GALIL_EXAMPLE_ERROR
        End If

        Do
            'Sleep while buffer is emptying
            System.Threading.Thread.Sleep(400)

            'Stores the available space of the contour buffer in the capacity variable
            capacity = gclib.GCmdI("CM?")
        Loop While Load_Buffer(gclib, positions_A, positions_B, capacity, cmd)

        gclib.GCommand("CD 0,0=0") 'End contour mode

        Return Examples.GALIL_EXAMPLE_OK

    End Function

    Private Function Load_Buffer(gclib As Gclib, positions_A As List(Of String), positions_B As List(Of String),
                                 capacity As Integer, ByRef cmd As Integer)
        For i = capacity To 1 Step -1
            If cmd + 1 < positions_A.Count() Then
                'Subtract previous position from new position to get how for of a move to make
                Dim cdA = Double.Parse(positions_A(cmd + 1)) - Double.Parse(positions_A(cmd))

                'Subtract previous position from new position to get how for of a move to make
                Dim cdB = Double.Parse(positions_B(cmd + 1)) - Double.Parse(positions_B(cmd))

                gclib.GCommand($"CD {cdA},{cdB}")

                cmd = cmd + 1
            Else
                Return False
            End If
        Next
        Return True
    End Function
End Module
