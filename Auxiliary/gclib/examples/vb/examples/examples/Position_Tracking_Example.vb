Module Position_Tracking_Example
    Function Main() As Integer
        Dim rc As Integer = Examples.GALIL_EXAMPLE_OK
        Dim gclib As Gclib = New Gclib()
        Dim args() As String = Environment.GetCommandLineArgs()
        Dim speed As Integer = 0
        Try
            If args.Count = 2 Then
                speed = 5000
            ElseIf args.Count = 3 Then

                Dim ok As Boolean = Byte.TryParse(args(2), speed)

                If Not ok Then
                    Console.WriteLine("An invalid speed was entered.  " +
                                      "Please enter a valid integer for speed.")
                    Console.WriteLine("Usage: position_tracking_example.exe <ADDRESS> <SPEED=5000>")

                    Console.Write(vbCrLf + "Press any key to close the example.")
                    Console.ReadKey()
                    Return Examples.GALIL_EXAMPLE_ERROR
                End If

            Else
                Console.WriteLine("Incorrect number of arguments provided")
                Console.WriteLine("Usage: position_tracking_example.exe <ADDRESS> <SPEED=5000>")
				
                Console.Write(vbCrLf + "Press any key to close the example.")
                Console.ReadKey()
                Return Examples.GALIL_EXAMPLE_ERROR
            End If


            Dim address As String = args(1) 'Retrieve address from command line
            gclib.GOpen(address) 'Open a connection at the provided address

            rc = Examples.Position_Tracking(gclib, speed) 'Begin position tracking mode
        Catch ex As Exception
            Examples.PrintError(gclib, ex)
            rc = Examples.GALIL_EXAMPLE_ERROR
        Finally
            gclib.GClose()
        End Try

        Console.Write(vbCrLf + "Press any key to close the example.")
        Console.ReadKey()

        Return rc
    End Function
End Module
