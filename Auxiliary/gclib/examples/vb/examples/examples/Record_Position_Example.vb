Module Record_Position_Example
    Function Main() As Integer
        Dim rc As Integer = Examples.GALIL_EXAMPLE_OK
        Dim gclib As Gclib = New Gclib()
        Dim args() As String = Environment.GetCommandLineArgs()
        Try
            If args.Count <> 4 Then
                Console.WriteLine("Incorrect number of arguments provided")
                Console.WriteLine("Usage: record_position_example.exe <ADDRESS> <FILE A> <FILE B>")

                Console.Write(vbCrLf + "Press any key to close the example.")
                Console.ReadKey()
                Return Examples.GALIL_EXAMPLE_ERROR
            End If

            Dim address As String = args(1) 'Retrieve address from command line
            Dim fileA As String = args(2) 'Retrieve filepath from command line
            Dim fileB As String = args(3) 'Retrieve filepath from command line

            gclib.GOpen(address) 'Open a connection at the provided address

            rc = Examples.Record_Position(gclib, fileA, fileB) 'Begin Recording Position
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
