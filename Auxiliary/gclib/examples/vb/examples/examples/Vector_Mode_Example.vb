Module Vector_Mode_Example

    Function Main() As Integer
        Dim rc As Integer = Examples.GALIL_EXAMPLE_OK
        Dim gclib As Gclib = New Gclib()
        Dim args() As String = Environment.GetCommandLineArgs()
        Try
            If args.Count() <> 3 Then
                Console.WriteLine("Incorrect number of arguments provided")
                Console.WriteLine("Usage: vector_mode_example.exe <ADDRESS> <FILE>")

                Console.Write(vbCrLf + "Press any key to close the example.")
                Console.ReadKey()
                Return Examples.GALIL_EXAMPLE_ERROR
            End If

            Dim address As String = args(1) 'Retrieve address from command line
            Dim file = args(2) 'Retrieve file from command line

            gclib.GOpen(address)

            rc = Examples.Vector_Mode(gclib, file)
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