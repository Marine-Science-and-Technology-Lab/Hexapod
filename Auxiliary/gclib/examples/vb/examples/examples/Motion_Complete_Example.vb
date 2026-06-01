Module Motion_Complete_Example
    Function Main() As Integer
        Dim rc As Integer = Examples.GALIL_EXAMPLE_OK
        Dim gclib As Gclib = New Gclib()
        Dim args() As String = Environment.GetCommandLineArgs()
        Try
            If args.Count <> 2 Then
                Console.WriteLine("Incorrect number of arguments provided")
                Console.WriteLine("Usage: motion_complete_example.exe <ADDRESS>")

                Console.Write(vbCrLf + "Press any key to close the example.")
                Console.ReadKey()
                Return Examples.GALIL_EXAMPLE_ERROR
            End If

            Dim address As String = args(1) 'Retrieve address from command line

            ' Open a connection at the provided address and subcribe to event interrupts
            gclib.GOpen(address + " --subscribe EI")

            rc = Examples.Motion_Complete(gclib)
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
