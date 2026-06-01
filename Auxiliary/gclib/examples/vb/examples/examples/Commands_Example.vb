''' <summary>
''' Demonstrates various uses of GCommand() And basic controller queries.
''' </summary>
''' <remarks>The first argument should be the IP Address of a Galil controller.</remarks>
Module Commands_Example

    ''' <summary>
    ''' Main function for the commands example.
    ''' </summary>
    ''' <returns>The success status Or error code of the function.</returns>
    ''' <remarks>The first argument should be the IP Address of a Galil controller.</remarks>
    Function Main() As Integer
        Dim rc As Integer = Examples.GALIL_EXAMPLE_OK
        Dim gclib As Gclib = New Gclib()
        Dim args() As String = Environment.GetCommandLineArgs()
        Try
            If args.Count() <> 2 Then
                Console.WriteLine("Incorrect number of arguments provided")
                Console.WriteLine("Usage: commands_example.exe <ADDRESS>")

                Console.Write(vbCrLf + "Press any key to close the example.")
                Console.ReadKey()
                Return Examples.GALIL_EXAMPLE_ERROR
            End If

            Dim address As String = args(1)

            gclib.GOpen(address)

            rc = Examples.Commands(gclib)
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