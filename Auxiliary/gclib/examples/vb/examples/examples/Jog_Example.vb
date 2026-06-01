'/**
' The class CMyClass is a sample class for Doxygen documentation.
'
' @author some weired guy
' @version 1.0 first documented version
'**/
Module Jog_Example

    Function Main() As Integer
        Dim rc As Integer = Examples.GALIL_EXAMPLE_OK
        Dim gclib As Gclib = New Gclib()
        Dim args() As String = Environment.GetCommandLineArgs()
        Try
            If args.Count() <> 2 Then
                Console.WriteLine("Incorrect number of arguments provided")
                Console.WriteLine("Usage: jog_example.exe <ADDRESS>")

                Console.Write(vbCrLf + "Press any key to close the example.")
                Console.ReadKey()
                Return Examples.GALIL_EXAMPLE_ERROR
            End If

            Dim address As String = args(1)

            gclib.GOpen(address)

            rc = Examples.Jog(gclib)
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