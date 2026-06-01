''' <summary>
''' Demonstrates various uses of GListServers() and GSetServer().
''' </summary>
''' <remarks>This example requires no command line arguments.</remarks>
Module Remote_Client_Example

    ''' <summary>
    ''' Main function for the Remote Client example.
    ''' </summary>
    ''' <returns>The success status Or error code of the function.</returns>
    ''' <remarks>This example requires no command line arguments.</remarks>
    Function Main() As Integer
        Dim rc As Integer = Examples.GALIL_EXAMPLE_OK
        Try
            rc = Examples.Remote_Client()
        Catch ex As Exception
            Console.WriteLine(ex.Message)
            rc = Examples.GALIL_EXAMPLE_ERROR
        End Try

        Console.Write(vbCrLf + "Press any key to close the example.")
        Console.ReadKey()

        Return rc
    End Function

End Module