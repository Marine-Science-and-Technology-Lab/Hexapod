''' <summary>
''' Demonstrates various uses of GPublishServer().
''' </summary>
''' <remarks>The first argument is optional and defines the name to publish the server under.</remarks>
Module Remote_Server_Example

    ''' <summary>
    ''' Main function for the Remote Server example.
    ''' </summary>
    ''' <returns>The success status Or error code of the function.</returns>
    ''' <remarks>The first argument is optional and defines the name to publish the server under.</remarks>
    Function Main() As Integer
        Dim rc As Integer = Examples.GALIL_EXAMPLE_OK
        Dim args() As String = Environment.GetCommandLineArgs()
        Try
            Dim server_name As String

            If args.Count() <> 2 Then
                Console.Write("Enter Server Name: ")
                server_name = Console.ReadLine()
            Else
                server_name = args(1)
            End If

            rc = Examples.Remote_Server(server_name)
        Catch ex As Exception
            Console.WriteLine(ex.Message)
            rc = Examples.GALIL_EXAMPLE_ERROR
        End Try

        Console.Write(vbCrLf + "Press any key to close the example.")
        Console.ReadKey()

        Return rc
    End Function

End Module