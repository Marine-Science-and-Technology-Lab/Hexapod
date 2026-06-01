Partial Public Module Examples
    ''' <summary>
    ''' Demonstrates various uses of GPublishServer()
    ''' </summary>
    ''' <param name="server_name">The name to publish the server under.</param>
    ''' <returns>The success status Or error code of the function.</returns>
    ''' <remarks>See remote_server_example.cs for an example.</remarks>
    Function Remote_Server(server_name As String) As Integer
        Dim gclib = New Gclib()

        Console.WriteLine("<p> Publish this server to the network" + vbNewLine +
                        "<r> Remove this server from the network" + vbNewLine +
                        "<q> Quit")

        Dim loop_bool = True

        While loop_bool
            Select Case Console.ReadKey(True).Key
                Case ConsoleKey.Q
                    loop_bool = False
                Case ConsoleKey.P
                    gclib.GPublishServer(server_name, True, False)
                    Console.WriteLine("Published Server")
                Case ConsoleKey.R
                    gclib.GPublishServer(server_name, False, False)
                    Console.WriteLine("Removed Server")
            End Select

        End While

        Return Examples.GALIL_EXAMPLE_OK
    End Function
End Module
