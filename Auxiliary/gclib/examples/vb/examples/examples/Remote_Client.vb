Partial Public Module Examples
    ''' <summary>
    ''' Demonstrates various uses of GListServers() and GSetServer()
    ''' </summary>
    ''' <returns>The success status Or error code of the function.</returns>
    ''' <remarks>See remote_client_example.cs for an example.</remarks>
    Function Remote_Client() As Integer
        Dim gclib = New Gclib()

        Console.WriteLine("<s> List available servers on the network" + vbNewLine +
                            "<h> List available hardware on currently connected server" + vbNewLine +
                            "<0-9> Enter numbers 0-9 to connect to a server by index" + vbNewLine +
                            "<l> Set active server back to local server" + vbNewLine +
                            "<q> Quit")

        Dim loop_bool = True
        Dim servers_list As String() = Array.Empty(Of String)

        While loop_bool
            Dim input As Char = Console.ReadKey(True).KeyChar

            If input = "q" Then
                loop_bool = False
            ElseIf input = "s" Then
                Console.WriteLine("Available Servers:")
                servers_list = gclib.GListServers()
                Print_Servers_List(servers_list)
            ElseIf input >= "0" And input <= "9" Then
                Dim index As Integer = Convert.ToInt16(input) - Convert.ToInt16("0"c)
                If servers_list.Length > 0 And index < servers_list.Length Then
                    gclib.GSetServer(servers_list(index))
                    Console.WriteLine("Server set to: " + servers_list(index))
                End If
            ElseIf input = "l" Then
                gclib.GSetServer("Local")
                Console.WriteLine("Server set to: Local")
            ElseIf input = "h" Then
                Dim addresses As String() = gclib.GAddresses()

                For Each address As String In addresses
                    Console.WriteLine(address)
                Next
            End If
        End While

        Return Examples.GALIL_EXAMPLE_OK
    End Function

    Private Sub Print_Servers_List(servers_list As String())
        If servers_list.Length = 0 Then
            Console.WriteLine("none")
        Else
            For i As Integer = 0 To servers_list.Length - 1
                Console.WriteLine("<" + i.ToString() + "> " + servers_list(i))
            Next
        End If

    End Sub
End Module
