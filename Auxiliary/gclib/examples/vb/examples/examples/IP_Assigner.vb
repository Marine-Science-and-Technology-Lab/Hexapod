Partial Public Module Examples
    Public Function IP_Assigner(gclib As Gclib, serial_num As String, address As Byte) As Integer
        Dim controller_found As Boolean = False
        Dim requests() As String

        Do 'Loop while no requests are found.
            Console.WriteLine("Searching...")

            'Listen for ~5 secods for controllers requesting IP addresses.
            requests = gclib.GIpRequests()

            For Each request In requests
                Console.WriteLine(request)
            Next request
        Loop While (requests.Count() < 1)

        For Each request In requests
            Dim controller_params() As String = request.Split({", "}, StringSplitOptions.None)

            'Parameters are ordered as
            '[Model #], [Serial #], [MAC Address], [Connection Name], [IP Address]

            If (controller_params.Count() < 5) Then
                Console.WriteLine("Unexpected controller format")
                Return GALIL_EXAMPLE_ERROR
            End If

            Dim mac As String = controller_params(2)
            Dim ip As String = controller_params(4)

            'If controller contains the user entered serial number
            If (serial_num = controller_params(1)) Then
                Console.WriteLine("Controller Match Found")
                controller_found = True

                'Splits the found ip address into individual bytes
                Dim ip_bytes() As String = ip.Split(".")

                'Rebuild the ip address using the user provided address as the last byte
                Dim new_ip = $"{ip_bytes(0)}.{ip_bytes(1)}.{ip_bytes(2)}.{address}"

                'Assign the New ip address to the controller
                gclib.GAssign(new_ip, mac)

                'Open a connection at the New ip address
                gclib.GOpen(new_ip)

                'Burns the newly assigned ip address to non-volatile EEPROM memory
                gclib.GCommand("BN")

                Console.WriteLine("IP Address assigned")

                'Write the connection string to the console
                Console.WriteLine(gclib.GInfo())

                Exit For
            End If
        Next request

        If Not controller_found Then
            Console.Write("No controller matched the entered serial number")
        End If

        Return GALIL_EXAMPLE_OK

    End Function
End Module
