Module IP_Assigner_Example

    Function Main() As Integer
        Dim rc As Integer = Examples.GALIL_EXAMPLE_OK
        Dim gclib As Gclib = New Gclib()
        Dim args() As String = Environment.GetCommandLineArgs()
        Try
            If args.Count() <> 3 Then
                Console.WriteLine("Incorrect number of arguments provided")
                Console.WriteLine("Usage: ip_assigner_example.exe <SERIAL #> <1 Byte Address>")

                Console.Write(vbCrLf + "Press any key to close the example.")
                Console.ReadKey()
                Return Examples.GALIL_EXAMPLE_ERROR
            End If

            Dim serial_num As String = args(1)
            Dim address As Byte
            Dim ok As Boolean = Byte.TryParse(args(2), address)

            If Not ok Then
                Console.WriteLine("Please enter a number between 0 and 255 for the address.\n" +
                    " This will be used as the last number in the IP Address\n" +
                    "Usage: ip_assigner_example.exe <SERIAL #> <1 Byte Address>")

                Console.Write(vbCrLf + "Press any key to close the example.")
                Console.ReadKey()
                Return Examples.GALIL_EXAMPLE_ERROR
            End If

            rc = Examples.IP_Assigner(gclib, serial_num, address)
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
