Partial Public Module Examples
    Public Function Message(gclib As Gclib)
        Console.WriteLine("***************************************************************")
        Console.WriteLine("Example GMessage() usage")
        Console.WriteLine("***************************************************************")

        gclib.GCommand("TR0") 'Turn off trace

        'This program will force one message to appear as two separate packets.
        gclib.GProgramDownload("MG ""HELLO "" {N}" + vbCr +
                                    "MG ""WORLD """ + vbCr +
                                    "EN")

        gclib.GCommand("XQ") 'Begins execution Of program On controller

        Dim buf = ""
        Dim msg = ""

        'It Is important to note that a message can be too large to read in one
        'GMessage() call. Keep calling GMessage() while there are no errors to 
        'get the full message.

        'While still receiving messages
        buf = gclib.GMessage()
        While buf <> ""
            For b = 0 To buf.Length - 1 'While message characters are in the buffer

                msg += buf(b) 'Copy chars from  buffer To message

                'If the message ends in "\r\n" it Is ready to be terminated
                If (msg.Length > 2) AndAlso (msg(msg.Length - 1) = vbLf) AndAlso (msg(msg.Length - 2) = vbCr) Then
                    Console.WriteLine(msg)
                    msg = "" 'Reset message index
                    Exit While
                End If
            Next
            buf = gclib.GMessage()
        End While


        'Downloads program to the controller
        gclib.GCommand("TR1") 'Turn On trace
        gclib.GProgramDownload(
                        "i=0" + vbCr +
                        "#A" + vbCr +
                        "MGi" + vbCr +
                        "i=i+1" + vbCr +
                        "WT100" + vbCr +
                        "JP#A,i<1" + vbCr +
                        "i=i/0" + vbCr +
                        "EN")

        gclib.GCommand("XQ") 'Begins execution Of program On controller

        'Lines returned by GMessage() can be one of three types
        '1) Standard Lines begin with a space (" ")
        '2) Crashed code begins with a question mark ("?")
        '3) Trace Lines begin with a line number ("1,6,15...")

        'While still receiving messages
        buf = gclib.GMessage()
        While buf <> ""
            For b = 0 To buf.Length - 1 'While message characters are in the buffer

                msg += buf(b) 'Copy chars from buffer To message

                'If the message ends in "\r\n" its ready to be terminated
                If (msg.Length > 2) AndAlso (msg(msg.Length - 1) = vbLf) AndAlso (msg(msg.Length - 2) = vbCr) Then

                    If (msg(0) = " ") Then 'Standard Lines begin with a space (" ")
                        Console.Write("Standard Line: ")
                    ElseIf (msg(0) = "?") Then 'Crashed code begins with a question mark ("?")
                        Console.Write("Crashed Code: ")
                    Else 'Trace Lines begin with a line number ("1,6,15...")
                        Console.Write("Trace Line: ")
                    End If
                    Console.WriteLine(msg)
                    msg = ""
                End If
            Next
            buf = gclib.GMessage()
        End While

        Return Examples.GALIL_EXAMPLE_OK
    End Function
End Module
