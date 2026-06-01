Partial Public Module Examples
    Public Function Motion_Complete(gclib As Gclib)
        Console.WriteLine("*************************************************************")
        Console.WriteLine("Example GInterrupt() usage")
        Console.WriteLine("*************************************************************")

        'Simple check for appropriate communication bus
        'EI will fail below if interrupts are Not supported at all
        Dim ei_support = False

        If gclib.GCommand("WH").Contains("IH") Then
            ei_support = True
        End If

        If Not ei_support Then
            Console.WriteLine("No support on this bus")
            Return GALIL_EXAMPLE_ERROR
        End If

        'Flush interrupts
        gclib.GCommand("EI0,0") 'Turn off interrupts
        gclib.GTimeout(0) 'Zero timeout

        'Flush interrupts, status will be zero when queue Is empty
        While (gclib.GInterrupt() > 0)
        End While

        gclib.GTimeout(-1) 'Restore timeout

        'Independent Motion
        gclib.GCommand("DP 0,0") 'define position zero On A And B
        Console.WriteLine("Position: " + gclib.GCommand("RP")) 'Print reference position

        gclib.GCommand("SP 4000,4000") 'Set up speed
        gclib.GCommand("AC 1280000, 1280000") 'acceleration
        gclib.GCommand("DC 1280000, 1280000") 'deceleration
        gclib.GCommand("PR 8000, 10000") 'Position Relative.  B will take longer To make its move.
        gclib.GCommand("SH AB") 'Servo Here

        Console.WriteLine("Beginning independent motion...")
        gclib.GCommand("BG AB") 'Begin motion
        Check_Interrupts(gclib, "AB") 'Block until motion Is complete On axes A And B
        Console.WriteLine("Motion Complete on A and B")
        Console.WriteLine("Position: " + gclib.GCommand("RP")) 'Print reference position

        Return GALIL_EXAMPLE_OK
    End Function

    Private Sub Check_Interrupts(gclib As Gclib, axes As String)
        'bit mask of running axes, axes arg Is trusted to provide running axes.
        'Low bit indicates running.
        Dim axis_mask As Byte = &HFF

        'iterate through all chars in axes to make the axis mask
        For i = 0 To axes.Length - 1

            'support just A-H
            Select Case (axes(i))
                Case "A"
                    axis_mask = axis_mask And &HFE
                    Exit Select
                Case "B"
                    axis_mask = axis_mask And &HFD
                    Exit Select
                Case "C"
                    axis_mask = axis_mask And &HFB
                    Exit Select
                Case "D"
                    axis_mask = axis_mask And &HF7
                    Exit Select
                Case "E"
                    axis_mask = axis_mask And &HEF
                    Exit Select
                Case "F"
                    axis_mask = axis_mask And &HDF
                    Exit Select
                Case "G"
                    axis_mask = axis_mask And &HBF
                    Exit Select
                Case "H"
                    axis_mask = axis_mask And &H7F
                    Exit Select
            End Select
        Next

        'send EI axis mask to set up interrupt events.
        gclib.GCommand("EI " + (Not axis_mask).ToString())

        Dim status As Byte

        While (axis_mask <> &HFF) 'wait for all interrupts to come in
            status = gclib.GInterrupt()
            Select Case (status)
                Case &HD0 'Axis A complete
                    axis_mask = axis_mask Or &H1
                    Exit Select
                Case &HD1 'Axis B complete
                    axis_mask = axis_mask Or &H2
                    Exit Select
                Case &HD2 'Axis C complete
                    axis_mask = axis_mask Or &H4
                    Exit Select
                Case &HD3 'Axis D complete
                    axis_mask = axis_mask Or &H8
                    Exit Select
                Case &HD4 'Axis E complete
                    axis_mask = axis_mask Or &H10
                    Exit Select
                Case &HD5 'Axis F complete
                    axis_mask = axis_mask Or &H20
                    Exit Select
                Case &HD6 'Axis G complete
                    axis_mask = axis_mask Or &H40
                    Exit Select
                Case &HD7 'Axis H complete
                    axis_mask = axis_mask Or &H80
                    Exit Select
            End Select
        End While
    End Sub
End Module
