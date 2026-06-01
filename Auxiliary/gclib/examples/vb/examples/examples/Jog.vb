Partial Public Module Examples
    Public Function Jog(gclib As Gclib) As Integer
        gclib.GCommand("ST")       ' Stop all motors
        gclib.GMotionComplete("A") ' Wait For motion To complete
        gclib.GCommand("SHA")      ' Set servo here
        gclib.GCommand("DPA=0")    ' Start position at absolute zero
        gclib.GCommand("JGA=0")    ' Start jogging With 0 speed
        gclib.GCommand("BGA")      ' Begin motion On A Axis

        Dim isJogging As Boolean = True
        Dim speed As Integer = 0

        Console.WriteLine("Enter a character on the keyboard to change the" +
                            " motor's speed:" + Environment.NewLine +
                            "<q> Quit" + Environment.NewLine +
                            "<a> -2000 counts/s" + Environment.NewLine +
                            "<s> -500  counts/s" + Environment.NewLine +
                            "<d> +500  counts/s" + Environment.NewLine +
                            "<f> +2000 counts/s" + Environment.NewLine +
                            "<r> Direction Reversal" + Environment.NewLine)

        While isJogging
            gclib.GCommand("JGA=" + speed.ToString())
            Console.WriteLine("Jog Speed: " + speed.ToString())

            Select Case Console.ReadKey(True).Key
                Case ConsoleKey.Q
                    isJogging = False
                Case ConsoleKey.A
                    speed -= 2000
                Case ConsoleKey.S
                    speed -= 500
                Case ConsoleKey.D
                    speed += 500
                Case ConsoleKey.F
                    speed += 2000
                Case ConsoleKey.R
                    speed *= -1
            End Select
        End While

        gclib.GCommand("ST")
        gclib.GMotionComplete("A")

        Return GALIL_EXAMPLE_OK
    End Function
End Module
