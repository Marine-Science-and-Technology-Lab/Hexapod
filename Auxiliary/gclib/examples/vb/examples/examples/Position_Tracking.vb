Partial Public Module Examples
    Public Function Position_Tracking(gclib As Gclib, speed As Integer)
        Dim acc = 100 * speed ' Set acceleration/deceleration to 100 times speed

        gclib.GCommand("STA")      'Stop motor
        gclib.GMotionComplete("A") 'Wait For motion To complete
        gclib.GCommand("SHA")      'Set servo here
        gclib.GCommand("DPA=0")    'Start position at absolute zero
        gclib.GCommand("PTA=1")    'Start position tracking mode On A axis

        gclib.GCommand("SPA=" + speed.ToString()) 'Set speed            
        gclib.GCommand("ACA=" + acc.ToString())   'Set acceleration            
        gclib.GCommand("DCA=" + acc.ToString())   'Set deceleration

        Console.WriteLine("Begin Position Tracking with speed " + speed.ToString() +
                            ". Enter a non-number to exit.\n")

        Dim position As Integer

        'Loop asking user for New position.  End loop when user enters a non-number
        While True
            Console.WriteLine("Enter a new position:")
            Dim ok = Integer.TryParse(Console.ReadLine(), position)

            If ok Then ' A Then valid position was provided
                gclib.GCommand("PAA=" + position.ToString()) ' Go To New position
            Else 'A non-number was entered
                Console.WriteLine("Position Tracking has exited")
                Exit While
            End If
        End While
        gclib.GCommand("STA") 'Stop motor
        gclib.GMotionComplete("A") 'Wait For motion To complete

        Return GALIL_EXAMPLE_OK
    End Function
End Module
