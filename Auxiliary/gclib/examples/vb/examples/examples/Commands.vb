Partial Public Module Examples
    ''' <summary>
    ''' Demonstrates various uses of GCommand() And basic controller queries.
    ''' </summary>
    ''' <param name="gclib">A gclib object with a valid connection.</param>
    ''' <returns>The success status Or error code of the function.</returns>
    ''' <remarks>See commands_example.cs for an example.</remarks>
    Function Commands(gclib As Gclib) As Integer
        Console.WriteLine("*****************************************************************************")
        Console.WriteLine("***********************    GCommand Trimmed example   ***********************")
        Console.WriteLine("*****************************************************************************")

        Console.WriteLine("GCommand(""PR ?,? "", true) will return a trimmed response of GCommand()")
        Console.WriteLine("The command 'PR ?,?' will return the relative " +
                            "position of the A and B axes")
        Console.WriteLine("<<PR ?,? with no trim: " + gclib.GCommand("PR ?,?", False) + ">>")
        Console.WriteLine("<<PR ?,? with trim: " + gclib.GCommand("PR ?,?", True) + ">>")

        Console.WriteLine("*****************************************************************************")
        Console.WriteLine("*************************    GCommand Int example   *************************")
        Console.WriteLine("*****************************************************************************")

        Console.WriteLine("Use GCmdI() to retrieve the value of GCommand as an int.")
        Console.WriteLine("The command 'MG _LMS' will return the available " +
                            "space in the vector buffer of the S plane.")

        Console.WriteLine("MG _LMS with GCmdI(): " + gclib.GCmdI("MG _LMS").ToString())

        Console.WriteLine("*****************************************************************************")
        Console.WriteLine("***********************    GCommand Double example   ************************")
        Console.WriteLine("*****************************************************************************")

        Console.WriteLine("Use GCmdD() to retrieve the value of GCommand as a double.")
        Console.WriteLine("The command 'MG @AN[1]' will return the value of Analog Input 1")

        Console.WriteLine("MG @AN[1] with GCmdD(): " + gclib.GCmdD("MG @AN[1]").ToString())

        Return Examples.GALIL_EXAMPLE_OK
    End Function
End Module
