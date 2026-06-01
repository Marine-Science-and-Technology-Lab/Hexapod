''' <summary>
''' Provides a class of shared constants And methods for gclib's example projects.
''' </summary>
Partial Public Module Examples
    Public Const GALIL_EXAMPLE_OK As Integer = 0
    Public Const GALIL_EXAMPLE_ERROR = -100

    ''' <summary>
    ''' Prints the exception to the console And queries the controller for the most recent
    ''' error message.  
    ''' </summary>
    ''' <param name="gclib">The gclib object from where the exception originated.</param>
    ''' <param name="ex">The exception object caught by the example.</param>
    ''' <remarks>See commands_example.cs for an example.</remarks>
    Public Sub PrintError(gclib As Gclib, ex As Exception)
        Console.WriteLine(ex.Message)

        'If exception was Not a GOpen() exception, safe to query
        'the controller for a human readable error string
        If Not ex.Message.Contains("-1101") Then
            Console.WriteLine(gclib.GCommand("TC 1"))
        End If
    End Sub
End Module
