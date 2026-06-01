/** @addtogroup cs_examples
* @{
*/
  
/*! \file commands.cs
*
* Function calls for the Command Example Project.
* <br><br>For VB.NET, see definition in file commands.vb
*/

using System;

namespace examples
{ 
    public static partial class Examples
    {
    /** @addtogroup cs_examples
    * @{
    */
        /// <summary>
        /// Demonstrates various uses of GCommand() and basic controller queries.
        /// </summary>
        /// <param name="gclib">A gclib object with a valid connection.</param>
        /// <returns>The success status or error code of the function.</returns>
        /// <remarks>See commands_example.cs for an example.
		/// </remarks>
		/*! For VB.NET, see definition in file commands.vb */
        public static int Commands(gclib gclib)
        {
            Console.WriteLine("*****************************************************************************");
            Console.WriteLine("***********************    GCommand Trimmed example   ***********************");
            Console.WriteLine("*****************************************************************************");

            Console.WriteLine("GCommand(\"PR ?,? \", true) will return a trimmed response of GCommand()");
            Console.WriteLine("The command 'PR ?,?' will return the relative " +
                                "position of the A and B axes");
            Console.WriteLine("<<PR ?,? with no trim: " + gclib.GCommand("PR ?,?", false) + ">>");
            Console.WriteLine("<<PR ?,? with trim: " + gclib.GCommand("PR ?,?", true) + ">>");

            Console.WriteLine("*****************************************************************************");
            Console.WriteLine("*************************    GCommand Int example   *************************");
            Console.WriteLine("*****************************************************************************");

            Console.WriteLine("Use GCmdI() to retrieve the value of GCommand as an int.");
            Console.WriteLine("The command 'MG _LMS' will return the available " +
                                "space in the vector buffer of the S plane.");

            Console.WriteLine("MG _LMS with GCmdI(): " + gclib.GCmdI("MG _LMS"));

            Console.WriteLine("*****************************************************************************");
            Console.WriteLine("***********************    GCommand Double example   ************************");
            Console.WriteLine("*****************************************************************************");

            Console.WriteLine("Use GCmdD() to retrieve the value of GCommand as a double.");
            Console.WriteLine("The command 'MG @AN[1]' will return the value of Analog Input 1");

            Console.WriteLine("MG @AN[1] with GCmdD(): " + gclib.GCmdD("MG @AN[1]"));

            return GALIL_EXAMPLE_OK;
        }
/** @}*/
    }
}
/** @}*/