/** @addtogroup cs_examples
  * @{
  */
  
/*! \file jog_example.cs
*
* See \link examples.Examples.Jog() Jog() \endlink for implementation of logic
* <br><br>For VB.NET, see definition in file jog_example.vb
*/
using System;
using System.Linq;

namespace examples
{
    /** @addtogroup cs_examples
    * @{
    */
	/// <summary>
	/// Accepts user-input at the command line to control the speed of the controller in Jog mode.
	/// </summary>
	/// <remarks>The first argument should be the IP Address of a Galil controller.</remarks>
	/*! For VB.NET, see definition in file jog_example.vb */
    class Jog_Example
    {
        /// <summary>
        /// Main function for the jog example.
        /// </summary>
        /// <param name="args">An array of command line arguments.</param>
        /// <returns>The success status or error code of the function.</returns>
        /// <remarks>The first argument should be the IP Address of a Galil controller.</remarks>
        public static int Main(string[] args)
        {
            int rc = Examples.GALIL_EXAMPLE_OK;
            gclib gclib = new gclib();

            try
            {
                if (args.Count() != 1)
                {
                    Console.WriteLine("Incorrect number of arguments provided");
                    Console.WriteLine("Usage: jog_example.exe <ADDRESS>");

                    Console.Write("\nPress any key to close the example");
                    Console.ReadKey();
                    return Examples.GALIL_EXAMPLE_ERROR;
                }

                string address = args[0]; //Get IP address from the command line

                gclib.GOpen(address); //Open a connection at the provided address

                rc = Examples.Jog(gclib);
            }
            catch(Exception ex)
            {
                Examples.PrintError(gclib, ex);
                rc = Examples.GALIL_EXAMPLE_ERROR;
            }
            finally
            {
                gclib.GClose();
            }


            Console.Write("\nPress any key to close the example");
            Console.ReadKey();
            return rc;
        }
    }
/** @}*/
}
/** @}*/