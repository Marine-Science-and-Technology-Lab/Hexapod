/** @addtogroup cs_examples
* @{
*/

/*! \file commands_example.cs
*
* See \link examples.Examples.Commands() Commands() \endlink for implementation of logic
* <br><br>For VB.NET, see definition in file commands_example.vb
*/
using System;
using System.Linq;

namespace examples
{
    /** @addtogroup cs_examples
    * @{
    */
	/// <summary>
	/// Demonstrates various uses of GCommand() and basic controller queries.
	/// </summary>
	/// <remarks>The first argument should be the IP Address of a Galil controller.</remarks>
	/*! For VB.NET, see definition in file commands_example.vb */
    class Commands_Example
    {
        /// <summary>
        /// Main function for the commands example.
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
                if(args.Count() != 1)
                {
                    Console.WriteLine("Incorrect number of arguments provided");
                    Console.WriteLine("Usage: commands_example.exe <ADDRESS>");

                    Console.Write("\nPress any key to close the example");
                    Console.ReadKey();
                    return Examples.GALIL_EXAMPLE_ERROR;
                }

                string address = args[0];

                gclib.GOpen(address);

                rc = Examples.Commands(gclib);
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