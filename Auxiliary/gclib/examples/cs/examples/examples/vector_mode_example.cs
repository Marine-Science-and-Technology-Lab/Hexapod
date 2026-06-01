/** @addtogroup cs_examples
  * @{
  */
  
/*! \file vector_mode_example.cs
*
* See \link examples.Examples.Vector_Mode() Vector_Mode() \endlink for implementation of logic
* <br><br>For VB.NET, see definition in file vector_mode_example.vb
*/
using System;
using System.Linq;

namespace examples
{
    /** @addtogroup cs_examples
    * @{
    */
	/// <summary>
	/// Takes a path to a file at the command line holding vector commands for the controller.
	/// The controller is placed into vector mode and commands are read from the file and sent to
	/// the controller.
	/// </summary>
	/// <remarks>The first argument should be the IP Address of a Galil controller.
	/// The second argument should be a path to a file containing vector commands.</remarks>
	/*! For VB.NET, see definition in file vector_mode_example.vb */
    class Vector_Mode_Example
    {
        /// <summary>
        /// Main function for the vector mode example.
        /// </summary>
        /// <param name="args">An array of command line arguments.</param>
        /// <returns>The success status or error code of the function.</returns>
        /// <remarks>The first argument should be the IP Address of a Galil controller.
        /// The second argument should be a path to a file containing vector commands.</remarks>
        public static int Main(string[] args)
        {
            gclib gclib = new gclib();
            int rc = Examples.GALIL_EXAMPLE_OK;

            try
            {
                if (args.Count() != 2)
                {
                    Console.WriteLine("Incorrect number of arguments provided");
                    Console.WriteLine("Usage: Vector_Example.exe <ADDRESS> <FILE>");

                    Console.Write("\nPress any key to close the example");
                    Console.ReadKey();
                    return Examples.GALIL_EXAMPLE_ERROR;
                }

                string file = args[1]; //Retrieve file from command line
                string address = args[0]; //Retrieve address from command line
                gclib.GOpen(address); //Open a connection at the provided address

                rc = Examples.Vector_Mode(gclib, file);
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