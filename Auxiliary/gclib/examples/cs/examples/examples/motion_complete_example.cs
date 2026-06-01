/** @addtogroup cs_examples
  * @{
  */
  
/*! \file motion_complete_example.cs
*
* See \link examples.Examples.Motion_Complete() Motion_Complete() \endlink for implementation of logic
* <br><br>For VB.NET, see definition in file motion_complete_example.vb
*/
using System;
using System.Linq;

namespace examples
{
    /** @addtogroup cs_examples
    * @{
    */
	/// <summary>
	/// Uses controller interrupts to detect when motion is complete.
	/// </summary>
	/// <remarks>The first argument should be the IP Address of a Galil controller.</remarks>
	/*! For VB.NET, see definition in file motion_complete_example.vb */
    class Motion_Complete_Example
    {
        /// <summary>
        /// Main function for the Motion Complete example.
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
                    Console.WriteLine("Usage: motion_complete_exmample.exe <ADDRESS>");

                    Console.Write("\nPress any key to close the example");
                    Console.ReadKey();
                    return Examples.GALIL_EXAMPLE_ERROR;
                }

                //Retrieves address from the command line
                string address = args[0];

                //Open a connection at the provided address and subcribe to event interrupts
                gclib.GOpen(address + " --subscribe EI");

                //Uses interrupts to track when the motion of the controller is completed                
                rc = Examples.Motion_Complete(gclib);
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