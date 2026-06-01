/** @addtogroup cs_examples
  * @{
  */
  
/*! \file contour_example.cs
*
* See \link examples.Examples.Contour() Contour() \endlink for implementation of logic
* <br><br> For VB.NET, see definition in file contour_example.vb
*/
using System;
using System.Linq;

namespace examples
{
    /** @addtogroup cs_examples
    * @{
    */
	/// <summary>
	/// Record user's training and plays back training through contour mode.
	/// </summary>
	/// <remarks>
	/// The first argument should be the IP Address of a Galil controller.\n
	/// The second argument should be a path to a csv file holding positional data for the A axis.\n
	/// The third argument should be a path to a csv file holding positional data for the B axis.
	/// </remarks>
	/*! For VB.NET, see definition in file contour_example.vb */
    class Contour_Example
    {
        /// <summary>
        /// Main function for the contour example.
        /// </summary>
        /// <param name="args">An array of command line arguments.</param>
        /// <returns>The success status or error code of the function.</returns>
        /// <remarks>
        /// The first argument should be the IP Address of a Galil controller.\n
        /// The second argument should be a path to a text file where training for Axis A will be recorded.\n
        /// The third argument should be a path to a text file where training for Axis B will be recorded..
        /// </remarks>
        public static int Main(string[] args)
        {
            gclib gclib = new gclib();
            int rc = Examples.GALIL_EXAMPLE_OK;

            if (args.Count() != 3)
            {
                Console.WriteLine("Incorrect number of arguments provided");
                Console.WriteLine("Usage: record_position_example.exe <ADDRESS> <FILE A> <FILE B>");

                Console.Write("\nPress any key to close the example");
                Console.ReadKey();
                return Examples.GALIL_EXAMPLE_ERROR;
            }

            try
            {
                string address = args[0]; //Retrieve address from command line
                string fileA = args[1]; //Retrieve filepath from command line
                string fileB = args[2]; //Retrieve filepath from command line
                
                gclib.GOpen(address); //Opens connection at the provided address

                //Record user's training and play back training through contour mode
                rc = Examples.Contour(gclib, fileA, fileB);
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