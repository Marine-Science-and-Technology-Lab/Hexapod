/** @addtogroup cs_examples
  * @{
  */
  
/*! \file record_position_example.cs
*
* See \link examples.Examples.Record_Position() Record_Position() \endlink for implementation of logic
* <br><br>For VB.NET, see definition in file record_position_example.vb
*/
using System;
using System.Linq;

namespace examples
{
    /** @addtogroup cs_examples
    * @{
    */
	/// <summary>
	/// Takes two file paths at the command line to hold positional data for Axis A and Axis B.  Positional
	/// data is saved to the two files until an analog input value changes.
	/// </summary>
	/// <remarks>The first argument should be the IP Address of a Galil controller.\n
	///	The second argument should be a path to a file to save Axis A positional data.\n
	///	The third argument should be a path to a file to save Axis B positional data.\n
	///	</remarks>
	/*! For VB.NET, see definition in file record_position_example.vb */
    class Record_Position_Example
    {
        /// <summary>
        /// Main function for the Record Position example. 
        /// </summary>
        /// <param name="args">An array of command line arguments.</param>
        /// <returns>The success status or error code of the function.</returns>
        /// <remarks>The first argument should be the IP Address of a Galil controller.\n
		///	The second argument should be a path to a file to save Axis A positional data.\n
		///	The third argument should be a path to a file to save Axis B positional data.\n
		///	</remarks>
        public static int Main(string[] args)
        {
            int rc = Examples.GALIL_EXAMPLE_OK;
            gclib gclib = new gclib();

            try
            {
                if(args.Count() != 3)
                {
                    Console.WriteLine("Incorrect number of arguments provided");
                    Console.WriteLine("Usage: record_position_example.exe <ADDRESS> <FILE A> <FILE B>");

                    Console.Write("\nPress any key to close the example");
                    Console.ReadKey();
                    return Examples.GALIL_EXAMPLE_ERROR;
                }

                string address = args[0]; //Retrieve address from command line
                string fileA = args[1]; //Retrieve filepath from command line
                string fileB = args[2]; //Retrieve filepath from command line

                gclib.GOpen(address); //Opens connection at the provided address

                //Record user's training and saves to a text file
                rc = Examples.Record_Position(gclib, fileA, fileB);
            }
            catch(Exception ex)
            {
                Examples.PrintError(gclib, ex);
                return Examples.GALIL_EXAMPLE_ERROR;
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