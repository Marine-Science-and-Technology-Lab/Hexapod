/** @addtogroup cs_examples
  * @{
  */
  
/*! \file position_tracking_example.cs
*
* See \link examples.Examples.Position_Tracking() Position_Tracking() \endlink for implementation of logic
* <br><br>For VB.NET, see definition in file position_tracking_example.vb
*/
using System;
using System.Linq;

namespace examples
{
    /** @addtogroup cs_examples
    * @{
    */
	/// <summary>
	/// Places controller into position tracking mode.  Accepts user-defined positional values 
	/// at the command line.
	/// </summary>
	/// <remarks>The first argument should be the IP Address of a Galil controller.\n
	///	The second argument is optional and defines the default speed of the controller in
	/// Position Tracking mode.</remarks>
	/*! For VB.NET, see definition in file position_tracking_example.vb */
    class Position_Tracking_Example
    {
        /// <summary>
        /// Main function for the position tracking example.
        /// </summary>
        /// <param name="args">An array of command line arguments.</param>
        /// <returns>The success status or error code of the function.</returns>
        /// <remarks>The first argument should be the IP Address of a Galil controller.
		///	The second argument is optional and defines the default speed of the controller in
		/// Position Tracking mode.</remarks>
        public static int Main(string[] args)
        {
            int rc = Examples.GALIL_EXAMPLE_OK;
            gclib gclib = new gclib();

            try
            {
                int speed = 0;
                
                if(args.Count() == 1)
                {
                    speed = 5000;
                }
                else if(args.Count() == 2)
                {
                    bool ok = int.TryParse(args[1], out speed);

                    if(!ok)
                    {
                        Console.WriteLine("An invalid speed was entered.  " +
                                          "Please enter a valid integer for speed.");
                        Console.WriteLine("Usage: position_tracking_example.exe <ADDRESS> <SPEED=5000>");

                        Console.Write("\nPress any key to close the example");
                        Console.ReadKey();
                        return Examples.GALIL_EXAMPLE_ERROR;
                    }
                }
                else
                {
                    Console.WriteLine("Incorrect number of arguments provided");
                    Console.WriteLine("Usage: position_tracking_example.exe <ADDRESS> <SPEED=5000>");
					
                    Console.Write("\nPress any key to close the example");
                    Console.ReadKey();					
                    return Examples.GALIL_EXAMPLE_ERROR;
                }

                string address = args[0]; //Retrieve address from command line
                gclib.GOpen(address); // Open a connection at the provided address

                rc = Examples.Position_Tracking(gclib, speed); // Begin position tracking mode
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