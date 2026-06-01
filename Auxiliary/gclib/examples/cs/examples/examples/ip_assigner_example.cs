/** @addtogroup cs_examples
  * @{
  */

/*! \file ip_assigner_example.cs
*
* See \link examples.Examples.IP_Assigner() IP_Assigner() \endlink for implementation of logic
* <br><br>For VB.NET, see definition in file ip_assigner_example.vb
*/
using System;
using System.Linq;

namespace examples
{
    /** @addtogroup cs_examples
    * @{
    */
	/// <summary>
	/// Assigns controller an IP Adress given a serial number and a 1 byte address.
	/// </summary>
	/// <remarks>The first argument should be the Serial # of a Galil Controller.\n
	/// The second argument should be a 1 Byte value that will be used as the final byte
	/// in the newly assigned IP Address.</remarks>
	/*! For VB.NET, see definition in file ip_assigner_example.vb */
    class IP_Assigner_Example
    {
        /// <summary>
        /// Main function for the IP Assigner example.
        /// </summary>
        /// <param name="args">An array of command line arguments.</param>
        /// <returns>The success status or error code of the function.</returns>
        /// <remarks>The first argument should be the Serial # of a Galil Controller.\n
        /// The second argument should be a 1 Byte value that will be used as the final byte
        /// in the newly assigned IP Address.</remarks>
        public static int Main(string[] args)
        {
            gclib gclib = new gclib();
            int rc = Examples.GALIL_EXAMPLE_OK;
            
            try
            {
                if(args.Count() != 2)
                {
                    Console.WriteLine("Incorrect number of arguments provided");
                    Console.WriteLine("Usage: ip_assigner_example.exe <SERIAL #> <1 Byte Address>");

                    Console.Write("\nPress any key to close the example");
                    Console.ReadKey();
                    return Examples.GALIL_EXAMPLE_ERROR;
                }

                string serial_num = args[0];
                bool ok = Byte.TryParse(args[1], out byte address);

                if(!ok || address < 0 || address > 255)
                {
                    Console.WriteLine("Please enter a number between 0 and 255 for the address.\n" +
                    " This will be used as the last number in the IP Address\n" +
                    "Usage: ip_assigner_example.exe <SERIAL #> <1 Byte Address>");

                    Console.Write("\nPress any key to close the example");
                    Console.ReadKey();
					
                    return Examples.GALIL_EXAMPLE_ERROR;
                }                    

                rc = Examples.IP_Assigner(gclib, serial_num, address);
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