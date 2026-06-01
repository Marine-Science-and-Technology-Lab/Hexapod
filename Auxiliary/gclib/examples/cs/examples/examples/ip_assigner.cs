/** @addtogroup cs_examples
  * @{
  */
  
/*! \file ip_assigner.cs
*
* Function calls for the IP Assigner Example Project.
* <br><br>For VB.NET, see definition in file ip_assigner.vb
*/
using System;
using System.Linq;

namespace examples
{
    public static partial class Examples
    {
        /** @addtogroup cs_examples
        * @{
        */
        /// <summary>
        /// Assigns controller an IP Adress given a serial number and a 1 byte address.
        /// </summary>
        /// <param name="gclib">A gclib object.</param>
        /// <param name="serial_num">The serial number of a Galil controller.</param>
        /// <param name="address">A 1 byte value to be added to the new IP Address.</param>
        /// <returns>The success status or error code of the function.</returns>
        /// <remarks>
        /// This function will listen on the network for controllers requesting an 
        /// IP Address.\n
        /// If a detected controller matches the serial number provided by
        /// the user, a new IP Address will be assigned based on the first 3 bytes of the
        /// detected IP Address combined with the user defined 1 byte address. \n\n
        /// See ip_assigner_example.cs for an example.
        /// </remarks>
		/*! For VB.NET, see definition in file ip_assigner.vb */
        public static int IP_Assigner(gclib gclib, string serial_num, byte address)
        {
            bool controller_found = false;
            string[] requests;
            do //Loop while no requests are found.
            {
                Console.WriteLine("Searching...");

                //Listen for ~5 secods for controllers requesting IP addresses.
                requests = gclib.GIpRequests();

                foreach (string request in requests)
                {
                    Console.WriteLine(request);
                }

            } while (requests.Count() < 1);

            foreach (string request in requests)
            {
                string[] controller_params = request.Split(new string[] { ", " }, StringSplitOptions.None);

                //Parameters are ordered as:
                //[Model #], [Serial #], [MAC Address], [Connection Name], [IP Address]

                if (controller_params.Count() < 5)
                {
                    Console.WriteLine("Unexpected controller format");
                    return GALIL_EXAMPLE_ERROR;
                }

                string mac = controller_params[2];
                string ip = controller_params[4];

                //If controller contains the user entered serial number
                if (serial_num == controller_params[1])
                {
                    Console.WriteLine("Controller Match Found");
                    controller_found = true;

                    //Splits the found ip address into individual bytes
                    string[] ip_bytes = ip.Split('.');

                    //Rebuild the ip address using the user provided address as the last byte
                    string new_ip = $"{ip_bytes[0]}.{ip_bytes[1]}.{ip_bytes[2]}.{address}";

                    //Assign the new ip address to the controller
                    gclib.GAssign(new_ip, mac);

                    //Open a connection at the new ip address
                    gclib.GOpen(new_ip);

                    //Burns the newly assigned ip address to non-volatile EEPROM memory
                    gclib.GCommand("BN");

                    Console.WriteLine("IP Address assigned");

                    //Write the connection string to the console
                    Console.WriteLine(gclib.GInfo());

                    break;
                }
            }

            if (!controller_found)
                Console.Write("No controller matched the entered serial number");

            return GALIL_EXAMPLE_OK;
        }
/** @}*/
    }
}
/** @}*/