/** @addtogroup cs_examples
  * @{
  */

/*! \file remote_client.cs
*
* Function calls for the Remote Client Example Project.
* <br><br>For VB.NET, see definition in file remote_client.vb
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
        /// Accepts user input to publish to list and connect to available servers.
        /// </summary>
        /// <returns>The success status or error code of the function.</returns>
        /// <remarks>
        /// |Key 	  | Usage										   |
        /// |:-------:|:----------------------------------------------:|
        /// | q    	  | Quit										   |
        /// | s  	  | List available servers on then network		   |
        /// | h  	  | List available hardware on the current server  |
        /// | 0-9  	  | Connect to server instance by number		   |
        /// | l  	  | Connect back to local server				   |
        /// See remote_client_example.cs for an example.</remarks>
        /*! For VB.NET, see definition in file remote_client.vb */
        public static int Remote_Client()
        {
            gclib gclib = new gclib();

            bool loop = true;
            string[] servers_list = Array.Empty<string>();

            Console.WriteLine("<s> List available servers on the network\n" +
                            "<h> List available hardware on currently connected server\n" +
                            "<0-9> Enter numbers 0-9 to connect to a server by index\n" +
                            "<l> Set active server back to local server\n" +
                            "<q> Quit");

            while (loop)
            {
                char input = Console.ReadKey(true).KeyChar;

                if(input == 'q')
                {
                    loop = false;
                }
                else if(input == 's')
                {
                    Console.WriteLine("Available Servers:");
                    servers_list = gclib.GListServers();
                    Print_Servers_List(servers_list);
                }
                else if(input >= '0' && input <= '9')
                {
                    int index = input - '0';
                    if(servers_list.Length > 0 && index < servers_list.Length)
                    {
                        gclib.GSetServer(servers_list[index]);
                        Console.WriteLine("Server set to: " + servers_list[index]);
                    }
                }
                else if(input == 'l')
                {
                    gclib.GSetServer("Local");
                    Console.WriteLine("Server set to: Local");
                }
                else if(input == 'h')
                {
                    string[] addresses = gclib.GAddresses();

                    foreach(string address in addresses)
                    {
                        Console.WriteLine(address);
                    }               
                }
            }

            return GALIL_EXAMPLE_OK;
        }

        private static void Print_Servers_List(string[] servers_list)
        {
            if(servers_list.Length == 0)
            {
                Console.WriteLine("none");
            }
            else
            {
                for(int i = 0; i < servers_list.Length; i++)
                {
                    Console.WriteLine("<" + i + "> " + servers_list[i]);
                }
            }
        }
        /** @}*/
    }
}
/** @}*/
