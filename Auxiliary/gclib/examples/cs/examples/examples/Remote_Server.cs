/** @addtogroup cs_examples
  * @{
  */

/*! \file remote_server.cs
*
* Function calls for the Remote Server Example Project.
* <br><br>For VB.NET, see definition in file remote_server.vb
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
        /// Accepts user input to publish or remove local gcaps server from the network.
        /// </summary>
        /// <param name="server_name">The name to publish local gcaps server under.</param>
        /// <returns>The success status or error code of the function.</returns>
        /// <remarks>
        /// |Key 	  | Usage                                   |
        /// |:-------:|:---------------------------------------:|
        /// | q    	  | Quit                                    |
        /// | p 	  | Publish this server to the network      |
        /// | r       | Remove this server from the network     |
        /// See remote_server_example.cs for an example.</remarks>
        /*! For VB.NET, see definition in file remote_server.vb */
        public static int Remote_Server(string server_name)
        {
            gclib gclib = new gclib();

            bool loop = true;

            Console.WriteLine("<p> Publish this server to the network\n" +
						     "<r> Remove this server from the network\n" +
						     "<q> Quit");

            while (loop)
            {
                switch (Console.ReadKey(true).Key)
                {
                    case ConsoleKey.Q:
                        loop = false;
                        break;
                    case ConsoleKey.P:
                        gclib.GPublishServer(server_name, true, false);
                        Console.WriteLine("Published Server");
                        break;
                    case ConsoleKey.R:
                        gclib.GPublishServer(server_name, false, false);
                        Console.WriteLine("Removed Server");
                        break;
                }
            }
            
            return GALIL_EXAMPLE_OK;
        }
        /** @}*/
    }
}
/** @}*/
