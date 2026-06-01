/** @addtogroup cs_examples
* @{
*/

/*! \file remote_server_example.cs
*
* See \link examples.Examples.Remote_Server() Remote_Server() \endlink for implementation of logic
* <br><br>For VB.NET, see definition in file remote_server_example.vb
*/

using System;
using System.Linq;

namespace examples
{
    /** @addtogroup cs_examples
    * @{
    */
    /// <summary>
    /// Demonstrates various uses of GPublishServer()
    /// </summary>
    /// <remarks>The first argument is an optional name to publish your server under.</remarks>
    /*! For VB.NET, see definition in file remote_server_example.vb */
    class Remote_Server_Example
    {
        /// <summary>
        /// Main function for the Remote Server example.
        /// </summary>
        /// <param name="args">An array of command line arguments.</param>
        /// <returns>The success status or error code of the function.</returns>
        /// <remarks>The first argument is optional and defines the name to publish your server under.</remarks>
        public static int Main(string[] args)
        {
            int rc = Examples.GALIL_EXAMPLE_OK;

            try
            {
                string server_name;

                if (args.Count() < 1)
                {
                    Console.Write("Enter server name: ");
                    server_name = Console.ReadLine();
                }
                else
                {
                    server_name = args[0];
                }
                
                rc = Examples.Remote_Server(server_name);
            }
            catch (Exception ex)
            {
                Console.WriteLine(ex.Message);
                rc = Examples.GALIL_EXAMPLE_ERROR;
            }

            Console.Write("\nPress any key to close the example");
            Console.ReadKey();

            return rc;
        }
    }
    /** @}*/
}
/** @}*/
