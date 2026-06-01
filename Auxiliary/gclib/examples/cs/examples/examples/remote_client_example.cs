/** @addtogroup cs_examples
* @{
*/

/*! \file remote_client_example.cs
*
* See \link examples.Examples.Remote_Client() Remote_Client() \endlink for implementation of logic
* <br><br>For VB.NET, see definition in file remote_client_example.vb
*/

using System;
using System.Linq;

namespace examples
{
    /** @addtogroup cs_examples
    * @{
    */
    /// <summary>
    /// Demonstrates various uses of GListServers() and GSetServer()
    /// </summary>
    /// <remarks>This example requires no command line arguments.</remarks>
    /*! For VB.NET, see definition in file remote_client_example.vb */
    class Remote_Client_Example
    {
        /// <summary>
        /// Main function for the Remote Client example.
        /// </summary>
        /// <returns>The success status or error code of the function.</returns>
        /// <remarks>The first argument is an optional name to publish your client under.</remarks>
        public static int Main()
        {
            int rc = Examples.GALIL_EXAMPLE_OK;

            try
            {
                rc = Examples.Remote_Client();
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
