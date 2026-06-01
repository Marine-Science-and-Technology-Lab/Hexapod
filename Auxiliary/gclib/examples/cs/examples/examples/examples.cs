/** @defgroup cs_examples C#/VB examples */

/** @addtogroup cs_examples
  * @brief Files included in the C# Examples
  *
  * @{
  */
  
/*! \file examples.cs
*
* Shared methods and constants for gclib example projects.
* <br><br>For VB.NET, see definition in file examples.vb
*/
using System;

namespace examples
{
/** @addtogroup cs_examples
  * @brief Files included in the C# Examples
  *
  * @{
  */
    /// <summary>
    /// Provides a class of shared constants and methods for gclib's example projects.
    /// </summary>
	/*! For VB.NET, see definition in file examples.vb */
    public static partial class Examples
    {
        public const int GALIL_EXAMPLE_OK = 0; //!< Examples success code.
        public const int GALIL_EXAMPLE_ERROR = -100; //!< Examples error code.

        /// <summary>
        /// Prints the exception to the console and queries the controller for the most recent
        /// error message.  
        /// </summary>
        /// <param name="gclib">The gclib object from where the exception originated.</param>
        /// <param name="ex">The exception object caught by the example.</param>
        /// <remarks>See commands_example.cs for an example.</remarks>
        public static void PrintError(gclib gclib, Exception ex)
        {
            Console.WriteLine(ex.Message);
            
            //If exception was not a GOpen() exception, safe to query
            //the controller for a human readable error string
            if(!ex.Message.Contains("-1101"))
                Console.WriteLine(gclib.GCommand("TC 1"));
        }
    }
/** @}*/
}
/** @}*/
