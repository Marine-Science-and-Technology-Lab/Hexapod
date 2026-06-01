/** @addtogroup cs_examples
  * @{
  */
  
/*! \file vector_mode.cs
*
* Function calls for the Vector Mode Example Project.
* <br><br>For VB.NET, see definition in file vector_mode.vb
*/
using System.IO;
using System.Threading;

namespace examples
{
    public static partial class Examples
    {
        /** @addtogroup cs_examples
        * @{
        */
        /// <summary>
        /// Puts controller into Vector Mode and accepts a file defining vector points.
        /// </summary>
        /// <param name="gclib">A gclib object with a valid connection.</param>
        /// <param name="file">A path to a file with stored vector commands.</param>
        /// <returns>The success status or error code of the function.</returns>
        /// <remarks>
        /// Example text file:
        /// ~~~
        /// VP -2219,-2667
        /// VP -2523,-2832
        /// VP 2844,-1425
        /// VP 728,1971
        /// VP 2127,183
        /// VP -997,688
        /// VP 725,-1893
        /// VP 527,2899
        /// VP -37,2523
        /// VP 1277,1425
        /// VP 857,2388
        /// VP 1096,-1694
        /// CR 1000,0,90
        /// ~~~
        /// See vector_mode_example.cs for an example.</remarks>
		/*! For VB.NET, see definition in file vector_mode.vb */
        public static int Vector_Mode(gclib gclib, string file)
        {
            gclib.GCommand("ST"); // Stop all motors
            gclib.GCommand("SH AB"); // Set servo here
            gclib.GCommand("DP 0,0"); // Start position at absolute zero

            gclib.GCommand("CAS"); // Defines S as active coordinate system
            gclib.GCommand("VS 20000"); // Defines vector speed
            gclib.GCommand("VA 200000"); // Defines vector acceleration
            gclib.GCommand("VD 200000"); // Defines vector decelerlation
            gclib.GCommand("VM AB"); // Begin vector segment

            using (StreamReader reader = new StreamReader(file))
            {
                //Stores the available space of the vector buffer in the capacity variable
                int capacity = gclib.GCmdI("MG _LMS");
                Load_Buffer(gclib, reader, capacity);

                gclib.GCommand("BG S");

                do // Load buffer with more commands
                {
                    Thread.Sleep(100);

                    //Stores the available space of the vector buffer in the capacity variable
                    capacity = gclib.GCmdI("MG _LMS");
                } while (Load_Buffer(gclib, reader, capacity));
            }

            gclib.GCommand("VE"); // Segment End
            gclib.GMotionComplete("S");

            return GALIL_EXAMPLE_OK;
        }

        private static bool Load_Buffer(gclib gclib, StreamReader reader, int capacity)
        {
            string s_cmd;
            // Fully load the vector buffer leaving room for one VE command
            for (; capacity > 1; capacity--)
            {
                // If there is another line of the text file
                if ((s_cmd = reader.ReadLine()) != null)
                {
                    // Run the command on each line of the text file
                    gclib.GCommand(s_cmd);
                }
                else
                    return false;
            }

            return true;
        }
/** @}*/
    }
}
/** @}*/