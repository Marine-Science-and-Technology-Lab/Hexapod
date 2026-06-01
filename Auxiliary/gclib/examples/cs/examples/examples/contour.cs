/** @addtogroup cs_examples
  * @{
  */
  
/*! \file contour.cs
*
* Function calls for the Contour Example Project.
* <br><br>For VB.NET, see definition in file contour.vb
*/
using System;
using System.Collections.Generic;
using System.Linq;
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
        /// Record user's training and plays back training through contour mode.
        /// </summary>
        /// <param name="gclib">A gclib object with a valid connection.</param>
        /// <param name="fileA">A Path to a text file where training for Axis A will be recorded.</param>
        /// <param name="fileB">A Path to a text file where training for Axis B will be recorded.</param>
        /// <returns>The success status or error code of the function.</returns>
        /// <remarks>See contour_example.cs for an example.</remarks>
		/*! For VB.NET, see definition in file contour.vb */
        public static int Contour(gclib gclib, string fileA, string fileB)
        {
            Record_Position(gclib, fileA, fileB); //Record positional data on Axis A and B

            List<string> positions_A = File.ReadAllText(fileA).Split(',').ToList();
            List<string> positions_B = File.ReadAllText(fileB).Split(',').ToList();

            gclib.GCommand("SH AB"); //Set servo here
            gclib.GCommand("PA 0, 0"); //Set current position to 0
            gclib.GMotionComplete("AB"); //Wait for motion to complete
            gclib.GCommand("CM AB"); //Put axis A & B in contour mode
            gclib.GCommand("DT -1"); //Pauses contour mode to pre-load buffer
            gclib.GCommand("CD 0,0"); //Pre load buffer with zeros to prevent under buffering
            gclib.GCommand("CD 0,0"); //Pre load buffer with zeros to prevent under buffering
            gclib.GCommand("CD 0,0"); //Pre load buffer with zeros to prevent under buffering
            gclib.GCommand("DT 1"); //Sets the time interval for contour mode to be 2 samples

            int capacity = 0; //Holds the capacity of the contour buffer
            int cmd = 0; //Holds the counter for which position to send next

            if (positions_A.Count() != positions_B.Count())
            {
                Console.WriteLine("Error: The two datasets are not the same size");
                return Examples.GALIL_EXAMPLE_ERROR;
            }

            do
            {
                //Sleep while buffer is emptying
                Thread.Sleep(400);

                //Stores the available space of the contour buffer in the capacity variable
                capacity = gclib.GCmdI("CM?");
            } while (Load_Buffer(gclib, positions_A, positions_B, capacity, ref cmd));

            gclib.GCommand("CD 0,0=0"); //End contour mode

            return Examples.GALIL_EXAMPLE_OK;
        }

        private static bool Load_Buffer(gclib gclib, List<string> positions_A, List<string> positions_B,
                                        int capacity, ref int cmd)
        {
            for (; capacity > 0; capacity--) //Fully load contour buffer
            {
                if (cmd + 1 < positions_A.Count())
                {
                    //Subtract previous position from new position to get how far of a move to make
                    double cdA = double.Parse(positions_A[cmd + 1]) - double.Parse(positions_A[cmd]);

                    //Subtract previous position from new position to get how far of a move to make
                    double cdB = double.Parse(positions_B[cmd + 1]) - double.Parse(positions_B[cmd]);

                    gclib.GCommand($"CD {cdA},{cdB}");

                    cmd++;
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