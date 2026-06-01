/** @addtogroup cs_examples
  * @{
  */
  
/*! \file record_position.cs
*
* Function calls for the Record Position Example Project.
* <br><br>For VB.NET, see definition in file record_position.vb
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
        /// Record user's training and saves to a text file.
        /// </summary>
        /// <param name="gclib">A gclib object with a valid connection.</param>
        /// <param name="fileA">A Path to a text file where training for Axis A will be recorded.</param>
        /// <param name="fileB">A Path to a text file where training for Axis B will be recorded.</param>
        /// <returns>The success status or error code of the function.</returns>
        /// <remarks>See record_position_example.cs for an example.</remarks>
		/*! For VB.NET, see definition in file record_position.vb */
        public static int Record_Position(gclib gclib, string fileA, string fileB)
        {
            StreamWriter writerA = new StreamWriter(fileA, false);
            StreamWriter writerB = new StreamWriter(fileB, false);

            int recording = 1;

            gclib.GProgramDownload(
                "RC 0;' Disable Recording\n" +
                "DP 0, 0;' Set current position to 0\r" +
                "DM posA[1000], posB[1000];' Define a new array that will hold positional data\r" +
                "RA posA[], posB[];' Sets position array to be where recorded data will be stored\r" +
                "RD _TPA, _TPB;' Defines Position to be the type of data that will be recorded\r" +
                "RC 1,-1000;' Begins recording at 512Hz in continuous mode\r" +
                "MO AB;' Turns motors off\r" +
                "AI -1;' Waits for active low on Input 1\r" +
                "RC 0;' Disable Recording after Input 1 goes low\r" +
                "EN;' End program"
            );

            gclib.GCommand("XQ");

            int rd = 0;
            int previous_rd = 0;
            bool leading_comma = false;

            do
            {
                Thread.Sleep(1000); //Sleep while we wait for roughly half the array to be written
                rd = gclib.GCmdI("MG _RD"); //Gets address of next value in the position array

                //Get values from posA[] array and write to file
                Write_Array_To_File(gclib, writerA, "posA", previous_rd, rd, leading_comma);

                //Get values from posB[] array and write to file
                Write_Array_To_File(gclib, writerB, "posB", previous_rd, rd, leading_comma);

                leading_comma = true;

                recording = gclib.GCmdI("MG _RC"); //Check status of RC

                previous_rd = rd;

            } while (recording > 0); //While recording is active

            writerA.Close();
            writerB.Close();

            return Examples.GALIL_EXAMPLE_OK;
        }

        private static void Write_Array_To_File(gclib gclib, StreamWriter writer, string array_name, int previous_rd, int rd, bool leading_comma)
        {
            List<double> values = new List<double>();
            if (previous_rd < rd) // No array wrap around
            {
                //Grab list of doubles from controller and add it to values
                values.AddRange(gclib.GArrayUpload(array_name, (short)previous_rd, (short)(rd - 1)));
            }
            else
            {
                //Grab list of doubles from controller and add it to values
                values.AddRange(gclib.GArrayUpload(array_name, (short)previous_rd, 999));

                if (rd != 0)
                {
                    //Grab list of doubles from controller and add it to values
                    values.AddRange(gclib.GArrayUpload(array_name, 0, (short)(rd - 1)));
                }
            }


            for (int i = 0; i < values.Count(); i++)
            {
                if (leading_comma)
                    writer.Write(", ");

                leading_comma = true;

                writer.Write(String.Format(("{0:0.000}"), values[i]));
            }
        }
/** @}*/
    }
}
/** @}*/