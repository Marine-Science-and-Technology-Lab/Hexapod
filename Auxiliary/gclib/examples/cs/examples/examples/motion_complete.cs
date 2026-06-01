/** @addtogroup cs_examples
  * @{
  */
  
/*! \file motion_complete.cs
*
* Function calls for the Motion Complete Example Project.
* <br><br>For VB.NET, see definition in file motion_complete.vb
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
        /// Uses interrupts to track when the motion of controller is completed.
        /// </summary>
        /// <param name="gclib">A gclib object with a valid connection.</param>
        /// <returns>The success status or error code of the function.</returns>
        /// <remarks>See motion_complete_example.cs for an example.</remarks>		
		/*! For VB.NET, see definition in file motion_complete.vb */
        public static int Motion_Complete(gclib gclib)
        {
            Console.WriteLine("*************************************************************");
            Console.WriteLine("Example GInterrupt() usage");
            Console.WriteLine("*************************************************************");

            //Simple check for appropriate communication bus
            //EI will fail below if interrupts are not supported at all
            bool ei_support = false;

            if (gclib.GCommand("WH").Contains("IH"))
                ei_support = true;

            if (!ei_support)
            {
                Console.WriteLine("No support on this bus");
                return GALIL_EXAMPLE_ERROR;
            }

            //Flush interrupts
            gclib.GCommand("EI0,0"); //Turn off interrupts
            gclib.GTimeout(0); //Zero timeout

            //Flush interrupts, status will be zero when queue is empty
            while (gclib.GInterrupt() > 0) ;

            gclib.GTimeout(-1); //Restore timeout

            //Independent Motion
            gclib.GCommand("DP 0,0"); //define position zero on A and B
            Console.WriteLine("Position: " + gclib.GCommand("RP")); //Print reference position

            gclib.GCommand("SP 4000,4000"); //Set up speed
            gclib.GCommand("AC 1280000, 1280000"); //acceleration
            gclib.GCommand("DC 1280000, 1280000"); //deceleration
            gclib.GCommand("PR 8000, 10000"); //Position Relative.  B will take longer to make its move.
            gclib.GCommand("SH AB"); //Servo Here

            Console.WriteLine("Beginning independent motion...");
            gclib.GCommand("BG AB"); //Begin motion
            Check_Interrupts(gclib, "AB"); //Block until motion is complete on axes A and B
            Console.WriteLine("Motion Complete on A and B");
            Console.WriteLine("Position: " + gclib.GCommand("RP")); //Print reference position

            return GALIL_EXAMPLE_OK;
        }

        private static void Check_Interrupts(gclib gclib, string axes)
        {
            //bit mask of running axes, axes arg is trusted to provide running axes.
            //Low bit indicates running.
            byte axis_mask = 0xFF;

            //iterate through all chars in axes to make the axis mask
            for (int i = 0; i < axes.Length; i++)
            {
                //support just A-H
                switch (axes[i])
                {
                    case 'A':
                        axis_mask &= 0xFE;
                        break;
                    case 'B':
                        axis_mask &= 0xFD;
                        break;
                    case 'C':
                        axis_mask &= 0xFB;
                        break;
                    case 'D':
                        axis_mask &= 0xF7;
                        break;
                    case 'E':
                        axis_mask &= 0xEF;
                        break;
                    case 'F':
                        axis_mask &= 0xDF;
                        break;
                    case 'G':
                        axis_mask &= 0xBF;
                        break;
                    case 'H':
                        axis_mask &= 0x7F;
                        break;
                }
            }

            //send EI axis mask to set up interrupt events.
            gclib.GCommand("EI " + ~axis_mask);

            byte status;

            while (axis_mask != 0xFF) //wait for all interrupts to come in
            {
                switch (status = gclib.GInterrupt())
                {
                    case 0xD0: //Axis A complete
                        axis_mask |= 0x01;
                        break;
                    case 0xD1: //Axis B complete
                        axis_mask |= 0x02;
                        break;
                    case 0xD2: //Axis C complete
                        axis_mask |= 0x04;
                        break;
                    case 0xD3: //Axis D complete
                        axis_mask |= 0x08;
                        break;
                    case 0xD4: //Axis E complete
                        axis_mask |= 0x10;
                        break;
                    case 0xD5: //Axis F complete
                        axis_mask |= 0x20;
                        break;
                    case 0xD6: //Axis G complete
                        axis_mask |= 0x40;
                        break;
                    case 0xD7: //Axis H complete
                        axis_mask |= 0x80;
                        break;
                }
            }
        }
/** @}*/
    }
}
/** @}*/