/** @addtogroup cs_examples
  * @{
  */
  
/*! \file position_tracking.cs
*
* Function calls for the Position Tracking Example Project.
* <br><br>For VB.NET, see definition in file position_tracking.vb
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
        /// Puts controller into Position Tracking Mode and accepts user-entered positions.
        /// </summary>
        /// <param name="gclib">A gclib object with a valid connection.</param>
        /// <param name="speed">Optional speed of the controller in Position Tracking Mode.
        /// Default value of 5000</param>
        /// <returns>The success status or error code of the function.</returns>
        /// <remarks>See position_tracking_example.cs for an example.</remarks>
		/*! For VB.NET, see definition in file position_tracking.vb */
        public static int Position_Tracking(gclib gclib, int speed)
        {
            int acc = 100 * speed; // Set acceleration/deceleration to 100 times speed

            gclib.GCommand("STA");      // Stop motor
            gclib.GMotionComplete("A"); // Wait for motion to complete
            gclib.GCommand("SHA");      // Set servo here
            gclib.GCommand("DPA=0");    // Start position at absolute zero
            gclib.GCommand("PTA=1");    // Start position tracking mode on A axis

            gclib.GCommand("SPA=" + speed); // Set speed            
            gclib.GCommand("ACA=" + acc); // Set acceleration            
            gclib.GCommand("DCA=" + acc); // Set deceleration

            Console.WriteLine("Begin Position Tracking with speed " + speed +
                                ". Enter a non-number to exit.\n");
            int position;

            //Loop asking user for new position.  End loop when user enters a non-number
            while (true)
            {
                Console.WriteLine("Enter a new position:");
                bool ok = int.TryParse(Console.ReadLine(), out position);

                if (ok) //A valid position was provided
                {
                    gclib.GCommand("PAA=" + position); // Go to new position
                }
                else //A non-number was entered
                {
                    Console.WriteLine("Position Tracking has exited");
                    break;
                }
            }

            gclib.GCommand("STA"); //stop motor
            gclib.GMotionComplete("A"); //Wait for motion to complete

            return GALIL_EXAMPLE_OK;
        }
/** @}*/
    }
}
/** @}*/