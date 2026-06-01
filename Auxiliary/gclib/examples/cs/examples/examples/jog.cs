/** @addtogroup cs_examples
  * @{
  */  
  
/*! \file jog.cs
*
* Function calls for the Jog Example Project.
* <br><br>For VB.NET, see definition in file jog.vb
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
        /// Puts controller into Jog Mode and accepts user input to adjust the speed.
        /// </summary>
        /// <param name="gclib">A gclib object with a valid connection.</param>
        /// <returns>The success status or error code of the function.</returns>
        /// <remarks>
        /// |Key 	  | Usage                   |
        /// |:-------:|:-----------------------:|
        /// | q    	  | Quit Jogging            |
        /// | a 	  | -2000 counts / second   |
        /// | s       | -500  counts / second   |
        /// | d  	  | +500  counts / second   |
        /// | f  	  | +2000 counts / second   |
        /// | r  	  | Direction Reversal      |
        /// See jog_example.cs for an example.</remarks>
		/*! For VB.NET, see definition in file jog.vb */
        public static int Jog(gclib gclib)
        {
            gclib.GCommand("ST");       // Stop all motors
            gclib.GMotionComplete("A"); // Wait for motion to complete
            gclib.GCommand("SHA");      // Set servo here
            gclib.GCommand("DPA=0");    // Start position at absolute zero
            gclib.GCommand("JGA=0");    // Start jogging with 0 speed
            gclib.GCommand("BGA");      // Begin motion on A Axis

            bool isJogging = true;
            int speed = 0;

            Console.WriteLine("Enter a character on the keyboard to change the" +
                              " motor's speed:\n<q> Quit\n<a> -2000 counts/s\n" +
                              "<s> -500  counts/s\n<d> +500  counts/s\n<f> " +
                              "+2000 counts/s\n<r> Direction Reversal\n");

            while (isJogging)
            {
                gclib.GCommand("JGA=" + speed);

                Console.WriteLine("Jog Speed: " + speed);

                switch (Console.ReadKey(true).Key)
                {
                    case ConsoleKey.Q:
                        isJogging = false;
                        break;
                    case ConsoleKey.A:
                        speed -= 2000;
                        break;
                    case ConsoleKey.S:
                        speed -= 500;
                        break;
                    case ConsoleKey.D:
                        speed += 500;
                        break;
                    case ConsoleKey.F:
                        speed += 2000;
                        break;
                    case ConsoleKey.R:
                        speed *= -1;
                        break;
                }
            }

            gclib.GCommand("ST");
            gclib.GMotionComplete("A");

            return GALIL_EXAMPLE_OK;
        }
/** @}*/
    }
}
/** @}*/