/** @addtogroup cs_examples
  * @{
  */
  
/*! \file message.cs
*
* Function calls for the Message Example Project.
* <br><br>For VB.NET, see definition in file message.vb
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
        /// Demonstrates how to receive messages from the controller and detect 
        /// differences in Trace and crashed code.
        /// </summary>
        /// <param name="gclib">A gclib object with a valid connection.</param>
        /// <returns>The success status or error code of the function.</returns>
        /// <remarks>See message_example.cs for an example.</remarks>
		/*! For VB.NET, see definition in file message.vb */
        public static int Message(gclib gclib)
        {
            Console.WriteLine("***************************************************************");
            Console.WriteLine("Example GMessage() usage");
            Console.WriteLine("***************************************************************");

            gclib.GCommand("TR0"); //Turn off trace

            //This program will force one message to appear as two separate packets.
            gclib.GProgramDownload("MG \"HELLO \" {N}\r" +
                                    "MG \"WORLD \"\r" +
                                    "EN");

            gclib.GCommand("XQ"); //Begins execution of program on controller

            string buf = "";
            string message = "";

            // It is important to note that a message can be too large to read in one
            // GMessage() call. Keep calling GMessage() while there are no errors to 
            // get the full message.

            //While still receiving messages
            while ((buf = gclib.GMessage()) != "")
            {
                for (int b = 0; b < buf.Length; b++) //While message characters are in the buffer
                {
                    message += buf[b]; //Copy chars from  buffer to message

                    //If the message ends in "\r\n" it is ready to be terminated
                    if (message.Length > 2 && message[message.Length - 1] == '\n' && message[message.Length - 2] == '\r')
                    {
                        Console.WriteLine(message);
                        message = ""; //Reset message index
                    }
                }
            }

            //Downloads program to the controller
            gclib.GCommand("TR1"); //Turn on trace
            gclib.GProgramDownload(
                "i=0\r" +
                "#A\r" +
                "MGi\r" +
                "i=i+1\r" +
                "WT100\r" +
                "JP#A,i<1\r" +
                "i=i/0\r" +
                "EN");
            gclib.GCommand("XQ"); //Begins execution of program on controller

            // Lines returned by GMessage() can be one of three types:
            // 1) Standard Lines begin with a space (" ")
            // 2) Crashed code begins with a question mark ("?")
            // 3) Trace Lines begin with a line number ("1,6,15...")

            //While still receiving messages
            while ((buf = gclib.GMessage()) != "")
            {
                for (int b = 0; b < buf.Length; b++) //While message characters are in the buffer
                {
                    message += buf[b]; //Copy chars from buffer to message

                    //If the message ends in "\r\n" its ready to be terminated
                    if (message.Length > 2 && message[message.Length - 1] == '\n' && message[message.Length - 2] == '\r')
                    {
                        if (message[0] == ' ') //Standard Lines begin with a space (" ")
                            Console.Write("Standard Line: ");
                        else if (message[0] == '?') //Crashed code begins with a question mark ("?")
                            Console.Write("Crashed Code: ");
                        else //Trace Lines begin with a line number ("1,6,15...")
                            Console.Write("Trace Line: ");

                        Console.WriteLine(message);
                        message = "";
                    }
                }
            }

            return Examples.GALIL_EXAMPLE_OK;
        }
/** @}*/
    }
}
/** @}*/