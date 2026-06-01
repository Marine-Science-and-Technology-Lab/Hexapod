/** @addtogroup cpp_examples
  * @{
  */
  
/*! \file message.cpp 
Function calls for the Message Example project
*/
#include "examples.h"

#include <iostream> //std::cout
#include <string.h>
using namespace std;

GReturn message(GCon g) 
{
	char buf[G_SMALL_BUFFER]; //traffic buffer
	cout << "***************************************************************\n";
	cout << "Example GMessage() usage\n";
	cout << "***************************************************************\n";

	e(GCmd(g, "TR0")); // Turn off trace
	
	//This program will force one message to appear as two separate packets.
	e(GProgramDownload(g, 
		"MG \"HELLO \" {N}\r"
		"MG \"WORLD \"\r"
		"EN", 0));
	e(GCmd(g, "XQ")); //Begins execution of program on controller

	int rc = 0;
	char message[G_SMALL_BUFFER];
	int b = 0; //iterator for buf
	int m = 0; //iterator for message

	// It is important to note that a message can be too large to read in one
	// GMessage() call. Keep calling GMessage() while there are no errors to 
	// get the full message.

	//While still receiving messages
	while ((rc = GMessage(g, buf, G_SMALL_BUFFER)) == G_NO_ERROR)
	{
		b = 0; //reset buffer index

		while (buf[b] != '\0') //While message characters are in the buffer
		{
			message[m] = buf[b]; //Copy chars from buffer to message

			//If the message ends in "\r\n" its ready to be terminated
			if (m > 0 && message[m] == '\n' && message[m - 1] == '\r')
			{
				message[m + 1] = '\0'; //Null terminate the message
				cout << '<' << message << ">\n";
				m = 0;  //Reset message index
			}
			else
			{
				m++; //Increment message index
			}

			b++; //Increment buf index
		}
	}
	
	//Downloads program to the controller
	e(GCmd(g, "TR1")); // Turn on trace
	e(GProgramDownload(g, "i=0\r"
		"#A\r"
		"MGi\r"
		"i=i+1\r"
		"WT100\r"
		"JP#A,i<1\r"
		"i=i/0\r"
		"EN", 0));
	e(GCmd(g, "XQ")); //Begins execution of program on controller

	m = 0; //Reset message buffer

	// Lines returned by GMessage() can be one of three types:
	// 1) Standard Lines begin with a space (" ")
	// 2) Crashed code begins with a question mark ("?")
	// 3) Trace Lines begin with a line number ("1,6,15...")

	//While still receiving messages
	while ((rc = GMessage(g, buf, G_SMALL_BUFFER)) == G_NO_ERROR)
	{
		b = 0; //reset buf index

		while (buf[b] != '\0') //While message characters are in the buffer
		{
			message[m] = buf[b]; //Copy chars from buffer to message

			//If the message ends in "\r\n" its ready to be terminated
			if (m > 0 && message[m] == '\n' && message[m - 1] == '\r')
			{
				message[m + 1] = '\0'; //Null terminate the message
				
				if (message[0] == ' ') //Standard Lines begin with a space (" ")
					cout << "Standard Line: ";
				else if (message[0] == '?') //Crashed code begins with a question mark ("?")
					cout << "Crashed Code: ";
				else //Trace Lines begin with a line number ("1,6,15...")
					cout << "Trace Line: ";
				
				cout << message;

				m = 0;  //Reset message index
			}
			else
			{
				m++; //Increment message index
			}

			b++; //Increment buf index
		}
	}

	// If no communication has been made to gcaps for 10 minutes the connection
	// will expire. This can be prevented by periodically sending the GUtility()
	// Keep Alive command
	e(GUtility(g, G_UTIL_GCAPS_KEEPALIVE, NULL, NULL));

	return	GALIL_EXAMPLE_OK;
}
/** @}*/
