/** @addtogroup cpp_examples
  * @{
  */
  
/*! \file position_tracking_example.cpp

See position_tracking() for implementation of logic

*/
#include "examples.h"

#include <iostream> //std::cout
using namespace std;

//! Main function for Position Tracking Example
/*!
\link position_tracking_example.cpp \endlink takes up to two arguments at the 
command line: an IP Address to a Galil controller and an optional speed value.
If only one argument is provided the program will default to a speed value of 5000.
*/
int main(int argc, char * argv[])
{
	GReturn rc = GALIL_EXAMPLE_OK;
	char buf[G_SMALL_BUFFER];

	//var used to refer to a unique connection. A valid connection is nonzero.
	GCon g = 0; 

	try
	{
		if (argc < 2 || argc > 3) //Invalid number of arguments
		{
			cerr << "Incorrect number of arguments provided\n";
			cerr << "Usage: position_tracking_example.exe <ADDRESS> <SPEED=5000>\n";
            pause();
			return GALIL_EXAMPLE_ERROR;
		}

		char* address = argv[1];  //Retrieve address from command line
		e(GOpen(address, &g)); //Opens a connection at the provided address

		if (argc == 3) //Position tracking with custom speed
		{
			//Retrieve speed from command line and convert to int
			char* end;
			int speed = strtol(argv[2], &end, 10); 
			//If this character is not a null character,
			//the user did not enter a valid integer for speed 
			if (*end != '\0') 
			{
				cerr << "An invalid speed was entered.  "
				"Please enter a valid integer for speed.\n"
				"Usage: position_tracking_example.exe <ADDRESS> <SPEED=5000>\n";
				pause();
				return GALIL_EXAMPLE_ERROR;
			}

			//Puts controller into Position Tracking Mode and accepts user-entered positions
			rc = position_tracking(g, speed);
		}
		else if (argc == 2) //Position tracking with default speed
		{
			//Puts controller into Position Tracking Mode and accepts user - entered positions
			rc = position_tracking(g);
		}
	}
	catch (GReturn gr)
	{
		error(g, gr);
		pause();
		return GALIL_EXAMPLE_ERROR;
	}

    pause();
	return GALIL_EXAMPLE_OK;
}
/** @}*/
