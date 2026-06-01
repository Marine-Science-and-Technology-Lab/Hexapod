/** @addtogroup cpp_examples
  * @{
  */
  
/*! \file motion_complete_example.cpp

See motion_complete() for implementation of logic

*/
#include "examples.h"

#include <iostream> //std::cout
using namespace std;

//! Main Function for Motion Complete Example
/*!
\link motion_complete_example.cpp \endlink takes one argument at the command 
line: an IP Address to a Galil controller.
*/
int main(int argc, char * argv[])
{
	GReturn rc = GALIL_EXAMPLE_OK;
	char buf[G_SMALL_BUFFER];

	//var used to refer to a unique connection. A valid connection is nonzero.
	GCon g = 0; 
	try
	{
		if (argc != 2) //Invalid number of arguments
		{
			cerr << "Incorrect number of arguments provided\n";
			cerr << "Usage: motion_complete_example.exe <ADDRESS>\n";
			pause();
			return GALIL_EXAMPLE_ERROR;
		}

		char* address = argv[1];  //Retrieve address from command line
		sprintf(buf, "%s --subscribe EI", address);  //Subscribe to event interrupts
		e(GOpen(buf, &g)); //Opens a connection at the provided address

		//Uses interrupts to track when the motion of controller is completed
		rc = motion_complete(g);
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
