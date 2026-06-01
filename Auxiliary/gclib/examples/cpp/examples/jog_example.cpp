/** @addtogroup cpp_examples
  * @{
  */
  
/*! \file jog_example.cpp

See jog() for implementation of logic

*/

#include "examples.h"

#include <iostream> //std::cout
using namespace std;

//! Main function for Jog Example
/*! 
\link jog_example.cpp \endlink takes one argument at the command line: an 
IP Address to a Galil controller. When the program is run the controller will 
be at rest.  Press a key at the console to adjust the speed of the controller.
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
			cerr << "Usage: jog_example.exe <ADDRESS>\n";
			pause();
			return GALIL_EXAMPLE_ERROR;
		}

		char* address = argv[1];  //Retrieve address from command line
		e(GOpen(address, &g)); //Opens a connection at the provided address

		//Puts controller into Jog Mode and accepts user input to adjust the speed
		rc = jog(g);
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
