/** @addtogroup cpp_examples
  * @{
  */
  
/*! \file commands_example.cpp

See commands() for implementation of logic
*/
#include "examples.h"

using namespace std;

//! Main function for Commands Example
/*!
\link commands_example.cpp \endlink takes one arguments at the command line:
 an IP Address to a Galil controllers.
*/
int main(int argc, char * argv[])
{
	char buf[G_SMALL_BUFFER];

	//var used to refer to a unique connection. A valid connection is nonzero.
	GCon g = 0; 

	try
	{
		if (argc != 2) //Invalid number of arguments
		{
			cerr << "Incorrect number of arguments provided\n";
			cerr << "Usage: commands_example.exe <ADDRESS>\n";
			pause();
			return GALIL_EXAMPLE_ERROR;
		}

		char* address = argv[1];  //Retrieve address from command line
		e(GOpen(address, &g)); //Opens a connection at the provided address

		//Demonstrates various uses of GCommand() and GUtility().
		commands(g);
	}
	catch (GReturn gr)
	{
	    error(g, gr); //see examples.h for error handling
	    pause();
		return GALIL_EXAMPLE_ERROR;
	}
	
	pause();
	return GALIL_EXAMPLE_OK;
}
/** @}*/
