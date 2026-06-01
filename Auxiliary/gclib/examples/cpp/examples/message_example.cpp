/** @addtogroup cpp_examples
  * @{
  */
  
/*! \file message_example.cpp

See message() for implementation of logic

*/
#include "examples.h"

#include <iostream> //std::cout
using namespace std;

//! Main function for Message Example
/*!
\link message_example.cpp \endlink takes one argument at the command line: 
an IP Address to a Galil controller.
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
			cerr << "Usage: message_example.exe <ADDRESS>\n";
			pause();
			return GALIL_EXAMPLE_ERROR;
		}

		char* address = argv[1];  //Retrieve address from command line
		sprintf(buf, "%s --subscribe MG", address);
		e(GOpen(buf, &g)); //Opens a connection at the provided address

		//Demonstrates how to receive messages from the controller 
		//and detect differences in Trace and crashed code.
		rc = message(g); 
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
