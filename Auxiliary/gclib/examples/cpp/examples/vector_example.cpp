/** @addtogroup cpp_examples
  * @{
  */
  
/*! \file vector_example.cpp

See vector() for implementation of logic

*/
#include "examples.h"

#include <iostream> //std::cout
using namespace std;

//! Main function for Vector Mode Example
/*! \link vector_example.cpp \endlink takes two arguments at the command line:
an IP Address to a Galil controller and a path to a text file defining 
vector points. When the program is run the controller will be put into vector 
mode and loaded with the points defined in the text file. The controller will 
run until it reaches all points defined in the text file.
*/
int main(int argc, char * argv[])
{
	GReturn rc = GALIL_EXAMPLE_OK;
	char buf[G_SMALL_BUFFER];

	//var used to refer to a unique connection. A valid connection is nonzero.
	GCon g = 0; 

	try
	{
		if (argc != 3)
		{
			cerr << "Incorrect number of arguments provided\n";
			cerr << "Usage: Vector_Example.exe <ADDRESS> <FILE>\n";
			pause();
			return GALIL_EXAMPLE_ERROR;
		}

		char* file = argv[2]; //Retrieve file from command line
		char* address = argv[1];  //Retrieve address from command line
		e(GOpen(address, &g)); //Opens a connection at the provided address
		
		// Puts controller into Vector Mode and accepts a file defining vector points
		rc = vector(g, file);  
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
