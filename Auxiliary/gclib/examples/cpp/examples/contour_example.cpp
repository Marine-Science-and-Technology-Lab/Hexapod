/** @addtogroup cpp_examples
  * @{
  */
  
/*! \file contour_example.cpp

See contour() for implementation of logic

*/
#include "examples.h"

#include <iostream> //std::cout
using namespace std;

//! Main function for Contour Example
/*!
\link contour_example.cpp \endlink takes three arguments at the command 
line: an IP Address to a Galil controller and a two text files to hold the 
positional data for two axes.
*/
int main(int argc, char * argv[])
{
	GReturn rc = GALIL_EXAMPLE_OK;
	char buf[G_SMALL_BUFFER];

	//var used to refer to a unique connection. A valid connection is nonzero.
	GCon g = 0;

	try
	{
		if (argc != 4) //Invalid number of arguments
		{
			cerr << "Incorrect number of arguments provided\n";
			cerr << "Usage: contour_example.exe <ADDRESS> <FileA> <FileB>\n";
			pause();
			return GALIL_EXAMPLE_ERROR;
		}

		char* fileA = argv[2];
		char* fileB = argv[3];
		char* address = argv[1];  //Retrieve address from command line
		e(GOpen(address, &g)); //Opens a connection at the provided address

		// Record user's training and plays back training through contour mode
		rc = contour(g, fileA, fileB);
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
