/** @addtogroup cpp_examples
  * @{
  */
  
/*! \file record_position_example.cpp

See record_position() for implementation of logic

*/
#include "examples.h"

#include <iostream> //std::cout
using namespace std;

//! Main function for Record Position Example
/*!
\link record_position_example.cpp \endlink takes three arguments at the 
command line: an IP Address to a Galil controller and a two text files to hold 
the positional data for two axes.
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
			cerr << "Usage: record_position_example.exe <ADDRESS> <File A> <File B>\n";
			pause();
			return GALIL_EXAMPLE_ERROR;
		}

		char* fileA = argv[2]; //Retrieve filepath from command line
		char* fileB = argv[3]; //Retrieve filepath from command line
		char* address = argv[1];  //Retrieve address from command line
		e(GOpen(address, &g)); //Opens a connection at the provided address

		//Record user's training and saves to a text file
		rc = record_position(g, fileA, fileB);
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
