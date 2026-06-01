/** @addtogroup cpp_examples
  * @{
  */
  
/*! \file ip_assigner_example.cpp

See ip_assigner() for implementation of logic
*/
#include "examples.h"

#include <iostream> //std::cout
using namespace std;

//! Main function for IP Assigner Example
/*!
\link ip_assigner_example.cpp \endlink takes two arguments at the command line: 
a Serial Number of a Galil controller and 1 byte address.
*/
int main(int argc, char * argv[])
{
	GReturn rc = GALIL_EXAMPLE_OK;
	char buf[G_SMALL_BUFFER];

	try
	{
		if (argc != 3) //Invalid number of arguments
		{
			cerr << "Incorrect number of arguments provided\n";
			cerr << "Usage: ipassigner_example.exe <SERIAL #> <1 Byte Address>\n";
			pause();
			return GALIL_EXAMPLE_ERROR;
		}
		else
		{
			char* serial_num = argv[1];

			//Retrieve address from the command line and convert to int
			char* end;
			int address = strtol(argv[2], &end, 10);
			if (*end != '\0' || address < 0 || address > 255) //If invalid address
			{
				cerr << "Please enter a number between 0 and 255 for the address."
					" This will be used as the last number in the IP Address\n"
				"Usage: ipassigner_example.exe <SERIAL #> <1 Byte Address>\n";
				return GALIL_EXAMPLE_ERROR;
			}
			//Assigns controller an IP Adress given a serial number and a 1 byte address
			rc = ip_assigner(serial_num, address);
		}
	}
	catch (GReturn gr)
	{		
		error(nullptr, gr); //see examples.h for error handling
		pause();
		return GALIL_EXAMPLE_ERROR;
	}

    pause();
	return GALIL_EXAMPLE_OK;
}
/** @}*/
