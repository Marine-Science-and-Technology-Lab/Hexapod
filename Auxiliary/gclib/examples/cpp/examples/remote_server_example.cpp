/** @addtogroup cpp_examples
  * @{
  */

/*! \file remote_server_example.cpp

See remote_server() for implementation of logic
*/
#include "examples.h"

#include <iostream> //std::cout
#include <string> //std::getline

using namespace std;

//! Main function for Remote Server Example
/*!
\link remote_server_example.cpp \endlink takes one argument at the command line:
 the name you wish to publish your server under.
*/
int main(int argc, char * argv[])
{
	char buf[G_SMALL_BUFFER];
	int rc = GALIL_EXAMPLE_OK;
	
	try
	{
		std::string server_name;

		if (argc != 2) //Invalid number of arguments
		{
			std::cout << "Enter name of your server: ";
			std::getline(std::cin, server_name);
		}
		else
		{
			server_name = argv[1];  //Retrieve address from command line
		}

		//Demonstrates various uses of GPublishServer().
		remote_server(server_name.c_str());
	}
	catch (GReturn gr)
	{
		error(nullptr, gr);
		pause();		
		return GALIL_EXAMPLE_ERROR;
	}

    pause();
	return GALIL_EXAMPLE_OK;
}
/** @}*/
