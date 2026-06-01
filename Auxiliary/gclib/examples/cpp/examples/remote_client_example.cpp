/** @addtogroup cpp_examples
  * @{
  */

  /*! \file remote_server_example.cpp

  See remote_server() for implementation of logic
  */
#include "examples.h"

#include <iostream> //std::cout

using namespace std;

//! Main function for Remote Server Example
/*!
\link remote_client_example.cpp \endlink takes no arguments at the command line.
*/
int main(int argc, char * argv[])
{
	char buf[G_SMALL_BUFFER];
	int rc = GALIL_EXAMPLE_OK;

	try
	{
		//Demonstrates various uses of GListServers() and GSetServer().
		remote_client();
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
