/** @addtogroup cpp_examples
  * @{
  */
  
/*! \file ip_assigner.cpp
*
* Function calls for the IP Assigner Example Project.
*
*/
#include "examples.h"

#include <iostream> //std::cout
#include <vector>
#include <string.h>
using namespace std;
typedef std::vector<string> tokens;

tokens string_split(const string& str, const string& token);

/*!
This function will listen on the network for controllers requesting an 
IP Address. If a detected controller matches the serial number provided by 
the user, a new IP Address will be assigned based on the first 3 bytes of the 
detected IP Address combined with the user defined 1 byte address.
*/
GReturn ip_assigner(char* serial_num, int address)
{
	char buf[G_SMALL_BUFFER]; //traffic buffer
	bool controller_found = false;

	do //Loop while no requests are found.
	{
		cout << "Searching...\n";

		//listen for ~5 seconds for controllers requesting IP addresses.
		e(GIpRequests(buf, G_SMALL_BUFFER)); 
		cout << buf << "\n";
	} while (strlen(buf) <= 1); 

	//Get a list of all found controllers
	tokens controllers = string_split(buf, "\n");
	//Iterate over all found controllers 
	for (tokens::iterator it = controllers.begin(); it != controllers.end(); it++) 
	{
		const string& controller = *it;
		//Get a list of the parameters of the controller
		tokens controller_params = string_split(controller, ", ");  
		//Parameters are ordered as: 
		//[Model #], [Serial #], [MAC Address], [Connection Name], [IP Address]
		if (controller_params.size() < 5)
		{
			cerr << "Unexpected controller format";
			return GALIL_EXAMPLE_ERROR;
		}
		const char* mac = controller_params[2].c_str();
		const char* ip = controller_params[4].c_str();
		//If controller contains the user entered serial number
		if (string(serial_num) == controller_params[1])  
		{
			controller_found = true;
			int ip1 = 0, ip2 = 0, ip3 = 0, ip4 = 0;
			//Parses the IP Address and breaks it into chunks in order to change the last byte
			sscanf(ip, "%d.%d.%d.%d", &ip1, &ip2, &ip3, &ip4);  

			char ipaddress[G_SMALL_BUFFER];
			//Store the first 3 bytes of the original IP Address with the user supplied address byte
			sprintf(ipaddress, "%d.%d.%d.%d", ip1, ip2, ip3, address);  

			//Assign the controller using the newly created IP Address and MAC Address
			e(GAssign(ipaddress, mac));  

			char info[G_SMALL_BUFFER];
			GCon g = 0;
			e(GOpen(ipaddress, &g)); //Opens a connection to the controller
			if (g != 0)
			{
				//Burns newly assigned IP Address to non-volatile EEPROM memory
				e(GCmd(g, "BN")); 

				e(GInfo(g, info, G_SMALL_BUFFER));
				cout << info;
			}
			break;
		}
	}

	if (!controller_found)
	{
		cout << "No controller matched the entered serial number";
	}

	return	GALIL_EXAMPLE_OK;
}

//! Splits a string into a vector based on a token.
tokens string_split(const string& str, const string& token)
{
	tokens split;  // A list that will contain strings separated by the given token
	int prev_position = 0 - token.length(); // Treat first position as if it came after a token
	int position = str.find(token); // Find position of first token

	while (position != string::npos) // Find new position of token in string
	{
		// Add characters between previous token and next token to the list
		split.push_back(str.substr(prev_position + token.length(),
								   position - prev_position - token.length()));  
		prev_position = position;
		position = str.find(token, prev_position + token.length());
	}

	// If there are any remaining characters
	if (str.length() > prev_position + token.length())  
	{
		// Add remaining characters to the list
		split.push_back(str.substr(prev_position + token.length(),
								   str.length() - prev_position - token.length()));  
	}

	return split;
}
/** @}*/