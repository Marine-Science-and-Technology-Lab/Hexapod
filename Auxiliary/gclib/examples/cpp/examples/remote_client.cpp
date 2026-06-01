/** @addtogroup cpp_examples
  * @{
  */

  /*! \file remote_client.cpp
  *
  * Function calls for the Remote Client Example Project.
  *
  */
#include "examples.h"

#include <iostream> //std::cout
#include <vector>
#include <string>

#ifdef _WIN32
#include <conio.h>
#elif __linux__
#include <ncurses.h>
#endif

using namespace std;

void print_client_message(const char* message)
{
#ifdef _WIN32
	std::cout << message << std::endl;
#elif __linux__
	printw(message);
	printw("\n");
#endif
}

void print_servers_list(const std::vector<std::string>& server_list)
{
	char buf[G_SMALL_BUFFER];


	if (server_list.size() == 0)
	{
		print_client_message("none");
	}
	else
	{
		for (int i = 0; i < server_list.size(); i++)
		{
			std::string test = server_list[i];
			sprintf(buf, "<%d> %s", i, test.c_str());
			print_client_message(buf);
		}
	}
}

void servers_to_list(std::vector<std::string>& server_list, std::string servers)
{
	server_list.clear();

	if (servers.length() == 0)
		return;

	int index = 0;
	std::string server;
	while (index < servers.length())
	{
		if (servers[index] == '\n')
		{
			server_list.push_back(server);
			server.clear();
		}
		else
		{
			server += servers[index];
		}
		index++;
	}

	server_list.push_back(server);
}

/*!
|Key 	  | Usage										   |
|:-------:|:----------------------------------------------:|
| q    	  | Quit										   |
| s  	  | List available servers on then network		   |
| h  	  | List available hardware on the current server  |
| 0-9  	  | Connect to server instance by number		   |
| l  	  | Connect back to local server				   |
*/
GReturn remote_client()
{
	bool loop = true;
	char servers[G_SMALL_BUFFER];
	char buf[G_SMALL_BUFFER];
	std::vector<std::string> server_list;
	
	char instructions[] = "<s> List available servers on the network\n"
		"<h> List available hardware on currently connected server\n"
		"<0-9> Enter numbers 0-9 to connect to a server by index\n"
		"<l> Set active server back to local server\n"
		"<q> Quit\n";

#ifdef _WIN32
	cout << instructions << std::endl;
#elif __linux__
	//These functions set up the ncurses library to capture keyboard input
	initscr(); // Initialization of ncurses library
	cbreak();  // Capture one character at a time
	noecho();  // Do not write back entered characters to the console
	printw(instructions); //Print instructions to console
#endif

	while (loop)
	{
#ifdef _WIN32
		char input = _getch(); //Capture keypress
#elif __linux__
		char input = getch(); //Capture keypress
#endif
		if(input == 'q')
			loop = false;
		else if (input == 's') 
		{
			print_client_message("Available Servers:");
			e(GListServers(servers, G_SMALL_BUFFER));
			
			servers_to_list(server_list, servers);

			print_servers_list(server_list);
		}
		else if(input >= '0' && input <= '9')
		{
			int index = input - '0';
			if (server_list.size() > 0 && index < server_list.size())
			{
				e(GSetServer(server_list[index].c_str()));
				sprintf(buf, "Server set to: %s", server_list[index].c_str());
				print_client_message(buf);
			}
		}
		else if (input == 'l')
		{
			e(GSetServer("Local"));
			print_client_message("Server set to: Local");
		}
		else if (input == 'h')
		{
			e(GAddresses(buf, G_SMALL_BUFFER));
			print_client_message(buf);
		}
	}

#if __linux__
	endwin(); //Restores terminal to previous state
#endif

	return GALIL_EXAMPLE_OK;
}
/** @}*/