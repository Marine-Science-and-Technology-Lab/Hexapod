/** @addtogroup cpp_examples
  * @{
  */

  /*! \file remote_server.cpp
  *
  * Function calls for the Remote Server Example Project.
  *
  */
#include "examples.h"

#include <iostream> //std::cout

#ifdef _WIN32
	#include <conio.h>
#elif __linux__
	#include <ncurses.h>
#endif

using namespace std;

void print_server_message(const char* message)
{
#ifdef _WIN32
	std::cout << message << std::endl;
#elif __linux__
	printw(message);
	printw("\n");
#endif
}

/*!
|Key 	  | Usage                                   |
|:-------:|:---------------------------------------:|
| q    	  | Quit                                    |
| p 	  | Publish this server to the network      |
| r       | Remove this server from the network     |
*/
GReturn remote_server(const char* server_name)
{
	bool loop = true;
	char connections[G_SMALL_BUFFER];
	char buf[G_SMALL_BUFFER];

	char instructions[] = "<p> Publish this server to the network\n"
						 "<r> Remove this server from the network\n"
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
		switch (_getch()) //Capture keypress
#elif __linux__
		switch (getch()) //Capture keypress
#endif
		{
		case 'q': //Quit
			loop = false;
			break;
		case 'p': //Publish this server to the network
			e(GPublishServer(server_name, true, false));
			print_server_message("Published Server");
			break;
		case 'r': //Remove this server from the network
			e(GPublishServer(server_name, false, false));
			print_server_message("Removed Server");
			break;
		}
	}
#if __linux__
	endwin(); //Restores terminal to previous state
#endif

	return GALIL_EXAMPLE_OK;
}
/** @}*/