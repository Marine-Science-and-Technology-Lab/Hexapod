/** @addtogroup cpp_examples
  * @{
  */
  
/*! \file jog.cpp
Function calls for the Jog Example Project.
*/
#include "examples.h"

#ifdef _WIN32
	#include <conio.h>
#elif __linux__
	#include <ncurses.h>
#endif

#include <iostream> //std::cout
using namespace std;

/*!
|Key 	  | Usage                   |
|:-------:|:-----------------------:|
| q    	  | Quit Jogging            |
| a 	  | -2000 counts / second   |
| s       | -500  counts / second   |
| d  	  | +500  counts / second   |
| f  	  | +2000 counts / second   |
| r  	  | Direction Reversal      |
*/
GReturn jog(GCon g)
{
	char buf[G_SMALL_BUFFER]; //traffic buffer
	if (g == 0) //Bad connection
	{
		cerr << "There was an error with the connection." << endl;
		return G_CONNECTION_NOT_ESTABLISHED;
	}

#ifdef __linux__
	//These functions set up the ncurses library to capture keyboard input
	initscr(); // Initialization of ncurses library
	cbreak();  // Capture one character at a time
	noecho();  // Do not write back entered characters to the console
#endif

	e(GCmd(g, "ST"));    // stop all motors
	e(GMotionComplete(g, "A")); //Wait for motion to complete
	e(GCmd(g, "SHA"));    // Set servo here
	e(GCmd(g, "DPA=0"));  // Start position at absolute zero
	e(GCmd(g, "JGA=0"));  // Start jogging with 0 speed
	e(GCmd(g, "BGA"));   // Begin motion on A Axis

	int isJogging = 1;
	int speed = 0;

	char instructions[] = "Enter a character on the keyboard to change the"
						  " motor's speed:\n<q> Quit\n<a> -2000 counts/s\n"
						  "<s> -500  counts/s\n<d> +500  counts/s\n<f> "
						  "+2000 counts/s\n<r> Direction Reversal\n";

#ifdef _WIN32
	cout << instructions;
#elif __linux__
	printw(instructions); //Print instructions to console
#endif

	while (isJogging)
	{
		sprintf(buf, "JGA=%d", speed);
		e(GCmd(g, buf)); // Set speed
#ifdef _WIN32
		cout << "Jog Speed: " << speed << "\n";
		switch (_getch()) //Capture keypress
#elif __linux__
		sprintf(buf, "Jog Speed: %d\n", speed);
		printw(buf); //Print speed to console
		switch (getch()) //Capture keypress
#endif
		{
			case 'q': //Quit Jogging
				isJogging = 0;
				break;
			case 'a': //Large speed change in negative direction
				speed -= 2000;
				break;
			case 's': //Small speed change in negative direction
				speed -= 500;
				break;
			case 'd': //Small speed change in positive direction
				speed += 500;
				break;
			case 'f': //Large speed change in positive direction
				speed += 2000;
				break;
			case 'r': //Reverse direction of speed
				speed *= -1;
				break;
		}
	}

#if __linux__
	endwin(); //Restores terminal to previous state
#endif
	e(GCmd(g, "ST")); // Stop all motors
	e(GMotionComplete(g, "A")); //Wait for motion to complete

	return	GALIL_EXAMPLE_OK;
}
/** @}*/