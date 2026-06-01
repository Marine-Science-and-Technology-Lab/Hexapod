/** @addtogroup cpp_examples
  * @{
  */
  
/*! \file position_tracking.cpp
*
* Function calls for the Position Tracking Example Project.
*
*/
#include "examples.h"

#include <iostream> //std::cout
using namespace std;

GReturn position_tracking(GCon g, int speed)
{
	char buf[G_SMALL_BUFFER]; //traffic buffer
	int acc = 100 * speed; //set acceleration/deceleration to 100 times speed
	
	if (g == 0) //Bad connection
	{
		cerr << "There was an error with the connection." << endl;
		return G_CONNECTION_NOT_ESTABLISHED;
	}

	e(GCmd(g, "STA"));    // stop motor
	e(GMotionComplete(g, "A")); //Wait for motion to complete
	e(GCmd(g, "SHA"));   // Set servo here
	e(GCmd(g, "DPA=0")); // Start position at absolute zero
	e(GCmd(g, "PTA=1")); // Start position tracking mode on A axis

	sprintf(buf, "SPA=%d", speed);
	e(GCmd(g, buf)); // Set speed

	sprintf(buf, "ACA=%d", acc);
	e(GCmd(g, buf)); // Set acceleration

	sprintf(buf, "DCA=%d", acc);
	e(GCmd(g, buf)); // Set deceleration
	
	cout << "Begin Position Tracking with speed " << speed;
	cout << ". Enter a non-number to exit.\n";
	int position;

	//Loop asking user for new position.  End loop when user enters a non-number
	while (1) 
	{
		cout << "Enter a new position:\n";
		cin >> position;

		if (cin.fail()) //A non-number was entered
		{
			cout << "Position Tracking has exited\n";
			break;
		}
		else
		{
			sprintf(buf, "PAA=%d", position);
			e(GCmd(g, buf)); // Go to new position
		}
	}

	e(GCmd(g, "STA")); //stop motor
	e(GMotionComplete(g, "A")); //Wait for motion to complete

	return	GALIL_EXAMPLE_OK;
}
/** @}*/