/** @addtogroup cpp_examples
  * @{
  */
  
/*! \file vector.cpp
*
* Function calls the Vector Mode Example Project.
*
*/
#include "examples.h"
#include <iostream> //std::cout
#include <string> //to_string, string, etc.
#include <fstream>
using namespace std;

bool load_buffer(GCon g, ifstream& fs, int capacity);

/*!
Example text file:
~~~
	VP -2219,-2667
	VP -2523,-2832
	VP 2844,-1425
	VP 728,1971
	VP 2127,183
	VP -997,688
	VP 725,-1893
	VP 527,2899
	VP -37,2523
	VP 1277,1425
	VP 857,2388
	VP 1096,-1694
	CR 1000,0,90
~~~
*/
GReturn vector(GCon g, char* file)
{
	if (g == 0) //Bad connection
	{
		cerr << "There was an error with the connection." << endl;
		return G_CONNECTION_NOT_ESTABLISHED;
	}

	e(GCmd(g, "ST"));     // stop all motors
	e(GCmd(g, "SH AB"));  // Set servo here
	e(GCmd(g, "DP 0,0")); // Start position at absolute zero

	e(GCmd(g, "CAS"));		  //Defines S as active coordinate system
	e(GCmd(g, "VS 20000"));  //Defines vector speed 
	e(GCmd(g, "VA 200000")); //Defines vector acceleration
	e(GCmd(g, "VD 200000")); //Defines vector deceleration
	e(GCmd(g, "VM AB")); //Begin Vector Segment

	ifstream fs;
	fs.open(file);  //Open user defined text file
	if (!fs) //If file could not be opened
	{
		cout << "Unable to open file\n";
		return G_BAD_FILE;
	}

	int capacity;
	//Stores the available space of the vector buffer in the capacity variable
	e(GCmdI(g, "MG _LMS", &capacity)); 
	load_buffer(g, fs, capacity);

	e(GCmd(g, "BG S")); //Begin Motion
	
	do //Load buffer with more commands
	{
		GSleep(100); //Sleep a bit while buffer is emptying
		
		//Stores the available space of the vector buffer in the capacity variable
		e(GCmdI(g, "MG _LMS", &capacity)); 
	} while (load_buffer(g, fs, capacity)); 
		
	fs.close(); //Close the file

	e(GCmd(g, "VE")); //Segment End
	e(GMotionComplete(g, "S"));

	return	GALIL_EXAMPLE_OK;
}

/*! Loads vector buffer with commands from the given text file.  
Returns false when there are no more lines in the text file
*/
bool load_buffer(GCon g, ifstream& fs, int capacity)
{
	string s_cmd;
	// Fully load the vector buffer leaving room for one VE command
	for (capacity; capacity > 1; capacity--) 
	{
		if (getline(fs, s_cmd)) //if there is another line of the text file
		{
			//Run the command on each line of the text file
			e(GCmd(g, s_cmd.c_str())); 
		}
		else
			return false;
	}

	return true;
}
/** @}*/