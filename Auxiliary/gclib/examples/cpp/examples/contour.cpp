/** @addtogroup cpp_examples
  * @{
  */
  
/*! \file contour.cpp 
Function calls for the Contour Example project
*/
#include "examples.h"

#include <iostream> //std::cout
#include <fstream>
#include <string> //to_string, string, etc.
#include <vector>

using namespace std;
bool load_buf(GCon g, const std::vector<int>& positions_A,
			  const std::vector<int>& positions_B, int capacity, int& cmd);
std::vector<int> csv_to_vector(ifstream& is);

GReturn contour(GCon g, char* fileA, char* fileB)
{
	char buf[G_SMALL_BUFFER]; //traffic buffer
	int rc; //Holds the status of the RC command
	int rd;
	int positionA = 0, positionB = 0;

	record_position(g, fileA, fileB); //Record positional data on Axis A and B

	ifstream isA, isB;
	isA.open(fileA);
	isB.open(fileB);

	std::vector<int> positions_A = csv_to_vector(isA);
	std::vector<int> positions_B = csv_to_vector(isB);

	e(GCmd(g, "SH AB")); //Set servo here
	e(GCmd(g, "PA 0, 0")); //Set current position to 0
	e(GMotionComplete(g, "AB")); //Wait for motion to complete
	e(GCmd(g, "CM AB")); //Put axis A & B in Contour Mode
	e(GCmd(g, "DT -1")); //Pauses contour mode to pre-load buffer
	e(GCmd(g, "CD 0,0")); //Pre load buffer with zeros to prevent under buffering
	e(GCmd(g, "CD 0,0")); //Pre load buffer with zeros to prevent under buffering
	e(GCmd(g, "CD 0,0")); //Pre load buffer with zeros to prevent under buffering
	e(GCmd(g, "DT 1")); //Sets time interval for contour mode to be 2 samples

	int capacity = 0; //Holds the capacity of the contour buffer
	int cmd = 0; //Holds the counter for which position to send next
		
	if (positions_A.size() != positions_B.size())
	{
		cout << "Error: The two datasets are not the same size\n";
		return GALIL_EXAMPLE_ERROR;
	}
	
	do //loop while there are still commands in the buffer
	{
		//sleep while buffer is emptying
		GSleep(400); 

		//Stores the available space of the contour buffer in the capacity variable
		e(GCmdI(g, "CM?", &capacity)); 

	} while (load_buf(g, positions_A, positions_B, capacity, cmd)); 

	e(GCmd(g, "CD 0,0=0"));  //End contour mode

	isA.close();
	isB.close();

	return	GALIL_EXAMPLE_OK;
}

//! Loads contour buffer with commands from the given text file
bool load_buf(GCon g, const std::vector<int>& positions_A,
			  const std::vector<int>& positions_B, int capacity, int& cmd)
{
	char buf[G_SMALL_BUFFER]; //traffic buffer
	for (capacity; capacity > 0; capacity--) // Fully load contour buffer
	{
		// While there are commands in the position vectors
		if (cmd + 1 < positions_A.size()) 
		{
			//Subtract previous position from new position to get how far of a move to make
			int cdA = positions_A[cmd + 1] - positions_A[cmd]; 

			//Subtract previous position from new position to get how far of a move to make
			int cdB = positions_B[cmd + 1] - positions_B[cmd]; 

			sprintf(buf, "CD %d,%d", cdA, cdB);
			e(GCmd(g, buf)); //Send contour distance command

			cmd++;
		}
		else
			return false;
	}

	return true;
}

//! Converts a file of comma separated values to a vector
std::vector<int> csv_to_vector(ifstream& is)
{
	std::vector<int> positions;
	while (is.good()) //While there are still characters in the file
	{
		char position[G_SMALL_BUFFER];

		//Get a char array of all the characters leading up to the next ','
		is.getline(position, 16, ','); 

		//Convert read value to an integer
		char* end;
		int i_position = strtol(position, &end, 10); 
		positions.push_back(i_position); //Add value to the vector
	}

	return positions;
}
/** @}*/