/** @addtogroup cpp_examples
  * @{
  */
  
/*! \file record_position.cpp 
Function calls for the Record Position Example project
*/
#include "examples.h"

#include <iostream> //std::cout
#include <fstream>

#define G_LASTINDEX 999

using namespace std; 

void write_array_to_file(GCon g, ofstream& os, const char* array_name,
						 int previous_rd, int rd);

GReturn record_position(GCon g, char* fileA, char* fileB)
{
	ofstream osA, osB;
	osA.open(fileA, ios::out | ios::trunc);
	osB.open(fileB, ios::out | ios::trunc);

	int recording = 1;
	
	e(GProgramDownload(g,
	"RC 0;' Disable Recording\n"					 
	"DP 0, 0;' Set current position to 0\r"					 
	"DM posA[1000], posB[1000];' Define a new array that will hold positional data\r"
	"RA posA[], posB[];' Sets position array to be where recorded data will be stored\r"		 
	"RD _TPA, _TPB;' Defines Position to be the type of data that will be recorded\r"			 
	"RC 1,-1000;' Begins recording at 512Hz in continuous mode\r"				 
	"MO AB;' Turns motors off\r"					 
	"AI -1;' Waits for active low on Input 1\r"					 
	"RC 0;' Disable Recording after Input 1 goes low\r"					 
	"EN;' End program", 0));					 
	e(GCmd(g, "XQ")); //Begins execution of program
	
	int rd = 0;
	int previous_rd = 0;

	do
	{
		GSleep(1000); //Sleep while we wait for roughly half the array to be written
		e(GCmdI(g, "MG _RD", &rd)); //Gets address of next value in the position array

		//Get values from posA[] array and write to file
		write_array_to_file(g, osA, "posA[]", previous_rd, rd);

		//Get values from posB[] array and write to file 
		write_array_to_file(g, osB, "posB[]", previous_rd, rd); 

		e(GCmdI(g, "MG _RC", &recording)); // Check status of RC

		previous_rd = rd;
	} while (recording); //While recording is active

	osA.close();
	osB.close();

	return GALIL_EXAMPLE_OK;
}

//! Grabs data from array on controller and writes it to the given text file
void write_array_to_file(GCon g, ofstream& os, const char* array_name,
						 int previous_rd, int rd)
{
	char buf[G_HUGE_BUFFER]; //traffic buffer
		
	if (previous_rd < rd) // No array wrap around
	{
		e(GArrayUpload(g, array_name, previous_rd, rd - 1, G_COMMA, buf, G_HUGE_BUFFER));
		os << ',' << buf;
	}
	else // Array wrapped around - grab two separate chunks from the array
	{
		e(GArrayUpload(g, array_name, previous_rd, G_LASTINDEX, G_COMMA, buf, G_HUGE_BUFFER));
		os << ',' << buf;

		if (rd != 0)
		{
			e(GArrayUpload(g, array_name, 0, rd - 1, G_COMMA, buf, G_HUGE_BUFFER));
			os << ',' << buf;
		}
	}	
}
/** @}*/